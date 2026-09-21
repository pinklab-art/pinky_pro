"""MuJoCo differential drive robot with standard ROS 2 interfaces."""
import argparse
from contextlib import nullcontext
import math
import os
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist, TwistStamped, TransformStamped, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, JointState, Image, CameraInfo
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time
from tf2_ros import TransformBroadcaster
from ament_index_python.packages import get_package_share_directory

from .model import Physics, build_model


def stamp(seconds):
    ns = round(seconds * 1_000_000_000)
    return Time(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


def set_planar_pose(pose, xyz):
    pose.position.x, pose.position.y = float(xyz[0]), float(xyz[1])
    pose.orientation.z, pose.orientation.w = math.sin(xyz[2] / 2), math.cos(xyz[2] / 2)


class FrontCamera:
    """Offscreen MuJoCo render of the URDF front camera, published as sensor_msgs Image + CameraInfo."""
    def __init__(self, node, physics):
        self.cfg = physics.settings['camera']
        self.node = node
        self.model = physics.model
        self.width, self.height = int(self.cfg['width']), int(self.cfg['height'])
        self.period = 1.0 / float(self.cfg['fps'])  # sim seconds between frames; raised on slow renderers
        self.last_time = -math.inf
        self.disabled = False
        # Reliable, depth 1: best-effort 700 kB frames get dropped on loaded CPUs (measured with
        # llvmpipe: 3.2 Hz published, 1.4 Hz received), and a reliable publisher still serves
        # best-effort subscribers such as rqt_image_view.
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.image_pub = node.create_publisher(Image, self.cfg['topic'], qos)
        self.info_pub = node.create_publisher(CameraInfo, self.cfg['info_topic'], qos)
        self.renderer = None  # created lazily in the rendering thread
        self.flags_off = []   # render passes disabled on software OpenGL
        fovy = math.radians(float(self.model.cam('front_camera').fovy[0]))
        f = (self.height / 2) / math.tan(fovy / 2)
        cx, cy = self.width / 2, self.height / 2
        self.info = CameraInfo(width=self.width, height=self.height, distortion_model='plumb_bob',
                               d=[0.0] * 5, k=[f, 0.0, cx, 0.0, f, cy, 0.0, 0.0, 1.0],
                               r=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                               p=[f, 0.0, cx, 0.0, 0.0, f, cy, 0.0, 0.0, 0.0, 1.0, 0.0])
        self.info.header.frame_id = self.cfg['frame_id']

    def due(self, sim_time):
        return not self.disabled and sim_time - self.last_time >= self.period - 1e-9

    def create(self, data):
        """Create the offscreen renderer; measure its speed and cap the frame rate on software GL."""
        try:
            import mujoco
            self.renderer = mujoco.Renderer(self.model, self.height, self.width)
            self.option = mujoco.MjvOption()
            self.option.geomgroup[1] = 0  # collision shapes
            self.renderer.update_scene(data, camera='front_camera', scene_option=self.option)
            self.renderer.render()
        except Exception as error:  # no display / no OpenGL: keep simulating without images
            self.node.get_logger().warning(f'camera disabled: cannot create an OpenGL renderer ({error})')
            self.disabled = True
            return False
        try:
            from OpenGL import GL
            renderer_name = GL.glGetString(GL.GL_RENDERER).decode()
        except Exception:
            renderer_name = 'unknown'
        software = any(word in renderer_name.lower() for word in ('llvmpipe', 'softpipe', 'swrast', 'software'))
        if software or self.cfg.get('simple_render'):
            # Shadow and reflection passes redraw the whole scene; on a CPU renderer they cost
            # more than the image itself (measured 62.6 -> 14.5 ms per 640x360 frame).
            flag = mujoco.mjtRndFlag
            self.flags_off = [flag.mjRND_SHADOW, flag.mjRND_REFLECTION, flag.mjRND_SKYBOX, flag.mjRND_FOG]
        started = time.perf_counter()
        for _ in range(3):
            self.render(data)
        render_time = (time.perf_counter() - started) / 3
        if self.flags_off:
            renderer_name += ', shadows/reflections off'
        # Rendering may use at most this share of wall time so physics keeps real time.
        share = float(self.cfg.get('max_render_share', 0.25))
        limited = max(self.period, render_time / share)
        if limited > self.period * 1.01:
            self.node.get_logger().warning(
                f'camera: {renderer_name} renders {self.width}x{self.height} in {render_time * 1000:.0f} ms; '
                f"limiting {self.cfg['fps']} Hz to {1 / limited:.1f} Hz (lower camera.width/height in mujoco.yaml, "
                'or set camera.enabled: false)')
            self.period = limited
        else:
            self.node.get_logger().info(f'camera: {renderer_name}, {render_time * 1000:.1f} ms per frame')
        return True

    def render(self, data):
        self.renderer.update_scene(data, camera='front_camera', scene_option=self.option)
        for flag in self.flags_off:
            self.renderer.scene.flags[flag] = 0
        return self.renderer.render()

    def publish(self, data, stamp):
        if self.renderer is None and not self.create(data):
            return
        self.last_time = data.time
        rgb = self.render(data)
        image = Image()
        image.header.stamp, image.header.frame_id = stamp, self.cfg['frame_id']
        image.height, image.width = self.height, self.width
        image.encoding, image.is_bigendian, image.step = 'rgb8', 0, self.width * 3
        image.data = rgb.tobytes()
        self.image_pub.publish(image)
        self.info.header.stamp = stamp
        self.info_pub.publish(self.info)


class Bridge(Node):
    def __init__(self, physics):
        super().__init__('pinky_mujoco')
        self.physics = physics
        self.control = physics.settings['control']
        rates = physics.settings['rates']
        self.state_ticks = round(rates['bridge_hz'] / rates['state_hz'])
        self.scan_ticks = round(rates['bridge_hz'] / rates['scan_hz'])
        self.command = (0.0, 0.0)
        self.command_time = -math.inf
        self.create_subscription(Twist, '/cmd_vel', self.receive_command, 10)
        self.create_subscription(TwistStamped, '/cmd_vel_stamped',
                                 lambda msg: self.receive_command(msg.twist), 10)
        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.truth_pub = self.create_publisher(PoseStamped, '/ground_truth', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.tf = TransformBroadcaster(self)
        self.joint_names = ['l_wheel_joint', 'r_wheel_joint', 'caster_rotate_joint', 'caster_wheel_joint']
        self.q_indices = [physics.model.joint(n).qposadr[0] for n in self.joint_names]
        self.v_indices = [physics.model.joint(n).dofadr[0] for n in self.joint_names]
        self.tick = 0
        camera = physics.settings.get('camera') or {}
        self.camera = FrontCamera(self, physics) if camera.get('enabled') else None
        self.get_logger().info(
            f'MuJoCo ready: wheel radius {physics.radius:.4f} m, track {physics.track:.4f} m; '
            'cmd_vel -> wheel motors; encoder odom + ray-cast scan'
            + (f"; camera {self.camera.width}x{self.camera.height} @ {camera['fps']} Hz on {camera['topic']}"
               if self.camera else '; camera disabled'))

    def receive_command(self, msg):
        if not (math.isfinite(msg.linear.x) and math.isfinite(msg.angular.z)):
            return
        v, w = self.control['max_linear'], self.control['max_angular']
        self.command = (max(-v, min(v, msg.linear.x)), max(-w, min(w, msg.angular.z)))
        self.command_time = time.monotonic()

    def advance(self):
        command = self.command if time.monotonic() - self.command_time < self.control['command_timeout'] else (0.0, 0.0)
        self.physics.step(*command)
        now = stamp(self.physics.data.time)
        self.clock_pub.publish(Clock(clock=now))
        self.tick += 1
        if self.tick % self.state_ticks == 0:
            self.publish_state(now)
        if self.tick % self.scan_ticks == 0:
            scan = LaserScan()
            scan.header.stamp, scan.header.frame_id = now, 'rplidar_link'
            scan.angle_min = float(self.physics.angles[0])
            scan.angle_max = float(self.physics.angles[-1])
            scan.angle_increment = float(self.physics.angles[1] - self.physics.angles[0])
            # All rays are sampled at one simulation instant (no rolling scan).
            scan.time_increment, scan.scan_time = 0.0, 1 / self.physics.settings['rates']['scan_hz']
            scan.range_min = float(self.physics.settings['lidar']['range_min'])
            scan.range_max = float(self.physics.settings['lidar']['range_max'])
            scan.ranges = self.physics.scan().astype('float32').tolist()
            self.scan_pub.publish(scan)
        if self.camera and self.camera.due(self.physics.data.time):
            self.camera.publish(self.physics.data, now)

    def publish_state(self, now):
        physics = self.physics
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id = now, 'odom'
        odom.child_frame_id = 'base_footprint'
        set_planar_pose(odom.pose.pose, physics.odom)
        odom.twist.twist.linear.x, odom.twist.twist.angular.z = map(float, physics.twist)
        for i, variance in enumerate([0.0025, 0.0025, 1e6, 1e6, 1e6, 0.01]):
            odom.pose.covariance[7 * i] = variance
            odom.twist.covariance[7 * i] = variance
        self.odom_pub.publish(odom)
        tf = TransformStamped()
        tf.header = odom.header
        tf.child_frame_id = odom.child_frame_id
        tf.transform.translation.x = odom.pose.pose.position.x
        tf.transform.translation.y = odom.pose.pose.position.y
        tf.transform.rotation = odom.pose.pose.orientation
        self.tf.sendTransform(tf)
        joints = JointState()
        joints.header.stamp = now
        joints.name = self.joint_names
        joints.position = physics.data.qpos[self.q_indices].tolist()
        joints.velocity = physics.data.qvel[self.v_indices].tolist()
        self.joint_pub.publish(joints)
        truth = PoseStamped()
        truth.header.stamp, truth.header.frame_id = now, 'world'
        set_planar_pose(truth.pose, physics.pose())
        self.truth_pub.publish(truth)


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--viewer', action='store_true')
    parser.add_argument('--no-camera', action='store_true', help='Do not render/publish the front camera')
    parser.add_argument('--simple-render', action='store_true',
                        help='No shadows/reflections/MSAA in viewer and camera (software OpenGL)')
    parser.add_argument('--urdf-file', help='Expanded URDF file (default: expand pinky_description robot.urdf.xacro)')
    parser.add_argument('--physics-config', help='MuJoCo-only YAML override file')
    parser.add_argument('--world', help='Gazebo .world/.sdf or worlds/*.yaml (default: worlds/room.yaml)')
    parser.add_argument('--output-dir', default=os.environ.get('PINKY_MODEL_DIR',
                        str(Path.home() / '.cache/pinky_mujoco')))
    options, ros_args = parser.parse_known_args(args)
    share = get_package_share_directory('pinky_mujoco')
    urdf = Path(options.urdf_file).read_text() if options.urdf_file else None
    physics = Physics(build_model(share, options.output_dir, urdf, options.physics_config, options.world,
                                  simple_render=options.simple_render), options.physics_config)
    if options.simple_render and physics.settings.get('camera'):
        physics.settings['camera']['simple_render'] = True
    if options.no_camera and physics.settings.get('camera'):
        physics.settings['camera']['enabled'] = False
    rclpy.init(args=ros_args)
    node = Bridge(physics)
    viewer = None
    try:
        if options.viewer:
            import mujoco.viewer
            viewer = mujoco.viewer.launch_passive(physics.model, physics.data)

            def place_viewer_camera():
                viewer.cam.lookat[:] = [0, 0, 0]
                viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = 5.5, 125, -55
                viewer.opt.geomgroup[1] = 0  # Hide collision meshes.
            place_viewer_camera()
        deadline = time.monotonic()
        while rclpy.ok() and (viewer is None or viewer.is_running()):
            rclpy.spin_once(node, timeout_sec=0.0)
            with viewer.lock() if viewer else nullcontext():
                node.advance()
            if viewer and node.tick % 3 == 0:
                viewer.sync()
                if node.tick == 60:  # the window thread may reset the camera while it starts up
                    with viewer.lock():
                        place_viewer_camera()
            deadline += physics.bridge_period
            delay = deadline - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            elif delay < -0.25:
                deadline = time.monotonic()
    except KeyboardInterrupt:
        pass
    finally:
        # A second Ctrl+C during cleanup must not abort GL/rcl teardown (segfault on exit).
        try:
            if viewer:
                viewer.close()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    main()
