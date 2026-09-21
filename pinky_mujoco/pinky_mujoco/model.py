"""Generate a MuJoCo model directly from the vendored Pinky URDF."""
from pathlib import Path
import math
import xml.etree.ElementTree as ET

import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
from .description import expand_urdf, resolve_mesh
from .world import WorldBuilder
from .meshes import convert_visual_mesh, AssetRegistry


def numbers(text):
    return np.array([float(x) for x in text.split()])


def vector(values):
    return ' '.join(f'{float(x):.12g}' for x in values)


def origin(element):
    tag = element.find('origin')
    return {'pos': tag.get('xyz', '0 0 0') if tag is not None else '0 0 0',
            'euler': tag.get('rpy', '0 0 0') if tag is not None else '0 0 0'}


def load_settings(config_file=None):
    path = Path(config_file) if config_file else Path(get_package_share_directory('pinky_mujoco')) / 'config/mujoco.yaml'
    settings = yaml.safe_load(path.read_text())
    dt, rates = settings['physics']['timestep'], settings['rates']
    for rate in rates.values():
        if not math.isfinite(rate) or rate <= 0:
            raise ValueError('MuJoCo rates must be finite and positive')
    if dt <= 0 or not math.isfinite(dt):
        raise ValueError('MuJoCo timestep must be finite and positive')
    for ratio in (1 / dt / rates['bridge_hz'], rates['bridge_hz'] / rates['state_hz'],
                  rates['bridge_hz'] / rates['scan_hz']):
        if ratio < 1 or not math.isclose(ratio, round(ratio), abs_tol=1e-8):
            raise ValueError('Physics/bridge/state/scan rates must be integer multiples')
    return settings


def default_world(share=None):
    return Path(share or get_package_share_directory('pinky_mujoco')) / 'worlds/room.yaml'


def build_model(share, output_dir, urdf=None, config_file=None, world=None, simple_render=False):
    """simple_render: no shadow map, no floor reflection, no MSAA (for software OpenGL)."""
    """Keep URDF frames/inertias and add only simulator-specific elements."""
    share, output_dir = Path(share), Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    settings = load_settings(config_file)
    world_file = Path(world or default_world(share))  # before `world` is rebound to <worldbody>
    urdf = urdf or expand_urdf()
    (output_dir / 'pinky.urdf').write_text(urdf)
    robot = ET.fromstring(urdf)
    links = {link.get('name'): link for link in robot.findall('link')}
    joints = robot.findall('joint')
    children = {}
    for joint in joints:
        children.setdefault(joint.find('parent').get('link'), []).append(joint)

    root = ET.Element('mujoco', model='pinky_pro')
    ET.SubElement(root, 'compiler', angle='radian', eulerseq='XYZ',
                  autolimits='true', inertiafromgeom='false', fusestatic='false')
    physics, contact = settings['physics'], settings['contact']
    ET.SubElement(root, 'option', timestep=str(physics['timestep']), gravity=vector(physics['gravity']),
                  integrator=physics['integrator'], cone='elliptic', iterations=str(physics['iterations']))
    camera = settings.get('camera') or {}
    visual = ET.SubElement(root, 'visual')
    ET.SubElement(visual, 'global', offwidth=str(max(1920, int(camera.get('width', 0)))),
                  offheight=str(max(1080, int(camera.get('height', 0)))))  # offscreen render size
    if simple_render:
        ET.SubElement(visual, 'quality', shadowsize='0', offsamples='0')
    default = ET.SubElement(root, 'default')
    ET.SubElement(default, 'geom', solref=vector(contact['solref']), solimp=vector(contact['solimp']),
                  friction=vector(contact['friction']), condim='4')
    ET.SubElement(default, 'joint', **{k: str(v) for k, v in settings['joints'].items()})
    asset = ET.SubElement(root, 'asset')
    ET.SubElement(asset, 'texture', name='floor_tex', type='2d', builtin='checker',
                  rgb1='0.84 0.87 0.9', rgb2='0.94 0.95 0.97', width='512', height='512')
    ET.SubElement(asset, 'material', name='floor_mat', texture='floor_tex',
                  texrepeat='40 40', reflectance='0' if simple_render else '0.05')
    world = ET.SubElement(root, 'worldbody')
    ET.SubElement(world, 'light', pos='0 0 4', dir='0 0 -1', diffuse='0.8 0.8 0.8')
    ET.SubElement(world, 'geom', name='floor', type='plane', size='20 20 0.1',
                  material='floor_mat', group='0', contype='1', conaffinity='2')
    # Gazebo .world/.sdf or the simple worlds/*.yaml box list.
    WorldBuilder(world, asset, output_dir).load(world_file)
    (output_dir / 'world.txt').write_text(str(world_file.resolve()) + '\n')

    registry = AssetRegistry(asset)

    def add_geometry(body, geom_tag, visual, link_name, index):
        geometry = geom_tag.find('geometry')[0]
        attrs = origin(geom_tag)
        attrs.update(name=f'{link_name}_{"visual" if visual else "collision"}_{index}',
                     group='2' if visual else '1',
                     contype='0' if visual else '2', conaffinity='0' if visual else '1')
        attrs['rgba'] = ('0.95 0.64 0.73 1' if link_name == 'base_link' else
                         '0.12 0.14 0.17 1' if 'wheel' in link_name or 'rplidar' in link_name else
                         '0.82 0.85 0.9 1') if visual else '0.4 0.4 0.4 0'
        if geometry.tag == 'mesh':
            path = resolve_mesh(geometry.get('filename'))
            scale = geometry.get('scale', '1 1 1')
            if visual:
                # Keep the DAE's own per-part colours/textures instead of one flat colour.
                base = attrs.pop('name')
                attrs.pop('rgba', None)
                for i, part in enumerate(convert_visual_mesh(path, output_dir)):
                    ET.SubElement(body, 'geom', **{**attrs, 'name': f'{base}_{i}', **registry.geom_attrs(part, scale)})
                return
            attrs.update(registry.geom_attrs({'file': str(path.resolve())}, scale))
            attrs['rgba'] = '0.4 0.4 0.4 0'
        elif geometry.tag == 'sphere':
            attrs.update(type='sphere', size=geometry.get('radius'))
        elif geometry.tag == 'box':
            attrs.update(type='box', size=vector(numbers(geometry.get('size')) / 2))
        elif geometry.tag == 'cylinder':
            attrs.update(type='cylinder', size=vector([
                float(geometry.get('radius')), float(geometry.get('length')) / 2]))
        else:
            raise ValueError(f'Unsupported URDF geometry {geometry.tag}')
        if not visual and link_name == 'caster_wheel':
            attrs.update(friction=vector(contact['caster_friction']), condim='3')
        if not visual and link_name in ('l_wheel', 'r_wheel'):
            attrs.update(friction=vector(contact['wheel_friction']), condim='4')
        ET.SubElement(body, 'geom', **attrs)

    def add_link(parent, name, joint=None):
        attrs = {'name': name}
        if joint is not None:
            attrs.update(origin(joint))
        else:
            attrs['pos'] = vector(settings['spawn']['position'])
        body = ET.SubElement(parent, 'body', **attrs)
        if joint is None:
            ET.SubElement(body, 'freejoint', name='floating_base')
        elif joint.get('type') == 'continuous':
            ET.SubElement(body, 'joint', name=joint.get('name'), type='hinge',
                          axis=joint.find('axis').get('xyz'))
        elif joint.get('type') != 'fixed':
            raise ValueError(f'Unsupported joint type: {joint.get("type")}')
        link = links[name]
        inertial = link.find('inertial')
        if inertial is not None:
            inertia = inertial.find('inertia')
            # All current Pinky inertial frames have zero rotation.
            if not np.allclose(numbers(origin(inertial)['euler']), 0):
                raise ValueError('Rotated inertias need tensor transformation')
            ET.SubElement(body, 'inertial', pos=origin(inertial)['pos'],
                          mass=inertial.find('mass').get('value'),
                          fullinertia=' '.join(inertia.get(k) for k in
                                             ('ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz')))
        for visual in (False, True):
            for i, tag in enumerate(link.findall('visual' if visual else 'collision')):
                add_geometry(body, tag, visual, name, i)
        if name == 'rplidar_link':
            ET.SubElement(body, 'site', name='lidar', size='0.002', rgba='1 0 0 1')
        if camera.get('enabled') and name == camera['link']:
            # Gazebo camera looks along the link +X with +Z up; a MuJoCo camera looks along
            # its own -Z with +Y up, so camera x = -link y, camera y = link z.
            fovy = math.degrees(2 * math.atan(math.tan(camera['horizontal_fov'] / 2)
                                              * camera['height'] / camera['width']))
            ET.SubElement(body, 'camera', name='front_camera', xyaxes='0 -1 0 0 0 1', fovy=f'{fovy:.6g}')
        for child_joint in children.get(name, []):
            add_link(body, child_joint.find('child').get('link'), child_joint)

    add_link(world, 'base_link')
    actuators = ET.SubElement(root, 'actuator')
    motors = settings['motors']
    for side in ('l', 'r'):
        ET.SubElement(actuators, 'velocity', name=f'{side}_motor',
                      joint=f'{side}_wheel_joint', kv=str(motors['velocity_gain']),
                      ctrlrange=vector([-motors['max_speed'], motors['max_speed']]),
                      forcerange=vector([-motors['max_torque'], motors['max_torque']]))
    ET.indent(root)
    path = output_dir / 'scene.xml'
    ET.ElementTree(root).write(path, encoding='unicode')
    return path


def yaw_from_quat(quat):
    w, x, y, z = quat
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


class Physics:
    """Physics, wheel encoder odometry and ray-cast lidar; no ROS dependency."""
    def __init__(self, model_path, config_file=None):
        import mujoco
        self.mj = mujoco
        self.settings = load_settings(config_file)
        self.bridge_period = 1 / self.settings['rates']['bridge_hz']
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.data = mujoco.MjData(self.model)
        self.radius = float(self.model.geom('l_wheel_collision_0').size[0])
        # Include collision-centre offsets in each URDF wheel frame.
        self.track = float(abs(
            self.model.body('l_wheel').pos[1] + self.model.geom('l_wheel_collision_0').pos[1]
            - self.model.body('r_wheel').pos[1] - self.model.geom('r_wheel_collision_0').pos[1]))
        self.wheel_q = [self.model.joint(f'{s}_wheel_joint').qposadr[0] for s in ('l', 'r')]
        self.wheel_v = [self.model.joint(f'{s}_wheel_joint').dofadr[0] for s in ('l', 'r')]
        self.odom = np.zeros(3)
        self.twist = np.zeros(2)
        samples = self.settings['lidar']['samples']
        self.angles = np.linspace(-math.pi, math.pi, samples, endpoint=False)
        self.ray_vectors = np.column_stack((np.cos(self.angles), np.sin(self.angles), np.zeros(samples)))
        self.ray_group = np.array([1, 0, 0, 0, 1, 0], dtype=np.uint8)  # world geoms: groups 0 and 4
        self.lidar_id = self.model.site('lidar').id
        mujoco.mj_forward(self.model, self.data)

    def step(self, linear=0.0, angular=0.0, steps=None):
        steps = steps or round(self.bridge_period / self.model.opt.timestep)
        left = (linear - angular * self.track / 2) / self.radius
        right = (linear + angular * self.track / 2) / self.radius
        motors = self.settings['motors']
        target = np.clip([left, right], -motors['max_speed'], motors['max_speed'])
        before = self.data.qpos[self.wheel_q].copy()
        ramp = motors.get('wheel_accel')
        if ramp:
            # Velocity profile ramp (Dynamixel Profile Acceleration), applied per physics step.
            limit = ramp * self.model.opt.timestep
            for _ in range(steps):
                self.data.ctrl[:] = self.data.ctrl + np.clip(target - self.data.ctrl, -limit, limit)
                self.mj.mj_step(self.model, self.data)
        else:
            self.data.ctrl[:] = target
            self.mj.mj_step(self.model, self.data, nstep=steps)
        dl, dr = (self.data.qpos[self.wheel_q] - before) * self.radius
        distance, angle = (dl + dr) / 2, (dr - dl) / self.track
        theta = self.odom[2] + angle / 2
        self.odom += [distance * math.cos(theta), distance * math.sin(theta), angle]
        dt = steps * self.model.opt.timestep
        self.twist[:] = [distance / dt, angle / dt]

    def scan(self):
        self.mj.mj_forward(self.model, self.data)
        rotation = self.data.site_xmat[self.lidar_id].reshape(3, 3)
        rays = np.ascontiguousarray(self.ray_vectors @ rotation.T)
        distances = np.empty(len(rays), dtype=np.float64)
        geom_ids = np.empty(len(rays), dtype=np.int32)
        self.mj.mj_multiRay(self.model, self.data, self.data.site_xpos[self.lidar_id],
                           rays.ravel(), self.ray_group, True, -1, geom_ids,
                           distances, None, len(rays), self.settings['lidar']['range_max'])
        distances[(distances < self.settings['lidar']['range_min']) |
                  (distances > self.settings['lidar']['range_max'])] = np.inf
        return distances

    def pose(self):
        return np.array([*self.data.qpos[:2], yaw_from_quat(self.data.qpos[3:7])])
