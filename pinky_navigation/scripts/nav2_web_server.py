#!/usr/bin/env python3
import threading
import time
import math
import os

from flask import Flask, jsonify, request, send_from_directory

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time

from nav_msgs.msg import OccupancyGrid, Path
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import Costmap  # global/local costmap
from geometry_msgs.msg import PoseWithCovarianceStamped  # AMCL initial pose
from action_msgs.msg import GoalStatus, GoalStatusArray  # Nav2 action status
from action_msgs.srv import CancelGoal  # cancel goals from any source

from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
)

# TF2
from tf2_ros import Buffer, TransformListener

# SLAM Toolbox services
from slam_toolbox.srv import SaveMap, Reset
from std_msgs.msg import String


############################################################
# Flask 설정
############################################################

BASE_DIR = os.path.dirname(os.path.abspath(__file__))

app = Flask(
    __name__,
    static_folder=BASE_DIR,   # 같은 폴더의 index.html 서빙
    static_url_path=""
)

ros_node = None   # 전역 ROS 노드 포인터


############################################################
# 유틸: Quaternion → yaw
############################################################
def quat_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


############################################################
# ROS2 노드 (Flask 브리지)
############################################################
class Nav2WebBridge(Node):
    def __init__(self):
        super().__init__("nav2_web_bridge_tf")
        
        self.declare_parameter("ip", "192.168.4.1")
        self.declare_parameter("port", 8080)

        # ROS 데이터
        self.map_msg = None
        self.path_msg = None
        self.local_costmap_msg = None
        self.global_costmap_msg = None

        # TF 기반 pose (x,y,yaw)
        self.tf_pose = None  # (x, y, yaw)

        self.lock = threading.Lock()

        # ---- map: TRANSIENT_LOCAL QoS (latched) ----
        map_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            OccupancyGrid,
            "map",              # 필요시 실제 토픽 이름으로 수정
            self.map_callback,
            map_qos,
        )

        # ---- path: 기본 QoS ----
        self.create_subscription(
            Path,
            "plan",
            self.path_callback,
            10,
        )

        # ---- local costmap: costmap / costmap_raw 둘 다 시도 ----
        self.local_costmap_seen = False
        self.create_subscription(
            Costmap,
            "local_costmap/costmap",
            self.local_costmap_callback,
            10,
        )
        self.create_subscription(
            Costmap,
            "local_costmap/costmap_raw",
            self.local_costmap_callback,
            10,
        )

        # ---- global costmap: costmap / costmap_raw 둘 다 시도 ----
        self.global_costmap_seen = False
        self.create_subscription(
            Costmap,
            "global_costmap/costmap",
            self.global_costmap_callback,
            10,
        )
        self.create_subscription(
            Costmap,
            "global_costmap/costmap_raw",
            self.global_costmap_callback,
            10,
        )

        # ---- TF2: map -> base_link ----
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

        # 주기적으로 TF에서 pose 업데이트
        self.create_timer(0.1, self.update_pose_from_tf)

        # Nav2 액션 클라이언트
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # Nav2 주행 상태: 액션 상태 토픽을 구독해 실제 주행 여부를 판단
        # (웹/RViz 등 goal 출처와 무관하게 동작)
        self._is_navigating = False

        # 액션 상태 토픽 QoS (transient_local + reliable, depth 1)
        status_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            GoalStatusArray,
            "navigate_to_pose/_action/status",
            self.nav_status_callback,
            status_qos,
        )

        # 모든 출처의 goal을 취소(정지)하기 위한 액션 cancel 서비스 클라이언트
        self.cancel_client = self.create_client(
            CancelGoal, "navigate_to_pose/_action/cancel_goal"
        )

        # AMCL 초기 위치(initial pose) 퍼블리셔
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "initialpose", 10
        )

        # ---- SLAM Toolbox 서비스 클라이언트 ----
        # /slam_toolbox/save_map : slam_toolbox/srv/SaveMap
        # /slam_toolbox/reset    : slam_toolbox/srv/Reset
        self.save_map_client = self.create_client(SaveMap, "/slam_toolbox/save_map")
        self.reset_client = self.create_client(Reset, "/slam_toolbox/reset")

        self.get_logger().info("Nav2WebBridge (TF-based + SLAM) started.")

    # ---------------- 콜백 ----------------
    def nav_status_callback(self, msg):
        """Nav2 액션 상태로 주행 중 여부 갱신 (출처 무관)."""
        active_states = (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING)
        navigating = any(s.status in active_states for s in msg.status_list)
        with self.lock:
            self._is_navigating = navigating

    def map_callback(self, msg):
        with self.lock:
            self.map_msg = msg

    def path_callback(self, msg):
        with self.lock:
            self.path_msg = msg

    def local_costmap_callback(self, msg):
        with self.lock:
            self.local_costmap_msg = msg
        if not self.local_costmap_seen:
            self.local_costmap_seen = True
            self.get_logger().info(
                f"Received first LOCAL costmap: "
                f"size=({msg.metadata.size_x}, {msg.metadata.size_y}), "
                f"res={msg.metadata.resolution}"
            )

    def global_costmap_callback(self, msg):
        with self.lock:
            self.global_costmap_msg = msg
        if not self.global_costmap_seen:
            self.global_costmap_seen = True
            self.get_logger().info(
                f"Received first GLOBAL costmap: "
                f"size=({msg.metadata.size_x}, {msg.metadata.size_y}), "
                f"res={msg.metadata.resolution}"
            )

    # ---------------- TF에서 pose 업데이트 ----------------
    def update_pose_from_tf(self):
        """
        map -> base_link TF를 읽어서 (x,y,yaw)로 저장.
        """
        try:
            # 최신 TF (Time()) 사용
            trans = self.tf_buffer.lookup_transform(
                "map", "base_link", Time()
            )
            t = trans.transform
            x = t.translation.x
            y = t.translation.y
            yaw = quat_to_yaw(t.rotation)

            with self.lock:
                self.tf_pose = (x, y, yaw)

        except Exception:
            # 아직 TF가 준비 안 되었을 수 있음
            pass

    # ---------------- JSON 스냅샷 ----------------
    def get_state_snapshot(self):
        with self.lock:
            map_msg = self.map_msg
            path_msg = self.path_msg
            local_costmap_msg = self.local_costmap_msg
            global_costmap_msg = self.global_costmap_msg
            tf_pose = self.tf_pose
            is_navigating = self._is_navigating

        # map
        map_json = None
        if map_msg is not None:
            info = map_msg.info
            map_json = {
                "width": info.width,
                "height": info.height,
                "resolution": info.resolution,
                "origin": {
                    "x": info.origin.position.x,
                    "y": info.origin.position.y,
                    "yaw": quat_to_yaw(info.origin.orientation)
                },
                "data": list(map_msg.data),
            }

        # pose (TF 기반)
        pose_json = None
        if tf_pose is not None:
            x, y, yaw = tf_pose
            pose_json = {
                "x": x,
                "y": y,
                "yaw": yaw,
            }

        # path
        path_json = []
        if path_msg is not None:
            for ps in path_msg.poses:
                path_json.append({
                    "x": ps.pose.position.x,
                    "y": ps.pose.position.y,
                })

        # local costmap
        local_costmap_json = None
        if local_costmap_msg is not None and len(local_costmap_msg.data) > 0:
            meta = local_costmap_msg.metadata
            local_costmap_json = {
                "width": meta.size_x,
                "height": meta.size_y,
                "resolution": meta.resolution,
                "origin": {
                    "x": meta.origin.position.x,
                    "y": meta.origin.position.y,
                    "yaw": quat_to_yaw(meta.origin.orientation),
                },
                "data": list(local_costmap_msg.data),
            }

        # global costmap
        global_costmap_json = None
        if global_costmap_msg is not None and len(global_costmap_msg.data) > 0:
            meta = global_costmap_msg.metadata
            global_costmap_json = {
                "width": meta.size_x,
                "height": meta.size_y,
                "resolution": meta.resolution,
                "origin": {
                    "x": meta.origin.position.x,
                    "y": meta.origin.position.y,
                    "yaw": quat_to_yaw(meta.origin.orientation),
                },
                "data": list(global_costmap_msg.data),
            }

        return {
            "map": map_json,
            "pose": pose_json,
            "path": path_json,
            "local_costmap": local_costmap_json,
            "global_costmap": global_costmap_json,
            "navigating": is_navigating,
        }

    # ---------------- Goal 전송 ----------------
    def send_goal(self, x, y, yaw):
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("navigate_to_pose Action Server not available.")
            return False

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(f"[WEB] send goal: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}")

        self.nav_client.send_goal_async(goal)
        # 즉각적인 UI 반응을 위해 우선 True로 표시 (이후 상태 토픽이 보정)
        with self.lock:
            self._is_navigating = True
        return True

    # ---------------- AMCL 초기 위치 설정 ----------------
    def set_initial_pose(self, x, y, yaw):
        """RViz의 '2D Pose Estimate'와 동일하게 /initialpose 퍼블리시."""
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        # RViz 기본값과 동일한 공분산 (x, y, yaw)
        msg.pose.covariance[0] = 0.25      # x
        msg.pose.covariance[7] = 0.25      # y
        msg.pose.covariance[35] = 0.06853891909122467  # yaw

        self.initial_pose_pub.publish(msg)
        self.get_logger().info(
            f"[WEB] set initial pose: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}"
        )
        return True

    # ---------------- Nav2 주행 상태/취소 ----------------
    def is_navigating(self) -> bool:
        with self.lock:
            return self._is_navigating

    def cancel_goal(self) -> bool:
        """Nav2 주행을 취소(정지). goal 출처와 무관하게 모든 goal 취소."""
        if not self.cancel_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error(
                "navigate_to_pose cancel service not available."
            )
            return False

        # 빈 goal_info(0 UUID + 0 stamp) → 모든 활성 goal 취소
        req = CancelGoal.Request()
        self.cancel_client.call_async(req)
        with self.lock:
            self._is_navigating = False
        self.get_logger().info("[WEB] Requested cancel of all goals (stop).")
        return True

    # ---------------- SLAM Toolbox 제어 ----------------
    def slam_reset(self) -> bool:
        """
        /slam_toolbox/reset 호출: 새 맵 시작(현재 pose-graph 리셋).
        """
        if not self.reset_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("/slam_toolbox/reset service not available.")
            return False

        req = Reset.Request()
        future = self.reset_client.call_async(req)
        # 결과는 굳이 기다리지 않고 True 반환 (비동기)
        self.get_logger().info("[WEB] Requested SLAM reset.")
        return True

    def slam_save_map(self, name: str) -> bool:
        """
        /slam_toolbox/save_map 호출: 현재 SLAM 맵을 pgm+yaml로 저장.
        name: 파일 이름 (확장자 없이). slam_toolbox가 실행중인 디렉토리에 저장됨.
        """
        if not self.save_map_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("/slam_toolbox/save_map service not available.")
            return False

        req = SaveMap.Request()
        req.name = String(data=name)

        future = self.save_map_client.call_async(req)
        self.get_logger().info(f"[WEB] Requested SLAM save_map: name='{name}'")
        # 마찬가지로 결과는 비동기로 처리하고 여기선 성공 요청만 리턴
        return True


############################################################
# Flask 라우트
############################################################

@app.route("/")
def serve_index():
    """브라우저에서 / 접근하면 index.html 반환"""
    return send_from_directory(BASE_DIR, "index.html")


@app.route("/api/state")
def api_state():
    """현재 맵 + TF 기반 pose + 경로 + local/global costmap 반환"""
    global ros_node
    if ros_node is None:
        return jsonify({"error": "ROS node not started"}), 500

    return jsonify(ros_node.get_state_snapshot())


@app.route("/api/goal", methods=["POST"])
def api_goal():
    """웹에서 goal 입력"""
    global ros_node
    if ros_node is None:
        return jsonify({"success": False, "msg": "ROS not ready"}), 500

    data = request.get_json()
    x = float(data["x"])
    y = float(data["y"])
    yaw = float(data.get("yaw", 0.0))  # 기본 yaw=0

    ok = ros_node.send_goal(x, y, yaw)
    return jsonify({"success": ok})


@app.route("/api/initialpose", methods=["POST"])
def api_initialpose():
    """웹에서 AMCL 초기 위치(2D Pose Estimate) 설정"""
    global ros_node
    if ros_node is None:
        return jsonify({"success": False, "msg": "ROS not ready"}), 500

    data = request.get_json()
    x = float(data["x"])
    y = float(data["y"])
    yaw = float(data.get("yaw", 0.0))

    ok = ros_node.set_initial_pose(x, y, yaw)
    return jsonify({"success": ok})


@app.route("/api/nav/status")
def api_nav_status():
    """현재 Nav2 주행 중 여부 반환"""
    global ros_node
    if ros_node is None:
        return jsonify({"error": "ROS node not started"}), 500

    return jsonify({"navigating": ros_node.is_navigating()})


@app.route("/api/nav/stop", methods=["POST"])
def api_nav_stop():
    """현재 Nav2 주행 취소(정지)"""
    global ros_node
    if ros_node is None:
        return jsonify({"success": False, "msg": "ROS not ready"}), 500

    ok = ros_node.cancel_goal()
    return jsonify({"success": ok})


@app.route("/api/slam/reset", methods=["POST"])
def api_slam_reset():
    """
    SLAM Toolbox /reset 호출: 웹에서 '새 맵 시작' 버튼 눌렀을 때.
    """
    global ros_node
    if ros_node is None:
        return jsonify({"success": False, "msg": "ROS not ready"}), 500

    ok = ros_node.slam_reset()
    return jsonify({"success": ok})


@app.route("/api/slam/save_map", methods=["POST"])
def api_slam_save_map():
    """
    SLAM Toolbox /save_map 호출: 웹에서 '맵 저장' 버튼 눌렀을 때.
    body: { "name": "pinky_lab_office" }
    """
    global ros_node
    if ros_node is None:
        return jsonify({"success": False, "msg": "ROS not ready"}), 500

    data = request.get_json() or {}
    name = data.get("name", "").strip()
    if not name:
        # 비어있으면 날짜 기반 기본 이름
        name = time.strftime("pinky_map_%Y%m%d_%H%M%S")

    ok = ros_node.slam_save_map(name)
    return jsonify({"success": ok, "name": name})


############################################################
# ROS 스레드
############################################################
def ros_spin_thread():
    try:
        rclpy.spin(ros_node)
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()


############################################################
# 메인 실행부
############################################################
if __name__ == "__main__":
    rclpy.init()    
    
    ros_node = Nav2WebBridge()

    ip_param = ros_node.get_parameter("ip").value
    port_param = ros_node.get_parameter("port").value

    # ROS2 스레드 시작
    t = threading.Thread(target=ros_spin_thread, daemon=True)
    t.start()

    time.sleep(1.0)

    print(f"Flask Web Server Running on http://{ip_param}:{port_param}")
    app.run(host=ip_param, port=int(port_param), debug=False)
