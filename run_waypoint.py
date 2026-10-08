#!/usr/bin/env python3
# 免 RViz 自动设置初始位姿 + 本地化初始化 + 路径点路线 + auto + engage（关 AD API 版）。
#
# 与 set_route.py 的差异：本脚本不再依赖 /api/*（AD API 已关），改用底层 component interface 服务：
#  - 本地化 -> /localization/initialize（autoware_localization_msgs/srv/InitializeLocalization，
#              request 为 pose_with_covariance 数组 + method=AUTO）。提供者：autoware_pose_initializer。
#  - 路线   -> /planning/set_waypoint_route（autoware_planning_msgs/srv/SetWaypointRoute）。提供者：mission_planner。
#  - auto   -> /system/operation_mode/change_operation_mode（autoware_system_msgs/srv/ChangeOperationMode，
#              mode=AUTONOMOUS=2）。
#  - engage -> /system/operation_mode/change_autoware_control（autoware_system_msgs/srv/ChangeAutowareControl，
#              autoware_control=true）。
#             以上两个提供者：autoware_operation_mode_transition_manager（control 模块，非 AD API）。
#
# 关键：只发 /initialpose3d 只会重置仿真器车辆位姿，不会让 pose_initializer 进入 INITIALIZED；
#   必须调 initialize 服务（method=AUTO），成功后 pose_initializer 自身会发 pose_reset 连带重置仿真器。
#   否则 change_operation_mode 一直报 "The target mode is not available"。
import math
from uuid import uuid4

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile

import yaml

from autoware_localization_msgs.srv import InitializeLocalization
from autoware_planning_msgs.srv import ClearRoute, SetWaypointRoute
from autoware_system_msgs.srv import ChangeAutowareControl, ChangeOperationMode
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import Odometry
from unique_identifier_msgs.msg import UUID


def yaw_to_quaternion(yaw_rad: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw_rad / 2.0)
    q.w = math.cos(yaw_rad / 2.0)
    return q


def make_pose_yaw(x: float, y: float, z: float, yaw_rad: float) -> Pose:
    p = Pose()
    p.position.x = float(x)
    p.position.y = float(y)
    p.position.z = float(z)
    p.orientation = yaw_to_quaternion(float(yaw_rad))
    return p


def make_pose_quat(d: dict) -> Pose:
    p = Pose()
    pos = d["position"]
    o = d["orientation"]
    p.position.x = float(pos["x"])
    p.position.y = float(pos["y"])
    p.position.z = float(pos.get("z", 0.0))
    p.orientation.x = float(o["x"])
    p.orientation.y = float(o["y"])
    p.orientation.z = float(o["z"])
    p.orientation.w = float(o["w"])
    return p


class WaypointRunner(Node):
    def __init__(self):
        super().__init__("waypoint_runner")
        input_file = self.declare_parameter("input_file", "").value
        self.get_logger().info(f"input_file = {input_file}")
        with open(input_file) as f:
            self.cfg = yaml.safe_load(f)

        self.waypoints = self.cfg["waypoints"]
        self.goal = self.waypoints[-1]

        # 循环绕圈：legs 定义每段路线（goal + 中间路径点），到达 goal 后切下一段，周而复始。
        # 第 0 段：A -> B -> C（初始，仅跑一次）；第 1 段：C -> B。之后 C <-> B 往返循环。
        b_pose = make_pose_yaw(
            self.waypoints[0]["x"], self.waypoints[0]["y"],
            self.waypoints[0].get("z", 0.0), self.waypoints[0].get("yaw", 0.0))
        self.legs = [
            {
                "goal": make_pose_yaw(
                    self.goal["x"], self.goal["y"], self.goal.get("z", 0.0),
                    self.goal.get("yaw", 0.0)),
                "waypoints": [
                    make_pose_yaw(wp["x"], wp["y"], wp.get("z", 0.0), wp.get("yaw", 0.0))
                    for wp in self.waypoints[:-1]
                ],
            },
            {
                "goal": b_pose,
                "waypoints": [],
            },
        ]
        self.loop = self.declare_parameter("loop", True).value
        self.arrival_threshold = self.declare_parameter("arrival_threshold", 3.0).value
        self.leg_index = 0
        self.next_loop_tick = 0
        self.loop_cooldown = 3  # 到达 goal 后等几秒再切下一段，避免抖动
        self.current_x = 0.0
        self.current_y = 0.0

        self.pub_init = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose3d", QoSProfile(depth=1)
        )
        self.cli_localize = self.create_client(
            InitializeLocalization, "/localization/initialize"
        )
        self.cli_route = self.create_client(SetWaypointRoute, "/planning/set_waypoint_route")
        self.cli_clear_route = self.create_client(ClearRoute, "/planning/clear_route")
        self.cli_autonomous = self.create_client(
            ChangeOperationMode, "/system/operation_mode/change_operation_mode"
        )
        self.cli_engage = self.create_client(
            ChangeAutowareControl, "/system/operation_mode/change_autoware_control"
        )

        self.sub_pose = self.create_subscription(
            Odometry, "/localization/kinematic_state", self.pose_cb, 1)

        self.init_msg = PoseWithCovarianceStamped()
        self.init_msg.header.frame_id = "map"
        self.init_msg.pose.pose = make_pose_quat(self.cfg["initial_pose"])

        self.sent_init = False
        self.localized = False
        self.localize_in_flight = False
        self.localize_attempts = 0
        self.next_localize_tick = 0
        self.route_succeeded = False
        self.route_in_flight = False
        self.route_attempts = 0
        self.next_retry_tick = 0
        self.need_clear_route = False
        self.clear_in_flight = False
        self.engaged = False
        self.engage_stage = 0  # 0=先切 AUTO, 1=再 engage, 2=完成
        self.engage_in_flight = False
        self.next_engage_tick = 0
        self.wait_ticks = 0
        self.timer = self.create_timer(1.0, self.tick)

    def tick(self):
        self.wait_ticks += 1

        # 0) 本地化初始化：调用 /localization/initialize，直到 pose_initializer 进入 INITIALIZED。
        if not self.localized:
            if not self.localize_in_flight and self.wait_ticks >= self.next_localize_tick:
                if self.cli_localize.service_is_ready():
                    self.localize_in_flight = True
                    self.localize_attempts += 1
                    req = InitializeLocalization.Request()
                    pose = PoseWithCovarianceStamped()
                    pose.header.frame_id = "map"
                    pose.header.stamp = self.get_clock().now().to_msg()
                    pose.pose.pose = make_pose_quat(self.cfg["initial_pose"])
                    req.pose_with_covariance = [pose]
                    req.method = InitializeLocalization.Request.AUTO
                    self.get_logger().info(
                        f"calling /localization/initialize (attempt={self.localize_attempts})"
                    )
                    future = self.cli_localize.call_async(req)
                    future.add_done_callback(self.localize_done)
                elif self.wait_ticks % 10 == 0:
                    self.get_logger().info("waiting for /localization/initialize service...")

        # 1) 初始位姿：周期发布（重置仿真器车辆位姿），直到 engage 后车开始动为止。
        #    循环跑时 engage 后不再重置位姿，否则会把车瞬移回起点。
        if not self.engaged:
            self.init_msg.header.stamp = self.get_clock().now().to_msg()
            self.pub_init.publish(self.init_msg)
            if not self.sent_init:
                self.sent_init = True
                self.get_logger().info(
                    "publishing initial pose -> /initialpose3d (periodic until engaged)"
                )

        # 1.5) 上一段已到达时 mission_planner 进入 ARRIVED，需先 clear_route 复位到 UNSET。
        if self.need_clear_route:
            if self.clear_in_flight:
                return  # 等 clear 返回
            if not self.cli_clear_route.service_is_ready():
                if self.wait_ticks % 10 == 0:
                    self.get_logger().info("waiting for /planning/clear_route service...")
                return
            self.clear_in_flight = True
            self.get_logger().info("calling /planning/clear_route (reset ARRIVED -> UNSET)")
            future = self.cli_clear_route.call_async(ClearRoute.Request())
            future.add_done_callback(self.clear_route_done)
            return

        # 2) 路线：本地化成功后设置，失败重试，直到成功。
        if self.localized and not self.route_succeeded:
            if self.route_in_flight:
                return  # 上一次调用还没返回
            if self.wait_ticks < self.next_retry_tick:
                return  # 重试冷却中
            if not self.cli_route.service_is_ready():
                if self.wait_ticks % 10 == 0:
                    self.get_logger().info("waiting for /planning/set_waypoint_route service...")
                return
            self.route_in_flight = True
            self.route_attempts += 1
            leg = self.legs[self.leg_index]
            req = SetWaypointRoute.Request()
            req.header.stamp = self.get_clock().now().to_msg()
            req.header.frame_id = "map"
            req.goal_pose = leg["goal"]
            for wp in leg["waypoints"]:
                req.waypoints.append(wp)
            req.allow_modification = True
            req.uuid = UUID(uuid=list(uuid4().bytes))
            self.get_logger().info(
                f"calling /planning/set_waypoint_route (leg={self.leg_index}, "
                f"attempt={self.route_attempts}, waypoints={len(req.waypoints)})"
            )
            future = self.cli_route.call_async(req)
            future.add_done_callback(self.route_done)

        # 3) 挂挡：路线成功后先切 AUTO 模式，再 engage，车辆才会沿轨迹行驶。
        if self.route_succeeded and not self.engaged:
            if self.engage_in_flight:
                return  # 上一次调用还没返回
            if self.wait_ticks < self.next_engage_tick:
                return  # 重试冷却中
            cli, label, req = (
                (
                    self.cli_autonomous,
                    "/system/operation_mode/change_operation_mode (AUTO)",
                    self._change_op_mode_req(),
                )
                if self.engage_stage == 0
                else (
                    self.cli_engage,
                    "/system/operation_mode/change_autoware_control (engage)",
                    self._change_autoware_control_req(),
                )
            )
            if not cli.service_is_ready():
                if self.wait_ticks % 10 == 0:
                    self.get_logger().info(f"waiting for {label} service...")
                return
            self.engage_in_flight = True
            self.get_logger().info(f"calling {label}")
            future = cli.call_async(req)
            future.add_done_callback(self.engage_done)

        # 4) 循环绕圈：到达当前 leg 的 goal 后，切下一段并重新设路线。
        if self.loop and self.engaged and self.route_succeeded:
            goal = self.legs[self.leg_index]["goal"]
            dist = math.hypot(
                self.current_x - goal.position.x, self.current_y - goal.position.y)
            if dist < self.arrival_threshold and self.wait_ticks >= self.next_loop_tick:
                self.get_logger().info(
                    f"=== REACHED GOAL (dist={dist:.2f}m), switch to next leg ===")
                self.leg_index = (self.leg_index + 1) % len(self.legs)
                self.route_succeeded = False
                self.route_in_flight = False
                self.route_attempts = 0
                self.next_retry_tick = self.wait_ticks + 2
                self.next_loop_tick = self.wait_ticks + self.loop_cooldown
                self.need_clear_route = True  # mission_planner 已进入 ARRIVED，需先清路线

        # 周期性打印等待状态（每 10 秒一次）
        if self.wait_ticks % 10 == 0:
            self.get_logger().info(
                f"status... localized={self.localized} (attempts={self.localize_attempts}) "
                f"route_ok={self.route_succeeded} (leg={self.leg_index}, attempts={self.route_attempts}) "
                f"engaged={self.engaged} pos=({self.current_x:.2f},{self.current_y:.2f}) "
                f"srv_localize={self.cli_localize.service_is_ready()} "
                f"srv_route={self.cli_route.service_is_ready()} "
                f"srv_auto={self.cli_autonomous.service_is_ready()} "
                f"srv_engage={self.cli_engage.service_is_ready()}"
            )

    def _change_op_mode_req(self):
        req = ChangeOperationMode.Request()
        req.mode = ChangeOperationMode.Request.AUTONOMOUS
        return req

    def _change_autoware_control_req(self):
        req = ChangeAutowareControl.Request()
        req.autoware_control = True
        return req

    def pose_cb(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def localize_done(self, future):
        self.localize_in_flight = False
        try:
            res = future.result()
            self.get_logger().info(
                f"localize response: success={res.status.success} "
                f"code={res.status.code} message={res.status.message}"
            )
            if res.status.success:
                self.localized = True
                self.get_logger().info("=== LOCALIZATION INITIALIZED ===")
            else:
                self.next_localize_tick = self.wait_ticks + 3  # 3 秒后重试
                self.get_logger().warn(
                    f"localize failed (code={res.status.code}), retrying in 3s"
                )
        except Exception as e:  # noqa: BLE001
            self.next_localize_tick = self.wait_ticks + 3
            self.get_logger().error(f"localize call exception: {e}")

    def clear_route_done(self, future):
        self.clear_in_flight = False
        try:
            res = future.result()
            self.get_logger().info(
                f"clear_route response: success={res.status.success} "
                f"code={res.status.code} message={res.status.message}"
            )
            if res.status.success:
                self.need_clear_route = False
                self.get_logger().info("=== ROUTE CLEARED ===")
            # 失败则保持 need_clear_route=True，下个 tick 重试
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"clear_route call exception: {e}")

    def route_done(self, future):
        self.route_in_flight = False
        try:
            res = future.result()
            self.get_logger().info(
                f"route response: success={res.status.success} "
                f"code={res.status.code} message={res.status.message}"
            )
            if res.status.success:
                self.route_succeeded = True
                self.get_logger().info("=== ROUTE SET OK ===")
            else:
                self.next_retry_tick = self.wait_ticks + 3  # 3 秒后重试
                self.get_logger().warn(
                    f"route failed, retrying in 3s (attempt {self.route_attempts})"
                )
        except Exception as e:  # noqa: BLE001
            self.next_retry_tick = self.wait_ticks + 3
            self.get_logger().error(f"route call exception: {e}")

    def engage_done(self, future):
        self.engage_in_flight = False
        try:
            res = future.result()
            label = (
                "change_operation_mode" if self.engage_stage == 0
                else "change_autoware_control"
            )
            self.get_logger().info(
                f"{label} response: success={res.status.success} "
                f"code={res.status.code} message={res.status.message}"
            )
            if res.status.success:
                if self.engage_stage == 0:
                    self.engage_stage = 1
                    self.get_logger().info("=== OPERATION MODE: AUTONOMOUS ===")
                else:
                    self.engaged = True
                    self.get_logger().info("=== ENGAGE OK (autonomous driving) ===")
            else:
                self.next_engage_tick = self.wait_ticks + 3  # 3 秒后重试
                self.get_logger().warn(
                    f"{label} failed (code={res.status.code}), retrying in 3s"
                )
        except Exception as e:  # noqa: BLE001
            self.next_engage_tick = self.wait_ticks + 3
            self.get_logger().error(f"engage call exception: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = WaypointRunner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
