#!/usr/bin/env python3
# 分三阶段投放三个静态行人障碍物（不同时投放）：
#   阶段1：车从起点累计行驶 trigger_distance 后，在车前方 first_distance 处、偏左 first_lateral 投放第1个行人。
#   阶段2：车继续行驶 bypass_distance（绕过第1个行人后），在车前方 second_distance 处、偏右 second_lateral 投放第2个行人。
#   阶段3：车再继续行驶 third_trigger_distance，在车前方 third_distance 处、偏左 third_lateral 投放第3个行人。
#
# 原理：planning_simulator 里 dummy_perception_publisher 通过
#   /simulation/dummy_perception_publisher/object_info 这个 topic 接收
#   tier4_simulation_msgs/msg/DummyObject（action=ADD）来注入感知目标。
#   本脚本订阅 /localization/kinematic_state 拿到车当前位姿并累计行驶里程，
#   分三次投放，避免障碍物同时出现。
#
# 用法（由 launch_waypoint.sh 后台启动，也可单独手动跑）：
#   python3 spawn_pedestrians.py --ros-args \
#     -p trigger_distance:=8.0  -p first_distance:=6.0  -p first_lateral:=0.3 \
#     -p bypass_distance:=10.0  -p second_distance:=7.0 -p second_lateral:=-0.3 \
#     -p third_trigger_distance:=20.0 -p third_distance:=8.0 -p third_lateral:=0.4 \
#     -p spawn_timeout:=120.0
import math
import random

import rclpy
from rclpy.node import Node

from autoware_perception_msgs.msg import ObjectClassification, Shape
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from tier4_simulation_msgs.msg import DummyObject, DummyObjectInitialState
from unique_identifier_msgs.msg import UUID


def quaternion_yaw(q) -> float:
    """四元数 -> 偏航角(rad)"""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def random_uuid() -> UUID:
    u = UUID()
    u.uuid = [random.randint(0, 255) for _ in range(16)]
    return u


def make_static_pedestrian(x: float, y: float, yaw: float) -> DummyObject:
    """构造一个位于 map 坐标系 (x, y)、朝向 yaw 的静态行人 DummyObject（速度=0）"""
    obj = DummyObject()
    obj.header.frame_id = "map"
    obj.id = random_uuid()
    obj.action = DummyObject.ADD
    obj.max_velocity = 0.0
    obj.min_velocity = 0.0

    cls = ObjectClassification()
    cls.label = ObjectClassification.PEDESTRIAN  # 7
    cls.probability = 1.0
    obj.classification = cls

    shp = Shape()
    shp.type = Shape.BOUNDING_BOX  # 0
    shp.dimensions = Vector3()
    shp.dimensions.x = 0.8  # 长
    shp.dimensions.y = 0.8  # 宽
    shp.dimensions.z = 1.7  # 高
    obj.shape = shp

    init = DummyObjectInitialState()
    init.pose_covariance.pose.position.x = float(x)
    init.pose_covariance.pose.position.y = float(y)
    init.pose_covariance.pose.position.z = 0.0
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    init.pose_covariance.pose.orientation.z = qz
    init.pose_covariance.pose.orientation.w = qw
    # twist/accel 保持默认 0，即静态
    obj.initial_state = init
    return obj


class PedestrianSpawner(Node):
    def __init__(self):
        super().__init__("pedestrian_spawner")

        # 阶段1：车从起点累计行驶超过该距离(m)后才投放第1个行人
        self.trigger_distance = float(self.declare_parameter("trigger_distance", 8.0).value)
        # 第1个行人在车前方多远(m)
        self.first_distance = float(self.declare_parameter("first_distance", 6.0).value)
        # 第1个行人横向偏移(m)：正=左，负=右
        self.first_lateral = float(self.declare_parameter("first_lateral", 0.3).value)
        # 投放第1个后，车再累计行驶该距离(m)绕过它，然后投放第2个
        self.bypass_distance = float(self.declare_parameter("bypass_distance", 10.0).value)
        # 第2个行人在车前方多远(m)
        self.second_distance = float(self.declare_parameter("second_distance", 7.0).value)
        # 第2个行人横向偏移(m)：正=左，负=右
        self.second_lateral = float(self.declare_parameter("second_lateral", -0.3).value)
        # 投放第2个后，车再累计行驶该距离(m)才投放第3个
        self.third_trigger_distance = float(self.declare_parameter("third_trigger_distance", 20.0).value)
        # 第3个行人在车前方多远(m)
        self.third_distance = float(self.declare_parameter("third_distance", 8.0).value)
        # 第3个行人横向偏移(m)：正=左，负=右
        self.third_lateral = float(self.declare_parameter("third_lateral", 0.4).value)
        # 兜底超时(s)：即使车一直没动，超过该时间也强行投放第1个（第2、3个仍只按里程投放）
        self.spawn_timeout = float(self.declare_parameter("spawn_timeout", 120.0).value)

        self.pub = self.create_publisher(
            DummyObject, "/simulation/dummy_perception_publisher/object_info", 10)
        self.sub = self.create_subscription(
            Odometry, "/localization/kinematic_state", self.pose_cb, 10)

        self.latest_x = 0.0
        self.latest_y = 0.0
        self.latest_yaw = 0.0
        self.have_pose = False
        self.last_pose = None
        self.total_dist = 0.0      # 从起点累计行驶里程(m)
        self.phase = 0             # 0=等第1个, 1=等第2个, 2=等第3个, 3=完成
        self.start_time = self.get_clock().now()
        self.last_log_time = self.start_time
        self.finish_time = None

    def pose_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self.last_pose is not None:
            self.total_dist += math.hypot(x - self.last_pose[0], y - self.last_pose[1])
        self.last_pose = (x, y)
        self.latest_x = x
        self.latest_y = y
        self.latest_yaw = quaternion_yaw(msg.pose.pose.orientation)
        self.have_pose = True

    def elapsed_seconds(self) -> float:
        return (self.get_clock().now() - self.start_time).nanoseconds * 1e-9

    def check_trigger(self):
        if self.phase >= 3:
            return
        elapsed = self.elapsed_seconds()

        # 等 dummy_perception_publisher 起来（出现订阅者）
        if self.pub.get_subscription_count() == 0:
            if elapsed > self.spawn_timeout:
                self.get_logger().warn("超时且未发现 dummy_perception_publisher，放弃投放")
                self.phase = 3
            return

        phase1_at = self.trigger_distance
        phase2_at = self.trigger_distance + self.bypass_distance
        phase3_at = phase2_at + self.third_trigger_distance

        if self.phase == 0:
            if self.total_dist >= phase1_at or elapsed >= self.spawn_timeout:
                self.spawn_one(self.first_distance, self.first_lateral, 1)
                self.phase = 1
        elif self.phase == 1:
            # 第2个只按里程触发：确保车已绕过第1个，不因超时同时投放
            if self.total_dist >= phase2_at:
                self.spawn_one(self.second_distance, self.second_lateral, 2)
                self.phase = 2
        elif self.phase == 2:
            # 第3个同样只按里程触发
            if self.total_dist >= phase3_at:
                self.spawn_one(self.third_distance, self.third_lateral, 3)
                self.phase = 3
                self.finish_time = self.get_clock().now()

        now = self.get_clock().now()
        if (now - self.last_log_time).nanoseconds * 1e-9 > 5.0:
            self.last_log_time = now
            if self.phase == 0:
                self.get_logger().info(
                    f"等待第1个行人：已行驶 {self.total_dist:.1f} m / {phase1_at:.1f} m")
            elif self.phase == 1:
                self.get_logger().info(
                    f"等待第2个行人（先绕过第1个）：已行驶 {self.total_dist:.1f} m / {phase2_at:.1f} m")
            elif self.phase == 2:
                self.get_logger().info(
                    f"等待第3个行人：已行驶 {self.total_dist:.1f} m / {phase3_at:.1f} m")

    def spawn_one(self, distance: float, lateral: float, idx: int):
        x, y, yaw = self.latest_x, self.latest_y, self.latest_yaw
        # 沿车头朝向推进 distance，再横向偏移 lateral（正=左，负=右）
        px = x + distance * math.cos(yaw) - lateral * math.sin(yaw)
        py = y + distance * math.sin(yaw) + lateral * math.cos(yaw)
        obj = make_static_pedestrian(px, py, yaw)
        obj.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(obj)
        side = "左" if lateral >= 0 else "右"
        self.get_logger().info(
            f"投放静态行人 #{idx} @ ({px:.2f}, {py:.2f})，距车 {distance:.1f} m，偏{side} {abs(lateral):.1f} m")


def main():
    rclpy.init()
    node = PedestrianSpawner()
    node.create_timer(0.5, node.check_trigger)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.2)
            if node.phase >= 3 and node.finish_time is not None:
                # 第3个投放后再等 1.5s 让消息发出去，然后退出
                if (node.get_clock().now() - node.finish_time).nanoseconds * 1e-9 > 1.5:
                    break
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
