#!/usr/bin/env python3
"""回灌实车包时补齐 planning 栈必需、但包里没有的感知输入。

包里只有定位/地图/规划/空的目标列表，没有感知模块输出的：
  * /perception/occupancy_grid_map/map          (behavior_path_planner 必需)
  * /perception/obstacle_segmentation/pointcloud (behavior_velocity_planner 必需)

这里发布：
  * 一张以自车为中心、全 free(全 0) 的 OccupancyGrid；
  * 一个只含 1 个 1km 外孤立点的 PointCloud2（满足下游"有点"的要求，又不构成障碍）。

注意：两者都**不代表真实障碍物**，只是让规划链路能跑通。要还原真实障碍，
需要包里本来就有感知结果。

参数：
  resolution      栅格分辨率 [m/格]，默认 0.5
  half_extent     栅格半边长 [m]，默认 100
  grid_rate       栅格发布频率 [Hz]，默认 2.0
  cloud_rate      点云发布频率 [Hz]，默认 10.0
  grid_frame_id   栅格坐标系，默认 map
  cloud_frame_id  点云坐标系，默认 base_link
"""

import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import PointCloud2, PointField


class ReplayInputsPublisher(Node):
    def __init__(self):
        super().__init__('replay_inputs_publisher')

        self.declare_parameter('resolution', 0.5)
        self.declare_parameter('half_extent', 100.0)
        self.declare_parameter('grid_rate', 2.0)
        self.declare_parameter('cloud_rate', 10.0)
        self.declare_parameter('grid_frame_id', 'map')
        self.declare_parameter('cloud_frame_id', 'base_link')

        self.resolution = float(self.get_parameter('resolution').value)
        self.half_extent = float(self.get_parameter('half_extent').value)
        grid_rate = float(self.get_parameter('grid_rate').value)
        cloud_rate = float(self.get_parameter('cloud_rate').value)
        self.grid_frame = str(self.get_parameter('grid_frame_id').value)
        self.cloud_frame = str(self.get_parameter('cloud_frame_id').value)

        self.size = max(1, int(round(2.0 * self.half_extent / self.resolution)))
        self.free_cells = [0] * (self.size * self.size)

        # RELIABLE 同时兼容 RELIABLE / BEST_EFFORT 订阅者
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)

        self.grid_pub = self.create_publisher(
            OccupancyGrid, '/perception/occupancy_grid_map/map', qos)
        self.cloud_pub = self.create_publisher(
            PointCloud2, '/perception/obstacle_segmentation/pointcloud', qos)
        self.create_subscription(
            Odometry, '/localization/kinematic_state', self.on_odometry, qos)

        self.ego = None
        self.create_timer(1.0 / max(grid_rate, 0.1), self.publish_grid)
        self.create_timer(1.0 / max(cloud_rate, 0.1), self.publish_cloud)

        self.get_logger().info(
            f'occupancy grid: {self.size}x{self.size} @ {self.resolution} m, '
            f'±{self.half_extent} m around ego (all free)')
        self.get_logger().info(
            'pointcloud: 1 far-away point on /perception/obstacle_segmentation/pointcloud')

    def on_odometry(self, msg: Odometry):
        self.ego = msg.pose.pose

    def publish_grid(self):
        if self.ego is None:
            return

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.grid_frame
        msg.info.resolution = self.resolution
        msg.info.width = self.size
        msg.info.height = self.size
        msg.info.origin.position.x = self.ego.position.x - self.half_extent
        msg.info.origin.position.y = self.ego.position.y - self.half_extent
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = self.free_cells
        self.grid_pub.publish(msg)

    def publish_cloud(self):
        # 不能发 width=0 的空点云：pcl::fromROSMsg 之后下游模块会取首点/包围盒
        # 而崩溃（实测 behavior_planning_container 段错误）。这里放一个 1 km 外的
        # 孤立点，既满足"有点"也不构成障碍。
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.cloud_frame
        msg.height = 1
        msg.width = 1
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12
        msg.is_dense = True
        msg.data = struct.pack('<fff', 1000.0, 1000.0, 0.0)
        self.cloud_pub.publish(msg)


def main():
    rclpy.init()
    node = ReplayInputsPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
