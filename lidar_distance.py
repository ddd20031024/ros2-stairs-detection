#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math


class LidarDistanceReader(Node):
    """
    订阅激光雷达的 /scan 话题,读取正前方(索引0)的距离值并打印。
    注意：不同雷达的 0° 朝向可能不同，若方向不对可调整索引为 len(msg.ranges)//2。
    """

    def __init__(self):
        super().__init__('lidar_distance_reader')
        # 创建订阅者，订阅 /scan 话题，队列大小 10
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.get_logger().info('测距节点已启动，正在监听 /scan...')

    def scan_callback(self, msg: LaserScan):
        """
        回调函数：处理接收到的 LaserScan 消息。
        """
        # 获取正前方距离（通常索引0对应雷达物理正前方）
        front_distance = msg.ranges[0]

        # 判断距离是否有效：无穷大、NaN 或 0.0 均视为无效
        if not math.isfinite(front_distance) or front_distance == 0.0:
            self.get_logger().info('正前方无障碍物或超出有效量程')
        else:
            self.get_logger().info(f'正前方障碍物距离: {front_distance:.3f} 米')


def main(args=None):
    rclpy.init(args=args)
    node = LidarDistanceReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass  # 允许 Ctrl+C 优雅退出
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()