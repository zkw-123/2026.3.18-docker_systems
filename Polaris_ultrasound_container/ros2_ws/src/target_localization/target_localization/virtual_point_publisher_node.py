#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Virtual Point Publisher Node
订阅同步数据并发布随机虚拟坐标
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
import json
import random
import time


class VirtualPointPublisherNode(Node):
    """监听 /synchronized_data 并发布随机虚拟坐标"""

    def __init__(self):
        super().__init__('virtual_point_publisher')

        # QoS 配置：可靠传输
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 订阅同步数据
        self.sync_sub = self.create_subscription(
            String,
            '/synchronized_data',
            self.sync_callback,
            qos_profile
        )

        # 发布虚拟点
        self.virtual_pub = self.create_publisher(
            String,
            '/virtual_point',
            qos_profile
        )

        self.get_logger().info('=' * 60)
        self.get_logger().info('Virtual Point Publisher Node started')
        self.get_logger().info('Subscribed to /synchronized_data')
        self.get_logger().info('Publishing to /virtual_point')
        self.get_logger().info('=' * 60)

    def sync_callback(self, msg):
        """当接收到同步数据时回调"""
        try:
            data = json.loads(msg.data)
            sync_time = data.get('sync_timestamp', time.time())

            # 生成随机虚拟坐标
            virtual_point = [round(random.uniform(0, 1), 3) for _ in range(3)]

            # 构建发布消息
            output = {
                'timestamp': time.time(),
                'sync_timestamp': sync_time,
                'virtual_point': virtual_point
            }

            msg_out = String()
            msg_out.data = json.dumps(output)
            self.virtual_pub.publish(msg_out)

            self.get_logger().info(
                f'Published virtual point {virtual_point} (sync_time={sync_time:.3f})'
            )

        except Exception as e:
            self.get_logger().error(f'Error processing synchronized data: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = VirtualPointPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

