#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Data Synchronizer Node
数据同步节点

功能 (Functions):
    - 同步超声图像和NDI追踪数据 (Synchronize ultrasound images with NDI tracking data)
    - 时间戳匹配（容差<5ms）(Timestamp matching with <5ms tolerance)
    - 质量检查：marker可见性和追踪质量 (Quality check: marker visibility and tracking quality)
    - 只发布配对成功且质量合格的数据 (Only publish successfully paired and quality-passed data)
    - 统计同步率和丢弃原因 (Statistics on sync rate and drop reasons)

话题 (Topics):
    订阅 (Subscribers):
        - /us_image_raw: sensor_msgs/Image - 超声图像
        - /ndi_transforms: std_msgs/String - NDI追踪数据
    
    发布 (Publishers):
        - /synchronized_data: std_msgs/String - 配对成功的数据(JSON格式)

参数 (Parameters):
    - marker_name: str - 要追踪的marker名称 (Marker name to track)
    - time_tolerance: float, default=0.005 - 时间同步容差(秒) (Time sync tolerance in seconds)
    - quality_threshold: float, default=0.5 - 最低质量阈值 (Minimum quality threshold)
    - enable_statistics: bool, default=True - 启用统计输出 (Enable statistics output)

作者 (Author): Your Name
日期 (Date): 2024
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import json
import numpy as np

# message_filters用于时间同步 (message_filters for time synchronization)
try:
    from message_filters import ApproximateTimeSynchronizer, Subscriber
except ImportError:
    print("错误: 无法导入 message_filters")
    print("Error: Cannot import message_filters")
    print("请安装: sudo apt-get install ros-<distro>-message-filters")
    sys.exit(1)


class DataSynchronizerNode(Node):
    """
    数据同步节点
    Data Synchronizer Node
    
    职责 (Responsibilities):
        1. 订阅超声图像和NDI数据 (Subscribe to US images and NDI data)
        2. 基于时间戳进行精确匹配 (Precise matching based on timestamps)
        3. 验证marker可见性 (Verify marker visibility)
        4. 检查追踪质量 (Check tracking quality)
        5. 只发布通过所有检查的配对数据 (Only publish paired data that passes all checks)
    
    优势 (Advantages):
        - 节省下游计算资源（只处理有效数据）
          (Save downstream computing resources - only process valid data)
        - 提供详细的同步统计信息
          (Provide detailed synchronization statistics)
        - 集中管理数据质量检查
          (Centralized data quality checking)
    """
    
    def __init__(self):
        super().__init__('data_synchronizer')
        
        # ============================================
        # 参数声明 (Parameter Declaration)
        # ============================================
        self.declare_parameter('marker_name', 'probe')
        self.declare_parameter('time_tolerance', 0.005)  # 5ms
        self.declare_parameter('quality_threshold', 0.5)
        self.declare_parameter('enable_statistics', True)
        self.declare_parameter('statistics_interval', 30)  # 每30帧输出一次统计
        
        # 获取参数 (Get parameters)
        self.marker_name = self.get_parameter('marker_name').value
        self.time_tolerance = self.get_parameter('time_tolerance').value
        self.quality_threshold = self.get_parameter('quality_threshold').value
        self.enable_statistics = self.get_parameter('enable_statistics').value
        self.statistics_interval = self.get_parameter('statistics_interval').value
        
        self.get_logger().info(f'目标marker (Target marker): {self.marker_name}')
        self.get_logger().info(f'时间容差 (Time tolerance): {self.time_tolerance*1000:.1f}ms')
        self.get_logger().info(f'质量阈值 (Quality threshold): {self.quality_threshold}')
        
        # ============================================
        # 统计信息 (Statistics)
        # ============================================
        self.total_callbacks = 0        # 总回调次数 (Total callbacks)
        self.synced_count = 0           # 成功同步次数 (Successful syncs)
        self.dropped_visibility = 0     # 因marker不可见丢弃 (Dropped due to invisibility)
        self.dropped_quality = 0        # 因质量过低丢弃 (Dropped due to low quality)
        self.dropped_parse_error = 0    # 因解析错误丢弃 (Dropped due to parse error)
        
        # ============================================
        # 创建订阅器 (Create Subscribers)
        # ============================================
        # 注意：使用message_filters.Subscriber而不是普通的create_subscription
        # Note: Use message_filters.Subscriber instead of regular create_subscription
        
        # QoS: 使用BEST_EFFORT匹配上游发布器
        # QoS: Use BEST_EFFORT to match upstream publishers
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 超声图像订阅器 (Ultrasound image subscriber)
        self.image_sub = Subscriber(
            self,
            Image,
            '/us_image_raw',
            qos_profile=qos_profile
        )
        
        # NDI数据订阅器 (NDI data subscriber)
        self.ndi_sub = Subscriber(
            self,
            String,
            '/ndi_transforms',
            qos_profile=qos_profile
        )
        
        self.get_logger().info('已创建message_filters订阅器')
        self.get_logger().info('Created message_filters subscribers')
        
        # ============================================
        # 创建时间同步器 (Create Time Synchronizer)
        # ============================================
        # ApproximateTimeSynchronizer: 允许时间戳有小的偏差
        # ApproximateTimeSynchronizer: Allow small timestamp deviation
        self.sync = ApproximateTimeSynchronizer(
            [self.image_sub, self.ndi_sub],
            queue_size=10,
            slop=self.time_tolerance,  # 时间容差 (Time tolerance)
            allow_headerless=True 
        )
        
        # 注册回调函数 (Register callback)
        self.sync.registerCallback(self.sync_callback)
        
        self.get_logger().info('时间同步器已创建')
        self.get_logger().info('Time synchronizer created')
        
        # ============================================
        # 创建发布器 (Create Publisher)
        # ============================================
        # 发布同步后的数据（JSON格式）
        # Publish synchronized data (JSON format)
        sync_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,  # 配对后的数据很重要，使用可靠传输
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.synced_publisher = self.create_publisher(
            String,
            '/synchronized_data',
            sync_qos
        )
        
        self.get_logger().info('创建同步数据发布器: /synchronized_data')
        self.get_logger().info('Created synchronized data publisher: /synchronized_data')
        
        # ============================================
        # 工具初始化 (Tool Initialization)
        # ============================================
        self.bridge = CvBridge()
        
        # ============================================
        # 启动信息 (Startup Information)
        # ============================================
        self.get_logger().info('=' * 60)
        self.get_logger().info('数据同步节点已启动 (Data Synchronizer Node Started)')
        self.get_logger().info('等待超声图像和NDI数据...')
        self.get_logger().info('Waiting for ultrasound images and NDI data...')
        self.get_logger().info('=' * 60)
    
    def sync_callback(self, image_msg, ndi_msg):
        """
        同步回调函数：当图像和NDI数据时间戳匹配时调用
        Sync callback: called when image and NDI data timestamps match
        
        参数 (Parameters):
            image_msg: sensor_msgs/Image - 超声图像消息
            ndi_msg: std_msgs/String - NDI数据消息(JSON)
        
        处理流程 (Processing flow):
            1. 解析NDI数据 (Parse NDI data)
            2. 检查目标marker是否可见 (Check if target marker is visible)
            3. 检查追踪质量 (Check tracking quality)
            4. 如果全部通过，发布配对数据 (If all checks pass, publish paired data)
            5. 更新统计信息 (Update statistics)
        """
        
        self.total_callbacks += 1
        
        # ============================================
        # 步骤1: 解析NDI数据 (Step 1: Parse NDI Data)
        # ============================================
        try:
            ndi_data = json.loads(ndi_msg.data)
        except json.JSONDecodeError as e:
            self.dropped_parse_error += 1
            self.get_logger().warn(f'NDI数据解析失败 (Failed to parse NDI data): {str(e)}')
            return
        except Exception as e:
            self.dropped_parse_error += 1
            self.get_logger().error(f'处理NDI数据时出错 (Error processing NDI data): {str(e)}')
            return
        
        # ============================================
        # 步骤2: 查找目标marker (Step 2: Find Target Marker)
        # ============================================
        marker_found = False
        marker_quality = 0.0
        marker_transform = None
        
        for transform in ndi_data.get('transforms', []):
            tool_name = transform.get('tool_name', '')
            
            if tool_name == self.marker_name:
                marker_found = True
                marker_quality = transform.get('quality', 0.0)
                marker_transform = transform
                break
        
        # 检查marker是否可见 (Check if marker is visible)
        if not marker_found:
            self.dropped_visibility += 1
            self.get_logger().debug(
                f'Marker "{self.marker_name}" 不可见，丢弃此帧 '
                f'(Marker "{self.marker_name}" not visible, dropping frame)'
            )
            self._log_statistics()
            return
        
        # ============================================
        # 步骤3: 检查追踪质量 (Step 3: Check Tracking Quality)
        # ============================================
        if marker_quality < self.quality_threshold:
            self.dropped_quality += 1
            self.get_logger().debug(
                f'质量过低 ({marker_quality:.2f} < {self.quality_threshold}), 丢弃此帧 '
                f'(Quality too low ({marker_quality:.2f} < {self.quality_threshold}), dropping frame)'
            )
            self._log_statistics()
            return
        
        # ============================================
        # 步骤4: 配对成功！构建同步数据 (Step 4: Pairing Success! Build Synced Data)
        # ============================================
        self.synced_count += 1
        
        # 计算时间差（用于监控同步质量）
        # Calculate time difference (for monitoring sync quality)
        image_time = image_msg.header.stamp.sec + image_msg.header.stamp.nanosec / 1e9
        ndi_time = ndi_data.get('timestamp', 0.0)
        time_diff = abs(image_time - ndi_time)
        
        # 构建同步数据包 (Build synchronized data package)
        synced_data = {
            # 时间信息 (Timing information)
            'sync_timestamp': image_time,  # 使用图像时间戳作为同步时间
            'image_timestamp': image_time,
            'ndi_timestamp': ndi_time,
            'time_difference': time_diff,
            
            # 图像信息 (Image information)
            'image': {
                'width': image_msg.width,
                'height': image_msg.height,
                'encoding': image_msg.encoding,
                'header': {
                    'stamp': {
                        'sec': image_msg.header.stamp.sec,
                        'nanosec': image_msg.header.stamp.nanosec
                    },
                    'frame_id': image_msg.header.frame_id
                }
            },
            
            # NDI追踪信息 (NDI tracking information)
            'ndi': {
                'marker_name': self.marker_name,
                'marker_quality': marker_quality,
                'marker_transform': marker_transform,
                'all_transforms': ndi_data.get('transforms', [])
            },
            
            # 同步质量信息 (Sync quality information)
            'sync_quality': {
                'time_diff_ms': time_diff * 1000,
                'quality': marker_quality,
                'frame_number': self.synced_count
            }
        }
        
        # ============================================
        # 步骤5: 发布同步数据 (Step 5: Publish Synchronized Data)
        # ============================================
        try:
            msg = String()
            msg.data = json.dumps(synced_data)
            self.synced_publisher.publish(msg)
            
            self.get_logger().debug(
                f'已发布同步数据 #{self.synced_count}, '
                f'时间差: {time_diff*1000:.2f}ms, '
                f'质量: {marker_quality:.2f}'
            )
            
        except Exception as e:
            self.get_logger().error(f'发布同步数据时出错 (Error publishing synced data): {str(e)}')
        
        # ============================================
        # 步骤6: 输出统计信息 (Step 6: Log Statistics)
        # ============================================
        self._log_statistics()
    
    def _log_statistics(self):
        """
        输出统计信息（定期）
        Log statistics (periodically)
        """
        if not self.enable_statistics:
            return
        
        # 每N次回调输出一次统计 (Log statistics every N callbacks)
        if self.total_callbacks % self.statistics_interval == 0:
            # 计算同步率 (Calculate sync rate)
            if self.total_callbacks > 0:
                sync_rate = (self.synced_count / self.total_callbacks) * 100
            else:
                sync_rate = 0.0
            
            # 输出详细统计 (Output detailed statistics)
            self.get_logger().info('=' * 60)
            self.get_logger().info('同步统计 (Synchronization Statistics):')
            self.get_logger().info(f'  总回调次数 (Total callbacks): {self.total_callbacks}')
            self.get_logger().info(f'  成功同步 (Successful syncs): {self.synced_count}')
            self.get_logger().info(f'  同步率 (Sync rate): {sync_rate:.1f}%')
            self.get_logger().info('丢弃原因 (Drop reasons):')
            self.get_logger().info(f'  不可见 (Invisible): {self.dropped_visibility}')
            self.get_logger().info(f'  质量低 (Low quality): {self.dropped_quality}')
            self.get_logger().info(f'  解析错误 (Parse error): {self.dropped_parse_error}')
            self.get_logger().info('=' * 60)
    
    def get_statistics(self):
        """
        获取统计信息字典
        Get statistics dictionary
        
        返回 (Returns):
            dict: 包含所有统计信息的字典
        """
        return {
            'total_callbacks': self.total_callbacks,
            'synced_count': self.synced_count,
            'dropped_visibility': self.dropped_visibility,
            'dropped_quality': self.dropped_quality,
            'dropped_parse_error': self.dropped_parse_error,
            'sync_rate': (self.synced_count / self.total_callbacks * 100) 
                        if self.total_callbacks > 0 else 0.0
        }
    
    def destroy_node(self):
        """
        节点销毁时的清理工作
        Cleanup when node is destroyed
        """
        self.get_logger().info('正在关闭数据同步节点...')
        self.get_logger().info('Shutting down data synchronizer node...')
        
        # 输出最终统计 (Output final statistics)
        stats = self.get_statistics()
        self.get_logger().info('最终统计 (Final Statistics):')
        self.get_logger().info(f'  总回调: {stats["total_callbacks"]}')
        self.get_logger().info(f'  成功同步: {stats["synced_count"]}')
        self.get_logger().info(f'  同步率: {stats["sync_rate"]:.1f}%')
        
        super().destroy_node()


def main(args=None):
    """
    主函数 (Main function)
    """
    # 初始化ROS2 (Initialize ROS2)
    rclpy.init(args=args)
    
    # 创建节点 (Create node)
    node = None
    try:
        node = DataSynchronizerNode()
        
        # 运行节点 (Run node)
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print('\n用户中断 (User interrupted)')
    except Exception as e:
        print(f'错误 (Error): {str(e)}')
    finally:
        # 清理资源 (Cleanup)
        if node is not None:
            node.destroy_node()
        
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
