#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Target Localizer Node
目标3D定位节点

功能 (Functions):
    - 接收像素坐标和NDI追踪数据 (Receive pixel coordinates and NDI tracking data)
    - 使用标定矩阵计算3D世界坐标 (Calculate 3D world coordinates using calibration matrix)
    - 发布目标的3D位置 (Publish target 3D position)

话题 (Topics):
    订阅 (Subscribers):
        - /synchronized_data: std_msgs/String - 同步数据(包含NDI信息)
        - /target_pixel: std_msgs/String - 像素坐标
    
    发布 (Publishers):
        - /target_3d_position: std_msgs/String - 3D位置(JSON格式)

参数 (Parameters):
    - marker_name: str - 追踪的marker名称
    - pixel_to_mm_x: float - X方向像素到毫米转换比例
    - pixel_to_mm_y: float - Y方向像素到毫米转换比例
    - image_origin_x: float - 图像原点X偏移(mm)
    - image_origin_y: float - 图像原点Y偏移(mm)
    - marker_to_image_transform: list[16] - 4x4变换矩阵(从marker到图像坐标系)

作者 (Author): Your Name
日期 (Date): 2024
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
import json
import numpy as np
import time


class TargetLocalizerNode(Node):
    """
    目标3D定位节点
    Target 3D Localizer Node
    
    职责 (Responsibilities):
        1. 接收像素坐标和NDI数据 (Receive pixel coords and NDI data)
        2. 像素坐标转换为图像坐标系物理坐标 (Convert pixel to image frame physical coords)
        3. 应用标定的变换矩阵 (Apply calibrated transformation matrix)
        4. 计算目标在世界坐标系中的3D位置 (Calculate target 3D position in world frame)
    
    坐标系变换链 (Coordinate Transformation Chain):
        Pixel (px) → Image (mm) → Marker (mm) → World (mm)
    """
    
    def __init__(self):
        super().__init__('target_localizer')
        
        # ============================================
        # 参数声明 (Parameter Declaration)
        # ============================================
        self.declare_parameter('marker_name', 'probe')
        
        # 图像标定参数 (Image calibration parameters)
        self.declare_parameter('pixel_to_mm_x', 0.1)
        self.declare_parameter('pixel_to_mm_y', 0.1)
        self.declare_parameter('image_origin_x', 0.0)
        self.declare_parameter('image_origin_y', 0.0)
        
        # marker到图像的变换矩阵 (Marker to image transformation matrix)
        # 默认为单位矩阵 (Default to identity matrix)
        self.declare_parameter('marker_to_image_transform', 
                              [1.0, 0.0, 0.0, 0.0,
                               0.0, 1.0, 0.0, 0.0,
                               0.0, 0.0, 1.0, 0.0,
                               0.0, 0.0, 0.0, 1.0])
        
        # 获取参数 (Get parameters)
        self.marker_name = self.get_parameter('marker_name').value
        self.pixel_to_mm_x = self.get_parameter('pixel_to_mm_x').value
        self.pixel_to_mm_y = self.get_parameter('pixel_to_mm_y').value
        self.image_origin_x = self.get_parameter('image_origin_x').value
        self.image_origin_y = self.get_parameter('image_origin_y').value
        
        # 解析变换矩阵 (Parse transformation matrix)
        transform_list = self.get_parameter('marker_to_image_transform').value
        self.marker_to_image = np.array(transform_list).reshape(4, 4)
        
        # 计算逆矩阵（从图像到marker）(Compute inverse: image to marker)
        try:
            self.image_to_marker = np.linalg.inv(self.marker_to_image)
        except np.linalg.LinAlgError:
            self.get_logger().error('变换矩阵不可逆！(Transformation matrix is singular!)')
            raise
        
        # 输出标定参数 (Output calibration parameters)
        self.get_logger().info('=' * 60)
        self.get_logger().info('标定参数 (Calibration Parameters):')
        self.get_logger().info(f'  Marker名称 (Marker name): {self.marker_name}')
        self.get_logger().info(f'  像素→mm X (Pixel to mm X): {self.pixel_to_mm_x}')
        self.get_logger().info(f'  像素→mm Y (Pixel to mm Y): {self.pixel_to_mm_y}')
        self.get_logger().info(f'  图像原点偏移 (Image origin): ({self.image_origin_x}, {self.image_origin_y}) mm')
        self.get_logger().info('  Marker→Image变换矩阵:')
        self.get_logger().info(f'\n{self.marker_to_image}')
        self.get_logger().info('=' * 60)
        
        # ============================================
        # 统计信息 (Statistics)
        # ============================================
        self.localized_count = 0
        self.total_computation_time = 0.0
        self.sync_failures = 0
        
        # 缓存最新数据 (Cache latest data)
        self.latest_synced_data = None
        self.latest_pixel_data = None
        
        # ============================================
        # QoS配置 (QoS Configuration)
        # ============================================
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # ============================================
        # 创建订阅器 (Create Subscribers)
        # ============================================
        
        # 订阅同步数据（包含NDI信息）(Subscribe to synced data with NDI info)
        self.synced_subscription = self.create_subscription(
            String,
            '/synchronized_data',
            self.synced_data_callback,
            qos_profile
        )
        
        # 订阅像素坐标 (Subscribe to pixel coordinates)
        self.pixel_subscription = self.create_subscription(
            String,
            '/target_pixel',
            self.pixel_callback,
            qos_profile
        )
        
        self.get_logger().info('已订阅: /synchronized_data, /target_pixel')
        self.get_logger().info('Subscribed to: /synchronized_data, /target_pixel')
        
        # ============================================
        # 创建发布器 (Create Publisher)
        # ============================================
        self.position_publisher = self.create_publisher(
            String,
            '/target_3d_position',
            qos_profile
        )
        self.get_logger().info('创建发布器: /target_3d_position')
        self.get_logger().info('Created publisher: /target_3d_position')
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('目标定位节点已启动 (Target Localizer Node Started)')
        self.get_logger().info('等待同步数据和像素坐标...')
        self.get_logger().info('Waiting for synced data and pixel coordinates...')
        self.get_logger().info('=' * 60)
    
    def synced_data_callback(self, msg):
        """
        同步数据回调：缓存NDI数据
        Synced data callback: cache NDI data
        """
        try:
            self.latest_synced_data = json.loads(msg.data)
            
            # 尝试与最新的像素数据匹配并计算 (Try to match with latest pixel data)
            self.try_compute_3d_position()
            
        except Exception as e:
            self.get_logger().error(f'处理同步数据时出错 (Error processing synced data): {str(e)}')
    
    def pixel_callback(self, msg):
        """
        像素坐标回调：缓存检测结果
        Pixel callback: cache detection result
        """
        try:
            self.latest_pixel_data = json.loads(msg.data)
            
            # 尝试与最新的同步数据匹配并计算 (Try to match with latest synced data)
            self.try_compute_3d_position()
            
        except Exception as e:
            self.get_logger().error(f'处理像素数据时出错 (Error processing pixel data): {str(e)}')
    
    def try_compute_3d_position(self):
        """
        尝试计算3D位置（当两个数据都可用时）
        Try to compute 3D position (when both data are available)
        """
        # 检查是否两个数据都已接收 (Check if both data are received)
        if self.latest_synced_data is None or self.latest_pixel_data is None:
            return
        
        # 检查时间戳是否匹配 (Check if timestamps match)
        synced_timestamp = self.latest_synced_data.get('sync_timestamp')
        pixel_timestamp = self.latest_pixel_data.get('timestamp')
        
        if synced_timestamp is None or pixel_timestamp is None:
            self.get_logger().warn('时间戳缺失 (Missing timestamps)')
            return
        
        # 检查时间差 (Check time difference)
        time_diff = abs(synced_timestamp - pixel_timestamp)
        if time_diff > 0.05:  # 50ms容差
            self.sync_failures += 1
            self.get_logger().debug(
                f'时间戳不匹配: 差值={time_diff*1000:.1f}ms '
                f'(Timestamps mismatch: diff={time_diff*1000:.1f}ms)'
            )
            return
        
        # ============================================
        # 执行3D位置计算 (Perform 3D Position Calculation)
        # ============================================
        try:
            start_time = time.time()
            
            position_3d = self.compute_3d_position(
                self.latest_pixel_data,
                self.latest_synced_data
            )
            
            computation_time = time.time() - start_time
            self.total_computation_time += computation_time
            
            if position_3d is not None:
                self.localized_count += 1
                
                # 发布3D位置 (Publish 3D position)
                self.publish_3d_position(position_3d, synced_timestamp, computation_time)
                
                # 清空缓存（避免重复计算）(Clear cache to avoid duplicate computation)
                self.latest_synced_data = None
                self.latest_pixel_data = None
        
        except Exception as e:
            self.get_logger().error(f'计算3D位置时出错 (Error computing 3D position): {str(e)}')
    
    def compute_3d_position(self, pixel_data, synced_data):
        """
        计算目标的3D世界坐标
        Compute target 3D world coordinates
        
        参数 (Parameters):
            pixel_data: dict - 像素坐标数据
            synced_data: dict - 同步数据（包含NDI信息）
        
        返回 (Returns):
            numpy.ndarray: 3D位置 [x, y, z] 单位mm
        
        计算流程 (Computation Flow):
            1. 像素坐标 → 图像坐标系物理坐标
               Pixel coords → Image frame physical coords
            
            2. 图像坐标 → Marker坐标系
               Image coords → Marker frame
            
            3. Marker坐标 → 世界坐标系
               Marker coords → World frame
        """
        
        # ============================================
        # 步骤1: 提取像素坐标 (Step 1: Extract Pixel Coordinates)
        # ============================================
        pixel_x = pixel_data.get('pixel_x')
        pixel_y = pixel_data.get('pixel_y')
        
        if pixel_x is None or pixel_y is None:
            self.get_logger().warn('像素坐标缺失 (Missing pixel coordinates)')
            return None
        
        # ============================================
        # 步骤2: 像素 → 图像物理坐标 (Step 2: Pixel → Image Physical Coords)
        # ============================================
        # 转换为毫米 (Convert to millimeters)
        image_x = pixel_x * self.pixel_to_mm_x + self.image_origin_x
        image_y = pixel_y * self.pixel_to_mm_y + self.image_origin_y
        image_z = 0.0  # 假设目标在图像平面 (Assume target is on image plane)
        
        # 构建齐次坐标 (Build homogeneous coordinates)
        point_in_image = np.array([image_x, image_y, image_z, 1.0])
        
        self.get_logger().debug(
            f'像素坐标 (Pixel): ({pixel_x:.1f}, {pixel_y:.1f}) → '
            f'图像坐标 (Image): ({image_x:.2f}, {image_y:.2f}, {image_z:.2f}) mm'
        )
        
        # ============================================
        # 步骤3: 图像坐标 → Marker坐标 (Step 3: Image → Marker Coords)
        # ============================================
        point_in_marker = self.image_to_marker @ point_in_image
        
        self.get_logger().debug(
            f'Marker坐标 (Marker): '
            f'({point_in_marker[0]:.2f}, {point_in_marker[1]:.2f}, {point_in_marker[2]:.2f}) mm'
        )
        
        # ============================================
        # 步骤4: 提取Marker的世界变换 (Step 4: Extract Marker World Transform)
        # ============================================
        ndi_info = synced_data.get('ndi', {})
        marker_transform_dict = ndi_info.get('marker_transform', {})
        marker_matrix = marker_transform_dict.get('matrix')
        
        if marker_matrix is None:
            self.get_logger().warn('Marker变换矩阵缺失 (Missing marker transform)')
            return None
        
        # 转换为numpy数组 (Convert to numpy array)
        marker_to_world = np.array(marker_matrix)
        
        if marker_to_world.shape != (4, 4):
            self.get_logger().warn(f'Marker变换矩阵形状错误: {marker_to_world.shape}')
            return None
        
        # ============================================
        # 步骤5: Marker坐标 → 世界坐标 (Step 5: Marker → World Coords)
        # ============================================
        point_in_world = marker_to_world @ point_in_marker
        
        # 提取3D坐标（去掉齐次坐标）(Extract 3D coords without homogeneous)
        position_3d = point_in_world[:3]
        
        self.get_logger().debug(
            f'世界坐标 (World): '
            f'({position_3d[0]:.2f}, {position_3d[1]:.2f}, {position_3d[2]:.2f}) mm'
        )
        
        return position_3d
    
    def publish_3d_position(self, position_3d, timestamp, computation_time):
        """
        发布3D位置
        Publish 3D position
        
        参数 (Parameters):
            position_3d: numpy.ndarray - 3D位置 [x, y, z]
            timestamp: float - 时间戳
            computation_time: float - 计算耗时(秒)
        """
        
        # 构建消息 (Build message)
        position_msg = {
            'timestamp': timestamp,
            'position': {
                'x': float(position_3d[0]),
                'y': float(position_3d[1]),
                'z': float(position_3d[2])
            },
            'unit': 'mm',
            'frame_id': 'world',
            'computation_time_ms': computation_time * 1000,
            'frame_number': self.localized_count
        }
        
        # 发布 (Publish)
        msg = String()
        msg.data = json.dumps(position_msg)
        self.position_publisher.publish(msg)
        
        # 日志 (Logging)
        self.get_logger().info(
            f'[{self.localized_count}] 3D位置: '
            f'({position_3d[0]:.2f}, {position_3d[1]:.2f}, {position_3d[2]:.2f}) mm | '
            f'耗时: {computation_time*1000:.2f}ms'
        )
        
        # 定期统计 (Periodic statistics)
        if self.localized_count % 30 == 0:
            avg_time = self.total_computation_time / self.localized_count * 1000
            self.get_logger().info(
                f'已定位 {self.localized_count} 帧 | '
                f'Localized {self.localized_count} frames | '
                f'平均耗时 (Avg time): {avg_time:.2f}ms | '
                f'同步失败 (Sync failures): {self.sync_failures}'
            )
    
    def destroy_node(self):
        """
        节点销毁时的清理工作
        Cleanup when node is destroyed
        """
        self.get_logger().info('正在关闭目标定位节点...')
        self.get_logger().info('Shutting down target localizer node...')
        
        # 输出最终统计 (Output final statistics)
        if self.localized_count > 0:
            avg_time = self.total_computation_time / self.localized_count * 1000
            
            self.get_logger().info('最终统计 (Final Statistics):')
            self.get_logger().info(f'  定位帧数 (Localized frames): {self.localized_count}')
            self.get_logger().info(f'  同步失败 (Sync failures): {self.sync_failures}')
            self.get_logger().info(f'  平均耗时 (Average time): {avg_time:.2f}ms')
        
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
        node = TargetLocalizerNode()
        
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
