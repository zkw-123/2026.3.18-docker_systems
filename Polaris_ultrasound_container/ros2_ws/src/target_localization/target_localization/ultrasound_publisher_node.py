#!/usr/bin/env python3
# coding: UTF-8

"""
Ultrasound Publisher Node

功能：
1. 从视频设备采集超声图像
2. 添加精确的ROS2时间戳
3. 双发布策略：
   - 实时话题：BEST_EFFORT, depth=1 (用于实时处理)
   - 录制话题：RELIABLE, depth=100 (用于数据保存)
4. 极简设计，不做任何图像处理，最小化延迟

作者：Your Name
日期：2024
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class UltrasoundPublisherNode(Node):
    """超声图像采集和发布节点"""
    
    def __init__(self):
        super().__init__('ultrasound_publisher')
        
        # ==================== 参数声明 ====================
        self.declare_parameter('device_id', 6)
        self.declare_parameter('frame_rate', 30.0)
        self.declare_parameter('image_width', 1920)
        self.declare_parameter('image_height', 1080)
        self.declare_parameter('enable_recording', False)
        
        # 获取参数
        self.device_id = self.get_parameter('device_id').value
        self.frame_rate = self.get_parameter('frame_rate').value
        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value
        self.enable_recording = self.get_parameter('enable_recording').value
        
        # ==================== 初始化摄像头 ====================
        self.cap = cv2.VideoCapture(self.device_id)
        
        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open video device {self.device_id}")
            raise RuntimeError(f"Cannot open device {self.device_id}")
        
        # 设置摄像头参数
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.image_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.image_height)
        self.cap.set(cv2.CAP_PROP_FPS, self.frame_rate)
        
        # 验证实际设置
        actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        
        self.get_logger().info(f"Camera initialized: {actual_width}x{actual_height} @ {actual_fps}fps")
        
        # ==================== QoS配置 ====================
        # 实时QoS：最低延迟，可丢帧
        realtime_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1  # 只保留最新一帧
        )
        
        # 录制QoS：可靠传输，大缓冲
        record_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=100  # 大缓冲，防止丢失
        )
        
        # ==================== 创建发布器 ====================
        # 实时话题：用于实时处理
        self.realtime_publisher = self.create_publisher(
            Image,
            'us_image_raw',
            realtime_qos
        )
        
        # 录制话题：用于数据保存（可选）
        if self.enable_recording:
            self.record_publisher = self.create_publisher(
                Image,
                'us_image_record',
                record_qos
            )
            self.get_logger().info("Recording topic enabled: us_image_record")
        else:
            self.record_publisher = None
            self.get_logger().info("Recording disabled")
        
        # ==================== CV Bridge ====================
        self.bridge = CvBridge()
        
        # ==================== 统计信息 ====================
        self.frame_count = 0
        self.failed_captures = 0
        self.start_time = self.get_clock().now()
        
        # ==================== 创建定时器 ====================
        timer_period = 1.0 / self.frame_rate  # 秒
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("Ultrasound Publisher Node initialized")
        self.get_logger().info(f"Device ID: {self.device_id}")
        self.get_logger().info(f"Target frame rate: {self.frame_rate} Hz")
        self.get_logger().info(f"Resolution: {actual_width}x{actual_height}")
        self.get_logger().info(f"Publishing to: us_image_raw")
        self.get_logger().info("=" * 60)
    
    def timer_callback(self):
        """定时器回调：采集图像并发布"""
        
        # ==================== 1. 采集图像 ====================
        ret, frame = self.cap.read()
        
        if not ret:
            self.failed_captures += 1
            self.get_logger().warn(
                f"Failed to capture frame (total failures: {self.failed_captures})"
            )
            return
        
        # ==================== 2. 立即打时间戳 ====================
        # 关键：采集后立即打戳，记录真实采集时刻
        capture_timestamp = self.get_clock().now()
        
        # ==================== 3. 创建ROS消息 ====================
        try:
            image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            image_msg.header.stamp = capture_timestamp.to_msg()
            image_msg.header.frame_id = "ultrasound_frame"
            
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {str(e)}")
            return
        
        # ==================== 4. 发布到实时话题 ====================
        # 优先级最高，必须立即发布
        self.realtime_publisher.publish(image_msg)
        
        # ==================== 5. 发布到录制话题（可选）====================
        if self.record_publisher is not None:
            # 异步传输，不阻塞实时流程
            self.record_publisher.publish(image_msg)
        
        # ==================== 6. 更新统计 ====================
        self.frame_count += 1
        
        # 每秒输出一次统计信息
        if self.frame_count % int(self.frame_rate) == 0:
            self.log_statistics()
    
    def log_statistics(self):
        """输出统计信息"""
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        actual_fps = self.frame_count / elapsed_time if elapsed_time > 0 else 0
        success_rate = (self.frame_count / (self.frame_count + self.failed_captures) * 100 
                       if (self.frame_count + self.failed_captures) > 0 else 0)
        
        self.get_logger().info(
            f"[US Stats] Frames: {self.frame_count}, "
            f"Actual FPS: {actual_fps:.1f}, "
            f"Success rate: {success_rate:.1f}%, "
            f"Failed: {self.failed_captures}"
        )
    
    def destroy_node(self):
        """清理资源"""
        self.get_logger().info("Shutting down Ultrasound Publisher Node...")
        
        # 最终统计
        self.log_statistics()
        
        # 释放摄像头
        if self.cap and self.cap.isOpened():
            self.cap.release()
            self.get_logger().info("Camera released")
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = UltrasoundPublisherNode()
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\n[US Publisher] Interrupted by user")
        
    except Exception as e:
        print(f"[US Publisher] Error: {str(e)}")
        
    finally:
        if node is not None:
            node.destroy_node()
        
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
