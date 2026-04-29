#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Target Detector Node
目标检测节点

功能 (Functions):
    - 接收已同步的超声图像数据 (Receive synchronized ultrasound image data)
    - 在图像中检测目标位置 (Detect target position in images)
    - 当前使用伪代码实现（易于替换）(Currently uses placeholder code - easy to replace)
    - 发布像素坐标和置信度 (Publish pixel coordinates and confidence)

话题 (Topics):
    订阅 (Subscribers):
        - /synchronized_data: std_msgs/String - 同步后的数据(JSON)
    
    发布 (Publishers):
        - /target_pixel: std_msgs/String - 检测结果(JSON: x, y, confidence)

参数 (Parameters):
    - detection_mode: str, default='center' - 检测模式 (Detection mode)
      选项 (Options): 'center', 'moving', 'random'
    - enable_visualization: bool, default=False - 启用可视化 (Enable visualization)

替换检测算法 (Replace Detection Algorithm):
    只需修改 detect_target() 方法即可
    Simply modify the detect_target() method
    
    示例 (Example):
        def detect_target(self, image):
            # 您的检测算法 (Your detection algorithm)
            result = your_detector.detect(image)
            x, y = result.center
            confidence = result.score
            return x, y, confidence

作者 (Author): Your Name
日期 (Date): 2024
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from cv_bridge import CvBridge
import json
import cv2
import numpy as np
import time
import base64


class TargetDetectorNode(Node):
    """
    目标检测节点
    Target Detector Node
    
    职责 (Responsibilities):
        1. 只处理已配对的图像数据 (Only process paired image data)
        2. 在图像中检测目标位置 (Detect target position in images)
        3. 返回像素坐标和置信度 (Return pixel coordinates and confidence)
        4. 提供易于替换的检测接口 (Provide easy-to-replace detection interface)
    
    设计原则 (Design Principles):
        - 专注单一职责：只做检测 (Single responsibility: detection only)
        - 算法无关：易于替换不同的检测方法 (Algorithm-agnostic: easy to replace)
        - 性能优先：只处理有效数据 (Performance-first: only process valid data)
    """
    
    def __init__(self):
        super().__init__('target_detector')
        
        # ============================================
        # 参数声明 (Parameter Declaration)
        # ============================================
        self.declare_parameter('detection_mode', 'center')
        self.declare_parameter('enable_visualization', False)
        
        # 获取参数 (Get parameters)
        self.detection_mode = self.get_parameter('detection_mode').value
        self.enable_visualization = self.get_parameter('enable_visualization').value
        
        self.get_logger().info(f'检测模式 (Detection mode): {self.detection_mode}')
        self.get_logger().info(f'可视化 (Visualization): {self.enable_visualization}')
        
        # ============================================
        # 统计信息 (Statistics)
        # ============================================
        self.processed_count = 0
        self.detection_failures = 0
        self.total_detection_time = 0.0
        
        # ============================================
        # QoS配置 (QoS Configuration)
        # ============================================
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # ============================================
        # 创建订阅器 (Create Subscriber)
        # ============================================
        # 订阅同步后的数据 (Subscribe to synchronized data)
        self.subscription = self.create_subscription(
            String,
            '/synchronized_data',
            self.synced_data_callback,
            qos_profile
        )
        self.get_logger().info('已订阅: /synchronized_data')
        self.get_logger().info('Subscribed to: /synchronized_data')
        
        # ============================================
        # 创建发布器 (Create Publisher)
        # ============================================
        # 发布检测结果 (Publish detection results)
        self.detection_publisher = self.create_publisher(
            String,
            '/target_pixel',
            qos_profile
        )
        self.get_logger().info('创建发布器: /target_pixel')
        self.get_logger().info('Created publisher: /target_pixel')
        
        # ============================================
        # 工具初始化 (Tool Initialization)
        # ============================================
        self.bridge = CvBridge()
        
        # ============================================
        # 检测算法初始化 (Detection Algorithm Initialization)
        # ============================================
        # TODO: 在这里初始化您的检测模型
        # TODO: Initialize your detection model here
        #
        # 示例 (Example):
        # self.detector = YourDetectorClass()
        # self.detector.load_model('path/to/model')
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('目标检测节点已启动 (Target Detector Node Started)')
        self.get_logger().info('使用伪代码检测算法 (Using placeholder detection algorithm)')
        self.get_logger().info('等待同步数据...')
        self.get_logger().info('Waiting for synchronized data...')
        self.get_logger().info('=' * 60)
    
    def synced_data_callback(self, msg):
        """
        同步数据回调：处理已配对的图像
        Synced data callback: process paired images
        
        参数 (Parameters):
            msg: std_msgs/String - 同步数据(JSON格式)
        """
        
        try:
            # ============================================
            # 步骤1: 解析同步数据 (Step 1: Parse Synced Data)
            # ============================================
            synced_data = json.loads(msg.data)
            
            # 提取图像信息 (Extract image information)
            image_info = synced_data.get('image', {})
            width = image_info.get('width')
            height = image_info.get('height')
            encoding = image_info.get('encoding')
            
            # 提取时间戳 (Extract timestamp)
            sync_timestamp = synced_data.get('sync_timestamp')
            
            # 注意：实际的图像数据需要从原始Image消息获取
            # Note: Actual image data needs to be obtained from original Image message
            # 这里我们从同步数据中重建图像信息
            # Here we reconstruct image info from synced data
            
            self.get_logger().debug(
                f'收到同步数据: {width}x{height}, 编码: {encoding}'
            )
            
            # ============================================
            # 步骤2: 生成模拟图像用于检测 (Step 2: Generate Mock Image for Detection)
            # ============================================
            # 注意：在实际应用中，您需要从原始图像消息获取真实图像
            # Note: In real application, you need to get actual image from original message
            # 
            # 这里我们创建一个空白图像用于演示
            # Here we create a blank image for demonstration
            mock_image = np.zeros((height, width, 3), dtype=np.uint8)
            
            # ============================================
            # 步骤3: 执行目标检测 (Step 3: Perform Target Detection)
            # ============================================
            start_time = time.time()
            
            x, y, confidence = self.detect_target(mock_image)
            
            detection_time = time.time() - start_time
            self.total_detection_time += detection_time
            
            # ============================================
            # 步骤4: 检查检测结果 (Step 4: Check Detection Result)
            # ============================================
            if x is None or y is None:
                self.detection_failures += 1
                self.get_logger().warn('目标检测失败 (Target detection failed)')
                return
            
            self.processed_count += 1
            
            # ============================================
            # 步骤5: 构建并发布检测结果 (Step 5: Build and Publish Detection Result)
            # ============================================
            detection_result = {
                'timestamp': sync_timestamp,
                'pixel_x': float(x),
                'pixel_y': float(y),
                'confidence': float(confidence),
                'image_width': width,
                'image_height': height,
                'detection_time_ms': detection_time * 1000,
                'frame_number': self.processed_count
            }
            
            result_msg = String()
            result_msg.data = json.dumps(detection_result)
            self.detection_publisher.publish(result_msg)
            
            # ============================================
            # 步骤6: 日志输出 (Step 6: Logging)
            # ============================================
            self.get_logger().debug(
                f'检测 #{self.processed_count}: '
                f'位置=({x:.0f}, {y:.0f}), '
                f'置信度={confidence:.2f}, '
                f'耗时={detection_time*1000:.1f}ms'
            )
            
            # 定期输出统计 (Periodic statistics output)
            if self.processed_count % 30 == 0:
                avg_time = self.total_detection_time / self.processed_count * 1000
                self.get_logger().info(
                    f'已处理 {self.processed_count} 帧 | '
                    f'Processed {self.processed_count} frames | '
                    f'平均耗时 (Avg time): {avg_time:.1f}ms | '
                    f'失败 (Failures): {self.detection_failures}'
                )
        
        except Exception as e:
            self.get_logger().error(f'处理同步数据时出错 (Error processing synced data): {str(e)}')
    
    def detect_target(self, image):
        """
        目标检测方法（伪代码实现）
        Target detection method (placeholder implementation)
        
        ⭐ 这是您需要替换的核心方法 ⭐
        ⭐ This is the core method you need to replace ⭐
        
        参数 (Parameters):
            image: numpy.ndarray - BGR格式的图像
        
        返回 (Returns):
            tuple: (x, y, confidence)
                x: float - 像素X坐标
                y: float - 像素Y坐标  
                confidence: float - 置信度 [0, 1]
        
        替换示例 (Replacement Examples):
        
        1. 模板匹配 (Template Matching):
            result = cv2.matchTemplate(image, template, cv2.TM_CCOEFF_NORMED)
            _, max_val, _, max_loc = cv2.minMaxLoc(result)
            x, y = max_loc
            confidence = max_val
        
        2. 深度学习 (Deep Learning):
            detections = self.model.predict(image)
            x, y = detections[0]['center']
            confidence = detections[0]['score']
        
        3. 传统图像处理 (Traditional Image Processing):
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            circles = cv2.HoughCircles(gray, ...)
            x, y = circles[0][0][:2]
            confidence = 1.0
        """
        
        height, width = image.shape[:2]
        
        # ============================================
        # 伪代码实现 (Placeholder Implementation)
        # ============================================
        
        if self.detection_mode == 'center':
            # 模式1: 固定中心点 (Mode 1: Fixed center point)
            x = width / 2
            y = height / 2
            confidence = 1.0
            
            self.get_logger().debug('使用中心点检测模式 (Using center point mode)')
        
        elif self.detection_mode == 'moving':
            # 模式2: 模拟移动目标 (Mode 2: Simulated moving target)
            # 使用正弦波模拟目标运动 (Use sine wave to simulate target movement)
            t = time.time()
            x = width / 2 + 100 * np.sin(t * 2)
            y = height / 2 + 100 * np.cos(t * 2)
            confidence = 0.95
            
            self.get_logger().debug('使用移动目标检测模式 (Using moving target mode)')
        
        elif self.detection_mode == 'random':
            # 模式3: 随机位置（测试用）(Mode 3: Random position for testing)
            x = np.random.randint(width * 0.2, width * 0.8)
            y = np.random.randint(height * 0.2, height * 0.8)
            confidence = np.random.uniform(0.7, 1.0)
            
            self.get_logger().debug('使用随机位置检测模式 (Using random position mode)')
        
        else:
            # 默认：中心点 (Default: center point)
            x = width / 2
            y = height / 2
            confidence = 1.0
        
        return x, y, confidence
    
    # ============================================
    # 以下是一些真实检测算法的示例接口
    # Below are example interfaces for real detection algorithms
    # ============================================
    
    def detect_by_template_matching(self, image, template):
        """
        模板匹配检测（示例）
        Template matching detection (example)
        """
        if template is None:
            return None, None, 0.0
        
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        result = cv2.matchTemplate(gray, template, cv2.TM_CCOEFF_NORMED)
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
        
        if max_val > 0.7:  # 阈值
            h, w = template.shape[:2]
            x = max_loc[0] + w / 2
            y = max_loc[1] + h / 2
            confidence = max_val
            return x, y, confidence
        
        return None, None, 0.0
    
    def detect_by_color(self, image, lower_hsv, upper_hsv):
        """
        颜色检测（示例）
        Color detection (example)
        """
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            largest = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest)
            if M["m00"] != 0:
                x = M["m10"] / M["m00"]
                y = M["m01"] / M["m00"]
                confidence = min(cv2.contourArea(largest) / 1000.0, 1.0)
                return x, y, confidence
        
        return None, None, 0.0
    
    def detect_by_deep_learning(self, image):
        """
        深度学习检测（示例接口）
        Deep learning detection (example interface)
        
        TODO: 实现您的深度学习模型
        TODO: Implement your deep learning model
        """
        # 示例伪代码 (Example pseudo-code):
        # preprocessed = self.preprocess(image)
        # predictions = self.model(preprocessed)
        # x, y = predictions['center']
        # confidence = predictions['score']
        # return x, y, confidence
        
        return None, None, 0.0
    
    def destroy_node(self):
        """
        节点销毁时的清理工作
        Cleanup when node is destroyed
        """
        self.get_logger().info('正在关闭目标检测节点...')
        self.get_logger().info('Shutting down target detector node...')
        
        # 输出最终统计 (Output final statistics)
        if self.processed_count > 0:
            avg_time = self.total_detection_time / self.processed_count * 1000
            success_rate = (self.processed_count / (self.processed_count + self.detection_failures)) * 100
            
            self.get_logger().info('最终统计 (Final Statistics):')
            self.get_logger().info(f'  处理帧数 (Processed frames): {self.processed_count}')
            self.get_logger().info(f'  检测失败 (Detection failures): {self.detection_failures}')
            self.get_logger().info(f'  成功率 (Success rate): {success_rate:.1f}%')
            self.get_logger().info(f'  平均耗时 (Average time): {avg_time:.1f}ms')
        
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
        node = TargetDetectorNode()
        
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
