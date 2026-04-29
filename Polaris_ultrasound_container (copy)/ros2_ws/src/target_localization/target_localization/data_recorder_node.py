#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Data Recorder Node (Optional)
数据录制节点（可选）

功能 (Functions):
    - 异步订阅和保存所有录制话题 (Asynchronously subscribe and save all recording topics)
    - 完全独立运行，不影响实时处理 (Runs completely independently, doesn't affect realtime processing)
    - 使用后台线程和队列缓冲 (Uses background threads and queue buffering)
    - 保存超声图像和NDI数据 (Save ultrasound images and NDI data)

话题 (Topics):
    订阅 (Subscribers):
        - /us_image_record: sensor_msgs/Image - 超声图像（录制话题）
        - /ndi_transforms_record: std_msgs/String - NDI数据（录制话题）

参数 (Parameters):
    - save_path: str - 保存路径
    - save_us_images: bool, default=True - 是否保存超声图像
    - save_ndi_data: bool, default=True - 是否保存NDI数据
    - buffer_size: int, default=1000 - 缓冲队列大小
    - num_worker_threads: int, default=4 - 工作线程数

作者 (Author): Your Name
日期 (Date): 2024
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import json
import os
import datetime
import queue
import threading
from concurrent.futures import ThreadPoolExecutor


class DataRecorderNode(Node):
    """
    数据录制节点
    Data Recorder Node
    
    职责 (Responsibilities):
        1. 低优先级订阅录制话题 (Subscribe to recording topics with low priority)
        2. 使用缓冲队列避免阻塞 (Use buffer queue to avoid blocking)
        3. 在后台线程中异步保存 (Save asynchronously in background threads)
        4. 完全独立，不影响实时流水线 (Completely independent, doesn't affect realtime pipeline)
    """
    
    def __init__(self):
        super().__init__('data_recorder')
        
        # ============================================
        # 参数声明 (Parameter Declaration)
        # ============================================
        self.declare_parameter('save_path', '/tmp/ultrasound_tracking_data')
        self.declare_parameter('save_us_images', True)
        self.declare_parameter('save_ndi_data', True)
        self.declare_parameter('buffer_size', 1000)
        self.declare_parameter('num_worker_threads', 4)
        
        # 获取参数 (Get parameters)
        self.save_path = self.get_parameter('save_path').value
        self.save_us_images = self.get_parameter('save_us_images').value
        self.save_ndi_data = self.get_parameter('save_ndi_data').value
        self.buffer_size = self.get_parameter('buffer_size').value
        self.num_worker_threads = self.get_parameter('num_worker_threads').value
        
        # ============================================
        # 创建保存目录 (Create Save Directories)
        # ============================================
        # 使用时间戳创建会话目录 (Create session directory with timestamp)
        self.session_timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.session_dir = os.path.join(self.save_path, self.session_timestamp)
        
        if self.save_us_images:
            self.us_dir = os.path.join(self.session_dir, 'ultrasound')
            os.makedirs(self.us_dir, exist_ok=True)
            self.get_logger().info(f'超声图像保存路径: {self.us_dir}')
        
        if self.save_ndi_data:
            self.ndi_dir = os.path.join(self.session_dir, 'ndi')
            os.makedirs(self.ndi_dir, exist_ok=True)
            self.get_logger().info(f'NDI数据保存路径: {self.ndi_dir}')
        
        # ============================================
        # 初始化队列和线程池 (Initialize Queues and Thread Pool)
        # ============================================
        # 使用队列缓冲数据 (Use queues to buffer data)
        self.us_queue = queue.Queue(maxsize=self.buffer_size)
        self.ndi_queue = queue.Queue(maxsize=self.buffer_size)
        
        # 使用线程池异步保存 (Use thread pool for async saving)
        self.executor = ThreadPoolExecutor(max_workers=self.num_worker_threads)
        
        # 统计信息 (Statistics)
        self.us_received = 0
        self.us_saved = 0
        self.us_dropped = 0
        self.ndi_received = 0
        self.ndi_saved = 0
        self.ndi_dropped = 0
        
        # ============================================
        # QoS配置：RELIABLE保证数据完整性 (QoS: RELIABLE for data integrity)
        # ============================================
        recording_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,  # 可靠传输
            history=HistoryPolicy.KEEP_LAST,
            depth=100,  # 大队列
            durability=DurabilityPolicy.VOLATILE
        )
        
        # ============================================
        # 创建订阅器 (Create Subscribers)
        # ============================================
        if self.save_us_images:
            self.us_subscription = self.create_subscription(
                Image,
                '/us_image_record',
                self.us_callback,
                recording_qos
            )
            self.get_logger().info('已订阅: /us_image_record')
        
        if self.save_ndi_data:
            self.ndi_subscription = self.create_subscription(
                String,
                '/ndi_transforms_record',
                self.ndi_callback,
                recording_qos
            )
            self.get_logger().info('已订阅: /ndi_transforms_record')
        
        # ============================================
        # 工具初始化 (Tool Initialization)
        # ============================================
        self.bridge = CvBridge()
        
        # ============================================
        # 启动持久化线程 (Start Persistence Threads)
        # ============================================
        # 专门的线程持续从队列取数据并保存
        # Dedicated threads continuously take data from queues and save
        if self.save_us_images:
            self.us_saver_thread = threading.Thread(
                target=self.us_saver_loop,
                daemon=True
            )
            self.us_saver_thread.start()
        
        if self.save_ndi_data:
            self.ndi_saver_thread = threading.Thread(
                target=self.ndi_saver_loop,
                daemon=True
            )
            self.ndi_saver_thread.start()
        
        # ============================================
        # 创建统计输出定时器 (Create Statistics Timer)
        # ============================================
        self.stats_timer = self.create_timer(5.0, self.print_statistics)
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('数据录制节点已启动 (Data Recorder Node Started)')
        self.get_logger().info(f'会话目录 (Session directory): {self.session_dir}')
        self.get_logger().info(f'缓冲队列大小 (Buffer size): {self.buffer_size}')
        self.get_logger().info(f'工作线程数 (Worker threads): {self.num_worker_threads}')
        self.get_logger().info('=' * 60)
    
    def us_callback(self, msg):
        """
        超声图像回调：快速放入队列
        US image callback: quickly put into queue
        """
        self.us_received += 1
        
        try:
            # 非阻塞放入队列 (Non-blocking put into queue)
            self.us_queue.put_nowait((msg, self.us_received))
        except queue.Full:
            self.us_dropped += 1
            self.get_logger().warn(
                f'超声图像队列已满，丢弃数据 (US queue full, dropping frame) '
                f'#{self.us_received}'
            )
    
    def ndi_callback(self, msg):
        """
        NDI数据回调：快速放入队列
        NDI data callback: quickly put into queue
        """
        self.ndi_received += 1
        
        try:
            # 非阻塞放入队列 (Non-blocking put into queue)
            self.ndi_queue.put_nowait((msg, self.ndi_received))
        except queue.Full:
            self.ndi_dropped += 1
            self.get_logger().warn(
                f'NDI队列已满，丢弃数据 (NDI queue full, dropping data) '
                f'#{self.ndi_received}'
            )
    
    def us_saver_loop(self):
        """
        超声图像保存循环：持续从队列取数据并保存
        US image saver loop: continuously take from queue and save
        """
        self.get_logger().info('超声图像保存线程已启动')
        self.get_logger().info('US image saver thread started')
        
        while rclpy.ok():
            try:
                # 从队列获取数据（阻塞，超时1秒）
                # Get data from queue (blocking, 1 second timeout)
                msg, frame_number = self.us_queue.get(timeout=1.0)
                
                # 提交到线程池异步保存 (Submit to thread pool for async saving)
                self.executor.submit(self._save_us_image, msg, frame_number)
                
            except queue.Empty:
                # 队列空，继续等待 (Queue empty, continue waiting)
                continue
            except Exception as e:
                self.get_logger().error(f'保存超声图像时出错: {str(e)}')
    
    def ndi_saver_loop(self):
        """
        NDI数据保存循环
        NDI data saver loop
        """
        self.get_logger().info('NDI数据保存线程已启动')
        self.get_logger().info('NDI data saver thread started')
        
        while rclpy.ok():
            try:
                msg, frame_number = self.ndi_queue.get(timeout=1.0)
                self.executor.submit(self._save_ndi_data, msg, frame_number)
                
            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f'保存NDI数据时出错: {str(e)}')
    
    def _save_us_image(self, msg, frame_number):
        """
        实际保存超声图像的方法（在线程池中执行）
        Actual method to save US image (executed in thread pool)
        """
        try:
            # 转换ROS消息为OpenCV图像 (Convert ROS message to OpenCV image)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 构建文件名 (Build filename)
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            filename = f"us_{frame_number:06d}_{timestamp:.6f}.png"
            filepath = os.path.join(self.us_dir, filename)
            
            # 保存图像 (Save image)
            cv2.imwrite(filepath, cv_image)
            
            self.us_saved += 1
            
            self.get_logger().debug(f'已保存超声图像: {filename}')
            
        except Exception as e:
            self.get_logger().error(f'保存超声图像失败 #{frame_number}: {str(e)}')
    
    def _save_ndi_data(self, msg, frame_number):
        """
        实际保存NDI数据的方法（在线程池中执行）
        Actual method to save NDI data (executed in thread pool)
        """
        try:
            # 解析JSON数据 (Parse JSON data)
            ndi_data = json.loads(msg.data)
            
            # 构建文件名 (Build filename)
            timestamp = ndi_data.get('timestamp', 0.0)
            filename = f"ndi_{frame_number:06d}_{timestamp:.6f}.json"
            filepath = os.path.join(self.ndi_dir, filename)
            
            # 保存JSON (Save JSON)
            with open(filepath, 'w') as f:
                json.dump(ndi_data, f, indent=2)
            
            self.ndi_saved += 1
            
            self.get_logger().debug(f'已保存NDI数据: {filename}')
            
        except Exception as e:
            self.get_logger().error(f'保存NDI数据失败 #{frame_number}: {str(e)}')
    
    def print_statistics(self):
        """
        定期输出统计信息
        Periodically print statistics
        """
        self.get_logger().info('=' * 60)
        self.get_logger().info('录制统计 (Recording Statistics):')
        
        if self.save_us_images:
            us_save_rate = (self.us_saved / self.us_received * 100) if self.us_received > 0 else 0
            self.get_logger().info(
                f'  超声图像 (US Images): '
                f'接收={self.us_received}, '
                f'已保存={self.us_saved}, '
                f'丢弃={self.us_dropped}, '
                f'保存率={us_save_rate:.1f}%'
            )
            self.get_logger().info(f'  队列大小 (Queue size): {self.us_queue.qsize()}/{self.buffer_size}')
        
        if self.save_ndi_data:
            ndi_save_rate = (self.ndi_saved / self.ndi_received * 100) if self.ndi_received > 0 else 0
            self.get_logger().info(
                f'  NDI数据 (NDI Data): '
                f'接收={self.ndi_received}, '
                f'已保存={self.ndi_saved}, '
                f'丢弃={self.ndi_dropped}, '
                f'保存率={ndi_save_rate:.1f}%'
            )
            self.get_logger().info(f'  队列大小 (Queue size): {self.ndi_queue.qsize()}/{self.buffer_size}')
        
        self.get_logger().info('=' * 60)
    
    def destroy_node(self):
        """
        节点销毁时的清理工作
        Cleanup when node is destroyed
        """
        self.get_logger().info('正在关闭数据录制节点...')
        self.get_logger().info('Shutting down data recorder node...')
        
        # 等待队列清空 (Wait for queues to be empty)
        self.get_logger().info('等待队列清空... (Waiting for queues to empty...)')
        
        timeout = 10.0  # 最多等待10秒
        start_time = self.get_clock().now().nanoseconds / 1e9
        
        while True:
            current_time = self.get_clock().now().nanoseconds / 1e9
            if current_time - start_time > timeout:
                self.get_logger().warn('等待超时，强制退出 (Timeout, forcing exit)')
                break
            
            us_empty = not self.save_us_images or self.us_queue.empty()
            ndi_empty = not self.save_ndi_data or self.ndi_queue.empty()
            
            if us_empty and ndi_empty:
                self.get_logger().info('队列已清空 (Queues emptied)')
                break
            
            self.get_logger().info(
                f'队列状态 (Queue status): '
                f'US={self.us_queue.qsize()}, NDI={self.ndi_queue.qsize()}'
            )
            
            import time
            time.sleep(0.5)
        
        # 关闭线程池 (Shutdown thread pool)
        self.get_logger().info('关闭线程池... (Shutting down thread pool...)')
        self.executor.shutdown(wait=True)
        
        # 输出最终统计 (Output final statistics)
        self.get_logger().info('最终统计 (Final Statistics):')
        if self.save_us_images:
            self.get_logger().info(
                f'  超声图像: 接收={self.us_received}, '
                f'已保存={self.us_saved}, 丢弃={self.us_dropped}'
            )
        if self.save_ndi_data:
            self.get_logger().info(
                f'  NDI数据: 接收={self.ndi_received}, '
                f'已保存={self.ndi_saved}, 丢弃={self.ndi_dropped}'
            )
        
        self.get_logger().info(f'数据已保存到: {self.session_dir}')
        self.get_logger().info(f'Data saved to: {self.session_dir}')
        
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
        node = DataRecorderNode()
        
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
