#!/usr/bin/env python3
# coding: UTF-8

"""
Polaris Publisher Node

功能：
1. 从NDI Polaris光学追踪系统读取数据
2. 添加精确的ROS2时间戳
3. 记录NDI硬件时间戳（用于时间对齐分析）
4. 双发布策略：
   - 实时话题：BEST_EFFORT (用于实时处理)
   - 录制话题：RELIABLE (用于数据保存)

"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
import json
import numpy as np
import threading
from sksurgerynditracker.nditracker import NDITracker


class PolarisPublisherNode(Node):
    """NDI Polaris追踪数据采集和发布节点"""
    
    def __init__(self):
        super().__init__('polaris_publisher')
        
        # ==================== 参数声明 ====================
        self.declare_parameter('rom_paths', '')
        self.declare_parameter('tool_names', '')
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('enable_recording', False)
        
        # 获取参数
        rom_files_str = self.get_parameter('rom_paths').value
        tool_names_str = self.get_parameter('tool_names').value
        self.serial_port = self.get_parameter('serial_port').value
        self.enable_recording = self.get_parameter('enable_recording').value
        
        # 解析ROM文件和工具名称（逗号分隔）
        self.rom_files = [f.strip() for f in rom_files_str.split(',') if f.strip()]
        self.tool_names = [n.strip() for n in tool_names_str.split(',') if n.strip()]
        
        # 验证参数
        if len(self.rom_files) == 0:
            self.get_logger().error("No ROM files provided!")
            raise ValueError("rom_paths parameter is required")
        
        # 自动填充工具名称（如果不足）
        while len(self.tool_names) < len(self.rom_files):
            self.tool_names.append(f"tool{len(self.tool_names)}")
        
        self.get_logger().info(f"ROM files: {self.rom_files}")
        self.get_logger().info(f"Tool names: {self.tool_names}")
        
        # ==================== 初始化NDI追踪器 ====================
        self.tracker_settings = {
            "tracker type": "polaris",
            "romfiles": self.rom_files,
            "serial port": self.serial_port
        }
        
        try:
            self.tracker = NDITracker(self.tracker_settings)
            self.get_logger().info(f"NDI Tracker initialized on {self.serial_port}")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize NDI tracker: {str(e)}")
            raise
        
        # ==================== QoS配置 ====================
        # 实时QoS：最低延迟
        realtime_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # 录制QoS：可靠传输
        record_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=100
        )
        
        # ==================== 创建发布器 ====================
        # 实时话题
        self.realtime_publisher = self.create_publisher(
            String,
            'ndi_transforms',
            realtime_qos
        )
        
        # 录制话题（可选）
        if self.enable_recording:
            self.record_publisher = self.create_publisher(
                String,
                'ndi_transforms_record',
                record_qos
            )
            self.get_logger().info("Recording topic enabled: ndi_transforms_record")
        else:
            self.record_publisher = None
            self.get_logger().info("Recording disabled")
        
        # ==================== 统计信息 ====================
        self.frame_count = 0
        self.start_time = self.get_clock().now()
        self.tracking_started = False
        
        # ==================== 启动追踪线程 ====================
        self.tracking_thread = threading.Thread(target=self.tracking_loop, daemon=True)
        self.running = True
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("Polaris Publisher Node initialized")
        self.get_logger().info(f"Serial port: {self.serial_port}")
        self.get_logger().info(f"Number of tools: {len(self.rom_files)}")
        self.get_logger().info(f"Publishing to: ndi_transforms")
        self.get_logger().info("=" * 60)
        
        # 启动追踪
        self.tracking_thread.start()
    
    def tracking_loop(self):
        """追踪数据采集循环（独立线程）"""
        
        try:
            # 启动追踪
            self.tracker.start_tracking()
            self.tracking_started = True
            self.get_logger().info("NDI tracking started")
            
            while rclpy.ok() and self.running:
                # ==================== 1. 读取NDI数据 ====================
                try:
                    port_handles, timestamps, framenumbers, trackings, qualities = self.tracker.get_frame()
                except Exception as e:
                    self.get_logger().error(f"Failed to get frame: {str(e)}")
                    continue
                
                # ==================== 2. 立即打ROS时间戳 ====================
                ros_timestamp = self.get_clock().now()
                
                # ==================== 3. 构建消息数据 ====================
                transforms_list = []
                
                for i in range(len(trackings)):
                    # 提取工具信息
                    tool_name = self.tool_names[i] if i < len(self.tool_names) else f"tool{i}"
                    tracking_matrix = trackings[i]
                    quality = qualities[i] if i < len(qualities) else 0.0
                    
                    # 转换为标准Python类型
                    if isinstance(tracking_matrix, np.ndarray):
                        matrix = tracking_matrix.tolist()
                    else:
                        matrix = tracking_matrix
                    
                    # 提取平移向量（如果是4x4矩阵）
                    translation = None
                    if isinstance(tracking_matrix, np.ndarray) and tracking_matrix.shape == (4, 4):
                        translation = tracking_matrix[:3, 3].tolist()
                    
                    # 构建transform信息
                    transform_info = {
                        "tool_id": int(port_handles[i]) if i < len(port_handles) else i,
                        "tool_name": tool_name,
                        "quality": float(quality),
                        "matrix": matrix
                    }
                    
                    if translation is not None:
                        transform_info["translation"] = translation
                    
                    transforms_list.append(transform_info)
                
                # ==================== 4. 构建完整消息 ====================
                data = {
                    "timestamp": ros_timestamp.nanoseconds / 1e9,  # ROS时间戳（秒）
                    "hw_timestamp": str(timestamps[0]) if timestamps else "",  # NDI硬件时间戳
                    "frame_number": self.frame_count,
                    "transforms": transforms_list
                }
                
                # ==================== 5. 发布消息 ====================
                msg = String()
                msg.data = json.dumps(data)
                
                # 发布到实时话题
                self.realtime_publisher.publish(msg)
                
                # 发布到录制话题（可选）
                if self.record_publisher is not None:
                    self.record_publisher.publish(msg)
                
                # ==================== 6. 更新统计 ====================
                self.frame_count += 1
                
                # 每60帧输出一次统计（约1秒，假设60Hz）
                if self.frame_count % 60 == 0:
                    self.log_statistics()
                
        except Exception as e:
            self.get_logger().error(f"Error in tracking loop: {str(e)}")
        
        finally:
            # 停止追踪
            try:
                if self.tracking_started:
                    self.tracker.stop_tracking()
                    self.tracker.close()
                    self.get_logger().info("NDI tracking stopped")
            except Exception as e:
                self.get_logger().error(f"Error stopping tracker: {str(e)}")
    
    def log_statistics(self):
        """输出统计信息"""
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        actual_fps = self.frame_count / elapsed_time if elapsed_time > 0 else 0
        
        self.get_logger().info(
            f"[NDI Stats] Frames: {self.frame_count}, "
            f"Actual FPS: {actual_fps:.1f}, "
            f"Tracking: {'Active' if self.tracking_started else 'Inactive'}"
        )
    
    def destroy_node(self):
        """清理资源"""
        self.get_logger().info("Shutting down Polaris Publisher Node...")
        
        # 停止追踪线程
        self.running = False
        
        # 等待线程结束
        if self.tracking_thread.is_alive():
            self.tracking_thread.join(timeout=2.0)
        
        # 最终统计
        self.log_statistics()
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = PolarisPublisherNode()
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\n[Polaris Publisher] Interrupted by user")
        
    except Exception as e:
        print(f"[Polaris Publisher] Error: {str(e)}")
        import traceback
        traceback.print_exc()
        
    finally:
        if node is not None:
            node.destroy_node()
        
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
