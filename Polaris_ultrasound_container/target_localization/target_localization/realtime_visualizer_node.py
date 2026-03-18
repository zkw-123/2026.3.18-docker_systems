#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Realtime Visualizer Node
实时可视化节点

功能 (Functions):
    - 订阅超声图像并叠加检测结果 (Subscribe to US images and overlay detection results)
    - 在图像上显示目标像素坐标和3D位置 (Display target pixel coords and 3D position on image)
    - 支持三种显示模式 (Support three display modes)

话题 (Topics):
    订阅 (Subscribers):
        - /us_image_raw: sensor_msgs/Image - 超声图像
        - /target_pixel: std_msgs/String - 检测到的像素坐标(JSON)
        - /target_3d_position: std_msgs/String - 目标3D位置(JSON)

参数 (Parameters):
    - display_mode: str, default='local' - 显示模式 (local / ros / none)
    - window_name: str - OpenCV窗口名称
    - publish_annotated: bool, default=False - 是否发布标注图像到ROS话题
    - font_scale: float, default=0.6 - 字体大小
    - line_thickness: int, default=2 - 线条粗细

快捷键 (Hotkeys):
    q / ESC - 退出可视化

"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import json
import numpy as np
import threading


class RealtimeVisualizerNode(Node):
    """
    实时可视化节点
    Realtime Visualizer Node

    职责 (Responsibilities):
        1. 订阅超声图像 (Subscribe to ultrasound images)
        2. 缓存最新的检测结果和3D位置 (Cache latest detection and 3D position)
        3. 将结果叠加到图像上 (Overlay results onto image)
        4. 以选定模式显示或发布 (Display or publish in selected mode)
    """

    def __init__(self):
        super().__init__('realtime_visualizer')

        # ============================================
        # 参数声明 (Parameter Declaration)
        # ============================================
        self.declare_parameter('display_mode', 'local')
        self.declare_parameter('window_name', 'Ultrasound Tracking Visualization')
        self.declare_parameter('publish_annotated', False)
        self.declare_parameter('font_scale', 0.6)
        self.declare_parameter('line_thickness', 2)

        self.display_mode = self.get_parameter('display_mode').value
        self.window_name = self.get_parameter('window_name').value
        self.publish_annotated = self.get_parameter('publish_annotated').value
        self.font_scale = self.get_parameter('font_scale').value
        self.line_thickness = self.get_parameter('line_thickness').value

        self.get_logger().info(f'显示模式 (Display mode): {self.display_mode}')

        # ============================================
        # 缓存最新结果 (Cache Latest Results)
        # ============================================
        self.latest_pixel = None    # dict from /target_pixel
        self.latest_3d = None       # dict from /target_3d_position
        self._lock = threading.Lock()

        # ============================================
        # QoS (QoS Configuration)
        # ============================================
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ============================================
        # 订阅器 (Subscribers)
        # ============================================
        self.image_sub = self.create_subscription(
            Image,
            '/us_image_raw',
            self.image_callback,
            best_effort_qos
        )
        self.pixel_sub = self.create_subscription(
            String,
            '/target_pixel',
            self.pixel_callback,
            reliable_qos
        )
        self.pos3d_sub = self.create_subscription(
            String,
            '/target_3d_position',
            self.pos3d_callback,
            reliable_qos
        )

        # ============================================
        # 发布器（可选）(Optional Publisher)
        # ============================================
        self.annotated_pub = None
        if self.publish_annotated or self.display_mode == 'ros':
            self.annotated_pub = self.create_publisher(
                Image,
                '/us_image_annotated',
                reliable_qos
            )
            self.get_logger().info('标注图像发布话题: /us_image_annotated')

        # ============================================
        # 工具 (Tools)
        # ============================================
        self.bridge = CvBridge()
        self.frame_count = 0

        self.get_logger().info('=' * 60)
        self.get_logger().info('实时可视化节点已启动 (Realtime Visualizer Node Started)')
        self.get_logger().info(f'模式 (Mode): {self.display_mode}')
        if self.display_mode == 'local':
            self.get_logger().info("按 'q' 或 ESC 退出 (Press 'q' or ESC to quit)")
        self.get_logger().info('=' * 60)

    # ------------------------------------------------
    # 回调函数 (Callbacks)
    # ------------------------------------------------

    def pixel_callback(self, msg: String):
        """缓存最新像素检测结果 (Cache latest pixel detection result)"""
        try:
            with self._lock:
                self.latest_pixel = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f'解析 /target_pixel 失败: {e}')

    def pos3d_callback(self, msg: String):
        """缓存最新3D位置 (Cache latest 3D position)"""
        try:
            with self._lock:
                self.latest_3d = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f'解析 /target_3d_position 失败: {e}')

    def image_callback(self, msg: Image):
        """
        图像回调：叠加信息并显示/发布
        Image callback: overlay info and display/publish
        """
        if self.display_mode == 'none':
            return

        # 转换图像 (Convert image)
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'图像转换失败 (Image conversion failed): {e}')
            return

        # 获取当前缓存结果 (Snapshot cached results)
        with self._lock:
            pixel_data = self.latest_pixel
            pos3d_data = self.latest_3d

        # 叠加信息 (Overlay information)
        annotated = self._draw_overlay(frame, pixel_data, pos3d_data)
        self.frame_count += 1

        # 显示或发布 (Display or publish)
        if self.display_mode == 'local':
            cv2.imshow(self.window_name, annotated)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):  # q or ESC
                self.get_logger().info('用户退出可视化 (User quit visualization)')
                cv2.destroyAllWindows()
                rclpy.shutdown()

        if self.annotated_pub is not None:
            try:
                out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
                out_msg.header = msg.header
                self.annotated_pub.publish(out_msg)
            except Exception as e:
                self.get_logger().error(f'发布标注图像失败: {e}')

    # ------------------------------------------------
    # 绘制叠加层 (Draw Overlay)
    # ------------------------------------------------

    def _draw_overlay(self, frame: np.ndarray, pixel_data, pos3d_data) -> np.ndarray:
        """
        在图像上绘制检测结果和3D位置
        Draw detection results and 3D position on image
        """
        img = frame.copy()
        h, w = img.shape[:2]
        font = cv2.FONT_HERSHEY_SIMPLEX
        fs = self.font_scale
        lw = self.line_thickness

        # ---------- 画检测点 (Draw detection point) ----------
        if pixel_data is not None:
            px = int(pixel_data.get('pixel_x', w / 2))
            py = int(pixel_data.get('pixel_y', h / 2))
            conf = pixel_data.get('confidence', 0.0)

            # 十字准星 (Crosshair)
            cross = 20
            color = (0, 255, 0)  # green
            cv2.line(img, (px - cross, py), (px + cross, py), color, lw)
            cv2.line(img, (px, py - cross), (px, py + cross), color, lw)
            cv2.circle(img, (px, py), 8, color, lw)

            # 置信度标签 (Confidence label)
            label = f'conf={conf:.2f}'
            cv2.putText(img, label, (px + 12, py - 12), font, fs, color, lw)

        # ---------- 显示3D位置 (Show 3D position) ----------
        if pos3d_data is not None:
            pos = pos3d_data.get('position', {})
            x = pos.get('x', 0.0)
            y = pos.get('y', 0.0)
            z = pos.get('z', 0.0)
            frame_no = pos3d_data.get('frame_number', 0)
            ct = pos3d_data.get('computation_time_ms', 0.0)

            lines = [
                f'3D Position (mm)',
                f'  X: {x:+8.2f}',
                f'  Y: {y:+8.2f}',
                f'  Z: {z:+8.2f}',
                f'  frame: {frame_no}  ct: {ct:.1f}ms',
            ]
            margin = 10
            line_h = int(28 * fs)
            for i, text in enumerate(lines):
                cv2.putText(img, text,
                            (margin, margin + (i + 1) * line_h),
                            font, fs, (0, 220, 255), lw)

        # ---------- 帧计数 (Frame counter) ----------
        cv2.putText(img, f'frame {self.frame_count}',
                    (w - 160, h - 10), font, fs * 0.8, (180, 180, 180), 1)

        return img

    # ------------------------------------------------
    # 清理 (Cleanup)
    # ------------------------------------------------

    def destroy_node(self):
        self.get_logger().info('正在关闭可视化节点... (Shutting down visualizer node...)')
        if self.display_mode == 'local':
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RealtimeVisualizerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n[Visualizer] Interrupted by user')
    except Exception as e:
        print(f'[Visualizer] Error: {e}')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
