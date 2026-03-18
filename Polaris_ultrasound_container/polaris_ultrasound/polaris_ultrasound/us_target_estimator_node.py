#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import json
from typing import List, Tuple, Optional

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile

def fit_circle_least_squares(points: np.ndarray) -> Optional[Tuple[float, float, float]]:
    """
    对一组 2D 点做最小二乘圆拟合。
    输入 points: (N,2) 数组，每行 [x, y]
    返回: (x0, y0, R) 或 None 如果点数太少/拟合失败.
    """
    if points is None or len(points) < 3:
        return None

    pts = np.asarray(points, dtype=np.float64)
    x = pts[:, 0]
    y = pts[:, 1]

    # 解线性最小二乘: A p = b
    # x^2 + y^2 + a x + b y + c = 0
    A = np.stack([2.0 * x, 2.0 * y, np.ones_like(x)], axis=1)
    b = x * x + y * y

    try:
        params, *_ = np.linalg.lstsq(A, b, rcond=None)
    except np.linalg.LinAlgError:
        return None

    x0, y0, c = params
    r_sq = x0 * x0 + y0 * y0 + c
    if r_sq <= 0:
        return None

    R = float(math.sqrt(r_sq))
    return float(x0), float(y0), R


class USTargetEstimatorNode(Node):
    """
    从超声图像中，在给定 mask 内提取底部亮弓形的外壳弧线，
    对弧线点做圆拟合，输出【机械臂 base_link 坐标系】下目标点的 3D 位置（单位：米）。
    """

    def __init__(self):
        super().__init__('us_target_estimator_node')

        # ===========================
        # ROS 参数
        # ===========================
        self.declare_parameter('image_topic', '/us_img')
        self.declare_parameter('target_topic', '/us_target_point')
        self.declare_parameter('debug_image_topic', '/us_target_debug')
        self.declare_parameter('mask_path', '')
        self.declare_parameter('percentile_thr', 97.0)       # 阈值百分位
        self.declare_parameter('min_component_area', 200)    # 最小连通域面积
        self.declare_parameter('min_arc_points', 20)         # 弧线点至少数量
        self.declare_parameter('transform_json_path', '')    # 关键：图像→机械臂变换 JSON

        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.target_topic = self.get_parameter('target_topic').get_parameter_value().string_value
        self.debug_image_topic = self.get_parameter('debug_image_topic').get_parameter_value().string_value
        self.mask_path = self.get_parameter('mask_path').get_parameter_value().string_value
        self.percentile_thr = float(self.get_parameter('percentile_thr').get_parameter_value().double_value)
        self.min_component_area = int(self.get_parameter('min_component_area').get_parameter_value().integer_value)
        self.min_arc_points = int(self.get_parameter('min_arc_points').get_parameter_value().integer_value)
        self.transform_json_path = self.get_parameter('transform_json_path').get_parameter_value().string_value

        if not self.mask_path:
            raise RuntimeError('Parameter "mask_path" must be set (path to mask image).')
        if not self.transform_json_path:
            raise RuntimeError('Parameter "transform_json_path" must be set (path to JSON).')

        # ===========================
        # 读入 mask, 计算 ROI
        # ===========================
        mask_gray = cv2.imread(self.mask_path, cv2.IMREAD_GRAYSCALE)
        if mask_gray is None:
            raise RuntimeError(f'Failed to load mask image: {self.mask_path}')

        _, self.mask_bin = cv2.threshold(mask_gray, 127, 255, cv2.THRESH_BINARY)
        self.mask_bin = self.mask_bin.astype(np.uint8)

        nz = cv2.findNonZero(self.mask_bin)
        if nz is None:
            raise RuntimeError('Mask image has no non-zero region.')

        x, y, w, h = cv2.boundingRect(nz)
        self.roi_rect = (x, y, w, h)

        self.get_logger().info(
            f'Loaded mask from {self.mask_path}, '
            f'roi_rect = (x={x}, y={y}, w={w}, h={h})'
        )

        # ===========================
        # 读入图像→机械臂变换 JSON
        # ===========================
        try:
            with open(self.transform_json_path, 'r') as f:
                tf_data = json.load(f)
        except Exception as e:
            raise RuntimeError(f'Failed to load transform json: {self.transform_json_path}, error: {e}')

        try:
            T_probe_from_image = np.array(
                tf_data['transforms']['T_probe_from_image']['matrix'],
                dtype=np.float64
            )
            T_arm_from_polaris = np.array(
                tf_data['transforms']['T_arm_from_polaris']['matrix'],
                dtype=np.float64
            )
        except KeyError as e:
            raise RuntimeError(f'Missing key in transform json: {e}')

        if T_probe_from_image.shape != (4, 4) or T_arm_from_polaris.shape != (4, 4):
            raise RuntimeError('Transform matrices must be 4x4.')

        # 合成 image → arm_base 的整体变换
        # p_arm = T_arm_from_polaris * T_probe_from_image * p_img
        self.T_arm_from_image = T_arm_from_polaris @ T_probe_from_image

        self.get_logger().info(
            'Loaded transforms from JSON.\n'
            f'  units = meter (as in json)\n'
            f'  T_arm_from_image = T_arm_from_polaris @ T_probe_from_image'
        )

        # ===========================
        # ROS 通信
        # ===========================
        self.bridge = CvBridge()

        self.target_pub = self.create_publisher(PointStamped, self.target_topic, 10)
        self.debug_pub = self.create_publisher(Image, self.debug_image_topic, 10)

        qos_profile = QoSProfile(depth=10)
        self.sub = self.create_subscription(
           Image,
           self.image_topic,
           self.image_callback,
           qos_profile
        )

        self.get_logger().info(
            'USTargetEstimatorNode started.\n'
            f'  image_topic         = {self.image_topic}\n'
            f'  target_topic        = {self.target_topic}\n'
            f'  debug_image_topic   = {self.debug_image_topic}\n'
            f'  mask_path           = {self.mask_path}\n'
            f'  transform_json_path = {self.transform_json_path}\n'
            f'  percentile_thr      = {self.percentile_thr}\n'
            f'  min_component_area  = {self.min_component_area}\n'
            f'  min_arc_points      = {self.min_arc_points}'
        )

    # ===========================
    # 图像回调
    # ===========================
    def image_callback(self, msg: Image):
        # 尽量保持原始编码（可能是 bgr8 或 mono8）
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().warn(f'Failed to convert image: {e}')
            return

        # 转灰度
        if img.ndim == 2:
            gray = img
        else:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        if gray.shape[:2] != self.mask_bin.shape[:2]:
            self.get_logger().warn(
                f'Image size {gray.shape[:2]} != mask size {self.mask_bin.shape[:2]}, '
                'skip this frame.'
            )
            return

        # ---------- Step 0: 应用 mask & 裁剪 ROI ----------
        masked = gray.copy()
        masked[self.mask_bin == 0] = 0

        x0, y0, w, h = self.roi_rect
        roi = masked[y0:y0 + h, x0:x0 + w]
        mask_roi = self.mask_bin[y0:y0 + h, x0:x0 + w]

        # ---------- Step 1: 预处理 ----------
        roi_blur = cv2.GaussianBlur(roi, (5, 5), 1.0)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        roi_enh = clahe.apply(roi_blur)

        # ---------- Step 2: 高百分位阈值分割亮弓形 ----------
        vals = roi_enh[mask_roi > 0]
        if vals.size == 0:
            self.get_logger().debug('ROI mask has no non-zero pixels, skip frame.')
            return

        thr = np.percentile(vals, self.percentile_thr)
        _, binary = cv2.threshold(roi_enh, thr, 255, cv2.THRESH_BINARY)
        binary[mask_roi == 0] = 0

        kernel = np.ones((5, 5), np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=1)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel, iterations=1)

        # ---------- Step 3: 连通域，选出底部最大亮块 ----------
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
            binary, connectivity=8
        )
        h_roi, w_roi = roi.shape

        best_label = -1
        best_score = -1.0

        for lab in range(1, num_labels):
            area = stats[lab, cv2.CC_STAT_AREA]
            if area < self.min_component_area:
                continue

            cx, cy = centroids[lab]
            # 简单打分：面积 + 垂直位置（越靠下权重越高）
            score = float(area) + 0.5 * max(cy - h_roi / 2.0, 0.0)
            if score > best_score:
                best_score = score
                best_label = lab

        if best_label < 0:
            # 没有找到合适的亮区域
            self.get_logger().debug('No valid bright component found in this frame.')
            return

        target_mask = np.zeros_like(binary, dtype=np.uint8)
        target_mask[labels == best_label] = 255

        # ---------- Step 4: 沿列扫描，提取外壳弧线点 ----------
        arc_points: List[Tuple[float, float]] = []
        for col in range(w_roi):
            rows = np.where(target_mask[:, col] > 0)[0]
            if rows.size == 0:
                continue
            y_top = int(rows.min())  # 最靠近探头的点（图像上是“上边”）
            arc_points.append((float(col), float(y_top)))

        if len(arc_points) < self.min_arc_points:
            self.get_logger().debug(
                f'Not enough arc points ({len(arc_points)} < {self.min_arc_points}), skip.'
            )
            return

        arc_pts_np = np.array(arc_points, dtype=np.float64)
        circle = fit_circle_least_squares(arc_pts_np)
        if circle is None:
            self.get_logger().debug('Circle fitting failed, skip frame.')
            return

        cx_roi, cy_roi, R = circle
        cx_abs = cx_roi + x0
        cy_abs = cy_roi + y0

        # ---------- Step 5: 图像坐标 → 机械臂 base_link 坐标（米） ----------
        # 假设标定矩阵已将像素缩放等全部编码进 T_arm_from_image
        # p_img = [u, v, 1, 1]^T
        p_img = np.array([cx_abs, cy_abs, 1.0, 1.0], dtype=np.float64)
        p_arm = self.T_arm_from_image @ p_img
        if abs(p_arm[3]) > 1e-9:
            p_arm = p_arm / p_arm[3]

        x_arm, y_arm, z_arm = float(p_arm[0]), float(p_arm[1]), float(p_arm[2])

        # ---------- Step 6: 发布目标点（机械臂坐标系，单位米） ----------
        pt_msg = PointStamped()
        pt_msg.header = msg.header           # 继承时间戳
        pt_msg.header.frame_id = 'robot_base_link'  # 明确坐标系（可改成你的实际名称）

        pt_msg.point.x = x_arm
        pt_msg.point.y = y_arm
        pt_msg.point.z = z_arm
        self.target_pub.publish(pt_msg)

        # ---------- Step 7: 生成调试图像（仍在图像上画圆心和拟合圆） ----------
        if img.ndim == 2:
            debug_img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        else:
            debug_img = img.copy()

        # 画弧线点（绿色）
        for px, py in arc_points:
            cv2.circle(
                debug_img,
                (int(px + x0), int(py + y0)),
                1,
                (0, 255, 0),
                -1,
            )

        # 画拟合圆和圆心（像素坐标系）
        cv2.circle(
            debug_img,
            (int(round(cx_abs)), int(round(cy_abs))),
            5,
            (0, 0, 255),
            -1,
        )
        cv2.circle(
            debug_img,
            (int(round(cx_abs)), int(round(cy_abs))),
            int(round(R)),
            (255, 0, 0),
            2,
        )

        debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
        debug_msg.header = msg.header
        self.debug_pub.publish(debug_msg)

        self.get_logger().debug(
            f'Target detected: '
            f'pixel=({cx_abs:.1f}, {cy_abs:.1f}), R={R:.1f}, '
            f'arm=({x_arm:.4f}, {y_arm:.4f}, {z_arm:.4f}) m, '
            f'arc_points={len(arc_points)}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = USTargetEstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
