import json
import os
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge


class USTargetEstimatorNode(Node):
    """
    Estimate target position from ultrasound image.

    Pipeline:
    - subscribe ultrasound image
    - optionally restrict ROI using mask
    - detect bright target region
    - publish target point
    - optionally transform target to robot/world frame using a 4x4 transform json
    - publish debug image
    """

    def __init__(self):
        super().__init__('us_target_estimator_node')

        self.declare_parameter('image_topic', '/us_img')
        self.declare_parameter('target_topic', '/target_point')
        self.declare_parameter('debug_image_topic', '/us_target_debug')

        self.declare_parameter('mask_path', '')
        self.declare_parameter('publish_debug_image', True)

        self.declare_parameter('threshold_mode', 'percentile')  # percentile | fixed
        self.declare_parameter('percentile_thr', 99.0)
        self.declare_parameter('fixed_thr', 220.0)
        self.declare_parameter('min_component_area', 50)

        self.declare_parameter('pixel_spacing_x_mm', 1.0)
        self.declare_parameter('pixel_spacing_y_mm', 1.0)
        self.declare_parameter('target_depth_mm', 0.0)

        self.declare_parameter('transform_json_path', '')
        self.declare_parameter('output_frame_id', 'base_link')
        self.declare_parameter('image_frame_id', 'us_image')

        self.image_topic = self.get_parameter('image_topic').value
        self.target_topic = self.get_parameter('target_topic').value
        self.debug_image_topic = self.get_parameter('debug_image_topic').value

        self.mask_path = self.get_parameter('mask_path').value
        self.publish_debug_image = bool(self.get_parameter('publish_debug_image').value)

        self.threshold_mode = self.get_parameter('threshold_mode').value
        self.percentile_thr = float(self.get_parameter('percentile_thr').value)
        self.fixed_thr = float(self.get_parameter('fixed_thr').value)
        self.min_component_area = int(self.get_parameter('min_component_area').value)

        self.pixel_spacing_x_mm = float(self.get_parameter('pixel_spacing_x_mm').value)
        self.pixel_spacing_y_mm = float(self.get_parameter('pixel_spacing_y_mm').value)
        self.target_depth_mm = float(self.get_parameter('target_depth_mm').value)

        self.transform_json_path = self.get_parameter('transform_json_path').value
        self.output_frame_id = self.get_parameter('output_frame_id').value
        self.image_frame_id = self.get_parameter('image_frame_id').value

        self.bridge = CvBridge()
        self.target_pub = self.create_publisher(PointStamped, self.target_topic, 10)
        self.debug_pub = None
        if self.publish_debug_image:
            self.debug_pub = self.create_publisher(Image, self.debug_image_topic, 10)

        self.image_sub = self.create_subscription(Image, self.image_topic, self.image_callback, 10)

        self.mask = None
        self.T_out_from_img = None

        self._load_mask()
        self._load_transform()

        self.get_logger().info(
            'USTargetEstimatorNode started\n'
            f'  image_topic          : {self.image_topic}\n'
            f'  target_topic         : {self.target_topic}\n'
            f'  debug_image_topic    : {self.debug_image_topic}\n'
            f'  mask_path            : {self.mask_path}\n'
            f'  threshold_mode       : {self.threshold_mode}\n'
            f'  percentile_thr       : {self.percentile_thr}\n'
            f'  fixed_thr            : {self.fixed_thr}\n'
            f'  min_component_area   : {self.min_component_area}\n'
            f'  transform_json_path  : {self.transform_json_path}'
        )

    def _load_mask(self):
        if not self.mask_path:
            return

        if not os.path.isfile(self.mask_path):
            self.get_logger().warn(f'Mask file does not exist: {self.mask_path}')
            return

        mask_img = cv2.imread(self.mask_path, cv2.IMREAD_GRAYSCALE)
        if mask_img is None:
            self.get_logger().warn(f'Failed to load mask image: {self.mask_path}')
            return

        self.mask = (mask_img > 0).astype(np.uint8)
        self.get_logger().info(f'Loaded mask: {self.mask_path}, shape={self.mask.shape}')

    def _load_transform(self):
        if not self.transform_json_path:
            self.get_logger().warn('transform_json_path is empty. Publishing in image frame.')
            return

        if not os.path.isfile(self.transform_json_path):
            self.get_logger().warn(
                f'transform_json_path does not exist: {self.transform_json_path}. '
                'Publishing in image frame.'
            )
            return

        try:
            with open(self.transform_json_path, 'r') as f:
                data = json.load(f)
        except Exception as e:
            self.get_logger().warn(f'Failed to read transform json: {e}')
            return

        # Accept either a direct 4x4 matrix or a dict field named "matrix"
        try:
            if isinstance(data, dict) and 'matrix' in data:
                T = np.array(data['matrix'], dtype=float)
            else:
                T = np.array(data, dtype=float)

            if T.shape != (4, 4):
                raise ValueError(f'Expected 4x4 matrix, got {T.shape}')

            self.T_out_from_img = T
            self.get_logger().info('Loaded 4x4 transform matrix successfully.')
        except Exception as e:
            self.get_logger().warn(f'Invalid transform json format: {e}')

    def image_callback(self, msg: Image):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'Failed to convert image: {e}')
            return

        target_px, debug_img = self._detect_target_pixel(bgr)

        if target_px is None:
            if self.publish_debug_image and debug_img is not None:
                self._publish_debug(debug_img, msg.header.stamp)
            return

        u, v = target_px

        point_img_mm = np.array([
            u * self.pixel_spacing_x_mm,
            v * self.pixel_spacing_y_mm,
            self.target_depth_mm,
            1.0
        ], dtype=float)

        point_msg = PointStamped()
        point_msg.header.stamp = msg.header.stamp

        if self.T_out_from_img is not None:
            point_out_mm = self.T_out_from_img @ point_img_mm
            point_msg.header.frame_id = self.output_frame_id
            point_msg.point.x = float(point_out_mm[0] * 0.001)
            point_msg.point.y = float(point_out_mm[1] * 0.001)
            point_msg.point.z = float(point_out_mm[2] * 0.001)
        else:
            point_msg.header.frame_id = self.image_frame_id
            point_msg.point.x = float(point_img_mm[0] * 0.001)
            point_msg.point.y = float(point_img_mm[1] * 0.001)
            point_msg.point.z = float(point_img_mm[2] * 0.001)

        self.target_pub.publish(point_msg)

        if self.publish_debug_image and debug_img is not None:
            cv2.circle(debug_img, (u, v), 6, (0, 255, 0), -1)
            self._publish_debug(debug_img, msg.header.stamp)

    def _detect_target_pixel(self, bgr: np.ndarray) -> Tuple[Optional[Tuple[int, int]], np.ndarray]:
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        debug = bgr.copy()

        h, w = gray.shape
        roi_mask = np.ones((h, w), dtype=np.uint8)

        if self.mask is not None:
            if self.mask.shape == gray.shape:
                roi_mask = self.mask.copy()
            else:
                self.get_logger().warn(
                    f'Mask shape {self.mask.shape} does not match image shape {gray.shape}. Ignoring mask.'
                )

        masked_gray = gray.copy()
        masked_gray[roi_mask == 0] = 0

        valid_pixels = masked_gray[roi_mask > 0]
        if valid_pixels.size == 0:
            self.get_logger().warn('Empty ROI in target estimator.')
            return None, debug

        if self.threshold_mode == 'fixed':
            thr = self.fixed_thr
        else:
            thr = np.percentile(valid_pixels, self.percentile_thr)

        _, binary = cv2.threshold(masked_gray, float(thr), 255, cv2.THRESH_BINARY)
        binary = binary.astype(np.uint8)

        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(binary, connectivity=8)

        best_label = None
        best_area = 0
        for i in range(1, num_labels):
            area = int(stats[i, cv2.CC_STAT_AREA])
            if area >= self.min_component_area and area > best_area:
                best_area = area
                best_label = i

        if best_label is None:
            return None, debug

        cx, cy = centroids[best_label]
        u, v = int(round(cx)), int(round(cy))

        component_mask = (labels == best_label).astype(np.uint8) * 255
        contours, _ = cv2.findContours(component_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(debug, contours, -1, (0, 0, 255), 2)

        return (u, v), debug

    def _publish_debug(self, debug_img: np.ndarray, stamp):
        if self.debug_pub is None:
            return

        msg = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
        msg.header.stamp = stamp
        msg.header.frame_id = self.image_frame_id
        self.debug_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = USTargetEstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
