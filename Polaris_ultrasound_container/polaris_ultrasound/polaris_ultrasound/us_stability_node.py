import os

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge


class USStabilityNode(Node):
    """
    Compute ultrasound stability index Sk from input image.

    Features:
    - subscribes to ultrasound image
    - computes intensity variation (C_sp) and gradient magnitude (C_grad) inside ROI
    - normalizes and fuses them into Sk in [0, 1]
    - applies exponential smoothing
    - publishes smoothed stability
    """

    def __init__(self):
        super().__init__('us_stability_node')

        self.declare_parameter('image_topic', '/us_img')
        self.declare_parameter('stability_topic', '/us_stability')
        self.declare_parameter('mask_path', '')
        self.declare_parameter('alpha', 0.2)
        self.declare_parameter('c_sp_max', 17.6)
        self.declare_parameter('c_grad_max', 7.1)
        self.declare_parameter('default_roi_mode', 'center_box')   # center_box | full_image
        self.declare_parameter('debug_log_every', 30)

        self.image_topic = self.get_parameter('image_topic').value
        self.stability_topic = self.get_parameter('stability_topic').value
        self.mask_path = self.get_parameter('mask_path').value
        self.alpha = float(self.get_parameter('alpha').value)
        self.c_sp_max = float(self.get_parameter('c_sp_max').value)
        self.c_grad_max = float(self.get_parameter('c_grad_max').value)
        self.default_roi_mode = self.get_parameter('default_roi_mode').value
        self.debug_log_every = int(self.get_parameter('debug_log_every').value)

        if not (0.0 < self.alpha <= 1.0):
            raise RuntimeError('alpha must be in (0, 1].')
        if self.c_sp_max <= 0.0 or self.c_grad_max <= 0.0:
            raise RuntimeError('c_sp_max and c_grad_max must be > 0.')

        self.bridge = CvBridge()
        self.stability_pub = self.create_publisher(Float32, self.stability_topic, 10)
        self.image_sub = self.create_subscription(Image, self.image_topic, self.image_callback, 10)

        self.mask = None
        self.s_bar_prev = None
        self.frame_count = 0

        self._load_mask()

        self.get_logger().info(
            'USStabilityNode started\n'
            f'  image_topic      : {self.image_topic}\n'
            f'  stability_topic  : {self.stability_topic}\n'
            f'  mask_path        : {self.mask_path}\n'
            f'  alpha            : {self.alpha}\n'
            f'  c_sp_max         : {self.c_sp_max}\n'
            f'  c_grad_max       : {self.c_grad_max}\n'
            f'  default_roi_mode : {self.default_roi_mode}'
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
        self.get_logger().info(f'Loaded ROI mask: {self.mask_path}, shape={self.mask.shape}')

    def image_callback(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'Failed to convert image: {e}')
            return

        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape
        if h < 20 or w < 20:
            self.get_logger().warn('Image too small for stability computation.')
            return

        roi_mask = self._build_roi_mask(gray.shape)
        if roi_mask is None or roi_mask.sum() == 0:
            self.get_logger().warn('ROI is empty. Skipping stability computation.')
            return

        roi_pixels = gray[roi_mask > 0]
        c_sp = float(np.std(roi_pixels))

        grad_x = cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=3)
        grad_y = cv2.Sobel(gray, cv2.CV_32F, 0, 1, ksize=3)
        grad_mag = np.sqrt(grad_x**2 + grad_y**2)
        c_grad = float(np.mean(grad_mag[roi_mask > 0]))

        c_sp_norm = np.clip(c_sp / self.c_sp_max, 0.0, 1.0)
        c_grad_norm = np.clip(c_grad / self.c_grad_max, 0.0, 1.0)
        s_raw = 0.5 * (c_sp_norm + c_grad_norm)

        if self.s_bar_prev is None:
            s_bar = s_raw
        else:
            s_bar = self.alpha * s_raw + (1.0 - self.alpha) * self.s_bar_prev
        self.s_bar_prev = s_bar

        out = Float32()
        out.data = float(s_bar)
        self.stability_pub.publish(out)

        self.frame_count += 1
        if self.debug_log_every > 0 and self.frame_count % self.debug_log_every == 0:
            self.get_logger().info(
                f'Sk={s_bar:.4f}, raw={s_raw:.4f}, C_sp={c_sp:.3f}, C_grad={c_grad:.3f}'
            )

    def _build_roi_mask(self, image_shape):
        h, w = image_shape

        if self.mask is not None:
            if self.mask.shape != (h, w):
                self.get_logger().warn(
                    f'Mask shape {self.mask.shape} does not match image shape {(h, w)}. '
                    'Falling back to default ROI.'
                )
            else:
                return self.mask.copy()

        if self.default_roi_mode == 'full_image':
            return np.ones((h, w), dtype=np.uint8)

        # default: center_box
        roi = np.zeros((h, w), dtype=np.uint8)
        y0, y1 = h // 3, 2 * h // 3
        x0, x1 = w // 3, 2 * w // 3
        roi[y0:y1, x0:x1] = 1
        return roi


def main(args=None):
    rclpy.init(args=args)
    node = USStabilityNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
