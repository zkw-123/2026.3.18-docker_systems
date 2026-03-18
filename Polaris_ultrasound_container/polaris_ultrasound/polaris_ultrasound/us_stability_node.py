import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Float32

from cv_bridge import CvBridge
import cv2
import numpy as np


class USStabilityNode(Node):
    """
    从超声图像计算稳定度 Sk 的节点。

    功能：
      - 订阅 /us_img (sensor_msgs/Image)
      - 在指定 ROI 内计算：
          Csp  : 灰度标准差（对比度）
          Cgrad: 梯度幅值平均值（纹理/边缘强度）
      - 将 Csp / Cgrad 归一化并合成 Sk ∈ [0,1]
      - 对 Sk 做指数平滑，输出 S_bar
      - 发布 /us_stability (std_msgs/Float32)

    ROI 支持两种模式：
      1) 使用 mask 图片（mask_path 参数）：
         - mask 与图像同尺寸
         - mask > 0 的区域为有效 ROI（任意形状）
      2) 无 mask 时，使用图像中心 1/3 矩形作为 ROI
    """

    def __init__(self):
        super().__init__('us_stability_node')

        # ---------------- 参数定义 ----------------
        # 图像输入话题
        self.declare_parameter('image_topic', '/us_img')
        # 稳定度输出话题
        self.declare_parameter('stability_topic', '/us_stability')
        # ROI mask 路径（可选）
        self.declare_parameter('mask_path', '')
        # 指数平滑系数 alpha
        self.declare_parameter('alpha', 0.2)
        # Csp / Cgrad 归一化用的上限（根据离线统计调整）
        # 原默认值为 60 / 1000，这里改为 17.6 / 7.1
        self.declare_parameter('c_sp_max', 17.6)
        self.declare_parameter('c_grad_max', 7.1)

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        stability_topic = self.get_parameter('stability_topic').get_parameter_value().string_value
        mask_path = self.get_parameter('mask_path').get_parameter_value().string_value
        self.alpha = float(self.get_parameter('alpha').get_parameter_value().double_value)
        self.c_sp_max = float(self.get_parameter('c_sp_max').get_parameter_value().double_value)
        self.c_grad_max = float(self.get_parameter('c_grad_max').get_parameter_value().double_value)

        # ---------------- ROS 通信部分 ----------------
        self.bridge = CvBridge()
        self.stability_pub = self.create_publisher(Float32, stability_topic, 10)
        self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, 10)

        # Sk 平滑历史值
        self.s_bar_prev = None
        # 帧计数，用于控制打印调试信息
        self.frame_count = 0

        # ---------------- 加载 ROI mask（如果有） ----------------
        self.mask = None  # uint8, 0/1
        if mask_path:
            mask_img = cv2.imread(mask_path, cv2.IMREAD_GRAYSCALE)
            if mask_img is None:
                self.get_logger().warn(f'Failed to load mask image from: {mask_path}')
            else:
                # 转为 0/1，后面用 bool 索引
                self.mask = (mask_img > 0).astype(np.uint8)
                self.get_logger().info(
                    f'Loaded ROI mask from {mask_path}, shape={self.mask.shape}'
                )

        self.get_logger().info(
            f'USStabilityNode started.\n'
            f'  image_topic    = {image_topic}\n'
            f'  stability_topic= {stability_topic}\n'
            f'  mask_path      = {mask_path}\n'
            f'  alpha          = {self.alpha}\n'
            f'  c_sp_max       = {self.c_sp_max}\n'
            f'  c_grad_max     = {self.c_grad_max}'
        )

    # --------------------------------------------------
    # 图像回调：核心计算逻辑
    # --------------------------------------------------
    def image_callback(self, msg: Image):
        # 1) ROS Image -> OpenCV BGR
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'Failed to convert image: {e}')
            return

        # 2) 转灰度
        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape
        if h < 20 or w < 20:
            self.get_logger().warn('Image too small for stability computation.')
            return

        # 3) 选择 ROI 像素（mask 模式 or 中心矩形）
        use_mask = self.mask is not None

        if use_mask:
            if self.mask.shape != gray.shape:
                self.get_logger().warn(
                    f'Mask shape {self.mask.shape} does not match image shape {gray.shape}. '
                    f'Please regenerate mask or check image resolution.'
                )
                return

            roi_pixels = gray[self.mask == 1]
            if roi_pixels.size < 50:
                self.get_logger().warn(
                    f'Too few pixels in ROI mask (n={roi_pixels.size}).'
                )
                return

            # 对整个图做梯度，再按 mask 取 ROI
            grad_x = cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=3)
            grad_y = cv2.Sobel(gray, cv2.CV_32F, 0, 1, ksize=3)
            grad_mag = cv2.magnitude(grad_x, grad_y)
            grad_roi = grad_mag[self.mask == 1]

            c_sp = float(np.std(roi_pixels))
            c_grad = float(np.mean(grad_roi))

        else:
            # 没有 mask 时：退回中心 1/3 矩形 ROI
            roi_h = h // 3
            roi_w = w // 3
            y0 = (h - roi_h) // 2
            x0 = (w - roi_w) // 2
            roi = gray[y0:y0 + roi_h, x0:x0 + roi_w]

            c_sp = float(np.std(roi))

            grad_x = cv2.Sobel(roi, cv2.CV_32F, 1, 0, ksize=3)
            grad_y = cv2.Sobel(roi, cv2.CV_32F, 0, 1, ksize=3)
            grad_mag = cv2.magnitude(grad_x, grad_y)
            c_grad = float(np.mean(grad_mag))

        # 4) 归一化 & 合成 Sk
        #    使用离线统计得到的 c_sp_max / c_grad_max 将两项映射到 [0,1]
        c_sp_norm = np.clip(c_sp / max(self.c_sp_max, 1e-6), 0.0, 1.0)
        c_grad_norm = np.clip(c_grad / max(self.c_grad_max, 1e-6), 0.0, 1.0)

        # 当前先用 0.5 / 0.5 权重，可之后改为参数化
        sk = 0.5 * c_sp_norm + 0.5 * c_grad_norm  # [0,1]

        # 5) 指数平滑 S_bar
        if self.s_bar_prev is None:
            s_bar = sk
        else:
            a = np.clip(self.alpha, 0.0, 1.0)
            s_bar = a * sk + (1.0 - a) * self.s_bar_prev
        self.s_bar_prev = s_bar

        # 6) 发布结果
        out_msg = Float32()
        out_msg.data = float(s_bar)
        self.stability_pub.publish(out_msg)

        # 7) 调试信息：只在前 50 帧打印，避免刷屏
        self.frame_count += 1
        if self.frame_count <= 50:
            self.get_logger().info(
                f'frame={self.frame_count}, '
                f'Csp={c_sp:.3f}, Csp_norm={c_sp_norm:.3f}, '
                f'Cgrad={c_grad:.3f}, Cgrad_norm={c_grad_norm:.3f}, '
                f'Sk_raw={sk:.3f}, S_bar={s_bar:.3f}, '
                f'use_mask={use_mask}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = USStabilityNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
