import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class UltrasoundReaderNode(Node):
    def __init__(self):
        super().__init__('ultrasound_reader')

        # ---------------- Parameters ----------------
        self.declare_parameter('device_id', 0)
        self.declare_parameter('image_topic', '/us_img')
        self.declare_parameter('roi_topic', '/us_roi')
        self.declare_parameter('publish_roi', False)
        self.declare_parameter('frame_rate', 30.0)
        self.declare_parameter('show_image', False)
        self.declare_parameter('width', 0)
        self.declare_parameter('height', 0)

        self.device_id = self.get_parameter('device_id').get_parameter_value().integer_value
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.roi_topic = self.get_parameter('roi_topic').get_parameter_value().string_value
        self.publish_roi = self.get_parameter('publish_roi').get_parameter_value().bool_value
        self.frame_rate = self.get_parameter('frame_rate').get_parameter_value().double_value
        self.show_image = self.get_parameter('show_image').get_parameter_value().bool_value
        self.width = self.get_parameter('width').get_parameter_value().integer_value
        self.height = self.get_parameter('height').get_parameter_value().integer_value

        # ---------------- Publishers ----------------
        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, self.image_topic, 10)
        self.roi_pub = self.create_publisher(Image, self.roi_topic, 10) if self.publish_roi else None

        # ---------------- Camera ----------------
        self.cap = cv2.VideoCapture(int(self.device_id))
        if not self.cap.isOpened():
            raise RuntimeError(f'Failed to open ultrasound video device: {self.device_id}')

        if self.width > 0:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(self.width))
        if self.height > 0:
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(self.height))

        if self.frame_rate <= 0.0:
            raise ValueError('frame_rate must be > 0')

        self.get_logger().info(f'Ultrasound device opened: {self.device_id}')
        self.get_logger().info(f'Image topic: {self.image_topic}')
        if self.publish_roi:
            self.get_logger().info(f'ROI topic: {self.roi_topic}')

        self.timer = self.create_timer(1.0 / self.frame_rate, self.timer_callback)

    def extract_roi(self, frame):
        """
        Default behavior: return the full frame.
        Replace this method later if you want a specific cropped ROI.
        """
        return frame

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret or frame is None:
            self.get_logger().warn('Failed to read frame from ultrasound device.')
            return

        # Publish full image
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        img_msg.header.stamp = self.get_clock().now().to_msg()
        self.image_pub.publish(img_msg)

        # Publish ROI if enabled
        if self.publish_roi and self.roi_pub is not None:
            roi = self.extract_roi(frame)
            roi_msg = self.bridge.cv2_to_imgmsg(roi, encoding='bgr8')
            roi_msg.header.stamp = img_msg.header.stamp
            self.roi_pub.publish(roi_msg)

        # Optional display
        if self.show_image:
            cv2.imshow('ultrasound_reader', frame)
            cv2.waitKey(1)

    def destroy_node(self):
        try:
            if hasattr(self, 'cap') and self.cap is not None:
                self.cap.release()
            if self.show_image:
                cv2.destroyAllWindows()
        except Exception as e:
            self.get_logger().warn(f'Error while closing ultrasound reader: {e}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = UltrasoundReaderNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if node is not None:
            node.get_logger().error(f'Ultrasound reader failed: {e}')
        else:
            print(f'Ultrasound reader failed before node init: {e}')
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
