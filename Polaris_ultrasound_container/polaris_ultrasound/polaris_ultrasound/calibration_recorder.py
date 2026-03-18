#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import os
import json
import datetime
from std_msgs.msg import String

class CalibrationDataRecorder(Node):
    def __init__(self):
        super().__init__('calibration_data_recorder')

        self.declare_parameter('device_id', 6)
        self.declare_parameter('save_path', '/ros2_ws/calibration_data')
        self.declare_parameter('frame_rate', 30.0)
        self.declare_parameter('tool_names', ['probe', 'phantom'])
        self.declare_parameter('record_time', 60.0)

        self.device_id = self.get_parameter('device_id').value
        self.save_path = self.get_parameter('save_path').value
        self.frame_rate = self.get_parameter('frame_rate').value
        self.tool_names = self.get_parameter('tool_names').value
        self.record_time = self.get_parameter('record_time').value

        self.session_timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.ultrasound_dir = os.path.join(self.save_path, "ultrasound", self.session_timestamp)
        self.polaris_dir = os.path.join(self.save_path, "polaris", self.session_timestamp)

        os.makedirs(self.ultrasound_dir, exist_ok=True)
        os.makedirs(self.polaris_dir, exist_ok=True)

        self.cap = cv2.VideoCapture(self.device_id)
        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open video device {self.device_id}")
            raise RuntimeError(f"Failed to open video device {self.device_id}")

        self.polaris_subscription = self.create_subscription(String, 'ndi_transforms', self.polaris_callback, 10)
        self.latest_polaris_data = None

        self.frame_counter = 0
        self.polaris_counter = 0

        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.timer = self.create_timer(1.0 / self.frame_rate, self.timer_callback)
        self.started = False  # 控制是否输出了开始信息

    def polaris_callback(self, msg):
        try:
            data = json.loads(msg.data)
            timestamp_ros = float(data['timestamp'])
            timestamp_hw = float(data.get('original_timestamp', timestamp_ros))

            transforms = {}
            for t in data.get('transforms', []):
                if 'tool_name' in t and 'matrix' in t:
                    tool = t['tool_name']
                    transforms[tool] = {
                        'tool_id': t.get('tool_id', ''),
                        'quality': t.get('quality', 0.0),
                        'matrix': t['matrix'],
                        'translation': t.get('translation', [])
                    }

            self.latest_polaris_data = {
                'timestamp_ros': timestamp_ros,
                'timestamp_hw': timestamp_hw,
                'transforms': transforms
            }

        except Exception as e:
            self.get_logger().error(f"Error in Polaris callback: {str(e)}")

    def timer_callback(self):
        now = self.get_clock().now().nanoseconds / 1e9
        elapsed = now - self.start_time

        if not self.started:
            self.get_logger().info(f"Start recording... total: {self.record_time:.1f} s")
            self.started = True

        # 实时动态进度条
        percent = min(elapsed / self.record_time, 1.0)
        percent_display = int(percent * 100)
        bar_len = 30
        filled_len = int(bar_len * percent)
        bar = '█' * filled_len + '░' * (bar_len - filled_len)
        print(f"\r[Recording] {bar} {percent_display:>3}% (Elapsed: {elapsed:.2f}s / {self.record_time:.1f}s)", end='', flush=True)

        if elapsed > self.record_time:
            print()  # 换行，清除进度条显示
            self.get_logger().info("Recording complete.")
            rclpy.shutdown()
            return

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to capture image.")
            return

        us_ts = self.get_clock().now().nanoseconds / 1e9
        us_filename = f"us_{self.frame_counter:06d}_{us_ts:.6f}.png"
        us_path = os.path.join(self.ultrasound_dir, us_filename)
        cv2.imwrite(us_path, frame)
        self.frame_counter += 1

        if self.latest_polaris_data and all(tool in self.latest_polaris_data['transforms'] for tool in self.tool_names):
            polaris_filename = f"frame_{self.polaris_counter:06d}_{self.latest_polaris_data['timestamp_hw']:.6f}.json"
            polaris_path = os.path.join(self.polaris_dir, polaris_filename)

            save_data = {
                'timestamp_ros': self.latest_polaris_data['timestamp_ros'],
                'timestamp_hw': self.latest_polaris_data['timestamp_hw'],
                'frame_number': self.polaris_counter,
                'transforms': self.latest_polaris_data['transforms']
            }

            with open(polaris_path, 'w') as f:
                json.dump(save_data, f, indent=2)

            self.polaris_counter += 1

    def destroy_node(self):
        if hasattr(self, 'cap') and self.cap:
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CalibrationDataRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
