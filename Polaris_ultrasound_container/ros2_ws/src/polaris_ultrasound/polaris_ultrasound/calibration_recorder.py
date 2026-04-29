#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import os
import json
import math
import datetime
from std_msgs.msg import String


class CalibrationDataRecorder(Node):
    def __init__(self):
        super().__init__('calibration_recorder')

        self.declare_parameter('device_id', 0)
        self.declare_parameter('save_path', '/workspace/data/calibration_data')
        self.declare_parameter('frame_rate', 30.0)
        self.declare_parameter('tool_names', 'phantom,stylus')
        self.declare_parameter('record_time', 60.0)
        self.declare_parameter('polaris_topic', 'ndi_transforms')

        self.device_id = int(self.get_parameter('device_id').value)
        self.save_path = str(self.get_parameter('save_path').value)
        self.frame_rate = float(self.get_parameter('frame_rate').value)
        self.record_time = float(self.get_parameter('record_time').value)
        self.polaris_topic = str(self.get_parameter('polaris_topic').value)

        tool_names_str = str(self.get_parameter('tool_names').value)
        self.tool_names = [x.strip() for x in tool_names_str.split(',') if x.strip()]
        if not self.tool_names:
            raise RuntimeError('tool_names is empty.')

        self.session_timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.ultrasound_dir = os.path.join(self.save_path, "ultrasound", self.session_timestamp)
        self.polaris_dir = os.path.join(self.save_path, "polaris", self.session_timestamp)

        os.makedirs(self.ultrasound_dir, exist_ok=True)
        os.makedirs(self.polaris_dir, exist_ok=True)

        self.cap = cv2.VideoCapture(self.device_id)
        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open video device {self.device_id}")
            raise RuntimeError(f"Failed to open video device {self.device_id}")

        self.polaris_subscription = self.create_subscription(
            String,
            self.polaris_topic,
            self.polaris_callback,
            10
        )

        self.latest_polaris_data = None
        self.frame_counter = 0
        self.polaris_counter = 0

        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.timer = self.create_timer(1.0 / self.frame_rate, self.timer_callback)
        self.started = False

        self.last_progress_log_time = 0.0
        self.progress_log_interval = 1.0

        self.last_status_log_time = 0.0
        self.status_log_interval = 1.0

        self.get_logger().info(
            f'Calibration recorder ready. '
            f'device_id={self.device_id}, save_path={self.save_path}, '
            f'tool_names={self.tool_names}, polaris_topic={self.polaris_topic}'
        )

    def _has_nan(self, value):
        try:
            if value is None:
                return False
            if isinstance(value, (int, float)):
                return math.isnan(value) if isinstance(value, float) else False
            if isinstance(value, list):
                for v in value:
                    if self._has_nan(v):
                        return True
            return False
        except Exception:
            return True

    def log_polaris_status(self):
        if self.latest_polaris_data is None:
            self.get_logger().warn('No Polaris data received yet.')
            return

        transforms = self.latest_polaris_data.get('transforms', {})
        parts = []

        for tool in self.tool_names:
            if tool not in transforms:
                parts.append(f'{tool}: missing')
                continue

            t = transforms[tool]
            valid = t.get('valid', False)
            quality = t.get('quality', None)
            translation = t.get('translation', None)
            matrix = t.get('matrix', None)
            has_nan = self._has_nan(translation) or self._has_nan(matrix)

            parts.append(
                f'{tool}: valid={valid}, quality={quality}, '
                f'translation={translation}, nan={has_nan}, '
                f'matrix={"ok" if matrix is not None else "None"}'
            )

        self.get_logger().info('Polaris status | ' + ' | '.join(parts))

    def polaris_callback(self, msg):
        try:
            data = json.loads(msg.data)

            # New Polaris payload format
            timestamp_ros = float(data.get('timestamp_ros_sec', 0.0))

            tracker_timestamp = data.get('tracker_timestamp', None)
            if isinstance(tracker_timestamp, list) and len(tracker_timestamp) > 0:
                try:
                    timestamp_hw = float(tracker_timestamp[0])
                except Exception:
                    timestamp_hw = timestamp_ros
            elif tracker_timestamp is not None:
                try:
                    timestamp_hw = float(tracker_timestamp)
                except Exception:
                    timestamp_hw = timestamp_ros
            else:
                timestamp_hw = timestamp_ros

            frame_number = data.get('frame_number', None)
            if isinstance(frame_number, list) and len(frame_number) > 0:
                frame_number_value = frame_number[0]
            else:
                frame_number_value = frame_number

            transforms = {}
            for t in data.get('tools', []):
                tool = t.get('tool_name', '')
                if not tool:
                    continue

                transforms[tool] = {
                    'quality': t.get('quality', None),
                    'valid': t.get('valid', False),
                    'matrix': t.get('matrix', None),
                    'translation': t.get('translation', []),
                    'rotation': t.get('rotation', None)
                }

            self.latest_polaris_data = {
                'timestamp_ros': timestamp_ros,
                'timestamp_hw': timestamp_hw,
                'frame_number': frame_number_value,
                'transforms': transforms
            }

            now = self.get_clock().now().nanoseconds / 1e9
            if now - self.last_status_log_time >= self.status_log_interval:
                self.log_polaris_status()
                self.last_status_log_time = now

        except Exception as e:
            self.get_logger().error(f"Error in Polaris callback: {str(e)}")

    def timer_callback(self):
        now = self.get_clock().now().nanoseconds / 1e9
        elapsed = now - self.start_time

        if not self.started:
            self.get_logger().info(f"Start recording... total: {self.record_time:.1f} s")
            self.started = True

        percent = min(elapsed / self.record_time, 1.0)
        percent_display = int(percent * 100)

        if elapsed - self.last_progress_log_time >= self.progress_log_interval:
            self.get_logger().info(
                f'Recording progress: {percent_display}% '
                f'({elapsed:.2f}s / {self.record_time:.1f}s), '
                f'us_frames={self.frame_counter}, polaris_frames={self.polaris_counter}'
            )
            self.last_progress_log_time = elapsed

        if elapsed > self.record_time:
            self.get_logger().info("Recording complete.")
            self.timer.cancel()
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

        if self.latest_polaris_data is None:
            return

        if not all(tool in self.latest_polaris_data['transforms'] for tool in self.tool_names):
            return

        for tool in self.tool_names:
            tool_data = self.latest_polaris_data['transforms'][tool]
            if not tool_data.get('valid', False):
                return
            if tool_data.get('matrix', None) is None:
                return
            if self._has_nan(tool_data.get('matrix', None)):
                return
            if self._has_nan(tool_data.get('translation', None)):
                return

        polaris_filename = (
            f"frame_{self.polaris_counter:06d}_"
            f"{self.latest_polaris_data['timestamp_hw']:.6f}.json"
        )
        polaris_path = os.path.join(self.polaris_dir, polaris_filename)

        save_data = {
            'timestamp_ros': self.latest_polaris_data['timestamp_ros'],
            'timestamp_hw': self.latest_polaris_data['timestamp_hw'],
            'frame_number': self.latest_polaris_data['frame_number'],
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
