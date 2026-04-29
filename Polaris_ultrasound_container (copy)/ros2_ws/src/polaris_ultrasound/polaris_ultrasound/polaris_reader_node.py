import json
import time
from typing import List, Dict, Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from sksurgerynditracker.nditracker import NDITracker


class PolarisReaderNode(Node):
    def __init__(self):
        super().__init__('polaris_reader')

        # ---------------- Parameters ----------------
        self.declare_parameter('rom_path', '')
        self.declare_parameter('tool_names', '')
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('output_topic', 'ndi_transforms')
        self.declare_parameter('frame_id', 'polaris')
        self.declare_parameter('publish_rate', 20.0)

        rom_path_str = self.get_parameter('rom_path').get_parameter_value().string_value
        tool_names_str = self.get_parameter('tool_names').get_parameter_value().string_value
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        self.rom_files = [x.strip() for x in rom_path_str.split(',') if x.strip()]
        self.tool_names = [x.strip() for x in tool_names_str.split(',') if x.strip()]

        if not self.rom_files:
            raise ValueError('Parameter "rom_path" is empty. Please provide at least one ROM file.')
        if not self.tool_names:
            raise ValueError('Parameter "tool_names" is empty. Please provide at least one tool name.')
        if len(self.rom_files) != len(self.tool_names):
            raise ValueError(
                f'rom_path count ({len(self.rom_files)}) does not match tool_names count ({len(self.tool_names)}).'
            )

        # ---------------- Publisher ----------------
        self.publisher_ = self.create_publisher(String, self.output_topic, 10)

        # ---------------- Tracker ----------------
        tracker_config = {
            'tracker type': 'polaris',
            'romfiles': self.rom_files,
            'serial port': self.serial_port,
        }

        self.get_logger().info(f'Initializing Polaris tracker on {self.serial_port}')
        self.get_logger().info(f'ROM files: {self.rom_files}')
        self.get_logger().info(f'Tool names: {self.tool_names}')
        self.get_logger().info(f'Output topic: {self.output_topic}')

        self.tracker = NDITracker(tracker_config)
        self.tracker.start_tracking()

        # ---------------- Timer ----------------
        if self.publish_rate <= 0.0:
            raise ValueError('publish_rate must be > 0')
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

    def timer_callback(self):
        try:
            frame = self.tracker.get_frame()
        except Exception as e:
            self.get_logger().error(f'Failed to get frame from Polaris: {e}')
            return

        if frame is None or len(frame) < 4:
            self.get_logger().warn('Invalid frame received from Polaris.')
            return

        port_handles, timestamps, framenumbers, tracking, quality = frame

        payload = {
            'frame_id': self.frame_id,
            'timestamp_ros_sec': self.get_clock().now().nanoseconds * 1e-9,
            'tracker_timestamp': timestamps,
            'frame_number': framenumbers,
            'tools': [],
        }

        for i, tool_name in enumerate(self.tool_names):
            tool_data: Dict[str, Any] = {
                'tool_name': tool_name,
                'valid': False,
                'quality': None,
                'matrix': None,
                'translation': None,
                'rotation': None,
            }

            if i < len(tracking) and tracking[i] is not None:
                mat = tracking[i]
                tool_data['valid'] = True
                tool_data['matrix'] = mat.tolist()
                tool_data['translation'] = mat[:3, 3].tolist()
                tool_data['rotation'] = mat[:3, :3].tolist()

            if i < len(quality):
                try:
                    tool_data['quality'] = float(quality[i])
                except Exception:
                    tool_data['quality'] = None

            payload['tools'].append(tool_data)

        msg = String()
        msg.data = json.dumps(payload)
        self.publisher_.publish(msg)

    def destroy_node(self):
        try:
            if hasattr(self, 'tracker'):
                self.tracker.stop_tracking()
                self.tracker.close()
        except Exception as e:
            self.get_logger().warn(f'Error while closing Polaris tracker: {e}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PolarisReaderNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if node is not None:
            node.get_logger().error(f'Polaris reader failed: {e}')
        else:
            print(f'Polaris reader failed before node init: {e}')
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
