import os
import glob
import json
import math
from typing import List, Tuple, Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge


def rotm_to_quat(R: np.ndarray):
    """3x3 rotation matrix -> quaternion (x, y, z, w)."""
    t = np.trace(R)
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    else:
        if (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
            s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = math.sqrt(1.0 - R[0, 0] + R[1, 1] - R[2, 2]) * 2.0
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = math.sqrt(1.0 - R[0, 0] - R[1, 1] + R[2, 2]) * 2.0
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
    return float(x), float(y), float(z), float(w)


class USDatasetPlayerNode(Node):
    """
    Offline playback for ultrasound dataset.

    Publishes:
    - image_topic: Image
    - probe_topic: PoseStamped

    Supported json modes:
    - paired       : use png/json pairs from data_dir
    - fixed_dir    : use json files from json_dir cyclically
    - fixed_single : use one json_file for all frames
    """

    VALID_JSON_MODES = {'paired', 'fixed_dir', 'fixed_single'}

    def __init__(self):
        super().__init__('us_dataset_player_node')

        # Parameters
        self.declare_parameter('data_dir', '')
        self.declare_parameter('image_topic', '/us_img')
        self.declare_parameter('probe_topic', '/probe_pose')
        self.declare_parameter('image_frame_id', 'us_image')
        self.declare_parameter('probe_frame_id', 'polaris_world')
        self.declare_parameter('fps', 10.0)
        self.declare_parameter('loop', True)

        self.declare_parameter('json_mode', 'paired')
        self.declare_parameter('json_dir', '')
        self.declare_parameter('json_file', '')

        self.data_dir = self.get_parameter('data_dir').value
        self.image_topic = self.get_parameter('image_topic').value
        self.probe_topic = self.get_parameter('probe_topic').value
        self.image_frame_id = self.get_parameter('image_frame_id').value
        self.probe_frame_id = self.get_parameter('probe_frame_id').value
        self.fps = float(self.get_parameter('fps').value)
        self.loop = bool(self.get_parameter('loop').value)

        self.json_mode = self.get_parameter('json_mode').value
        self.json_dir = self.get_parameter('json_dir').value
        self.json_file = self.get_parameter('json_file').value

        if not self.data_dir:
            raise RuntimeError('Parameter "data_dir" must be set.')
        if not os.path.isdir(self.data_dir):
            raise RuntimeError(f'data_dir does not exist: {self.data_dir}')

        if self.json_mode not in self.VALID_JSON_MODES:
            raise RuntimeError(
                f'Unknown json_mode: {self.json_mode}. '
                f'Valid values: {sorted(self.VALID_JSON_MODES)}'
            )

        if self.fps <= 0.0:
            raise RuntimeError('fps must be > 0.')

        self.bridge = CvBridge()
        self.img_pub = self.create_publisher(Image, self.image_topic, 10)
        self.pose_pub = self.create_publisher(PoseStamped, self.probe_topic, 10)

        self.use_pairs: bool = False
        self.pairs: List[Tuple[str, str]] = []
        self.image_files: List[str] = []
        self.json_files: List[str] = []
        self.idx = 0

        self._init_dataset()

        self.timer = self.create_timer(1.0 / self.fps, self.timer_callback)

        n_frames = len(self.pairs) if self.use_pairs else len(self.image_files)
        self.get_logger().info(
            'USDatasetPlayerNode started\n'
            f'  data_dir      : {self.data_dir}\n'
            f'  json_mode     : {self.json_mode}\n'
            f'  json_dir      : {self.json_dir}\n'
            f'  json_file     : {self.json_file}\n'
            f'  n_frames      : {n_frames}\n'
            f'  fps           : {self.fps}\n'
            f'  loop          : {self.loop}\n'
            f'  image_topic   : {self.image_topic}\n'
            f'  probe_topic   : {self.probe_topic}'
        )

    def _init_dataset(self):
        png_list = sorted(glob.glob(os.path.join(self.data_dir, '*.png')))
        if not png_list:
            raise RuntimeError(f'No png files found in data_dir: {self.data_dir}')

        if self.json_mode == 'paired':
            json_list = sorted(glob.glob(os.path.join(self.data_dir, '*.json')))
            if not json_list:
                self.get_logger().warn(
                    f'json_mode=paired but no json files found in {self.data_dir}. '
                    'Only images will be published.'
                )
                self.use_pairs = False
                self.image_files = png_list
                self.json_files = []
                return

            n = min(len(png_list), len(json_list))
            if len(png_list) != len(json_list):
                self.get_logger().warn(
                    f'png count={len(png_list)}, json count={len(json_list)}. '
                    f'Using first {n} pairs by index.'
                )
            self.use_pairs = True
            self.pairs = list(zip(png_list[:n], json_list[:n]))
            return

        self.use_pairs = False
        self.image_files = png_list

        if self.json_mode == 'fixed_dir':
            if not self.json_dir:
                raise RuntimeError('json_mode=fixed_dir but "json_dir" is empty.')
            if not os.path.isdir(self.json_dir):
                raise RuntimeError(f'json_dir does not exist: {self.json_dir}')

            self.json_files = sorted(glob.glob(os.path.join(self.json_dir, '*.json')))
            if not self.json_files:
                raise RuntimeError(f'No json files found in json_dir: {self.json_dir}')

        elif self.json_mode == 'fixed_single':
            if not self.json_file:
                raise RuntimeError('json_mode=fixed_single but "json_file" is empty.')
            if not os.path.isfile(self.json_file):
                raise RuntimeError(f'json_file does not exist: {self.json_file}')
            self.json_files = [self.json_file]

    def timer_callback(self):
        if self.use_pairs:
            self._play_from_pairs()
        else:
            self._play_from_separate_lists()

    def _play_from_pairs(self):
        if not self.pairs:
            return

        if self.idx >= len(self.pairs):
            if self.loop:
                self.idx = 0
                self.get_logger().info('Dataset finished (paired), looping...')
            else:
                self.get_logger().info('Dataset finished (paired), stopping.')
                self.timer.cancel()
                return

        png_path, json_path = self.pairs[self.idx]
        self.idx += 1
        self._publish_one(png_path, json_path)

    def _play_from_separate_lists(self):
        if not self.image_files:
            return

        if self.idx >= len(self.image_files):
            if self.loop:
                self.idx = 0
                self.get_logger().info('Dataset finished, looping...')
            else:
                self.get_logger().info('Dataset finished, stopping.')
                self.timer.cancel()
                return

        png_path = self.image_files[self.idx]

        json_path: Optional[str] = None
        if self.json_files:
            if len(self.json_files) == 1:
                json_path = self.json_files[0]
            else:
                j_idx = self.idx % len(self.json_files)
                json_path = self.json_files[j_idx]

        self.idx += 1
        self._publish_one(png_path, json_path)

    def _publish_one(self, png_path: str, json_path: Optional[str]):
        img = cv2.imread(png_path, cv2.IMREAD_COLOR)
        if img is None:
            self.get_logger().warn(f'Failed to read image: {png_path}')
            return

        img_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        now = self.get_clock().now().to_msg()
        img_msg.header.stamp = now
        img_msg.header.frame_id = self.image_frame_id
        self.img_pub.publish(img_msg)

        if json_path is None:
            return

        try:
            with open(json_path, 'r') as f:
                data = json.load(f)
        except Exception as e:
            self.get_logger().warn(f'Failed to read json {json_path}: {e}')
            return

        transforms = data.get('transforms', None)
        if transforms is None or 'probe' not in transforms:
            self.get_logger().warn(
                f'No "transforms/probe" found in {os.path.basename(json_path)}. Skipping pose.'
            )
            return

        probe = transforms['probe']
        if 'matrix' not in probe or 'translation' not in probe:
            self.get_logger().warn(
                f'Incomplete probe transform in {os.path.basename(json_path)}. Skipping pose.'
            )
            return

        try:
            mat = np.array(probe['matrix'], dtype=float)
            R = mat[0:3, 0:3]
            t_mm = np.array(probe['translation'], dtype=float)
            t_m = t_mm * 0.001
            qx, qy, qz, qw = rotm_to_quat(R)
        except Exception as e:
            self.get_logger().warn(f'Failed to parse pose from {json_path}: {e}')
            return

        pose_msg = PoseStamped()
        pose_msg.header.stamp = now
        pose_msg.header.frame_id = self.probe_frame_id
        pose_msg.pose.position.x = float(t_m[0])
        pose_msg.pose.position.y = float(t_m[1])
        pose_msg.pose.position.z = float(t_m[2])
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw
        self.pose_pub.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    node = USDatasetPlayerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
