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
    """
    3x3 旋转矩阵 -> 四元数 (x, y, z, w)
    """
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
    离线播放一组 (png + json) 数据：
      - /us_img : Image
      - /probe_pose : PoseStamped
    支持三种 json 来源模式：
      1) paired       : 使用 data_dir 里与 png 一一对应的 json（默认）
      2) fixed_dir    : 使用 json_dir 里的 json 序列，循环映射到所有 png
      3) fixed_single : 使用 json_file 指定的单个 json，所有帧同一姿态
    """

    def __init__(self):
        super().__init__('us_dataset_player_node')

        # 基本参数
        self.declare_parameter('data_dir', '')
        self.declare_parameter('image_topic', '/us_img')
        self.declare_parameter('probe_topic', '/probe_pose')
        self.declare_parameter('fps', 10.0)
        self.declare_parameter('loop', True)

        # json 来源模式参数
        self.declare_parameter('json_mode', 'paired')      # 'paired' | 'fixed_dir' | 'fixed_single'
        self.declare_parameter('json_dir', '')             # 当 json_mode = fixed_dir 使用
        self.declare_parameter('json_file', '')            # 当 json_mode = fixed_single 使用

        self.data_dir = self.get_parameter('data_dir').get_parameter_value().string_value
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.probe_topic = self.get_parameter('probe_topic').get_parameter_value().string_value
        self.fps = float(self.get_parameter('fps').get_parameter_value().double_value)
        self.loop = bool(self.get_parameter('loop').get_parameter_value().bool_value)

        self.json_mode = self.get_parameter('json_mode').get_parameter_value().string_value
        self.json_dir = self.get_parameter('json_dir').get_parameter_value().string_value
        self.json_file = self.get_parameter('json_file').get_parameter_value().string_value

        if not self.data_dir:
            raise RuntimeError('Parameter "data_dir" must be set.')

        if self.fps <= 0.0:
            self.fps = 10.0

        self.bridge = CvBridge()
        self.img_pub = self.create_publisher(Image, self.image_topic, 10)
        self.pose_pub = self.create_publisher(PoseStamped, self.probe_topic, 10)

        # 数据存储结构
        self.use_pairs: bool = False
        self.pairs: List[Tuple[str, str]] = []     # (png, json) 当 use_pairs=True
        self.image_files: List[str] = []           # 只保存 png
        self.json_files: List[str] = []            # 当 json_mode != paired 时用
        self.idx = 0

        self._init_dataset()

        period = 1.0 / self.fps
        self.timer = self.create_timer(period, self.timer_callback)

        if self.use_pairs:
            n_pairs = len(self.pairs)
        else:
            n_pairs = len(self.image_files)

        self.get_logger().info(
            f'USDatasetPlayerNode started.\n'
            f'  data_dir={self.data_dir}\n'
            f'  json_mode={self.json_mode}\n'
            f'  json_dir={self.json_dir}\n'
            f'  json_file={self.json_file}\n'
            f'  n_frames={n_pairs}, fps={self.fps}, loop={self.loop},\n'
            f'  image_topic={self.image_topic}, probe_topic={self.probe_topic}'
        )

    # ---------------- 初始化数据集 ----------------

    def _init_dataset(self):
        # 收集所有 png
        png_list = sorted(glob.glob(os.path.join(self.data_dir, '*.png')))
        if not png_list:
            raise RuntimeError(f'No png files found in data_dir: {self.data_dir}')

        if self.json_mode == 'paired':
            json_list = sorted(glob.glob(os.path.join(self.data_dir, '*.json')))
            if not json_list:
                self.get_logger().warn(
                    f'json_mode=paired but no json in data_dir={self.data_dir}, '
                    f'probe_pose will not be published.'
                )
                # 退化为只有图像的情况
                self.use_pairs = False
                self.image_files = png_list
                self.json_files = []
                return

            n = min(len(png_list), len(json_list))
            if len(png_list) != len(json_list):
                self.get_logger().warn(
                    f'json_mode=paired: png count={len(png_list)}, '
                    f'json count={len(json_list)}, using first {n} pairs by index.'
                )
            self.use_pairs = True
            self.pairs = list(zip(png_list[:n], json_list[:n]))

        elif self.json_mode == 'fixed_dir':
            if not self.json_dir:
                raise RuntimeError('json_mode=fixed_dir but "json_dir" is empty.')
            json_list = sorted(glob.glob(os.path.join(self.json_dir, '*.json')))
            if not json_list:
                raise RuntimeError(f'json_mode=fixed_dir but no json in json_dir: {self.json_dir}')
            self.use_pairs = False
            self.image_files = png_list
            self.json_files = json_list
            self.get_logger().info(
                f'json_mode=fixed_dir: {len(self.image_files)} images, '
                f'{len(self.json_files)} json files (will be used in cyclic way).'
            )

        elif self.json_mode == 'fixed_single':
            if not self.json_file:
                raise RuntimeError('json_mode=fixed_single but "json_file" is empty.')
            if not os.path.isfile(self.json_file):
                raise RuntimeError(f'json_mode=fixed_single but json_file not found: {self.json_file}')
            self.use_pairs = False
            self.image_files = png_list
            self.json_files = [self.json_file]
            self.get_logger().info(
                f'json_mode=fixed_single: {len(self.image_files)} images, '
                f'using single json_file={self.json_file} for all frames.'
            )

        else:
            raise RuntimeError(f'Unknown json_mode: {self.json_mode}')

    # ---------------- 定时器回调 ----------------

    def timer_callback(self):
        if self.use_pairs:
            self._from_pairs()
        else:
            self._from_separate_lists()

    def _from_pairs(self):
        if not self.pairs:
            return

        if self.idx >= len(self.pairs):
            if self.loop:
                self.idx = 0
                self.get_logger().info('Dataset finished (paired), looping...')
            else:
                self.get_logger().info('Dataset finished (paired), stopping timer.')
                self.timer.cancel()
                return

        png_path, json_path = self.pairs[self.idx]
        self.idx += 1
        self._publish_one(png_path, json_path)

    def _from_separate_lists(self):
        if not self.image_files:
            return

        if self.idx >= len(self.image_files):
            if self.loop:
                self.idx = 0
                self.get_logger().info('Dataset finished, looping...')
            else:
                self.get_logger().info('Dataset finished, stopping timer.')
                self.timer.cancel()
                return

        png_path = self.image_files[self.idx]

        # 根据模式选 json
        json_path: Optional[str] = None
        if self.json_files:
            if len(self.json_files) == 1:
                json_path = self.json_files[0]  # fixed_single
            else:
                # fixed_dir: 循环使用 json 序列
                j_idx = self.idx % len(self.json_files)
                json_path = self.json_files[j_idx]

        self.idx += 1
        self._publish_one(png_path, json_path)

    # ---------------- 发布一帧 ----------------

    def _publish_one(self, png_path: str, json_path: Optional[str]):
        # 1) 图像
        img = cv2.imread(png_path)
        if img is None:
            self.get_logger().warn(f'Failed to read image: {png_path}')
            return

        if len(img.shape) == 2:
            img_bgr = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        else:
            img_bgr = img

        img_msg = self.bridge.cv2_to_imgmsg(img_bgr, encoding='bgr8')
        now = self.get_clock().now().to_msg()
        img_msg.header.stamp = now
        img_msg.header.frame_id = 'us_image'
        self.img_pub.publish(img_msg)

        # 2) probe_pose（如果有 json）
        if json_path is None:
            # 允许只发图像
            self.get_logger().debug(
                f'Published image only (no json): {os.path.basename(png_path)}'
            )
            return

        try:
            with open(json_path, 'r') as f:
                data = json.load(f)
        except Exception as e:
            self.get_logger().warn(f'Failed to read json: {json_path}, err={e}')
            return

        transforms = data.get('transforms', None)
        if transforms is None or 'probe' not in transforms:
            self.get_logger().warn(
                f'No "transforms/probe" in json, skip pose: {os.path.basename(json_path)}'
            )
            return

        probe = transforms['probe']

        mat = np.array(probe['matrix'], dtype=float)
        R = mat[0:3, 0:3]
        t_mm = np.array(probe['translation'], dtype=float)
        t_m = t_mm * 0.001

        qx, qy, qz, qw = rotm_to_quat(R)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = now
        pose_msg.header.frame_id = 'polaris_world'

        pose_msg.pose.position.x = float(t_m[0])
        pose_msg.pose.position.y = float(t_m[1])
        pose_msg.pose.position.z = float(t_m[2])

        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw

        self.pose_pub.publish(pose_msg)

        self.get_logger().debug(
            f'Published pair: img={os.path.basename(png_path)}, '
            f'json={os.path.basename(json_path)}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = USDatasetPlayerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
