#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
import os
from typing import List, Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import Float32
from geometry_msgs.msg import PointStamped


class PerceptionReplayNode(Node):
    """
    Replay perception results from a CSV file (recorded from real experiments)
    and publish them in simulation.

    Publishes:
      - /perception/target_point (geometry_msgs/PointStamped)
      - /perception/sk           (std_msgs/Float32)

    Key feature:
      - Apply optional fixed offset (offset_x/y/z) to target point before publishing.

    Parameters:
      - csv_path (string): path to CSV file
      - rate_hz (double): publish rate (Hz)
      - loop (bool): loop playback
      - frame_id (string): frame_id for PointStamped (e.g., robot_base_link)

      - target_topic (string): default '/perception/target_point'
      - sk_topic (string): default '/perception/sk'

      - offset_x (double): meters
      - offset_y (double): meters
      - offset_z (double): meters

      - col_x (string): default 'x_arm'
      - col_y (string): default 'y_arm'
      - col_z (string): default 'z_arm'
      - col_sk (string): default 'sk'   (if missing, will publish 1.0)
    """

    def __init__(self):
        super().__init__("perception_replay_node")

        # ---- parameters ----
        self.csv_path = str(self.declare_parameter("csv_path", "").value)
        self.rate_hz = float(self.declare_parameter("rate_hz", 20.0).value)
        self.loop = bool(self.declare_parameter("loop", True).value)
        self.frame_id = str(self.declare_parameter("frame_id", "robot_base_link").value)

        self.target_topic = str(self.declare_parameter("target_topic", "/perception/target_point").value)
        self.sk_topic = str(self.declare_parameter("sk_topic", "/perception/sk").value)

        self.offset_x = float(self.declare_parameter("offset_x", 0.0).value)
        self.offset_y = float(self.declare_parameter("offset_y", 0.0).value)
        self.offset_z = float(self.declare_parameter("offset_z", 0.0).value)

        self.col_x = str(self.declare_parameter("col_x", "x_arm").value)
        self.col_y = str(self.declare_parameter("col_y", "y_arm").value)
        self.col_z = str(self.declare_parameter("col_z", "z_arm").value)
        self.col_sk = str(self.declare_parameter("col_sk", "sk").value)

        # ---- publishers ----
        self.pub_target = self.create_publisher(PointStamped, self.target_topic, qos_profile_sensor_data)
        self.pub_sk = self.create_publisher(Float32, self.sk_topic, qos_profile_sensor_data)

        # ---- load CSV ----
        self.rows: List[Dict[str, str]] = []
        self.idx = 0

        self._load_csv_or_raise()

        # ---- timer ----
        period = 1.0 / max(1e-6, self.rate_hz)
        self.timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f"Started perception replay.\n"
            f"  csv_path='{self.csv_path}' rows={len(self.rows)} rate_hz={self.rate_hz} loop={self.loop}\n"
            f"  frame_id='{self.frame_id}'\n"
            f"  target_topic='{self.target_topic}', sk_topic='{self.sk_topic}'\n"
            f"  offset(m)=({self.offset_x:.6f}, {self.offset_y:.6f}, {self.offset_z:.6f})\n"
            f"  cols: x='{self.col_x}', y='{self.col_y}', z='{self.col_z}', sk='{self.col_sk}'"
        )

    def _load_csv_or_raise(self):
        if not self.csv_path:
            raise RuntimeError("Parameter 'csv_path' is empty. Provide -p csv_path:=/path/to/file.csv")

        if not os.path.exists(self.csv_path):
            raise RuntimeError(f"CSV not found: {self.csv_path}")

        with open(self.csv_path, "r", newline="") as f:
            reader = csv.DictReader(f)
            if reader.fieldnames is None:
                raise RuntimeError("CSV has no header/fieldnames. Ensure the first row is a header.")

            self.rows = list(reader)

        if len(self.rows) == 0:
            raise RuntimeError("CSV is empty (0 data rows).")

        # minimal column existence check for x/y/z
        for c in (self.col_x, self.col_y, self.col_z):
            if c not in (self.rows[0].keys()):
                raise RuntimeError(
                    f"CSV missing required column '{c}'. "
                    f"Available columns: {list(self.rows[0].keys())}"
                )

    def _get_float(self, row: Dict[str, str], key: str, default: Optional[float] = None) -> float:
        if key not in row or row[key] is None or row[key] == "":
            if default is None:
                raise ValueError(f"Missing value for column '{key}' in CSV row.")
            return float(default)
        return float(row[key])

    def _publish_one(self, row: Dict[str, str]):
        stamp = self.get_clock().now().to_msg()

        # ---- target point ----
        x = self._get_float(row, self.col_x) + self.offset_x
        y = self._get_float(row, self.col_y) + self.offset_y
        z = self._get_float(row, self.col_z) + self.offset_z

        msg_t = PointStamped()
        msg_t.header.stamp = stamp
        msg_t.header.frame_id = self.frame_id
        msg_t.point.x = float(x)
        msg_t.point.y = float(y)
        msg_t.point.z = float(z)
        self.pub_target.publish(msg_t)

        # ---- sk ----
        # If the CSV doesn't have col_sk, publish 1.0 by default (stable)
        if self.col_sk in row and row[self.col_sk] not in (None, ""):
            sk_val = float(row[self.col_sk])
        else:
            sk_val = 1.0

        msg_sk = Float32()
        msg_sk.data = float(sk_val)
        self.pub_sk.publish(msg_sk)

    def _on_timer(self):
        if self.idx >= len(self.rows):
            if self.loop:
                self.idx = 0
            else:
                self.get_logger().info("Replay finished (loop:=false). Stopping timer.")
                self.timer.cancel()
                return

        try:
            self._publish_one(self.rows[self.idx])
        except Exception as e:
            self.get_logger().error(f"Failed to publish row idx={self.idx}: {e}")
        finally:
            self.idx += 1


def main():
    rclpy.init()
    node = PerceptionReplayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

