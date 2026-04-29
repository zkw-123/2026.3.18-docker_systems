#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
from typing import Any, Dict, Tuple

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import PointStamped


class TargetPointAdapter(Node):
    """
    Adapter from JSON target position to geometry_msgs/PointStamped.

    Expected input example:
        {"x": 12.3, "y": 45.6, "z": 78.9}

    or:
        {"position": {"x": 12.3, "y": 45.6, "z": 78.9}}

    or:
        {"point": [12.3, 45.6, 78.9]}

    Unit conversion:
        input_unit = "mm" -> output in meters
        input_unit = "m"  -> output in meters
    """

    def __init__(self) -> None:
        super().__init__("target_point_adapter")

        self.declare_parameter("input_topic", "/target_3d_position")
        self.declare_parameter("output_topic", "/perception/target_point")
        self.declare_parameter("frame_id", "robot_base_link")
        self.declare_parameter("input_unit", "mm")

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.input_unit = str(self.get_parameter("input_unit").value).lower()

        if self.input_unit == "mm":
            self.scale = 1.0e-3
        elif self.input_unit == "m":
            self.scale = 1.0
        else:
            raise ValueError("input_unit must be 'mm' or 'm'.")

        self.sub = self.create_subscription(
            String,
            self.input_topic,
            self._callback,
            10,
        )

        self.pub = self.create_publisher(
            PointStamped,
            self.output_topic,
            10,
        )

        self.get_logger().info("Target point adapter started.")
        self.get_logger().info(f"input_topic = {self.input_topic}")
        self.get_logger().info(f"output_topic = {self.output_topic}")
        self.get_logger().info(f"input_unit = {self.input_unit}, scale = {self.scale}")

    def _extract_xyz(self, data: Dict[str, Any]) -> Tuple[float, float, float]:
        if all(k in data for k in ["x", "y", "z"]):
            return float(data["x"]), float(data["y"]), float(data["z"])

        if "position" in data and isinstance(data["position"], dict):
            p = data["position"]
            return float(p["x"]), float(p["y"]), float(p["z"])

        if "point" in data:
            p = data["point"]
            if isinstance(p, list) and len(p) >= 3:
                return float(p[0]), float(p[1]), float(p[2])
            if isinstance(p, dict):
                return float(p["x"]), float(p["y"]), float(p["z"])

        if "target" in data:
            p = data["target"]
            if isinstance(p, dict):
                return float(p["x"]), float(p["y"]), float(p["z"])
            if isinstance(p, list) and len(p) >= 3:
                return float(p[0]), float(p[1]), float(p[2])

        raise ValueError(
            "Cannot extract x, y, z. Expected keys: "
            "{x,y,z}, position, point, or target."
        )

    def _callback(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            x, y, z = self._extract_xyz(data)
        except Exception as exc:
            self.get_logger().error(f"Failed to parse target JSON: {exc}")
            return

        out = PointStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id

        out.point.x = x * self.scale
        out.point.y = y * self.scale
        out.point.z = z * self.scale

        self.pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TargetPointAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()