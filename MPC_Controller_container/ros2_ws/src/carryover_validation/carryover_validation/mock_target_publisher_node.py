#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import random
from typing import Dict

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float64


class MockTargetPublisher(Node):
    """
    Mock target publisher for carryover_validation simulation tests.

    This node publishes a fake target position as std_msgs/String JSON.

    Output:
        /target_3d_position
        std_msgs/msg/String

    JSON format:
        {
            "x": 300.0,
            "y": 0.0,
            "z": 200.0,
            "unit": "mm",
            "source": "mock_target_publisher"
        }

    The target is updated when /carryover/dk_cmd is received:

        target_lateral += beta_true * d_k

    where d_k is assumed to be in meters.
    The published target position is in millimeters by default.
    """

    def __init__(self) -> None:
        super().__init__("mock_target_publisher")

        # Topics
        self.declare_parameter("output_topic", "/target_3d_position")
        self.declare_parameter("dk_topic", "/carryover/dk_cmd")

        # Target initial position, in mm
        self.declare_parameter("initial_x_mm", 300.0)
        self.declare_parameter("initial_y_mm", 0.0)
        self.declare_parameter("initial_z_mm", 200.0)

        # Carry-over simulation
        self.declare_parameter("lateral_axis", "x")
        self.declare_parameter("beta_true", 0.4)

        # Noise, in mm
        self.declare_parameter("noise_std_mm", 0.0)

        # Publish rate
        self.declare_parameter("publish_rate_hz", 20.0)

        # Optional constant drift per received dk command, in mm
        self.declare_parameter("drift_x_mm_per_step", 0.0)
        self.declare_parameter("drift_y_mm_per_step", 0.0)
        self.declare_parameter("drift_z_mm_per_step", 0.0)

        self.output_topic = str(self.get_parameter("output_topic").value)
        self.dk_topic = str(self.get_parameter("dk_topic").value)

        self.position_mm: Dict[str, float] = {
            "x": float(self.get_parameter("initial_x_mm").value),
            "y": float(self.get_parameter("initial_y_mm").value),
            "z": float(self.get_parameter("initial_z_mm").value),
        }

        self.lateral_axis = str(self.get_parameter("lateral_axis").value).lower()
        self.beta_true = float(self.get_parameter("beta_true").value)
        self.noise_std_mm = float(self.get_parameter("noise_std_mm").value)

        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)

        self.drift_mm_per_step = {
            "x": float(self.get_parameter("drift_x_mm_per_step").value),
            "y": float(self.get_parameter("drift_y_mm_per_step").value),
            "z": float(self.get_parameter("drift_z_mm_per_step").value),
        }

        if self.lateral_axis not in ["x", "y", "z"]:
            raise ValueError("lateral_axis must be x, y, or z.")

        if self.publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be positive.")

        self.publisher = self.create_publisher(
            String,
            self.output_topic,
            10,
        )

        self.dk_subscriber = self.create_subscription(
            Float64,
            self.dk_topic,
            self._dk_callback,
            10,
        )

        self.timer = self.create_timer(
            1.0 / self.publish_rate_hz,
            self._timer_callback,
        )

        self.step_count = 0

        self.get_logger().info("Mock target publisher started.")
        self.get_logger().info(f"output_topic = {self.output_topic}")
        self.get_logger().info(f"dk_topic = {self.dk_topic}")
        self.get_logger().info(f"initial_position_mm = {self.position_mm}")
        self.get_logger().info(f"lateral_axis = {self.lateral_axis}")
        self.get_logger().info(f"beta_true = {self.beta_true}")
        self.get_logger().info(f"noise_std_mm = {self.noise_std_mm}")

    def _dk_callback(self, msg: Float64) -> None:
        dk_m = float(msg.data)
        dk_mm = dk_m * 1000.0

        target_shift_mm = self.beta_true * dk_mm

        self.position_mm[self.lateral_axis] += target_shift_mm

        self.position_mm["x"] += self.drift_mm_per_step["x"]
        self.position_mm["y"] += self.drift_mm_per_step["y"]
        self.position_mm["z"] += self.drift_mm_per_step["z"]

        self.step_count += 1

        self.get_logger().info(
            f"Received dk = {dk_m:.6e} m "
            f"({dk_mm:.3f} mm), "
            f"applied target shift = {target_shift_mm:.3f} mm, "
            f"new target = {self.position_mm}"
        )

    def _timer_callback(self) -> None:
        x = self.position_mm["x"]
        y = self.position_mm["y"]
        z = self.position_mm["z"]

        if self.noise_std_mm > 0.0:
            x += random.gauss(0.0, self.noise_std_mm)
            y += random.gauss(0.0, self.noise_std_mm)
            z += random.gauss(0.0, self.noise_std_mm)

        data = {
            "x": x,
            "y": y,
            "z": z,
            "unit": "mm",
            "source": "mock_target_publisher",
            "step_count": self.step_count,
        }

        msg = String()
        msg.data = json.dumps(data)

        self.publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MockTargetPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()