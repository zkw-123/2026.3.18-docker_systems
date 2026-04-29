#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import copy
import json
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float64, Int32
from geometry_msgs.msg import PoseStamped, PointStamped


def _now_ns(node: Node) -> int:
    return node.get_clock().now().nanoseconds


def _point_msg_to_dict(msg: PointStamped) -> Dict[str, float]:
    return {
        "x": float(msg.point.x),
        "y": float(msg.point.y),
        "z": float(msg.point.z),
    }


def _pose_msg_to_xyz_dict(msg: PoseStamped) -> Dict[str, float]:
    return {
        "x": float(msg.pose.position.x),
        "y": float(msg.pose.position.y),
        "z": float(msg.pose.position.z),
    }


def _axis_to_vector(axis: str) -> List[float]:
    axis = axis.lower().strip()
    if axis == "x":
        return [1.0, 0.0, 0.0]
    if axis == "y":
        return [0.0, 1.0, 0.0]
    if axis == "z":
        return [0.0, 0.0, 1.0]
    raise ValueError(f"Unsupported axis: {axis}. Use x, y, or z.")


class CarryoverValidationRunner(Node):
    """
    Open-loop carry-over validation runner.

    This node sends known lateral correction steps d_k to the robot while adding
    a small nominal axial insertion increment. It records target/robot states
    before and after each step, then publishes one JSON step event.

    Main output:
        /carryover/step_event   std_msgs/String, JSON

    Robot command output:
        /robot_command          std_msgs/String

    Expected inputs:
        /robot_current_pose     geometry_msgs/PoseStamped
        /perception/target_point geometry_msgs/PointStamped
    """

    def __init__(self) -> None:
        super().__init__("carryover_validation_runner")

        # Experiment parameters
        self.declare_parameter("num_steps", 20)
        self.declare_parameter("substrate_name", "substrate_A")

        self.declare_parameter("lateral_axis", "x")
        self.declare_parameter("axial_axis", "z")
        self.declare_parameter("axial_direction_sign", -1.0)

        self.declare_parameter("dk_sequence_mode", "alternating")
        self.declare_parameter("dk_magnitude_m", 0.0005)
        self.declare_parameter("dk_sequence_m_csv", "")

        self.declare_parameter("axial_speed_mps", 0.0005)
        self.declare_parameter("step_interval_s", 2.0)
        self.declare_parameter("settle_time_s", 0.5)
        self.declare_parameter("start_delay_s", 2.0)

        # Topics
        self.declare_parameter("robot_command_topic", "/robot_command")
        self.declare_parameter("robot_pose_topic", "/robot_current_pose")
        self.declare_parameter("target_point_topic", "/perception/target_point")

        # Command format
        self.declare_parameter("command_mode", "move")

        self.num_steps = int(self.get_parameter("num_steps").value)
        self.substrate_name = str(self.get_parameter("substrate_name").value)

        self.lateral_axis = str(self.get_parameter("lateral_axis").value).lower()
        self.axial_axis = str(self.get_parameter("axial_axis").value).lower()
        self.axial_direction_sign = float(self.get_parameter("axial_direction_sign").value)

        self.dk_sequence_mode = str(self.get_parameter("dk_sequence_mode").value).lower()
        self.dk_magnitude_m = float(self.get_parameter("dk_magnitude_m").value)
        self.dk_sequence_m_csv = str(self.get_parameter("dk_sequence_m_csv").value)

        self.axial_speed_mps = float(self.get_parameter("axial_speed_mps").value)
        self.step_interval_s = float(self.get_parameter("step_interval_s").value)
        self.settle_time_s = float(self.get_parameter("settle_time_s").value)
        self.start_delay_s = float(self.get_parameter("start_delay_s").value)

        self.robot_command_topic = str(self.get_parameter("robot_command_topic").value)
        self.robot_pose_topic = str(self.get_parameter("robot_pose_topic").value)
        self.target_point_topic = str(self.get_parameter("target_point_topic").value)

        self.command_mode = str(self.get_parameter("command_mode").value).lower()

        self.dk_sequence = self._generate_dk_sequence()

        self.latest_robot_pose: Optional[PoseStamped] = None
        self.latest_target_point: Optional[PointStamped] = None

        self.current_step = 0
        self.phase = "WAIT_START"
        self.phase_start_ns = _now_ns(self)
        self.command_sent_ns: Optional[int] = None

        self.robot_before: Optional[Dict[str, float]] = None
        self.target_before: Optional[Dict[str, float]] = None
        self.timestamp_before_ns: Optional[int] = None

        self.robot_pose_sub = self.create_subscription(
            PoseStamped,
            self.robot_pose_topic,
            self._robot_pose_callback,
            10,
        )

        self.target_point_sub = self.create_subscription(
            PointStamped,
            self.target_point_topic,
            self._target_point_callback,
            10,
        )

        self.robot_command_pub = self.create_publisher(
            String,
            self.robot_command_topic,
            10,
        )

        self.step_event_pub = self.create_publisher(
            String,
            "/carryover/step_event",
            10,
        )

        self.dk_cmd_pub = self.create_publisher(
            Float64,
            "/carryover/dk_cmd",
            10,
        )

        self.step_index_pub = self.create_publisher(
            Int32,
            "/carryover/step_index",
            10,
        )

        self.state_pub = self.create_publisher(
            String,
            "/carryover/state",
            10,
        )

        self.timer = self.create_timer(0.1, self._timer_callback)

        self.get_logger().info("Carry-over validation runner started.")
        self.get_logger().info(f"num_steps = {len(self.dk_sequence)}")
        self.get_logger().info(f"dk_sequence = {self.dk_sequence}")
        self.get_logger().info(f"robot_command_topic = {self.robot_command_topic}")
        self.get_logger().info(f"robot_pose_topic = {self.robot_pose_topic}")
        self.get_logger().info(f"target_point_topic = {self.target_point_topic}")

    def _robot_pose_callback(self, msg: PoseStamped) -> None:
        self.latest_robot_pose = msg

    def _target_point_callback(self, msg: PointStamped) -> None:
        self.latest_target_point = msg

    def _publish_state(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.state_pub.publish(msg)

    def _generate_dk_sequence(self) -> List[float]:
        if self.dk_sequence_m_csv.strip():
            values = [
                float(v.strip())
                for v in self.dk_sequence_m_csv.split(",")
                if v.strip()
            ]
            if not values:
                raise ValueError("dk_sequence_m_csv was provided but no valid values were parsed.")
            return values

        n = self.num_steps
        mag = self.dk_magnitude_m
        mode = self.dk_sequence_mode

        if mode == "alternating":
            return [mag if i % 2 == 0 else -mag for i in range(n)]
        if mode == "positive":
            return [mag for _ in range(n)]
        if mode == "negative":
            return [-mag for _ in range(n)]
        if mode == "zero":
            return [0.0 for _ in range(n)]

        raise ValueError(
            f"Unsupported dk_sequence_mode: {mode}. "
            "Use alternating, positive, negative, zero, or dk_sequence_m_csv."
        )

    def _has_required_data(self) -> bool:
        return self.latest_robot_pose is not None and self.latest_target_point is not None

    def _elapsed_s_since(self, start_ns: int) -> float:
        return float(_now_ns(self) - start_ns) * 1e-9

    def _compute_next_robot_xyz(
        self,
        robot_xyz: Dict[str, float],
        dk_m: float,
        axial_increment_m: float,
    ) -> Tuple[float, float, float]:
        new_xyz = copy.deepcopy(robot_xyz)

        if self.lateral_axis not in new_xyz:
            raise ValueError(f"Unsupported lateral_axis: {self.lateral_axis}")
        if self.axial_axis not in new_xyz:
            raise ValueError(f"Unsupported axial_axis: {self.axial_axis}")

        new_xyz[self.lateral_axis] += dk_m
        new_xyz[self.axial_axis] += self.axial_direction_sign * axial_increment_m

        return new_xyz["x"], new_xyz["y"], new_xyz["z"]

    def _publish_robot_move_command(self, x: float, y: float, z: float) -> None:
        if self.command_mode != "move":
            raise ValueError(
                f"Unsupported command_mode: {self.command_mode}. "
                "This first implementation supports command_mode='move'."
            )

        command = f"move {x:.6f} {y:.6f} {z:.6f}"

        msg = String()
        msg.data = command
        self.robot_command_pub.publish(msg)

        self.get_logger().info(f"Published robot command: {command}")

    def _timer_callback(self) -> None:
        if self.phase == "WAIT_START":
            self._publish_state("WAIT_START")
            if self._elapsed_s_since(self.phase_start_ns) >= self.start_delay_s:
                self.phase = "WAIT_DATA"
                self.phase_start_ns = _now_ns(self)
            return

        if self.phase == "WAIT_DATA":
            self._publish_state("WAIT_DATA")
            if self._has_required_data():
                self.phase = "READY"
                self.phase_start_ns = _now_ns(self)
                self.get_logger().info("Required robot pose and target point received.")
            return

        if self.phase == "DONE":
            self._publish_state("DONE")
            return

        if self.current_step >= len(self.dk_sequence):
            self.phase = "DONE"
            self._publish_state("DONE")
            self.get_logger().info("Carry-over validation completed.")
            return

        if self.phase == "READY":
            if not self._has_required_data():
                self.phase = "WAIT_DATA"
                return

            dk_m = float(self.dk_sequence[self.current_step])
            axial_increment_m = self.axial_speed_mps * self.step_interval_s

            self.robot_before = _pose_msg_to_xyz_dict(self.latest_robot_pose)
            self.target_before = _point_msg_to_dict(self.latest_target_point)
            self.timestamp_before_ns = _now_ns(self)

            x_new, y_new, z_new = self._compute_next_robot_xyz(
                self.robot_before,
                dk_m,
                axial_increment_m,
            )

            self._publish_robot_move_command(x_new, y_new, z_new)

            dk_msg = Float64()
            dk_msg.data = dk_m
            self.dk_cmd_pub.publish(dk_msg)

            step_msg = Int32()
            step_msg.data = int(self.current_step)
            self.step_index_pub.publish(step_msg)

            self.command_sent_ns = _now_ns(self)
            self.phase = "WAIT_SETTLE"
            self.phase_start_ns = _now_ns(self)

            self._publish_state("WAIT_SETTLE")
            return

        if self.phase == "WAIT_SETTLE":
            self._publish_state("WAIT_SETTLE")

            if self.command_sent_ns is None:
                self.phase = "READY"
                return

            if self._elapsed_s_since(self.command_sent_ns) < self.settle_time_s:
                return

            if not self._has_required_data():
                self.phase = "WAIT_DATA"
                return

            target_after = _point_msg_to_dict(self.latest_target_point)
            robot_after = _pose_msg_to_xyz_dict(self.latest_robot_pose)
            timestamp_after_ns = _now_ns(self)

            dk_m = float(self.dk_sequence[self.current_step])
            axial_increment_m = self.axial_speed_mps * self.step_interval_s

            event = {
                "schema_version": 1,
                "source": "carryover_validation_runner",
                "step_index": int(self.current_step),
                "substrate_name": self.substrate_name,
                "timestamp_before_ns": int(self.timestamp_before_ns),
                "timestamp_after_ns": int(timestamp_after_ns),
                "dk_m": dk_m,
                "axial_increment_m": axial_increment_m,
                "lateral_axis": self.lateral_axis,
                "axial_axis": self.axial_axis,
                "lateral_axis_vector": _axis_to_vector(self.lateral_axis),
                "robot_before": self.robot_before,
                "robot_after": robot_after,
                "target_before": self.target_before,
                "target_after": target_after,
            }

            msg = String()
            msg.data = json.dumps(event)
            self.step_event_pub.publish(msg)

            self.get_logger().info(
                f"Step {self.current_step} event published. "
                f"dk = {dk_m:.6f} m"
            )

            self.current_step += 1
            self.phase = "READY"
            self.phase_start_ns = _now_ns(self)
            self.command_sent_ns = None
            return


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CarryoverValidationRunner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()