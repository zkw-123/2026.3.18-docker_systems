#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
import json
import os
from datetime import datetime
from pathlib import Path
from typing import Any, Dict

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


def _get_nested(data: Dict[str, Any], path: str, default: Any = "") -> Any:
    current = data
    for key in path.split("."):
        if not isinstance(current, dict) or key not in current:
            return default
        current = current[key]
    return current


class CarryoverExperimentLogger(Node):
    """
    Step-level CSV logger for carry-over validation.

    Preferred input:
        /carryover/rls_update

    Each row corresponds to one lateral correction step.
    """

    FIELDNAMES = [
        "step_index",
        "substrate_name",
        "timestamp_before_ns",
        "timestamp_after_ns",
        "dk_m",
        "axial_increment_m",
        "lateral_axis",
        "axial_axis",

        "robot_before_x",
        "robot_before_y",
        "robot_before_z",
        "robot_after_x",
        "robot_after_y",
        "robot_after_z",

        "target_before_x",
        "target_before_y",
        "target_before_z",
        "target_after_x",
        "target_after_y",
        "target_after_z",

        "measured_delta_m",
        "beta_before",
        "beta_after",
        "P_before",
        "P_after",
        "rls_gain",
        "predicted_delta_before_m",
        "prediction_error_before_m",
        "predicted_delta_after_m",
        "prediction_error_after_m",
        "update_used",
    ]

    def __init__(self) -> None:
        super().__init__("experiment_logger")

        self.declare_parameter("output_dir", "/workspace/logs/carryover_validation")
        self.declare_parameter("file_prefix", "carryover_validation")
        self.declare_parameter("substrate_name", "substrate_A")
        self.declare_parameter("log_raw_step_event", False)

        self.output_dir = str(self.get_parameter("output_dir").value)
        self.file_prefix = str(self.get_parameter("file_prefix").value)
        self.substrate_name = str(self.get_parameter("substrate_name").value)
        self.log_raw_step_event = bool(self.get_parameter("log_raw_step_event").value)

        Path(self.output_dir).mkdir(parents=True, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{self.file_prefix}_{self.substrate_name}_{timestamp}.csv"
        self.csv_path = os.path.join(self.output_dir, filename)

        self.csv_file = open(self.csv_path, "w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.csv_file, fieldnames=self.FIELDNAMES)
        self.writer.writeheader()
        self.csv_file.flush()

        self.rls_update_sub = self.create_subscription(
            String,
            "/carryover/rls_update",
            self._rls_update_callback,
            10,
        )

        self.step_event_sub = self.create_subscription(
            String,
            "/carryover/step_event",
            self._step_event_callback,
            10,
        )

        self.get_logger().info("Carry-over experiment logger started.")
        self.get_logger().info(f"CSV output: {self.csv_path}")

    def _build_row(self, data: Dict[str, Any]) -> Dict[str, Any]:
        return {
            "step_index": data.get("step_index", ""),
            "substrate_name": data.get("substrate_name", self.substrate_name),
            "timestamp_before_ns": data.get("timestamp_before_ns", ""),
            "timestamp_after_ns": data.get("timestamp_after_ns", ""),
            "dk_m": data.get("dk_m", ""),
            "axial_increment_m": data.get("axial_increment_m", ""),
            "lateral_axis": data.get("lateral_axis", ""),
            "axial_axis": data.get("axial_axis", ""),

            "robot_before_x": _get_nested(data, "robot_before.x"),
            "robot_before_y": _get_nested(data, "robot_before.y"),
            "robot_before_z": _get_nested(data, "robot_before.z"),
            "robot_after_x": _get_nested(data, "robot_after.x"),
            "robot_after_y": _get_nested(data, "robot_after.y"),
            "robot_after_z": _get_nested(data, "robot_after.z"),

            "target_before_x": _get_nested(data, "target_before.x"),
            "target_before_y": _get_nested(data, "target_before.y"),
            "target_before_z": _get_nested(data, "target_before.z"),
            "target_after_x": _get_nested(data, "target_after.x"),
            "target_after_y": _get_nested(data, "target_after.y"),
            "target_after_z": _get_nested(data, "target_after.z"),

            "measured_delta_m": data.get("measured_delta_m", ""),
            "beta_before": data.get("beta_before", ""),
            "beta_after": data.get("beta_after", ""),
            "P_before": data.get("P_before", ""),
            "P_after": data.get("P_after", ""),
            "rls_gain": data.get("rls_gain", ""),
            "predicted_delta_before_m": data.get("predicted_delta_before_m", ""),
            "prediction_error_before_m": data.get("prediction_error_before_m", ""),
            "predicted_delta_after_m": data.get("predicted_delta_after_m", ""),
            "prediction_error_after_m": data.get("prediction_error_after_m", ""),
            "update_used": data.get("update_used", ""),
        }

    def _write_row(self, row: Dict[str, Any]) -> None:
        self.writer.writerow(row)
        self.csv_file.flush()

    def _rls_update_callback(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f"Failed to parse rls_update JSON: {exc}")
            return

        row = self._build_row(data)
        self._write_row(row)

        self.get_logger().info(
            f"Logged step {row['step_index']} | "
            f"beta_after={row['beta_after']} | "
            f"err_before={row['prediction_error_before_m']}"
        )

    def _step_event_callback(self, msg: String) -> None:
        if not self.log_raw_step_event:
            return

        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f"Failed to parse step_event JSON: {exc}")
            return

        row = self._build_row(data)
        self._write_row(row)

    def destroy_node(self) -> bool:
        try:
            if hasattr(self, "csv_file") and not self.csv_file.closed:
                self.csv_file.flush()
                self.csv_file.close()
        finally:
            return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CarryoverExperimentLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()