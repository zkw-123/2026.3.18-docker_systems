#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
from typing import Dict, List

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float64


def _dot(a: List[float], b: List[float]) -> float:
    return float(sum(float(x) * float(y) for x, y in zip(a, b)))


def _vec_sub(p2: Dict[str, float], p1: Dict[str, float]) -> List[float]:
    return [
        float(p2["x"]) - float(p1["x"]),
        float(p2["y"]) - float(p1["y"]),
        float(p2["z"]) - float(p1["z"]),
    ]


def _axis_to_vector(axis: str) -> List[float]:
    axis = axis.lower().strip()
    if axis == "x":
        return [1.0, 0.0, 0.0]
    if axis == "y":
        return [0.0, 1.0, 0.0]
    if axis == "z":
        return [0.0, 0.0, 1.0]
    raise ValueError(f"Unsupported axis: {axis}")


class CarryoverRLSEstimator(Node):
    """
    Online scalar RLS estimator for beta_perp.

    Model:
        measured_delta_k = beta_perp * d_k + noise

    Important:
        prediction_error_before is computed using beta before current update.
        This is the one-step prediction error and should be used for model
        predictive accuracy evaluation.
    """

    def __init__(self) -> None:
        super().__init__("rls_estimator")

        self.declare_parameter("beta0", 0.0)
        self.declare_parameter("P0", 1000.0)
        self.declare_parameter("forgetting_factor", 0.99)
        self.declare_parameter("min_abs_dk_m", 1.0e-5)
        self.declare_parameter("lateral_axis", "x")
        self.declare_parameter("enable_beta_clamp", False)
        self.declare_parameter("beta_min", -2.0)
        self.declare_parameter("beta_max", 2.0)

        self.beta = float(self.get_parameter("beta0").value)
        self.P = float(self.get_parameter("P0").value)
        self.forgetting_factor = float(self.get_parameter("forgetting_factor").value)
        self.min_abs_dk_m = float(self.get_parameter("min_abs_dk_m").value)
        self.lateral_axis = str(self.get_parameter("lateral_axis").value).lower()
        self.enable_beta_clamp = bool(self.get_parameter("enable_beta_clamp").value)
        self.beta_min = float(self.get_parameter("beta_min").value)
        self.beta_max = float(self.get_parameter("beta_max").value)

        if not (0.0 < self.forgetting_factor <= 1.0):
            raise ValueError("forgetting_factor must be in (0, 1].")

        self.step_event_sub = self.create_subscription(
            String,
            "/carryover/step_event",
            self._step_event_callback,
            10,
        )

        self.beta_pub = self.create_publisher(
            Float64,
            "/carryover/beta_perp",
            10,
        )

        self.measured_delta_pub = self.create_publisher(
            Float64,
            "/carryover/measured_delta",
            10,
        )

        self.predicted_before_pub = self.create_publisher(
            Float64,
            "/carryover/predicted_delta_before",
            10,
        )

        self.error_before_pub = self.create_publisher(
            Float64,
            "/carryover/prediction_error_before",
            10,
        )

        self.predicted_after_pub = self.create_publisher(
            Float64,
            "/carryover/predicted_delta_after",
            10,
        )

        self.error_after_pub = self.create_publisher(
            Float64,
            "/carryover/prediction_error_after",
            10,
        )

        self.rls_update_pub = self.create_publisher(
            String,
            "/carryover/rls_update",
            10,
        )

        self.get_logger().info("Carry-over RLS estimator started.")
        self.get_logger().info(
            f"beta0={self.beta}, P0={self.P}, lambda={self.forgetting_factor}"
        )

    def _publish_float(self, pub, value: float) -> None:
        msg = Float64()
        msg.data = float(value)
        pub.publish(msg)

    def _extract_measured_delta(self, event: Dict) -> float:
        target_before = event["target_before"]
        target_after = event["target_after"]

        delta_vec = _vec_sub(target_after, target_before)

        if "lateral_axis_vector" in event:
            axis_vec = [float(v) for v in event["lateral_axis_vector"]]
        else:
            axis = str(event.get("lateral_axis", self.lateral_axis))
            axis_vec = _axis_to_vector(axis)

        return _dot(delta_vec, axis_vec)

    def _apply_beta_clamp(self) -> None:
        if self.enable_beta_clamp:
            self.beta = max(self.beta_min, min(self.beta_max, self.beta))

    def _step_event_callback(self, msg: String) -> None:
        try:
            event = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f"Failed to parse step_event JSON: {exc}")
            return

        try:
            dk_m = float(event["dk_m"])
            measured_delta_m = self._extract_measured_delta(event)
        except Exception as exc:
            self.get_logger().error(f"Invalid step_event content: {exc}")
            return

        beta_before = float(self.beta)
        P_before = float(self.P)

        predicted_delta_before_m = beta_before * dk_m
        prediction_error_before_m = measured_delta_m - predicted_delta_before_m

        update_used = abs(dk_m) >= self.min_abs_dk_m
        rls_gain = 0.0

        if update_used:
            phi = dk_m
            lam = self.forgetting_factor

            denominator = lam + phi * self.P * phi
            rls_gain = self.P * phi / denominator

            self.beta = self.beta + rls_gain * prediction_error_before_m
            self._apply_beta_clamp()

            self.P = (self.P - rls_gain * phi * self.P) / lam

            if not math.isfinite(self.P) or self.P <= 0.0:
                self.get_logger().warn(
                    f"Invalid P detected ({self.P}). Resetting P to P_before."
                )
                self.P = max(P_before, 1.0)
        else:
            self.get_logger().warn(
                f"Skip RLS update because |dk|={abs(dk_m):.6e} < min_abs_dk_m."
            )

        beta_after = float(self.beta)
        P_after = float(self.P)

        predicted_delta_after_m = beta_after * dk_m
        prediction_error_after_m = measured_delta_m - predicted_delta_after_m

        self._publish_float(self.beta_pub, beta_after)
        self._publish_float(self.measured_delta_pub, measured_delta_m)
        self._publish_float(self.predicted_before_pub, predicted_delta_before_m)
        self._publish_float(self.error_before_pub, prediction_error_before_m)
        self._publish_float(self.predicted_after_pub, predicted_delta_after_m)
        self._publish_float(self.error_after_pub, prediction_error_after_m)

        update = dict(event)
        update.update(
            {
                "source": "carryover_rls_estimator",
                "measured_delta_m": measured_delta_m,
                "beta_before": beta_before,
                "beta_after": beta_after,
                "P_before": P_before,
                "P_after": P_after,
                "rls_gain": float(rls_gain),
                "predicted_delta_before_m": predicted_delta_before_m,
                "prediction_error_before_m": prediction_error_before_m,
                "predicted_delta_after_m": predicted_delta_after_m,
                "prediction_error_after_m": prediction_error_after_m,
                "update_used": bool(update_used),
            }
        )

        out = String()
        out.data = json.dumps(update)
        self.rls_update_pub.publish(out)

        step_index = event.get("step_index", "unknown")
        self.get_logger().info(
            f"step={step_index}, dk={dk_m:.6e}, "
            f"measured={measured_delta_m:.6e}, "
            f"beta={beta_after:.6f}, "
            f"err_before={prediction_error_before_m:.6e}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CarryoverRLSEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()