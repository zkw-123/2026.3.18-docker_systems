#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import csv
import math
import os
from pathlib import Path
from typing import Dict, List, Optional


def _to_float(value: str) -> Optional[float]:
    try:
        if value is None or value == "":
            return None
        return float(value)
    except ValueError:
        return None


def _mean(values: List[float]) -> float:
    if not values:
        return float("nan")
    return sum(values) / len(values)


def _rmse(values: List[float]) -> float:
    if not values:
        return float("nan")
    return math.sqrt(sum(v * v for v in values) / len(values))


def _mae(values: List[float]) -> float:
    if not values:
        return float("nan")
    return sum(abs(v) for v in values) / len(values)


def _max_abs(values: List[float]) -> float:
    if not values:
        return float("nan")
    return max(abs(v) for v in values)


def _std(values: List[float]) -> float:
    if len(values) < 2:
        return 0.0
    m = _mean(values)
    return math.sqrt(sum((v - m) ** 2 for v in values) / (len(values) - 1))


def _read_rows(input_csv: str) -> List[Dict[str, str]]:
    with open(input_csv, "r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        return list(reader)


def _find_convergence_step(
    steps: List[int],
    betas: List[float],
    window: int,
    std_threshold: float,
) -> Optional[int]:
    if len(betas) < window:
        return None

    for i in range(0, len(betas) - window + 1):
        current_window = betas[i:i + window]
        if _std(current_window) <= std_threshold:
            return steps[i + window - 1]

    return None


def analyze(
    input_csv: str,
    output_summary_csv: Optional[str],
    steady_tail_n: int,
    convergence_window: int,
    convergence_std_threshold: float,
) -> Dict[str, float]:
    rows = _read_rows(input_csv)

    steps: List[int] = []
    dk_values: List[float] = []
    measured_values: List[float] = []
    beta_after_values: List[float] = []
    error_before_values: List[float] = []
    error_after_values: List[float] = []

    substrate_name = ""

    for row in rows:
        if not substrate_name:
            substrate_name = row.get("substrate_name", "")

        step_raw = row.get("step_index", "")
        try:
            step_index = int(float(step_raw))
        except ValueError:
            step_index = len(steps)

        dk = _to_float(row.get("dk_m", ""))
        measured = _to_float(row.get("measured_delta_m", ""))
        beta_after = _to_float(row.get("beta_after", ""))
        err_before = _to_float(row.get("prediction_error_before_m", ""))
        err_after = _to_float(row.get("prediction_error_after_m", ""))

        if dk is not None:
            dk_values.append(dk)
        if measured is not None:
            measured_values.append(measured)
        if beta_after is not None:
            steps.append(step_index)
            beta_after_values.append(beta_after)
        if err_before is not None:
            error_before_values.append(err_before)
        if err_after is not None:
            error_after_values.append(err_after)

    tail = beta_after_values[-steady_tail_n:] if beta_after_values else []
    steady_state_beta = _mean(tail)
    steady_state_beta_std = _std(tail)

    convergence_step = _find_convergence_step(
        steps=steps,
        betas=beta_after_values,
        window=convergence_window,
        std_threshold=convergence_std_threshold,
    )

    summary = {
        "num_rows": float(len(rows)),
        "num_valid_beta": float(len(beta_after_values)),
        "num_valid_error_before": float(len(error_before_values)),

        "dk_mean_m": _mean(dk_values),
        "dk_mean_mm": _mean(dk_values) * 1000.0 if dk_values else float("nan"),

        "measured_delta_mean_m": _mean(measured_values),
        "measured_delta_mean_mm": _mean(measured_values) * 1000.0 if measured_values else float("nan"),

        "prediction_error_before_rmse_m": _rmse(error_before_values),
        "prediction_error_before_rmse_mm": _rmse(error_before_values) * 1000.0 if error_before_values else float("nan"),
        "prediction_error_before_mae_m": _mae(error_before_values),
        "prediction_error_before_mae_mm": _mae(error_before_values) * 1000.0 if error_before_values else float("nan"),
        "prediction_error_before_max_abs_m": _max_abs(error_before_values),
        "prediction_error_before_max_abs_mm": _max_abs(error_before_values) * 1000.0 if error_before_values else float("nan"),

        "prediction_error_after_rmse_m": _rmse(error_after_values),
        "prediction_error_after_rmse_mm": _rmse(error_after_values) * 1000.0 if error_after_values else float("nan"),

        "steady_state_beta": steady_state_beta,
        "steady_state_beta_std": steady_state_beta_std,
        "steady_tail_n": float(steady_tail_n),

        "convergence_step": float(convergence_step) if convergence_step is not None else float("nan"),
        "convergence_window": float(convergence_window),
        "convergence_std_threshold": convergence_std_threshold,
    }

    print("")
    print("========== Carry-over Validation Summary ==========")
    print(f"input_csv: {input_csv}")
    print(f"substrate_name: {substrate_name}")
    print(f"num_rows: {len(rows)}")
    print(f"steady_state_beta: {steady_state_beta:.6f}")
    print(f"steady_state_beta_std: {steady_state_beta_std:.6e}")
    print(f"prediction_error_before_rmse: {summary['prediction_error_before_rmse_mm']:.6f} mm")
    print(f"prediction_error_before_mae:  {summary['prediction_error_before_mae_mm']:.6f} mm")
    print(f"prediction_error_before_max:  {summary['prediction_error_before_max_abs_mm']:.6f} mm")

    if convergence_step is None:
        print("convergence_step: not detected")
    else:
        print(f"convergence_step: {convergence_step}")

    print("===================================================")
    print("")

    if output_summary_csv:
        Path(os.path.dirname(output_summary_csv)).mkdir(parents=True, exist_ok=True)

        with open(output_summary_csv, "w", newline="", encoding="utf-8") as f:
            fieldnames = ["metric", "value"]
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()

            writer.writerow({"metric": "input_csv", "value": input_csv})
            writer.writerow({"metric": "substrate_name", "value": substrate_name})

            for key, value in summary.items():
                writer.writerow({"metric": key, "value": value})

        print(f"Summary saved to: {output_summary_csv}")

    return summary


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Offline analyzer for carry-over validation CSV logs."
    )

    parser.add_argument(
        "--input",
        required=True,
        help="Input CSV file generated by experiment_logger.",
    )

    parser.add_argument(
        "--output",
        default="",
        help="Output summary CSV path. If omitted, only prints summary.",
    )

    parser.add_argument(
        "--steady-tail-n",
        type=int,
        default=5,
        help="Number of final beta samples used to compute steady-state beta.",
    )

    parser.add_argument(
        "--convergence-window",
        type=int,
        default=5,
        help="Window length for beta convergence detection.",
    )

    parser.add_argument(
        "--convergence-std-threshold",
        type=float,
        default=0.02,
        help="Beta std threshold for convergence detection.",
    )

    args = parser.parse_args()

    output_path = args.output.strip() if args.output.strip() else None

    analyze(
        input_csv=args.input,
        output_summary_csv=output_path,
        steady_tail_n=args.steady_tail_n,
        convergence_window=args.convergence_window,
        convergence_std_threshold=args.convergence_std_threshold,
    )


if __name__ == "__main__":
    main()