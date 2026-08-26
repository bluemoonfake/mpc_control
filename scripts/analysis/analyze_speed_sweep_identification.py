#!/usr/bin/env python3
"""Estimate horizontal drag and collective thrust gain from TMPC speed-sweep telemetry."""

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np


REQUIRED_COLUMNS = (
    "timestamp_seconds",
    "valid",
    "recovery_active",
    "deadline_missed",
    "applied_command_available",
    "tilt_rad",
    "measured_vx_m_s",
    "measured_vy_m_s",
    "measured_vz_m_s",
    "measured_collective_specific_force_m_s2",
    "applied_roll_rad",
    "applied_pitch_rad",
    "applied_yaw_rad",
    "applied_collective_specific_force_m_s2",
)


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="CSV from record_tpmc_metrics.py")
    parser.add_argument("--output", type=Path, help="JSON report destination")
    parser.add_argument("--maximum-speed-m-s", type=float, default=18.0)
    parser.add_argument("--speed-bin-width-m-s", type=float, default=3.0)
    parser.add_argument("--minimum-samples-per-bin", type=int, default=25)
    parser.add_argument("--maximum-tilt-deg", type=float, default=35.0)
    parser.add_argument("--maximum-rmse-m-s2", type=float, default=1.5)
    return parser.parse_args()


def finite_float(row: dict[str, str], name: str) -> float:
    try:
        value = float(row[name])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f"invalid {name}") from error
    if not math.isfinite(value):
        raise ValueError(f"non-finite {name}")
    return value


def load_rows(path: Path, maximum_tilt_rad: float) -> dict[str, np.ndarray]:
    with path.open(encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None:
            raise ValueError("telemetry CSV has no header")
        missing = [name for name in REQUIRED_COLUMNS if name not in reader.fieldnames]
        if missing:
            raise ValueError(
                "telemetry CSV is missing speed-sweep fields: " + ", ".join(missing)
            )
        selected = []
        for row in reader:
            try:
                valid = int(row["valid"]) != 0
                recovered = int(row["recovery_active"]) != 0
                deadline_missed = int(row["deadline_missed"]) != 0
                applied = int(row["applied_command_available"]) != 0
                tilt = finite_float(row, "tilt_rad")
                if not valid or recovered or deadline_missed or not applied:
                    continue
                if tilt > maximum_tilt_rad:
                    continue
                selected.append(
                    [finite_float(row, name) for name in REQUIRED_COLUMNS
                     if name not in {"valid", "recovery_active", "deadline_missed",
                                     "applied_command_available", "tilt_rad"}]
                )
            except ValueError:
                continue

    if len(selected) < 40:
        raise ValueError("fewer than 40 valid, applied speed-sweep samples")
    values = np.asarray(selected, dtype=float)
    order = np.argsort(values[:, 0])
    values = values[order]
    unique = np.concatenate(([True], np.diff(values[:, 0]) > 1.0e-6))
    values = values[unique]
    if len(values) < 40 or np.any(np.diff(values[:, 0]) <= 0.0):
        raise ValueError("timestamps are not suitable for acceleration estimation")

    return {
        "time": values[:, 0],
        "velocity": values[:, 1:4],
        "measured_collective": values[:, 4],
        "roll": values[:, 5],
        "pitch": values[:, 6],
        "yaw": values[:, 7],
        "applied_collective": values[:, 8],
    }


def body_z_world(roll: np.ndarray, pitch: np.ndarray, yaw: np.ndarray) -> np.ndarray:
    return np.column_stack((
        np.cos(yaw) * np.sin(pitch) * np.cos(roll) + np.sin(yaw) * np.sin(roll),
        np.sin(yaw) * np.sin(pitch) * np.cos(roll) - np.cos(yaw) * np.sin(roll),
        np.cos(pitch) * np.cos(roll),
    ))


def robust_fit(features: np.ndarray, target: np.ndarray) -> tuple[np.ndarray, float, float]:
    coefficients, *_ = np.linalg.lstsq(features, target, rcond=None)
    for _ in range(12):
        residual = target - features @ coefficients
        scale = max(1.0e-6, 1.4826 * np.median(np.abs(residual)))
        weights = np.minimum(1.0, 1.345 * scale / np.maximum(np.abs(residual), 1.0e-9))
        weighted_features = features * np.sqrt(weights)[:, None]
        weighted_target = target * np.sqrt(weights)
        updated, *_ = np.linalg.lstsq(weighted_features, weighted_target, rcond=None)
        if np.max(np.abs(updated - coefficients)) < 1.0e-9:
            coefficients = updated
            break
        coefficients = updated
    residual = target - features @ coefficients
    rmse = float(np.sqrt(np.mean(residual * residual)))
    condition = float(np.linalg.cond(features))
    return coefficients, rmse, condition


def coverage(speed: np.ndarray, maximum_speed: float, width: float) -> list[dict[str, float | int]]:
    bins = []
    lower = 0.0
    while lower < maximum_speed - 1.0e-9:
        upper = min(maximum_speed, lower + width)
        count = int(np.count_nonzero((speed >= lower) & (speed < upper)))
        bins.append({"lower_m_s": lower, "upper_m_s": upper, "samples": count})
        lower = upper
    return bins


def analyze(samples: dict[str, np.ndarray], arguments: argparse.Namespace) -> dict[str, object]:
    time = samples["time"]
    velocity = samples["velocity"]
    acceleration = np.column_stack(
        tuple(np.gradient(velocity[:, axis], time) for axis in range(3))
    )
    speed = np.linalg.norm(velocity[:, :2], axis=1)
    force = body_z_world(samples["roll"], samples["pitch"], samples["yaw"])
    force *= samples["measured_collective"][:, None]

    drag_target = np.concatenate((force[:, 0] - acceleration[:, 0],
                                  force[:, 1] - acceleration[:, 1]))
    drag_features = np.vstack((
        np.column_stack((velocity[:, 0], speed * velocity[:, 0])),
        np.column_stack((velocity[:, 1], speed * velocity[:, 1])),
    ))
    drag_coefficients, drag_rmse, drag_condition = robust_fit(drag_features, drag_target)

    thrust_features = np.column_stack((samples["applied_collective"],
                                        np.ones_like(speed)))
    thrust_coefficients, thrust_rmse, thrust_condition = robust_fit(
        thrust_features, samples["measured_collective"]
    )
    bins = coverage(speed, arguments.maximum_speed_m_s, arguments.speed_bin_width_m_s)
    coverage_complete = all(
        entry["samples"] >= arguments.minimum_samples_per_bin for entry in bins
    ) and float(np.max(speed)) >= arguments.maximum_speed_m_s - 0.5
    accepted = (
        coverage_complete
        and drag_coefficients[0] >= 0.0
        and drag_coefficients[1] >= 0.0
        and drag_rmse <= arguments.maximum_rmse_m_s2
        and drag_condition < 1.0e4
        and 0.5 <= thrust_coefficients[0] <= 1.5
        and thrust_condition < 1.0e4
    )
    return {
        "accepted_for_model_update": bool(accepted),
        "rejection_reasons": [
            reason for reason, condition in (
                ("speed_coverage_incomplete", not coverage_complete),
                ("negative_drag_coefficient", np.any(drag_coefficients < 0.0)),
                ("drag_rmse_too_high", drag_rmse > arguments.maximum_rmse_m_s2),
                ("drag_regression_ill_conditioned", drag_condition >= 1.0e4),
                ("thrust_gain_outside_sanity_range",
                 not 0.5 <= thrust_coefficients[0] <= 1.5),
                ("thrust_regression_ill_conditioned", thrust_condition >= 1.0e4),
            ) if condition
        ],
        "samples": int(len(time)),
        "speed_range_m_s": {"minimum": float(np.min(speed)), "maximum": float(np.max(speed))},
        "speed_bins": bins,
        "drag_specific_force_model": {
            "equation": "a_xy = collective*b3_xy - d1*v_xy - d2*norm(v_xy)*v_xy",
            "linear_drag_s_inverse": float(drag_coefficients[0]),
            "quadratic_drag_m_inverse": float(drag_coefficients[1]),
            "rmse_m_s2": drag_rmse,
            "condition_number": drag_condition,
        },
        "thrust_static_model": {
            "equation": "measured_collective = gain*applied_collective + bias",
            "gain": float(thrust_coefficients[0]),
            "bias_m_s2": float(thrust_coefficients[1]),
            "rmse_m_s2": thrust_rmse,
            "condition_number": thrust_condition,
        },
    }


def main() -> None:
    arguments = parse_arguments()
    if arguments.maximum_speed_m_s <= 0.0 or arguments.speed_bin_width_m_s <= 0.0:
        raise SystemExit("speed limits must be positive")
    maximum_tilt_rad = math.radians(arguments.maximum_tilt_deg)
    report = analyze(load_rows(arguments.input, maximum_tilt_rad), arguments)
    formatted = json.dumps(report, indent=2, sort_keys=True)
    if arguments.output:
        arguments.output.parent.mkdir(parents=True, exist_ok=True)
        arguments.output.write_text(formatted + "\n", encoding="utf-8")
    print(formatted)


if __name__ == "__main__":
    main()
