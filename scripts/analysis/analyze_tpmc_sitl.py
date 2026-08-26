#!/usr/bin/env python3
"""Summarize PX4 ULog and TMPC telemetry from one SITL run."""

import argparse
import csv
import json
from pathlib import Path
from typing import Iterable

import numpy as np
from pyulog import ULog


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("ulog", type=Path, help="PX4 ULog from the SITL run.")
    parser.add_argument(
        "--metrics",
        type=Path,
        help="CSV written by record_tpmc_metrics.py.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="Optional JSON report path.",
    )
    return parser.parse_args()


def percent(value: np.ndarray) -> float:
    return float(np.mean(value) * 100.0) if value.size else 0.0


def active_interval_seconds(ulog: ULog) -> tuple[float, float] | None:
    status = ulog.get_dataset("vehicle_status").data
    timestamps = status["timestamp"] * 1.0e-6
    active = status["executor_in_charge"] != 0
    if not np.any(active):
        return None
    indices = np.flatnonzero(active)
    return float(timestamps[indices[0]]), float(timestamps[indices[-1]])


def within_interval(
    timestamps: np.ndarray, interval: tuple[float, float] | None
) -> np.ndarray:
    if interval is None:
        return np.ones(timestamps.shape, dtype=bool)
    return (timestamps >= interval[0]) & (timestamps <= interval[1])


def ulog_metrics(ulog_path: Path) -> dict[str, float | int | None]:
    ulog = ULog(str(ulog_path))
    interval = active_interval_seconds(ulog)

    position = ulog.get_dataset("vehicle_local_position").data
    position_time = position["timestamp"] * 1.0e-6
    position_mask = within_interval(position_time, interval)
    altitude_m = -position["z"][position_mask]

    attitude = ulog.get_dataset("vehicle_attitude").data
    attitude_time = attitude["timestamp"] * 1.0e-6
    attitude_mask = within_interval(attitude_time, interval)
    roll_quaternion = attitude["q[1]"][attitude_mask]
    pitch_quaternion = attitude["q[2]"][attitude_mask]
    body_z_world_z = 1.0 - 2.0 * (roll_quaternion**2 + pitch_quaternion**2)
    tilt_deg = np.degrees(np.arccos(np.clip(body_z_world_z, -1.0, 1.0)))

    motors = ulog.get_dataset("actuator_motors").data
    motor_time = motors["timestamp"] * 1.0e-6
    motor_mask = within_interval(motor_time, interval)
    motor_values = np.stack(
        [motors[f"control[{index}]"][motor_mask] for index in range(4)], axis=1
    )
    motor_limit = np.any((motor_values <= 1.0e-3) | (motor_values >= 0.999), axis=1)

    return {
        "external_mode_start_s": None if interval is None else interval[0],
        "external_mode_end_s": None if interval is None else interval[1],
        "external_mode_duration_s": None if interval is None else interval[1] - interval[0],
        "altitude_min_m": float(np.min(altitude_m)) if altitude_m.size else None,
        "altitude_max_m": float(np.max(altitude_m)) if altitude_m.size else None,
        "altitude_std_m": float(np.std(altitude_m)) if altitude_m.size else None,
        "tilt_mean_deg": float(np.mean(tilt_deg)) if tilt_deg.size else None,
        "tilt_max_deg": float(np.max(tilt_deg)) if tilt_deg.size else None,
        "motor_samples_at_limit_percent": percent(motor_limit),
    }


def float_column(rows: Iterable[dict[str, str]], name: str) -> np.ndarray:
    return np.asarray([float(row[name]) for row in rows], dtype=float)


def optional_float_column(
    rows: list[dict[str, str]], name: str
) -> np.ndarray | None:
    if not rows or name not in rows[0]:
        return None
    return float_column(rows, name)


def timing_metrics(rows: list[dict[str, str]]) -> dict[str, float]:
    metrics: dict[str, float] = {}
    timing_columns = (
        "preparation_time_ms",
        "acados_wall_time_ms",
        "postprocessing_time_ms",
        "acados_metadata_time_ms",
        "diagnostics_time_ms",
        "sqp_statistics_time_ms",
        "prediction_read_time_ms",
        "constraint_validation_time_ms",
        "result_finalization_time_ms",
        "postprocessing_unattributed_time_ms",
        "end_to_end_time_ms",
        "setpoint_age_ms",
        "reference_age_ms",
    )
    for column in timing_columns:
        values = optional_float_column(rows, column)
        if values is None:
            continue
        prefix = column.removesuffix("_ms")
        metrics[f"{prefix}_mean_ms"] = float(np.mean(values))
        metrics[f"{prefix}_p95_ms"] = float(np.percentile(values, 95.0))
        metrics[f"{prefix}_max_ms"] = float(np.max(values))

    postprocessing = optional_float_column(rows, "postprocessing_time_ms")
    if postprocessing is None:
        return metrics
    mean_postprocessing_ms = float(np.mean(postprocessing))
    if mean_postprocessing_ms <= 0.0:
        return metrics
    for column in (
        "acados_metadata_time_ms",
        "diagnostics_time_ms",
        "sqp_statistics_time_ms",
        "prediction_read_time_ms",
        "constraint_validation_time_ms",
        "result_finalization_time_ms",
        "postprocessing_unattributed_time_ms",
    ):
        values = optional_float_column(rows, column)
        if values is None:
            continue
        prefix = column.removesuffix("_ms")
        metrics[f"{prefix}_share_of_postprocessing_percent"] = (
            float(np.mean(values)) / mean_postprocessing_ms * 100.0
        )
    return metrics


def telemetry_metrics(metrics_path: Path) -> dict[str, float | int]:
    with metrics_path.open(newline="", encoding="utf-8") as metrics_file:
        rows = list(csv.DictReader(metrics_file))
    if not rows:
        return {"telemetry_samples": 0}

    position_error_m = float_column(rows, "position_error_m")
    altitude_error_m = float_column(rows, "altitude_error_m")
    solve_time_ms = float_column(rows, "solve_time_ms")
    valid = float_column(rows, "valid")
    recovery = float_column(rows, "recovery_active")
    deadline_missed = float_column(rows, "deadline_missed")
    metrics: dict[str, float | int] = {
        "telemetry_samples": len(rows),
        "position_error_rmse_m": float(np.sqrt(np.mean(position_error_m**2))),
        "position_error_max_m": float(np.max(np.abs(position_error_m))),
        "altitude_error_rmse_m": float(np.sqrt(np.mean(altitude_error_m**2))),
        "altitude_error_max_m": float(np.max(np.abs(altitude_error_m))),
        "solve_time_mean_ms": float(np.mean(solve_time_ms)),
        "solve_time_p95_ms": float(np.percentile(solve_time_ms, 95.0)),
        "solve_time_max_ms": float(np.max(solve_time_ms)),
        "solver_valid_percent": percent(valid != 0.0),
        "recovery_active_percent": percent(recovery != 0.0),
        "deadline_missed_percent": percent(deadline_missed != 0.0),
    }
    metrics.update(timing_metrics(rows))
    return metrics


def main() -> None:
    arguments = parse_arguments()
    report: dict[str, float | int | None] = ulog_metrics(arguments.ulog)
    if arguments.metrics:
        report.update(telemetry_metrics(arguments.metrics))

    print(json.dumps(report, indent=2, sort_keys=True))
    if arguments.output:
        arguments.output.parent.mkdir(parents=True, exist_ok=True)
        arguments.output.write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )


if __name__ == "__main__":
    main()
