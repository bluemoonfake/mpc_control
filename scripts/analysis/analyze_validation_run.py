#!/usr/bin/env python3
"""Analyze a TMPC validation run and produce a structured pass/reject report.

Usage:
    python3 scripts/analyze_validation_run.py /tmp/mpc_controller_sim/tpmc_metrics.csv \
        --mission config/missions/benchmark_obstacle_slalom.json \
        --output /tmp/mpc_controller_sim/validation_report.json

Obstacle clearance is optional because waypoint-only missions do not retain
the obstacle map used to plan their collision-free path. When present,
geometry is read from ``mission.validation``. The supported shapes
are ``cylinder`` (``x``, ``y``, ``radiusM``) and ``box`` (``x``, ``y``,
``halfExtentXM``, ``halfExtentYM``). ``minimumObstacleClearanceM`` is the
required point-to-obstacle clearance. Optional ``zMinM`` and ``zMaxM`` limit
an obstacle to a vertical range. The vehicle radius must be included in the
configured minimum clearance.
"""

import argparse
import csv
import json
import math
import sys
from pathlib import Path
from dataclasses import asdict, dataclass

import numpy as np


REQUIRED_COLUMNS = (
    "timestamp_seconds",
    "valid",
    "solver_status",
    "recovery_active",
    "deadline_missed",
    "measured_x",
    "measured_y",
    "measured_z",
    "reference_x",
    "reference_y",
    "reference_z",
    "measured_roll_rad",
    "measured_pitch_rad",
    "tilt_rad",
    "control_roll_rad",
    "control_pitch_rad",
    "solve_time_ms",
    "acados_wall_time_ms",
    "end_to_end_time_ms",
    "setpoint_age_ms",
)

# Pass criteria thresholds
MAX_ALTITUDE_DEVIATION_M = 2.0
MAX_DEADLINE_MISS_RATE = 0.01
MAX_HOVER_TILT_STD_DEG = 5.0
MAX_MISSION_TRACKING_RMSE_XY_M = 1.0
MAX_MISSION_TRACKING_ERROR_XY_M = 3.0
MAX_MISSION_VELOCITY_RMSE_M_S = 1.0
MAX_MISSION_VELOCITY_ERROR_M_S = 3.0
MAXIMUM_TILT_DEG = 40.0
MAXIMUM_MOTOR_SATURATION_RATE = 0.0
MAXIMUM_MOTOR_TELEMETRY_AGE_MS = 100.0
CONTROL_DEADLINE_MS = 18.0
MOTOR_LOWER_SATURATION_THRESHOLD = 0.02
MOTOR_UPPER_SATURATION_THRESHOLD = 0.98


@dataclass(frozen=True)
class ValidationCriteria:
    maximum_altitude_deviation_m: float
    maximum_deadline_miss_rate: float
    maximum_hover_tilt_std_deg: float
    maximum_mission_tracking_rmse_xy_m: float
    maximum_mission_tracking_error_xy_m: float
    maximum_mission_velocity_rmse_m_s: float
    maximum_mission_velocity_error_m_s: float
    maximum_tilt_deg: float
    maximum_motor_saturation_rate: float
    maximum_motor_telemetry_age_ms: float
    maximum_solve_time_ms: float
    maximum_end_to_end_time_ms: float
    minimum_obstacle_clearance_m: float | None


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="CSV from record_tpmc_metrics.py")
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="JSON report destination (default: stdout)",
    )
    parser.add_argument(
        "--mission",
        type=Path,
        default=None,
        help="Mission JSON used for the run; obstacle geometry is optional.",
    )
    parser.add_argument(
        "--max-xy-rmse-m",
        type=float,
        default=MAX_MISSION_TRACKING_RMSE_XY_M,
    )
    parser.add_argument(
        "--max-xy-error-m",
        type=float,
        default=MAX_MISSION_TRACKING_ERROR_XY_M,
    )
    parser.add_argument(
        "--max-velocity-rmse-m-s",
        type=float,
        default=MAX_MISSION_VELOCITY_RMSE_M_S,
    )
    parser.add_argument(
        "--max-velocity-error-m-s",
        type=float,
        default=MAX_MISSION_VELOCITY_ERROR_M_S,
    )
    parser.add_argument(
        "--max-tilt-deg", type=float, default=MAXIMUM_TILT_DEG
    )
    parser.add_argument(
        "--max-motor-saturation-rate",
        type=float,
        default=MAXIMUM_MOTOR_SATURATION_RATE,
        help="Maximum permitted fraction of fresh mission samples with a motor at <=2%% or >=98%%.",
    )
    parser.add_argument(
        "--max-motor-telemetry-age-ms",
        type=float,
        default=MAXIMUM_MOTOR_TELEMETRY_AGE_MS,
    )
    parser.add_argument(
        "--control-deadline-ms", type=float, default=CONTROL_DEADLINE_MS
    )
    parser.add_argument(
        "--minimum-obstacle-clearance-m",
        type=float,
        default=None,
        help="Override mission.validation.minimumObstacleClearanceM.",
    )
    return parser.parse_args()


def finite_float(row: dict, name: str) -> float:
    try:
        value = float(row[name])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f"invalid {name}") from error
    if not math.isfinite(value):
        raise ValueError(f"non-finite {name}")
    return value


def safe_int(row: dict, name: str) -> int:
    try:
        return int(row[name])
    except (KeyError, TypeError, ValueError):
        return 0


def optional_float(row: dict, name: str) -> float | None:
    if name not in row or row[name] == "":
        return None
    try:
        value = float(row[name])
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def load_samples(path: Path) -> list[dict]:
    """Load and validate CSV rows."""
    samples = []
    with open(path, newline="") as csvfile:
        reader = csv.DictReader(csvfile)
        if reader.fieldnames is None:
            raise ValueError("empty CSV")
        missing = set(REQUIRED_COLUMNS) - set(reader.fieldnames)
        if missing:
            raise ValueError(f"missing columns: {sorted(missing)}")
        for row_number, row in enumerate(reader, start=2):
            try:
                sample = {
                    "t": finite_float(row, "timestamp_seconds"),
                    "valid": safe_int(row, "valid"),
                    "solver_status": safe_int(row, "solver_status"),
                    "recovery": safe_int(row, "recovery_active"),
                    "deadline_missed": safe_int(row, "deadline_missed"),
                    "mx": finite_float(row, "measured_x"),
                    "my": finite_float(row, "measured_y"),
                    "mz": finite_float(row, "measured_z"),
                    "rx": finite_float(row, "reference_x"),
                    "ry": finite_float(row, "reference_y"),
                    "rz": finite_float(row, "reference_z"),
                    "roll_meas": finite_float(row, "measured_roll_rad"),
                    "pitch_meas": finite_float(row, "measured_pitch_rad"),
                    "tilt": finite_float(row, "tilt_rad"),
                    "roll_cmd": finite_float(row, "control_roll_rad"),
                    "pitch_cmd": finite_float(row, "control_pitch_rad"),
                    "solve_ms": finite_float(row, "solve_time_ms"),
                    "acados_ms": finite_float(row, "acados_wall_time_ms"),
                    "e2e_ms": finite_float(row, "end_to_end_time_ms"),
                    "age_ms": finite_float(row, "setpoint_age_ms"),
                    "mvx": optional_float(row, "measured_vx_m_s"),
                    "mvy": optional_float(row, "measured_vy_m_s"),
                    "mvz": optional_float(row, "measured_vz_m_s"),
                    "rvx": optional_float(row, "reference_vx_m_s"),
                    "rvy": optional_float(row, "reference_vy_m_s"),
                    "rvz": optional_float(row, "reference_vz_m_s"),
                    "motor_available": safe_int(row, "motor_telemetry_available"),
                    "motor_age_ms": optional_float(row, "motor_telemetry_age_ms"),
                    "motor_min": optional_float(row, "motor_min_normalized"),
                    "motor_max": optional_float(row, "motor_max_normalized"),
                }
                samples.append(sample)
            except ValueError:
                continue
    return samples


def classify_phases(samples: list[dict]) -> dict[str, list[dict]]:
    """Split samples into hover and mission phases."""
    hover = []
    mission = []
    for s in samples:
        if abs(s["rx"]) < 1.0 and abs(s["ry"]) < 1.0 and 8.0 < s["rz"] < 11.0:
            hover.append(s)
        elif s["mz"] > 2.0:
            mission.append(s)
    return {"hover": hover, "mission": mission}


def attitude_stats(samples: list[dict], key_roll: str, key_pitch: str) -> dict:
    if not samples:
        return {}
    rolls = np.array([s[key_roll] for s in samples]) * 180.0 / np.pi
    pitches = np.array([s[key_pitch] for s in samples]) * 180.0 / np.pi
    tilts = np.array([s["tilt"] for s in samples]) * 180.0 / np.pi
    return {
        "roll_mean_deg": float(np.mean(rolls)),
        "roll_std_deg": float(np.std(rolls)),
        "roll_range_deg": [float(np.min(rolls)), float(np.max(rolls))],
        "pitch_mean_deg": float(np.mean(pitches)),
        "pitch_std_deg": float(np.std(pitches)),
        "pitch_range_deg": [float(np.min(pitches)), float(np.max(pitches))],
        "tilt_max_deg": float(np.max(tilts)),
        "tilt_p95_deg": float(np.percentile(tilts, 95)),
        "tilt_above_30_pct": float(np.mean(tilts > 30.0) * 100.0),
        "tilt_above_35_pct": float(np.mean(tilts > 35.0) * 100.0),
        "tilt_above_40_pct": float(np.mean(tilts > 40.0) * 100.0),
        "tilt_above_45_pct": float(np.mean(tilts > 45.0) * 100.0),
    }


def tracking_stats(samples: list[dict]) -> dict:
    if not samples:
        return {}
    err_xy = np.array(
        [
            math.sqrt((s["mx"] - s["rx"]) ** 2 + (s["my"] - s["ry"]) ** 2)
            for s in samples
        ]
    )
    err_z = np.array([abs(s["mz"] - s["rz"]) for s in samples])
    return {
        "xy_rmse_m": float(np.sqrt(np.mean(err_xy**2))),
        "xy_max_m": float(np.max(err_xy)),
        "xy_mean_m": float(np.mean(err_xy)),
        "z_rmse_m": float(np.sqrt(np.mean(err_z**2))),
        "z_max_m": float(np.max(err_z)),
        "z_mean_m": float(np.mean(err_z)),
    }


def velocity_tracking_stats(samples: list[dict]) -> dict:
    valid_samples = [
        sample
        for sample in samples
        if all(
            sample[key] is not None
            for key in ("mvx", "mvy", "mvz", "rvx", "rvy", "rvz")
        )
    ]
    if not valid_samples:
        return {"available": False, "sample_count": 0}

    horizontal_errors = np.array(
        [
            math.hypot(sample["mvx"] - sample["rvx"], sample["mvy"] - sample["rvy"])
            for sample in valid_samples
        ]
    )
    three_dimensional_errors = np.array(
        [
            math.sqrt(
                (sample["mvx"] - sample["rvx"]) ** 2
                + (sample["mvy"] - sample["rvy"]) ** 2
                + (sample["mvz"] - sample["rvz"]) ** 2
            )
            for sample in valid_samples
        ]
    )
    return {
        "available": True,
        "sample_count": len(valid_samples),
        "horizontal_rmse_m_s": float(np.sqrt(np.mean(horizontal_errors**2))),
        "horizontal_max_m_s": float(np.max(horizontal_errors)),
        "three_dimensional_rmse_m_s": float(
            np.sqrt(np.mean(three_dimensional_errors**2))
        ),
        "three_dimensional_max_m_s": float(np.max(three_dimensional_errors)),
    }


def timing_stats(samples: list[dict]) -> dict:
    if not samples:
        return {}
    solve = np.array([s["solve_ms"] for s in samples])
    acados = np.array([s["acados_ms"] for s in samples])
    e2e = np.array([s["e2e_ms"] for s in samples])
    age = np.array([s["age_ms"] for s in samples])
    return {
        "solve_mean_ms": float(np.mean(solve)),
        "solve_p95_ms": float(np.percentile(solve, 95)),
        "solve_max_ms": float(np.max(solve)),
        "acados_mean_ms": float(np.mean(acados)),
        "acados_p95_ms": float(np.percentile(acados, 95)),
        "e2e_mean_ms": float(np.mean(e2e)),
        "e2e_p95_ms": float(np.percentile(e2e, 95)),
        "e2e_max_ms": float(np.max(e2e)),
        "age_mean_ms": float(np.mean(age)),
        "age_p95_ms": float(np.percentile(age, 95)),
    }


def motor_saturation_stats(samples: list[dict], maximum_age_ms: float) -> dict:
    fresh_samples = [
        sample
        for sample in samples
        if sample["motor_available"] == 1
        and sample["motor_age_ms"] is not None
        and sample["motor_age_ms"] <= maximum_age_ms
        and sample["motor_min"] is not None
        and sample["motor_max"] is not None
    ]
    if not fresh_samples:
        return {
            "available": False,
            "fresh_sample_count": 0,
            "saturation_count": 0,
            "saturation_rate": None,
        }

    saturated = [
        sample
        for sample in fresh_samples
        if sample["motor_min"] <= MOTOR_LOWER_SATURATION_THRESHOLD
        or sample["motor_max"] >= MOTOR_UPPER_SATURATION_THRESHOLD
    ]
    return {
        "available": True,
        "fresh_sample_count": len(fresh_samples),
        "saturation_count": len(saturated),
        "saturation_rate": len(saturated) / len(fresh_samples),
        "minimum_output": min(sample["motor_min"] for sample in fresh_samples),
        "maximum_output": max(sample["motor_max"] for sample in fresh_samples),
    }


def validation_section(mission_path: Path | None) -> tuple[dict | None, str | None]:
    if mission_path is None:
        return None, "mission file was not provided"
    try:
        root = json.loads(mission_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        return None, f"cannot read mission file: {error}"

    mission = root.get("mission")
    if not isinstance(mission, dict):
        return None, "mission JSON does not contain a mission object"
    validation = mission.get("validation")
    if not isinstance(validation, dict):
        return None, "mission.validation is not configured"
    return validation, None


def finite_validation_number(value: object) -> float | None:
    if isinstance(value, bool):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def obstacle_clearance_stats(
    samples: list[dict], mission_path: Path | None, minimum_clearance_override_m: float | None
) -> dict:
    validation, error = validation_section(mission_path)
    if error is not None:
        return {
            "available": False,
            "applicable": False,
            "reason": error,
        }

    obstacles = validation.get("obstacles")
    configured_minimum = (
        minimum_clearance_override_m
        if minimum_clearance_override_m is not None
        else finite_validation_number(validation.get("minimumObstacleClearanceM"))
    )
    if configured_minimum is None or configured_minimum < 0.0:
        return {
            "available": False,
            "applicable": True,
            "reason": "minimumObstacleClearanceM is missing or invalid",
        }
    if not isinstance(obstacles, list) or not obstacles:
        return {
            "available": False,
            "applicable": True,
            "reason": "mission.validation.obstacles is empty",
        }
    for index, obstacle in enumerate(obstacles):
        error = obstacle_validation_error(obstacle)
        if error is not None:
            return {
                "available": False,
                "applicable": True,
                "reason": f"obstacle {index}: {error}",
            }

    nearest_clearance = math.inf
    closest_obstacle_id = None
    evaluated_samples = 0
    for sample in samples:
        for index, obstacle in enumerate(obstacles):
            clearance = obstacle_clearance(sample, obstacle)
            if clearance is None:
                continue
            evaluated_samples += 1
            if clearance < nearest_clearance:
                nearest_clearance = clearance
                closest_obstacle_id = obstacle.get("id", f"obstacle_{index}")

    if evaluated_samples == 0:
        return {
            "available": False,
            "applicable": True,
            "reason": "no mission sample intersects obstacle altitude",
        }
    return {
        "available": True,
        "applicable": True,
        "configured_minimum_clearance_m": configured_minimum,
        "evaluated_sample_count": evaluated_samples,
        "minimum_clearance_m": nearest_clearance,
        "closest_obstacle_id": closest_obstacle_id,
    }


def obstacle_validation_error(obstacle: object) -> str | None:
    if not isinstance(obstacle, dict):
        return "must be an object"

    obstacle_type = obstacle.get("type", "cylinder")
    if obstacle_type not in {"cylinder", "box"}:
        return f"unsupported type {obstacle_type!r}"
    if finite_validation_number(obstacle.get("x")) is None:
        return "x must be finite"
    if finite_validation_number(obstacle.get("y")) is None:
        return "y must be finite"
    if "zMinM" in obstacle and finite_validation_number(obstacle["zMinM"]) is None:
        return "zMinM must be finite"
    if "zMaxM" in obstacle and finite_validation_number(obstacle["zMaxM"]) is None:
        return "zMaxM must be finite"
    if "zMinM" in obstacle and "zMaxM" in obstacle:
        z_min = finite_validation_number(obstacle["zMinM"])
        z_max = finite_validation_number(obstacle["zMaxM"])
        if z_min is not None and z_max is not None and z_min > z_max:
            return "zMinM must not exceed zMaxM"

    if obstacle_type == "cylinder":
        radius = finite_validation_number(obstacle.get("radiusM"))
        return None if radius is not None and radius > 0.0 else "radiusM must be positive"

    half_x = finite_validation_number(obstacle.get("halfExtentXM"))
    half_y = finite_validation_number(obstacle.get("halfExtentYM"))
    if half_x is None or half_x <= 0.0:
        return "halfExtentXM must be positive"
    if half_y is None or half_y <= 0.0:
        return "halfExtentYM must be positive"
    return None


def obstacle_clearance(sample: dict, obstacle: dict) -> float | None:
    z_min = -math.inf
    z_max = math.inf
    if "zMinM" in obstacle:
        z_min = finite_validation_number(obstacle["zMinM"])
    if "zMaxM" in obstacle:
        z_max = finite_validation_number(obstacle["zMaxM"])
    if z_min is None or z_max is None or z_min > z_max:
        return None
    if sample["mz"] < z_min or sample["mz"] > z_max:
        return None

    x = finite_validation_number(obstacle.get("x"))
    y = finite_validation_number(obstacle.get("y"))
    if x is None or y is None:
        return None

    obstacle_type = obstacle.get("type", "cylinder")
    if obstacle_type == "cylinder":
        radius = finite_validation_number(obstacle.get("radiusM"))
        if radius is None or radius <= 0.0:
            return None
        return math.hypot(sample["mx"] - x, sample["my"] - y) - radius
    if obstacle_type == "box":
        half_x = finite_validation_number(obstacle.get("halfExtentXM"))
        half_y = finite_validation_number(obstacle.get("halfExtentYM"))
        if half_x is None or half_y is None or half_x <= 0.0 or half_y <= 0.0:
            return None
        dx = abs(sample["mx"] - x) - half_x
        dy = abs(sample["my"] - y) - half_y
        return math.hypot(max(dx, 0.0), max(dy, 0.0)) + min(max(dx, dy), 0.0)
    return None


def safety_events(samples: list[dict]) -> dict:
    """Count adapter clipping, QP fallback, and deadline misses."""
    # Handover samples (during initial 1s mode transfer) are expected to have
    # recovery active by design while settling collective force.
    # Operational samples are when the controller has completed handover.
    operational_samples = [s for s in samples if s["valid"] == 1]
    
    qp_fallback_count = sum(
        1 for s in operational_samples if s["recovery"] == 1 and s["solver_status"] != 1
    )
    deadline_miss_count = sum(1 for s in operational_samples if s["deadline_missed"] == 1)
    
    # Active operational solver failures (excluding benign MINSTEP=3 and pre-handover=1)
    solver_failure_count = sum(
        1 for s in operational_samples
        if s["solver_status"] != 0 and s["solver_status"] != 3 and s["solver_status"] != 1
    )
    handover_hold_count = sum(1 for s in samples if s["solver_status"] == 1)
    minstep_count = sum(1 for s in samples if s["solver_status"] == 3)
    return {
        "total_samples": len(samples),
        "operational_samples": len(operational_samples),
        "handover_hold_samples": handover_hold_count,
        "qp_fallback_count": qp_fallback_count,
        "deadline_miss_count": deadline_miss_count,
        "deadline_miss_rate": (
            deadline_miss_count / len(operational_samples) if operational_samples else 0.0
        ),
        "solver_failure_count": solver_failure_count,
        "solver_minstep_count": minstep_count,
    }


def unavailable_verdict(reason: str) -> dict:
    return {"pass": False, "available": False, "reason": reason}


def evaluate_pass_criteria(report: dict, criteria: ValidationCriteria) -> dict:
    """Evaluate pass/reject criteria and return verdicts."""
    verdicts = {}

    # 1. No QP fallback
    qp_fb = report["safety"]["qp_fallback_count"]
    verdicts["no_qp_fallback"] = {
        "pass": qp_fb == 0,
        "value": qp_fb,
        "threshold": 0,
    }

    # 2. Altitude deviation
    z_max = max(
        report.get("hover_tracking", {}).get("z_max_m", 0.0),
        report.get("mission_tracking", {}).get("z_max_m", 0.0),
    )
    verdicts["altitude_deviation"] = {
        "pass": z_max < criteria.maximum_altitude_deviation_m,
        "value_m": z_max,
        "threshold_m": criteria.maximum_altitude_deviation_m,
    }

    # 3. Deadline miss rate
    dm_rate = report["safety"]["deadline_miss_rate"]
    verdicts["deadline_miss_rate"] = {
        "pass": dm_rate < criteria.maximum_deadline_miss_rate,
        "value": dm_rate,
        "threshold": criteria.maximum_deadline_miss_rate,
    }

    # 4. Hover tilt stability
    hover_roll_std = report.get("hover_attitude", {}).get("roll_std_deg", 999.0)
    hover_pitch_std = report.get("hover_attitude", {}).get("pitch_std_deg", 999.0)
    verdicts["hover_tilt_stability"] = {
        "pass": hover_roll_std < criteria.maximum_hover_tilt_std_deg
        and hover_pitch_std < criteria.maximum_hover_tilt_std_deg,
        "roll_std_deg": hover_roll_std,
        "pitch_std_deg": hover_pitch_std,
        "threshold_deg": criteria.maximum_hover_tilt_std_deg,
    }

    # 5. No solver failures (excluding MINSTEP)
    sf = report["safety"]["solver_failure_count"]
    verdicts["no_solver_failure"] = {
        "pass": sf == 0,
        "value": sf,
        "threshold": 0,
    }

    mission_tracking = report.get("mission_tracking", {})
    xy_rmse = mission_tracking.get("xy_rmse_m")
    xy_max = mission_tracking.get("xy_max_m")
    if xy_rmse is None or xy_max is None:
        verdicts["mission_xy_tracking"] = unavailable_verdict(
            "mission position samples are unavailable"
        )
    else:
        verdicts["mission_xy_tracking"] = {
            "pass": (
                xy_rmse <= criteria.maximum_mission_tracking_rmse_xy_m
                and xy_max <= criteria.maximum_mission_tracking_error_xy_m
            ),
            "xy_rmse_m": xy_rmse,
            "xy_max_m": xy_max,
            "maximum_rmse_m": criteria.maximum_mission_tracking_rmse_xy_m,
            "maximum_error_m": criteria.maximum_mission_tracking_error_xy_m,
        }

    velocity_tracking = report.get("mission_velocity_tracking", {})
    if not velocity_tracking.get("available", False):
        verdicts["mission_velocity_tracking"] = unavailable_verdict(
            "velocity telemetry is unavailable"
        )
    else:
        velocity_rmse = velocity_tracking["three_dimensional_rmse_m_s"]
        velocity_max = velocity_tracking["three_dimensional_max_m_s"]
        verdicts["mission_velocity_tracking"] = {
            "pass": (
                velocity_rmse <= criteria.maximum_mission_velocity_rmse_m_s
                and velocity_max <= criteria.maximum_mission_velocity_error_m_s
            ),
            "three_dimensional_rmse_m_s": velocity_rmse,
            "three_dimensional_max_m_s": velocity_max,
            "maximum_rmse_m_s": criteria.maximum_mission_velocity_rmse_m_s,
            "maximum_error_m_s": criteria.maximum_mission_velocity_error_m_s,
        }

    obstacle_clearance = report.get("obstacle_clearance", {})
    if not obstacle_clearance.get("applicable", True):
        verdicts["minimum_obstacle_clearance"] = {
            "pass": None,
            "required": False,
            "status": "not_applicable",
            "reason": obstacle_clearance.get(
                "reason", "mission does not provide obstacle geometry"
            ),
        }
    elif not obstacle_clearance.get("available", False):
        verdicts["minimum_obstacle_clearance"] = unavailable_verdict(
            obstacle_clearance.get("reason", "obstacle clearance is unavailable")
        )
    else:
        observed_clearance = obstacle_clearance["minimum_clearance_m"]
        configured_clearance = obstacle_clearance["configured_minimum_clearance_m"]
        verdicts["minimum_obstacle_clearance"] = {
            "pass": observed_clearance >= configured_clearance,
            "minimum_clearance_m": observed_clearance,
            "required_clearance_m": configured_clearance,
            "closest_obstacle_id": obstacle_clearance["closest_obstacle_id"],
        }

    maximum_tilt = report.get("full_flight_attitude", {}).get("tilt_max_deg")
    if maximum_tilt is None:
        verdicts["maximum_tilt"] = unavailable_verdict("attitude telemetry is unavailable")
    else:
        verdicts["maximum_tilt"] = {
            "pass": maximum_tilt <= criteria.maximum_tilt_deg,
            "tilt_max_deg": maximum_tilt,
            "maximum_tilt_deg": criteria.maximum_tilt_deg,
        }

    motor_saturation = report.get("motor_saturation", {})
    if not motor_saturation.get("available", False):
        verdicts["motor_saturation"] = unavailable_verdict(
            "fresh motor telemetry is unavailable"
        )
    else:
        saturation_rate = motor_saturation["saturation_rate"]
        verdicts["motor_saturation"] = {
            "pass": saturation_rate <= criteria.maximum_motor_saturation_rate,
            "saturation_count": motor_saturation["saturation_count"],
            "fresh_sample_count": motor_saturation["fresh_sample_count"],
            "saturation_rate": saturation_rate,
            "maximum_saturation_rate": criteria.maximum_motor_saturation_rate,
        }

    timing = report.get("timing", {})
    if not timing:
        verdicts["worst_case_timing"] = unavailable_verdict(
            "controller-active timing samples are unavailable"
        )
    else:
        solve_max = timing["solve_max_ms"]
        e2e_max = timing["e2e_max_ms"]
        verdicts["worst_case_timing"] = {
            "pass": (
                solve_max <= criteria.maximum_solve_time_ms
                and e2e_max <= criteria.maximum_end_to_end_time_ms
            ),
            "solve_max_ms": solve_max,
            "maximum_solve_time_ms": criteria.maximum_solve_time_ms,
            "end_to_end_max_ms": e2e_max,
            "maximum_end_to_end_time_ms": criteria.maximum_end_to_end_time_ms,
        }

    # Overall
    all_pass = all(
        verdict["pass"] is True
        for verdict in verdicts.values()
        if verdict.get("required", True)
    )
    verdicts["overall"] = {"pass": all_pass}

    return verdicts


def main() -> None:
    args = parse_arguments()
    samples = load_samples(args.input)
    if not samples:
        print("ERROR: No valid samples found", file=sys.stderr)
        sys.exit(1)

    phases = classify_phases(samples)
    criteria = ValidationCriteria(
        maximum_altitude_deviation_m=MAX_ALTITUDE_DEVIATION_M,
        maximum_deadline_miss_rate=MAX_DEADLINE_MISS_RATE,
        maximum_hover_tilt_std_deg=MAX_HOVER_TILT_STD_DEG,
        maximum_mission_tracking_rmse_xy_m=args.max_xy_rmse_m,
        maximum_mission_tracking_error_xy_m=args.max_xy_error_m,
        maximum_mission_velocity_rmse_m_s=args.max_velocity_rmse_m_s,
        maximum_mission_velocity_error_m_s=args.max_velocity_error_m_s,
        maximum_tilt_deg=args.max_tilt_deg,
        maximum_motor_saturation_rate=args.max_motor_saturation_rate,
        maximum_motor_telemetry_age_ms=args.max_motor_telemetry_age_ms,
        maximum_solve_time_ms=args.control_deadline_ms,
        maximum_end_to_end_time_ms=args.control_deadline_ms,
        minimum_obstacle_clearance_m=args.minimum_obstacle_clearance_m,
    )
    operational_samples = [sample for sample in samples if sample["valid"] == 1]

    report = {
        "input_file": str(args.input),
        "mission_file": str(args.mission) if args.mission else None,
        "criteria": asdict(criteria),
        "total_samples": len(samples),
        "hover_samples": len(phases["hover"]),
        "mission_samples": len(phases["mission"]),
        "safety": safety_events(samples),
        "timing": timing_stats(samples),
        "operational_timing": timing_stats(operational_samples),
        "hover_attitude": attitude_stats(
            phases["hover"], "roll_meas", "pitch_meas"
        ),
        "hover_tracking": tracking_stats(phases["hover"]),
        "mission_attitude": attitude_stats(
            phases["mission"], "roll_meas", "pitch_meas"
        ),
        "mission_tracking": tracking_stats(phases["mission"]),
        "mission_velocity_tracking": velocity_tracking_stats(phases["mission"]),
        "full_flight_attitude": attitude_stats(samples, "roll_meas", "pitch_meas"),
        "motor_saturation": motor_saturation_stats(
            phases["mission"], criteria.maximum_motor_telemetry_age_ms
        ),
        "obstacle_clearance": obstacle_clearance_stats(
            phases["mission"], args.mission, criteria.minimum_obstacle_clearance_m
        ),
    }

    report["verdicts"] = evaluate_pass_criteria(report, criteria)

    # Pretty-print report
    report_json = json.dumps(report, indent=2)

    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(report_json + "\n")
        print(f"Report written to {args.output}")
    else:
        print(report_json)

    # Print summary to stderr
    print("\n" + "=" * 60, file=sys.stderr)
    print("VALIDATION REPORT SUMMARY", file=sys.stderr)
    print("=" * 60, file=sys.stderr)
    print(f"  Total samples:        {report['total_samples']}", file=sys.stderr)
    print(f"  Hover samples:        {report['hover_samples']}", file=sys.stderr)
    print(f"  Mission samples:      {report['mission_samples']}", file=sys.stderr)
    print(f"  QP fallback:          {report['safety']['qp_fallback_count']}", file=sys.stderr)
    print(f"  Deadline misses:      {report['safety']['deadline_miss_count']}", file=sys.stderr)
    print(f"  Solver failures:      {report['safety']['solver_failure_count']}", file=sys.stderr)
    print(f"  Solver MINSTEP:       {report['safety']['solver_minstep_count']}", file=sys.stderr)

    if report.get("hover_attitude"):
        ha = report["hover_attitude"]
        print(f"  Hover roll std:       {ha['roll_std_deg']:.2f}°", file=sys.stderr)
        print(f"  Hover pitch std:      {ha['pitch_std_deg']:.2f}°", file=sys.stderr)
        print(f"  Hover tilt max:       {ha['tilt_max_deg']:.2f}°", file=sys.stderr)

    if report.get("mission_attitude"):
        ma = report["mission_attitude"]
        print(f"  Mission tilt max:     {ma['tilt_max_deg']:.2f}°", file=sys.stderr)
        print(f"  Mission tilt >30°:    {ma['tilt_above_30_pct']:.1f}%", file=sys.stderr)
        print(f"  Mission tilt >35°:    {ma['tilt_above_35_pct']:.1f}%", file=sys.stderr)
        print(f"  Mission tilt >40°:    {ma['tilt_above_40_pct']:.1f}%", file=sys.stderr)

    if report.get("mission_tracking"):
        mt = report["mission_tracking"]
        print(f"  Mission XY RMSE:      {mt['xy_rmse_m']:.3f} m", file=sys.stderr)
        print(f"  Mission XY max err:   {mt['xy_max_m']:.3f} m", file=sys.stderr)
        print(f"  Mission Z max err:    {mt['z_max_m']:.3f} m", file=sys.stderr)

    velocity = report.get("mission_velocity_tracking", {})
    if velocity.get("available", False):
        print(
            "  Mission velocity RMSE: "
            f"{velocity['three_dimensional_rmse_m_s']:.3f} m/s",
            file=sys.stderr,
        )

    clearance = report.get("obstacle_clearance", {})
    if clearance.get("available", False):
        print(
            "  Minimum obstacle clearance: "
            f"{clearance['minimum_clearance_m']:.3f} m",
            file=sys.stderr,
        )

    motors = report.get("motor_saturation", {})
    if motors.get("available", False):
        print(
            "  Motor saturation rate: "
            f"{motors['saturation_rate'] * 100.0:.3f}%",
            file=sys.stderr,
        )

    t = report.get("timing", {})
    if t:
        print(f"  Solve mean:           {t['solve_mean_ms']:.2f} ms (p95: {t['solve_p95_ms']:.2f})", file=sys.stderr)
        print(f"  Solve/e2e max:        {t['solve_max_ms']:.2f}/{t['e2e_max_ms']:.2f} ms", file=sys.stderr)
        print(f"  Setpoint age mean:    {t['age_mean_ms']:.2f} ms", file=sys.stderr)

    verdicts = report.get("verdicts", {})
    overall = verdicts.get("overall", {}).get("pass", False)
    print("-" * 60, file=sys.stderr)
    for name, v in verdicts.items():
        if name == "overall":
            continue
        if not v.get("required", True):
            status = "➖ N/A"
        else:
            status = "✅ PASS" if v["pass"] else "❌ FAIL"
        print(f"  {name}: {status}", file=sys.stderr)
    print("=" * 60, file=sys.stderr)
    overall_str = "✅ ALL PASS" if overall else "❌ SOME FAILED"
    print(f"  OVERALL: {overall_str}", file=sys.stderr)
    print("=" * 60, file=sys.stderr)

    sys.exit(0 if overall else 1)


if __name__ == "__main__":
    main()
