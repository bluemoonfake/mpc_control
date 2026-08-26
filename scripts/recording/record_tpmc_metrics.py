#!/usr/bin/env python3
"""Record TMPC output telemetry required for a SITL tracking assessment."""

import argparse
import csv
import math
import time
from dataclasses import dataclass
from pathlib import Path

import rclpy
from mpc_controller.msg import MpcTranslationalOutput
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from px4_msgs.msg import ActuatorMotors
from px4_msgs.msg import VehicleStatus


MOTOR_LOWER_SATURATION_THRESHOLD = 0.02
MOTOR_UPPER_SATURATION_THRESHOLD = 0.98


@dataclass(frozen=True)
class MotorTelemetry:
    """Latest finite normalized motor outputs observed from PX4."""

    received_monotonic_seconds: float
    minimum_output: float
    maximum_output: float
    motor_count: int

    @property
    def saturated(self) -> bool:
        return (
            self.minimum_output <= MOTOR_LOWER_SATURATION_THRESHOLD
            or self.maximum_output >= MOTOR_UPPER_SATURATION_THRESHOLD
        )


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        required=True,
        help="CSV file receiving TMPC output telemetry.",
    )
    return parser.parse_args()


def tilt_angle_rad(roll: float, pitch: float) -> float:
    body_z_world_z = math.cos(roll) * math.cos(pitch)
    return math.acos(max(-1.0, min(1.0, body_z_world_z)))


class TpmcMetricsRecorder(Node):
    def __init__(self, output_path: Path) -> None:
        super().__init__("tpmc_metrics_recorder")
        output_path.parent.mkdir(parents=True, exist_ok=True)
        self._output_file = output_path.open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(
            self._output_file,
            fieldnames=(
                "timestamp_seconds",
                "position_error_m",
                "altitude_error_m",
                "tilt_rad",
                "solve_time_ms",
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
                "solver_status",
                "solver_iterations",
                "max_constraint_violation",
                "valid",
                "recovery_active",
                "deadline_missed",
                "measured_x",
                "measured_y",
                "measured_z",
                "measured_vx_m_s",
                "measured_vy_m_s",
                "measured_vz_m_s",
                "reference_x",
                "reference_y",
                "reference_z",
                "reference_vx_m_s",
                "reference_vy_m_s",
                "reference_vz_m_s",
                "measured_roll_rad",
                "measured_pitch_rad",
                "measured_yaw_rad",
                "measured_yaw_rate_rad_s",
                "measured_collective_specific_force_m_s2",
                "reference_yaw_rad",
                "reference_yaw_rate_rad_s",
                "control_roll_rad",
                "control_pitch_rad",
                "control_yaw_rad",
                "control_collective_specific_force_m_s2",
                "raw_roll_rad",
                "raw_pitch_rad",
                "raw_yaw_rad",
                "raw_collective_specific_force_m_s2",
                "filtered_roll_rad",
                "filtered_pitch_rad",
                "filtered_yaw_rad",
                "filtered_collective_specific_force_m_s2",
                "applied_roll_rad",
                "applied_pitch_rad",
                "applied_yaw_rad",
                "applied_collective_specific_force_m_s2",
                "applied_command_age_ms",
                "applied_command_available",
                "motor_telemetry_available",
                "motor_telemetry_age_ms",
                "motor_count",
                "motor_min_normalized",
                "motor_max_normalized",
                "motor_saturated",
                "failure_reason",
            ),
        )
        self._writer.writeheader()
        self._output_file.flush()
        self._external_mode_active = False
        self._latest_motor_telemetry: MotorTelemetry | None = None
        self._vehicle_status_subscription = self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status_v1",
            self._update_external_mode,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self._actuator_motors_subscription = self.create_subscription(
            ActuatorMotors,
            "/fmu/out/actuator_motors",
            self._update_motor_telemetry,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self._subscription = self.create_subscription(
            MpcTranslationalOutput,
            "mpc_translational_output",
            self._record,
            50,
        )

    def destroy_node(self) -> bool:
        self._output_file.close()
        return super().destroy_node()

    def _update_external_mode(self, message: VehicleStatus) -> None:
        external_mode_active = message.executor_in_charge != 0
        if external_mode_active != self._external_mode_active:
            self.get_logger().info(
                "External Mode telemetry recording "
                f"{'enabled' if external_mode_active else 'paused'}"
            )
        self._external_mode_active = external_mode_active

    def _update_motor_telemetry(self, message: ActuatorMotors) -> None:
        finite_outputs = [
            float(output) for output in message.control if math.isfinite(float(output))
        ]
        if not finite_outputs:
            return
        self._latest_motor_telemetry = MotorTelemetry(
            received_monotonic_seconds=time.monotonic(),
            minimum_output=min(finite_outputs),
            maximum_output=max(finite_outputs),
            motor_count=len(finite_outputs),
        )

    def _motor_telemetry_fields(self) -> dict[str, float | int]:
        telemetry = self._latest_motor_telemetry
        if telemetry is None:
            return {
                "motor_telemetry_available": 0,
                "motor_telemetry_age_ms": math.nan,
                "motor_count": 0,
                "motor_min_normalized": math.nan,
                "motor_max_normalized": math.nan,
                "motor_saturated": 0,
            }

        return {
            "motor_telemetry_available": 1,
            "motor_telemetry_age_ms": (
                time.monotonic() - telemetry.received_monotonic_seconds
            )
            * 1.0e3,
            "motor_count": telemetry.motor_count,
            "motor_min_normalized": telemetry.minimum_output,
            "motor_max_normalized": telemetry.maximum_output,
            "motor_saturated": int(telemetry.saturated),
        }

    def _record(self, message: MpcTranslationalOutput) -> None:
        if not self._external_mode_active:
            return
        measured = message.measured_state
        reference = message.first_reference_state
        position_error_m = math.sqrt(
            sum((reference[index] - measured[index]) ** 2 for index in range(3))
        )
        row = {
                "timestamp_seconds": message.header.stamp.sec
                + message.header.stamp.nanosec * 1.0e-9,
                "position_error_m": position_error_m,
                "altitude_error_m": reference[2] - measured[2],
                "tilt_rad": tilt_angle_rad(measured[6], measured[7]),
                "solve_time_ms": message.solve_time_seconds * 1.0e3,
                "preparation_time_ms": message.preparation_time_seconds * 1.0e3,
                "acados_wall_time_ms": message.acados_wall_time_seconds * 1.0e3,
                "postprocessing_time_ms": message.postprocessing_time_seconds
                * 1.0e3,
                "acados_metadata_time_ms": message.acados_metadata_time_seconds
                * 1.0e3,
                "diagnostics_time_ms": message.diagnostics_time_seconds * 1.0e3,
                "sqp_statistics_time_ms": message.sqp_statistics_time_seconds
                * 1.0e3,
                "prediction_read_time_ms": message.prediction_read_time_seconds
                * 1.0e3,
                "constraint_validation_time_ms": (
                    message.constraint_validation_time_seconds * 1.0e3
                ),
                "result_finalization_time_ms": (
                    message.result_finalization_time_seconds * 1.0e3
                ),
                "postprocessing_unattributed_time_ms": (
                    message.postprocessing_unattributed_time_seconds * 1.0e3
                ),
                "end_to_end_time_ms": message.end_to_end_time_seconds * 1.0e3,
                "setpoint_age_ms": message.setpoint_age_seconds * 1.0e3,
                "reference_age_ms": message.reference_age_seconds * 1.0e3,
                "solver_status": message.solver_status,
                "solver_iterations": message.solver_iterations,
                "max_constraint_violation": message.max_constraint_violation,
                "valid": int(message.valid),
                "recovery_active": int(message.recovery_active),
                "deadline_missed": int(message.deadline_missed),
                "measured_x": measured[0],
                "measured_y": measured[1],
                "measured_z": measured[2],
                "measured_vx_m_s": measured[3],
                "measured_vy_m_s": measured[4],
                "measured_vz_m_s": measured[5],
                "reference_x": reference[0],
                "reference_y": reference[1],
                "reference_z": reference[2],
                "reference_vx_m_s": reference[3],
                "reference_vy_m_s": reference[4],
                "reference_vz_m_s": reference[5],
                "measured_roll_rad": measured[6],
                "measured_pitch_rad": measured[7],
                "measured_yaw_rad": measured[8],
                "measured_yaw_rate_rad_s": measured[9],
                "measured_collective_specific_force_m_s2": measured[10],
                "reference_yaw_rad": reference[8],
                "reference_yaw_rate_rad_s": reference[9],
                "control_roll_rad": message.control_input[0],
                "control_pitch_rad": message.control_input[1],
                "control_yaw_rad": message.control_input[2],
                "control_collective_specific_force_m_s2": message.control_input[3],
                "raw_roll_rad": message.raw_control_input[0],
                "raw_pitch_rad": message.raw_control_input[1],
                "raw_yaw_rad": message.raw_control_input[2],
                "raw_collective_specific_force_m_s2": message.raw_control_input[3],
                "filtered_roll_rad": message.filtered_control_input[0],
                "filtered_pitch_rad": message.filtered_control_input[1],
                "filtered_yaw_rad": message.filtered_control_input[2],
                "filtered_collective_specific_force_m_s2": (
                    message.filtered_control_input[3]
                ),
                "applied_roll_rad": message.applied_control_input[0],
                "applied_pitch_rad": message.applied_control_input[1],
                "applied_yaw_rad": message.applied_control_input[2],
                "applied_collective_specific_force_m_s2": (
                    message.applied_control_input[3]
                ),
                "applied_command_age_ms": (
                    message.applied_command_age_seconds * 1.0e3
                ),
                "applied_command_available": int(
                    message.applied_command_available
                ),
            "failure_reason": message.failure_reason,
        }
        row.update(self._motor_telemetry_fields())
        self._writer.writerow(row)
        self._output_file.flush()


def main() -> None:
    arguments = parse_arguments()
    rclpy.init()
    recorder = TpmcMetricsRecorder(arguments.output)
    try:
        rclpy.spin(recorder)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        recorder.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
