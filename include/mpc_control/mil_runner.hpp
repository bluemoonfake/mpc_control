#pragma once

#include "mpc_trajectory_core.hpp"

#include <cstddef>

namespace mpc_control
{

enum class MilFailureReason
{
  None,
  InvalidConfiguration,
  InvalidReference,
  SamplingFailure,
  CoreFailure,
  ModelMismatch,
  NonFiniteMetric,
  ConstraintViolation,
};

struct MilRunConfiguration
{
  MpcCoreConfiguration core{};
  ReferenceSamplerConfig sampler{};
  double start_time_seconds = 0.0;
  double duration_seconds = 1.0;
  double update_period_seconds = 0.01;
  double model_tolerance = 1.0e-7;
  double constraint_tolerance = 1.0e-6;
};

struct MilMetrics
{
  std::size_t update_count = 0;
  double shaping_position_rmse = 0.0;
  double shaping_position_max = 0.0;
  double final_position_error = 0.0;
  double final_velocity_norm = 0.0;
  double final_acceleration_norm = 0.0;
  double max_virtual_velocity = 0.0;
  double max_virtual_acceleration = 0.0;
  double max_jerk = 0.0;
  double max_command_total_variation = 0.0;
  double max_model_consistency_error = 0.0;
  std::size_t constraint_violations = 0;
};

struct MilRunResult
{
  bool valid = false;
  MilFailureReason failure_reason = MilFailureReason::InvalidConfiguration;
  MpcFailureReason core_failure_reason = MpcFailureReason::None;
  ReferenceError reference_error = ReferenceError::None;
  MilMetrics metrics{};
  VirtualTrajectoryState final_virtual_state{};
};

// Offline model-in-the-loop runner for the virtual trajectory generator. It
// intentionally reports shaping and virtual-state metrics only. UAV/PX4
// tracking error requires a measured plant state and belongs to later MIL/SITL
// integration layers.
class MilRunner
{
public:
  MilRunResult run(
      const ReferenceTrajectory& reference,
      const VehicleState& initial_state,
      const MilRunConfiguration& configuration = {}) const noexcept;

private:
  static bool validConfiguration(
      const MilRunConfiguration& configuration) noexcept;
};

}  // namespace mpc_control
