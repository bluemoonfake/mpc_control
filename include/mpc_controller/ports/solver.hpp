#pragma once

#include "mpc_controller/domain/model/types.hpp"

#include <Eigen/Core>

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>

namespace mpc_controller::coupled_mpc
{

inline constexpr std::size_t kStateDimension = types::kStateDimension;
inline constexpr std::size_t kInputDimension = types::kInputDimension;
inline constexpr std::size_t kHorizonLength = types::kHorizonLength;
inline constexpr std::size_t kPolygonSides = 6;
// Keep hard input-rate constraints over the full prediction horizon. Shorter
// rate horizons were rejected by the paired semantic stress comparison.
inline constexpr std::size_t kRateConstraintHorizonLength = kHorizonLength;
inline constexpr std::size_t kControlVariableCount =
  kInputDimension * kHorizonLength;
// HPIPM keeps the prediction state in the OCP-QP.
inline constexpr std::size_t kDecisionVariableCount = kControlVariableCount;
inline constexpr std::size_t kFullConstraintRowsPerStage =
  3 * kPolygonSides + 4;
inline constexpr std::size_t kRateConstraintRowsPerStage =
  (kPolygonSides / 2) + 1;
  // Full-stage row count used by the OCP layout.
inline constexpr std::size_t kConstraintRowsPerStage =
  kFullConstraintRowsPerStage;
inline constexpr std::size_t kConstraintRowCount =
  kFullConstraintRowsPerStage * kRateConstraintHorizonLength
  + (kFullConstraintRowsPerStage - kRateConstraintRowsPerStage)
  * (kHorizonLength - kRateConstraintHorizonLength);
inline constexpr std::size_t kOcpStateDimension = kStateDimension + kInputDimension;
inline constexpr std::size_t kOcpStageVariableCount =
  (kHorizonLength + 1U) * kOcpStateDimension
  + kHorizonLength * kInputDimension;
inline constexpr std::size_t kOcpGeneralConstraintCount =
  kFullConstraintRowsPerStage;
inline constexpr std::size_t kRateRowLocalBegin = kPolygonSides / 2;
inline constexpr std::size_t kVelocityRowLocalBegin = kPolygonSides + 1;

constexpr std::size_t constraintRowIndex(
  std::size_t step, std::size_t full_stage_local_row) noexcept
{
  const std::size_t compacted_stage_offset =
    (kFullConstraintRowsPerStage - kRateConstraintRowsPerStage) * step;
  const std::size_t active_rate_offset =
    kRateConstraintRowsPerStage
    * (step < kRateConstraintHorizonLength ? step : kRateConstraintHorizonLength);
  const std::size_t local_offset =
    step < kRateConstraintHorizonLength || full_stage_local_row < kRateRowLocalBegin
      ? full_stage_local_row
      : full_stage_local_row - kRateConstraintRowsPerStage;
  return compacted_stage_offset + active_rate_offset + local_offset;
}

using Clock = std::chrono::steady_clock;
using State = Eigen::Matrix<double, kStateDimension, 1>;
using Input = Eigen::Matrix<double, kInputDimension, 1>;
using Reference = std::array<State, kHorizonLength>;
using Prediction = std::array<State, kHorizonLength>;

enum class Status : uint8_t
{
  success = 0,
  invalid_input = 1,
  infeasible_bounds = 2,
  factorization_failure = 3,
  deadline_exceeded = 4,
  max_iterations = 5,
  non_finite_output = 6,
  output_limit = 7
};

struct Configuration
{
  double dt_first = 0.20;
  double dt_later = 0.20;
  std::array<double, 3> model_time_constant_xyz{};
  std::array<double, 3> stage_weights_xy{};
  std::array<double, 3> terminal_weights_xy{};
  std::array<double, 3> stage_weights_z{};
  std::array<double, 3> terminal_weights_z{};
  std::array<double, 3> control_weights{};
  std::array<double, 3> control_rate_weights{};
  double max_speed_xy = 0.0;
  double max_speed_z = 0.0;
  double max_acceleration_xy = 0.0;
  double max_acceleration_z = 0.0;
  double max_control_xy = 0.0;
  double max_control_z = 0.0;
  double max_control_rate_xy = 0.0;
  double max_control_rate_z = 0.0;
  double gravity_m_s2 = 9.80665;
  double max_tilt_rad = 0.7853981633974483;   //45 degree
  double min_collective_specific_force_m_s2 = 1.0;
  double max_collective_specific_force_m_s2 = 16.0;
  int max_iterations = 400;
  double absolute_tolerance = 1.0e-5;
  double relative_tolerance = 1.0e-5;
};

struct Limits
{
  double max_speed_xy = 0.0;
  double max_speed_z = 0.0;
  // Mission acceleration limits tighten the existing command envelope. They
  // do not instantiate predicted-acceleration rows in the reduced QP.
  double max_acceleration_xy = 0.0;
  double max_acceleration_z = 0.0;
  double max_control_rate_xy = 0.0;
  double max_control_rate_z = 0.0;
};

struct Result
{
  bool valid = false;
  Status status = Status::invalid_input;
  int iterations = 0;
  double primal_residual = std::numeric_limits<double>::infinity();
  double dual_residual = std::numeric_limits<double>::infinity();
  double primal_tolerance = 0.0;
  double dual_tolerance = 0.0;
  double max_constraint_violation = std::numeric_limits<double>::infinity();
  double max_predicted_speed_xy = 0.0;
  double max_predicted_acceleration_xy = 0.0;
  double max_predicted_tilt_rad = 0.0;
  double max_predicted_collective_specific_force_m_s2 = 0.0;
  double problem_update_seconds = 0.0;
  double vector_copy_seconds = 0.0;
  double warm_start_seconds = 0.0;
  double warm_start_prepare_seconds = 0.0;
  double solve_seconds = 0.0;
  double result_postprocess_seconds = 0.0;
  double remaining_budget_seconds = 0.0;
  double coupled_solve_cpu_seconds = 0.0;
  double warm_start_cpu_seconds = 0.0;
  double solve_cpu_seconds = 0.0;
  Input first_control = Input::Zero();
  Prediction prediction{};
};

class Solver final
{
public:
  explicit Solver(const Configuration &configuration);
  ~Solver();

  Solver(const Solver &) = delete;
  Solver &operator=(const Solver &) = delete;

  bool configured() const noexcept;
  void reset() noexcept;
  Result solve(
    const State &initial_state, const Reference &reference,
    const Input &last_control, const Limits &limits,
    Clock::time_point deadline) noexcept;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace mpc_controller::coupled_mpc
