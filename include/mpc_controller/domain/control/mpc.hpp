#pragma once

#include "mpc_controller/domain/control/limits.hpp"
#include "mpc_controller/domain/model/types.hpp"
#include "mpc_controller/ports/solver.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace mpc_controller::translational
{

inline constexpr std::size_t kHorizonLength = reference::kHorizonLength;
inline constexpr std::size_t kStateSize = 3;
inline constexpr std::size_t kAxisCount = reference::kAxisCount;
inline constexpr double kDtFirst = 0.20;
inline constexpr double kDtLater = 0.20;

using Vector3 = reference::Vector3;
using ReferencePoint = reference::Point;
using ReferenceTrajectoryData = reference::Trajectory;
using ReferenceHorizon = reference::Horizon;
using ValidatedTrajectory = reference::ValidatedTrajectory;
using ReferenceSampler = reference::TrajectoryResampler;

struct MeasuredState
{
  Vector3 position{};
  Vector3 velocity{};
  Vector3 acceleration{};
};

struct Config
{
  double dt_first = kDtFirst;
  double dt_later = kDtLater;
  double solver_deadline_seconds = 0.009;
  int max_iterations = 400;
  std::array<double, 3> model_time_constant_xyz{0.0, 0.0, 0.0};
  double solver_absolute_tolerance = 1.0e-5;
  double solver_relative_tolerance = 1.0e-5;
  std::array<double, 3> q_xy{500.0, 100.0, 100.0};
  std::array<double, 3> s_xy{1000.0, 300.0, 300.0};
  std::array<double, 3> q_z{100.0, 10.0, 10.0};
  std::array<double, 3> s_z{100.0, 10.0, 10.0};
  double control_weight_xy = 0.0;
  double control_rate_weight_xy = 0.0;
  double control_weight_z = 0.0;
  double control_rate_weight_z = 0.0;
  double max_speed_xy = 2.0;
  double max_acceleration_xy = 2.0;
  double max_control_xy = 2.0;
  double max_control_rate_xy = 5.0;
  double max_speed_z = 2.0;
  double max_acceleration_z = 2.0;
  double max_control_z = 50.0;
  double max_control_rate_z = 5.0;
  double gravity_m_s2 = 9.80665;
  double max_tilt_rad = 0.7853981633974483;
  double min_collective_specific_force_m_s2 = 1.0;
  double max_collective_specific_force_m_s2 = 16.0;
};

enum class FailureReason
{
  none,
  invalid_configuration,
  invalid_measured_state,
  invalid_reference,
  solver_not_converged,
  non_finite_solver_output
};

struct UpdateResult
{
  bool valid = false;
  FailureReason failure_reason = FailureReason::none;
  Vector3 control{};
  Vector3 first_predicted_acceleration{};
  coupled_mpc::Result coupled{};
  bool coupled_solver_ran = true;
  bool coupled_control_active = true;
  double coupled_solve_time_seconds = 0.0;
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
  bool deadline_missed = false;
  double solve_time_seconds = 0.0;
};

inline bool finite(const Vector3 &value) noexcept
{
  return std::all_of(value.begin(), value.end(), [](double item) {
    return std::isfinite(item);
  });
}

inline bool finite(const ReferencePoint &point) noexcept
{
  return reference::finite(point);
}

inline bool validConfig(const Config &config) noexcept
{
  const auto nonnegative = [](const std::array<double, 3> &values) {
      return std::all_of(values.begin(), values.end(), [](double value) {
        return std::isfinite(value) && value >= 0.0;
      });
    };
  const std::array positive_values{
    config.dt_first, config.dt_later, config.solver_deadline_seconds,
    config.solver_absolute_tolerance, config.solver_relative_tolerance,
    config.max_speed_xy,
    config.max_acceleration_xy, config.max_control_xy,
    config.max_control_rate_xy, config.max_speed_z,
    config.max_acceleration_z, config.max_control_z,
    config.max_control_rate_z, config.gravity_m_s2, config.max_tilt_rad,
    config.min_collective_specific_force_m_s2,
    config.max_collective_specific_force_m_s2};
  return std::all_of(positive_values.begin(), positive_values.end(), [](double value) {
      return std::isfinite(value) && value > 0.0;
    }) && config.max_iterations > 0
    && nonnegative(config.q_xy) && nonnegative(config.s_xy)
    && nonnegative(config.q_z) && nonnegative(config.s_z)
    && nonnegative(config.model_time_constant_xyz)
    && std::isfinite(config.control_weight_xy) && config.control_weight_xy >= 0.0
    && std::isfinite(config.control_rate_weight_xy)
    && config.control_rate_weight_xy >= 0.0
    && std::isfinite(config.control_weight_z) && config.control_weight_z >= 0.0
    && std::isfinite(config.control_rate_weight_z)
    && config.control_rate_weight_z >= 0.0
    && config.max_tilt_rad < 0.5 * M_PI
    && config.min_collective_specific_force_m_s2
    < config.max_collective_specific_force_m_s2;
}

class TranslationalMpc final
{
public:
  explicit TranslationalMpc(const Config &config)
  : config_(config), coupled_solver_(coupledConfiguration(config))
  {
  }

  void reset() noexcept {reset({0.0, 0.0, 0.0});}

  void reset(const Vector3 &input_memory) noexcept
  {
    last_input_ = input_memory;
    coupled_solver_.reset();
  }

  // Clear only the solver's receding-horizon warm start.  The previous input
  // remains available for the first-input rate constraint on the next solve.
  void resetWarmStart() noexcept {coupled_solver_.reset();}

  UpdateResult update(const MeasuredState &measured, const ReferenceHorizon &reference)
  {
    return updateWithReference(measured, reference, configuredLimits());
  }

  UpdateResult update(
    const MeasuredState &measured, const ReferenceHorizon &reference,
    const SolveLimits &limits)
  {
    return updateWithReference(measured, reference, limits);
  }

  void setInputMemoryForNextSolve(const Vector3 &input) noexcept
  {
    if (finite(input)) {
      last_input_ = input;
    }
  }

private:
  UpdateResult updateWithReference(const MeasuredState &measured,
                                   const ReferenceHorizon &reference,
                                   const SolveLimits &limits)
  {
    UpdateResult output;
    if (!validConfig(config_) || !coupled_solver_.configured()) {
      output.failure_reason = FailureReason::invalid_configuration;
      return output;
    }
    if (!mpc_controller::translational::finite(measured.position) ||
      !mpc_controller::translational::finite(measured.velocity) ||
      !mpc_controller::translational::finite(measured.acceleration)) {
      output.failure_reason = FailureReason::invalid_measured_state;
      return output;
    }
    if (!std::all_of(reference.points.begin(), reference.points.end(),
      [](const ReferencePoint &point) {
        return mpc_controller::translational::finite(point);
      })) {
      output.failure_reason = FailureReason::invalid_reference;
      return output;
    }

    setInputs(measured, reference);
    const auto start = coupled_mpc::Clock::now();
    const auto solve_budget = std::chrono::duration_cast<coupled_mpc::Clock::duration>(
      std::chrono::duration<double>(config_.solver_deadline_seconds));
    const auto coupled_deadline = start + solve_budget;

    output.coupled_solver_ran = true;
    output.coupled = coupled_solver_.solve(
      coupled_initial_, coupled_reference_,
      coupled_mpc::Input(last_input_[0], last_input_[1], last_input_[2]),
      coupledLimits(limits),
      coupled_deadline);
    output.coupled_solve_time_seconds = std::chrono::duration<double>(
      coupled_mpc::Clock::now() - start).count();
    output.solve_time_seconds = output.coupled_solve_time_seconds;
    output.problem_update_seconds = output.coupled.problem_update_seconds;
    output.vector_copy_seconds = output.coupled.vector_copy_seconds;
    output.warm_start_seconds = output.coupled.warm_start_seconds;
    output.warm_start_prepare_seconds =
      output.coupled.warm_start_prepare_seconds;
    output.solve_seconds = output.coupled.solve_seconds;
    output.result_postprocess_seconds =
      output.coupled.result_postprocess_seconds;
    output.remaining_budget_seconds = output.coupled.remaining_budget_seconds;
    output.coupled_solve_cpu_seconds =
      output.coupled.coupled_solve_cpu_seconds;
    output.warm_start_cpu_seconds = output.coupled.warm_start_cpu_seconds;
    output.solve_cpu_seconds = output.coupled.solve_cpu_seconds;
    output.deadline_missed = (output.coupled.status == coupled_mpc::Status::deadline_exceeded);

    if (!output.coupled.valid) {
      output.failure_reason = FailureReason::solver_not_converged;
      return output;
    }

    output.coupled_control_active = true;
    for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
      output.control[axis] = output.coupled.first_control(axis);
      output.first_predicted_acceleration[axis] = output.coupled.prediction[0](6 + axis);
    }

    if (!std::isfinite(output.solve_time_seconds) ||
      !mpc_controller::translational::finite(output.control) ||
      !mpc_controller::translational::finite(output.first_predicted_acceleration)) {
      output.failure_reason = FailureReason::non_finite_solver_output;
      return output;
    }

    last_input_ = output.control;
    output.valid = true;
    output.failure_reason = FailureReason::none;
    return output;
  }

  SolveLimits configuredLimits() const noexcept
  {
    return {config_.max_speed_xy,
            config_.max_speed_z,
            config_.max_acceleration_xy,
            config_.max_acceleration_z,
            config_.max_control_rate_xy,
            config_.max_control_rate_z};
  }

  static coupled_mpc::Limits coupledLimits(const SolveLimits &limits) noexcept
  {
    return {limits.max_speed_xy,
            limits.max_speed_z,
            limits.max_acceleration_xy,
            limits.max_acceleration_z,
            limits.max_control_rate_xy,
            limits.max_control_rate_z};
  }

  static coupled_mpc::Configuration coupledConfiguration(const Config &config)
  {
    coupled_mpc::Configuration output;
    output.dt_first = config.dt_first;
    output.dt_later = config.dt_later;
    output.model_time_constant_xyz = config.model_time_constant_xyz;
    output.stage_weights_xy = config.q_xy;
    output.terminal_weights_xy = config.s_xy;
    output.stage_weights_z = config.q_z;
    output.terminal_weights_z = config.s_z;
    output.control_weights = {
      config.control_weight_xy, config.control_weight_xy, config.control_weight_z};
    output.control_rate_weights = {
      config.control_rate_weight_xy, config.control_rate_weight_xy,
      config.control_rate_weight_z};
    output.max_speed_xy = config.max_speed_xy;
    output.max_speed_z = config.max_speed_z;
    output.max_acceleration_xy = config.max_acceleration_xy;
    output.max_acceleration_z = config.max_acceleration_z;
    output.max_control_xy = config.max_control_xy;
    output.max_control_z = config.max_control_z;
    output.max_control_rate_xy = config.max_control_rate_xy;
    output.max_control_rate_z = config.max_control_rate_z;
    output.gravity_m_s2 = config.gravity_m_s2;
    output.max_tilt_rad = config.max_tilt_rad;
    output.min_collective_specific_force_m_s2 =
      config.min_collective_specific_force_m_s2;
    output.max_collective_specific_force_m_s2 =
      config.max_collective_specific_force_m_s2;
    output.max_iterations = config.max_iterations;
    output.absolute_tolerance = config.solver_absolute_tolerance;
    output.relative_tolerance = config.solver_relative_tolerance;
    return output;
  }

  void setInputs(
    const MeasuredState &measured,
    const ReferenceHorizon &reference) noexcept
  {
    coupled_initial_ <<
      measured.position[0], measured.position[1], measured.position[2],
      measured.velocity[0], measured.velocity[1], measured.velocity[2],
      measured.acceleration[0], measured.acceleration[1], measured.acceleration[2];
    for (std::size_t step = 0; step < kHorizonLength; ++step) {
      coupled_reference_[step] <<
        reference.points[step].position[0], reference.points[step].position[1],
        reference.points[step].position[2], reference.points[step].velocity[0],
        reference.points[step].velocity[1], reference.points[step].velocity[2],
        reference.points[step].acceleration[0], reference.points[step].acceleration[1],
        reference.points[step].acceleration[2];
    }
  }

  Config config_;
  coupled_mpc::Solver coupled_solver_;
  coupled_mpc::State coupled_initial_ = coupled_mpc::State::Zero();
  coupled_mpc::Reference coupled_reference_{};
  Vector3 last_input_{};
};

}  // namespace mpc_controller::translational
