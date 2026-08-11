#pragma once

#include "mpc_controller/mpc_solver.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

namespace mpc_controller::translational
{

inline constexpr std::size_t kHorizonLength = 26;
inline constexpr std::size_t kStateSize = 3;
inline constexpr std::size_t kAxisCount = 3;
inline constexpr double kDtFirst = 0.01;
inline constexpr double kDtLater = 0.20;

using Vector3 = std::array<double, 3>;

struct ReferencePoint
{
  double time_from_start = 0.0;
  Vector3 position{};
  Vector3 velocity{};
  Vector3 acceleration{};
  double yaw = 0.0;
  double yaw_rate = 0.0;
};

struct ReferenceTrajectoryData
{
  double header_time_seconds = 0.0;
  bool hold_after_end = false;
  std::vector<ReferencePoint> points;
};

struct ReferenceHorizon
{
  std::array<ReferencePoint, kHorizonLength> points{};
  std::array<double, kHorizonLength> time_offsets{};
};

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
  int max_iterations = 400;
  std::array<double, 3> model_alpha_xyz{0.0, 0.0, 0.0};
  double admm_rho = 0.02;
  double solver_absolute_tolerance = 1.0e-5;
  double solver_relative_tolerance = 1.0e-5;

  std::array<double, 3> q_xy{500.0, 100.0, 100.0};
  std::array<double, 3> s_xy{1000.0, 300.0, 300.0};
  std::array<double, 3> q_z{100.0, 10.0, 10.0};
  std::array<double, 3> s_z{100.0, 10.0, 10.0};

  double max_speed_xy = 2.0;
  double max_control_xy = 2.0;
  double max_control_rate_xy = 5.0;
  double max_speed_z = 2.0;
  double max_acceleration_z = 2.0;
  double max_control_z = 50.0;
  double max_control_rate_z = 999.0;
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
  std::array<double, 3> control{};  // optimizer input u0 for X/Y/Z
  std::array<double, 3> first_predicted_acceleration{};  // a[k+1]
  std::array<int, 3> iterations{};
  std::array<double, 3> initial_state_x{};
  std::array<double, 3> initial_state_y{};
  std::array<double, 3> initial_state_z{};
  // Layout: prediction[step * 9 + axis * 3 + state], state=[p,v,a].
  std::array<double, kHorizonLength * kAxisCount * kStateSize> prediction{};
  double solve_time_seconds = 0.0;
};

inline bool finite(const Vector3 &value) noexcept
{
  return std::isfinite(value[0]) && std::isfinite(value[1]) && std::isfinite(value[2]);
}

inline bool finite(const ReferencePoint &point) noexcept
{
  return std::isfinite(point.time_from_start) && point.time_from_start >= 0.0
    && finite(point.position) && finite(point.velocity) && finite(point.acceleration)
    && std::isfinite(point.yaw) && std::isfinite(point.yaw_rate);
}

inline bool validConfig(const Config &config) noexcept
{
  const auto finite_array = [](const std::array<double, 3> &values) {
      return std::all_of(values.begin(), values.end(), [](double value) {
        return std::isfinite(value) && value >= 0.0;
      });
    };
  return std::isfinite(config.dt_first) && config.dt_first > 0.0
    && std::isfinite(config.dt_later) && config.dt_later > 0.0
    && config.max_iterations > 0 && finite_array(config.q_xy) && finite_array(config.s_xy)
    && finite_array(config.q_z) && finite_array(config.s_z)
    && std::all_of(config.model_alpha_xyz.begin(), config.model_alpha_xyz.end(), [](double value) {
      return std::isfinite(value) && value >= 0.0 && value < 1.0;
    })
    && std::isfinite(config.admm_rho) && config.admm_rho > 0.0
    && std::isfinite(config.solver_absolute_tolerance)
    && config.solver_absolute_tolerance > 0.0
    && std::isfinite(config.solver_relative_tolerance)
    && config.solver_relative_tolerance > 0.0
    && std::isfinite(config.max_speed_xy) && config.max_speed_xy > 0.0
    && std::isfinite(config.max_control_xy) && config.max_control_xy > 0.0
    && std::isfinite(config.max_control_rate_xy) && config.max_control_rate_xy > 0.0
    && std::isfinite(config.max_speed_z) && config.max_speed_z > 0.0
    && std::isfinite(config.max_acceleration_z) && config.max_acceleration_z > 0.0
    && std::isfinite(config.max_control_z) && config.max_control_z > 0.0
    && std::isfinite(config.max_control_rate_z) && config.max_control_rate_z > 0.0;
}

class ReferenceSampler final
{
public:
  static double shortestAngle(double from, double to) noexcept
  {
    return std::atan2(std::sin(to - from), std::cos(to - from));
  }

  static bool validTrajectory(const ReferenceTrajectoryData &trajectory) noexcept
  {
    if (!std::isfinite(trajectory.header_time_seconds) || trajectory.points.empty()) {
      return false;
    }
    for (std::size_t i = 0; i < trajectory.points.size(); ++i) {
      if (!finite(trajectory.points[i])) {
        return false;
      }
      if (i > 0 && trajectory.points[i].time_from_start <= trajectory.points[i - 1].time_from_start) {
        return false;
      }
    }
    return true;
  }

  static bool sampleAt(
    const ReferenceTrajectoryData &trajectory, double time_from_start, ReferencePoint &output) noexcept
  {
    if (!validTrajectory(trajectory) || !std::isfinite(time_from_start) || time_from_start < 0.0) {
      return false;
    }
    if (time_from_start <= trajectory.points.front().time_from_start) {
      output = trajectory.points.front();
      return true;
    }
    for (std::size_t i = 1; i < trajectory.points.size(); ++i) {
      const auto &left = trajectory.points[i - 1];
      const auto &right = trajectory.points[i];
      if (time_from_start <= right.time_from_start) {
        const double alpha = (time_from_start - left.time_from_start)
          / (right.time_from_start - left.time_from_start);
        output = interpolate(left, right, alpha);
        return true;
      }
    }
    if (!trajectory.hold_after_end) {
      return false;
    }
    output = trajectory.points.back();
    return true;
  }

  static bool buildHorizon(
    const ReferenceTrajectoryData &trajectory, double elapsed_seconds,
    double dt_first, double dt_later, ReferenceHorizon &output) noexcept
  {
    if (!validTrajectory(trajectory) || !std::isfinite(elapsed_seconds) || elapsed_seconds < 0.0
      || !std::isfinite(dt_first) || dt_first <= 0.0 || !std::isfinite(dt_later) || dt_later <= 0.0) {
      return false;
    }
    for (std::size_t i = 0; i < kHorizonLength; ++i) {
      output.time_offsets[i] = dt_first + static_cast<double>(i) * dt_later;
      if (!sampleAt(trajectory, elapsed_seconds + output.time_offsets[i], output.points[i])) {
        return false;
      }
    }
    return true;
  }

private:
  static ReferencePoint interpolate(
    const ReferencePoint &left, const ReferencePoint &right, double alpha) noexcept
  {
    ReferencePoint output;
    output.time_from_start = left.time_from_start
      + alpha * (right.time_from_start - left.time_from_start);
    for (int axis = 0; axis < 3; ++axis) {
      output.position[axis] = left.position[axis] + alpha * (right.position[axis] - left.position[axis]);
      output.velocity[axis] = left.velocity[axis] + alpha * (right.velocity[axis] - left.velocity[axis]);
      output.acceleration[axis] = left.acceleration[axis]
        + alpha * (right.acceleration[axis] - left.acceleration[axis]);
    }
    output.yaw = left.yaw + alpha * shortestAngle(left.yaw, right.yaw);
    output.yaw_rate = left.yaw_rate + alpha * (right.yaw_rate - left.yaw_rate);
    return output;
  }
};

class TranslationalMpc final
{
public:
  explicit TranslationalMpc(const Config &config)
  : config_(config),
    solver_x_(axisConfiguration(config, 0, config.q_xy, config.s_xy,
      config.max_speed_xy, 999.0, config.max_control_xy, config.max_control_rate_xy)),
    solver_y_(axisConfiguration(config, 1, config.q_xy, config.s_xy,
      config.max_speed_xy, 999.0, config.max_control_xy, config.max_control_rate_xy)),
    solver_z_(axisConfiguration(config, 2, config.q_z, config.s_z,
      config.max_speed_z, config.max_acceleration_z, config.max_control_z,
      config.max_control_rate_z))
  {
  }

  void reset() noexcept
  {
    last_input_ = {0.0, 0.0, 0.0};
    solver_x_.reset();
    solver_y_.reset();
    solver_z_.reset();
  }

  UpdateResult update(const MeasuredState &measured, const ReferenceHorizon &reference)
  {
    UpdateResult result;
    if (!validConfig(config_) || !solver_x_.configured()
      || !solver_y_.configured() || !solver_z_.configured()) {
      result.failure_reason = FailureReason::invalid_configuration;
      return result;
    }
    if (!finite(measured.position) || !finite(measured.velocity) || !finite(measured.acceleration)) {
      result.failure_reason = FailureReason::invalid_measured_state;
      return result;
    }
    for (const auto &point : reference.points) {
      if (!finite(point)) {
        result.failure_reason = FailureReason::invalid_reference;
        return result;
      }
    }

    setInitialState(measured);
    packReference(reference);
    result.initial_state_x = {measured.position[0], measured.velocity[0], measured.acceleration[0]};
    result.initial_state_y = {measured.position[1], measured.velocity[1], measured.acceleration[1]};
    result.initial_state_z = {measured.position[2], measured.velocity[2], measured.acceleration[2]};

    const auto start = std::chrono::steady_clock::now();
    const auto x_solve = solver_x_.solve(initial_x_, reference_x_, last_input_[0]);
    const auto y_solve = solver_y_.solve(initial_y_, reference_y_, last_input_[1]);
    const auto z_solve = solver_z_.solve(initial_z_, reference_z_, last_input_[2]);
    result.iterations = {x_solve.iterations, y_solve.iterations, z_solve.iterations};
    result.solve_time_seconds = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - start).count();

    if (!x_solve.valid || !y_solve.valid || !z_solve.valid) {
      result.failure_reason = FailureReason::solver_not_converged;
      return result;
    }

    prediction_x_ = x_solve.prediction;
    prediction_y_ = y_solve.prediction;
    prediction_z_ = z_solve.prediction;
    result.control = {
      x_solve.first_control, y_solve.first_control, z_solve.first_control};
    result.first_predicted_acceleration = {
      prediction_x_[0](2), prediction_y_[0](2), prediction_z_[0](2)};

    for (std::size_t step = 0; step < kHorizonLength; ++step) {
      for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
        const axis_mpc::Prediction *prediction = axis == 0 ? &prediction_x_
          : (axis == 1 ? &prediction_y_ : &prediction_z_);
        for (std::size_t state = 0; state < kStateSize; ++state) {
          result.prediction[step * 9 + axis * 3 + state] =
            (*prediction)[step](static_cast<Eigen::Index>(state));
        }
      }
    }
    if (!std::isfinite(result.solve_time_seconds)
      || !finite(result.control) || !finite(result.first_predicted_acceleration)
      || !std::all_of(result.prediction.begin(), result.prediction.end(),
        [](double value) {return std::isfinite(value);})) {
      result.failure_reason = FailureReason::non_finite_solver_output;
      return result;
    }

    // The rate-limit memory advances only after all three axis solves and
    // their outputs have passed validation.
    last_input_ = result.control;
    result.valid = true;
    result.failure_reason = FailureReason::none;
    return result;
  }

private:
  static axis_mpc::Configuration axisConfiguration(
    const Config &config, std::size_t axis, const std::array<double, 3> &stage_weights,
    const std::array<double, 3> &terminal_weights, double max_speed,
    double max_acceleration, double max_control, double max_control_rate)
  {
    axis_mpc::Configuration output;
    output.dt_first = config.dt_first;
    output.dt_later = config.dt_later;
    output.model_alpha = config.model_alpha_xyz[axis];
    output.stage_weights = stage_weights;
    output.terminal_weights = terminal_weights;
    output.max_speed = max_speed;
    output.max_acceleration = max_acceleration;
    output.max_control = max_control;
    output.max_control_rate = max_control_rate;
    output.max_iterations = config.max_iterations;
    output.admm_rho = config.admm_rho;
    output.absolute_tolerance = config.solver_absolute_tolerance;
    output.relative_tolerance = config.solver_relative_tolerance;
    return output;
  }

  void setInitialState(const MeasuredState &measured)
  {
    initial_x_ = axis_mpc::State(measured.position[0], measured.velocity[0], measured.acceleration[0]);
    initial_y_ = axis_mpc::State(measured.position[1], measured.velocity[1], measured.acceleration[1]);
    initial_z_ = axis_mpc::State(measured.position[2], measured.velocity[2], measured.acceleration[2]);
  }

  void packReference(const ReferenceHorizon &reference)
  {
    for (std::size_t i = 0; i < kHorizonLength; ++i) {
      reference_x_[i] = axis_mpc::State(
        reference.points[i].position[0], reference.points[i].velocity[0],
        reference.points[i].acceleration[0]);
      reference_y_[i] = axis_mpc::State(
        reference.points[i].position[1], reference.points[i].velocity[1],
        reference.points[i].acceleration[1]);
      reference_z_[i] = axis_mpc::State(
        reference.points[i].position[2], reference.points[i].velocity[2],
        reference.points[i].acceleration[2]);
    }
  }

  Config config_;
  axis_mpc::Solver solver_x_;
  axis_mpc::Solver solver_y_;
  axis_mpc::Solver solver_z_;
  axis_mpc::State initial_x_ = axis_mpc::State::Zero();
  axis_mpc::State initial_y_ = axis_mpc::State::Zero();
  axis_mpc::State initial_z_ = axis_mpc::State::Zero();
  axis_mpc::Reference reference_x_{};
  axis_mpc::Reference reference_y_{};
  axis_mpc::Reference reference_z_{};
  axis_mpc::Prediction prediction_x_{};
  axis_mpc::Prediction prediction_y_{};
  axis_mpc::Prediction prediction_z_{};
  std::array<double, 3> last_input_{0.0, 0.0, 0.0};
};

}  // namespace mpc_controller::translational
