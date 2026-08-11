#pragma once

#include <mrs_mpc_solvers/mpc_controller.h>

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
  int max_iterations = 45;
  bool solver_verbose = false;

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
    solver_x_("mpc_x", config.solver_verbose, config.max_iterations,
      toVector(config.q_xy), toVector(config.s_xy), config.dt_first, config.dt_later, 0.0, 1.0),
    solver_y_("mpc_y", config.solver_verbose, config.max_iterations,
      toVector(config.q_xy), toVector(config.s_xy), config.dt_first, config.dt_later, 0.0, 1.0),
    solver_z_("mpc_z", config.solver_verbose, config.max_iterations,
      // M3 consumes u_z directly as a desired physical acceleration. Keep the
      // prediction model at the same command boundary so a raw measured a_z
      // transient cannot be inverted into a large actuator command.
      toVector(config.q_z), toVector(config.s_z), config.dt_first, config.dt_later, 0.0, 1.0),
    initial_x_(3, 1), initial_y_(3, 1), initial_z_(3, 1),
    reference_x_(kHorizonLength * kStateSize, 1),
    reference_y_(kHorizonLength * kStateSize, 1),
    reference_z_(kHorizonLength * kStateSize, 1),
    prediction_x_(kHorizonLength * kStateSize, 1),
    prediction_y_(kHorizonLength * kStateSize, 1),
    prediction_z_(kHorizonLength * kStateSize, 1)
  {
  }

  void reset() noexcept
  {
    last_input_ = {0.0, 0.0, 0.0};
  }

  UpdateResult update(const MeasuredState &measured, const ReferenceHorizon &reference)
  {
    UpdateResult result;
    if (!validConfig(config_)) {
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
    try {
      solver_x_.setLastInput(last_input_[0]);
      solver_x_.loadReference(reference_x_);
      solver_x_.setLimits(config_.max_speed_xy, 999.0, config_.max_control_xy,
        config_.max_control_rate_xy, config_.dt_first, config_.dt_later);
      solver_x_.setInitialState(initial_x_);
      result.iterations[0] = solver_x_.solveMPC();

      solver_y_.setLastInput(last_input_[1]);
      solver_y_.loadReference(reference_y_);
      solver_y_.setLimits(config_.max_speed_xy, 999.0, config_.max_control_xy,
        config_.max_control_rate_xy, config_.dt_first, config_.dt_later);
      solver_y_.setInitialState(initial_y_);
      result.iterations[1] = solver_y_.solveMPC();

      solver_z_.setLastInput(last_input_[2]);
      solver_z_.loadReference(reference_z_);
      solver_z_.setLimits(config_.max_speed_z, config_.max_acceleration_z,
        config_.max_control_z, config_.max_control_rate_z, config_.dt_first, config_.dt_later);
      solver_z_.setInitialState(initial_z_);
      result.iterations[2] = solver_z_.solveMPC();
    } catch (...) {
      result.failure_reason = FailureReason::solver_not_converged;
      return result;
    }
    result.solve_time_seconds = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - start).count();

    const std::array<int, 3> iterations = result.iterations;
    if (std::any_of(iterations.begin(), iterations.end(), [this](int value) {
        return value <= 0 || value >= config_.max_iterations;
      })) {
      result.failure_reason = FailureReason::solver_not_converged;
      return result;
    }

    prediction_x_.setZero();
    prediction_y_.setZero();
    prediction_z_.setZero();
    solver_x_.getStates(prediction_x_);
    solver_y_.getStates(prediction_y_);
    solver_z_.getStates(prediction_z_);
    result.control = {
      solver_x_.getFirstControlInput(), solver_y_.getFirstControlInput(), solver_z_.getFirstControlInput()};
    result.first_predicted_acceleration = {
      prediction_x_(2, 0), prediction_y_(2, 0), prediction_z_(2, 0)};

    for (std::size_t step = 0; step < kHorizonLength; ++step) {
      for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
        const Eigen::MatrixXd *prediction = axis == 0 ? &prediction_x_
          : (axis == 1 ? &prediction_y_ : &prediction_z_);
        for (std::size_t state = 0; state < kStateSize; ++state) {
          result.prediction[step * 9 + axis * 3 + state] =
            (*prediction)(static_cast<Eigen::Index>(step * 3 + state), 0);
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
  static std::vector<double> toVector(const std::array<double, 3> &values)
  {
    return {values[0], values[1], values[2]};
  }

  void setInitialState(const MeasuredState &measured)
  {
    initial_x_ << measured.position[0], measured.velocity[0], measured.acceleration[0];
    initial_y_ << measured.position[1], measured.velocity[1], measured.acceleration[1];
    initial_z_ << measured.position[2], measured.velocity[2], measured.acceleration[2];
  }

  void packReference(const ReferenceHorizon &reference)
  {
    for (std::size_t i = 0; i < kHorizonLength; ++i) {
      reference_x_(static_cast<Eigen::Index>(3 * i + 0), 0) = reference.points[i].position[0];
      reference_x_(static_cast<Eigen::Index>(3 * i + 1), 0) = reference.points[i].velocity[0];
      reference_x_(static_cast<Eigen::Index>(3 * i + 2), 0) = reference.points[i].acceleration[0];
      reference_y_(static_cast<Eigen::Index>(3 * i + 0), 0) = reference.points[i].position[1];
      reference_y_(static_cast<Eigen::Index>(3 * i + 1), 0) = reference.points[i].velocity[1];
      reference_y_(static_cast<Eigen::Index>(3 * i + 2), 0) = reference.points[i].acceleration[1];
      reference_z_(static_cast<Eigen::Index>(3 * i + 0), 0) = reference.points[i].position[2];
      reference_z_(static_cast<Eigen::Index>(3 * i + 1), 0) = reference.points[i].velocity[2];
      reference_z_(static_cast<Eigen::Index>(3 * i + 2), 0) = reference.points[i].acceleration[2];
    }
  }

  Config config_;
  mrs_mpc_solvers::mpc_controller::Solver solver_x_;
  mrs_mpc_solvers::mpc_controller::Solver solver_y_;
  mrs_mpc_solvers::mpc_controller::Solver solver_z_;
  Eigen::MatrixXd initial_x_;
  Eigen::MatrixXd initial_y_;
  Eigen::MatrixXd initial_z_;
  Eigen::MatrixXd reference_x_;
  Eigen::MatrixXd reference_y_;
  Eigen::MatrixXd reference_z_;
  Eigen::MatrixXd prediction_x_;
  Eigen::MatrixXd prediction_y_;
  Eigen::MatrixXd prediction_z_;
  std::array<double, 3> last_input_{0.0, 0.0, 0.0};
};

}  // namespace mpc_controller::translational
