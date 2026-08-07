#include <mpc_control/reference_trajectory.hpp>

#include <algorithm>
#include <cmath>
#include <utility>

namespace mpc_control
{

ReferenceTrajectory::ReferenceTrajectory(std::vector<ReferencePoint> points)
: points_(std::move(points))
{
}

const std::vector<ReferencePoint>& ReferenceTrajectory::points() const noexcept
{
  return points_;
}

std::size_t ReferenceTrajectory::size() const noexcept
{
  return points_.size();
}

bool ReferenceTrajectory::empty() const noexcept
{
  return points_.empty();
}

ReferenceValidationResult ReferenceTrajectory::validate(
    const ReferenceValidationOptions& options) const noexcept
{
  ReferenceValidationResult result;

  if (points_.empty()) {
    result.error = ReferenceError::Empty;
    return result;
  }

  if ((options.check_derivative_consistency
       && (!std::isfinite(options.position_derivative_tolerance)
           || !std::isfinite(options.velocity_derivative_tolerance)
           || options.position_derivative_tolerance < 0.0
           || options.velocity_derivative_tolerance < 0.0))
      || (options.limits.has_max_speed
          && (!std::isfinite(options.limits.max_speed)
              || options.limits.max_speed < 0.0))
      || (options.limits.has_max_acceleration
          && (!std::isfinite(options.limits.max_acceleration)
              || options.limits.max_acceleration < 0.0))) {
    result.error = ReferenceError::InvalidOptions;
    return result;
  }

  for (std::size_t index = 0; index < points_.size(); ++index) {
    const auto& point = points_[index];
    if (!std::isfinite(point.time_seconds)
        || !point.position.allFinite()
        || !point.velocity.allFinite()
        || !point.acceleration.allFinite()
        || !std::isfinite(point.yaw)
        || !std::isfinite(point.yaw_rate)) {
      result.error = ReferenceError::NonFinite;
      result.index = index;
      return result;
    }

    if (index > 0
        && point.time_seconds <= points_[index - 1].time_seconds) {
      result.error = ReferenceError::NonMonotonicTime;
      result.index = index;
      return result;
    }

    if (options.limits.has_max_speed
        && point.velocity.cwiseAbs().maxCoeff() > options.limits.max_speed) {
      result.error = ReferenceError::Infeasible;
      result.index = index;
      return result;
    }

    if (options.limits.has_max_acceleration
        && point.acceleration.cwiseAbs().maxCoeff()
             > options.limits.max_acceleration) {
      result.error = ReferenceError::Infeasible;
      result.index = index;
      return result;
    }
  }

  if (options.check_derivative_consistency) {
    for (std::size_t index = 0; index + 1 < points_.size(); ++index) {
      const auto& lower = points_[index];
      const auto& upper = points_[index + 1];
      const double dt = upper.time_seconds - lower.time_seconds;
      const Eigen::Vector3d average_velocity =
          (lower.velocity + upper.velocity) * 0.5;
      const Eigen::Vector3d average_acceleration =
          (lower.acceleration + upper.acceleration) * 0.5;
      const Eigen::Vector3d position_rate =
          (upper.position - lower.position) / dt;
      const Eigen::Vector3d velocity_rate =
          (upper.velocity - lower.velocity) / dt;

      if ((position_rate - average_velocity).cwiseAbs().maxCoeff()
            > options.position_derivative_tolerance
          || (velocity_rate - average_acceleration).cwiseAbs().maxCoeff()
            > options.velocity_derivative_tolerance) {
        result.error = ReferenceError::DerivativeInconsistent;
        result.index = index;
        return result;
      }
    }
  }

  result.valid = true;
  result.error = ReferenceError::None;
  return result;
}

}  // namespace mpc_control
