#pragma once

#include <Eigen/Core>

#include <array>
#include <cstddef>
#include <vector>

namespace mpc_control
{

struct ReferencePoint
{
  double time_seconds = 0.0;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
  double yaw = 0.0;
  double yaw_rate = 0.0;
};

struct ReferenceLimits
{
  bool has_max_speed = false;
  bool has_max_acceleration = false;
  double max_speed = 0.0;
  double max_acceleration = 0.0;
};

struct ReferenceValidationOptions
{
  bool check_derivative_consistency = false;
  double position_derivative_tolerance = 1.0e-3;
  double velocity_derivative_tolerance = 1.0e-3;
  ReferenceLimits limits{};
};

enum class ReferenceError
{
  None,
  Empty,
  NonFinite,
  NonMonotonicTime,
  DerivativeInconsistent,
  Infeasible,
  InvalidOptions,
  InvalidGrid,
  BeforeStart,
  AfterEnd,
};

struct ReferenceValidationResult
{
  bool valid = false;
  ReferenceError error = ReferenceError::Empty;
  std::size_t index = 0;
};

class ReferenceTrajectory
{
public:
  ReferenceTrajectory() = default;
  explicit ReferenceTrajectory(std::vector<ReferencePoint> points);

  const std::vector<ReferencePoint>& points() const noexcept;
  std::size_t size() const noexcept;
  bool empty() const noexcept;

  ReferenceValidationResult validate(
      const ReferenceValidationOptions& options = {}) const noexcept;

private:
  std::vector<ReferencePoint> points_;
};

}  // namespace mpc_control
