#pragma once

#include <Eigen/Core>

namespace mpc_control
{

struct DiscreteVirtualAxisModelConfiguration
{
  double dt = 0.01;
  double model_p1 = 0.0;
  double model_p2 = 1.0;
};

struct DiscreteVirtualAxisStepResult
{
  bool valid = false;
  Eigen::Vector3d state = Eigen::Vector3d::Zero();
};

// Discrete dynamics used by the pinned solver backend:
//   p[k+1] = p[k] + dt * v[k]
//   v[k+1] = v[k] + dt * a[k]
//   a[k+1] = p1 * a[k] + p2 * u[k]
//
// The input u is intentionally scalar. Its physical meaning depends on p1/p2
// and must not be generalized to "acceleration" for every axis.
class DiscreteVirtualAxisModel
{
public:
  explicit DiscreteVirtualAxisModel(
      const DiscreteVirtualAxisModelConfiguration& configuration) noexcept;

  bool valid() const noexcept;
  const DiscreteVirtualAxisModelConfiguration& configuration() const noexcept;

  Eigen::Matrix3d stateTransition() const noexcept;
  Eigen::Vector3d inputTransition() const noexcept;

  DiscreteVirtualAxisStepResult step(
      const Eigen::Vector3d& state,
      double input) const noexcept;

private:
  static bool validConfiguration(
      const DiscreteVirtualAxisModelConfiguration& configuration) noexcept;

  DiscreteVirtualAxisModelConfiguration configuration_{};
  bool valid_ = false;
};

}  // namespace mpc_control
