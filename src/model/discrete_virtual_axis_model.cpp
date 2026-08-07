#include <mpc_control/discrete_virtual_axis_model.hpp>

#include <cmath>

namespace mpc_control
{

DiscreteVirtualAxisModel::DiscreteVirtualAxisModel(
    const DiscreteVirtualAxisModelConfiguration& configuration) noexcept
: configuration_(configuration),
  valid_(validConfiguration(configuration))
{
}

bool DiscreteVirtualAxisModel::valid() const noexcept
{
  return valid_;
}

const DiscreteVirtualAxisModelConfiguration&
DiscreteVirtualAxisModel::configuration() const noexcept
{
  return configuration_;
}

Eigen::Matrix3d DiscreteVirtualAxisModel::stateTransition() const noexcept
{
  Eigen::Matrix3d matrix = Eigen::Matrix3d::Identity();
  matrix(0, 1) = configuration_.dt;
  matrix(1, 2) = configuration_.dt;
  matrix(2, 2) = configuration_.model_p1;
  return matrix;
}

Eigen::Vector3d DiscreteVirtualAxisModel::inputTransition() const noexcept
{
  Eigen::Vector3d vector = Eigen::Vector3d::Zero();
  vector(2) = configuration_.model_p2;
  return vector;
}

DiscreteVirtualAxisStepResult DiscreteVirtualAxisModel::step(
    const Eigen::Vector3d& state,
    const double input) const noexcept
{
  DiscreteVirtualAxisStepResult result;
  if (!valid_ || !state.allFinite() || !std::isfinite(input)) {
    return result;
  }

  result.state = stateTransition() * state + inputTransition() * input;
  result.valid = result.state.allFinite();
  if (!result.valid) {
    result.state.setZero();
  }
  return result;
}

bool DiscreteVirtualAxisModel::validConfiguration(
    const DiscreteVirtualAxisModelConfiguration& configuration) noexcept
{
  return std::isfinite(configuration.dt)
      && configuration.dt > 0.0
      && std::isfinite(configuration.model_p1)
      && std::isfinite(configuration.model_p2);
}

}  // namespace mpc_control
