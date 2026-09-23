#include "mpc_controller/application/control_runtime.hpp"

#include <algorithm>
#include <cmath>

namespace mpc_controller::application
{

ControlRuntime::ControlRuntime(const translational::Config &config,
                               const RecoveryConfig &recovery)
: controller_(config), recovery_(recovery)
{
}

void ControlRuntime::reset() noexcept
{
  controller_.reset();
}

ControlOutput ControlRuntime::update(
  const translational::MeasuredState &measured,
  const translational::ReferenceHorizon &horizon,
  const translational::ReferencePoint &reference)
{
  ControlOutput output;
  output.result = controller_.update(measured, horizon);
  if (output.result.valid) {
    output.command = output.result.control;
    return output;
  }

  output.command = recoveryCommand(measured, reference);
  output.recovery_active = true;
  controller_.setInputMemoryForNextSolve(output.command);
  return output;
}

translational::Vector3 ControlRuntime::recoveryCommand(
  const translational::MeasuredState &measured,
  const translational::ReferencePoint &reference) const noexcept
{
  translational::Vector3 command{
    -recovery_.velocity_gain * measured.velocity[0], -recovery_.velocity_gain * measured.velocity[1],
    recovery_.position_gain_z * (reference.position[2] - measured.position[2]) -
    recovery_.velocity_gain * measured.velocity[2]};

  const double horizontal = std::hypot(command[0], command[1]);
  if (horizontal > recovery_.max_acceleration_xy) {
    const double scale = recovery_.max_acceleration_xy / horizontal;
    command[0] *= scale;
    command[1] *= scale;
  }
  command[2] = std::clamp(command[2], -recovery_.max_acceleration_z, recovery_.max_acceleration_z);
  return command;
}

}  // namespace mpc_controller::application
