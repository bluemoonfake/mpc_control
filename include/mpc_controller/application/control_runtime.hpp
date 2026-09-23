#pragma once

#include "mpc_controller/domain/control/mpc.hpp"

namespace mpc_controller::application
{

struct RecoveryConfig
{
  double velocity_gain = 1.0;
  double position_gain_z = 0.5;
  double max_acceleration_xy = 2.5;
  double max_acceleration_z = 1.5;
};

struct ControlOutput
{
  translational::UpdateResult result{};
  translational::Vector3 command{};
  bool recovery_active = false;
};

class ControlRuntime final
{
public:
  ControlRuntime(const translational::Config &config,
                 const RecoveryConfig &recovery);

  void reset() noexcept;

  ControlOutput update(
    const translational::MeasuredState &measured,
    const translational::ReferenceHorizon &horizon,
    const translational::ReferencePoint &reference);

private:
  translational::Vector3 recoveryCommand(
    const translational::MeasuredState &measured,
    const translational::ReferencePoint &reference) const noexcept;

  translational::TranslationalMpc controller_;
  RecoveryConfig recovery_;
};

}  // namespace mpc_controller::application
