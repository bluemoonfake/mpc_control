#pragma once

#include "mpc_controller/domain/mission/loader.hpp"
#include "mpc_controller/domain/mission/trajectory.hpp"

#include <string>

namespace mpc_controller::application
{

class MissionRuntime final
{
public:
  explicit MissionRuntime(const mission::MissionTrajectory::Config &config);

  mission::MissionTrajectory &trajectory() noexcept { return trajectory_; }
  const mission::MissionTrajectory &trajectory() const noexcept
  {
    return trajectory_;
  }

  mission::RuntimeMissionLoader::Result loadAndStart(
    const std::string &mission_path, double now_seconds);
  void abortToHold() noexcept;

private:
  mission::MissionTrajectory trajectory_;
  mission::RuntimeMissionLoader loader_;
};

}  // namespace mpc_controller::application
