#include "mpc_controller/application/mission_runtime.hpp"

namespace mpc_controller::application
{

MissionRuntime::MissionRuntime(const mission::MissionTrajectory::Config &config)
: trajectory_(config), loader_(trajectory_)
{
}

mission::RuntimeMissionLoader::Result MissionRuntime::loadAndStart(
  const std::string &mission_path, double now_seconds)
{
  return loader_.loadAndStart(mission_path, now_seconds);
}

void MissionRuntime::abortToHold() noexcept
{
  loader_.abortToHold();
}

}  // namespace mpc_controller::application
