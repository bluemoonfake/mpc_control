#include "mpc_controller/mission/runtime_mission_loader.hpp"

namespace mpc_controller::mission {

void RuntimeMissionLoader::clearMission() {
  Mission invalid_mission;
  std::string ignored_error;
  generator_.setMission(invalid_mission, ignored_error);
}

void RuntimeMissionLoader::abortToHold() {
  generator_.reset();
  clearMission();
}

RuntimeMissionLoader::Result
RuntimeMissionLoader::loadAndStart(const std::string &mission_path,
                                   double now_seconds) {
  // Capture the current valid pose, then clear the old mission before parsing.
  // A parse failure therefore cannot leave old waypoints executable.
  abortToHold();

  if (mission_path.empty()) {
    return {false, "mission path is empty"};
  }

  const auto mission = parser_.load(mission_path);
  std::string error;
  if (!generator_.setMission(mission, error)) {
    return {false, error.empty() ? "mission validation failed" : error};
  }
  if (!generator_.start(now_seconds)) {
    clearMission();
    return {false, "mission could not be started"};
  }
  return {true, "mission loaded and started"};
}

} // namespace mpc_controller::mission
