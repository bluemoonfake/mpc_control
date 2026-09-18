#pragma once

#include "mpc_controller/mission/mission_json_parser.hpp"
#include "mpc_controller/mission/mission_trajectory.hpp"

#include <string>

namespace mpc_controller::mission {

// Executes the replace/load/start sequence as one operation. A failed request
// leaves no executable mission, so a previous trajectory cannot be restarted.
class RuntimeMissionLoader final {
public:
  struct Result {
    bool success = false;
    std::string message;
  };

  explicit RuntimeMissionLoader(MissionReferenceGenerator &generator)
      : generator_(generator) {}

  // Stops the active trajectory, captures the admitted vehicle pose as hold,
  // and removes every executable waypoint.
  void abortToHold();

  Result loadAndStart(const std::string &mission_path,
                      double now_seconds);

private:
  void clearMission();

  MissionJsonParser parser_;
  MissionReferenceGenerator &generator_;
};

} // namespace mpc_controller::mission
