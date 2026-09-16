#pragma once

#include "mpc_controller/mission/mission.hpp"

#include <string>

namespace mpc_controller::mission {

class MissionJsonParser final {
public:
  Mission load(const std::string &source) const;
};

} // namespace mpc_controller::mission
