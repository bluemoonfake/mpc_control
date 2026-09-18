#pragma once

#include <string_view>

namespace mpc_controller::mission {

inline constexpr std::string_view kMpcController{"mpc"};
inline constexpr std::string_view kPx4PidController{"px4_pid"};

inline constexpr std::string_view kMpcExternalModeStateTopic{
    "/reference_generator_node/external_mode_active/mpc"};
inline constexpr std::string_view kPx4PidExternalModeStateTopic{
    "/reference_generator_node/external_mode_active/px4_pid"};

constexpr bool isKnownController(std::string_view controller) noexcept {
  return controller == kMpcController || controller == kPx4PidController;
}

} // namespace mpc_controller::mission
