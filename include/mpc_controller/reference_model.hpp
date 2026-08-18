#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <string>

namespace mpc_controller::reference
{

struct Sample
{
  std::array<double, 3> position{0.0, 0.0, 0.0};
  std::array<double, 3> velocity{0.0, 0.0, 0.0};
  std::array<double, 3> acceleration{0.0, 0.0, 0.0};
  double yaw = 0.0;
  double yaw_rate = 0.0;
};

struct Parameters
{
  std::string type = "mission";
  std::array<double, 3> hold_position{0.0, 0.0, 1.0};
  double hold_yaw_rad = 0.0;
};

inline bool finite(const std::array<double, 3> &value) noexcept
{
  return std::isfinite(value[0]) && std::isfinite(value[1]) && std::isfinite(value[2]);
}

}  // namespace mpc_controller::reference
