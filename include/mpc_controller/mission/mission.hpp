#pragma once

#include <array>
#include <cmath>
#include <string>
#include <vector>

namespace mpc_controller::mission
{

enum class ItemType
{
  Takeoff,
  Waypoint,
  Hold,
  ChangeSettings,
  Land,
  Rtl,
  Unknown
};

struct Defaults
{
  double horizontal_velocity_m_s = 4.0;
  double vertical_velocity_m_s = 1.5;
  double max_heading_rate_deg_s = 60.0;
};

struct WaypointData
{
  std::array<double, 3> position_enu{};
  double heading_rad = NAN;
};

struct HoldData
{
  double duration_seconds = 1.0;
};

struct ChangeSettingsData
{
  bool reset_all = false;
  double horizontal_velocity_m_s = NAN;
  double vertical_velocity_m_s = NAN;
  double max_heading_rate_deg_s = NAN;
};

struct MissionItem
{
  ItemType type = ItemType::Unknown;
  std::string id;
  WaypointData waypoint;
  HoldData hold;
  ChangeSettingsData settings;
};

struct Mission
{
  int version = 1;
  Defaults defaults;
  std::vector<MissionItem> items;
  bool valid = false;
  std::string error;
};

}  // namespace mpc_controller::mission
