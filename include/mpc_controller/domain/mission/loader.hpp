#pragma once

#include <array>
#include <cmath>
#include <string>
#include <string_view>
#include <vector>

namespace mpc_controller::mission
{

// Mission schema model. Parsing and runtime loading share this contract so
// callers do not need to know which implementation owns the data.
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
  double maximum_acceleration_m_s2 = NAN;
  double maximum_jerk_m_s3 = NAN;
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
  double maximum_acceleration_m_s2 = NAN;
  double maximum_jerk_m_s3 = NAN;
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

class MissionJsonParser final
{
public:
  Mission load(const std::string &source) const;
};

inline constexpr std::string_view kMpcController{"mpc"};
inline constexpr std::string_view kPx4PidController{"px4_pid"};

inline constexpr std::string_view kMpcExternalModeStateTopic{
  "/reference_generator_node/external_mode_active/mpc"};
inline constexpr std::string_view kPx4PidExternalModeStateTopic{
  "/reference_generator_node/external_mode_active/px4_pid"};

constexpr bool isKnownController(std::string_view controller) noexcept
{
  return controller == kMpcController || controller == kPx4PidController;
}

class MissionTrajectory;

// Executes the replace/load/start sequence as one operation. A failed request
// leaves no executable mission, so a previous trajectory cannot be restarted.
class RuntimeMissionLoader final
{
public:
  struct Result
  {
    bool success = false;
    std::string message;
  };

  explicit RuntimeMissionLoader(MissionTrajectory &generator)
  : generator_(generator)
  {
  }

  // Stops the active trajectory, captures the admitted vehicle pose as hold,
  // and removes every executable waypoint.
  void abortToHold();

  Result loadAndStart(const std::string &mission_path, double now_seconds);

private:
  void clearMission();

  MissionJsonParser parser_;
  MissionTrajectory &generator_;
};

}  // namespace mpc_controller::mission
