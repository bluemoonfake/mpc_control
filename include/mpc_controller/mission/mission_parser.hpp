#pragma once

// Mission JSON parser conforming to
// third_party/px4-ros2-interface-lib/mission/schema.yaml (version 1).
//
// Waypoints use the LOCAL ENU frame directly so they map 1:1 to the ENU
// reference produced by the existing reference_generator_node pipeline.
// Global WGS84 support is intentionally omitted for now; it would require the
// PX4 map-projection reference that is only available inside the flight stack.

#include <array>
#include <cmath>
#include <fstream>
#include <string>
#include <vector>

#include <px4_ros2/third_party/nlohmann/json.hpp>

namespace mpc_controller::mission
{

inline constexpr double kDegreesToRadians = 0.017453292519943295;

// --------------------------------------------------------------------------
// Data model
// --------------------------------------------------------------------------

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
  double maximum_acceleration_m_s2 = 2.5;
  double maximum_jerk_m_s3 = 5.0;
};

struct WaypointData
{
  std::array<double, 3> position_enu{};   // [east, north, up] metres
  double heading_rad = NAN;               // optional target heading
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

// --------------------------------------------------------------------------
// Parser
// --------------------------------------------------------------------------

inline Mission parse(const std::string & json_path)
{
  Mission mission;
  std::ifstream file(json_path);
  if (!file.is_open()) {
    mission.error = "cannot open file: " + json_path;
    return mission;
  }

  nlohmann::json root;
  try {
    root = nlohmann::json::parse(file);
  } catch (const nlohmann::json::parse_error & e) {
    mission.error = std::string("JSON parse error: ") + e.what();
    return mission;
  }

  // Version check
  if (root.contains("version")) {
    mission.version = root["version"].get<int>();
    if (mission.version != 1) {
      mission.error = "unsupported schema version: " + std::to_string(mission.version);
      return mission;
    }
  }

  if (!root.contains("mission") || !root["mission"].is_object()) {
    mission.error = "missing 'mission' object";
    return mission;
  }
  const auto & m = root["mission"];

  // Defaults
  if (m.contains("defaults") && m["defaults"].is_object()) {
    const auto & d = m["defaults"];
    if (d.contains("horizontalVelocity")) {
      mission.defaults.horizontal_velocity_m_s = d["horizontalVelocity"].get<double>();
    }
    if (d.contains("verticalVelocity")) {
      mission.defaults.vertical_velocity_m_s = d["verticalVelocity"].get<double>();
    }
    if (d.contains("maxHeadingRate")) {
      mission.defaults.max_heading_rate_deg_s = d["maxHeadingRate"].get<double>();
    }
    if (d.contains("tpmc") && d["tpmc"].is_object()) {
      const auto & trajectory = d["tpmc"];
      if (trajectory.contains("maximumAcceleration")) {
        mission.defaults.maximum_acceleration_m_s2 =
          trajectory["maximumAcceleration"].get<double>();
      }
      if (trajectory.contains("maximumJerk")) {
        mission.defaults.maximum_jerk_m_s3 =
          trajectory["maximumJerk"].get<double>();
      }
    }
  }

  // Items
  if (!m.contains("items") || !m["items"].is_array()) {
    mission.error = "missing 'items' array";
    return mission;
  }

  for (const auto & item_json : m["items"]) {
    MissionItem item;
    if (item_json.contains("id")) {
      item.id = item_json["id"].get<std::string>();
    }

    const std::string type_str = item_json.value("type", "");

    if (type_str == "takeoff") {
      item.type = ItemType::Takeoff;
      // Takeoff altitude may be specified via z; default to current hold alt.
      if (item_json.contains("z")) {
        item.waypoint.position_enu[2] = item_json["z"].get<double>();
      }

    } else if (type_str == "navigation") {
      const std::string nav_type = item_json.value("navigationType", "");
      if (nav_type == "waypoint") {
        item.type = ItemType::Waypoint;
        // schema.yaml: x, y, z are required for waypoints.
        // We support both "global" and "local" frame. For local frame the
        // coordinates are directly ENU [east, north, up].
        const std::string frame = item_json.value("frame", "local");
        if (frame == "local") {
          item.waypoint.position_enu[0] = item_json.value("x", 0.0);
          item.waypoint.position_enu[1] = item_json.value("y", 0.0);
          item.waypoint.position_enu[2] = item_json.value("z", 0.0);
        } else if (frame == "global") {
          // For global frame, store lat/lon/alt and flag for later conversion.
          // In this initial implementation we treat x=lat, y=lon, z=alt(MSL)
          // but conversion to ENU requires a reference point. The node will
          // handle this when it receives the home position.
          item.waypoint.position_enu[0] = item_json.value("x", 0.0);
          item.waypoint.position_enu[1] = item_json.value("y", 0.0);
          item.waypoint.position_enu[2] = item_json.value("z", 0.0);
        }
        if (item_json.contains("heading")) {
          item.waypoint.heading_rad = item_json["heading"].get<double>();
        }
        if (item_json.contains("tpmc") && item_json["tpmc"].is_object()) {
          const auto & trajectory = item_json["tpmc"];
          if (trajectory.contains("headingDeg")) {
            item.waypoint.heading_rad =
              trajectory["headingDeg"].get<double>() * kDegreesToRadians;
          }
        }
      } else {
        item.type = ItemType::Unknown;
      }

    } else if (type_str == "hold") {
      item.type = ItemType::Hold;
      item.hold.duration_seconds = item_json.value("duration", 2.0);

    } else if (type_str == "changeSettings") {
      item.type = ItemType::ChangeSettings;
      item.settings.reset_all = item_json.value("resetAll", false);
      if (item_json.contains("horizontalVelocity")) {
        item.settings.horizontal_velocity_m_s = item_json["horizontalVelocity"].get<double>();
      }
      if (item_json.contains("verticalVelocity")) {
        item.settings.vertical_velocity_m_s = item_json["verticalVelocity"].get<double>();
      }
      if (item_json.contains("maxHeadingRate")) {
        item.settings.max_heading_rate_deg_s = item_json["maxHeadingRate"].get<double>();
      }
      if (item_json.contains("tpmc") && item_json["tpmc"].is_object()) {
        const auto & trajectory = item_json["tpmc"];
        if (trajectory.contains("maximumAcceleration")) {
          item.settings.maximum_acceleration_m_s2 =
            trajectory["maximumAcceleration"].get<double>();
        }
        if (trajectory.contains("maximumJerk")) {
          item.settings.maximum_jerk_m_s3 = trajectory["maximumJerk"].get<double>();
        }
      }

    } else if (type_str == "land") {
      item.type = ItemType::Land;
    } else if (type_str == "rtl") {
      item.type = ItemType::Rtl;
    } else {
      item.type = ItemType::Unknown;
    }

    mission.items.push_back(item);
  }

  if (mission.items.empty()) {
    mission.error = "mission has no items";
    return mission;
  }

  const auto finitePositive = [](double value) {
    return std::isfinite(value) && value > 0.0;
  };
  if (!finitePositive(mission.defaults.horizontal_velocity_m_s) ||
      !finitePositive(mission.defaults.vertical_velocity_m_s) ||
      !finitePositive(mission.defaults.max_heading_rate_deg_s) ||
      !finitePositive(mission.defaults.maximum_acceleration_m_s2) ||
      !finitePositive(mission.defaults.maximum_jerk_m_s3)) {
    mission.error = "mission defaults must be finite and positive";
    return mission;
  }

  mission.valid = true;
  return mission;
}

// --------------------------------------------------------------------------
// Helpers
// --------------------------------------------------------------------------

/// Extract only the waypoint items with their ENU positions and the defaults
/// velocity profile for trajectory generation.
inline std::vector<std::array<double, 3>> extractWaypoints(const Mission & mission)
{
  std::vector<std::array<double, 3>> waypoints;
  for (const auto & item : mission.items) {
    if (item.type == ItemType::Waypoint) {
      waypoints.push_back(item.waypoint.position_enu);
    }
  }
  return waypoints;
}

}  // namespace mpc_controller::mission
