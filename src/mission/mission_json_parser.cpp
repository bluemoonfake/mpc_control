#include "mpc_controller/mission/mission_json_parser.hpp"

#include <fstream>
#include <string>

#include <nlohmann/json.hpp>

namespace mpc_controller::mission {

Mission MissionJsonParser::load(const std::string &source) const {
  Mission parsed_mission;
  std::ifstream file(source);
  if (!file.is_open()) {
    parsed_mission.error = "cannot open file: " + source;
    return parsed_mission;
  }

  nlohmann::json root;
  try {
    root = nlohmann::json::parse(file);

    if (root.contains("version")) {
      parsed_mission.version = root["version"].get<int>();
      if (parsed_mission.version != 1) {
        parsed_mission.error = "unsupported schema version: " +
                               std::to_string(parsed_mission.version);
        return parsed_mission;
      }
    }

    if (!root.contains("mission") || !root["mission"].is_object()) {
      parsed_mission.error = "missing 'mission' object";
      return parsed_mission;
    }
    const auto &mission_json = root["mission"];

    if (mission_json.contains("defaults") &&
        mission_json["defaults"].is_object()) {
      const auto &defaults_json = mission_json["defaults"];
      if (defaults_json.contains("horizontalVelocity")) {
        parsed_mission.defaults.horizontal_velocity_m_s =
            defaults_json["horizontalVelocity"].get<double>();
      }
      if (defaults_json.contains("verticalVelocity")) {
        parsed_mission.defaults.vertical_velocity_m_s =
            defaults_json["verticalVelocity"].get<double>();
      }
      if (defaults_json.contains("maxHeadingRate")) {
        parsed_mission.defaults.max_heading_rate_deg_s =
            defaults_json["maxHeadingRate"].get<double>();
      }
    }

    if (!mission_json.contains("items") || !mission_json["items"].is_array()) {
      parsed_mission.error = "missing 'items' array";
      return parsed_mission;
    }

    for (const auto &item_json : mission_json["items"]) {
      MissionItem item;
      if (item_json.contains("id")) {
        item.id = item_json["id"].get<std::string>();
      }

      const std::string type = item_json.value("type", "");
      if (type == "takeoff") {
        item.type = ItemType::Takeoff;
        if (item_json.contains("z")) {
          item.waypoint.position_enu[2] = item_json["z"].get<double>();
        }
      } else if (type == "navigation") {
        if (item_json.value("navigationType", "") == "waypoint") {
          item.type = ItemType::Waypoint;
          item.waypoint.position_enu[0] = item_json.value("x", 0.0);
          item.waypoint.position_enu[1] = item_json.value("y", 0.0);
          item.waypoint.position_enu[2] = item_json.value("z", 0.0);
          if (item_json.contains("heading")) {
            item.waypoint.heading_rad = item_json["heading"].get<double>();
          }
        }
      } else if (type == "hold") {
        item.type = ItemType::Hold;
        item.hold.duration_seconds = item_json.value("duration", 2.0);
      } else if (type == "changeSettings") {
        item.type = ItemType::ChangeSettings;
        item.settings.reset_all = item_json.value("resetAll", false);
        if (item_json.contains("horizontalVelocity")) {
          item.settings.horizontal_velocity_m_s =
              item_json["horizontalVelocity"].get<double>();
        }
        if (item_json.contains("verticalVelocity")) {
          item.settings.vertical_velocity_m_s =
              item_json["verticalVelocity"].get<double>();
        }
        if (item_json.contains("maxHeadingRate")) {
          item.settings.max_heading_rate_deg_s =
              item_json["maxHeadingRate"].get<double>();
        }
      } else if (type == "land") {
        item.type = ItemType::Land;
      } else if (type == "rtl") {
        item.type = ItemType::Rtl;
      }
      parsed_mission.items.push_back(item);
    }
  } catch (const nlohmann::json::exception &error) {
    parsed_mission.error = std::string("JSON mission error: ") + error.what();
    return parsed_mission;
  }

  if (parsed_mission.items.empty()) {
    parsed_mission.error = "mission has no items";
    return parsed_mission;
  }

  parsed_mission.valid = true;
  return parsed_mission;
}

} // namespace mpc_controller::mission
