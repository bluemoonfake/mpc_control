#include "mpc_controller/mission/mission_json_parser.hpp"

#include <cmath>
#include <fstream>
#include <string>

#include <nlohmann/json.hpp>

namespace mpc_controller::mission {

namespace {

bool readFinite(const nlohmann::json &object, const char *name, double &output,
                bool allow_zero, std::string &error) {
  if (!object.contains(name)) {
    return true;
  }
  output = object.at(name).get<double>();
  if (std::isfinite(output) && (allow_zero ? output >= 0.0 : output > 0.0)) {
    return true;
  }
  error = std::string(name) + " must be finite and " +
          (allow_zero ? "non-negative" : "positive");
  return false;
}

bool readTpmc(const nlohmann::json &object, double &acceleration, double &jerk,
              std::string &error) {
  if (!object.contains("tpmc"))
    return true;
  const auto &tpmc = object.at("tpmc");
  if (!tpmc.is_object()) {
    error = "tpmc must be an object";
    return false;
  }
  return readFinite(tpmc, "maximumAcceleration", acceleration, false, error) &&
         readFinite(tpmc, "maximumJerk", jerk, false, error);
}

} // namespace

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
      if (!readFinite(defaults_json, "horizontalVelocity",
                      parsed_mission.defaults.horizontal_velocity_m_s, false,
                      parsed_mission.error) ||
          !readFinite(defaults_json, "verticalVelocity",
                      parsed_mission.defaults.vertical_velocity_m_s, false,
                      parsed_mission.error) ||
          !readFinite(defaults_json, "maxHeadingRate",
                      parsed_mission.defaults.max_heading_rate_deg_s, false,
                      parsed_mission.error) ||
          !readTpmc(defaults_json,
                    parsed_mission.defaults.maximum_acceleration_m_s2,
                    parsed_mission.defaults.maximum_jerk_m_s3,
                    parsed_mission.error)) {
        return parsed_mission;
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
        if (!std::isfinite(item.hold.duration_seconds) ||
            item.hold.duration_seconds < 0.0) {
          parsed_mission.error = "duration must be finite and non-negative";
          return parsed_mission;
        }
      } else if (type == "changeSettings") {
        item.type = ItemType::ChangeSettings;
        item.settings.reset_all = item_json.value("resetAll", false);
        if (!readFinite(item_json, "horizontalVelocity",
                        item.settings.horizontal_velocity_m_s, false,
                        parsed_mission.error) ||
            !readFinite(item_json, "verticalVelocity",
                        item.settings.vertical_velocity_m_s, false,
                        parsed_mission.error) ||
            !readFinite(item_json, "maxHeadingRate",
                        item.settings.max_heading_rate_deg_s, false,
                        parsed_mission.error) ||
            !readTpmc(item_json, item.settings.maximum_acceleration_m_s2,
                      item.settings.maximum_jerk_m_s3, parsed_mission.error)) {
          return parsed_mission;
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
