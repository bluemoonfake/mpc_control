#pragma once

#include "mpc_controller/mission/mission.hpp"

#include <array>
#include <cstddef>
#include <string>
#include <vector>

namespace mpc_controller::mission {

struct TrajectorySample {
  std::array<double, 3> position{0.0, 0.0, 0.0};
  std::array<double, 3> velocity{0.0, 0.0, 0.0};
  std::array<double, 3> acceleration{0.0, 0.0, 0.0};
  double yaw = 0.0;
  double yaw_rate = 0.0;
};

struct ReferenceParameters {
  std::array<double, 3> hold_position{0.0, 0.0, 1.0};
  double hold_yaw_rad = 0.0;
};

class MissionReferenceGenerator {
public:
  struct Config {
    ReferenceParameters reference;
    double acceptance_radius_m = 2.5;
    double speed_override_m_s = 0.0;
    double horizon_seconds = 3.0;
    double sample_period_seconds = 0.1;
  };

  struct VehicleState {
    std::array<double, 3> position{};
    std::array<double, 3> velocity{};
    double yaw = 0.0;
    bool valid = false;
  };

  struct Waypoint {
    std::string id;
    ItemType type = ItemType::Unknown;
    std::array<double, 3> position{0.0, 0.0, 1.0};
    double horizontal_speed = 4.0;
    double vertical_speed = 1.5;
    double hold_duration_s = 0.0;
    double heading_rad = NAN;
    double max_heading_rate_rad_s = 1.0471975511965976;
    double maximum_acceleration_m_s2 = NAN;
    double maximum_jerk_m_s3 = NAN;
    bool return_to_mission_start_xy = false;
  };

  struct Update {
    std::vector<TrajectorySample> samples;
    bool waypoint_reached = false;
    bool mission_completed = false;
    std::size_t reached_waypoint_index = 0;
    double distance_to_waypoint_m = 0.0;
    bool crossed_finish_plane = false;
  };

  explicit MissionReferenceGenerator(Config config);

  bool setMission(const Mission &mission, std::string &error);
  void updateVehicleState(const VehicleState &state) noexcept;
  void invalidateVehicleState() noexcept;
  void captureHoldFromVehicle() noexcept;
  bool start(double now_seconds) noexcept;
  void reset() noexcept;
  Update update(double now_seconds);

  const std::vector<Waypoint> &waypoints() const noexcept { return waypoints_; }
  std::size_t currentWaypointIndex() const noexcept { return waypoint_index_; }
  bool active() const noexcept { return active_; }

private:
  double legDuration(const std::array<double, 3> &from,
                     const Waypoint &to) const noexcept;
  TrajectorySample sample(double now_seconds,
                          double horizon_offset_seconds) const noexcept;

  Config config_;
  VehicleState vehicle_state_{};
  std::vector<Waypoint> waypoints_;
  std::size_t waypoint_index_ = 0;
  std::array<double, 3> leg_start_position_{0.0, 0.0, 1.0};
  double leg_start_yaw_ = 0.0;
  double leg_duration_seconds_ = 0.0;
  double leg_started_at_seconds_ = 0.0;
  bool leg_started_ = false;
  bool active_ = false;
};

} // namespace mpc_controller::mission
