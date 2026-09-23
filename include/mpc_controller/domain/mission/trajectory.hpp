#pragma once

#include "mpc_controller/domain/mission/loader.hpp"
#include "mpc_controller/domain/mission/tracker.hpp"
#include "mpc_controller/domain/model/types.hpp"

#include <array>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace mpc_controller::mission {

struct TrajectorySample {
  std::array<double, 3> position{0.0, 0.0, 0.0};
  std::array<double, 3> velocity{0.0, 0.0, 0.0};
  std::array<double, 3> acceleration{0.0, 0.0, 0.0};
  double yaw = 0.0;
  double yaw_rate = 0.0;
  double max_speed_xy = 0.0;
  double max_speed_z = 0.0;
  double max_acceleration_xy = 0.0;
  double max_acceleration_z = 0.0;
  double max_control_rate_xy = 0.0;
  double max_control_rate_z = 0.0;
};

struct ReferenceParameters {
  std::array<double, 3> hold_position{0.0, 0.0, 1.0};
  double hold_yaw_rad = 0.0;
};

class MissionTrajectory {
public:
  struct Config {
    ReferenceParameters reference;
    double acceptance_radius_m = 2.5;
    double acceptance_speed_m_s = 2.0;
    double speed_override_m_s = 0.0;
    double horizon_seconds = 3.0;
    double sample_period_seconds = 0.1;
    double planner_max_speed_xy_m_s = 18.0;
    double planner_max_speed_z_m_s = 3.0;
    double planner_max_acceleration_xy_m_s2 = 6.0;
    double planner_max_acceleration_z_m_s2 = 5.0;
    double planner_max_jerk_xy_m_s3 = 8.0;
    double planner_max_jerk_z_m_s3 = 4.0;
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

  explicit MissionTrajectory(Config config);

  bool setMission(const Mission &mission, std::string &error);
  void updateVehicleState(const VehicleState &state) noexcept;
  void invalidateVehicleState() noexcept;
  void captureHoldFromVehicle() noexcept;
  bool start(double now_seconds) noexcept;
  void reset() noexcept;
  Update update(double now_seconds);

  const std::vector<Waypoint> &waypoints() const noexcept { return waypoints_; }
  std::size_t currentWaypointIndex() const noexcept {
    return active_waypoint_index_;
  }
  bool active() const noexcept { return active_; }

private:
  using KinematicState = TrajectoryCurve::State;

  bool isFlyThroughWaypoint(std::size_t index) const noexcept;
  TrajectoryCurve::Limits curveLimits(std::size_t index) const
      noexcept;
  double nominalDuration(const KinematicState &from,
                         const Waypoint &to) const noexcept;
  KinematicState stopWaypointState(std::size_t index) const noexcept;
  std::optional<TrajectoryCurve> createCurve(
      const KinematicState &start, std::size_t target_index,
      bool clamp_start) const noexcept;
  void prepareCaptureCurve(std::size_t target_index) noexcept;
  void buildWaypointStates() noexcept;
  bool buildActiveCurve(const KinematicState &start,
                          std::size_t target_index) noexcept;
  void bindMissionToMeasuredOrigin() noexcept;
  TrajectorySample sample(double now_seconds,
                          double horizon_offset_seconds) const noexcept;

  Config config_;
  VehicleState vehicle_state_{};
  std::vector<Waypoint> waypoints_;
  std::vector<std::array<double, 3>> mission_local_positions_;
  std::vector<double> mission_local_headings_;
  MissionProgressTracker progress_tracker_;
  std::vector<KinematicState> waypoint_states_;
  std::optional<TrajectoryCurve> active_curve_;
  std::optional<TrajectoryCurve> capture_curve_;
  std::optional<TrajectoryCurve> pending_curve_;
  std::array<double, 3> leg_start_position_{0.0, 0.0, 1.0};
  double leg_start_yaw_ = 0.0;
  double leg_started_at_seconds_ = 0.0;
  std::size_t active_waypoint_index_ = 0;
  std::optional<std::size_t> pending_transition_from_index_;
  bool pending_completion_ = false;
  bool capture_active_ = false;
  bool leg_started_ = false;
  bool active_ = false;
  std::optional<Waypoint> completed_waypoint_;
};

} // namespace mpc_controller::mission
