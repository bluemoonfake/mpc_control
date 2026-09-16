#include "mpc_controller/mission/mission_trajectory.hpp"

#include <algorithm>
#include <cmath>

namespace mpc_controller::mission {

MissionReferenceGenerator::MissionReferenceGenerator(Config config)
    : config_(config), leg_start_position_(config.reference.hold_position),
      leg_start_yaw_(config.reference.hold_yaw_rad) {}

bool MissionReferenceGenerator::setMission(const Mission &mission,
                                           std::string &error) {
  waypoints_.clear();
  active_ = false;
  leg_started_ = false;
  waypoint_index_ = 0;
  if (!mission.valid) {
    error = mission.error.empty() ? "invalid mission" : mission.error;
    return false;
  }

  const double horizontal_speed =
      config_.speed_override_m_s > 0.0
          ? config_.speed_override_m_s
          : mission.defaults.horizontal_velocity_m_s;
  const double vertical_speed = mission.defaults.vertical_velocity_m_s;

  for (const auto &item : mission.items) {
    Waypoint waypoint;
    if (item.type == ItemType::Takeoff) {
      waypoint.id = item.id.empty() ? "takeoff" : item.id;
      waypoint.position = config_.reference.hold_position;
      if (std::isfinite(item.waypoint.position_enu[2]) &&
          item.waypoint.position_enu[2] > config_.reference.hold_position[2]) {
        waypoint.position[2] = item.waypoint.position_enu[2];
      }
      waypoint.horizontal_speed = 0.0;
      waypoint.vertical_speed = std::clamp(vertical_speed, 0.5, 1.2);
      waypoints_.push_back(waypoint);
    } else if (item.type == ItemType::Waypoint) {
      waypoint.id = item.id;
      waypoint.position = item.waypoint.position_enu;
      waypoint.horizontal_speed = horizontal_speed;
      waypoint.vertical_speed = vertical_speed;
      waypoints_.push_back(waypoint);
    } else if (item.type == ItemType::Hold && !waypoints_.empty()) {
      waypoints_.back().hold_duration_s = item.hold.duration_seconds;
    } else if (item.type == ItemType::Land) {
      waypoint.id = item.id.empty() ? "landing" : item.id;
      waypoint.position = waypoints_.empty() ? config_.reference.hold_position
                                             : waypoints_.back().position;
      waypoint.position[2] = 0.0;
      waypoint.horizontal_speed = 0.0;
      waypoint.vertical_speed = std::clamp(vertical_speed, 0.4, 0.8);
      waypoints_.push_back(waypoint);
    }
  }

  if (waypoints_.empty()) {
    error = "mission contains no executable waypoints";
    return false;
  }
  error.clear();
  return true;
}

void MissionReferenceGenerator::updateVehicleState(
    const VehicleState &state) noexcept {
  vehicle_state_ = state;
}

void MissionReferenceGenerator::invalidateVehicleState() noexcept {
  vehicle_state_.valid = false;
}

void MissionReferenceGenerator::captureHoldFromVehicle() noexcept {
  if (!vehicle_state_.valid) {
    return;
  }
  config_.reference.hold_position = vehicle_state_.position;
  config_.reference.hold_yaw_rad = vehicle_state_.yaw;
}

bool MissionReferenceGenerator::start(double now_seconds) noexcept {
  if (waypoints_.empty() || !std::isfinite(now_seconds)) {
    return false;
  }
  waypoint_index_ = 0;
  leg_start_position_ = vehicle_state_.valid ? vehicle_state_.position
                                             : config_.reference.hold_position;
  leg_start_yaw_ = vehicle_state_.valid ? vehicle_state_.yaw
                                        : config_.reference.hold_yaw_rad;
  leg_duration_seconds_ = legDuration(leg_start_position_, waypoints_.front());
  leg_started_at_seconds_ = now_seconds;
  leg_started_ = true;
  active_ = true;
  return true;
}

void MissionReferenceGenerator::reset() noexcept {
  waypoint_index_ = 0;
  active_ = false;
  leg_started_ = false;
  captureHoldFromVehicle();
}

double
MissionReferenceGenerator::legDuration(const std::array<double, 3> &from,
                                       const Waypoint &to) const noexcept {
  const double dx = to.position[0] - from[0];
  const double dy = to.position[1] - from[1];
  const double dz = to.position[2] - from[2];
  const double horizontal_time =
      std::hypot(dx, dy) > 1e-3 && to.horizontal_speed > 0.0
          ? std::hypot(dx, dy) / to.horizontal_speed
          : 0.0;
  const double vertical_time = std::abs(dz) > 1e-3 && to.vertical_speed > 0.0
                                   ? std::abs(dz) / to.vertical_speed
                                   : 0.0;
  return std::max({horizontal_time, vertical_time, 0.5}) + to.hold_duration_s;
}

MissionReferenceGenerator::Update
MissionReferenceGenerator::update(double now_seconds) {
  Update result;
  if (!std::isfinite(now_seconds) || config_.sample_period_seconds <= 0.0 ||
      config_.horizon_seconds < config_.sample_period_seconds) {
    return result;
  }

  if (active_ && leg_started_ && waypoint_index_ < waypoints_.size()) {
    const auto &target = waypoints_[waypoint_index_];
    const double elapsed = std::max(0.0, now_seconds - leg_started_at_seconds_);
    double distance = 1000.0;
    bool crossed_finish_plane = false;
    if (vehicle_state_.valid) {
      const double ex = vehicle_state_.position[0] - target.position[0];
      const double ey = vehicle_state_.position[1] - target.position[1];
      const double ez = vehicle_state_.position[2] - target.position[2];
      distance = std::sqrt(ex * ex + ey * ey + ez * ez);
      const double dx = target.position[0] - leg_start_position_[0];
      const double dy = target.position[1] - leg_start_position_[1];
      const double dz = target.position[2] - leg_start_position_[2];
      const double length = std::sqrt(dx * dx + dy * dy + dz * dz);
      if (length > 1e-3) {
        crossed_finish_plane =
            ex * dx / length + ey * dy / length + ez * dz / length >= 0.0;
      }
    }

    const bool landing = target.id == "landing" || target.position[2] <= 0.1;
    const double acceptance = landing ? 0.35 : config_.acceptance_radius_m;
    // Timing and finish-plane crossing are useful diagnostics, but neither is
    // evidence that the vehicle reached a waypoint.  Advancing on either one
    // makes a tracking error turn into a new reference segment and silently
    // cuts corners in a mission such as the slalom benchmark.
    result.distance_to_waypoint_m = distance;
    result.crossed_finish_plane = crossed_finish_plane;
    const bool reached =
        distance <= acceptance && elapsed >= target.hold_duration_s;

    if (reached) {
      result.waypoint_reached = true;
      result.reached_waypoint_index = waypoint_index_;
      const double dx = target.position[0] - leg_start_position_[0];
      const double dy = target.position[1] - leg_start_position_[1];
      const double completed_yaw =
          std::hypot(dx, dy) > 0.5 ? std::atan2(dy, dx) : leg_start_yaw_;
      ++waypoint_index_;
      if (waypoint_index_ >= waypoints_.size()) {
        config_.reference.hold_position = target.position;
        active_ = false;
        leg_started_ = false;
        result.mission_completed = true;
      } else {
        leg_start_position_ = target.position;
        leg_start_yaw_ = completed_yaw;
        leg_duration_seconds_ =
            legDuration(leg_start_position_, waypoints_[waypoint_index_]);
        leg_started_at_seconds_ = now_seconds;
      }
    }
  }

  const std::size_t count =
      static_cast<std::size_t>(config_.horizon_seconds /
                               config_.sample_period_seconds) +
      1U;
  result.samples.reserve(count);
  for (std::size_t index = 0; index < count; ++index) {
    result.samples.push_back(
        sample(now_seconds, index * config_.sample_period_seconds));
  }
  return result;
}

TrajectorySample MissionReferenceGenerator::sample(
    double now_seconds, double horizon_offset_seconds) const noexcept {
  TrajectorySample point;
  if (!active_ || !leg_started_ || waypoint_index_ >= waypoints_.size()) {
    point.position = config_.reference.hold_position;
    point.yaw = config_.reference.hold_yaw_rad;
    return point;
  }

  const double elapsed = std::max(0.0, now_seconds - leg_started_at_seconds_);
  const auto &target = waypoints_[waypoint_index_];
  const bool takeoff_or_landing =
      target.id == "takeoff" || target.id == "landing";
  if (takeoff_or_landing) {
    const double alpha =
        leg_duration_seconds_ > 1e-3
            ? std::min(1.0, (elapsed + horizon_offset_seconds) /
                                leg_duration_seconds_)
            : 1.0;
    const double dx = target.position[0] - leg_start_position_[0];
    const double dy = target.position[1] - leg_start_position_[1];
    const double dz = target.position[2] - leg_start_position_[2];
    const double position_scale =
        alpha < 1.0 ? 3.0 * alpha * alpha - 2.0 * alpha * alpha * alpha : 1.0;
    const double velocity_scale =
        alpha < 1.0 ? 6.0 * alpha * (1.0 - alpha) : 0.0;
    point.position = {leg_start_position_[0] + position_scale * dx,
                      leg_start_position_[1] + position_scale * dy,
                      leg_start_position_[2] + position_scale * dz};
    if (alpha < 1.0 && leg_duration_seconds_ > 1e-3) {
      point.velocity = {velocity_scale * dx / leg_duration_seconds_,
                        velocity_scale * dy / leg_duration_seconds_,
                        velocity_scale * dz / leg_duration_seconds_};
      point.yaw = std::hypot(dx, dy) > 1e-3 ? std::atan2(dy, dx)
                                            : config_.reference.hold_yaw_rad;
    } else {
      point.yaw = config_.reference.hold_yaw_rad;
    }
    return point;
  }

  const double remaining_current =
      std::max(0.0, leg_duration_seconds_ - elapsed);
  if (horizon_offset_seconds <= remaining_current ||
      waypoint_index_ + 1 >= waypoints_.size()) {
    const double alpha =
        leg_duration_seconds_ > 1e-3
            ? std::min(1.0, (elapsed + horizon_offset_seconds) /
                                leg_duration_seconds_)
            : 1.0;
    const double dx = target.position[0] - leg_start_position_[0];
    const double dy = target.position[1] - leg_start_position_[1];
    const double dz = target.position[2] - leg_start_position_[2];
    point.position = {leg_start_position_[0] + alpha * dx,
                      leg_start_position_[1] + alpha * dy,
                      leg_start_position_[2] + alpha * dz};
    if (alpha < 1.0 && leg_duration_seconds_ > 1e-3) {
      point.velocity = {dx / leg_duration_seconds_, dy / leg_duration_seconds_,
                        dz / leg_duration_seconds_};
      point.yaw = std::hypot(dx, dy) > 1e-3 ? std::atan2(dy, dx)
                                            : config_.reference.hold_yaw_rad;
    } else {
      point.yaw = config_.reference.hold_yaw_rad;
    }
    return point;
  }

  // A waypoint is advanced solely by measured acceptance in update().  The
  // optimization horizon must respect that same state machine: previewing an
  // unaccepted successor lets a long horizon pull the controller through a
  // corner (or even toward return/landing) before the current target is
  // reached.  Hold this target after its nominal arrival time; update() will
  // expose the next leg only after it observes actual arrival.
  point.position = target.position;
  const double dx = target.position[0] - leg_start_position_[0];
  const double dy = target.position[1] - leg_start_position_[1];
  point.yaw = std::hypot(dx, dy) > 1e-3 ? std::atan2(dy, dx)
                                        : leg_start_yaw_;
  return point;
}

} // namespace mpc_controller::mission
