#include "mpc_controller/domain/mission/trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace mpc_controller::mission {

namespace {

struct ActiveSettings {
  double horizontal_speed;
  double vertical_speed;
  double max_heading_rate_rad_s;
  double maximum_acceleration_m_s2;
  double maximum_jerk_m_s3;
};

ActiveSettings missionSettings(const Defaults &defaults,
                               double speed_override_m_s) {
  return {speed_override_m_s > 0.0 ? speed_override_m_s
                                   : defaults.horizontal_velocity_m_s,
          defaults.vertical_velocity_m_s,
          defaults.max_heading_rate_deg_s * M_PI / 180.0,
          defaults.maximum_acceleration_m_s2, defaults.maximum_jerk_m_s3};
}

void applySettings(const ChangeSettingsData &change, const Defaults &defaults,
                   double speed_override_m_s, ActiveSettings &settings) {
  if (change.reset_all) {
    settings = missionSettings(defaults, speed_override_m_s);
  }
  if (std::isfinite(change.horizontal_velocity_m_s)) {
    settings.horizontal_speed = change.horizontal_velocity_m_s;
  }
  if (std::isfinite(change.vertical_velocity_m_s)) {
    settings.vertical_speed = change.vertical_velocity_m_s;
  }
  if (std::isfinite(change.max_heading_rate_deg_s)) {
    settings.max_heading_rate_rad_s =
        change.max_heading_rate_deg_s * M_PI / 180.0;
  }
  if (std::isfinite(change.maximum_acceleration_m_s2)) {
    settings.maximum_acceleration_m_s2 = change.maximum_acceleration_m_s2;
  }
  if (std::isfinite(change.maximum_jerk_m_s3)) {
    settings.maximum_jerk_m_s3 = change.maximum_jerk_m_s3;
  }
}

void assignSettings(MissionTrajectory::Waypoint &waypoint,
                    const ActiveSettings &settings) {
  waypoint.horizontal_speed = settings.horizontal_speed;
  waypoint.vertical_speed = settings.vertical_speed;
  waypoint.max_heading_rate_rad_s = settings.max_heading_rate_rad_s;
  waypoint.maximum_acceleration_m_s2 = settings.maximum_acceleration_m_s2;
  waypoint.maximum_jerk_m_s3 = settings.maximum_jerk_m_s3;
}

bool validWaypoint(const MissionTrajectory::Waypoint &waypoint) {
  const auto finite = [](double value) { return std::isfinite(value); };
  return std::all_of(waypoint.position.begin(), waypoint.position.end(),
                     finite) &&
         finite(waypoint.horizontal_speed) &&
         waypoint.horizontal_speed >= 0.0 && finite(waypoint.vertical_speed) &&
         waypoint.vertical_speed >= 0.0 &&
         finite(waypoint.max_heading_rate_rad_s) &&
         waypoint.max_heading_rate_rad_s > 0.0 &&
         finite(waypoint.hold_duration_s) && waypoint.hold_duration_s >= 0.0 &&
         (std::isnan(waypoint.maximum_acceleration_m_s2) ||
          (finite(waypoint.maximum_acceleration_m_s2) &&
           waypoint.maximum_acceleration_m_s2 > 0.0)) &&
         (std::isnan(waypoint.maximum_jerk_m_s3) ||
          (finite(waypoint.maximum_jerk_m_s3) &&
           waypoint.maximum_jerk_m_s3 > 0.0));
}

double requestedLimit(double value) noexcept {
  return std::isfinite(value) && value > 0.0 ? value : 0.0;
}

void assignTrackingLimits(TrajectorySample &sample,
                          const MissionTrajectory::Waypoint &waypoint) noexcept {
  sample.max_speed_xy = requestedLimit(waypoint.horizontal_speed);
  sample.max_speed_z = requestedLimit(waypoint.vertical_speed);
  sample.max_acceleration_xy = requestedLimit(waypoint.maximum_acceleration_m_s2);
  sample.max_acceleration_z = requestedLimit(waypoint.maximum_acceleration_m_s2);
  sample.max_control_rate_xy = requestedLimit(waypoint.maximum_jerk_m_s3);
  sample.max_control_rate_z = requestedLimit(waypoint.maximum_jerk_m_s3);
}

using Vector3 = std::array<double, 3>;

double norm(const Vector3 &value) noexcept {
  return std::sqrt(value[0] * value[0] + value[1] * value[1] +
                   value[2] * value[2]);
}

Vector3 subtract(const Vector3 &left, const Vector3 &right) noexcept {
  return {left[0] - right[0], left[1] - right[1], left[2] - right[2]};
}

Vector3 scale(const Vector3 &value, double factor) noexcept {
  return {value[0] * factor, value[1] * factor, value[2] * factor};
}

Vector3 add(const Vector3 &left, const Vector3 &right) noexcept {
  return {left[0] + right[0], left[1] + right[1], left[2] + right[2]};
}

Vector3 normalized(const Vector3 &value) noexcept {
  const double length = norm(value);
  return length > 1.0e-9 ? scale(value, 1.0 / length) : Vector3{};
}

double normalizeAngle(double yaw) noexcept {
  return std::atan2(std::sin(yaw), std::cos(yaw));
}

double clampPositive(double value, double fallback) noexcept {
  return std::isfinite(value) && value > 0.0 ? value : fallback;
}

void clampAcceleration(Vector3 &value, double max_xy, double max_z) noexcept {
  const double xy = std::hypot(value[0], value[1]);
  if (xy > max_xy && xy > 1.0e-9) {
    const double factor = max_xy / xy;
    value[0] *= factor;
    value[1] *= factor;
  }
  value[2] = std::clamp(value[2], -max_z, max_z);
}

} // namespace

MissionTrajectory::MissionTrajectory(Config config)
    : config_(config), progress_tracker_(config.acceptance_speed_m_s),
      leg_start_position_(config.reference.hold_position),
      leg_start_yaw_(config.reference.hold_yaw_rad) {}

bool MissionTrajectory::setMission(const Mission &mission,
                                           std::string &error) {
  waypoints_.clear();
  mission_local_positions_.clear();
  mission_local_headings_.clear();
  progress_tracker_.cancel();
  completed_waypoint_.reset();
  waypoint_states_.clear();
  active_curve_.reset();
  capture_curve_.reset();
  pending_curve_.reset();
  pending_transition_from_index_.reset();
  pending_completion_ = false;
  capture_active_ = false;
  active_waypoint_index_ = 0;
  active_ = false;
  leg_started_ = false;
  if (!mission.valid) {
    error = mission.error.empty() ? "invalid mission" : mission.error;
    return false;
  }

  ActiveSettings settings =
      missionSettings(mission.defaults, config_.speed_override_m_s);
  bool return_to_start_seen = false;

  for (const auto &item : mission.items) {
    if (return_to_start_seen) {
      error = "rtl must be the final mission item";
      waypoints_.clear();
      return false;
    }

    Waypoint waypoint;
    if (item.type == ItemType::Takeoff) {
      waypoint.id = item.id.empty() ? "takeoff" : item.id;
      waypoint.type = ItemType::Takeoff;
      waypoint.position = config_.reference.hold_position;
      if (std::isfinite(item.waypoint.position_enu[2]) &&
          item.waypoint.position_enu[2] > config_.reference.hold_position[2]) {
        waypoint.position[2] = item.waypoint.position_enu[2];
      }
      assignSettings(waypoint, settings);
      waypoint.horizontal_speed = 0.0;
      waypoint.vertical_speed = std::clamp(settings.vertical_speed, 0.5, 1.2);
      waypoints_.push_back(waypoint);
    } else if (item.type == ItemType::Waypoint) {
      waypoint.id = item.id;
      waypoint.type = ItemType::Waypoint;
      waypoint.position = item.waypoint.position_enu;
      waypoint.heading_rad = item.waypoint.heading_rad;
      assignSettings(waypoint, settings);
      waypoints_.push_back(waypoint);
    } else if (item.type == ItemType::Hold && !waypoints_.empty()) {
      waypoints_.back().hold_duration_s = item.hold.duration_seconds;
    } else if (item.type == ItemType::ChangeSettings) {
      applySettings(item.settings, mission.defaults, config_.speed_override_m_s,
                    settings);
    } else if (item.type == ItemType::Land) {
      waypoint.id = item.id.empty() ? "landing" : item.id;
      waypoint.type = ItemType::Land;
      waypoint.position = waypoints_.empty() ? config_.reference.hold_position
                                             : waypoints_.back().position;
      waypoint.position[2] = 0.0;
      assignSettings(waypoint, settings);
      waypoint.horizontal_speed = 0.0;
      waypoint.vertical_speed = std::clamp(settings.vertical_speed, 0.4, 0.8);
      waypoints_.push_back(waypoint);
    } else if (item.type == ItemType::Rtl) {
      if (waypoints_.empty()) {
        error = "rtl requires a preceding executable waypoint";
        return false;
      }
      waypoint.id = item.id.empty() ? "rtl" : item.id;
      waypoint.type = ItemType::Rtl;
      waypoint.position = waypoints_.back().position;
      assignSettings(waypoint, settings);
      waypoint.return_to_mission_start_xy = true;
      waypoints_.push_back(waypoint);
      return_to_start_seen = true;
    }
  }

  if (waypoints_.empty()) {
    error = "mission contains no executable waypoints";
    return false;
  }
  if (!std::all_of(waypoints_.begin(), waypoints_.end(), validWaypoint)) {
    error = "mission contains invalid compiled waypoint settings";
    waypoints_.clear();
    return false;
  }

  mission_local_positions_.reserve(waypoints_.size());
  mission_local_headings_.reserve(waypoints_.size());
  for (std::size_t index = 0; index < waypoints_.size(); ++index) {
    auto local_position = waypoints_[index].position;
    if (waypoints_[index].type == ItemType::Takeoff) {
      // Takeoff is the local mission origin.  Its compiled XY is only the
      // pre-start hold and must not leak into the mission anchor.
      local_position[0] = 0.0;
      local_position[1] = 0.0;
    } else if (waypoints_[index].type == ItemType::Land && index > 0U) {
      // Land inherits the preceding local XY, just as setMission() inherits
      // the preceding compiled waypoint before the origin is known.
      local_position[0] = mission_local_positions_[index - 1U][0];
      local_position[1] = mission_local_positions_[index - 1U][1];
    }
    mission_local_positions_.push_back(local_position);
    mission_local_headings_.push_back(waypoints_[index].heading_rad);
  }
  error.clear();
  return true;
}

void MissionTrajectory::updateVehicleState(
    const VehicleState &state) noexcept {
  vehicle_state_ = state;
}

void MissionTrajectory::invalidateVehicleState() noexcept {
  vehicle_state_.valid = false;
}

void MissionTrajectory::captureHoldFromVehicle() noexcept {
  if (!vehicle_state_.valid) {
    return;
  }
  config_.reference.hold_position = vehicle_state_.position;
  config_.reference.hold_yaw_rad = vehicle_state_.yaw;
}

bool MissionTrajectory::start(double now_seconds) noexcept {
  if (waypoints_.empty() || !std::isfinite(now_seconds) ||
      !std::isfinite(config_.acceptance_speed_m_s) ||
      config_.acceptance_speed_m_s < 0.0) {
    return false;
  }
  completed_waypoint_.reset();
  pending_transition_from_index_.reset();
  pending_completion_ = false;
  capture_curve_.reset();
  pending_curve_.reset();
  capture_active_ = false;
  active_waypoint_index_ = 0U;
  leg_start_position_ = vehicle_state_.valid ? vehicle_state_.position
                                             : config_.reference.hold_position;
  leg_start_yaw_ = vehicle_state_.valid ? vehicle_state_.yaw
                                        : config_.reference.hold_yaw_rad;
  bindMissionToMeasuredOrigin();
  for (auto &waypoint : waypoints_) {
    if (waypoint.return_to_mission_start_xy) {
      waypoint.position[0] = leg_start_position_[0];
      waypoint.position[1] = leg_start_position_[1];
    }
  }

  MissionProgressTracker::VehicleState initial_state;
  initial_state.position = leg_start_position_;
  initial_state.velocity = vehicle_state_.valid
    ? vehicle_state_.velocity : std::array<double, 3>{0.0, 0.0, 0.0};
  initial_state.yaw = leg_start_yaw_;
  std::vector<MissionProgressTracker::Target> tracker_targets;
  tracker_targets.reserve(waypoints_.size());
  for (std::size_t index = 0; index < waypoints_.size(); ++index) {
    const auto &waypoint = waypoints_[index];
    MissionProgressTracker::Target target;
    target.id = waypoint.id;
    target.position = waypoint.position;
    target.acceptance_radius_m = waypoint.type == ItemType::Land
      ? 0.35 : config_.acceptance_radius_m;
    target.hold_duration_s = waypoint.hold_duration_s;
    target.horizontal_speed_m_s = waypoint.horizontal_speed;
    target.vertical_speed_m_s = waypoint.vertical_speed;
    target.heading_rad = waypoint.heading_rad;
    target.max_heading_rate_rad_s = waypoint.max_heading_rate_rad_s;
    target.maximum_acceleration_m_s2 = waypoint.maximum_acceleration_m_s2;
    target.maximum_jerk_m_s3 = waypoint.maximum_jerk_m_s3;
    target.acceptance_policy =
        isFlyThroughWaypoint(index)
            ? AcceptancePolicy::FlyThrough
            : AcceptancePolicy::StopAndDwell;
    tracker_targets.push_back(std::move(target));
  }
  if (!progress_tracker_.load(std::move(tracker_targets), initial_state)) {
    return false;
  }

  buildWaypointStates();
  KinematicState initial_trajectory_state;
  initial_trajectory_state.position = leg_start_position_;
  initial_trajectory_state.velocity = initial_state.velocity;
  if (!buildActiveCurve(initial_trajectory_state, 0U)) {
    progress_tracker_.cancel();
    return false;
  }
  leg_started_at_seconds_ = now_seconds;
  active_waypoint_index_ = 0U;
  leg_started_ = true;
  active_ = true;
  return true;
}

void MissionTrajectory::bindMissionToMeasuredOrigin() noexcept {
  if (mission_local_positions_.size() != waypoints_.size() ||
      mission_local_headings_.size() != waypoints_.size()) {
    return;
  }
  const double cos_yaw = std::cos(leg_start_yaw_);
  const double sin_yaw = std::sin(leg_start_yaw_);
  for (std::size_t index = 0; index < waypoints_.size(); ++index) {
    const auto &local_position = mission_local_positions_[index];
    auto &waypoint = waypoints_[index];
    waypoint.position[0] = leg_start_position_[0] +
        cos_yaw * local_position[0] - sin_yaw * local_position[1];
    waypoint.position[1] = leg_start_position_[1] +
        sin_yaw * local_position[0] + cos_yaw * local_position[1];
    waypoint.position[2] = local_position[2];
    if (std::isfinite(mission_local_headings_[index])) {
      waypoint.heading_rad = normalizeAngle(
          mission_local_headings_[index] + leg_start_yaw_);
    } else {
      waypoint.heading_rad = NAN;
    }
  }
}

void MissionTrajectory::reset() noexcept {
  progress_tracker_.cancel();
  completed_waypoint_.reset();
  active_curve_.reset();
  capture_curve_.reset();
  pending_curve_.reset();
  pending_transition_from_index_.reset();
  pending_completion_ = false;
  capture_active_ = false;
  active_waypoint_index_ = 0U;
  active_ = false;
  leg_started_ = false;
  captureHoldFromVehicle();
}

bool MissionTrajectory::isFlyThroughWaypoint(
    std::size_t index) const noexcept {
  if (index >= waypoints_.size()) {
    return false;
  }
  const auto &waypoint = waypoints_[index];
  return waypoint.type == ItemType::Waypoint &&
         index + 1U < waypoints_.size() && waypoint.hold_duration_s <= 0.0;
}

TrajectoryCurve::Limits MissionTrajectory::curveLimits(
    std::size_t index) const noexcept {
  TrajectoryCurve::Limits limits;
  if (index >= waypoints_.size()) {
    return limits;
  }
  const auto &waypoint = waypoints_[index];
  limits.max_speed_xy_m_s = std::min(
      config_.planner_max_speed_xy_m_s,
      clampPositive(waypoint.horizontal_speed,
                    config_.planner_max_speed_xy_m_s));
  limits.max_speed_z_m_s = std::min(
      config_.planner_max_speed_z_m_s,
      clampPositive(waypoint.vertical_speed,
                    config_.planner_max_speed_z_m_s));
  limits.max_acceleration_xy_m_s2 = std::min(
      config_.planner_max_acceleration_xy_m_s2,
      clampPositive(waypoint.maximum_acceleration_m_s2,
                    config_.planner_max_acceleration_xy_m_s2));
  limits.max_acceleration_z_m_s2 = std::min(
      config_.planner_max_acceleration_z_m_s2,
      clampPositive(waypoint.maximum_acceleration_m_s2,
                    config_.planner_max_acceleration_z_m_s2));
  limits.max_jerk_xy_m_s3 = std::min(
      config_.planner_max_jerk_xy_m_s3,
      clampPositive(waypoint.maximum_jerk_m_s3,
                    config_.planner_max_jerk_xy_m_s3));
  limits.max_jerk_z_m_s3 = std::min(
      config_.planner_max_jerk_z_m_s3,
      clampPositive(waypoint.maximum_jerk_m_s3,
                    config_.planner_max_jerk_z_m_s3));
  return limits;
}

double MissionTrajectory::nominalDuration( const KinematicState &from, const Waypoint &to) const noexcept {
  const double horizontal_distance = std::hypot(to.position[0] - from.position[0], to.position[1] - from.position[1]);
  const double vertical_distance = std::abs(to.position[2] - from.position[2]);
  const double horizontal_speed = std::min(config_.planner_max_speed_xy_m_s,
      clampPositive(to.horizontal_speed, config_.planner_max_speed_xy_m_s));
  const double vertical_speed = std::min(config_.planner_max_speed_z_m_s,
      clampPositive(to.vertical_speed, config_.planner_max_speed_z_m_s));
  const double horizontal_time = horizontal_distance > 1.0e-3 ? horizontal_distance / horizontal_speed : 0.0;
  const double vertical_time = vertical_distance > 1.0e-3 ? vertical_distance / vertical_speed : 0.0;
  return std::max({horizontal_time, vertical_time, 0.5});
}

MissionTrajectory::KinematicState
MissionTrajectory::stopWaypointState(std::size_t index) const noexcept {
  KinematicState state;
  if (index < waypoint_states_.size()) {
    state = waypoint_states_[index];
  } else if (index < waypoints_.size()) {
    state.position = waypoints_[index].position;
  }
  state.velocity = {0.0, 0.0, 0.0};
  state.acceleration = {0.0, 0.0, 0.0};
  return state;
}

void MissionTrajectory::buildWaypointStates() noexcept {
  waypoint_states_.assign(waypoints_.size(), KinematicState{});
  for (std::size_t index = 0; index < waypoints_.size(); ++index) {
    auto &state = waypoint_states_[index];
    state.position = waypoints_[index].position;
    if (!isFlyThroughWaypoint(index)) {
      continue;
    }

    const Vector3 previous_position = index == 0U
        ? leg_start_position_ : waypoints_[index - 1U].position;
    const Vector3 next_position = waypoints_[index + 1U].position;
    const Vector3 incoming = subtract(state.position, previous_position);
    const Vector3 outgoing = subtract(next_position, state.position);
    const double incoming_length = norm(incoming);
    const double outgoing_length = norm(outgoing);
    if (incoming_length <= 1.0e-6 || outgoing_length <= 1.0e-6) {
      continue;
    }

    const Vector3 incoming_unit = scale(incoming, 1.0 / incoming_length);
    const Vector3 outgoing_unit = scale(outgoing, 1.0 / outgoing_length);
    const Vector3 bisector = scale(
        add(scale(incoming_unit, outgoing_length),scale(outgoing_unit, incoming_length)),1.0 / (incoming_length + outgoing_length));
    const double turn_factor = std::clamp(0.5 * norm(bisector), 0.0, 1.0);
    const Vector3 tangent = normalized(bisector);
    const double horizontal_tangent = std::hypot(tangent[0], tangent[1]);
    const double vertical_tangent = std::abs(tangent[2]);
    const auto &waypoint = waypoints_[index];
    const auto &successor = waypoints_[index + 1U];
    const double horizontal_speed = std::min({
        config_.planner_max_speed_xy_m_s,
        clampPositive(waypoint.horizontal_speed,config_.planner_max_speed_xy_m_s),
        clampPositive(successor.horizontal_speed,config_.planner_max_speed_xy_m_s)});
    const double vertical_speed = std::min({
        config_.planner_max_speed_z_m_s,
        clampPositive(waypoint.vertical_speed,config_.planner_max_speed_z_m_s),
        clampPositive(successor.vertical_speed,config_.planner_max_speed_z_m_s)});
    double scalar_speed = std::numeric_limits<double>::infinity();
    if (horizontal_tangent > 1.0e-6) {
      scalar_speed = std::min(scalar_speed,horizontal_speed / horizontal_tangent);
    }
    if (vertical_tangent > 1.0e-6) {
      scalar_speed = std::min(scalar_speed,vertical_speed / vertical_tangent);
    }
    if (!std::isfinite(scalar_speed)) {
      continue;
    }
    state.velocity = scale(tangent, scalar_speed * turn_factor);

    // Keep the shared fly-through knot acceleration at zero.  A heuristic
    // finite-difference acceleration can satisfy the knot envelope while the
    // boundary-conditioned polynomial overshoots it between knots, making the
    // successor curve infeasible and aborting the mission.  The shared
    // zero acceleration is still C2-continuous; the curve duration search
    // enforces the configured acceleration and jerk limits over the whole leg.
    state.acceleration = {0.0, 0.0, 0.0};
  }
}

std::optional<TrajectoryCurve>
MissionTrajectory::createCurve(const KinematicState &start,
                                         std::size_t target_index,
                                         bool clamp_start) const noexcept {
  if (target_index >= waypoints_.size() ||
      target_index >= waypoint_states_.size()) {
    return std::nullopt;
  }
  const auto limits = curveLimits(target_index);
  KinematicState bounded_start = start;
  if (clamp_start) {
    const double start_xy = std::hypot(bounded_start.velocity[0],
                                       bounded_start.velocity[1]);
    if (start_xy > limits.max_speed_xy_m_s && start_xy > 1.0e-9) {
      const double factor = limits.max_speed_xy_m_s / start_xy;
      bounded_start.velocity[0] *= factor;
      bounded_start.velocity[1] *= factor;
    }
    bounded_start.velocity[2] = std::clamp(
        bounded_start.velocity[2], -limits.max_speed_z_m_s,
        limits.max_speed_z_m_s);
    clampAcceleration(bounded_start.acceleration,
                      limits.max_acceleration_xy_m_s2,
                      limits.max_acceleration_z_m_s2);
  }

  const double duration = nominalDuration(bounded_start,
                                          waypoints_[target_index]);
  return TrajectoryCurve::create(
      bounded_start, waypoint_states_[target_index], duration, limits);
}

void MissionTrajectory::prepareCaptureCurve(
    std::size_t target_index) noexcept {
  capture_curve_.reset();
  if (!active_curve_ || !isFlyThroughWaypoint(target_index)) {
    return;
  }
  const auto endpoint = active_curve_->sample(active_curve_->durationSeconds());
  capture_curve_ = TrajectoryCurve::create(
      {endpoint.position, endpoint.velocity, endpoint.acceleration},
      stopWaypointState(target_index), 0.5, curveLimits(target_index));
}

bool MissionTrajectory::buildActiveCurve(
    const KinematicState &start, std::size_t target_index) noexcept {
  const auto curve = createCurve(start, target_index, true);
  if (!curve) {
    active_curve_.reset();
    return false;
  }
  active_curve_ = curve;
  pending_curve_.reset();
  capture_active_ = false;
  prepareCaptureCurve(target_index);
  return true;
}

MissionTrajectory::Update
MissionTrajectory::update(double now_seconds) {
  Update result;
  if (!std::isfinite(now_seconds) || config_.sample_period_seconds <= 0.0 ||
      config_.horizon_seconds < config_.sample_period_seconds) {
    return result;
  }

  if (active_ && leg_started_ &&
      active_waypoint_index_ < waypoints_.size()) {
    const auto &target = waypoints_[active_waypoint_index_];
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

    // Timing and finish-plane crossing are useful diagnostics, but neither is
    // evidence that the vehicle reached a waypoint.  Advancing on either one
    // makes a tracking error turn into a new reference curve and silently
    // cuts corners in a mission such as the slalom benchmark.
    result.distance_to_waypoint_m = distance;
    result.crossed_finish_plane = crossed_finish_plane;
    if (vehicle_state_.valid && !pending_transition_from_index_) {
      MissionProgressTracker::VehicleState progress_state;
      progress_state.position = vehicle_state_.position;
      progress_state.velocity = vehicle_state_.velocity;
      progress_state.yaw = vehicle_state_.yaw;
      const auto progress = progress_tracker_.update(progress_state, now_seconds);
      if (progress.target_accepted) {
        result.waypoint_reached = true;
        result.reached_waypoint_index = progress.reached_index;
        pending_transition_from_index_ = progress.reached_index;
        pending_completion_ = progress.completed_now;
        if (!pending_completion_) {
          const auto successor_start = capture_active_
              ? stopWaypointState(progress.reached_index)
              : waypoint_states_[progress.reached_index];
          pending_curve_ = createCurve(
              successor_start, progress_tracker_.currentIndex(), false);
          if (!pending_curve_) {
            pending_curve_ = createCurve(
                successor_start, progress_tracker_.currentIndex(), true);
          }
        }
      }
    }

    const double elapsed =
        std::max(0.0, now_seconds - leg_started_at_seconds_);
    bool committed_transition = false;
    if (pending_transition_from_index_ && active_curve_ &&
        elapsed >= active_curve_->durationSeconds()) {
      const std::size_t reached_index = *pending_transition_from_index_;
      const auto endpoint = active_curve_->sample(
          active_curve_->durationSeconds());
      const KinematicState handoff_state = capture_active_
          ? KinematicState{endpoint.position, endpoint.velocity,
                           endpoint.acceleration}
          : waypoint_states_[reached_index];
      if (pending_completion_) {
        completed_waypoint_ = waypoints_[reached_index];
        config_.reference.hold_position =
            completed_waypoint_->position;
        active_curve_.reset();
        capture_curve_.reset();
        pending_curve_.reset();
        active_ = false;
        leg_started_ = false;
        capture_active_ = false;
        result.mission_completed = true;
      } else {
        const std::size_t successor_index =
            progress_tracker_.currentIndex();
        const double dx = waypoints_[reached_index].position[0] -
                          leg_start_position_[0];
        const double dy = waypoints_[reached_index].position[1] -
                          leg_start_position_[1];
        const double completed_yaw = std::hypot(dx, dy) > 0.5
            ? std::atan2(dy, dx) : leg_start_yaw_;
        active_waypoint_index_ = successor_index;
        active_curve_ = pending_curve_;
        pending_curve_.reset();
        capture_active_ = false;
        leg_start_position_ = handoff_state.position;
        leg_start_yaw_ = completed_yaw;
        if (!active_curve_) {
          config_.reference.hold_position = handoff_state.position;
          config_.reference.hold_yaw_rad = completed_yaw;
          active_ = false;
          leg_started_ = false;
        } else {
          prepareCaptureCurve(successor_index);
          leg_started_at_seconds_ = now_seconds;
        }
        committed_transition = true;
      }
      pending_transition_from_index_.reset();
      pending_completion_ = false;
    }

    if (!committed_transition && active_ && active_curve_ && !capture_active_ &&
        !pending_transition_from_index_ &&
        isFlyThroughWaypoint(active_waypoint_index_) &&
        elapsed >= active_curve_->durationSeconds() && capture_curve_) {
      const double capture_start_time =
          leg_started_at_seconds_ + active_curve_->durationSeconds();
      active_curve_ = capture_curve_;
      capture_curve_.reset();
      capture_active_ = true;
      leg_started_at_seconds_ = capture_start_time;
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

TrajectorySample MissionTrajectory::sample(
    double now_seconds, double horizon_offset_seconds) const noexcept {
  TrajectorySample point;
  if (!active_ || !leg_started_ ||
      active_waypoint_index_ >= waypoints_.size() ||
      !active_curve_) {
    point.position = config_.reference.hold_position;
    point.yaw = config_.reference.hold_yaw_rad;
    if (completed_waypoint_) {
      assignTrackingLimits(point, *completed_waypoint_);
    }
    return point;
  }

  const double elapsed = std::max(0.0, now_seconds - leg_started_at_seconds_);
  const auto &target = waypoints_[active_waypoint_index_];
  assignTrackingLimits(point, target);
  const double remaining =
      std::max(0.0, active_curve_->durationSeconds() - elapsed);
  if (horizon_offset_seconds > remaining) {
    const TrajectoryCurve *continuation = nullptr;
    if (!capture_active_) {
      continuation = pending_curve_ ? &*pending_curve_
                                      : (capture_curve_ ? &*capture_curve_
                                                           : nullptr);
    }
    if (continuation) {
      const auto continuation_sample = continuation->sample(
          horizon_offset_seconds - remaining);
      point.position = continuation_sample.position;
      point.velocity = continuation_sample.velocity;
      point.acceleration = continuation_sample.acceleration;
      point.yaw = std::isfinite(target.heading_rad)
          ? target.heading_rad : leg_start_yaw_;
      return point;
    }

    // The measured tracker still owns progression.  Once the current curve
    // ends without an accepted successor, hold the current target with zero
    // derivatives rather than publishing a stationary point with stale
    // fly-through velocity or acceleration.
    point.position = target.position;
    point.yaw = std::isfinite(target.heading_rad)
        ? target.heading_rad : leg_start_yaw_;
    return point;
  }

  const auto curve_sample =
      active_curve_->sample(elapsed + horizon_offset_seconds);
  point.position = curve_sample.position;
  point.velocity = curve_sample.velocity;
  point.acceleration = curve_sample.acceleration;
  if (std::isfinite(target.heading_rad)) {
    point.yaw = target.heading_rad;
  // } else if (std::hypot(point.velocity[0], point.velocity[1]) > 1.0e-3) {
  //   point.yaw = std::atan2(point.velocity[1], point.velocity[0]);
  } else {
    point.yaw = leg_start_yaw_;
  }
  return point;
}

} // namespace mpc_controller::mission
