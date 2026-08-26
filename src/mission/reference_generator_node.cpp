#include "mpc_controller/msg/reference_trajectory.hpp"
#include "mpc_controller/msg/trajectory_point.hpp"
#include "mpc_controller/msg/vehicle_state.hpp"
#include "mpc_controller/mission/mission_parser.hpp"
#include "mpc_controller/mission/minimum_time_trajectory.hpp"
#include "mpc_controller/controller/reference_model.hpp"

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_ros2/utils/message_version.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

class ReferenceGeneratorNode final : public rclcpp::Node
{
public:
  ReferenceGeneratorNode()
  : Node("reference_generator_node")
  {
    declareAndGet("frame_id", frame_id_);
    declareAndGet("state_topic", state_topic_);
    declareAndGet("mission_file_path", mission_file_path_);
    declareAndGet("mission_acceptance_radius_m", mission_acceptance_radius_m_);
    declareAndGet("mission_speed_override_m_s", mission_speed_override_m_s_);
    declareAndGet("hold_yaw_rad", parameters_.hold_yaw_rad);
    declareAndGet("auto_capture_current_hold", auto_capture_current_hold_);
    declareAndGet("horizon_seconds", horizon_seconds_);
    declareAndGet("sample_period_seconds", sample_period_seconds_);
    declareAndGet("publish_rate_hz", publish_rate_hz_);
    declareAndGet("visualization_enabled", visualization_enabled_);
    declareAndGet("visualization_publish_rate_hz", visualization_publish_rate_hz_);

    declare_parameter("hold_position", std::vector<double>{0.0, 0.0, 1.0});
    getVectorParameter("hold_position", parameters_.hold_position);

    valid_config_ = !frame_id_.empty() && !state_topic_.empty()
      && std::isfinite(horizon_seconds_) && horizon_seconds_ > 0.0
      && std::isfinite(sample_period_seconds_) && sample_period_seconds_ > 0.0
      && std::isfinite(publish_rate_hz_) && publish_rate_hz_ > 0.0;

    if (!valid_config_) {
      RCLCPP_ERROR(get_logger(), "Invalid reference generator parameters; publishing disabled");
      return;
    }

    // Publishers
    publisher_ = create_publisher<Reference>("reference_trajectory", 10);
    mission_completed_publisher_ = create_publisher<std_msgs::msg::Bool>(
      "/reference_generator_node/mission_completed", 10);
    if (visualization_enabled_) {
      visualization_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "~/visualization_markers", 10);
    }

    // Subscriptions
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    state_subscription_ = create_subscription<State>(
      state_topic_, qos,
      std::bind(&ReferenceGeneratorNode::stateCallback, this, std::placeholders::_1));

    external_mode_subscription_ = create_subscription<px4_msgs::msg::VehicleStatus>(
      "fmu/out/vehicle_status" +
      px4_ros2::getMessageNameVersion<px4_msgs::msg::VehicleStatus>(), qos,
      std::bind(&ReferenceGeneratorNode::externalModeCallback, this, std::placeholders::_1));

    // Services
    start_mission_service_ = create_service<std_srvs::srv::Trigger>(
      "~/start_mission",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
             std_srvs::srv::Trigger::Response::SharedPtr response) {
        response->success = startMission();
        response->message = response->success ? "Mission started" :
          "Mission trajectory could not be planned";
      });

    reset_mission_service_ = create_service<std_srvs::srv::Trigger>(
      "~/reset_mission",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
             std_srvs::srv::Trigger::Response::SharedPtr response) {
        resetMission();
        response->success = true;
        response->message = "Mission reset to initial state";
      });

    // Load mission waypoints
    loadMissionWaypoints();

    // Main publish timer
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / std::max(publish_rate_hz_, 1.0)));
    timer_ = create_wall_timer(period, std::bind(&ReferenceGeneratorNode::publish, this));

    RCLCPP_INFO(
      get_logger(),
      "ReferenceGeneratorNode (MPC Attitude + Mission) initialized: mission='%s' waypoints=%zu rate=%.1f Hz",
      mission_file_path_.c_str(), mission_waypoints_.size(), publish_rate_hz_);
  }

private:
  using Reference = mpc_controller::msg::ReferenceTrajectory;
  using Point = mpc_controller::msg::TrajectoryPoint;
  using State = mpc_controller::msg::VehicleState;
  using SteadyClock = std::chrono::steady_clock;
  using TrajectorySegment = mpc_controller::trajectory::QuinticSegment;

  struct StateInput
  {
    std::array<double, 3> position{};
    std::array<double, 3> velocity{};
    double yaw = 0.0;
    bool valid = false;
  };

  struct MissionWaypoint
  {
    std::string id;
    std::array<double, 3> position{0.0, 0.0, 1.0};
    double horizontal_speed = 4.0;
    double vertical_speed = 1.5;
    double maximum_acceleration = 2.5;
    double maximum_jerk = 5.0;
    double maximum_heading_rate_rad_s = 1.0471975511965976;
    double heading_rad = NAN;
    double hold_duration_s = 0.0;
  };

  template<typename T>
  void declareAndGet(const std::string & name, T & value)
  {
    declare_parameter(name, value);
    get_parameter(name, value);
  }

  void getVectorParameter(const std::string & name, std::array<double, 3> & value)
  {
    const auto vec = get_parameter(name).as_double_array();
    if (vec.size() == 3 && std::all_of(vec.begin(), vec.end(), [](double v) {return std::isfinite(v);})) {
      value = {vec[0], vec[1], vec[2]};
    }
  }

  static builtin_interfaces::msg::Duration durationMessage(uint64_t nanoseconds) noexcept
  {
    builtin_interfaces::msg::Duration duration;
    duration.sec = static_cast<int32_t>(nanoseconds / 1000000000ULL);
    duration.nanosec = static_cast<uint32_t>(nanoseconds % 1000000000ULL);
    return duration;
  }

  mpc_controller::trajectory::Limits limitsFor(const MissionWaypoint &waypoint) const noexcept
  {
    return {waypoint.horizontal_speed, waypoint.vertical_speed,
            waypoint.maximum_acceleration, waypoint.maximum_jerk,
            waypoint.maximum_heading_rate_rad_s};
  }

  mpc_controller::reference::Sample missionSampleFromState(
    const StateInput &state) const noexcept
  {
    mpc_controller::reference::Sample sample;
    sample.position = state.position;
    sample.velocity = state.velocity;
    sample.yaw = state.yaw;
    return sample;
  }

  std::array<double, 3> terminalVelocityForWaypoint(std::size_t waypoint_index) const noexcept
  {
    if (waypoint_index == 0 || waypoint_index + 1 >= mission_waypoints_.size() ||
        mission_waypoints_[waypoint_index].hold_duration_s > 0.0) {
      return {};
    }
    const MissionWaypoint &previous_waypoint = mission_waypoints_[waypoint_index - 1];
    const MissionWaypoint &waypoint = mission_waypoints_[waypoint_index];
    const MissionWaypoint &next_waypoint = mission_waypoints_[waypoint_index + 1];
    const double horizontal_speed = std::min(waypoint.horizontal_speed,
                                             next_waypoint.horizontal_speed);
    const double vertical_speed = std::min(waypoint.vertical_speed,
                                           next_waypoint.vertical_speed);
    return mpc_controller::trajectory::blendedCornerVelocity(
      previous_waypoint.position, waypoint.position, next_waypoint.position,
      horizontal_speed, vertical_speed);
  }

  double terminalHeadingForWaypoint(
    const mpc_controller::reference::Sample &start,
    std::size_t waypoint_index) const noexcept
  {
    const MissionWaypoint &target = mission_waypoints_[waypoint_index];
    if (std::isfinite(target.heading_rad)) {
      return start.yaw + std::atan2(
        std::sin(target.heading_rad - start.yaw),
        std::cos(target.heading_rad - start.yaw));
    }

    const auto terminal_velocity = terminalVelocityForWaypoint(waypoint_index);
    if (std::hypot(terminal_velocity[0], terminal_velocity[1]) > 1.0e-3) {
      const double heading = std::atan2(terminal_velocity[1], terminal_velocity[0]);
      return start.yaw + std::atan2(
        std::sin(heading - start.yaw), std::cos(heading - start.yaw));
    }

    const double dx = target.position[0] - start.position[0];
    const double dy = target.position[1] - start.position[1];
    if (std::hypot(dx, dy) <= 1.0e-3) {
      return start.yaw;
    }
    const double heading = std::atan2(dy, dx);
    return start.yaw + std::atan2(
      std::sin(heading - start.yaw), std::cos(heading - start.yaw));
  }

  std::optional<TrajectorySegment> planSegment(
    const mpc_controller::reference::Sample &start,
    std::size_t waypoint_index) const noexcept
  {
    if (waypoint_index >= mission_waypoints_.size()) {
      return std::nullopt;
    }
    const MissionWaypoint &target = mission_waypoints_[waypoint_index];
    mpc_controller::trajectory::Boundary initial;
    initial.sample = start;
    mpc_controller::trajectory::Boundary finish;
    finish.sample.position = target.position;
    finish.sample.velocity = terminalVelocityForWaypoint(waypoint_index);
    finish.sample.yaw = terminalHeadingForWaypoint(start, waypoint_index);
    return TrajectorySegment::create(initial, finish, limitsFor(target));
  }

  bool planMission(const mpc_controller::reference::Sample &start)
  {
    mission_segments_.clear();
    mpc_controller::reference::Sample boundary = start;
    for (std::size_t index = 0; index < mission_waypoints_.size(); ++index) {
      const auto segment = planSegment(boundary, index);
      if (!segment) {
        RCLCPP_ERROR(get_logger(), "Cannot build a feasible trajectory to waypoint '%s'",
                     mission_waypoints_[index].id.c_str());
        mission_segments_.clear();
        return false;
      }
      boundary = segment->sample(segment->durationSeconds());
      mission_segments_.push_back(*segment);
    }
    return true;
  }

  bool beginMissionLeg()
  {
    if (mission_wp_index_ >= mission_segments_.size()) {
      return false;
    }
    const MissionWaypoint &target = mission_waypoints_[mission_wp_index_];
    active_segment_ = mission_segments_[mission_wp_index_];
    mission_leg_duration_s_ = active_segment_->durationSeconds() + target.hold_duration_s;
    RCLCPP_INFO(
      get_logger(),
      "Planned trajectory to '%s': duration=%.2fs limits=[v_xy=%.2f v_z=%.2f a=%.2f j=%.2f yaw_rate=%.2f deg/s]",
      target.id.c_str(), active_segment_->durationSeconds(), target.horizontal_speed,
      target.vertical_speed, target.maximum_acceleration, target.maximum_jerk,
      target.maximum_heading_rate_rad_s / mpc_controller::mission::kDegreesToRadians);
    return true;
  }

  mpc_controller::reference::Sample activeMissionSample(double elapsed_seconds) const noexcept
  {
    if (!active_segment_) {
      return {};
    }
    return active_segment_->sample(std::min(elapsed_seconds, active_segment_->durationSeconds()));
  }

  static mpc_controller::reference::Sample stationary(
    mpc_controller::reference::Sample sample) noexcept
  {
    sample.velocity = {};
    sample.acceleration = {};
    sample.yaw_rate = 0.0;
    return sample;
  }

  mpc_controller::reference::Sample missionSampleAt(double elapsed_seconds) const noexcept
  {
    std::size_t segment_index = mission_wp_index_;
    double remaining_seconds = std::max(0.0, elapsed_seconds);
    while (segment_index < mission_segments_.size()) {
      const TrajectorySegment &segment = mission_segments_[segment_index];
      const double segment_duration = segment.durationSeconds();
      if (remaining_seconds <= segment_duration) {
        return segment.sample(remaining_seconds);
      }
      remaining_seconds -= segment_duration;
      const auto endpoint = stationary(segment.sample(segment_duration));
      const double hold_duration = mission_waypoints_[segment_index].hold_duration_s;
      if (hold_duration > 0.0 && remaining_seconds <= hold_duration) {
        return endpoint;
      }
      remaining_seconds -= hold_duration;
      ++segment_index;
    }
    return mission_segments_.empty() ? mpc_controller::reference::Sample{} :
      stationary(mission_segments_.back().sample(mission_segments_.back().durationSeconds()));
  }

  bool loadMissionWaypoints()
  {
    if (mission_file_path_.empty()) {
      RCLCPP_ERROR(get_logger(), "Mission file path is empty");
      return false;
    }

    const auto mission = mpc_controller::mission::parse(mission_file_path_);
    if (!mission.valid) {
      RCLCPP_ERROR(
        get_logger(), "Failed to parse mission JSON '%s': %s",
        mission_file_path_.c_str(), mission.error.c_str());
      return false;
    }

    mission_waypoints_.clear();
    mpc_controller::mission::Defaults settings = mission.defaults;
    if (mission_speed_override_m_s_ > 0.0) {
      settings.horizontal_velocity_m_s = mission_speed_override_m_s_;
    }

    const auto makeWaypoint = [&settings] {
      MissionWaypoint waypoint;
      waypoint.horizontal_speed = settings.horizontal_velocity_m_s;
      waypoint.vertical_speed = settings.vertical_velocity_m_s;
      waypoint.maximum_acceleration = settings.maximum_acceleration_m_s2;
      waypoint.maximum_jerk = settings.maximum_jerk_m_s3;
      waypoint.maximum_heading_rate_rad_s =
        settings.max_heading_rate_deg_s * mpc_controller::mission::kDegreesToRadians;
      return waypoint;
    };

    for (const auto & item : mission.items) {
      if (item.type == mpc_controller::mission::ItemType::Takeoff) {
        MissionWaypoint wp = makeWaypoint();
        wp.id = item.id.empty() ? "takeoff" : item.id;
        wp.position = parameters_.hold_position;
        if (std::isfinite(item.waypoint.position_enu[2]) && item.waypoint.position_enu[2] > parameters_.hold_position[2]) {
          wp.position[2] = item.waypoint.position_enu[2];
        }
        wp.vertical_speed = std::clamp(settings.vertical_velocity_m_s, 0.5, 1.2);
        mission_waypoints_.push_back(wp);
      } else if (item.type == mpc_controller::mission::ItemType::Waypoint) {
        MissionWaypoint wp = makeWaypoint();
        wp.id = item.id;
        wp.position = item.waypoint.position_enu;
        wp.heading_rad = item.waypoint.heading_rad;
        mission_waypoints_.push_back(wp);
      } else if (item.type == mpc_controller::mission::ItemType::Hold) {
        if (!mission_waypoints_.empty()) {
          mission_waypoints_.back().hold_duration_s = item.hold.duration_seconds;
        }
      } else if (item.type == mpc_controller::mission::ItemType::ChangeSettings) {
        if (item.settings.reset_all) {
          settings = mission.defaults;
          if (mission_speed_override_m_s_ > 0.0) {
            settings.horizontal_velocity_m_s = mission_speed_override_m_s_;
          }
        }
        if (std::isfinite(item.settings.horizontal_velocity_m_s)) {
          settings.horizontal_velocity_m_s = item.settings.horizontal_velocity_m_s;
        }
        if (std::isfinite(item.settings.vertical_velocity_m_s)) {
          settings.vertical_velocity_m_s = item.settings.vertical_velocity_m_s;
        }
        if (std::isfinite(item.settings.max_heading_rate_deg_s)) {
          settings.max_heading_rate_deg_s = item.settings.max_heading_rate_deg_s;
        }
        if (std::isfinite(item.settings.maximum_acceleration_m_s2)) {
          settings.maximum_acceleration_m_s2 = item.settings.maximum_acceleration_m_s2;
        }
        if (std::isfinite(item.settings.maximum_jerk_m_s3)) {
          settings.maximum_jerk_m_s3 = item.settings.maximum_jerk_m_s3;
        }
      } else if (item.type == mpc_controller::mission::ItemType::Land) {
        MissionWaypoint wp = makeWaypoint();
        wp.id = item.id.empty() ? "landing" : item.id;
        wp.position = mission_waypoints_.empty() ? parameters_.hold_position : mission_waypoints_.back().position;
        wp.position[2] = 0.0;
        wp.vertical_speed = std::clamp(settings.vertical_velocity_m_s, 0.4, 0.8);
        mission_waypoints_.push_back(wp);
      }
    }

    if (mission_waypoints_.empty()) {
      RCLCPP_ERROR(get_logger(), "Mission contains no valid waypoints");
      return false;
    }

    RCLCPP_INFO(
      get_logger(), "Loaded %zu mission waypoints from '%s'",
      mission_waypoints_.size(), mission_file_path_.c_str());
    return true;
  }

  bool startMission()
  {
    if (mission_waypoints_.empty() && !loadMissionWaypoints()) {
      RCLCPP_ERROR(get_logger(), "Cannot start mission: no waypoints loaded");
      return false;
    }

    mission_wp_index_ = 0;
    mpc_controller::reference::Sample start;
    if (latest_input_ && latest_input_->valid) {
      start = missionSampleFromState(*latest_input_);
    } else {
      start.position = parameters_.hold_position;
      start.yaw = parameters_.hold_yaw_rad;
    }
    if (!planMission(start) || !beginMissionLeg()) {
      parameters_.type = "hold";
      return false;
    }
    mission_leg_started_at_ = SteadyClock::now();
    parameters_.type = "mission";

    RCLCPP_INFO(
      get_logger(), "Mission started: %zu waypoints, targeting wp 1 ('%s') [%.2f, %.2f, %.2f]",
      mission_waypoints_.size(), mission_waypoints_[0].id.c_str(),
      mission_waypoints_[0].position[0], mission_waypoints_[0].position[1],
      mission_waypoints_[0].position[2]);
    return true;
  }

  void resetMission()
  {
    mission_wp_index_ = 0;
    mission_leg_started_at_.reset();
    active_segment_.reset();
    mission_segments_.clear();
    parameters_.type = "hold";
    if (latest_input_ && latest_input_->valid) {
      parameters_.hold_position = latest_input_->position;
      parameters_.hold_yaw_rad = latest_input_->yaw;
    }
    RCLCPP_INFO(get_logger(), "Mission reset to hold mode at [%.2f, %.2f, %.2f]",
      parameters_.hold_position[0], parameters_.hold_position[1], parameters_.hold_position[2]);
  }

  void stateCallback(const State::SharedPtr message)
  {
    StateInput input;
    input.position = {message->position[0], message->position[1], message->position[2]};
    input.velocity = {message->velocity[0], message->velocity[1], message->velocity[2]};
    input.yaw = message->yaw;
    // Reference timing needs a valid pose/velocity estimate.  PX4's heading
    // readiness is a control-mode gate and must not prevent the mission from
    // remembering the actual position at External Mode entry.
    input.valid = message->valid && message->position_valid &&
      message->velocity_valid && message->attitude_valid;
    latest_input_ = input;

  }

  void captureHoldAtExternalModeEntry()
  {
    if (!auto_capture_current_hold_) {
      return;
    }
    if (!latest_input_ || !latest_input_->valid) {
      RCLCPP_WARN(get_logger(),
        "External Mode entered before a valid vehicle state; retaining the existing hold reference");
      return;
    }

    parameters_.type = "hold";
    parameters_.hold_position = latest_input_->position;
    parameters_.hold_yaw_rad = latest_input_->yaw;
    mission_wp_index_ = 0;
    mission_leg_started_at_.reset();
    active_segment_.reset();
    mission_segments_.clear();
    RCLCPP_INFO(
      get_logger(),
      "External Mode hold captured: [%.3f, %.3f, %.3f] yaw=%.3f rad",
      parameters_.hold_position[0], parameters_.hold_position[1],
      parameters_.hold_position[2], parameters_.hold_yaw_rad);
  }

  void externalModeCallback(const px4_msgs::msg::VehicleStatus::SharedPtr message)
  {
    const bool external_mode_active = message && message->executor_in_charge != 0;
    if (external_mode_active && !external_mode_active_) {
      captureHoldAtExternalModeEntry();
    } else if (!external_mode_active && external_mode_active_) {
      RCLCPP_INFO(get_logger(), "External Mode exited");
    }
    external_mode_active_ = external_mode_active;
  }

  void publish()
  {
    if (!valid_config_ || !publisher_) {
      return;
    }

    const auto steady_now = SteadyClock::now();
    const uint64_t horizon_ns = static_cast<uint64_t>(horizon_seconds_ * 1.0e9);
    const uint64_t sample_ns = static_cast<uint64_t>(sample_period_seconds_ * 1.0e9);
    if (sample_ns == 0 || horizon_ns < sample_ns) {
      return;
    }

    // Advance only after the scheduled segment/hold has elapsed and the
    // vehicle reached the current waypoint. While late, the reference holds
    // the endpoint rather than progressing through future legs.
    if (parameters_.type == "mission" && mission_leg_started_at_ && active_segment_ &&
        mission_wp_index_ < mission_waypoints_.size()) {
      const auto & target_wp = mission_waypoints_[mission_wp_index_];
      const double leg_elapsed = std::chrono::duration<double>(steady_now - *mission_leg_started_at_).count();
      double dist_to_target = 1000.0;

      if (latest_input_ && latest_input_->valid) {
        const double ex = latest_input_->position[0] - target_wp.position[0];
        const double ey = latest_input_->position[1] - target_wp.position[1];
        const double ez = latest_input_->position[2] - target_wp.position[2];
        dist_to_target = std::sqrt(ex * ex + ey * ey + ez * ez);
      }

      const bool is_landing_wp = (target_wp.id == "landing" || target_wp.position[2] <= 0.1);
      const double acceptance_radius = is_landing_wp ? 0.35 : mission_acceptance_radius_m_;
      const auto completed_reference = activeMissionSample(active_segment_->durationSeconds());
      const double terminal_speed = std::sqrt(
        completed_reference.velocity[0] * completed_reference.velocity[0] +
        completed_reference.velocity[1] * completed_reference.velocity[1] +
        completed_reference.velocity[2] * completed_reference.velocity[2]);
      const bool fly_through = !is_landing_wp && target_wp.hold_duration_s <= 0.0 &&
        terminal_speed > 1.0e-3;
      // A fly-through segment was planned with a non-zero terminal velocity
      // that exactly matches the next segment's initial velocity. Advancing it
      // on schedule preserves that continuity. Holding its endpoint while
      // waiting for the acceptance radius would introduce an artificial
      // zero-velocity reference for one or more controller samples.
      const bool waypoint_reached = fly_through ?
        leg_elapsed >= active_segment_->durationSeconds() :
        leg_elapsed >= mission_leg_duration_s_ && dist_to_target < acceptance_radius;

      if (waypoint_reached) {
        RCLCPP_INFO(
          get_logger(),
          "Mission waypoint %zu/%zu reached: id='%s' [%.2f, %.2f, %.2f] (dist=%.2fm)",
          mission_wp_index_ + 1, mission_waypoints_.size(),
          target_wp.id.c_str(), target_wp.position[0], target_wp.position[1], target_wp.position[2],
          dist_to_target);

        mission_wp_index_++;
        if (mission_wp_index_ >= mission_waypoints_.size()) {
          parameters_.hold_position = target_wp.position;
          parameters_.hold_yaw_rad = completed_reference.yaw;
          parameters_.type = "hold";
          mission_leg_started_at_.reset();
          active_segment_.reset();
          RCLCPP_INFO(get_logger(), "Mission completed successfully; handing off to native landing...");

          if (mission_completed_publisher_) {
            std_msgs::msg::Bool msg;
            msg.data = true;
            mission_completed_publisher_->publish(msg);
          }
        } else {
          if (beginMissionLeg()) {
            mission_leg_started_at_ = steady_now;
          } else {
            parameters_.type = "hold";
            active_segment_.reset();
          }
        }
      }
    }

    // Preview continuous future legs while the current one is on schedule. If
    // the vehicle is late, hold the active endpoint until it is reached.
    Reference message;
    message.header.stamp = get_clock()->now();
    message.header.frame_id = frame_id_;
    message.trajectory_id = trajectory_id_++;
    message.hold_after_end = true;

    const uint64_t count = horizon_ns / sample_ns + 1;
    message.points.reserve(static_cast<std::size_t>(count));

    for (uint64_t index = 0; index < count; ++index) {
      Point point;
      point.time_from_start = durationMessage(index * sample_ns);
      const double dt_horizon = static_cast<double>(index * sample_ns) * 1.0e-9;

      if (parameters_.type == "mission" && mission_leg_started_at_ && active_segment_ &&
          mission_wp_index_ < mission_waypoints_.size()) {
        const double leg_elapsed = std::max(0.0,
          std::chrono::duration<double>(steady_now - *mission_leg_started_at_).count());
        const bool on_schedule = leg_elapsed <= mission_leg_duration_s_;
        const auto sample = on_schedule ? missionSampleAt(leg_elapsed + dt_horizon) :
          stationary(activeMissionSample(active_segment_->durationSeconds()));
        point.position = sample.position;
        point.velocity = sample.velocity;
        point.acceleration = sample.acceleration;
        point.yaw = sample.yaw;
        point.yaw_rate = sample.yaw_rate;
        message.points.push_back(point);
        continue;
      }

      point.position = parameters_.hold_position;
      point.yaw = parameters_.hold_yaw_rad;
      message.points.push_back(point);
    }

    publisher_->publish(message);
    last_reference_ = message;

    // Publish RViz Visualization
    if (visualization_enabled_ && visualization_publisher_) {
      publishVisualization(steady_now);
    }
  }

  void publishVisualization(const SteadyClock::time_point & steady_now)
  {
    if (last_visualization_published_at_ &&
      std::chrono::duration<double>(steady_now - *last_visualization_published_at_).count()
      < (1.0 / std::max(visualization_publish_rate_hz_, 1.0))) {
      return;
    }
    last_visualization_published_at_ = steady_now;

    visualization_msgs::msg::MarkerArray array;

    // Marker 1: Mission Waypoint Path (Line Strip)
    visualization_msgs::msg::Marker path_marker;
    path_marker.header.stamp = get_clock()->now();
    path_marker.header.frame_id = frame_id_;
    path_marker.ns = "mission_path";
    path_marker.id = 0;
    path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    path_marker.action = visualization_msgs::msg::Marker::ADD;
    path_marker.scale.x = 0.15;
    path_marker.color.r = 0.0f;
    path_marker.color.g = 0.8f;
    path_marker.color.b = 0.2f;
    path_marker.color.a = 0.8f;

    for (const auto & wp : mission_waypoints_) {
      geometry_msgs::msg::Point p;
      p.x = wp.position[0];
      p.y = wp.position[1];
      p.z = wp.position[2];
      path_marker.points.push_back(p);
    }
    array.markers.push_back(path_marker);

    // Marker 2: Waypoint Spheres
    for (size_t i = 0; i < mission_waypoints_.size(); ++i) {
      visualization_msgs::msg::Marker wp_marker;
      wp_marker.header.stamp = get_clock()->now();
      wp_marker.header.frame_id = frame_id_;
      wp_marker.ns = "mission_waypoints";
      wp_marker.id = static_cast<int>(i + 1);
      wp_marker.type = visualization_msgs::msg::Marker::SPHERE;
      wp_marker.action = visualization_msgs::msg::Marker::ADD;
      wp_marker.pose.position.x = mission_waypoints_[i].position[0];
      wp_marker.pose.position.y = mission_waypoints_[i].position[1];
      wp_marker.pose.position.z = mission_waypoints_[i].position[2];
      wp_marker.scale.x = 0.4;
      wp_marker.scale.y = 0.4;
      wp_marker.scale.z = 0.4;
      if (i == mission_wp_index_) {
        wp_marker.color.r = 1.0f;
        wp_marker.color.g = 0.6f;
        wp_marker.color.b = 0.0f;
        wp_marker.color.a = 1.0f;
      } else {
        wp_marker.color.r = 0.1f;
        wp_marker.color.g = 0.5f;
        wp_marker.color.b = 1.0f;
        wp_marker.color.a = 0.8f;
      }
      array.markers.push_back(wp_marker);
    }

    // Marker 3: MPC Prediction Horizon Preview (Cyan Line Strip)
    if (last_reference_ && !last_reference_->points.empty()) {
      visualization_msgs::msg::Marker preview_marker;
      preview_marker.header.stamp = get_clock()->now();
      preview_marker.header.frame_id = frame_id_;
      preview_marker.ns = "mpc_horizon_preview";
      preview_marker.id = 100;
      preview_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      preview_marker.action = visualization_msgs::msg::Marker::ADD;
      preview_marker.scale.x = 0.08;
      preview_marker.color.r = 0.0f;
      preview_marker.color.g = 1.0f;
      preview_marker.color.b = 1.0f;
      preview_marker.color.a = 0.9f;

      for (const auto & pt : last_reference_->points) {
        geometry_msgs::msg::Point p;
        p.x = pt.position[0];
        p.y = pt.position[1];
        p.z = pt.position[2];
        preview_marker.points.push_back(p);
      }
      array.markers.push_back(preview_marker);
    }

    visualization_publisher_->publish(array);
  }

  // Configuration and State Variables
  mpc_controller::reference::Parameters parameters_{};
  std::string mission_file_path_{"config/missions/benchmark_square.json"};
  double mission_acceptance_radius_m_ = 2.5;
  double mission_speed_override_m_s_ = 0.0;
  std::vector<MissionWaypoint> mission_waypoints_;
  std::vector<TrajectorySegment> mission_segments_;
  std::size_t mission_wp_index_ = 0;
  double mission_leg_duration_s_ = 0.0;
  std::optional<SteadyClock::time_point> mission_leg_started_at_;
  std::optional<TrajectorySegment> active_segment_;
  std::string frame_id_ = "map";
  double horizon_seconds_ = 30.0;
  double sample_period_seconds_ = 0.1;
  double publish_rate_hz_ = 50.0;
  bool visualization_enabled_ = true;
  double visualization_publish_rate_hz_ = 20.0;
  bool auto_capture_current_hold_ = true;
  bool external_mode_active_ = false;
  std::optional<StateInput> latest_input_;
  std::optional<Reference> last_reference_;
  std::optional<SteadyClock::time_point> last_visualization_published_at_;
  uint64_t trajectory_id_ = 1;
  std::string state_topic_ = "vehicle_state";
  bool valid_config_ = false;

  // ROS Handles
  rclcpp::Publisher<Reference>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mission_completed_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visualization_publisher_;
  rclcpp::Subscription<State>::SharedPtr state_subscription_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr external_mode_subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_mission_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_mission_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ReferenceGeneratorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
