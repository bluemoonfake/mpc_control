#include "mpc_controller/msg/reference_trajectory.hpp"
#include "mpc_controller/msg/trajectory_point.hpp"
#include "mpc_controller/msg/vehicle_state.hpp"
#include "mpc_controller/mission_parser.hpp"
#include "mpc_controller/reference_model.hpp"

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
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
    declareAndGet("mission_file_path", mission_file_path_);
    declareAndGet("mission_acceptance_radius_m", mission_acceptance_radius_m_);
    declareAndGet("mission_speed_override_m_s", mission_speed_override_m_s_);
    declareAndGet("auto_start_mission_on_offboard", auto_start_mission_on_offboard_);
    declareAndGet("hold_yaw_rad", parameters_.hold_yaw_rad);
    declareAndGet("auto_capture_current_hold", auto_capture_current_hold_);
    declareAndGet("state_timeout_seconds", state_timeout_seconds_);
    declareAndGet("horizon_seconds", horizon_seconds_);
    declareAndGet("sample_period_seconds", sample_period_seconds_);
    declareAndGet("publish_rate_hz", publish_rate_hz_);
    declareAndGet("visualization_enabled", visualization_enabled_);
    declareAndGet("visualization_publish_rate_hz", visualization_publish_rate_hz_);
    declareAndGet("visualization_arrow_length_m", visualization_arrow_length_m_);
    declareAndGet("visualization_direction_deadband", visualization_direction_deadband_);

    declare_parameter("hold_position", std::vector<double>{0.0, 0.0, 1.0});
    getVectorParameter("hold_position", parameters_.hold_position);

    valid_config_ = !frame_id_.empty()
      && std::isfinite(horizon_seconds_) && horizon_seconds_ > 0.0
      && std::isfinite(sample_period_seconds_) && sample_period_seconds_ > 0.0
      && std::isfinite(publish_rate_hz_) && publish_rate_hz_ > 0.0
      && std::isfinite(state_timeout_seconds_) && state_timeout_seconds_ > 0.0;

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
      "/vehicle_state_bridge_node/vehicle_state", qos,
      std::bind(&ReferenceGeneratorNode::stateCallback, this, std::placeholders::_1));

    control_mode_subscription_ = create_subscription<px4_msgs::msg::VehicleControlMode>(
      "/fmu/out/vehicle_control_mode", qos,
      std::bind(&ReferenceGeneratorNode::controlModeCallback, this, std::placeholders::_1));

    // Services
    start_mission_service_ = create_service<std_srvs::srv::Trigger>(
      "~/start_mission",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
             std_srvs::srv::Trigger::Response::SharedPtr response) {
        startMission();
        response->success = true;
        response->message = "Mission started";
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

  double computeLegDuration(const std::array<double, 3> & from, const MissionWaypoint & to) const
  {
    const double dx = to.position[0] - from[0];
    const double dy = to.position[1] - from[1];
    const double dz = to.position[2] - from[2];
    const double d_xy = std::hypot(dx, dy);
    const double d_z = std::abs(dz);
    const double t_xy = (d_xy > 1e-3 && to.horizontal_speed > 0.0) ? d_xy / to.horizontal_speed : 0.0;
    const double t_z = (d_z > 1e-3 && to.vertical_speed > 0.0) ? d_z / to.vertical_speed : 0.0;
    return std::max({t_xy, t_z, 0.5}) + to.hold_duration_s;
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
    const double h_speed = (mission_speed_override_m_s_ > 0.0)
      ? mission_speed_override_m_s_ : mission.defaults.horizontal_velocity_m_s;
    const double v_speed = mission.defaults.vertical_velocity_m_s;

    for (const auto & item : mission.items) {
      if (item.type == mpc_controller::mission::ItemType::Takeoff) {
        MissionWaypoint wp;
        wp.id = item.id.empty() ? "takeoff" : item.id;
        wp.position = parameters_.hold_position;
        if (std::isfinite(item.waypoint.position_enu[2]) && item.waypoint.position_enu[2] > parameters_.hold_position[2]) {
          wp.position[2] = item.waypoint.position_enu[2];
        }
        wp.horizontal_speed = 0.0;
        wp.vertical_speed = std::clamp(v_speed, 0.5, 1.2);
        mission_waypoints_.push_back(wp);
      } else if (item.type == mpc_controller::mission::ItemType::Waypoint) {
        MissionWaypoint wp;
        wp.id = item.id;
        wp.position = item.waypoint.position_enu;
        wp.horizontal_speed = h_speed;
        wp.vertical_speed = v_speed;
        mission_waypoints_.push_back(wp);
      } else if (item.type == mpc_controller::mission::ItemType::Hold) {
        if (!mission_waypoints_.empty()) {
          mission_waypoints_.back().hold_duration_s = item.hold.duration_seconds;
        }
      } else if (item.type == mpc_controller::mission::ItemType::Land) {
        MissionWaypoint wp;
        wp.id = item.id.empty() ? "landing" : item.id;
        wp.position = mission_waypoints_.empty() ? parameters_.hold_position : mission_waypoints_.back().position;
        wp.position[2] = 0.0;
        wp.horizontal_speed = 0.0;
        wp.vertical_speed = std::clamp(v_speed, 0.4, 0.8);
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

  void startMission()
  {
    if (mission_waypoints_.empty() && !loadMissionWaypoints()) {
      RCLCPP_ERROR(get_logger(), "Cannot start mission: no waypoints loaded");
      return;
    }

    mission_wp_index_ = 0;
    if (latest_input_ && latest_input_->valid) {
      mission_leg_start_pos_ = latest_input_->position;
      mission_leg_start_yaw_ = latest_input_->yaw;
    } else {
      mission_leg_start_pos_ = parameters_.hold_position;
      mission_leg_start_yaw_ = parameters_.hold_yaw_rad;
    }
    mission_leg_duration_s_ = computeLegDuration(mission_leg_start_pos_, mission_waypoints_[0]);
    mission_leg_started_at_ = SteadyClock::now();
    parameters_.type = "mission";

    RCLCPP_INFO(
      get_logger(), "Mission started: %zu waypoints, targeting wp 1 ('%s') [%.2f, %.2f, %.2f]",
      mission_waypoints_.size(), mission_waypoints_[0].id.c_str(),
      mission_waypoints_[0].position[0], mission_waypoints_[0].position[1],
      mission_waypoints_[0].position[2]);
  }

  void resetMission()
  {
    mission_wp_index_ = 0;
    mission_leg_started_at_.reset();
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
    const auto steady_now = SteadyClock::now();
    last_state_received_at_ = steady_now;
    last_state_timestamp_ = message->header.stamp.sec * 1000000000ULL + message->header.stamp.nanosec;

    StateInput input;
    input.position = {message->position[0], message->position[1], message->position[2]};
    input.velocity = {message->velocity[0], message->velocity[1], message->velocity[2]};
    input.yaw = message->yaw;
    input.valid = message->control_ready;
    latest_input_ = input;

    if (auto_capture_current_hold_ && !hold_reference_captured_ && input.valid) {
      parameters_.hold_position = input.position;
      parameters_.hold_yaw_rad = input.yaw;
      hold_reference_captured_ = true;
      RCLCPP_INFO(
        get_logger(),
        "Initial hold position captured: [%.3f, %.3f, %.3f] yaw=%.3f rad",
        parameters_.hold_position[0], parameters_.hold_position[1],
        parameters_.hold_position[2], parameters_.hold_yaw_rad);
    }
  }

  void controlModeCallback(const px4_msgs::msg::VehicleControlMode::SharedPtr message)
  {
    const bool is_offboard = message->flag_control_offboard_enabled;
    if (is_offboard && !offboard_active_) {
      RCLCPP_INFO(get_logger(), "Offboard mode activated");
      if (auto_start_mission_on_offboard_) {
        startMission();
      }
    } else if (!is_offboard && offboard_active_) {
      RCLCPP_INFO(get_logger(), "Offboard mode deactivated");
    }
    offboard_active_ = is_offboard;
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

    // Waypoint progression check
    if (parameters_.type == "mission" && mission_leg_started_at_ && !mission_waypoints_.empty()) {
      const auto & target_wp = mission_waypoints_[mission_wp_index_];
      const double leg_elapsed = std::chrono::duration<double>(steady_now - *mission_leg_started_at_).count();
      double dist_to_target = 1000.0;
      bool crossed_finish_plane = false;

      if (latest_input_ && latest_input_->valid) {
        const double ex = latest_input_->position[0] - target_wp.position[0];
        const double ey = latest_input_->position[1] - target_wp.position[1];
        const double ez = latest_input_->position[2] - target_wp.position[2];
        dist_to_target = std::sqrt(ex * ex + ey * ey + ez * ez);

        // Cross-track plane test: has the drone passed the waypoint plane along the leg direction?
        const double leg_dx = target_wp.position[0] - mission_leg_start_pos_[0];
        const double leg_dy = target_wp.position[1] - mission_leg_start_pos_[1];
        const double leg_dz = target_wp.position[2] - mission_leg_start_pos_[2];
        const double leg_len = std::sqrt(leg_dx * leg_dx + leg_dy * leg_dy + leg_dz * leg_dz);
        if (leg_len > 1e-3) {
          const double dot = ex * (leg_dx / leg_len) + ey * (leg_dy / leg_len) + ez * (leg_dz / leg_len);
          crossed_finish_plane = (dot >= 0.0);
        }
      }

      const bool is_landing_wp = (target_wp.id == "landing" || target_wp.position[2] <= 0.1);
      const double acceptance_radius = is_landing_wp ? 0.35 : mission_acceptance_radius_m_;

      // Waypoint reached conditions:
      // 1. Classic: within acceptance radius and hold duration satisfied
      // 2. Crossed finish plane: drone has flown past the waypoint plane (at least 50% leg time)
      // 3. Time elapsed: allocated cruise time for the leg has elapsed and drone is in proximity
      const bool waypoint_reached =
        (dist_to_target < acceptance_radius && leg_elapsed >= target_wp.hold_duration_s)
        || (crossed_finish_plane && !is_landing_wp && leg_elapsed >= 0.5 * mission_leg_duration_s_)
        || (leg_elapsed >= mission_leg_duration_s_ && !is_landing_wp && dist_to_target < std::max(4.0, acceptance_radius * 2.5));

      if (waypoint_reached) {
        RCLCPP_INFO(
          get_logger(),
          "Mission waypoint %zu/%zu reached: id='%s' [%.2f, %.2f, %.2f] (dist=%.2fm, crossed_plane=%s)",
          mission_wp_index_ + 1, mission_waypoints_.size(),
          target_wp.id.c_str(), target_wp.position[0], target_wp.position[1], target_wp.position[2],
          dist_to_target, crossed_finish_plane ? "yes" : "no");

        const double dx_leg = target_wp.position[0] - mission_leg_start_pos_[0];
        const double dy_leg = target_wp.position[1] - mission_leg_start_pos_[1];
        const double completed_leg_yaw = (std::hypot(dy_leg, dx_leg) > 0.5)
          ? std::atan2(dy_leg, dx_leg) : mission_leg_start_yaw_;

        mission_wp_index_++;
        if (mission_wp_index_ >= mission_waypoints_.size()) {
          parameters_.hold_position = target_wp.position;
          parameters_.type = "hold";
          mission_leg_started_at_.reset();
          RCLCPP_INFO(get_logger(), "Mission completed successfully; handing off to native landing...");

          if (mission_completed_publisher_) {
            std_msgs::msg::Bool msg;
            msg.data = true;
            mission_completed_publisher_->publish(msg);
          }
        } else {
          mission_leg_start_pos_ = target_wp.position;
          mission_leg_start_yaw_ = completed_leg_yaw;
          mission_leg_duration_s_ = computeLegDuration(mission_leg_start_pos_, mission_waypoints_[mission_wp_index_]);
          mission_leg_started_at_ = steady_now;
        }
      }
    }

    // Build Reference message with 30-step Multi-Waypoint Horizon Preview
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

      if (parameters_.type == "mission" && mission_leg_started_at_ && mission_wp_index_ < mission_waypoints_.size()) {
        const double leg_elapsed = std::max(0.0,
          std::chrono::duration<double>(steady_now - *mission_leg_started_at_).count());
        const auto & target_wp = mission_waypoints_[mission_wp_index_];
        const bool is_takeoff_or_landing = (target_wp.id == "takeoff" || target_wp.id == "landing");

        if (is_takeoff_or_landing) {
          const double t_future = leg_elapsed + dt_horizon;
          const double alpha = (mission_leg_duration_s_ > 1e-3)
            ? std::min(1.0, t_future / mission_leg_duration_s_) : 1.0;
          const double dx = target_wp.position[0] - mission_leg_start_pos_[0];
          const double dy = target_wp.position[1] - mission_leg_start_pos_[1];
          const double dz = target_wp.position[2] - mission_leg_start_pos_[2];

          // Slew-rate smoothed interpolation (Smoothstep) for takeoff & landing
          const double s_pos = (alpha < 1.0) ? (3.0 * alpha * alpha - 2.0 * alpha * alpha * alpha) : 1.0;
          const double s_vel = (alpha < 1.0 && mission_leg_duration_s_ > 1e-3) ? (6.0 * alpha * (1.0 - alpha)) : 0.0;

          point.position[0] = mission_leg_start_pos_[0] + s_pos * dx;
          point.position[1] = mission_leg_start_pos_[1] + s_pos * dy;
          point.position[2] = mission_leg_start_pos_[2] + s_pos * dz;

          if (alpha < 1.0 && mission_leg_duration_s_ > 1e-3) {
            point.velocity[0] = (s_vel * dx) / mission_leg_duration_s_;
            point.velocity[1] = (s_vel * dy) / mission_leg_duration_s_;
            point.velocity[2] = (s_vel * dz) / mission_leg_duration_s_;
            point.yaw = (std::hypot(dy, dx) > 1e-3) ? std::atan2(dy, dx) : parameters_.hold_yaw_rad;
          } else {
            point.velocity = {0.0, 0.0, 0.0};
            point.yaw = parameters_.hold_yaw_rad;
          }
        } else {
          // Multi-Waypoint Horizon Preview: lookahead seamlessly across current and upcoming legs
          const double rem_in_curr_leg = std::max(0.0, mission_leg_duration_s_ - leg_elapsed);
          if (dt_horizon <= rem_in_curr_leg || mission_wp_index_ + 1 >= mission_waypoints_.size()) {
            const double t_future = leg_elapsed + dt_horizon;
            const double alpha = (mission_leg_duration_s_ > 1e-3)
              ? std::min(1.0, t_future / mission_leg_duration_s_) : 1.0;
            const double dx = target_wp.position[0] - mission_leg_start_pos_[0];
            const double dy = target_wp.position[1] - mission_leg_start_pos_[1];
            const double dz = target_wp.position[2] - mission_leg_start_pos_[2];

            point.position[0] = mission_leg_start_pos_[0] + alpha * dx;
            point.position[1] = mission_leg_start_pos_[1] + alpha * dy;
            point.position[2] = mission_leg_start_pos_[2] + alpha * dz;

            if (alpha < 1.0 && mission_leg_duration_s_ > 1e-3) {
              point.velocity[0] = dx / mission_leg_duration_s_;
              point.velocity[1] = dy / mission_leg_duration_s_;
              point.velocity[2] = dz / mission_leg_duration_s_;
              point.yaw = (std::hypot(dy, dx) > 1e-3) ? std::atan2(dy, dx) : parameters_.hold_yaw_rad;
            } else {
              point.velocity = {0.0, 0.0, 0.0};
              point.yaw = parameters_.hold_yaw_rad;
            }
          } else {
            // dt_horizon extends into future waypoints along the mission path
            double t_rem = dt_horizon - rem_in_curr_leg;
            std::size_t next_idx = mission_wp_index_ + 1;
            std::array<double, 3> seg_start = target_wp.position;
            bool found = false;

            while (next_idx < mission_waypoints_.size()) {
              const auto & future_wp = mission_waypoints_[next_idx];
              const double leg_dur = computeLegDuration(seg_start, future_wp);
              const double dx = future_wp.position[0] - seg_start[0];
              const double dy = future_wp.position[1] - seg_start[1];
              const double dz = future_wp.position[2] - seg_start[2];

              if (t_rem <= leg_dur) {
                const double alpha = (leg_dur > 1e-3) ? (t_rem / leg_dur) : 1.0;
                point.position[0] = seg_start[0] + alpha * dx;
                point.position[1] = seg_start[1] + alpha * dy;
                point.position[2] = seg_start[2] + alpha * dz;
                point.velocity[0] = (leg_dur > 1e-3) ? (dx / leg_dur) : 0.0;
                point.velocity[1] = (leg_dur > 1e-3) ? (dy / leg_dur) : 0.0;
                point.velocity[2] = (leg_dur > 1e-3) ? (dz / leg_dur) : 0.0;
                point.yaw = (std::hypot(dy, dx) > 1e-3) ? std::atan2(dy, dx) : parameters_.hold_yaw_rad;
                found = true;
                break;
              }
              t_rem -= leg_dur;
              seg_start = future_wp.position;
              next_idx++;
            }

            if (!found) {
              const auto & last_wp = mission_waypoints_.back();
              point.position = last_wp.position;
              point.velocity = {0.0, 0.0, 0.0};
              point.yaw = parameters_.hold_yaw_rad;
            }
          }
        }
        point.acceleration = {0.0, 0.0, 0.0};
        point.yaw_rate = 0.0;
      } else {
        // Hold position fallback
        point.position = parameters_.hold_position;
        point.velocity = {0.0, 0.0, 0.0};
        point.acceleration = {0.0, 0.0, 0.0};
        point.yaw = parameters_.hold_yaw_rad;
        point.yaw_rate = 0.0;
      }
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
  bool auto_start_mission_on_offboard_ = true;
  std::vector<MissionWaypoint> mission_waypoints_;
  std::size_t mission_wp_index_ = 0;
  std::array<double, 3> mission_leg_start_pos_{0.0, 0.0, 1.0};
  double mission_leg_start_yaw_ = 0.0;
  double mission_leg_duration_s_ = 0.0;
  std::optional<SteadyClock::time_point> mission_leg_started_at_;
  std::string frame_id_ = "map";
  double horizon_seconds_ = 30.0;
  double sample_period_seconds_ = 0.1;
  double publish_rate_hz_ = 50.0;
  bool visualization_enabled_ = true;
  double visualization_publish_rate_hz_ = 20.0;
  double visualization_arrow_length_m_ = 1.0;
  double visualization_direction_deadband_ = 0.08;
  double state_timeout_seconds_ = 0.25;
  bool auto_capture_current_hold_ = true;
  bool offboard_active_ = false;
  bool hold_reference_captured_ = false;
  std::optional<StateInput> latest_input_;
  std::optional<uint64_t> last_state_timestamp_;
  std::optional<SteadyClock::time_point> last_state_received_at_;
  std::optional<Reference> last_reference_;
  std::optional<SteadyClock::time_point> last_visualization_published_at_;
  uint64_t trajectory_id_ = 1;
  bool valid_config_ = false;

  // ROS Handles
  rclcpp::Publisher<Reference>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mission_completed_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visualization_publisher_;
  rclcpp::Subscription<State>::SharedPtr state_subscription_;
  rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr control_mode_subscription_;
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
