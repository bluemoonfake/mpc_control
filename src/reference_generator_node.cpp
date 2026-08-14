#include "mpc_controller/msg/mpc_translational_output.hpp"
#include "mpc_controller/msg/reference_trajectory.hpp"
#include "mpc_controller/msg/reference_step.hpp"
#include "mpc_controller/msg/trajectory_point.hpp"
#include "mpc_controller/msg/vehicle_state.hpp"
#include "mpc_controller/reference_model.hpp"

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/manual_control_setpoint.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
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
#include <utility>
#include <vector>

class ReferenceGeneratorNode final : public rclcpp::Node
{
public:
  ReferenceGeneratorNode()
  : Node("reference_generator_node")
  {
    // Declare one configuration surface for both autonomous trajectories and
    // the rolling joystick planner. Mode-specific settings remain dormant.
    declareAndGet("reference_input_mode", reference_input_mode_);
    declareAndGet("frame_id", frame_id_);
    declareAndGet("line_duration_seconds", parameters_.line_duration_seconds);
    declareAndGet("circle_radius", parameters_.circle_radius);
    declareAndGet("circle_reference_speed_limit_m_s", circle_reference_speed_limit_m_s_);
    declareAndGet("circle_acceleration_limit_m_s2", circle_acceleration_limit_m_s2_);
    declareAndGet("circle_direction", parameters_.circle_direction);
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
    declareAndGet("manual_input_timeout_seconds", manual_timeout_);
    declareAndGet("manual_update_rate_hz", manual_rate_hz_);
    declareAndGet("manual_stick_deadband", manual_deadband_);
    declareAndGet("manual_max_horizontal_speed_m_s", planner_config_.max_speed_xy);
    declareAndGet("manual_max_vertical_speed_m_s", planner_config_.max_speed_z);
    declareAndGet("manual_max_horizontal_acceleration_m_s2", planner_config_.max_acceleration_xy);
    declareAndGet("manual_max_vertical_acceleration_m_s2", planner_config_.max_acceleration_z);
    declareAndGet("manual_max_horizontal_jerk_m_s3", planner_config_.max_jerk_xy);
    declareAndGet("manual_max_vertical_jerk_m_s3", planner_config_.max_jerk_z);
    declareAndGet("manual_max_yaw_rate_rad_s", manual_yaw_rate_max_);
    declareAndGet("manual_max_horizontal_position_error_m", manual_lead_xy_);
    declareAndGet("manual_max_vertical_position_error_m", manual_lead_z_);
    declareAndGet("planner_horizon_seconds", planner_config_.horizon_seconds);
    declareAndGet("planner_sample_period_seconds", planner_config_.sample_seconds);
    declareAndGet("planner_velocity_response_seconds", planner_config_.response_seconds);
    declareAndGet("planner_intent_weight", planner_config_.intent_weight);
    declareAndGet("planner_progress_weight", planner_config_.progress_weight);
    declareAndGet("planner_acceleration_weight", planner_config_.acceleration_weight);
    declareAndGet("planner_jerk_weight", planner_config_.jerk_weight);
    declareAndGet("planner_switch_weight", planner_config_.switch_weight);
    declareAndGet("planner_hysteresis", planner_config_.hysteresis);
    declareAndGet("visualization_max_prediction_points", visualization_max_prediction_points_);

    declare_parameter("hold_position", std::vector<double>{0.0, 0.0, 1.0});
    declare_parameter("line_relative_delta", std::vector<double>{2.0, 0.0, 0.0});
    declare_parameter("planner_heading_offsets_deg", std::vector<double>{-45.0, -30.0, -15.0, 0.0, 15.0, 30.0, 45.0});
    declare_parameter("planner_speed_scales", planner_config_.speed_scales);

    valid_config_ = true;
    getVectorParameter("hold_position", parameters_.hold_position);
    getVectorParameter("line_relative_delta", line_relative_delta_);
    const auto heading_offsets_deg = get_parameter("planner_heading_offsets_deg").as_double_array();
    planner_config_.heading_offsets_rad.clear();
    planner_config_.heading_offsets_rad.reserve(heading_offsets_deg.size());
    constexpr double degrees_to_radians = 0.017453292519943295;
    for (double offset : heading_offsets_deg) {
      planner_config_.heading_offsets_rad.push_back(offset * degrees_to_radians);
    }
    planner_config_.speed_scales = get_parameter("planner_speed_scales").as_double_array();
    get_parameter("planner_intent_weight", planner_config_.intent_weight);
    get_parameter("planner_progress_weight", planner_config_.progress_weight);
    get_parameter("planner_acceleration_weight", planner_config_.acceleration_weight);
    get_parameter("planner_jerk_weight", planner_config_.jerk_weight);
    get_parameter("planner_switch_weight", planner_config_.switch_weight);
    get_parameter("planner_hysteresis", planner_config_.hysteresis);
    get_parameter("visualization_max_prediction_points", visualization_max_prediction_points_);

    // Circle period and ramp are derived, not independently tuned, so radius,
    // speed and centripetal acceleration cannot contradict each other.
    const auto circle_timing = mpc_controller::reference::deriveCircleTiming(
      parameters_.circle_radius, circle_reference_speed_limit_m_s_,
      circle_acceleration_limit_m_s2_);
    if (circle_timing.valid) {
      parameters_.circle_period_seconds = circle_timing.period_seconds;
      parameters_.circle_ramp_seconds = circle_timing.ramp_seconds;
      circle_cruise_speed_m_s_ = circle_timing.cruise_speed_m_s;
    }

    valid_config_ = valid_config_ && circle_timing.valid
      && mpc_controller::reference::valid(parameters_)
      && std::all_of(line_relative_delta_.begin(), line_relative_delta_.end(),[](double value) {return std::isfinite(value);})
      && std::any_of(line_relative_delta_.begin(), line_relative_delta_.end(),[](double value) {return std::abs(value) > 0.0;})
      && std::isfinite(horizon_seconds_) && horizon_seconds_ >= 0.0
      && std::isfinite(sample_period_seconds_) && sample_period_seconds_ > 0.0
      && std::isfinite(publish_rate_hz_) && publish_rate_hz_ > 0.0
      && std::isfinite(state_timeout_seconds_) && state_timeout_seconds_ > 0.0
      && std::isfinite(visualization_publish_rate_hz_) && visualization_publish_rate_hz_ > 0.0
      && std::isfinite(visualization_arrow_length_m_) && visualization_arrow_length_m_ > 0.0
      && std::isfinite(visualization_direction_deadband_)
      && visualization_direction_deadband_ >= 0.0
      && std::isfinite(manual_timeout_) && manual_timeout_ > 0.0
      && (reference_input_mode_ == "trajectory" || reference_input_mode_ == "manual_velocity")
      && std::isfinite(manual_rate_hz_) && manual_rate_hz_ > 0.0
      && std::isfinite(manual_deadband_) && manual_deadband_ >= 0.0
      && manual_deadband_ < 1.0
      && std::isfinite(manual_yaw_rate_max_) && manual_yaw_rate_max_ > 0.0
      && std::isfinite(manual_lead_xy_) && manual_lead_xy_ > 0.0
      && std::isfinite(manual_lead_z_) && manual_lead_z_ > 0.0
      && mpc_controller::reference::valid(planner_config_)
      && visualization_max_prediction_points_ > 0
      && !frame_id_.empty();
    if (!valid_config_) {
      RCLCPP_ERROR(get_logger(), "Invalid reference generator parameters; publishing disabled");
    } else {
      RCLCPP_INFO(
        get_logger(),
        "Circle timing derived: radius=%.3f m cruise=%.3f m/s ramp=%.3f s period=%.3f s "
        "acceleration_limit=%.3f m/s^2",
        parameters_.circle_radius, circle_cruise_speed_m_s_,
        parameters_.circle_ramp_seconds, parameters_.circle_period_seconds,
        circle_acceleration_limit_m_s2_);
      // ROS wiring: measured state/manual input enter here; exactly one
      // ReferenceTrajectory stream leaves this node.
      publisher_ = create_publisher<Reference>("reference_trajectory", 10);
      step_subscription_ = create_subscription<ReferenceStep>("reference_step", 10,std::bind(&ReferenceGeneratorNode::referenceStepCallback, this, std::placeholders::_1));
      state_subscription_ = create_subscription<State>(
        "vehicle_state", rclcpp::QoS(10),std::bind(&ReferenceGeneratorNode::stateCallback, this, std::placeholders::_1));
      start_line_service_ = create_service<Trigger>(
        "~/start_line",std::bind(&ReferenceGeneratorNode::startLine, this, std::placeholders::_1,std::placeholders::_2));
      start_circle_service_ = create_service<Trigger>(
        "~/start_circle",std::bind(&ReferenceGeneratorNode::startCircle, this, std::placeholders::_1,std::placeholders::_2));
      RCLCPP_INFO(
        get_logger(), auto_capture_current_hold_
        ? "Current-state hold tracking enabled; freezes on PX4 Offboard entry"
        : "Current-state hold tracking disabled; configured hold is used");
      if (auto_capture_current_hold_ || manualVelocityMode()) {
        const auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
        control_mode_subscription_ = create_subscription<px4_msgs::msg::VehicleControlMode>("fmu/out/vehicle_control_mode", sensor_qos,
          [this](px4_msgs::msg::VehicleControlMode::SharedPtr message) {
            if (!message) return;
            const bool was_offboard = offboard_active_;
            offboard_active_ = message->flag_control_offboard_enabled;
            if (!was_offboard && offboard_active_ && hold_reference_captured_) {
              if (manualVelocityMode()) {
                resetManual();
                publishManual();
              } else {
                publish();
              }
              RCLCPP_INFO(
                get_logger(),
                "SITL hold frozen on Offboard entry: position=[%.3f %.3f %.3f] yaw=%.4f mode=%s",
                parameters_.hold_position[0], parameters_.hold_position[1],
                parameters_.hold_position[2], parameters_.hold_yaw_rad,
                reference_input_mode_.c_str());
            } else if (was_offboard && !offboard_active_) {
              manual_ready_ = false;
              manual_updated_at_.reset();
            }
          });
      }
      if (visualization_enabled_ || manualVelocityMode()) {
        const auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
        manual_control_subscription_ = create_subscription<ManualControl>("fmu/out/manual_control_setpoint", sensor_qos,
          [this](ManualControl::SharedPtr message) {
            if (!message) return;
            manual_input_ = *message;
            manual_received_at_ = SteadyClock::now();
          });
      }
      mpc_output_subscription_ = create_subscription<MpcOutput>(
        "mpc_translational_output", rclcpp::QoS(10),
        [this](MpcOutput::SharedPtr message) {
          if (!message) return;
          latest_mpc_output_ = *message;
          mpc_output_received_at_ = SteadyClock::now();
          if (!message->valid) cancelTrajectoryForMpcFailure();
          if (visualization_enabled_) publishVisualization();
        });
      if (visualization_enabled_) {
        visualization_publisher_ = create_publisher<MarkerArray>("mpc_visualization", rclcpp::QoS(1).transient_local());
        RCLCPP_INFO(
          get_logger(),
          "MPC visualization enabled: user=green reference=red prediction=cyan");
      }
      timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / publish_rate_hz_)),
        std::bind(&ReferenceGeneratorNode::publish, this));
      if (manualVelocityMode()) {
        manual_timer_ = create_wall_timer(
          std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / manual_rate_hz_)),
          std::bind(&ReferenceGeneratorNode::manualUpdate, this));
        RCLCPP_INFO(get_logger(),
          "Predictive joystick planner enabled: horizon=%.2f s candidates=%zu "
          "speed=[%.2f %.2f] m/s acceleration=[%.2f %.2f] m/s^2 "
          "jerk=[%.2f %.2f] m/s^3",
          planner_config_.horizon_seconds,
          planner_config_.heading_offsets_rad.size() * planner_config_.speed_scales.size() + 1U,
          planner_config_.max_speed_xy, planner_config_.max_speed_z,
          planner_config_.max_acceleration_xy, planner_config_.max_acceleration_z,
          planner_config_.max_jerk_xy, planner_config_.max_jerk_z);
      }
    }
  }

private:
  template <typename T>
  void declareAndGet(const std::string &name, T &target)
  {
    declare_parameter(name, target);
    get_parameter(name, target);
  }

  using Reference = mpc_controller::msg::ReferenceTrajectory;
  using ReferenceStep = mpc_controller::msg::ReferenceStep;
  using MpcOutput = mpc_controller::msg::MpcTranslationalOutput;
  using Point = mpc_controller::msg::TrajectoryPoint;
  using State = mpc_controller::msg::VehicleState;
  using ManualControl = px4_msgs::msg::ManualControlSetpoint;
  using Marker = visualization_msgs::msg::Marker;
  using MarkerArray = visualization_msgs::msg::MarkerArray;
  using Trigger = std_srvs::srv::Trigger;
  using SteadyClock = std::chrono::steady_clock;

  // Minimal state needed by hold capture, manual reference and visualization.
  struct StateInput
  {
    std::array<double, 3> position{};
    std::array<double, 3> velocity{};
    std::array<double, 3> acceleration{};
    double yaw = 0.0;
    std::uint64_t timestamp = 0;
    bool valid = false;
  };

  bool manualVelocityMode() const noexcept
  {
    return reference_input_mode_ == "manual_velocity";
  }

  bool mpcHealthy() const noexcept
  {
    return latest_mpc_output_ && mpc_output_received_at_
      && latest_mpc_output_->valid && !latest_mpc_output_->recovery_command_active
      && std::chrono::duration<double>(SteadyClock::now() - *mpc_output_received_at_).count()
      <= state_timeout_seconds_;
  }

  void cancelTrajectoryForMpcFailure()
  {
    if (!offboard_active_ || (parameters_.type != "line" && parameters_.type != "circle")) {
      return;
    }
    const std::string cancelled = parameters_.type;
    parameters_.type = "hold";
    line_started_at_.reset();
    circle_started_at_.reset();
    if (latest_input_ && latest_input_->valid) {
      parameters_.hold_position = latest_input_->position;
      parameters_.hold_yaw_rad = latest_input_->yaw;
      hold_reference_captured_ = true;
    }
    publish();
    RCLCPP_ERROR(
      get_logger(),
      "%s cancelled because MPC became invalid; current position/yaw changed to hold",
      cancelled.c_str());
  }

  void stateCallback(const State::SharedPtr message)
  {
    if (!message) {
      return;
    }

    const auto timestamp = rclcpp::Time(message->header.stamp).nanoseconds();
    if (timestamp <= 0 ||
      (last_state_timestamp_ && timestamp < static_cast<int64_t>(*last_state_timestamp_))) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Hold capture state rejected: timestamp is zero or moved backwards");
      return;
    }

    StateInput input;
    input.position = {message->position[0], message->position[1], message->position[2]};
    input.velocity = {message->velocity[0], message->velocity[1], message->velocity[2]};
    const std::array<double, 3> measured_acceleration{
      message->acceleration[0], message->acceleration[1], message->acceleration[2]};
    if (message->acceleration_valid
      && mpc_controller::reference::finite(measured_acceleration)) {
      input.acceleration = measured_acceleration;
    }
    input.yaw = message->yaw;
    input.timestamp = static_cast<std::uint64_t>(timestamp);
    input.valid = message->valid && message->position_valid && message->velocity_valid
      && message->heading_valid;
    latest_input_ = input;
    last_state_timestamp_ = input.timestamp;
    last_state_received_at_ = SteadyClock::now();
    if (auto_capture_current_hold_ && !offboard_active_ && input.valid
      && std::isfinite(input.position[0]) && std::isfinite(input.position[1])
      && std::isfinite(input.position[2]) && std::isfinite(input.yaw)) {
      const bool first_capture = !hold_reference_captured_;
      parameters_.type = "hold";
      line_started_at_.reset();
      circle_started_at_.reset();
      parameters_.hold_position = input.position;
      parameters_.hold_yaw_rad = input.yaw;
      hold_reference_captured_ = true;
      if (first_capture) {
        RCLCPP_INFO(
          get_logger(),
          "SITL current-hold tracking started: position=[%.3f %.3f %.3f] yaw=%.4f",
          input.position[0], input.position[1], input.position[2], input.yaw);
      }
    }
  }

  void referenceStepCallback(const ReferenceStep::SharedPtr message)
  {
    if (manualVelocityMode()) {
      RCLCPP_WARN(get_logger(), "Relative reference step rejected in manual_velocity mode");
      return;
    }
    if (!message || !message->valid || !valid_config_) {
      RCLCPP_WARN(get_logger(), "Relative reference step rejected: invalid command");
      return;
    }
    if (!hold_reference_captured_ || parameters_.type != "hold") {
      RCLCPP_WARN(get_logger(),"Relative reference step rejected: a captured hold reference is not active");
      return;
    }
    if (!std::all_of(
        message->delta_position.begin(), message->delta_position.end(),[](double value) {return std::isfinite(value);})) {
      RCLCPP_WARN(get_logger(), "Relative reference step rejected: non-finite delta");
      return;
    }

    const auto previous_position = parameters_.hold_position;
    for (std::size_t axis = 0; axis < parameters_.hold_position.size(); ++axis) {
      parameters_.hold_position[axis] += message->delta_position[axis];
    }
    publish();
    RCLCPP_INFO(get_logger(),
      "Relative ENU hold step applied: delta=[%.3f %.3f %.3f], "
      "position=[%.3f %.3f %.3f] -> [%.3f %.3f %.3f], yaw held at %.4f",
      message->delta_position[0], message->delta_position[1], message->delta_position[2],
      previous_position[0], previous_position[1], previous_position[2],
      parameters_.hold_position[0], parameters_.hold_position[1],
      parameters_.hold_position[2], parameters_.hold_yaw_rad);
  }

  void startLine(
    const std::shared_ptr<Trigger::Request>, const std::shared_ptr<Trigger::Response> response)
  {
    if (manualVelocityMode()) {
      response->success = false;
      response->message = "line rejected: reference_input_mode is manual_velocity";
      return;
    }
    if (!valid_config_ || !offboard_active_) {
      response->success = false;
      response->message = "line rejected: PX4 Offboard is not active";
      return;
    }
    if (!hold_reference_captured_ || parameters_.type != "hold") {
      response->success = false;
      response->message = "line rejected: a captured hold reference is not active";
      return;
    }

    parameters_.line_start = parameters_.hold_position;
    for (std::size_t axis = 0; axis < parameters_.line_end.size(); ++axis) {
      parameters_.line_end[axis] = parameters_.line_start[axis] + line_relative_delta_[axis];
    }
    parameters_.type = "line";
    circle_started_at_.reset();
    line_started_at_ = SteadyClock::now();
    publish();

    response->success = true;
    response->message = "relative line started";
    RCLCPP_INFO(
      get_logger(),
      "Relative ENU line started: start=[%.3f %.3f %.3f] end=[%.3f %.3f %.3f] "
      "duration=%.3f s yaw_hold=%.4f",
      parameters_.line_start[0], parameters_.line_start[1], parameters_.line_start[2],
      parameters_.line_end[0], parameters_.line_end[1], parameters_.line_end[2],
      parameters_.line_duration_seconds, parameters_.hold_yaw_rad);
  }

  void startCircle(
    const std::shared_ptr<Trigger::Request>, const std::shared_ptr<Trigger::Response> response)
  {
    if (manualVelocityMode()) {
      response->success = false;
      response->message = "circle rejected: reference_input_mode is manual_velocity";
      return;
    }
    if (!valid_config_ || !offboard_active_) {
      response->success = false;
      response->message = "circle rejected: PX4 Offboard is not active";
      return;
    }
    if (!hold_reference_captured_ || parameters_.type != "hold") {
      response->success = false;
      response->message = "circle rejected: a captured hold reference is not active";
      return;
    }
    if (!mpcHealthy()) {
      response->success = false;
      response->message = "circle rejected: MPC output is stale, invalid, or recovering";
      return;
    }

    // phase=0 starts at center+[radius, 0, 0]. Offset the center so the
    // circle begins exactly at the captured hold, without a position jump.
    parameters_.circle_center = parameters_.hold_position;
    parameters_.circle_center[0] -= parameters_.circle_radius;
    parameters_.circle_phase_rad = 0.0;
    parameters_.type = "circle";
    line_started_at_.reset();
    circle_started_at_ = SteadyClock::now();
    publish();

    response->success = true;
    response->message = "circle started";
    RCLCPP_INFO(
      get_logger(),
      "ENU circle started: start=[%.3f %.3f %.3f] center=[%.3f %.3f %.3f] "
      "radius=%.3f m period=%.3f s ramp=%.3f s direction=%d yaw_hold=%.4f",
      parameters_.hold_position[0], parameters_.hold_position[1],
      parameters_.hold_position[2], parameters_.circle_center[0],
      parameters_.circle_center[1], parameters_.circle_center[2],
      parameters_.circle_radius, parameters_.circle_period_seconds,
      parameters_.circle_ramp_seconds,
      parameters_.circle_direction, parameters_.hold_yaw_rad);
  }

  void getVectorParameter(const std::string &name, std::array<double, 3> &output)
  {
    const auto value = get_parameter(name).as_double_array();
    if (value.size() == 3) {
      std::copy(value.begin(), value.end(), output.begin());
    } else {
      valid_config_ = false;
      RCLCPP_ERROR(get_logger(), "Parameter '%s' must contain exactly 3 values", name.c_str());
    }
  }

  static builtin_interfaces::msg::Duration durationMessage(uint64_t nanoseconds)
  {
    builtin_interfaces::msg::Duration duration;
    duration.sec = static_cast<int32_t>(nanoseconds / 1000000000ULL);
    duration.nanosec = static_cast<uint32_t>(nanoseconds % 1000000000ULL);
    return duration;
  }

  static double norm(const std::array<double, 3> &value) noexcept
  {
    return std::sqrt(value[0] * value[0] + value[1] * value[1] + value[2] * value[2]);
  }

  static double applyDeadband(double value, double deadband) noexcept
  {
    if (!std::isfinite(value) || std::abs(value) <= deadband) return 0.0;
    return std::copysign((std::abs(value) - deadband) / (1.0 - deadband), value);
  }

  static double wrapAngle(double angle) noexcept
  {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  double targetYawRate() const noexcept
  {
    if (!manual_input_ || !manual_received_at_) return 0.0;
    const double age = std::chrono::duration<double>(
      SteadyClock::now() - *manual_received_at_).count();
    if (age > manual_timeout_ || !manual_input_->valid
      || !std::isfinite(manual_input_->yaw)) return 0.0;
    // PX4 yaw stick is positive clockwise in NED; ENU yaw is positive CCW.
    return -applyDeadband(manual_input_->yaw, manual_deadband_) * manual_yaw_rate_max_;
  }

  Marker directionMarker(
    int id, const std::string &name, const std::array<double, 3> &direction,
    bool valid, double direction_deadband, float red, float green, float blue) const
  {
    Marker marker;
    marker.header.stamp = get_clock()->now();
    marker.header.frame_id = frame_id_;
    marker.ns = name;
    marker.id = id;
    marker.type = Marker::ARROW;
    marker.action = Marker::DELETE;
    marker.scale.x = 0.055;
    marker.scale.y = 0.13;
    marker.scale.z = 0.18;
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = 0.95F;
    if (!valid || !latest_input_) {
      return marker;
    }

    const double magnitude = norm(direction);
    if (!std::isfinite(magnitude) || magnitude <= direction_deadband) {
      marker.action = Marker::DELETE;
      return marker;
    }
    geometry_msgs::msg::Point start;
    start.x = latest_input_->position[0];
    start.y = latest_input_->position[1];
    start.z = latest_input_->position[2];
    geometry_msgs::msg::Point end = start;
    end.x += visualization_arrow_length_m_ * direction[0] / magnitude;
    end.y += visualization_arrow_length_m_ * direction[1] / magnitude;
    end.z += visualization_arrow_length_m_ * direction[2] / magnitude;
    marker.points = {start, end};
    marker.action = Marker::ADD;
    return marker;
  }

  std::pair<std::array<double, 3>, bool> stickDirection() const
  {
    if (!manual_input_ || !manual_received_at_ || !latest_input_) {
      return {{0.0, 0.0, 0.0}, false};
    }
    const double age = std::chrono::duration<double>(
      SteadyClock::now() - *manual_received_at_).count();
    const auto &manual = *manual_input_;
    if (age > manual_timeout_ || !manual.valid ||
      !std::isfinite(manual.pitch) || !std::isfinite(manual.roll) ||
      !std::isfinite(manual.throttle) || !std::isfinite(latest_input_->yaw)) {
      return {{0.0, 0.0, 0.0}, false};
    }

    // ManualControlSetpoint: pitch is forward, roll is right and throttle is up.
    // Convert body FLU [forward, left, up] to world ENU using measured yaw.
    const double forward = applyDeadband(manual.pitch, manual_deadband_);
    const double left = -applyDeadband(manual.roll, manual_deadband_);
    const double up = applyDeadband(manual.throttle, manual_deadband_);
    const double yaw = latest_input_->yaw;
    const double cosine = std::cos(yaw);
    const double sine = std::sin(yaw);
    return {{
      cosine * forward - sine * left,
      sine * forward + cosine * left,
      up}, true};
  }

  std::array<double, 3> targetVelocity() const
  {
    auto [direction, valid] = stickDirection();
    if (!valid) return {0.0, 0.0, 0.0};
    const double horizontal_norm = std::hypot(direction[0], direction[1]);
    if (horizontal_norm > 1.0) {
      direction[0] /= horizontal_norm;
      direction[1] /= horizontal_norm;
    }
    direction[0] *= planner_config_.max_speed_xy;
    direction[1] *= planner_config_.max_speed_xy;
    direction[2] = std::clamp(direction[2], -1.0, 1.0) * planner_config_.max_speed_z;
    return direction;
  }

  mpc_controller::reference::PlannerState plannerState() const noexcept
  {
    return {manual_position_, manual_velocity_, manual_acceleration_};
  }

  void setPlannerState(const mpc_controller::reference::PlannerState &state) noexcept
  {
    manual_position_ = state.position;
    manual_velocity_ = state.velocity;
    manual_acceleration_ = state.acceleration;
  }

  bool collisionFree(
    const std::vector<mpc_controller::reference::Sample> &) const noexcept
  {
    // Obstacle sensing is intentionally absent in this development stage.
    // This interface becomes the hard collision check without changing MPC.
    return true;
  }

  bool resetManual()
  {
    // Start a manual session from measured p/v/a and captured yaw. Clamping
    // only initializes the reference model; measured controller feedback is untouched.
    if (!latest_input_ || !last_state_received_at_ || !latest_input_->valid ||
      !mpc_controller::reference::finite(latest_input_->position) ||
      !mpc_controller::reference::finite(latest_input_->velocity) ||
      !std::isfinite(latest_input_->yaw)) {
      return false;
    }
    const double state_age = std::chrono::duration<double>(
      SteadyClock::now() - *last_state_received_at_).count();
    if (!std::isfinite(state_age) || state_age > state_timeout_seconds_) return false;

    manual_position_ = latest_input_->position;
    manual_velocity_ = latest_input_->velocity;
    const double horizontal_speed = std::hypot(
      manual_velocity_[0], manual_velocity_[1]);
    if (horizontal_speed > planner_config_.max_speed_xy) {
      const double scale = planner_config_.max_speed_xy / horizontal_speed;
      manual_velocity_[0] *= scale;
      manual_velocity_[1] *= scale;
    }
    manual_velocity_[2] = std::clamp(
      manual_velocity_[2], -planner_config_.max_speed_z, planner_config_.max_speed_z);
    manual_acceleration_ = latest_input_->acceleration;
    const double horizontal_acceleration = std::hypot(
      manual_acceleration_[0], manual_acceleration_[1]);
    if (horizontal_acceleration > planner_config_.max_acceleration_xy) {
      const double scale = planner_config_.max_acceleration_xy / horizontal_acceleration;
      manual_acceleration_[0] *= scale;
      manual_acceleration_[1] *= scale;
    }
    manual_acceleration_[2] = std::clamp(
      manual_acceleration_[2], -planner_config_.max_acceleration_z,
      planner_config_.max_acceleration_z);
    manual_target_velocity_ = manual_velocity_;
    selected_candidate_id_ = -1;
    manual_yaw_ = wrapAngle(latest_input_->yaw);
    parameters_.hold_yaw_rad = manual_yaw_;
    manual_updated_at_ = SteadyClock::now();
    manual_ready_ = true;
    return true;
  }

  void limitManualLead()
  {
    if (!latest_input_ || !mpc_controller::reference::finite(latest_input_->position)) return;
    double error_x = manual_position_[0] - latest_input_->position[0];
    double error_y = manual_position_[1] - latest_input_->position[1];
    const double error_xy = std::hypot(error_x, error_y);
    if (error_xy > manual_lead_xy_) {
      const double scale = manual_lead_xy_ / error_xy;
      error_x *= scale;
      error_y *= scale;
      manual_position_[0] = latest_input_->position[0] + error_x;
      manual_position_[1] = latest_input_->position[1] + error_y;
    }
    manual_position_[2] = latest_input_->position[2] + std::clamp(
      manual_position_[2] - latest_input_->position[2],
      -manual_lead_z_, manual_lead_z_);
  }

  void updateReferenceDirection(const Reference &message)
  {
    reference_direction_valid_ = false;
    reference_direction_enu_ = {0.0, 0.0, 0.0};
    for (const auto &point : message.points) {
      const std::array<double, 3> direction{
        point.velocity[0], point.velocity[1], point.velocity[2]};
      if (norm(direction) > visualization_direction_deadband_) {
        reference_direction_enu_ = direction;
        reference_direction_valid_ = true;
        break;
      }
    }
    if (!reference_direction_valid_ && latest_input_ && !message.points.empty()) {
      const auto &target = message.points.front().position;
      const std::array<double, 3> displacement{
        target[0] - latest_input_->position[0],
        target[1] - latest_input_->position[1],
        target[2] - latest_input_->position[2]};
      if (norm(displacement) > visualization_direction_deadband_) {
        reference_direction_enu_ = displacement;
        reference_direction_valid_ = true;
      }
    }
  }

  void publishManual()
  {
    // Re-plan the full rolling horizon, publish only the selected candidate,
    // and retain it for the next-cycle branch memory and RViz overlay.
    if (!manual_ready_) return;
    const uint64_t sample_ns = static_cast<uint64_t>(
      std::llround(planner_config_.sample_seconds * 1.0e9));
    if (sample_ns == 0) return;

    const auto intent = targetVelocity();
    const auto plan = mpc_controller::reference::selectPlan(
      planner_config_, plannerState(), intent, selected_candidate_id_,
      [this](const std::vector<mpc_controller::reference::Sample> &samples) {
        return collisionFree(samples);
      });
    if (!plan.valid || plan.selected.samples.empty()) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Predictive joystick planner found no dynamically feasible candidate");
      return;
    }

    Reference message;
    message.header.stamp = get_clock()->now();
    message.header.frame_id = frame_id_;
    message.trajectory_id = trajectory_id_++;
    message.hold_after_end = true;
    message.points.reserve(plan.selected.samples.size());
    const double yaw_rate = targetYawRate();
    for (std::size_t index = 0; index < plan.selected.samples.size(); ++index) {
      const auto &sample = plan.selected.samples[index];
      Point point;
      point.time_from_start = durationMessage(index * sample_ns);
      point.position = sample.position;
      point.velocity = sample.velocity;
      point.acceleration = sample.acceleration;
      point.yaw = wrapAngle(
        manual_yaw_ + yaw_rate * planner_config_.sample_seconds * static_cast<double>(index));
      point.yaw_rate = yaw_rate;
      message.points.push_back(point);
    }
    selected_candidate_id_ = plan.selected.id;
    selected_candidate_cost_ = plan.selected.cost;
    feasible_candidate_count_ = plan.feasible_count;
    manual_target_velocity_ = plan.selected.target_velocity;
    last_reference_ = message;
    updateReferenceDirection(message);
    publisher_->publish(message);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Planner candidate=%d feasible=%d cost=%.3f intent=[%.2f %.2f %.2f] "
      "selected=[%.2f %.2f %.2f]",
      selected_candidate_id_, feasible_candidate_count_, selected_candidate_cost_,
      intent[0], intent[1], intent[2], manual_target_velocity_[0],
      manual_target_velocity_[1], manual_target_velocity_[2]);
  }

  void manualUpdate()
  {
    // Receding-horizon policy: execute one short step of the previously chosen
    // primitive, correct excessive reference lead, then optimize again.
    if (!manualVelocityMode() || !offboard_active_) return;
    if (!manual_ready_ && !resetManual()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Manual reference waiting for a fresh valid vehicle state");
      return;
    }

    const auto current_time = SteadyClock::now();
    double dt = 1.0 / manual_rate_hz_;
    if (manual_updated_at_) {
      dt = std::chrono::duration<double>(current_time - *manual_updated_at_).count();
    }
    if (!std::isfinite(dt) || dt <= 0.0) return;
    dt = std::min(dt, 2.0 / manual_rate_hz_);
    const double yaw_rate = targetYawRate();
    setPlannerState(mpc_controller::reference::advance(
      planner_config_, plannerState(), manual_target_velocity_, dt));
    limitManualLead();
    manual_yaw_ = wrapAngle(manual_yaw_ + yaw_rate * dt);
    manual_updated_at_ = current_time;
    parameters_.hold_position = manual_position_;
    parameters_.hold_yaw_rad = manual_yaw_;
    publishManual();
  }

  Marker predictedPathMarker() const
  {
    Marker marker;
    marker.header.stamp = get_clock()->now();
    marker.header.frame_id = frame_id_;
    marker.ns = "mpc_predicted_path";
    marker.id = 2;
    marker.type = Marker::LINE_STRIP;
    marker.action = Marker::DELETE;
    marker.scale.x = 0.045;
    marker.color.g = 0.85F;
    marker.color.b = 1.0F;
    marker.color.a = 0.95F;
    if (!latest_mpc_output_ || !latest_mpc_output_->valid ||
      latest_mpc_output_->predicted_states.size() < 9) {
      return marker;
    }

    // MpcTranslationalOutput layout: [step][axis][position, velocity, acceleration].
    constexpr std::size_t values_per_step = 9;
    const auto available = latest_mpc_output_->predicted_states.size() / values_per_step;
    const auto count = std::min<std::size_t>(
      available, static_cast<std::size_t>(visualization_max_prediction_points_));
    marker.points.reserve(count + 1);
    if (latest_input_) {
      geometry_msgs::msg::Point measured;
      measured.x = latest_input_->position[0];
      measured.y = latest_input_->position[1];
      measured.z = latest_input_->position[2];
      marker.points.push_back(measured);
    }
    for (std::size_t step = 0; step < count; ++step) {
      const std::size_t offset = step * values_per_step;
      geometry_msgs::msg::Point point;
      point.x = latest_mpc_output_->predicted_states[offset];
      point.y = latest_mpc_output_->predicted_states[offset + 3];
      point.z = latest_mpc_output_->predicted_states[offset + 6];
      if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
        marker.points.push_back(point);
      }
    }
    if (marker.points.size() >= 2) {
      marker.action = Marker::ADD;
    }
    return marker;
  }

  Marker selectedPathMarker() const
  {
    Marker marker;
    marker.header.stamp = get_clock()->now();
    marker.header.frame_id = frame_id_;
    marker.ns = "selected_reference_path";
    marker.id = 3;
    marker.type = Marker::LINE_STRIP;
    marker.action = Marker::DELETE;
    marker.scale.x = 0.055;
    marker.color.r = 1.0F;
    marker.color.g = 0.1F;
    marker.color.b = 0.05F;
    marker.color.a = 0.95F;
    if (!last_reference_ || last_reference_->points.size() < 2U) return marker;
    marker.points.reserve(last_reference_->points.size());
    for (const auto &sample : last_reference_->points) {
      geometry_msgs::msg::Point point;
      point.x = sample.position[0];
      point.y = sample.position[1];
      point.z = sample.position[2];
      if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
        marker.points.push_back(point);
      }
    }
    if (marker.points.size() >= 2U) marker.action = Marker::ADD;
    return marker;
  }

  void publishVisualization()
  {
    // Marker contract: user intent=green, selected reference=red, MPC=cyan.
    if (!visualization_publisher_) return;
    const auto current_time = SteadyClock::now();
    if (last_visualization_published_at_) {
      const double age = std::chrono::duration<double>(
        current_time - *last_visualization_published_at_).count();
      if (age < 1.0 / visualization_publish_rate_hz_) return;
    }
    last_visualization_published_at_ = current_time;

    const auto [manual_direction, manual_valid] = stickDirection();
    MarkerArray markers;
    markers.markers.push_back(directionMarker(
      0, "user_direction", manual_direction, manual_valid, 1.0e-6, 0.0F, 1.0F, 0.0F));
    markers.markers.push_back(directionMarker(
      1, "reference_direction", reference_direction_enu_, reference_direction_valid_,
      visualization_direction_deadband_, 1.0F, 0.0F, 0.0F));
    markers.markers.push_back(predictedPathMarker());
    markers.markers.push_back(selectedPathMarker());
    visualization_publisher_->publish(markers);
  }

  void publish()
  {
    // Autonomous hold/line/circle publisher. Manual ACTIVE publication is
    // owned by manualUpdate() at the higher joystick update rate.
    // The high-rate manual timer owns ReferenceTrajectory while Offboard is
    // active. Keep this low-rate timer only for prestream hold publication.
    if (manualVelocityMode() && offboard_active_) return;
    if (auto_capture_current_hold_ && !hold_reference_captured_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Hold reference is waiting for a fresh vehicle state");
      return;
    }
    const uint64_t sample_ns = static_cast<uint64_t>(std::llround(sample_period_seconds_ * 1.0e9));
    const uint64_t horizon_ns = static_cast<uint64_t>(std::llround(horizon_seconds_ * 1.0e9));
    if (sample_ns == 0) {
      return;
    }

    const auto steady_now = SteadyClock::now();
    if (parameters_.type == "line" && line_started_at_
      && std::chrono::duration<double>(steady_now - *line_started_at_).count()
      >= parameters_.line_duration_seconds) {
      parameters_.hold_position = parameters_.line_end;
      parameters_.type = "hold";
      line_started_at_.reset();
      RCLCPP_INFO(get_logger(), "Line completed; endpoint promoted to hold");
    } else if (parameters_.type == "circle" && circle_started_at_
      && std::chrono::duration<double>(steady_now - *circle_started_at_).count()
      >= parameters_.circle_period_seconds) {
      // A complete lap returns exactly to its captured start position.
      parameters_.type = "hold";
      circle_started_at_.reset();
      RCLCPP_INFO(get_logger(), "Circle completed; start point promoted to hold");
    }

    Reference message;
    message.header.stamp = get_clock()->now();
    message.header.frame_id = frame_id_;
    message.trajectory_id = trajectory_id_++;
    message.hold_after_end = true;
    const uint64_t count = horizon_ns / sample_ns + 1;
    message.points.reserve(static_cast<std::size_t>(count));
    double trajectory_elapsed_seconds = 0.0;
    if (parameters_.type == "line" && line_started_at_) {
      trajectory_elapsed_seconds = std::max(
        0.0, std::chrono::duration<double>(steady_now - *line_started_at_).count());
    } else if (parameters_.type == "circle" && circle_started_at_) {
      trajectory_elapsed_seconds = std::max(
        0.0, std::chrono::duration<double>(steady_now - *circle_started_at_).count());
    }
    for (uint64_t index = 0; index < count; ++index) {
      const double time = trajectory_elapsed_seconds +
        static_cast<double>(index * sample_ns) * 1.0e-9;
      const bool circle_complete = parameters_.type == "circle" &&
        time >= parameters_.circle_period_seconds;
      const double model_time = circle_complete ? parameters_.circle_period_seconds : time;
      mpc_controller::reference::Sample sample;
      if (!mpc_controller::reference::sample(parameters_, model_time, sample)) {
        RCLCPP_ERROR(get_logger(), "Reference sample failed at t=%.6f", time);
        return;
      }
      if (parameters_.type == "line" || parameters_.type == "circle") {
        sample.yaw = parameters_.hold_yaw_rad;
        sample.yaw_rate = 0.0;
      }
      if (circle_complete) {
        sample.velocity = {0.0, 0.0, 0.0};
        sample.acceleration = {0.0, 0.0, 0.0};
      }
      Point point;
      point.time_from_start = durationMessage(index * sample_ns);
      point.position = sample.position;
      point.velocity = sample.velocity;
      point.acceleration = sample.acceleration;
      point.yaw = sample.yaw;
      point.yaw_rate = sample.yaw_rate;
      message.points.push_back(point);
    }
    last_reference_ = message;
    updateReferenceDirection(message);
    publisher_->publish(message);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "ReferenceTrajectory published id=%lu points=%lu hold_capture=%s position=[%.3f %.3f %.3f] yaw=%.3f",
      static_cast<unsigned long>(message.trajectory_id),
      static_cast<unsigned long>(message.points.size()),
      hold_reference_captured_ ? "yes" : "no",
      parameters_.hold_position[0], parameters_.hold_position[1],
      parameters_.hold_position[2], parameters_.hold_yaw_rad);
  }

  mpc_controller::reference::Parameters parameters_{};
  std::string reference_input_mode_ = "trajectory";
  std::array<double, 3> line_relative_delta_{2.0, 0.0, 0.0};
  std::string frame_id_ = "map";
  double horizon_seconds_ = 30.0;
  double sample_period_seconds_ = 0.1;
  double publish_rate_hz_ = 1.0;
  bool visualization_enabled_ = true;
  double visualization_publish_rate_hz_ = 20.0;
  double visualization_arrow_length_m_ = 1.0;
  double visualization_direction_deadband_ = 0.08;
  double manual_timeout_ = 0.25;
  double manual_rate_hz_ = 25.0;
  double manual_deadband_ = 0.08;
  double manual_yaw_rate_max_ = 0.8;
  double manual_lead_xy_ = 2.0;
  double manual_lead_z_ = 1.0;
  int visualization_max_prediction_points_ = 50;
  double state_timeout_seconds_ = 0.25;
  double circle_reference_speed_limit_m_s_ = 4.0;
  double circle_acceleration_limit_m_s2_ = 1.5;
  double circle_cruise_speed_m_s_ = 0.0;
  bool auto_capture_current_hold_ = false;
  bool offboard_active_ = false;
  bool hold_reference_captured_ = false;
  std::optional<StateInput> latest_input_;
  std::optional<std::uint64_t> last_state_timestamp_;
  std::optional<SteadyClock::time_point> last_state_received_at_;
  std::optional<SteadyClock::time_point> line_started_at_;
  std::optional<SteadyClock::time_point> circle_started_at_;
  std::optional<ManualControl> manual_input_;
  std::optional<SteadyClock::time_point> manual_received_at_;
  std::optional<MpcOutput> latest_mpc_output_;
  std::optional<SteadyClock::time_point> mpc_output_received_at_;
  std::optional<SteadyClock::time_point> last_visualization_published_at_;
  std::optional<SteadyClock::time_point> manual_updated_at_;
  std::array<double, 3> manual_position_{0.0, 0.0, 0.0};
  std::array<double, 3> manual_velocity_{0.0, 0.0, 0.0};
  std::array<double, 3> manual_acceleration_{0.0, 0.0, 0.0};
  std::array<double, 3> manual_target_velocity_{0.0, 0.0, 0.0};
  mpc_controller::reference::PlannerConfig planner_config_{};
  std::optional<Reference> last_reference_;
  int selected_candidate_id_ = -1;
  int feasible_candidate_count_ = 0;
  double selected_candidate_cost_ = 0.0;
  double manual_yaw_ = 0.0;
  bool manual_ready_ = false;
  std::array<double, 3> reference_direction_enu_{0.0, 0.0, 0.0};
  bool reference_direction_valid_ = false;
  bool valid_config_ = false;
  uint64_t trajectory_id_ = 1;
  rclcpp::Publisher<Reference>::SharedPtr publisher_;
  rclcpp::Subscription<ReferenceStep>::SharedPtr step_subscription_;
  rclcpp::Service<Trigger>::SharedPtr start_line_service_;
  rclcpp::Service<Trigger>::SharedPtr start_circle_service_;
  rclcpp::Subscription<State>::SharedPtr state_subscription_;
  rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr control_mode_subscription_;
  rclcpp::Subscription<ManualControl>::SharedPtr manual_control_subscription_;
  rclcpp::Subscription<MpcOutput>::SharedPtr mpc_output_subscription_;
  rclcpp::Publisher<MarkerArray>::SharedPtr visualization_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr manual_timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ReferenceGeneratorNode>());
  rclcpp::shutdown();
  return 0;
}
