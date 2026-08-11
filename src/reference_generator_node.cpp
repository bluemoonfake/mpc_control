#include "mpc_controller/msg/reference_trajectory.hpp"
#include "mpc_controller/msg/reference_step.hpp"
#include "mpc_controller/msg/trajectory_point.hpp"
#include "mpc_controller/msg/vehicle_state.hpp"
#include "mpc_controller/msg/handover_status.hpp"
#include "mpc_controller/msg/handover_status_delivery_diagnostics.hpp"
#include "mpc_controller/hold_capture.hpp"
#include "mpc_controller/reference_model.hpp"
#include "mpc_controller/timing_diagnostics.hpp"

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <algorithm>
#include <array>
#include <chrono>
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
    declare_parameter("trajectory_type", parameters_.type);
    declare_parameter("frame_id", frame_id_);
    declare_parameter("hold_position", std::vector<double>{0.0, 0.0, 1.0});
    declare_parameter("line_start", std::vector<double>{0.0, 0.0, 1.0});
    declare_parameter("line_end", std::vector<double>{1.0, 0.0, 1.0});
    declare_parameter("line_duration_seconds", parameters_.line_duration_seconds);
    declare_parameter("circle_center", std::vector<double>{0.0, 0.0, 1.0});
    declare_parameter("circle_radius", parameters_.circle_radius);
    declare_parameter("circle_period_seconds", parameters_.circle_period_seconds);
    declare_parameter("circle_phase_rad", parameters_.circle_phase_rad);
    declare_parameter("circle_direction", parameters_.circle_direction);
    declare_parameter("hold_yaw_rad", parameters_.hold_yaw_rad);
    declare_parameter("hold_current_state_on_enable", false);
    declare_parameter("auto_capture_current_hold", false);
    declare_parameter("state_timeout_seconds", state_timeout_seconds_);
    declare_parameter("status_publish_rate_hz", status_publish_rate_hz_);
    declare_parameter("stable_hover_max_velocity_xy_m_s", thresholds_.max_velocity_xy_m_s);
    declare_parameter("stable_hover_max_velocity_z_m_s", thresholds_.max_velocity_z_m_s);
    declare_parameter(
      "stable_hover_max_acceleration_xy_m_s2", thresholds_.max_acceleration_xy_m_s2);
    declare_parameter(
      "stable_hover_max_acceleration_z_m_s2", thresholds_.max_acceleration_z_m_s2);
    declare_parameter("stable_hover_max_body_rate_xy_rad_s", thresholds_.max_body_rate_xy_rad_s);
    declare_parameter("stable_hover_max_body_rate_z_rad_s", thresholds_.max_body_rate_z_rad_s);
    declare_parameter("stable_hover_max_roll_rad", thresholds_.max_roll_rad);
    declare_parameter("stable_hover_max_pitch_rad", thresholds_.max_pitch_rad);
    declare_parameter("stable_hover_dwell_seconds", thresholds_.dwell_seconds);
    declare_parameter(
      "stable_hover_exit_max_velocity_xy_m_s", thresholds_.exit_max_velocity_xy_m_s);
    declare_parameter(
      "stable_hover_exit_max_velocity_z_m_s", thresholds_.exit_max_velocity_z_m_s);
    declare_parameter(
      "stable_hover_exit_max_acceleration_xy_m_s2", thresholds_.exit_max_acceleration_xy_m_s2);
    declare_parameter(
      "stable_hover_exit_max_acceleration_z_m_s2", thresholds_.exit_max_acceleration_z_m_s2);
    declare_parameter(
      "stable_hover_exit_max_body_rate_xy_rad_s", thresholds_.exit_max_body_rate_xy_rad_s);
    declare_parameter(
      "stable_hover_exit_max_body_rate_z_rad_s", thresholds_.exit_max_body_rate_z_rad_s);
    declare_parameter("stable_hover_exit_max_roll_rad", thresholds_.exit_max_roll_rad);
    declare_parameter("stable_hover_exit_max_pitch_rad", thresholds_.exit_max_pitch_rad);
    declare_parameter("stable_hover_exit_dwell_seconds", thresholds_.exit_dwell_seconds);
    declare_parameter("horizon_seconds", horizon_seconds_);
    declare_parameter("sample_period_seconds", sample_period_seconds_);
    declare_parameter("publish_rate_hz", publish_rate_hz_);

    get_parameter("trajectory_type", parameters_.type);
    get_parameter("frame_id", frame_id_);
    valid_config_ = true;
    getVectorParameter("hold_position", parameters_.hold_position);
    getVectorParameter("line_start", parameters_.line_start);
    getVectorParameter("line_end", parameters_.line_end);
    get_parameter("line_duration_seconds", parameters_.line_duration_seconds);
    getVectorParameter("circle_center", parameters_.circle_center);
    get_parameter("circle_radius", parameters_.circle_radius);
    get_parameter("circle_period_seconds", parameters_.circle_period_seconds);
    get_parameter("circle_phase_rad", parameters_.circle_phase_rad);
    get_parameter("circle_direction", parameters_.circle_direction);
    get_parameter("hold_yaw_rad", parameters_.hold_yaw_rad);
    get_parameter("hold_current_state_on_enable", hold_current_state_on_enable_);
    get_parameter("auto_capture_current_hold", auto_capture_current_hold_);
    get_parameter("state_timeout_seconds", state_timeout_seconds_);
    get_parameter("status_publish_rate_hz", status_publish_rate_hz_);
    get_parameter("stable_hover_max_velocity_xy_m_s", thresholds_.max_velocity_xy_m_s);
    get_parameter("stable_hover_max_velocity_z_m_s", thresholds_.max_velocity_z_m_s);
    get_parameter("stable_hover_max_acceleration_xy_m_s2", thresholds_.max_acceleration_xy_m_s2);
    get_parameter("stable_hover_max_acceleration_z_m_s2", thresholds_.max_acceleration_z_m_s2);
    get_parameter("stable_hover_max_body_rate_xy_rad_s", thresholds_.max_body_rate_xy_rad_s);
    get_parameter("stable_hover_max_body_rate_z_rad_s", thresholds_.max_body_rate_z_rad_s);
    get_parameter("stable_hover_max_roll_rad", thresholds_.max_roll_rad);
    get_parameter("stable_hover_max_pitch_rad", thresholds_.max_pitch_rad);
    get_parameter("stable_hover_dwell_seconds", thresholds_.dwell_seconds);
    get_parameter(
      "stable_hover_exit_max_velocity_xy_m_s", thresholds_.exit_max_velocity_xy_m_s);
    get_parameter("stable_hover_exit_max_velocity_z_m_s", thresholds_.exit_max_velocity_z_m_s);
    get_parameter(
      "stable_hover_exit_max_acceleration_xy_m_s2", thresholds_.exit_max_acceleration_xy_m_s2);
    get_parameter(
      "stable_hover_exit_max_acceleration_z_m_s2", thresholds_.exit_max_acceleration_z_m_s2);
    get_parameter(
      "stable_hover_exit_max_body_rate_xy_rad_s", thresholds_.exit_max_body_rate_xy_rad_s);
    get_parameter(
      "stable_hover_exit_max_body_rate_z_rad_s", thresholds_.exit_max_body_rate_z_rad_s);
    get_parameter("stable_hover_exit_max_roll_rad", thresholds_.exit_max_roll_rad);
    get_parameter("stable_hover_exit_max_pitch_rad", thresholds_.exit_max_pitch_rad);
    get_parameter("stable_hover_exit_dwell_seconds", thresholds_.exit_dwell_seconds);
    get_parameter("horizon_seconds", horizon_seconds_);
    get_parameter("sample_period_seconds", sample_period_seconds_);
    get_parameter("publish_rate_hz", publish_rate_hz_);

    valid_config_ = valid_config_ && mpc_controller::reference::valid(parameters_)
      && std::isfinite(horizon_seconds_) && horizon_seconds_ >= 0.0
      && std::isfinite(sample_period_seconds_) && sample_period_seconds_ > 0.0
      && std::isfinite(publish_rate_hz_) && publish_rate_hz_ > 0.0
      && std::isfinite(state_timeout_seconds_) && state_timeout_seconds_ > 0.0
      && std::isfinite(status_publish_rate_hz_) && status_publish_rate_hz_ > 0.0
      && mpc_controller::hold::validThresholds(thresholds_)
      && !frame_id_.empty();
    hold_capture_ = mpc_controller::hold::CaptureOnce(thresholds_);
    handover_stability_ = mpc_controller::hold::HysteresisGate(thresholds_);
    status_timer_tracker_.setPeriodSeconds(1.0 / std::max(status_publish_rate_hz_, 1.0));
    if (!valid_config_) {
      RCLCPP_ERROR(get_logger(), "Invalid reference generator parameters; publishing disabled");
    } else {
      publisher_ = create_publisher<Reference>("reference_trajectory", 10);
      step_subscription_ = create_subscription<ReferenceStep>(
        "reference_step", 10,
        std::bind(&ReferenceGeneratorNode::referenceStepCallback, this, std::placeholders::_1));
      status_publisher_ = create_publisher<HandoverStatus>("hold_capture_status", 10);
      status_delivery_diagnostics_publisher_ = create_publisher<DeliveryDiagnostics>(
        "handover_status_source_diagnostics", 10);
      capture_service_ = create_service<Trigger>(
        "~/capture_hold",
        std::bind(
          &ReferenceGeneratorNode::captureHold, this, std::placeholders::_1,
          std::placeholders::_2));
      if (hold_current_state_on_enable_) {
        state_subscription_ = create_subscription<State>(
          "vehicle_state", rclcpp::QoS(10),
          std::bind(&ReferenceGeneratorNode::stateCallback, this, std::placeholders::_1));
        RCLCPP_INFO(
          get_logger(), auto_capture_current_hold_
          ? "SITL current-state hold tracking enabled; freezes on PX4 Offboard entry"
          : "Explicit hold capture enabled; waiting for operator service ~/capture_hold");
      }
      if (auto_capture_current_hold_) {
        const auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
        control_mode_subscription_ = create_subscription<px4_msgs::msg::VehicleControlMode>(
          "fmu/out/vehicle_control_mode", sensor_qos,
          [this](px4_msgs::msg::VehicleControlMode::SharedPtr message) {
            if (!message) return;
            const bool was_offboard = offboard_active_;
            offboard_active_ = message->flag_control_offboard_enabled;
            if (!was_offboard && offboard_active_ && hold_reference_captured_) {
              publish();
              RCLCPP_INFO(
                get_logger(),
                "SITL hold frozen on Offboard entry: position=[%.3f %.3f %.3f] yaw=%.4f",
                parameters_.hold_position[0], parameters_.hold_position[1],
                parameters_.hold_position[2], parameters_.hold_yaw_rad);
            }
          });
      }
      timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(1.0 / publish_rate_hz_)),
        std::bind(&ReferenceGeneratorNode::publish, this));
      status_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(1.0 / status_publish_rate_hz_)),
        std::bind(&ReferenceGeneratorNode::publishStatus, this));
    }
  }

private:
  using Reference = mpc_controller::msg::ReferenceTrajectory;
  using ReferenceStep = mpc_controller::msg::ReferenceStep;
  using Point = mpc_controller::msg::TrajectoryPoint;
  using State = mpc_controller::msg::VehicleState;
  using HandoverStatus = mpc_controller::msg::HandoverStatus;
  using DeliveryDiagnostics = mpc_controller::msg::HandoverStatusDeliveryDiagnostics;
  using Trigger = std_srvs::srv::Trigger;
  using SteadyClock = std::chrono::steady_clock;

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

    mpc_controller::hold::Input input;
    input.position = {message->position[0], message->position[1], message->position[2]};
    input.velocity = {message->velocity[0], message->velocity[1], message->velocity[2]};
    input.acceleration = {
      message->acceleration[0], message->acceleration[1], message->acceleration[2]};
    input.attitude_wxyz = {
      message->attitude[0], message->attitude[1], message->attitude[2], message->attitude[3]};
    input.body_rate = {message->body_rate[0], message->body_rate[1], message->body_rate[2]};
    input.yaw = message->yaw;
    input.timestamp = static_cast<std::uint64_t>(timestamp);
    input.valid = message->valid && message->position_valid && message->velocity_valid &&
      message->acceleration_valid;
    input.fresh = true;
    input.control_ready = message->control_ready;
    input.heading_valid = message->heading_valid;
    input.yaw_valid = message->yaw_valid;
    input.attitude_valid = message->attitude_valid;
    input.body_rate_valid = message->body_rate_valid;
    latest_input_ = input;
    last_state_timestamp_ = input.timestamp;
    last_state_received_at_ = SteadyClock::now();
    refreshGate();
    if (auto_capture_current_hold_ && !offboard_active_
      && std::isfinite(input.position[0]) && std::isfinite(input.position[1])
      && std::isfinite(input.position[2]) && std::isfinite(input.yaw)) {
      const bool first_capture = !hold_reference_captured_;
      parameters_.type = "hold";
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

  static double steadySeconds(const SteadyClock::time_point &time) noexcept
  {
    return std::chrono::duration<double>(time.time_since_epoch()).count();
  }

  void referenceStepCallback(const ReferenceStep::SharedPtr message)
  {
    if (!message || !message->valid || !valid_config_) {
      RCLCPP_WARN(get_logger(), "Relative reference step rejected: invalid command");
      return;
    }
    if (!hold_reference_captured_ || parameters_.type != "hold") {
      RCLCPP_WARN(
        get_logger(),
        "Relative reference step rejected: a captured hold reference is not active");
      return;
    }
    if (!std::all_of(
        message->delta_position.begin(), message->delta_position.end(),
        [](double value) {return std::isfinite(value);})) {
      RCLCPP_WARN(get_logger(), "Relative reference step rejected: non-finite delta");
      return;
    }

    const auto previous_position = parameters_.hold_position;
    for (std::size_t axis = 0; axis < parameters_.hold_position.size(); ++axis) {
      parameters_.hold_position[axis] += message->delta_position[axis];
    }
    publish();
    RCLCPP_INFO(
      get_logger(),
      "Relative ENU hold step applied: delta=[%.3f %.3f %.3f], "
      "position=[%.3f %.3f %.3f] -> [%.3f %.3f %.3f], yaw held at %.4f",
      message->delta_position[0], message->delta_position[1], message->delta_position[2],
      previous_position[0], previous_position[1], previous_position[2],
      parameters_.hold_position[0], parameters_.hold_position[1],
      parameters_.hold_position[2], parameters_.hold_yaw_rad);
  }

  void refreshGate()
  {
    if (!latest_input_) {
      return;
    }
    auto input = *latest_input_;
    input.fresh = last_state_received_at_ &&
      std::chrono::duration<double>(SteadyClock::now() - *last_state_received_at_).count()
      <= state_timeout_seconds_;
    latest_input_ = input;
    const double now = steadySeconds(SteadyClock::now());
    hold_capture_.update(now, input);
    handover_stability_.update(now, input);
  }

  void captureHold(
    const std::shared_ptr<Trigger::Request>, const std::shared_ptr<Trigger::Response> response)
  {
    refreshGate();
    if (!hold_current_state_on_enable_) {
      response->success = false;
      response->message = "hold capture mode is disabled";
      return;
    }
    const double now = steadySeconds(SteadyClock::now());
    if (!hold_capture_.requestCapture(now)) {
      response->success = false;
      response->message = std::string("capture rejected in state ") +
        mpc_controller::hold::stateName(hold_capture_.state());
      RCLCPP_WARN(get_logger(), "Explicit hold capture rejected: %s", response->message.c_str());
      return;
    }
    const auto &snapshot = hold_capture_.snapshot();
    parameters_.type = "hold";
    parameters_.hold_position = snapshot.position;
    parameters_.hold_yaw_rad = snapshot.yaw;
    hold_reference_captured_ = true;
    response->success = true;
    response->message = "hold reference captured exactly once";
    RCLCPP_INFO(
      get_logger(),
      "Captured current-state hold reference exactly once: p0=[%.3f %.3f %.3f] "
      "v0=[%.3f %.3f %.3f] a0=[%.3f %.3f %.3f] q0=[%.5f %.5f %.5f %.5f] "
      "omega0=[%.4f %.4f %.4f] yaw0=%.4f state_timestamp=%lu "
      "metrics=[vxy %.4f vz %.4f axy %.4f az %.4f wxy %.4f wz %.4f roll %.5f pitch %.5f]",
      snapshot.position[0], snapshot.position[1], snapshot.position[2],
      snapshot.velocity[0], snapshot.velocity[1], snapshot.velocity[2],
      snapshot.acceleration[0], snapshot.acceleration[1], snapshot.acceleration[2],
      snapshot.attitude_wxyz[0], snapshot.attitude_wxyz[1], snapshot.attitude_wxyz[2],
      snapshot.attitude_wxyz[3], snapshot.body_rate[0], snapshot.body_rate[1],
      snapshot.body_rate[2], snapshot.yaw, static_cast<unsigned long>(snapshot.timestamp),
      snapshot.stability.velocity_xy_m_s, snapshot.stability.velocity_z_m_s,
      snapshot.stability.acceleration_xy_m_s2, snapshot.stability.acceleration_z_m_s2,
      snapshot.stability.body_rate_xy_rad_s, snapshot.stability.body_rate_z_rad_s,
      snapshot.stability.roll_rad, snapshot.stability.pitch_rad);
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

  void publish()
  {
    if (hold_current_state_on_enable_ && !hold_reference_captured_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Hold reference is not captured; call ~/capture_hold after stable-hover dwell");
      return;
    }
    const uint64_t sample_ns = static_cast<uint64_t>(std::llround(sample_period_seconds_ * 1.0e9));
    const uint64_t horizon_ns = static_cast<uint64_t>(std::llround(horizon_seconds_ * 1.0e9));
    if (sample_ns == 0) {
      return;
    }

    Reference message;
    message.header.stamp = get_clock()->now();
    message.header.frame_id = frame_id_;
    message.trajectory_id = trajectory_id_++;
    message.hold_after_end = true;
    const uint64_t count = horizon_ns / sample_ns + 1;
    message.points.reserve(static_cast<std::size_t>(count));
    for (uint64_t index = 0; index < count; ++index) {
      const double time = static_cast<double>(index * sample_ns) * 1.0e-9;
      mpc_controller::reference::Sample sample;
      if (!mpc_controller::reference::sample(parameters_, time, sample)) {
        RCLCPP_ERROR(get_logger(), "Reference sample failed at t=%.6f", time);
        return;
      }
      Point point;
      point.time_from_start = durationMessage(index * sample_ns);
      point.position = sample.position;
      point.velocity = sample.velocity;
      point.acceleration = sample.acceleration;
      point.yaw = sample.yaw;
      point.yaw_rate = sample.yaw_rate;
      point.yaw_valid = true;
      message.points.push_back(point);
    }
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

  void publishStatus()
  {
    if (!status_publisher_) {
      return;
    }
    mpc_controller::timing::TimerGuard timer_guard(
      status_timer_tracker_, mpc_controller::timing::steadyNowNs());
    const auto timer_sample = timer_guard.sample();
    refreshGate();
    HandoverStatus message{};
    message.header.stamp = get_clock()->now();
    message.header.frame_id = frame_id_;
    message.publisher_steady_timestamp_ns = mpc_controller::timing::steadyNowNs();
    message.timer_sequence = timer_sample.sequence;
    message.timer_expected_fire_steady_timestamp_ns =
      timer_sample.expected_fire_steady_timestamp_ns;
    message.timer_actual_start_steady_timestamp_ns = timer_sample.actual_start_steady_timestamp_ns;
    message.timer_callback_end_steady_timestamp_ns = mpc_controller::timing::steadyNowNs();
    const auto state = hold_capture_.state();
    message.state = static_cast<std::uint8_t>(state);
    if (latest_input_) {
      const auto &input = *latest_input_;
      message.vehicle_state_valid = input.valid;
      message.vehicle_state_fresh = input.fresh;
      message.heading_valid = input.heading_valid;
      message.yaw_valid = input.yaw_valid;
    message.control_ready = input.control_ready;
    }
    message.capture_ready = hold_current_state_on_enable_ &&
      hold_capture_.currentReady() && hold_capture_.currentStable() &&
      hold_capture_.dwellComplete(steadySeconds(SteadyClock::now())) &&
      !hold_capture_.captured();
    message.stable_hover = hold_capture_.currentStable();
    message.handover_stability_valid = handover_stability_.stable();
    message.dwell_complete = hold_capture_.dwellComplete(steadySeconds(SteadyClock::now()));
    message.capture_valid = hold_capture_.captured();
    message.capture_timestamp = 0U;
    message.stable_duration_seconds = hold_capture_.dwellSeconds(steadySeconds(SteadyClock::now()));
    if (const auto &metrics = hold_capture_.latestMetrics()) {
      message.stability_metrics = {
        metrics->velocity_xy_m_s, metrics->velocity_z_m_s,
        metrics->acceleration_xy_m_s2, metrics->acceleration_z_m_s2,
        metrics->body_rate_xy_rad_s, metrics->body_rate_z_rad_s,
        metrics->roll_rad, metrics->pitch_rad};
    }
    message.stability_limits = {
      thresholds_.max_velocity_xy_m_s, thresholds_.max_velocity_z_m_s,
      thresholds_.max_acceleration_xy_m_s2, thresholds_.max_acceleration_z_m_s2,
      thresholds_.max_body_rate_xy_rad_s, thresholds_.max_body_rate_z_rad_s,
      thresholds_.max_roll_rad, thresholds_.max_pitch_rad};
    if (hold_capture_.captured()) {
      message.capture_timestamp = hold_capture_.snapshot().timestamp;
    }
    status_publisher_->publish(message);
    const auto source_callback_end_ns = mpc_controller::timing::steadyNowNs();
    if (status_delivery_diagnostics_publisher_) {
      DeliveryDiagnostics diagnostics{};
      diagnostics.header = message.header;
      diagnostics.role = DeliveryDiagnostics::PRODUCER;
      diagnostics.diagnostic_sequence = message.timer_sequence;
      diagnostics.source_publish_sequence = message.timer_sequence;
      diagnostics.source_publish_steady_timestamp_ns = message.publisher_steady_timestamp_ns;
      diagnostics.source_publish_ros_timestamp_ns =
        static_cast<std::uint64_t>(message.header.stamp.sec) * 1000000000ULL +
        message.header.stamp.nanosec;
      diagnostics.source_callback_start_steady_timestamp_ns =
        message.timer_actual_start_steady_timestamp_ns;
      diagnostics.source_callback_end_steady_timestamp_ns =
        source_callback_end_ns;
      status_delivery_diagnostics_publisher_->publish(diagnostics);
    }
  }

  mpc_controller::reference::Parameters parameters_{};
  std::string frame_id_ = "map";
  double horizon_seconds_ = 30.0;
  double sample_period_seconds_ = 0.1;
  double publish_rate_hz_ = 1.0;
  double status_publish_rate_hz_ = 50.0;
  double state_timeout_seconds_ = 0.25;
  bool hold_current_state_on_enable_ = false;
  bool auto_capture_current_hold_ = false;
  bool offboard_active_ = false;
  bool hold_reference_captured_ = false;
  mpc_controller::hold::Thresholds thresholds_{};
  mpc_controller::hold::CaptureOnce hold_capture_{thresholds_};
  mpc_controller::hold::HysteresisGate handover_stability_{thresholds_};
  std::optional<mpc_controller::hold::Input> latest_input_;
  std::optional<std::uint64_t> last_state_timestamp_;
  std::optional<SteadyClock::time_point> last_state_received_at_;
  bool valid_config_ = false;
  uint64_t trajectory_id_ = 1;
  rclcpp::Publisher<Reference>::SharedPtr publisher_;
  rclcpp::Subscription<ReferenceStep>::SharedPtr step_subscription_;
  rclcpp::Publisher<HandoverStatus>::SharedPtr status_publisher_;
  rclcpp::Publisher<DeliveryDiagnostics>::SharedPtr status_delivery_diagnostics_publisher_;
  rclcpp::Service<Trigger>::SharedPtr capture_service_;
  rclcpp::Subscription<State>::SharedPtr state_subscription_;
  rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr control_mode_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  mpc_controller::timing::TimerTracker status_timer_tracker_{};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ReferenceGeneratorNode>());
  rclcpp::shutdown();
  return 0;
}
