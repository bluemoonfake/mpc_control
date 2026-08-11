#include "mpc_controller/msg/attitude_setpoint_preview.hpp"
#include "mpc_controller/msg/m3_control_output.hpp"
#include "mpc_controller/msg/vehicle_state.hpp"
#include "mpc_controller/px4_attitude_setpoint_contract.hpp"
#include "mpc_controller/timing_diagnostics.hpp"

#include <px4_msgs/msg/hover_thrust_estimate.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/timesync_status.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
#include <optional>

class Px4AttitudeSetpointNode final : public rclcpp::Node
{
public:
  Px4AttitudeSetpointNode()
  : Node("px4_attitude_setpoint_node")
  {
    declare_parameter("publish_rate_hz", publish_rate_hz_);
    declare_parameter("heartbeat_rate_hz", heartbeat_rate_hz_);
    declare_parameter("state_timeout_seconds", state_timeout_seconds_);
    declare_parameter("command_timeout_seconds", command_timeout_seconds_);
    declare_parameter("timesync_timeout_seconds", timesync_timeout_seconds_);
    declare_parameter(
      "hover_thrust_estimate_timeout_seconds", hover_thrust_estimate_timeout_seconds_);
    declare_parameter("motor_count", motor_mapping_.motor_count);
    declare_parameter(
      "motor_thrust_coefficient_n_per_rad_s2",
      motor_mapping_.thrust_coefficient_n_per_rad_s2);
    declare_parameter("motor_max_speed_rad_s", motor_mapping_.maximum_motor_speed_rad_s);
    declare_parameter("vehicle_mass", thrust_mapping_.vehicle_mass_kg);
    declare_parameter("gravity", thrust_mapping_.gravity_mps2);
    declare_parameter("hover_thrust_normalized", thrust_mapping_.hover_thrust_normalized);

    get_parameter("publish_rate_hz", publish_rate_hz_);
    get_parameter("heartbeat_rate_hz", heartbeat_rate_hz_);
    get_parameter("state_timeout_seconds", state_timeout_seconds_);
    get_parameter("command_timeout_seconds", command_timeout_seconds_);
    get_parameter("timesync_timeout_seconds", timesync_timeout_seconds_);
    get_parameter(
      "hover_thrust_estimate_timeout_seconds", hover_thrust_estimate_timeout_seconds_);
    get_parameter("motor_count", motor_mapping_.motor_count);
    get_parameter(
      "motor_thrust_coefficient_n_per_rad_s2",
      motor_mapping_.thrust_coefficient_n_per_rad_s2);
    get_parameter("motor_max_speed_rad_s", motor_mapping_.maximum_motor_speed_rad_s);
    get_parameter("vehicle_mass", thrust_mapping_.vehicle_mass_kg);
    get_parameter("gravity", thrust_mapping_.gravity_mps2);
    get_parameter("hover_thrust_normalized", thrust_mapping_.hover_thrust_normalized);

    config_valid_ = std::isfinite(publish_rate_hz_) && publish_rate_hz_ > 0.0
      && std::isfinite(heartbeat_rate_hz_) && heartbeat_rate_hz_ > 0.0
      && std::isfinite(state_timeout_seconds_) && state_timeout_seconds_ > 0.0
      && std::isfinite(command_timeout_seconds_) && command_timeout_seconds_ > 0.0
      && std::isfinite(timesync_timeout_seconds_) && timesync_timeout_seconds_ > 0.0
      && std::isfinite(hover_thrust_estimate_timeout_seconds_)
      && hover_thrust_estimate_timeout_seconds_ > 0.0
      && mpc_controller::px4_attitude::validMotorModelThrustMapping(motor_mapping_)
      && mpc_controller::px4_thrust::valid(thrust_mapping_);
    if (!config_valid_) {
      RCLCPP_ERROR(get_logger(), "Invalid SITL attitude adapter configuration");
    }

    const auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    m3_subscription_ = create_subscription<M3Output>(
      "m3_control_output", 10,
      [this](M3Output::SharedPtr message) {
        if (!message) return;
        std::lock_guard<std::mutex> lock(mutex_);
        m3_output_ = *message;
        m3_received_at_ = Clock::now();
        m3_received_steady_timestamp_ns_ = mpc_controller::timing::steadyNowNs();
      });
    vehicle_state_subscription_ = create_subscription<VehicleState>(
      "vehicle_state", 10,
      [this](VehicleState::SharedPtr message) {
        if (!message) return;
        std::lock_guard<std::mutex> lock(mutex_);
        vehicle_state_ = *message;
        vehicle_state_received_at_ = Clock::now();
      });
    timesync_subscription_ = create_subscription<Timesync>(
      "fmu/out/timesync_status", sensor_qos,
      [this](Timesync::SharedPtr message) {
        if (!message || message->source_protocol != Timesync::SOURCE_PROTOCOL_DDS
          || message->timestamp == 0U) {
          return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        timesync_timestamp_ = message->timestamp;
        timesync_received_at_ = Clock::now();
      });
    hover_thrust_estimate_subscription_ = create_subscription<HoverThrustEstimate>(
      "fmu/out/hover_thrust_estimate", sensor_qos,
      [this](HoverThrustEstimate::SharedPtr message) {
        if (!message) return;
        std::lock_guard<std::mutex> lock(mutex_);
        hover_thrust_estimate_ = *message;
        hover_thrust_estimate_received_at_ = Clock::now();
      });
    control_mode_subscription_ = create_subscription<ControlMode>(
      "fmu/out/vehicle_control_mode", sensor_qos,
      [this](ControlMode::SharedPtr message) {
        if (!message) return;
        std::lock_guard<std::mutex> lock(mutex_);
        const bool was_offboard = offboard_active_;
        offboard_active_ = message->flag_control_offboard_enabled;
        if (!was_offboard && offboard_active_) {
          captureYawLocked();
          latchHoverThrustLocked();
          state_ = State::active;
          RCLCPP_INFO(
            get_logger(),
            "PX4 Offboard entered; yaw_hold_enu=%.6f rad; hover_thrust=%.6f (%s)",
            yaw_hold_enu_rad_.value_or(std::numeric_limits<double>::quiet_NaN()),
            latched_hover_thrust_normalized_.value_or(thrust_mapping_.hover_thrust_normalized),
            latched_hover_thrust_from_estimator_ ? "PX4 HTE" : "configured fallback");
        } else if (was_offboard && !offboard_active_) {
          yaw_hold_enu_rad_.reset();
          latched_hover_thrust_normalized_.reset();
          latched_hover_thrust_from_estimator_ = false;
          state_ = State::offboard_lost;
          RCLCPP_WARN(get_logger(), "PX4 Offboard lost; attitude publication stopped");
        }
      });

    attitude_publisher_ = create_publisher<Attitude>(
      // VehicleAttitudeSetpoint is MESSAGE_VERSION=1 in pinned PX4 v1.17.
      "fmu/in/vehicle_attitude_setpoint_v1", sensor_qos);
    preview_publisher_ = create_publisher<Preview>("px4_attitude_setpoint_preview", 10);
    heartbeat_publisher_ = create_publisher<Offboard>(
      "fmu/in/offboard_control_mode", sensor_qos);

    setpoint_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / publish_rate_hz_)),
      [this]() {publishAttitudeSetpoint();});
    heartbeat_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / heartbeat_rate_hz_)),
      [this]() {publishHeartbeat();});

    const auto maximum_thrust =
      mpc_controller::px4_attitude::maximumCollectiveThrustN(motor_mapping_);
    RCLCPP_INFO(
      get_logger(),
      "SITL-first attitude adapter ready: WAIT_DATA -> PRESTREAM -> ACTIVE; "
      "motor-model Fmax=%.6f N; no arm or mode command is sent",
      maximum_thrust.value_or(std::numeric_limits<double>::quiet_NaN()));
  }

private:
  using M3Output = mpc_controller::msg::M3ControlOutput;
  using VehicleState = mpc_controller::msg::VehicleState;
  using Preview = mpc_controller::msg::AttitudeSetpointPreview;
  using Timesync = px4_msgs::msg::TimesyncStatus;
  using HoverThrustEstimate = px4_msgs::msg::HoverThrustEstimate;
  using Attitude = px4_msgs::msg::VehicleAttitudeSetpoint;
  using Offboard = px4_msgs::msg::OffboardControlMode;
  using ControlMode = px4_msgs::msg::VehicleControlMode;
  using Clock = std::chrono::steady_clock;

  enum class State : uint8_t
  {
    wait_data = Preview::STATE_WAIT_DATA,
    prestream = Preview::STATE_PRESTREAM,
    active = Preview::STATE_ACTIVE,
    offboard_lost = Preview::STATE_OFFBOARD_LOST,
  };

  struct Snapshot
  {
    M3Output m3{};
    VehicleState vehicle{};
    uint64_t px4_timestamp = 0U;
    double state_age_seconds = std::numeric_limits<double>::quiet_NaN();
    double m3_age_seconds = std::numeric_limits<double>::quiet_NaN();
    double timesync_age_seconds = std::numeric_limits<double>::quiet_NaN();
    double yaw_command_enu_rad = std::numeric_limits<double>::quiet_NaN();
    double hover_thrust_estimate = std::numeric_limits<double>::quiet_NaN();
    double hover_thrust_estimate_variance = std::numeric_limits<double>::quiet_NaN();
    double hover_thrust_estimate_age_seconds = std::numeric_limits<double>::quiet_NaN();
    double active_hover_thrust_normalized = std::numeric_limits<double>::quiet_NaN();
    bool state_fresh = false;
    bool m3_fresh = false;
    bool timesync_fresh = false;
    bool offboard_active = false;
    bool yaw_hold_valid = false;
    bool hover_thrust_estimate_valid = false;
    bool hover_thrust_estimate_fresh = false;
    bool active_hover_thrust_from_estimator = false;

    bool ready() const noexcept
    {
      return state_fresh && m3_fresh && timesync_fresh;
    }
  };

  static bool freshAge(double age_seconds, double timeout_seconds) noexcept
  {
    return std::isfinite(age_seconds) && age_seconds >= 0.0
      && age_seconds <= timeout_seconds;
  }

  void captureYawLocked()
  {
    if (vehicle_state_ && std::isfinite(vehicle_state_->yaw)) {
      yaw_hold_enu_rad_ = vehicle_state_->yaw;
    } else {
      yaw_hold_enu_rad_.reset();
    }
  }

  bool hoverThrustEstimateUsableLocked(Clock::time_point current_time) const noexcept
  {
    if (!hover_thrust_estimate_ || !hover_thrust_estimate_received_at_) return false;
    const double age = std::chrono::duration<double>(
      current_time - *hover_thrust_estimate_received_at_).count();
    return freshAge(age, hover_thrust_estimate_timeout_seconds_)
      && hover_thrust_estimate_->valid
      && std::isfinite(hover_thrust_estimate_->hover_thrust)
      && hover_thrust_estimate_->hover_thrust >= 0.1F
      && hover_thrust_estimate_->hover_thrust <= 0.9F
      && std::isfinite(hover_thrust_estimate_->hover_thrust_var)
      && hover_thrust_estimate_->hover_thrust_var >= 0.0F;
  }

  void latchHoverThrustLocked()
  {
    if (hoverThrustEstimateUsableLocked(Clock::now())) {
      latched_hover_thrust_normalized_ = hover_thrust_estimate_->hover_thrust;
      latched_hover_thrust_from_estimator_ = true;
    } else {
      latched_hover_thrust_normalized_ = thrust_mapping_.hover_thrust_normalized;
      latched_hover_thrust_from_estimator_ = false;
    }
  }

  Snapshot snapshot()
  {
    Snapshot output;
    const auto current_time = Clock::now();
    std::lock_guard<std::mutex> lock(mutex_);
    if (m3_output_ && m3_received_at_) {
      output.m3 = *m3_output_;
      output.m3_age_seconds = std::chrono::duration<double>(
        current_time - *m3_received_at_).count();
      output.m3_fresh = freshAge(output.m3_age_seconds, command_timeout_seconds_);
    }
    if (vehicle_state_ && vehicle_state_received_at_) {
      output.vehicle = *vehicle_state_;
      output.state_age_seconds = std::chrono::duration<double>(
        current_time - *vehicle_state_received_at_).count();
      output.state_fresh = freshAge(output.state_age_seconds, state_timeout_seconds_);
    }
    if (timesync_timestamp_ && timesync_received_at_) {
      output.timesync_age_seconds = std::chrono::duration<double>(
        current_time - *timesync_received_at_).count();
      output.timesync_fresh = freshAge(output.timesync_age_seconds, timesync_timeout_seconds_);
      if (output.timesync_fresh) {
        output.px4_timestamp = *timesync_timestamp_
          + static_cast<uint64_t>(output.timesync_age_seconds * 1.0e6);
      }
    }
    if (hover_thrust_estimate_ && hover_thrust_estimate_received_at_) {
      output.hover_thrust_estimate = hover_thrust_estimate_->hover_thrust;
      output.hover_thrust_estimate_variance = hover_thrust_estimate_->hover_thrust_var;
      output.hover_thrust_estimate_age_seconds = std::chrono::duration<double>(
        current_time - *hover_thrust_estimate_received_at_).count();
      output.hover_thrust_estimate_fresh = freshAge(
        output.hover_thrust_estimate_age_seconds, hover_thrust_estimate_timeout_seconds_);
      output.hover_thrust_estimate_valid = hoverThrustEstimateUsableLocked(current_time);
    }
    output.offboard_active = offboard_active_;
    output.yaw_hold_valid = yaw_hold_enu_rad_.has_value();
    output.yaw_command_enu_rad = yaw_hold_enu_rad_.value_or(output.vehicle.yaw);
    if (offboard_active_ && latched_hover_thrust_normalized_) {
      output.active_hover_thrust_normalized = *latched_hover_thrust_normalized_;
      output.active_hover_thrust_from_estimator = latched_hover_thrust_from_estimator_;
    } else if (output.hover_thrust_estimate_valid) {
      output.active_hover_thrust_normalized = output.hover_thrust_estimate;
      output.active_hover_thrust_from_estimator = true;
    } else {
      output.active_hover_thrust_normalized = thrust_mapping_.hover_thrust_normalized;
      output.active_hover_thrust_from_estimator = false;
    }
    return output;
  }

  bool updateState(const Snapshot &sample)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ == State::offboard_lost) {
      state_ = State::wait_data;
      return false;
    }
    if (!config_valid_ || !sample.ready()) {
      state_ = State::wait_data;
      return false;
    }
    if (sample.offboard_active) {
      if (!yaw_hold_enu_rad_) captureYawLocked();
      if (!latched_hover_thrust_normalized_) latchHoverThrustLocked();
      state_ = State::active;
    } else {
      yaw_hold_enu_rad_.reset();
      state_ = State::prestream;
    }
    return true;
  }

  State stateValue() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
  }

  void publishHeartbeat()
  {
    const auto sample = snapshot();
    if (!updateState(sample)) return;

    Offboard heartbeat{};
    heartbeat.timestamp = sample.px4_timestamp;
    heartbeat.attitude = true;
    heartbeat_publisher_->publish(heartbeat);
  }

  void publishAttitudeSetpoint()
  {
    auto sample = snapshot();
    if (!updateState(sample)) {
      publishPreview(sample, {});
      return;
    }

    // updateState may latch yaw after the snapshot was taken.
    sample = snapshot();
    mpc_controller::px4_attitude::Output converted;
    const Eigen::Quaterniond m3_attitude(
      sample.m3.desired_attitude_wxyz[0], sample.m3.desired_attitude_wxyz[1],
      sample.m3.desired_attitude_wxyz[2], sample.m3.desired_attitude_wxyz[3]);
    const auto held_attitude = mpc_controller::px4_attitude::withEnuYaw(
      m3_attitude, sample.yaw_command_enu_rad);
    if (held_attitude) {
      mpc_controller::px4_attitude::Input input;
      input.desired_body_flu_to_world_enu = *held_attitude;
      input.desired_collective_thrust_n = sample.m3.desired_thrust_force_n;
      input.desired_yaw_rate_enu_rad_s = 0.0;
      input.valid = true;
      // VehicleAttitudeSetpoint.thrust_body is a normalized PX4 control
      // coordinate, not physical thrust divided by the SDF motor ceiling.
      auto active_thrust_mapping = thrust_mapping_;
      active_thrust_mapping.hover_thrust_normalized = sample.active_hover_thrust_normalized;
      converted = mpc_controller::px4_attitude::convert(input, active_thrust_mapping);
    }

    publishPreview(sample, converted);
    if (!converted.valid) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "SITL setpoint computation invalid; state remains freshness-driven (failure=%u)",
        static_cast<unsigned>(converted.failure_reason));
      return;
    }

    Attitude message{};
    message.timestamp = sample.px4_timestamp;
    message.q_d = converted.q_d_wxyz;
    message.thrust_body = converted.thrust_body_frd;
    message.yaw_sp_move_rate = 0.0F;
    attitude_publisher_->publish(message);
  }

  void publishPreview(
    const Snapshot &sample, const mpc_controller::px4_attitude::Output &converted)
  {
    Preview preview{};
    preview.header.stamp = now();
    preview.px4_timestamp = sample.px4_timestamp;
    preview.q_d_wxyz = converted.q_d_wxyz;
    preview.thrust_body_frd = converted.thrust_body_frd;
    preview.yaw_sp_move_rate_ned_rad_s = 0.0F;
    preview.valid = converted.valid;
    preview.prestream_enabled = stateValue() == State::prestream;
    preview.px4_offboard_active = sample.offboard_active;
    preview.state = static_cast<uint8_t>(stateValue());
    preview.state_age_seconds = sample.state_age_seconds;
    preview.m3_age_seconds = sample.m3_age_seconds;
    preview.timesync_age_seconds = sample.timesync_age_seconds;
    preview.state_fresh = sample.state_fresh;
    preview.m3_fresh = sample.m3_fresh;
    preview.timesync_fresh = sample.timesync_fresh;
    preview.yaw_hold_valid = sample.yaw_hold_valid;
    preview.yaw_hold_enu_rad = sample.yaw_command_enu_rad;
    preview.m3_message_valid = sample.m3.valid;
    preview.m3_math_valid = sample.m3.math_valid;
    preview.vehicle_state_valid = sample.vehicle.valid;
    preview.failure_reason = static_cast<uint8_t>(converted.failure_reason);
    preview.m3_sequence = sample.m3.sequence;
    preview.m3_publisher_steady_timestamp_ns = sample.m3.publisher_steady_timestamp_ns;
    preview.m3_adapter_received_steady_timestamp_ns = m3_received_steady_timestamp_ns_;
    preview.m3_delivery_delay_seconds = preview.m3_publisher_steady_timestamp_ns > 0U
      && preview.m3_adapter_received_steady_timestamp_ns >=
      preview.m3_publisher_steady_timestamp_ns
      ? static_cast<double>(preview.m3_adapter_received_steady_timestamp_ns
        - preview.m3_publisher_steady_timestamp_ns) * 1.0e-9
      : std::numeric_limits<double>::quiet_NaN();
    const auto maximum = mpc_controller::px4_attitude::maximumCollectiveThrustN(motor_mapping_);
    preview.motor_model_max_collective_thrust_n = maximum.value_or(
      std::numeric_limits<double>::quiet_NaN());
    preview.hover_thrust_estimate = static_cast<float>(sample.hover_thrust_estimate);
    preview.hover_thrust_estimate_variance = static_cast<float>(
      sample.hover_thrust_estimate_variance);
    preview.hover_thrust_estimate_age_seconds = sample.hover_thrust_estimate_age_seconds;
    preview.hover_thrust_estimate_valid = sample.hover_thrust_estimate_valid;
    preview.hover_thrust_estimate_fresh = sample.hover_thrust_estimate_fresh;
    preview.active_hover_thrust_normalized = sample.active_hover_thrust_normalized;
    preview.active_hover_thrust_from_estimator = sample.active_hover_thrust_from_estimator;
    preview_publisher_->publish(preview);
  }

  mutable std::mutex mutex_;
  bool config_valid_ = false;
  bool offboard_active_ = false;
  State state_ = State::wait_data;
  double publish_rate_hz_ = 50.0;
  double heartbeat_rate_hz_ = 10.0;
  double state_timeout_seconds_ = 0.25;
  double command_timeout_seconds_ = 0.25;
  double timesync_timeout_seconds_ = 1.5;
  double hover_thrust_estimate_timeout_seconds_ = 1.0;
  mpc_controller::px4_attitude::MotorModelThrustMapping motor_mapping_{};
  mpc_controller::px4_thrust::Mapping thrust_mapping_{};
  std::optional<double> yaw_hold_enu_rad_;
  std::optional<double> latched_hover_thrust_normalized_;
  bool latched_hover_thrust_from_estimator_ = false;
  std::optional<M3Output> m3_output_;
  std::optional<VehicleState> vehicle_state_;
  std::optional<HoverThrustEstimate> hover_thrust_estimate_;
  std::optional<Clock::time_point> m3_received_at_;
  std::optional<Clock::time_point> vehicle_state_received_at_;
  std::optional<Clock::time_point> hover_thrust_estimate_received_at_;
  std::optional<uint64_t> timesync_timestamp_;
  std::optional<Clock::time_point> timesync_received_at_;
  uint64_t m3_received_steady_timestamp_ns_ = 0U;

  rclcpp::Subscription<M3Output>::SharedPtr m3_subscription_;
  rclcpp::Subscription<VehicleState>::SharedPtr vehicle_state_subscription_;
  rclcpp::Subscription<Timesync>::SharedPtr timesync_subscription_;
  rclcpp::Subscription<HoverThrustEstimate>::SharedPtr hover_thrust_estimate_subscription_;
  rclcpp::Subscription<ControlMode>::SharedPtr control_mode_subscription_;
  rclcpp::Publisher<Attitude>::SharedPtr attitude_publisher_;
  rclcpp::Publisher<Preview>::SharedPtr preview_publisher_;
  rclcpp::Publisher<Offboard>::SharedPtr heartbeat_publisher_;
  rclcpp::TimerBase::SharedPtr setpoint_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Px4AttitudeSetpointNode>());
  rclcpp::shutdown();
  return 0;
}
