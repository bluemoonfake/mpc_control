#include "mpc_control_px4/px4_trajectory_setpoint_publisher.hpp"

#include <cmath>
#include <functional>

namespace mpc_control_px4
{

Px4TrajectorySetpointPublisher::Px4TrajectorySetpointPublisher(
    const rclcpp::NodeOptions& options)
: rclcpp::Node("px4_trajectory_setpoint_publisher", options)
{
  declare_parameter("command_topic", command_topic_);
  declare_parameter("timesync_topic", timesync_topic_);
  declare_parameter("estimator_topic", estimator_topic_);
  declare_parameter("trajectory_setpoint_topic", trajectory_setpoint_topic_);
  declare_parameter("offboard_control_mode_topic", offboard_control_mode_topic_);
  declare_parameter("setpoint_rate_hz", setpoint_rate_hz_);
  declare_parameter("heartbeat_rate_hz", heartbeat_rate_hz_);
  declare_parameter("command_timeout_seconds", command_timeout_seconds_);
  declare_parameter("timesync_timeout_seconds", timesync_timeout_seconds_);
  declare_parameter("estimator_timeout_seconds", estimator_timeout_seconds_);
  declare_parameter("input_frame_id", adapter_.configuration().input_frame_id);

  get_parameter("command_topic", command_topic_);
  get_parameter("timesync_topic", timesync_topic_);
  get_parameter("estimator_topic", estimator_topic_);
  get_parameter("trajectory_setpoint_topic", trajectory_setpoint_topic_);
  get_parameter("offboard_control_mode_topic", offboard_control_mode_topic_);
  get_parameter("setpoint_rate_hz", setpoint_rate_hz_);
  get_parameter("heartbeat_rate_hz", heartbeat_rate_hz_);
  get_parameter("command_timeout_seconds", command_timeout_seconds_);
  get_parameter("timesync_timeout_seconds", timesync_timeout_seconds_);
  get_parameter("estimator_timeout_seconds", estimator_timeout_seconds_);
  std::string input_frame_id;
  get_parameter("input_frame_id", input_frame_id);

  if (command_topic_.empty() || timesync_topic_.empty() || estimator_topic_.empty()
      || trajectory_setpoint_topic_.empty()
      || offboard_control_mode_topic_.empty() || input_frame_id.empty()
      || !std::isfinite(setpoint_rate_hz_) || setpoint_rate_hz_ <= 0.0
      || !std::isfinite(heartbeat_rate_hz_) || heartbeat_rate_hz_ <= 0.0
      || !std::isfinite(command_timeout_seconds_)
      || command_timeout_seconds_ <= 0.0
      || !std::isfinite(timesync_timeout_seconds_)
      || timesync_timeout_seconds_ <= 0.0
      || !std::isfinite(estimator_timeout_seconds_)
      || estimator_timeout_seconds_ <= 0.0
      || heartbeat_rate_hz_ > setpoint_rate_hz_) {
    RCLCPP_ERROR(get_logger(), "Invalid PX4 setpoint publisher configuration");
    return;
  }

  adapter_ = Px4TrajectoryAdapter(Px4TrajectoryAdapterConfig{input_frame_id});
  const auto input_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
  const auto px4_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

  command_subscription_ = create_subscription<CommandMessage>(
      command_topic_, input_qos,
      std::bind(&Px4TrajectorySetpointPublisher::commandCallback, this,
                std::placeholders::_1));
  timesync_subscription_ = create_subscription<TimesyncMessage>(
      timesync_topic_, px4_qos,
      std::bind(&Px4TrajectorySetpointPublisher::timesyncCallback, this,
                std::placeholders::_1));
  estimator_subscription_ = create_subscription<LocalPositionMessage>(
      estimator_topic_, px4_qos,
      std::bind(&Px4TrajectorySetpointPublisher::localPositionCallback, this,
                std::placeholders::_1));
  trajectory_setpoint_publisher_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      trajectory_setpoint_topic_, px4_qos);
  offboard_control_mode_publisher_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
      offboard_control_mode_topic_, px4_qos);

  setpoint_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(1.0 / setpoint_rate_hz_)),
      std::bind(&Px4TrajectorySetpointPublisher::publishSetpoint, this));
  heartbeat_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(1.0 / heartbeat_rate_hz_)),
      std::bind(&Px4TrajectorySetpointPublisher::publishHeartbeat, this));
  configuration_valid_ = true;
}

void Px4TrajectorySetpointPublisher::commandCallback(
    const CommandMessage::SharedPtr message)
{
  if (!message) {
    return;
  }
  std::lock_guard<std::mutex> lock(command_mutex_);
  const auto identity = commandIdentity(*message);
  if (estimator_monitor_.resetPending()) {
    if (!reset_blocked_command_ || isNewerCommand(identity, *reset_blocked_command_)) {
      if (estimator_monitor_.acknowledgeReset()) {
        reset_blocked_command_.reset();
      }
    }
  }
  last_command_ = *message;
  last_command_received_at_ = std::chrono::steady_clock::now();
}

void Px4TrajectorySetpointPublisher::timesyncCallback(
    const TimesyncMessage::SharedPtr message)
{
  if (!message) {
    return;
  }
  timestamp_source_.update(*message, std::chrono::steady_clock::now());
}

void Px4TrajectorySetpointPublisher::localPositionCallback(
    const LocalPositionMessage::SharedPtr message)
{
  if (!message) {
    return;
  }

  const auto result = estimator_monitor_.update(
      *message, std::chrono::steady_clock::now());
  if (!result.reset_detected) {
    return;
  }

  std::lock_guard<std::mutex> lock(command_mutex_);
  if (last_command_) {
    reset_blocked_command_ = commandIdentity(*last_command_);
  } else {
    reset_blocked_command_.reset();
  }
}

bool Px4TrajectorySetpointPublisher::commandFresh(
    const std::chrono::steady_clock::time_point now,
    CommandMessage& command) const
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  if (!last_command_ || !last_command_received_at_) {
    return false;
  }
  const double age_seconds = std::chrono::duration<double>(
      now - *last_command_received_at_).count();
  if (!std::isfinite(age_seconds) || age_seconds < 0.0
      || age_seconds > command_timeout_seconds_) {
    return false;
  }
  command = *last_command_;
  return true;
}

bool Px4TrajectorySetpointPublisher::estimatorUsable(
    const std::chrono::steady_clock::time_point now) const
{
  return estimator_monitor_.usable(now, estimator_timeout_seconds_);
}

Px4TrajectorySetpointPublisher::CommandIdentity
Px4TrajectorySetpointPublisher::commandIdentity(const CommandMessage& command) noexcept
{
  return CommandIdentity{command.trajectory_id, command.sequence};
}

bool Px4TrajectorySetpointPublisher::isNewerCommand(
    const CommandIdentity& candidate,
    const CommandIdentity& baseline) noexcept
{
  return candidate.trajectory_id > baseline.trajectory_id
    || (candidate.trajectory_id == baseline.trajectory_id
      && candidate.sequence > baseline.sequence);
}

void Px4TrajectorySetpointPublisher::publishSetpoint()
{
  if (!configuration_valid_) {
    return;
  }
  CommandMessage command;
  const auto now = std::chrono::steady_clock::now();
  if (!commandFresh(now, command) || !estimatorUsable(now)) {
    return;
  }
  const auto timestamp = timestamp_source_.nextTimestamp(
      now, timesync_timeout_seconds_);
  if (!timestamp) {
    return;
  }
  const auto result = adapter_.convert(command, *timestamp);
  if (result.valid) {
    trajectory_setpoint_publisher_->publish(result.setpoint);
  }
}

void Px4TrajectorySetpointPublisher::publishHeartbeat()
{
  if (!configuration_valid_) {
    return;
  }
  CommandMessage command;
  const auto now = std::chrono::steady_clock::now();
  if (!commandFresh(now, command) || !estimatorUsable(now)) {
    return;
  }
  const auto timestamp = timestamp_source_.nextTimestamp(
      now, timesync_timeout_seconds_);
  if (!timestamp) {
    return;
  }

  px4_msgs::msg::OffboardControlMode message;
  message.timestamp = *timestamp;
  // Position is the selected PX4 Offboard control level. Velocity and
  // acceleration remain finite feedforward fields in TrajectorySetpoint.
  message.position = true;
  message.velocity = false;
  message.acceleration = false;
  message.attitude = false;
  message.body_rate = false;
  message.thrust_and_torque = false;
  message.direct_actuator = false;
  offboard_control_mode_publisher_->publish(message);
}

}  // namespace mpc_control_px4
