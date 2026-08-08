#include "mpc_controller/msg/m3_control_output.hpp"
#include "mpc_controller/px4_wrench_contract.hpp"

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/timesync_status.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>
#include <px4_msgs/msg/vehicle_torque_setpoint.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <mutex>
#include <optional>

class Px4WrenchBridgeNode final : public rclcpp::Node
{
public:
  Px4WrenchBridgeNode()
  : Node("px4_wrench_bridge_node")
  {
    declare_parameter("m3_output_topic", m3_output_topic_);
    declare_parameter("timesync_topic", timesync_topic_);
    declare_parameter("offboard_control_mode_topic", offboard_control_mode_topic_);
    declare_parameter("thrust_setpoint_topic", thrust_setpoint_topic_);
    declare_parameter("torque_setpoint_topic", torque_setpoint_topic_);
    declare_parameter("command_timeout_seconds", command_timeout_seconds_);
    declare_parameter("timesync_timeout_seconds", timesync_timeout_seconds_);
    declare_parameter("publish_rate_hz", publish_rate_hz_);
    declare_parameter("heartbeat_rate_hz", heartbeat_rate_hz_);
    declare_parameter("vehicle_mass", thrust_mapping_.vehicle_mass_kg);
    declare_parameter("gravity", thrust_mapping_.gravity_mps2);
    declare_parameter(
      "px4_hover_thrust_normalized", thrust_mapping_.hover_thrust_normalized);
    get_parameter("m3_output_topic", m3_output_topic_);
    get_parameter("timesync_topic", timesync_topic_);
    get_parameter("offboard_control_mode_topic", offboard_control_mode_topic_);
    get_parameter("thrust_setpoint_topic", thrust_setpoint_topic_);
    get_parameter("torque_setpoint_topic", torque_setpoint_topic_);
    get_parameter("command_timeout_seconds", command_timeout_seconds_);
    get_parameter("timesync_timeout_seconds", timesync_timeout_seconds_);
    get_parameter("publish_rate_hz", publish_rate_hz_);
    get_parameter("heartbeat_rate_hz", heartbeat_rate_hz_);
    get_parameter("vehicle_mass", thrust_mapping_.vehicle_mass_kg);
    get_parameter("gravity", thrust_mapping_.gravity_mps2);
    get_parameter(
      "px4_hover_thrust_normalized", thrust_mapping_.hover_thrust_normalized);

    if (!mpc_controller::px4_wrench::validThrustMapping(thrust_mapping_)) {
      RCLCPP_ERROR(get_logger(), "Invalid PX4 thrust mapping parameters; publishing disabled");
      mapping_valid_ = false;
    }

    // Keep the 50 Hz wrench path independent from the lower-rate heartbeat
    // and input callbacks. This does not alter the fail-closed policy; it
    // only prevents a slow callback from serializing both timers.
    input_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    wrench_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    heartbeat_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

    const auto m3_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    // PX4/px4_ros2 uses best-effort depth 1 for FMU topics: keep only the
    // newest command and never accumulate stale setpoints.
    const auto px4_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    rclcpp::SubscriptionOptions input_options;
    input_options.callback_group = input_callback_group_;
    m3_subscription_ = create_subscription<M3Output>(
      m3_output_topic_, m3_qos,
      [this](M3Output::SharedPtr message) {
        if (!message) {
          return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        latest_command_ = *message;
        command_received_at_ = Clock::now();
      }, input_options);
    timesync_subscription_ = create_subscription<Timesync>(
      timesync_topic_, px4_qos,
      [this](Timesync::SharedPtr message) {
        if (!message || message->source_protocol != Timesync::SOURCE_PROTOCOL_DDS
          || message->timestamp == 0U) {
          return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        timesync_timestamp_ = message->timestamp;
        timesync_received_at_ = Clock::now();
      }, input_options);

    offboard_control_mode_publisher_ = create_publisher<OffboardControlMode>(
      offboard_control_mode_topic_, px4_qos);
    thrust_publisher_ = create_publisher<ThrustSetpoint>(thrust_setpoint_topic_, px4_qos);
    torque_publisher_ = create_publisher<TorqueSetpoint>(torque_setpoint_topic_, px4_qos);

    setpoint_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / std::max(publish_rate_hz_, 1.0))),
      std::bind(&Px4WrenchBridgeNode::publishWrench, this), wrench_callback_group_);
    heartbeat_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / std::max(heartbeat_rate_hz_, 1.0))),
      std::bind(&Px4WrenchBridgeNode::publishHeartbeat, this), heartbeat_callback_group_);
  }

private:
  using M3Output = mpc_controller::msg::M3ControlOutput;
  using OffboardControlMode = px4_msgs::msg::OffboardControlMode;
  using ThrustSetpoint = px4_msgs::msg::VehicleThrustSetpoint;
  using TorqueSetpoint = px4_msgs::msg::VehicleTorqueSetpoint;
  using Timesync = px4_msgs::msg::TimesyncStatus;
  using Clock = std::chrono::steady_clock;
  using Conversion = mpc_controller::px4_wrench::ConversionResult;

  struct ReadySample
  {
    M3Output command;
    Conversion conversion;
    uint64_t timestamp = 0;
  };

  struct TimingStats
  {
    std::optional<Clock::time_point> previous_call;
    Clock::time_point last_report = Clock::now();
    uint64_t intervals = 0;
    double sum_period_seconds = 0.0;
    double min_period_seconds = std::numeric_limits<double>::infinity();
    double max_period_seconds = 0.0;
  };

  void recordTimerInvocation(const char *name, TimingStats &stats)
  {
    const auto now = Clock::now();
    bool report = false;
    double mean = 0.0;
    double minimum = 0.0;
    double maximum = 0.0;
    uint64_t intervals = 0;
    {
      std::lock_guard<std::mutex> lock(timing_mutex_);
      if (stats.previous_call) {
        const double period = std::chrono::duration<double>(now - *stats.previous_call).count();
        if (std::isfinite(period) && period >= 0.0) {
          ++stats.intervals;
          stats.sum_period_seconds += period;
          stats.min_period_seconds = std::min(stats.min_period_seconds, period);
          stats.max_period_seconds = std::max(stats.max_period_seconds, period);
        }
      }
      stats.previous_call = now;
      if (std::chrono::duration<double>(now - stats.last_report).count() >= 5.0
        && stats.intervals > 0) {
        stats.last_report = now;
        report = true;
        intervals = stats.intervals;
        mean = stats.sum_period_seconds / static_cast<double>(stats.intervals);
        minimum = stats.min_period_seconds;
        maximum = stats.max_period_seconds;
        stats.intervals = 0;
        stats.sum_period_seconds = 0.0;
        stats.min_period_seconds = std::numeric_limits<double>::infinity();
        stats.max_period_seconds = 0.0;
      }
    }
    if (report) {
      RCLCPP_INFO(
        get_logger(), "M4 %s timer: intervals=%lu mean_period_ms=%.3f min_ms=%.3f max_ms=%.3f",
        name, static_cast<unsigned long>(intervals), mean * 1.0e3,
        minimum * 1.0e3, maximum * 1.0e3);
    }
  }

  std::optional<uint64_t> px4Timestamp()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!timesync_timestamp_ || !timesync_received_at_) {
      return std::nullopt;
    }
    const double age = std::chrono::duration<double>(Clock::now() - *timesync_received_at_).count();
    if (!std::isfinite(age) || age < 0.0 || age > timesync_timeout_seconds_) {
      return std::nullopt;
    }
    return *timesync_timestamp_ + static_cast<uint64_t>(age * 1.0e6);
  }

  std::optional<ReadySample> readySample()
  {
    M3Output command;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!latest_command_ || !command_received_at_) {
        return std::nullopt;
      }
      const double age = std::chrono::duration<double>(Clock::now() - *command_received_at_).count();
      const bool timestamp_valid = latest_command_->header.stamp.sec != 0
        || latest_command_->header.stamp.nanosec != 0;
      if (!mpc_controller::px4_wrench::commandReady(
          latest_command_->valid, latest_command_->math_valid,
          latest_command_->active_control_ready, timestamp_valid,
          age, command_timeout_seconds_)) {
        return std::nullopt;
      }
      command = *latest_command_;
    }

    if (!mapping_valid_) {
      return std::nullopt;
    }

    const auto timestamp = px4Timestamp();
    if (!timestamp) {
      return std::nullopt;
    }
    const Eigen::Vector3d action(
      command.normalized_torque_command_flu[0],
      command.normalized_torque_command_flu[1],
      command.normalized_torque_command_flu[2]);
    const Conversion conversion = mpc_controller::px4_wrench::convert(
      command.desired_thrust_force_n, action, thrust_mapping_);
    if (conversion.status != mpc_controller::px4_wrench::Status::within_range) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "M4 command rejected: thrust/action outside verified PX4 envelope");
      return std::nullopt;
    }
    return ReadySample{command, conversion, *timestamp};
  }

  void publishWrench()
  {
    recordTimerInvocation("wrench", wrench_timing_);
    const auto sample = readySample();
    if (!sample) {
      return;
    }

    ThrustSetpoint thrust;
    thrust.timestamp = sample->timestamp;
    thrust.timestamp_sample = sample->timestamp;
    thrust.xyz = {
      static_cast<float>(sample->conversion.thrust_frd.x()),
      static_cast<float>(sample->conversion.thrust_frd.y()),
      static_cast<float>(sample->conversion.thrust_frd.z())};

    TorqueSetpoint torque;
    torque.timestamp = sample->timestamp;
    torque.timestamp_sample = sample->timestamp;
    torque.xyz = {
      static_cast<float>(sample->conversion.torque_frd.x()),
      static_cast<float>(sample->conversion.torque_frd.y()),
      static_cast<float>(sample->conversion.torque_frd.z())};

    recordTimerInvocation("wrench_publish", wrench_publish_timing_);
    thrust_publisher_->publish(thrust);
    torque_publisher_->publish(torque);
  }

  void publishHeartbeat()
  {
    recordTimerInvocation("heartbeat", heartbeat_timing_);
    const auto sample = readySample();
    if (!sample) {
      return;
    }
    OffboardControlMode heartbeat;
    heartbeat.timestamp = sample->timestamp;
    const auto flags = mpc_controller::px4_wrench::wrenchOffboardFlags();
    heartbeat.position = flags.position;
    heartbeat.velocity = flags.velocity;
    heartbeat.acceleration = flags.acceleration;
    heartbeat.attitude = flags.attitude;
    heartbeat.body_rate = flags.body_rate;
    heartbeat.thrust_and_torque = flags.thrust_and_torque;
    heartbeat.direct_actuator = flags.direct_actuator;
    recordTimerInvocation("heartbeat_publish", heartbeat_publish_timing_);
    offboard_control_mode_publisher_->publish(heartbeat);
  }

  std::mutex mutex_;
  std::mutex timing_mutex_;
  std::optional<M3Output> latest_command_;
  std::optional<Clock::time_point> command_received_at_;
  std::optional<uint64_t> timesync_timestamp_;
  std::optional<Clock::time_point> timesync_received_at_;
  std::string m3_output_topic_ = "m3_control_output";
  std::string timesync_topic_ = "fmu/out/timesync_status";
  std::string offboard_control_mode_topic_ = "fmu/in/offboard_control_mode";
  std::string thrust_setpoint_topic_ = "fmu/in/vehicle_thrust_setpoint";
  std::string torque_setpoint_topic_ = "fmu/in/vehicle_torque_setpoint";
  double command_timeout_seconds_ = 0.25;
  // PX4 v1.17 performs uxr_sync_session() once per second.  The timeout is
  // therefore derived from that source cadence, rather than from the 50 Hz
  // wrench period.  It still fails closed after one missed synchronization
  // interval plus a bounded margin.
  double timesync_timeout_seconds_ = 1.5;
  double publish_rate_hz_ = 50.0;
  double heartbeat_rate_hz_ = 10.0;
  mpc_controller::px4_wrench::ThrustMapping thrust_mapping_{};
  bool mapping_valid_ = true;
  rclcpp::Subscription<M3Output>::SharedPtr m3_subscription_;
  rclcpp::Subscription<Timesync>::SharedPtr timesync_subscription_;
  rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
  rclcpp::Publisher<ThrustSetpoint>::SharedPtr thrust_publisher_;
  rclcpp::Publisher<TorqueSetpoint>::SharedPtr torque_publisher_;
  rclcpp::TimerBase::SharedPtr setpoint_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  rclcpp::CallbackGroup::SharedPtr input_callback_group_;
  rclcpp::CallbackGroup::SharedPtr wrench_callback_group_;
  rclcpp::CallbackGroup::SharedPtr heartbeat_callback_group_;
  TimingStats wrench_timing_;
  TimingStats heartbeat_timing_;
  TimingStats wrench_publish_timing_;
  TimingStats heartbeat_publish_timing_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Px4WrenchBridgeNode>();
  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), 3);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
