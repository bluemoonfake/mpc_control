#include "mpc_controller/msg/direct_acceleration_command.hpp"

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/timesync_status.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <mutex>
#include <optional>
#include <utility>

class Px4AttitudeSetpointNode final : public rclcpp::Node
{
public:
  Px4AttitudeSetpointNode()
  : Node("px4_attitude_setpoint_node")
  {
    command_subscription_ = create_subscription<Command>(
        "direct_acceleration_command", 10,
        [this](Command::SharedPtr message) {
          std::lock_guard<std::mutex> lock(mutex_);
          command_ = std::move(*message);
          command_received_at_ = std::chrono::steady_clock::now();
        });
    timesync_subscription_ = create_subscription<Timesync>(
        "fmu/out/timesync_status",
        rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
        [this](Timesync::SharedPtr message) {
          if (message && message->source_protocol == Timesync::SOURCE_PROTOCOL_DDS
              && message->timestamp != 0U) {
            std::lock_guard<std::mutex> lock(mutex_);
            timesync_timestamp_ = message->timestamp;
            timesync_received_at_ = std::chrono::steady_clock::now();
          }
        });
    attitude_publisher_ = create_publisher<Attitude>(
        "fmu/in/vehicle_attitude_setpoint",
        rclcpp::QoS(rclcpp::KeepLast(10)).best_effort());
    heartbeat_publisher_ = create_publisher<Offboard>(
        "fmu/in/offboard_control_mode",
        rclcpp::QoS(rclcpp::KeepLast(10)).best_effort());
    setpoint_timer_ = create_wall_timer(std::chrono::milliseconds(20),
        std::bind(&Px4AttitudeSetpointNode::publishSetpoint, this));
    heartbeat_timer_ = create_wall_timer(std::chrono::milliseconds(100),
        std::bind(&Px4AttitudeSetpointNode::publishHeartbeat, this));
  }

private:
  using Command = mpc_controller::msg::DirectAccelerationCommand;
  using Timesync = px4_msgs::msg::TimesyncStatus;
  using Attitude = px4_msgs::msg::VehicleAttitudeSetpoint;
  using Offboard = px4_msgs::msg::OffboardControlMode;

  std::optional<uint64_t> timestamp()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!timesync_timestamp_ || !timesync_received_at_) {
      return std::nullopt;
    }
    const auto now = std::chrono::steady_clock::now();
    const double age = std::chrono::duration<double>(now - *timesync_received_at_).count();
    if (!std::isfinite(age) || age < 0.0 || age > 0.5) {
      return std::nullopt;
    }
    return *timesync_timestamp_ + static_cast<uint64_t>(age * 1.0e6);
  }

  bool fresh(Command& command)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!command_ || !command_received_at_) {
      return false;
    }
    const double age = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - *command_received_at_).count();
    if (!std::isfinite(age) || age < 0.0 || age > 0.25 || !command_->valid) {
      return false;
    }
    command = *command_;
    return true;
  }

  bool map(const Command& command, uint64_t px4_timestamp, Attitude& output)
  {
    constexpr double gravity = 9.80665;
    constexpr double max_thrust_acceleration = 19.6133;
    const double force_enu[3] = {
      command.acceleration[0], command.acceleration[1], command.acceleration[2] + gravity};
    const double force_ned[3] = {force_enu[1], force_enu[0], -force_enu[2]};
    const double norm = std::sqrt(force_ned[0] * force_ned[0]
        + force_ned[1] * force_ned[1] + force_ned[2] * force_ned[2]);
    if (!std::isfinite(norm) || norm < 1.0e-6) {
      return false;
    }
    const double b3[3] = {-force_ned[0] / norm, -force_ned[1] / norm, -force_ned[2] / norm};
    const double yaw_ned = 1.5707963267948966 - command.yaw;
    const double heading[3] = {std::cos(yaw_ned), std::sin(yaw_ned), 0.0};
    double b2[3] = {
      b3[1] * heading[2] - b3[2] * heading[1],
      b3[2] * heading[0] - b3[0] * heading[2],
      b3[0] * heading[1] - b3[1] * heading[0]};
    const double b2_norm = std::sqrt(b2[0] * b2[0] + b2[1] * b2[1] + b2[2] * b2[2]);
    if (!std::isfinite(b2_norm) || b2_norm < 1.0e-9) {
      return false;
    }
    for (double& value : b2) {
      value /= b2_norm;
    }
    const double b1[3] = {
      b2[1] * b3[2] - b2[2] * b3[1],
      b2[2] * b3[0] - b2[0] * b3[2],
      b2[0] * b3[1] - b2[1] * b3[0]};
    const double trace = b1[0] + b2[1] + b3[2];
    double qw, qx, qy, qz;
    if (trace > 0.0) {
      const double scale = 0.5 / std::sqrt(trace + 1.0);
      qw = 0.25 / scale; qx = (b2[2] - b3[1]) * scale;
      qy = (b3[0] - b1[2]) * scale; qz = (b1[1] - b2[0]) * scale;
    } else {
      // The remaining branches keep the mapper finite around 180-degree rotations.
      const double scale = 2.0 * std::sqrt(std::max(1.0e-12, 1.0 + b3[2] - b1[0] - b2[1]));
      qw = (b1[1] - b2[0]) / scale; qx = (b3[0] + b1[2]) / scale;
      qy = (b3[1] + b2[2]) / scale; qz = 0.25 * scale;
    }
    const double qnorm = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
    if (!std::isfinite(qnorm) || qnorm < 1.0e-9) {
      return false;
    }
    output.timestamp = px4_timestamp;
    output.q_d = {static_cast<float>(qw / qnorm), static_cast<float>(qx / qnorm),
      static_cast<float>(qy / qnorm), static_cast<float>(qz / qnorm)};
    output.thrust_body = {0.0F, 0.0F, static_cast<float>(-
      std::clamp(norm / max_thrust_acceleration, 0.0, 1.0))};
    output.yaw_sp_move_rate = static_cast<float>(-command.yaw_rate);
    return true;
  }

  void publishSetpoint()
  {
    Command command;
    if (!fresh(command)) {
      return;
    }
    const auto stamp = timestamp();
    if (!stamp) {
      return;
    }
    Attitude message;
    if (map(command, *stamp, message)) {
      attitude_publisher_->publish(message);
    }
  }

  void publishHeartbeat()
  {
    Command command;
    if (!fresh(command)) {
      return;
    }
    const auto stamp = timestamp();
    if (!stamp) {
      return;
    }
    Offboard message;
    message.timestamp = *stamp;
    message.attitude = true;
    heartbeat_publisher_->publish(message);
  }

  std::mutex mutex_;
  std::optional<Command> command_;
  std::optional<std::chrono::steady_clock::time_point> command_received_at_;
  std::optional<uint64_t> timesync_timestamp_;
  std::optional<std::chrono::steady_clock::time_point> timesync_received_at_;
  rclcpp::Subscription<Command>::SharedPtr command_subscription_;
  rclcpp::Subscription<Timesync>::SharedPtr timesync_subscription_;
  rclcpp::Publisher<Attitude>::SharedPtr attitude_publisher_;
  rclcpp::Publisher<Offboard>::SharedPtr heartbeat_publisher_;
  rclcpp::TimerBase::SharedPtr setpoint_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Px4AttitudeSetpointNode>());
  rclcpp::shutdown();
  return 0;
}
