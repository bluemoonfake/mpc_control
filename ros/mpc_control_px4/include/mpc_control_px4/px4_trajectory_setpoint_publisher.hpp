#pragma once

#include "mpc_control_px4/px4_timestamp_source.hpp"
#include "mpc_control_px4/px4_estimator_validity_monitor.hpp"
#include "mpc_control_px4/px4_trajectory_adapter.hpp"

#include <mpc_control_msgs/msg/trajectory_command.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/timesync_status.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

namespace mpc_control_px4
{

class Px4TrajectorySetpointPublisher final : public rclcpp::Node
{
public:
  explicit Px4TrajectorySetpointPublisher(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  Px4TrajectorySetpointPublisher(const Px4TrajectorySetpointPublisher&) = delete;
  Px4TrajectorySetpointPublisher& operator=(const Px4TrajectorySetpointPublisher&) = delete;

private:
  using CommandMessage = mpc_control_msgs::msg::TrajectoryCommand;
  using TimesyncMessage = px4_msgs::msg::TimesyncStatus;
  using LocalPositionMessage = px4_msgs::msg::VehicleLocalPosition;

  struct CommandIdentity
  {
    std::uint64_t trajectory_id = 0;
    std::uint64_t sequence = 0;
  };

  void commandCallback(const CommandMessage::SharedPtr message);
  void timesyncCallback(const TimesyncMessage::SharedPtr message);
  void localPositionCallback(const LocalPositionMessage::SharedPtr message);
  void publishSetpoint();
  void publishHeartbeat();

  bool commandFresh(
      std::chrono::steady_clock::time_point now,
      CommandMessage& command) const;
  bool estimatorUsable(std::chrono::steady_clock::time_point now) const;
  static CommandIdentity commandIdentity(const CommandMessage& command) noexcept;
  static bool isNewerCommand(
      const CommandIdentity& candidate,
      const CommandIdentity& baseline) noexcept;

  std::string command_topic_ = "trajectory_command";
  std::string timesync_topic_ = "fmu/out/timesync_status";
  std::string estimator_topic_ = "fmu/out/vehicle_local_position_v1";
  std::string trajectory_setpoint_topic_ = "fmu/in/trajectory_setpoint";
  std::string offboard_control_mode_topic_ = "fmu/in/offboard_control_mode";
  double setpoint_rate_hz_ = 50.0;
  double heartbeat_rate_hz_ = 10.0;
  double command_timeout_seconds_ = 0.25;
  double timesync_timeout_seconds_ = 0.5;
  double estimator_timeout_seconds_ = 0.25;
  bool configuration_valid_ = false;

  mutable std::mutex command_mutex_;
  std::optional<CommandMessage> last_command_;
  std::optional<std::chrono::steady_clock::time_point> last_command_received_at_;
  std::optional<CommandIdentity> reset_blocked_command_;

  Px4TimestampSource timestamp_source_;
  Px4EstimatorValidityMonitor estimator_monitor_;
  Px4TrajectoryAdapter adapter_;

  rclcpp::Subscription<CommandMessage>::SharedPtr command_subscription_;
  rclcpp::Subscription<TimesyncMessage>::SharedPtr timesync_subscription_;
  rclcpp::Subscription<LocalPositionMessage>::SharedPtr estimator_subscription_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr
    trajectory_setpoint_publisher_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr
    offboard_control_mode_publisher_;
  rclcpp::TimerBase::SharedPtr setpoint_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
};

}  // namespace mpc_control_px4
