#pragma once

#include <mpc_control/mpc_trajectory_core.hpp>
#include <mpc_control/reference_sampler.hpp>

#include <mpc_control_msgs/msg/mpc_diagnostics.hpp>
#include <mpc_control_msgs/msg/predicted_trajectory.hpp>
#include <mpc_control_msgs/msg/reference_trajectory.hpp>
#include <mpc_control_msgs/msg/trajectory_command.hpp>
#include <mpc_control_msgs/msg/trajectory_point.hpp>
#include <mpc_control_msgs/msg/vehicle_state.hpp>

#include <builtin_interfaces/msg/duration.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

namespace mpc_control_ros
{

class MpcTrajectoryNode final : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit MpcTrajectoryNode(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  MpcTrajectoryNode(const MpcTrajectoryNode&) = delete;
  MpcTrajectoryNode& operator=(const MpcTrajectoryNode&) = delete;

protected:
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State& state) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State& state) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State& state) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State& state) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State& state) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_error(const rclcpp_lifecycle::State& state) override;

private:
  using ReferenceMessage = mpc_control_msgs::msg::ReferenceTrajectory;
  using StateMessage = mpc_control_msgs::msg::VehicleState;
  using CommandMessage = mpc_control_msgs::msg::TrajectoryCommand;
  using PredictionMessage = mpc_control_msgs::msg::PredictedTrajectory;
  using DiagnosticsMessage = mpc_control_msgs::msg::MpcDiagnostics;
  using CommandPublisher = rclcpp_lifecycle::LifecyclePublisher<CommandMessage>;
  using PredictionPublisher = rclcpp_lifecycle::LifecyclePublisher<PredictionMessage>;

  struct CachedReference
  {
    mpc_control::ReferenceTrajectory trajectory;
    rclcpp::Time epoch{0, 0, RCL_ROS_TIME};
    rclcpp::Time received_at{0, 0, RCL_ROS_TIME};
    std::uint64_t trajectory_id = 0;
    bool hold_after_end = true;
    bool yaw_valid = false;
  };

  struct CachedState
  {
    mpc_control::VehicleState state;
    rclcpp::Time measured_at{0, 0, RCL_ROS_TIME};
    rclcpp::Time received_at{0, 0, RCL_ROS_TIME};
    std::uint64_t sequence = 0;
    std::uint32_t reset_counter = 0;
    bool reset_counter_valid = false;
  };

  void referenceCallback(const ReferenceMessage::SharedPtr message);
  void stateCallback(const StateMessage::SharedPtr message);
  void updateCallback();

  bool convertReference(
      const ReferenceMessage& message,
      CachedReference& converted,
      std::string& error) const;
  bool convertState(
      const StateMessage& message,
      CachedState& converted,
      std::string& error) const;

  bool referenceFresh(const rclcpp::Time& now, double& age_seconds) const;
  bool stateFresh(const rclcpp::Time& now, double& age_seconds) const;
  bool activateCoreIfNeeded(
      double reference_time_seconds,
      const CachedState& state,
      std::string& error);
  bool resetCoreIfNeeded(
      double reference_time_seconds,
      const CachedState& state);

  void publishCommand(
      const mpc_control::MpcUpdateResult& result,
      const mpc_control::ReferenceHorizon& horizon,
      const rclcpp::Time& now);
  void publishPrediction(
      const mpc_control::MpcUpdateResult& result,
      const mpc_control::ReferenceHorizon& horizon,
      const rclcpp::Time& now);
  void publishDiagnostics(
      const rclcpp::Time& now,
      std::uint8_t failure_reason,
      const std::string& detail,
      bool command_published,
      bool prediction_valid,
      double reference_age_seconds,
      double state_age_seconds,
      double solve_time_seconds,
      const mpc_control::MpcUpdateResult* result = nullptr,
      bool reset_detected = false,
      bool deadline_missed = false);

  static std::uint8_t lifecycleStateId(std::uint8_t lifecycle_id) noexcept;
  static std::uint8_t mapCoreFailure(mpc_control::MpcFailureReason reason) noexcept;
  static bool finitePoint(const ReferenceMessage::_points_type::value_type& point) noexcept;
  static bool finiteState(const StateMessage& message) noexcept;
  static double durationSeconds(const builtin_interfaces::msg::Duration& duration) noexcept;
  static builtin_interfaces::msg::Duration durationMessage(double seconds) noexcept;

  void resetRuntimeState() noexcept;

  mutable std::mutex mutex_;

  std::string frame_id_ = "map";
  std::string reference_topic_ = "reference_trajectory";
  std::string state_topic_ = "vehicle_state";
  std::string command_topic_ = "trajectory_command";
  std::string prediction_topic_ = "predicted_trajectory";
  std::string diagnostics_topic_ = "mpc_diagnostics";
  double update_rate_hz_ = 50.0;
  double reference_timeout_seconds_ = 0.5;
  double state_timeout_seconds_ = 0.5;
  double clock_stall_timeout_seconds_ = 0.25;
  double clock_jump_threshold_seconds_ = 0.5;
  double derivative_position_tolerance_ = 1.0e-3;
  double derivative_velocity_tolerance_ = 1.0e-3;
  bool check_derivative_consistency_ = true;
  bool use_measured_acceleration_on_activate_ = false;

  mpc_control::MpcCoreConfiguration core_configuration_{};
  mpc_control::ReferenceSamplerConfig sampler_configuration_{};
  std::unique_ptr<mpc_control::MpcTrajectoryCore> core_;
  mpc_control::ReferenceSampler sampler_{};

  std::optional<CachedReference> reference_;
  std::optional<CachedState> state_;
  std::optional<rclcpp::Time> last_state_timestamp_;
  std::optional<rclcpp::Time> last_update_time_;
  std::optional<std::chrono::steady_clock::time_point> last_clock_progress_wall_time_;
  std::uint32_t last_reset_counter_ = 0;
  bool last_reset_counter_valid_ = false;
  bool reference_valid_ = false;
  bool state_valid_ = false;
  bool reset_detected_ = false;
  std::uint8_t last_failure_reason_ = DiagnosticsMessage::FAILURE_NONE;
  std::string last_failure_detail_;
  std::uint32_t consecutive_failures_ = 0;
  std::uint64_t published_sequence_ = 0;

  rclcpp::Subscription<ReferenceMessage>::SharedPtr reference_subscription_;
  rclcpp::Subscription<StateMessage>::SharedPtr state_subscription_;
  std::shared_ptr<CommandPublisher> command_publisher_;
  std::shared_ptr<PredictionPublisher> prediction_publisher_;
  rclcpp::Publisher<DiagnosticsMessage>::SharedPtr diagnostics_publisher_;
  rclcpp::TimerBase::SharedPtr update_timer_;
};

}  // namespace mpc_control_ros
