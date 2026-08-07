#include "mpc_control_px4/px4_trajectory_setpoint_publisher.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

namespace
{

using namespace std::chrono_literals;
using mpc_control_msgs::msg::TrajectoryCommand;
using mpc_control_px4::Px4TrajectorySetpointPublisher;
using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::TimesyncStatus;
using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::VehicleLocalPosition;

class Px4TrajectorySetpointPublisherTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    int argc = 0;
    rclcpp::init(argc, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    const auto* test_info = ::testing::UnitTest::GetInstance()->current_test_info();
    const std::string test_name = test_info == nullptr ? "unknown" : test_info->name();
    const std::string prefix = "/m6_wp2_"
      + std::to_string(static_cast<std::uint64_t>(::getpid()))
      + "_" + test_name;
    command_topic_ = prefix + "/command";
    timesync_topic_ = prefix + "/fmu/out/timesync_status";
    estimator_topic_ = prefix + "/fmu/out/vehicle_local_position_v1";
    trajectory_setpoint_topic_ = prefix + "/fmu/in/trajectory_setpoint";
    offboard_control_mode_topic_ = prefix + "/fmu/in/offboard_control_mode";

    rclcpp::NodeOptions options;
    options.append_parameter_override("command_topic", command_topic_);
    options.append_parameter_override("timesync_topic", timesync_topic_);
    options.append_parameter_override("estimator_topic", estimator_topic_);
    options.append_parameter_override(
      "trajectory_setpoint_topic", trajectory_setpoint_topic_);
    options.append_parameter_override(
      "offboard_control_mode_topic", offboard_control_mode_topic_);
    options.append_parameter_override("setpoint_rate_hz", 20.0);
    options.append_parameter_override("heartbeat_rate_hz", 10.0);
    options.append_parameter_override("command_timeout_seconds", 0.20);
    options.append_parameter_override("timesync_timeout_seconds", 0.40);
    options.append_parameter_override("estimator_timeout_seconds", 0.40);

    publisher_node_ = std::make_shared<Px4TrajectorySetpointPublisher>(options);
    io_node_ = std::make_shared<rclcpp::Node>(
      "m6_wp2_io_" + std::to_string(static_cast<std::uint64_t>(::getpid())));

    const auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    const auto px4_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    command_publisher_ = io_node_->create_publisher<TrajectoryCommand>(
      command_topic_, reliable_qos);
    timesync_publisher_ = io_node_->create_publisher<TimesyncStatus>(
      timesync_topic_, px4_qos);
    estimator_publisher_ = io_node_->create_publisher<VehicleLocalPosition>(
      estimator_topic_, px4_qos);
    trajectory_setpoint_subscription_ = io_node_->create_subscription<TrajectorySetpoint>(
      trajectory_setpoint_topic_, px4_qos,
      [this](const TrajectorySetpoint::SharedPtr message) {
        std::lock_guard<std::mutex> lock(received_mutex_);
        setpoints_.push_back(*message);
      });
    offboard_control_mode_subscription_ = io_node_->create_subscription<OffboardControlMode>(
      offboard_control_mode_topic_, px4_qos,
      [this](const OffboardControlMode::SharedPtr message) {
        std::lock_guard<std::mutex> lock(received_mutex_);
        heartbeats_.push_back(*message);
      });

    executor_.add_node(publisher_node_);
    executor_.add_node(io_node_);
  }

  void TearDown() override
  {
    executor_.remove_node(publisher_node_);
    executor_.remove_node(io_node_);
    publisher_node_.reset();
    io_node_.reset();
  }

  void spinFor(const std::chrono::milliseconds duration)
  {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some();
      std::this_thread::sleep_for(5ms);
    }
    executor_.spin_some();
  }

  bool waitFor(const std::function<bool()>& predicate,
               const std::chrono::milliseconds timeout = 2s)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some();
      if (predicate()) {
        return true;
      }
      std::this_thread::sleep_for(5ms);
    }
    executor_.spin_some();
    return predicate();
  }

  TrajectoryCommand makeCommand() const
  {
    TrajectoryCommand command;
    command.header.frame_id = "map";
    command.trajectory_id = 9U;
    command.sequence = 4U;
    command.position = {1.0, 2.0, 3.0};
    command.velocity = {4.0, 5.0, 6.0};
    command.acceleration = {7.0, 8.0, 9.0};
    command.yaw_valid = false;
    return command;
  }

  TimesyncStatus makeTimesync(const std::uint64_t timestamp) const
  {
    TimesyncStatus status;
    status.timestamp = timestamp;
    status.source_protocol = TimesyncStatus::SOURCE_PROTOCOL_DDS;
    status.remote_timestamp = timestamp;
    return status;
  }

  VehicleLocalPosition makeLocalPosition(const std::uint64_t timestamp) const
  {
    VehicleLocalPosition message;
    message.timestamp = timestamp;
    message.xy_valid = true;
    message.z_valid = true;
    message.v_xy_valid = true;
    message.v_z_valid = true;
    message.x = 1.0F;
    message.y = 2.0F;
    message.z = -3.0F;
    message.vx = 0.1F;
    message.vy = 0.2F;
    message.vz = 0.3F;
    message.heading = 0.4F;
    message.heading_good_for_control = true;
    return message;
  }

  void publishValidInputs(const int count)
  {
    const auto command = makeCommand();
    for (int index = 0; index < count; ++index) {
      timesync_publisher_->publish(makeTimesync(
          1000000U + static_cast<std::uint64_t>(index) * 20000U));
      estimator_publisher_->publish(makeLocalPosition(
          2000000U + static_cast<std::uint64_t>(index) * 20000U));
      command_publisher_->publish(command);
      spinFor(20ms);
    }
  }

  std::shared_ptr<Px4TrajectorySetpointPublisher> publisher_node_;
  std::shared_ptr<rclcpp::Node> io_node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Publisher<TrajectoryCommand>::SharedPtr command_publisher_;
  rclcpp::Publisher<TimesyncStatus>::SharedPtr timesync_publisher_;
  rclcpp::Publisher<VehicleLocalPosition>::SharedPtr estimator_publisher_;
  rclcpp::Subscription<TrajectorySetpoint>::SharedPtr
    trajectory_setpoint_subscription_;
  rclcpp::Subscription<OffboardControlMode>::SharedPtr
    offboard_control_mode_subscription_;
  std::mutex received_mutex_;
  std::vector<TrajectorySetpoint> setpoints_;
  std::vector<OffboardControlMode> heartbeats_;
  std::string command_topic_;
  std::string timesync_topic_;
  std::string estimator_topic_;
  std::string trajectory_setpoint_topic_;
  std::string offboard_control_mode_topic_;
};

TEST_F(
  Px4TrajectorySetpointPublisherTest,
  TopicEchoPublishesSetpointAndIndependentHeartbeatWithoutArm)
{
  ASSERT_TRUE(waitFor([this]() {
    return command_publisher_->get_subscription_count() > 0U
      && timesync_publisher_->get_subscription_count() > 0U
      && estimator_publisher_->get_subscription_count() > 0U
      && publisher_node_->count_subscribers(trajectory_setpoint_topic_) > 0U
      && publisher_node_->count_subscribers(offboard_control_mode_topic_) > 0U;
  }));

  publishValidInputs(12);
  ASSERT_TRUE(waitFor([this]() {
    std::lock_guard<std::mutex> lock(received_mutex_);
    return setpoints_.size() >= 3U && heartbeats_.size() >= 2U;
  }));

  std::lock_guard<std::mutex> lock(received_mutex_);
  ASSERT_FALSE(setpoints_.empty());
  ASSERT_FALSE(heartbeats_.empty());
  EXPECT_EQ(setpoints_.front().position[0], 2.0F);
  EXPECT_EQ(setpoints_.front().position[1], 1.0F);
  EXPECT_EQ(setpoints_.front().position[2], -3.0F);
  EXPECT_EQ(setpoints_.front().velocity[0], 5.0F);
  EXPECT_EQ(setpoints_.front().acceleration[2], -9.0F);
  EXPECT_TRUE(std::isfinite(setpoints_.front().position[0]));
  EXPECT_TRUE(std::isnan(setpoints_.front().yaw));
  EXPECT_TRUE(std::isnan(setpoints_.front().jerk[0]));

  for (std::size_t index = 1; index < setpoints_.size(); ++index) {
    EXPECT_GT(setpoints_[index].timestamp, setpoints_[index - 1].timestamp);
  }
  for (std::size_t index = 1; index < heartbeats_.size(); ++index) {
    EXPECT_GT(heartbeats_[index].timestamp, heartbeats_[index - 1].timestamp);
  }

  EXPECT_TRUE(heartbeats_.front().position);
  EXPECT_FALSE(heartbeats_.front().velocity);
  EXPECT_FALSE(heartbeats_.front().acceleration);
  EXPECT_FALSE(heartbeats_.front().attitude);
  EXPECT_FALSE(heartbeats_.front().body_rate);
  EXPECT_FALSE(heartbeats_.front().thrust_and_torque);
  EXPECT_FALSE(heartbeats_.front().direct_actuator);

  EXPECT_EQ(publisher_node_->count_publishers("/fmu/in/vehicle_command"), 0U);
  EXPECT_EQ(publisher_node_->count_publishers("/fmu/in/actuator_motors"), 0U);
}

TEST_F(Px4TrajectorySetpointPublisherTest, StopsBothStreamsAfterCommandTimeout)
{
  ASSERT_TRUE(waitFor([this]() {
    return command_publisher_->get_subscription_count() > 0U
      && timesync_publisher_->get_subscription_count() > 0U
      && estimator_publisher_->get_subscription_count() > 0U;
  }));

  publishValidInputs(6);
  ASSERT_TRUE(waitFor([this]() {
    std::lock_guard<std::mutex> lock(received_mutex_);
    return !setpoints_.empty() && !heartbeats_.empty();
  }));

  spinFor(350ms);
  std::size_t setpoint_count_after_timeout = 0U;
  std::size_t heartbeat_count_after_timeout = 0U;
  {
    std::lock_guard<std::mutex> lock(received_mutex_);
    setpoint_count_after_timeout = setpoints_.size();
    heartbeat_count_after_timeout = heartbeats_.size();
  }
  spinFor(250ms);
  {
    std::lock_guard<std::mutex> lock(received_mutex_);
    EXPECT_EQ(setpoints_.size(), setpoint_count_after_timeout);
    EXPECT_EQ(heartbeats_.size(), heartbeat_count_after_timeout);
  }
}

TEST_F(
  Px4TrajectorySetpointPublisherTest,
  ResumesAfterTimesyncGapWithFreshMonotonicAnchor)
{
  ASSERT_TRUE(waitFor([this]() {
    return command_publisher_->get_subscription_count() > 0U
      && timesync_publisher_->get_subscription_count() > 0U
      && estimator_publisher_->get_subscription_count() > 0U;
  }));

  const auto command = makeCommand();
  auto state = makeLocalPosition(2000000U);
  for (int index = 0; index < 12; ++index) {
    timesync_publisher_->publish(makeTimesync(
        1000000U + static_cast<std::uint64_t>(index) * 20000U));
    state.timestamp = 2000000U + static_cast<std::uint64_t>(index) * 20000U;
    estimator_publisher_->publish(state);
    command_publisher_->publish(command);
    spinFor(20ms);
  }

  ASSERT_TRUE(waitFor([this]() {
    std::lock_guard<std::mutex> lock(received_mutex_);
    return !setpoints_.empty() && !heartbeats_.empty();
  }));

  // Simulate a DDS/time-sync outage while the estimator and command remain
  // fresh. The publisher must stop after the bounded timesync timeout.
  for (int index = 0; index < 30; ++index) {
    state.timestamp = 3000000U + static_cast<std::uint64_t>(index) * 20000U;
    estimator_publisher_->publish(state);
    command_publisher_->publish(command);
    spinFor(20ms);
  }
  std::size_t setpoints_before_recovery = 0U;
  std::size_t heartbeats_before_recovery = 0U;
  {
    std::lock_guard<std::mutex> lock(received_mutex_);
    setpoints_before_recovery = setpoints_.size();
    heartbeats_before_recovery = heartbeats_.size();
  }
  spinFor(150ms);
  {
    std::lock_guard<std::mutex> lock(received_mutex_);
    EXPECT_EQ(setpoints_.size(), setpoints_before_recovery);
    EXPECT_EQ(heartbeats_.size(), heartbeats_before_recovery);
  }

  // A strictly newer DDS anchor is sufficient for a timesync-only recovery;
  // estimator reset recovery has a separate newer-command identity policy.
  bool resumed = false;
  for (int index = 0; index < 24; ++index) {
    timesync_publisher_->publish(makeTimesync(
        6000000U + static_cast<std::uint64_t>(index) * 20000U));
    state.timestamp = 6000000U + static_cast<std::uint64_t>(index) * 20000U;
    estimator_publisher_->publish(state);
    command_publisher_->publish(command);
    spinFor(20ms);
    {
      std::lock_guard<std::mutex> lock(received_mutex_);
      if (setpoints_.size() > setpoints_before_recovery
          && heartbeats_.size() > heartbeats_before_recovery) {
        resumed = true;
        break;
      }
    }
  }
  EXPECT_TRUE(resumed);
}

TEST_F(
  Px4TrajectorySetpointPublisherTest,
  StopsOnEstimatorInvalidityAndRequiresNewCommandAfterReset)
{
  ASSERT_TRUE(waitFor([this]() {
    return command_publisher_->get_subscription_count() > 0U
      && timesync_publisher_->get_subscription_count() > 0U
      && estimator_publisher_->get_subscription_count() > 0U;
  }));

  publishValidInputs(12);
  ASSERT_TRUE(waitFor([this]() {
    std::lock_guard<std::mutex> lock(received_mutex_);
    return !setpoints_.empty() && !heartbeats_.empty();
  }));

  auto invalid = makeLocalPosition(3000000U);
  invalid.xy_valid = false;
  for (int index = 0; index < 12; ++index) {
    timesync_publisher_->publish(makeTimesync(
        3000000U + static_cast<std::uint64_t>(index) * 20000U));
    invalid.timestamp = 3000000U + static_cast<std::uint64_t>(index) * 20000U;
    estimator_publisher_->publish(invalid);
    command_publisher_->publish(makeCommand());
    spinFor(20ms);
  }

  std::size_t setpoints_after_invalid = 0U;
  std::size_t heartbeats_after_invalid = 0U;
  {
    std::lock_guard<std::mutex> lock(received_mutex_);
    setpoints_after_invalid = setpoints_.size();
    heartbeats_after_invalid = heartbeats_.size();
  }
  spinFor(150ms);
  {
    std::lock_guard<std::mutex> lock(received_mutex_);
    EXPECT_EQ(setpoints_.size(), setpoints_after_invalid);
    EXPECT_EQ(heartbeats_.size(), heartbeats_after_invalid);
  }

  auto recovered_state = makeLocalPosition(4000000U);
  recovered_state.xy_reset_counter = 1U;
  for (int index = 0; index < 8; ++index) {
    timesync_publisher_->publish(makeTimesync(
        4000000U + static_cast<std::uint64_t>(index) * 20000U));
    recovered_state.timestamp = 4000000U + static_cast<std::uint64_t>(index) * 20000U;
    estimator_publisher_->publish(recovered_state);
    command_publisher_->publish(makeCommand());
    spinFor(20ms);
  }
  {
    std::lock_guard<std::mutex> lock(received_mutex_);
    EXPECT_EQ(setpoints_.size(), setpoints_after_invalid);
    EXPECT_EQ(heartbeats_.size(), heartbeats_after_invalid);
  }

  auto new_command = makeCommand();
  new_command.sequence = 5U;
  bool resumed = false;
  for (int index = 0; index < 30; ++index) {
    timesync_publisher_->publish(makeTimesync(
        5000000U + static_cast<std::uint64_t>(index) * 20000U));
    recovered_state.timestamp = 5000000U + static_cast<std::uint64_t>(index) * 20000U;
    estimator_publisher_->publish(recovered_state);
    command_publisher_->publish(new_command);
    spinFor(20ms);
    {
      std::lock_guard<std::mutex> lock(received_mutex_);
      if (setpoints_.size() > setpoints_after_invalid
          && heartbeats_.size() > heartbeats_after_invalid) {
        resumed = true;
        break;
      }
    }
  }
  EXPECT_TRUE(resumed);
}

}  // namespace
