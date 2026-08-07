#include "mpc_control_ros/mpc_trajectory_node.hpp"

#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

namespace
{

using namespace std::chrono_literals;
using mpc_control_msgs::msg::MpcDiagnostics;
using mpc_control_msgs::msg::ReferenceTrajectory;
using mpc_control_msgs::msg::TrajectoryPoint;
using mpc_control_msgs::msg::VehicleState;

class MpcTrajectoryNodeTest : public ::testing::Test
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
    const auto *test_info = ::testing::UnitTest::GetInstance()->current_test_info();
    test_name_ = test_info == nullptr ? "unknown" : test_info->name();
    use_sim_time_ = test_name_ == "ClockJumpStopsCommandAndReportsTimeJump";

    const std::string suffix = std::to_string(static_cast<std::uint64_t>(::getpid()))
      + "_" + test_name_;
    const std::string prefix = "/mpc_ros_test_" + suffix;
    reference_topic_ = prefix + "/reference";
    state_topic_ = prefix + "/state";
    command_topic_ = prefix + "/command";
    prediction_topic_ = prefix + "/prediction";
    diagnostics_topic_ = prefix + "/diagnostics";

    io_node_ = std::make_shared<rclcpp::Node>("mpc_ros_test_io_" + suffix);
    if (use_sim_time_) {
      clock_publisher_ = io_node_->create_publisher<rosgraph_msgs::msg::Clock>(
        "/clock", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    }

    createNode();

    const auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    reference_publisher_ = io_node_->create_publisher<ReferenceTrajectory>(
      reference_topic_, reliable_qos);
    state_publisher_ = io_node_->create_publisher<VehicleState>(state_topic_, reliable_qos);
    if (test_name_ == "BestEffortInputsDoNotMatchReliableSubscriptions") {
      best_effort_reference_publisher_ = io_node_->create_publisher<ReferenceTrajectory>(
        reference_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).best_effort());
      best_effort_state_publisher_ = io_node_->create_publisher<VehicleState>(
        state_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).best_effort());
    }

    command_subscription_ = io_node_->create_subscription<
      mpc_control_msgs::msg::TrajectoryCommand>(
        command_topic_, reliable_qos,
        [this](const mpc_control_msgs::msg::TrajectoryCommand::SharedPtr message) {
          std::lock_guard<std::mutex> lock(received_mutex_);
          last_command_ = *message;
          command_sequences_.push_back(message->sequence);
          command_count_++;
        });
    prediction_subscription_ = io_node_->create_subscription<
      mpc_control_msgs::msg::PredictedTrajectory>(
        prediction_topic_, reliable_qos,
        [this](const mpc_control_msgs::msg::PredictedTrajectory::SharedPtr message) {
          std::lock_guard<std::mutex> lock(received_mutex_);
          last_prediction_ = *message;
        });
    diagnostics_subscription_ = io_node_->create_subscription<MpcDiagnostics>(
      diagnostics_topic_, reliable_qos,
      [this](const MpcDiagnostics::SharedPtr message) {
        std::lock_guard<std::mutex> lock(received_mutex_);
        last_diagnostics_ = *message;
      });

    executor_.add_node(node_->get_node_base_interface());
    executor_.add_node(io_node_);
  }

  void TearDown() override
  {
    if (node_) {
      executor_.remove_node(node_->get_node_base_interface());
    }
    executor_.remove_node(io_node_);
    node_.reset();
    io_node_.reset();
  }

  rclcpp::NodeOptions nodeOptions() const
  {
    rclcpp::NodeOptions options;
    options.append_parameter_override("reference_topic", reference_topic_);
    options.append_parameter_override("state_topic", state_topic_);
    options.append_parameter_override("command_topic", command_topic_);
    options.append_parameter_override("prediction_topic", prediction_topic_);
    options.append_parameter_override("diagnostics_topic", diagnostics_topic_);
    options.append_parameter_override("update_rate_hz", 20.0);
    options.append_parameter_override("reference_timeout_seconds", 0.30);
    options.append_parameter_override("state_timeout_seconds", 0.30);
    if (use_sim_time_) {
      options.append_parameter_override("use_sim_time", true);
    }
    return options;
  }

  void createNode()
  {
    node_ = std::make_shared<mpc_control_ros::MpcTrajectoryNode>(nodeOptions());
  }

  void replaceNode()
  {
    executor_.remove_node(node_->get_node_base_interface());
    node_.reset();
    spinFor(100ms);
    createNode();
    executor_.add_node(node_->get_node_base_interface());
    spinFor(100ms);
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

  void publishClock(const double seconds)
  {
    ASSERT_TRUE(clock_publisher_ != nullptr);
    rosgraph_msgs::msg::Clock clock;
    const auto whole_seconds = static_cast<std::int64_t>(std::floor(seconds));
    clock.clock.sec = static_cast<std::int32_t>(whole_seconds);
    clock.clock.nanosec = static_cast<std::uint32_t>(
      std::llround((seconds - static_cast<double>(whole_seconds)) * 1.0e9));
    clock_publisher_->publish(clock);
    spinFor(30ms);
  }

  ReferenceTrajectory makeReference()
  {
    return makeReferenceAt(node_->get_clock()->now());
  }

  ReferenceTrajectory makeReferenceAt(const rclcpp::Time& stamp)
  {
    ReferenceTrajectory reference;
    reference.header.stamp = static_cast<builtin_interfaces::msg::Time>(stamp);
    reference.header.frame_id = "map";
    reference.trajectory_id = 42U;
    reference.hold_after_end = true;
    reference.points.resize(2);
    for (std::size_t index = 0; index < reference.points.size(); ++index) {
      auto& point = reference.points[index];
      point.time_from_start.sec = static_cast<std::int32_t>(index * 10U);
      point.time_from_start.nanosec = 0U;
      point.position = {0.0, 0.0, 0.0};
      point.velocity = {0.0, 0.0, 0.0};
      point.acceleration = {0.0, 0.0, 0.0};
      point.yaw = 0.0;
      point.yaw_rate = 0.0;
      point.yaw_valid = false;
    }
    return reference;
  }

  VehicleState makeState()
  {
    return makeStateAt(node_->get_clock()->now(), 1U);
  }

  VehicleState makeStateAt(const rclcpp::Time& stamp, const std::uint64_t sequence)
  {
    VehicleState state;
    state.header.stamp = static_cast<builtin_interfaces::msg::Time>(stamp);
    state.header.frame_id = "map";
    state.sequence = sequence;
    state.position = {0.0, 0.0, 0.0};
    state.velocity = {0.0, 0.0, 0.0};
    state.acceleration = {0.0, 0.0, 0.0};
    state.yaw = 0.0;
    state.yaw_rate = 0.0;
    state.valid = true;
    state.position_valid = true;
    state.velocity_valid = true;
    state.acceleration_valid = false;
    state.yaw_valid = true;
    state.reset_counter_valid = true;
    state.reset_counter = 0U;
    return state;
  }

  void publishInputs()
  {
    auto reference = makeReference();
    auto state = makeState();
    for (int attempt = 0; attempt < 5; ++attempt) {
      reference_publisher_->publish(reference);
      state.header.stamp = static_cast<builtin_interfaces::msg::Time>(
        node_->get_clock()->now());
      state_publisher_->publish(state);
      spinFor(20ms);
    }
  }

  void configureAndActivate()
  {
    using lifecycle_msgs::msg::Transition;
    using CallbackReturn =
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    CallbackReturn callback_return = CallbackReturn::FAILURE;
    const auto& configured_state = node_->trigger_transition(
      Transition::TRANSITION_CONFIGURE, callback_return);
    ASSERT_EQ(callback_return, CallbackReturn::SUCCESS);
    ASSERT_EQ(
      configured_state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

    const auto& active_state = node_->trigger_transition(
      Transition::TRANSITION_ACTIVATE, callback_return);
    ASSERT_EQ(callback_return, CallbackReturn::SUCCESS);
    ASSERT_EQ(active_state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  }

  std::shared_ptr<mpc_control_ros::MpcTrajectoryNode> node_;
  std::shared_ptr<rclcpp::Node> io_node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Publisher<ReferenceTrajectory>::SharedPtr reference_publisher_;
  rclcpp::Publisher<VehicleState>::SharedPtr state_publisher_;
  rclcpp::Publisher<ReferenceTrajectory>::SharedPtr best_effort_reference_publisher_;
  rclcpp::Publisher<VehicleState>::SharedPtr best_effort_state_publisher_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher_;
  rclcpp::Subscription<mpc_control_msgs::msg::TrajectoryCommand>::SharedPtr
    command_subscription_;
  rclcpp::Subscription<mpc_control_msgs::msg::PredictedTrajectory>::SharedPtr
    prediction_subscription_;
  rclcpp::Subscription<MpcDiagnostics>::SharedPtr diagnostics_subscription_;
  std::mutex received_mutex_;
  mpc_control_msgs::msg::TrajectoryCommand last_command_;
  mpc_control_msgs::msg::PredictedTrajectory last_prediction_;
  MpcDiagnostics last_diagnostics_;
  std::vector<std::uint64_t> command_sequences_;
  std::size_t command_count_ = 0;
  bool use_sim_time_ = false;
  std::string test_name_;
  std::string reference_topic_;
  std::string state_topic_;
  std::string command_topic_;
  std::string prediction_topic_;
  std::string diagnostics_topic_;
};

TEST_F(MpcTrajectoryNodeTest, LifecycleAndReferenceTimeoutAreObservable)
{
  configureAndActivate();
  publishInputs();

  spinFor(150ms);
  std::size_t command_count_after_activation = 0U;
  {
    std::lock_guard<std::mutex> lock(received_mutex_);
    command_count_after_activation = command_count_;
    ASSERT_GT(command_count_, 0U);
    ASSERT_EQ(last_prediction_.points.size(), 26U);
    EXPECT_EQ(last_command_.trajectory_id, 42U);
    EXPECT_FALSE(last_command_.yaw_valid);
    EXPECT_TRUE(last_diagnostics_.command_published);
  }

  spinFor(400ms);
  std::size_t command_count_after_timeout = 0U;
  {
    std::lock_guard<std::mutex> lock(received_mutex_);
    EXPECT_GE(command_count_, command_count_after_activation);
    command_count_after_timeout = command_count_;
    EXPECT_FALSE(last_diagnostics_.command_published);
    EXPECT_EQ(
      last_diagnostics_.failure_reason,
      MpcDiagnostics::FAILURE_STALE_REFERENCE);
    EXPECT_EQ(
      last_diagnostics_.lifecycle_state,
      MpcDiagnostics::LIFECYCLE_ACTIVE);
  }

  spinFor(200ms);
  {
    std::lock_guard<std::mutex> lock(received_mutex_);
    EXPECT_EQ(command_count_, command_count_after_timeout);
  }
}

TEST_F(MpcTrajectoryNodeTest, ClockJumpStopsCommandAndReportsTimeJump)
{
  ASSERT_TRUE(use_sim_time_);
  publishClock(1000.0);
  configureAndActivate();
  publishInputs();
  spinFor(150ms);

  {
    std::lock_guard<std::mutex> lock(received_mutex_);
    ASSERT_GT(command_count_, 0U);
    EXPECT_TRUE(last_diagnostics_.command_published);
  }

  publishClock(999.0);
  spinFor(120ms);

  std::lock_guard<std::mutex> lock(received_mutex_);
  EXPECT_FALSE(last_diagnostics_.command_published);
  EXPECT_EQ(last_diagnostics_.failure_reason, MpcDiagnostics::FAILURE_TIME_JUMP);
}

TEST_F(MpcTrajectoryNodeTest, OutOfOrderStateStopsCommandAndReportsOutOfOrder)
{
  configureAndActivate();
  publishInputs();
  spinFor(150ms);

  std::size_t command_count_before_invalid_state = 0U;
  rclcpp::Time latest_state_time;
  {
    std::lock_guard<std::mutex> lock(received_mutex_);
    ASSERT_GT(command_count_, 0U);
    command_count_before_invalid_state = command_count_;
    latest_state_time = node_->get_clock()->now();
  }

  auto out_of_order_state = makeStateAt(latest_state_time - rclcpp::Duration::from_seconds(1.0), 2U);
  state_publisher_->publish(out_of_order_state);
  spinFor(120ms);

  std::lock_guard<std::mutex> lock(received_mutex_);
  EXPECT_EQ(command_count_, command_count_before_invalid_state);
  EXPECT_FALSE(last_diagnostics_.command_published);
  EXPECT_EQ(last_diagnostics_.failure_reason, MpcDiagnostics::FAILURE_OUT_OF_ORDER);
}

TEST_F(MpcTrajectoryNodeTest, BestEffortInputsDoNotMatchReliableSubscriptions)
{
  configureAndActivate();
  const auto reference = makeReference();
  const auto state = makeState();

  for (int attempt = 0; attempt < 10; ++attempt) {
    best_effort_reference_publisher_->publish(reference);
    best_effort_state_publisher_->publish(state);
    spinFor(20ms);
  }
  spinFor(150ms);

  std::lock_guard<std::mutex> lock(received_mutex_);
  EXPECT_EQ(command_count_, 0U);
  EXPECT_FALSE(last_diagnostics_.command_published);
  EXPECT_EQ(
    last_diagnostics_.failure_reason,
    MpcDiagnostics::FAILURE_INVALID_REFERENCE);
}

TEST_F(MpcTrajectoryNodeTest, RestartRecreatesActiveStreamWithFreshSequence)
{
  configureAndActivate();
  publishInputs();
  spinFor(150ms);

  std::size_t command_count_before_restart = 0U;
  {
    std::lock_guard<std::mutex> lock(received_mutex_);
    ASSERT_GT(command_count_, 0U);
    command_count_before_restart = command_count_;
  }

  replaceNode();
  std::size_t sequence_start_index = 0U;
  {
    std::lock_guard<std::mutex> lock(received_mutex_);
    sequence_start_index = command_sequences_.size();
  }
  configureAndActivate();
  publishInputs();
  spinFor(150ms);

  std::lock_guard<std::mutex> lock(received_mutex_);
  ASSERT_GT(command_count_, command_count_before_restart);
  const auto first_new_command = std::find(
    command_sequences_.begin() + static_cast<std::ptrdiff_t>(sequence_start_index),
    command_sequences_.end(), 0U);
  EXPECT_NE(first_new_command, command_sequences_.end());
  EXPECT_TRUE(last_diagnostics_.command_published);
}

}  // namespace
