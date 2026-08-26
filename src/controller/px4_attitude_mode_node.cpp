#include "mpc_controller/msg/force_attitude_setpoint.hpp"
#include "mpc_controller/controller/command_safety_limiter.hpp"
#include "mpc_controller/controller/geometric_controller.hpp"

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/mode_executor.hpp>
#include <px4_ros2/components/node_with_mode.hpp>
#include <px4_ros2/control/setpoint_types/experimental/attitude.hpp>
#include <px4_msgs/msg/hover_thrust_estimate.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_ros2/utils/message_version.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

static const std::string kNodeName = "px4_attitude_mode_node";

class MpcFlightMode : public px4_ros2::ModeBase
{
public:
  explicit MpcFlightMode(rclcpp::Node & node)
  : ModeBase(
      node,
      Settings{"MPC Controller"}
        .activateEvenWhileDisarmed(false)
        .preventArming(false)),
    node_(node)
  {
    attitude_setpoint_ = std::make_shared<px4_ros2::AttitudeSetpointType>(*this);
    external_mode_state_publisher_ = node_.create_publisher<std_msgs::msg::Bool>(
      "mpc_external_mode_active", rclcpp::QoS(1).reliable().transient_local());
    applied_yaw_publisher_ = node_.create_publisher<std_msgs::msg::Float64>(
      "mpc_applied_yaw", rclcpp::QoS(10));
    applied_collective_publisher_ = node_.create_publisher<std_msgs::msg::Float64>(
      "mpc_applied_collective", rclcpp::QoS(10));
    applied_setpoint_publisher_ =
      node_.create_publisher<mpc_controller::msg::ForceAttitudeSetpoint>(
      "mpc_applied_force_attitude_setpoint", rclcpp::QoS(10));
    publishExternalModeState(false);
    node_.declare_parameter("command_safety_max_tilt_rad", 0.7853981633974483);
    node_.declare_parameter("px4_system_id", 2);
    node_.declare_parameter("command_safety_min_collective_specific_force_m_s2", 7.0);
    node_.declare_parameter("command_safety_max_collective_specific_force_m_s2", 14.0);
    node_.declare_parameter("command_safety_max_collective_rate_m_s3", 25.0);
    node_.declare_parameter("command_safety_max_yaw_rate_rad_s", 2.0);
    node_.declare_parameter("mission_timeout_seconds", 300.0);
    command_safety_limits_.maximum_tilt_rad = parameter(
      "command_safety_max_tilt_rad", command_safety_limits_.maximum_tilt_rad);
    command_safety_limits_.minimum_collective_specific_force_m_s2 = parameter(
      "command_safety_min_collective_specific_force_m_s2",
      command_safety_limits_.minimum_collective_specific_force_m_s2);
    command_safety_limits_.maximum_collective_specific_force_m_s2 = parameter(
      "command_safety_max_collective_specific_force_m_s2",
      command_safety_limits_.maximum_collective_specific_force_m_s2);
    command_safety_limits_.maximum_collective_rate_m_s3 = parameter(
      "command_safety_max_collective_rate_m_s3",
      command_safety_limits_.maximum_collective_rate_m_s3);
    command_safety_limits_.maximum_yaw_rate_rad_s = parameter(
      "command_safety_max_yaw_rate_rad_s",
      command_safety_limits_.maximum_yaw_rate_rad_s);
    if (!mpc_controller::command_safety::valid(command_safety_limits_)) {
      RCLCPP_ERROR(node_.get_logger(),
        "Invalid command safety limits; using the conservative defaults");
      command_safety_limits_ = {};
    }
    command_safety_limiter_.configure(command_safety_limits_);

    force_setpoint_sub_ = node.create_subscription<mpc_controller::msg::ForceAttitudeSetpoint>(
      "force_attitude_setpoint", rclcpp::QoS(10),
      [this](const mpc_controller::msg::ForceAttitudeSetpoint::SharedPtr msg) {
        if (!msg) {
          return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        latest_setpoint_ = *msg;
        last_setpoint_time_ = node_.now();
      });

    hover_thrust_sub_ = node.create_subscription<px4_msgs::msg::HoverThrustEstimate>(
      "/fmu/out/hover_thrust_estimate", rclcpp::QoS(10).best_effort(),
      [this](const px4_msgs::msg::HoverThrustEstimate::SharedPtr msg) {
        if (msg && msg->valid && std::isfinite(msg->hover_thrust) && msg->hover_thrust > 0.05f) {
          std::lock_guard<std::mutex> lock(mutex_);
          hover_thrust_ = std::clamp(
            static_cast<double>(msg->hover_thrust), kMinimumHoverThrust,
            kMaximumHoverThrust);
        }
      });

    vehicle_attitude_sub_ = node.create_subscription<px4_msgs::msg::VehicleAttitude>(
      "/fmu/out/vehicle_attitude" +
      px4_ros2::getMessageNameVersion<px4_msgs::msg::VehicleAttitude>(),
      rclcpp::QoS(10).best_effort(),
      [this](const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
        if (!msg) {
          return;
        }
        const Eigen::Quaterniond attitude_frd_ned(
          msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
        const auto attitude_flu_enu =
          mpc_controller::px4_control::frdNedToFluEnu(attitude_frd_ned);
        if (!attitude_flu_enu) {
          return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        measured_yaw_enu_rad_ =
          mpc_controller::command_safety::yawAngleRad(*attitude_flu_enu);
      });

    RCLCPP_INFO(node_.get_logger(),
      "MpcFlightMode initialized: registered with PX4 Flight Mode Manager");
  }

  void onActivate() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    activation_time_ = node_.now();
    last_streamed_at_ = *activation_time_;
    const double activation_yaw_rad = measured_yaw_enu_rad_.value_or(0.0);
    command_safety_limiter_.reset(activation_yaw_rad);
    publishAppliedYaw(activation_yaw_rad);
    publishAppliedCollective(kGravityMps2);
    activation_setpoint_sequence_ =
      latest_setpoint_ ? latest_setpoint_->sequence : 0;
    publishExternalModeState(true);
    RCLCPP_INFO(node_.get_logger(), "ACTIVATED: yaw_reset=%.3f rad",
                activation_yaw_rad);
  }

  void onDeactivate() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    activation_time_.reset();
    activation_setpoint_sequence_.reset();
    last_streamed_at_.reset();
    command_safety_limiter_.reset();
    publishAppliedYaw(0.0);
    publishAppliedCollective(0.0);
    publishExternalModeState(false);
    RCLCPP_INFO(node_.get_logger(), "DEACTIVATED");
  }

  void updateSetpoint(float dt_s) override
  {
    (void)dt_s;
    std::lock_guard<std::mutex> lock(mutex_);
    sendSetpoint();
  }

  void sendSetpoint()
  {
    if (!hasPostActivationSetpoint()) {
      RCLCPP_INFO_THROTTLE(
        node_.get_logger(), *node_.get_clock(), 1000,
        "Waiting for a post-activation TMPC setpoint; streaming level hover");
      publishLevelHoverSetpoint();
      return;
    }

    if (activationHoldActive()) {
      RCLCPP_INFO_THROTTLE(
        node_.get_logger(), *node_.get_clock(), 1000,
        "Holding level hover while the TMPC reference handover settles");
      publishLevelHoverSetpoint();
      return;
    }

    const auto now = node_.now();
    const double age = (now - *last_setpoint_time_).seconds();
    if (age > 0.5) {
      publishLevelHoverSetpoint();
      return;
    }

    // Convert desired quaternion from body FLU -> world ENU (ROS) to body FRD -> world NED (PX4)
    const auto &q_flu_enu_raw = latest_setpoint_->desired_attitude_wxyz;
    Eigen::Quaterniond q_flu_enu(q_flu_enu_raw[0], q_flu_enu_raw[1], q_flu_enu_raw[2], q_flu_enu_raw[3]);
    const double stream_interval_seconds = last_streamed_at_
      ? (now - *last_streamed_at_).seconds()
      : 0.0;
    const auto safe_command = command_safety_limiter_.limit(
      q_flu_enu, latest_setpoint_->desired_collective_specific_force_m_s2,
      stream_interval_seconds);
    if (!safe_command.valid) {
      RCLCPP_WARN_THROTTLE(
        node_.get_logger(), *node_.get_clock(), 1000,
        "Rejected MPC safety command; streaming level hover");
      publishLevelHoverSetpoint();
      return;
    }
    const auto safe_q_frd_ned_opt =
      mpc_controller::px4_control::fluEnuToFrdNed(
      safe_command.attitude_body_flu_to_world_enu);
    if (!safe_q_frd_ned_opt) {
      publishLevelHoverSetpoint();
      return;
    }
    const Eigen::Quaternionf q_frd_ned = safe_q_frd_ned_opt->cast<float>();
    const double requested_yaw_rad =
      mpc_controller::command_safety::yawAngleRad(q_flu_enu);
    const double applied_yaw_rad = mpc_controller::command_safety::yawAngleRad(
      safe_command.attitude_body_flu_to_world_enu);
    publishAppliedYaw(applied_yaw_rad);

    const double specific_force =
      safe_command.collective_specific_force_m_s2;
    const mpc_controller::px4_thrust::Mapping mapping{
      kGravityMps2, hover_thrust_};
    const auto thrust_body_z =
      mpc_controller::px4_thrust::specificForceToBodyFrdZ(specific_force, mapping);
    if (!thrust_body_z) {
      RCLCPP_WARN_THROTTLE(
        node_.get_logger(), *node_.get_clock(), 1000,
        "Rejected MPC thrust: specific_force=%.3f hover_thrust=%.3f; "
        "streaming level hover",
        specific_force, hover_thrust_);
      publishLevelHoverSetpoint();
      return;
    }

    // Body FRD Z thrust is negative (e.g. [0, 0, -T])
    const Eigen::Vector3f thrust_frd(
      0.f, 0.f, static_cast<float>(*thrust_body_z));

    attitude_setpoint_->update(q_frd_ned, thrust_frd);
    publishAppliedCollective(specific_force);
    publishAppliedSetpoint(
      now, safe_command.attitude_body_flu_to_world_enu, specific_force,
      *thrust_body_z, latest_setpoint_->recovery_active,
      safe_command.collective_limited);
    last_streamed_at_ = now;
    RCLCPP_INFO_THROTTLE(
      node_.get_logger(), *node_.get_clock(), 1000,
      "MPC setpoint streamed: recovery=%s requested_force=%.3f force=%.3f "
      "tilt_limited=%s force_limited=%s yaw_limited=%s hover_thrust=%.3f "
      "requested_yaw=%.3f yaw=%.3f thrust_frd_z=%.3f "
      "age_ms=%.1f",
      latest_setpoint_->recovery_active ? "yes" : "no",
      latest_setpoint_->desired_collective_specific_force_m_s2,
      specific_force, safe_command.tilt_limited ? "yes" : "no",
      safe_command.collective_limited ? "yes" : "no",
      safe_command.yaw_limited ? "yes" : "no", hover_thrust_,
      requested_yaw_rad, applied_yaw_rad,
      *thrust_body_z, age * 1.0e3);
  }

  bool hasPostActivationSetpoint() const noexcept
  {
    return activation_setpoint_sequence_ && latest_setpoint_ &&
      last_setpoint_time_ &&
      latest_setpoint_->sequence > *activation_setpoint_sequence_;
  }

  bool activationHoldActive() const noexcept
  {
    return activation_time_ &&
      (node_.now() - *activation_time_).seconds() <
        kReferenceHandoverHoldSeconds;
  }

  void publishLevelHoverSetpoint()
  {
    const auto q_level_frd_ned = mpc_controller::px4_control::fluEnuToFrdNed(
      command_safety_limiter_.levelAttitude());
    if (!q_level_frd_ned) {
      RCLCPP_ERROR_THROTTLE(
        node_.get_logger(), *node_.get_clock(), 1000,
        "Cannot convert level-hover attitude from ENU/FLU to NED/FRD");
      return;
    }
    const Eigen::Vector3f thrust_frd(
      0.f, 0.f, -static_cast<float>(hover_thrust_));
    attitude_setpoint_->update(q_level_frd_ned->cast<float>(), thrust_frd);
    publishAppliedYaw(mpc_controller::command_safety::yawAngleRad(
        command_safety_limiter_.levelAttitude()));
    publishAppliedCollective(kGravityMps2);
    publishAppliedSetpoint(
      node_.now(), command_safety_limiter_.levelAttitude(), kGravityMps2,
      -hover_thrust_, true, false);
    command_safety_limiter_.holdCollective();
    last_streamed_at_ = node_.now();
  }

  void publishExternalModeState(bool active)
  {
    std_msgs::msg::Bool state;
    state.data = active;
    external_mode_state_publisher_->publish(state);
  }

  void publishAppliedYaw(double yaw_rad)
  {
    if (!std::isfinite(yaw_rad)) {
      return;
    }
    std_msgs::msg::Float64 message;
    message.data = mpc_controller::command_safety::wrapAngleRad(yaw_rad);
    applied_yaw_publisher_->publish(message);
  }

  void publishAppliedCollective(double collective_m_s2)
  {
    if (!std::isfinite(collective_m_s2)) {
      return;
    }
    std_msgs::msg::Float64 message;
    message.data = collective_m_s2;
    applied_collective_publisher_->publish(message);
  }

  void publishAppliedSetpoint(
    const rclcpp::Time & timestamp, const Eigen::Quaterniond & attitude,
    double collective_m_s2, double thrust_body_frd_z, bool recovery_active,
    bool collective_limited)
  {
    if (!std::isfinite(collective_m_s2) || !std::isfinite(thrust_body_frd_z) ||
        !attitude.coeffs().allFinite() || attitude.norm() < 1.0e-9) {
      return;
    }
    const Eigen::Quaterniond normalized_attitude = attitude.normalized();
    const Eigen::Vector3d specific_force_world =
      normalized_attitude.toRotationMatrix().col(2) * collective_m_s2;
    mpc_controller::msg::ForceAttitudeSetpoint message;
    message.header.stamp = timestamp;
    message.header.frame_id = "map";
    message.sequence = ++applied_setpoint_sequence_;
    message.desired_specific_force_world_m_s2 = {
      specific_force_world.x(), specific_force_world.y(), specific_force_world.z()};
    message.desired_acceleration_m_s2 = {
      specific_force_world.x(), specific_force_world.y(),
      specific_force_world.z() - kGravityMps2};
    message.desired_attitude_wxyz = {
      normalized_attitude.w(), normalized_attitude.x(),
      normalized_attitude.y(), normalized_attitude.z()};
    message.desired_collective_specific_force_m_s2 = collective_m_s2;
    message.tilt_angle_rad =
      mpc_controller::command_safety::tiltAngleRad(normalized_attitude);
    message.recovery_active = recovery_active || collective_limited;
    applied_setpoint_publisher_->publish(message);
  }

  double parameter(const std::string & name, double fallback) const
  {
    double value = fallback;
    node_.get_parameter(name, value);
    return value;
  }

private:
  rclcpp::Node & node_;
  std::shared_ptr<px4_ros2::AttitudeSetpointType> attitude_setpoint_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr external_mode_state_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr applied_yaw_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr applied_collective_publisher_;
  rclcpp::Publisher<mpc_controller::msg::ForceAttitudeSetpoint>::SharedPtr
    applied_setpoint_publisher_;
  rclcpp::Subscription<mpc_controller::msg::ForceAttitudeSetpoint>::SharedPtr force_setpoint_sub_;
  rclcpp::Subscription<px4_msgs::msg::HoverThrustEstimate>::SharedPtr hover_thrust_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr vehicle_attitude_sub_;
  std::optional<mpc_controller::msg::ForceAttitudeSetpoint> latest_setpoint_;
  std::optional<rclcpp::Time> last_setpoint_time_;
  std::optional<rclcpp::Time> activation_time_;
  std::optional<std::uint64_t> activation_setpoint_sequence_;
  std::optional<rclcpp::Time> last_streamed_at_;
  std::optional<double> measured_yaw_enu_rad_;
  std::uint64_t applied_setpoint_sequence_ = 0;
  static constexpr double kGravityMps2 = 9.80665;
  static constexpr double kMinimumHoverThrust = 0.05;
  static constexpr double kMaximumHoverThrust = 0.95;
  static constexpr double kReferenceHandoverHoldSeconds = 0.5;
  double hover_thrust_ = 0.60;
  mpc_controller::command_safety::Limits command_safety_limits_;
  mpc_controller::command_safety::Limiter command_safety_limiter_;
  std::mutex mutex_;
};

class MpcModeExecutor : public px4_ros2::ModeExecutorBase
{
public:
  enum class State
  {
    Idle,
    Arming,
    TakingOff,
    MpcRunning,
    Landing,
    Disarming,
    Done
  };

  explicit MpcModeExecutor(px4_ros2::ModeBase & owned_mode)
  : ModeExecutorBase(px4_ros2::ModeExecutorBase::Settings{}, owned_mode),
      node_(owned_mode.node())
  {
    node_.get_parameter("px4_system_id", px4_system_id_);
    if (px4_system_id_ < kMinimumSystemId || px4_system_id_ > kMaximumSystemId) {
      RCLCPP_WARN(
        node_.get_logger(),
        "Invalid px4_system_id=%ld; using system ID %ld",
        px4_system_id_, kDefaultSystemId);
      px4_system_id_ = kDefaultSystemId;
    }
    node_.get_parameter("mission_timeout_seconds", mission_timeout_seconds_);
    if (!std::isfinite(mission_timeout_seconds_) || mission_timeout_seconds_ <= 0.0) {
      RCLCPP_WARN(
        node_.get_logger(),
        "Invalid mission_timeout_seconds=%.3f; using %.1f seconds",
        mission_timeout_seconds_, kDefaultMissionTimeoutSeconds);
      mission_timeout_seconds_ = kDefaultMissionTimeoutSeconds;
    }
    status_sub_ = node_.create_subscription<px4_msgs::msg::VehicleStatus>(
      "fmu/out/vehicle_status" +
      px4_ros2::getMessageNameVersion<px4_msgs::msg::VehicleStatus>(),
      rclcpp::QoS(1).best_effort(),
      [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
        if (!msg) {
          return;
        }
        {
          std::lock_guard<std::mutex> lock(feedback_mutex_);
          latest_status_ = *msg;
        }
        feedback_cv_.notify_all();
      });

    local_position_sub_ = node_.create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "fmu/out/vehicle_local_position" +
      px4_ros2::getMessageNameVersion<px4_msgs::msg::VehicleLocalPosition>(),
      rclcpp::QoS(1).best_effort(),
      [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
        if (!msg) {
          return;
        }
        {
          std::lock_guard<std::mutex> lock(feedback_mutex_);
          latest_local_position_ = *msg;
        }
        feedback_cv_.notify_all();
      });

    operator_command_publisher_ = node_.create_publisher<px4_msgs::msg::VehicleCommand>(
      "fmu/in/vehicle_command" +
      px4_ros2::getMessageNameVersion<px4_msgs::msg::VehicleCommand>(), rclcpp::QoS(1));
    mission_completed_sub_ = node_.create_subscription<std_msgs::msg::Bool>(
      "/reference_generator_node/mission_completed", rclcpp::QoS(10),
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg && msg->data) {
          mission_completed_.store(true);
          feedback_cv_.notify_all();
        }
      });

    start_service_ = node_.create_service<std_srvs::srv::Trigger>(
      "mpc_mode_executor/start",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        RCLCPP_INFO(node_.get_logger(), "[Executor] Start service requested");
        if (state_.load() != State::Idle && state_.load() != State::Done) {
          response->success = false;
          response->message = "Executor is currently active in state: " +
            std::to_string(static_cast<int>(state_.load()));
          return;
        }

        std::lock_guard<std::mutex> lock(sequence_mutex_);
        if (sequence_thread_.joinable()) {
          sequence_thread_.join();
        }
        stop_requested_.store(false);
        mission_completed_.store(false);
        land_requested_.store(false);
        state_.store(State::Arming);
        sequence_thread_ = std::thread(&MpcModeExecutor::runAutonomousSequence, this);
        response->success = true;
        response->message =
          "MpcModeExecutor started: Arming -> Takeoff -> wait for operator External Mode selection";
      });

    land_service_ = node_.create_service<std_srvs::srv::Trigger>(
      "mpc_mode_executor/land",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        RCLCPP_INFO(node_.get_logger(), "[Executor] Land service requested");
        land_requested_.store(true);
        feedback_cv_.notify_all();

        if (state_.load() == State::Idle || state_.load() == State::Done) {
          std::lock_guard<std::mutex> lock(sequence_mutex_);
          if (sequence_thread_.joinable()) {
            sequence_thread_.join();
          }
          stop_requested_.store(false);
          state_.store(State::Landing);
          sequence_thread_ = std::thread(&MpcModeExecutor::runLandingSequence, this);
        }
        response->success = true;
        response->message = "Landing sequence initiated";
      });
  }

  ~MpcModeExecutor() override
  {
    stop_requested_.store(true);
    feedback_cv_.notify_all();
    std::lock_guard<std::mutex> lock(sequence_mutex_);
    if (sequence_thread_.joinable()) {
      sequence_thread_.join();
    }
  }

  void onActivate() override
  {
    // PX4 transfers executor ownership only after the operator selects this mode.
    // Registration and heartbeat must never change the current PX4 navigation mode.
    RCLCPP_INFO(node_.get_logger(), "MpcModeExecutor activated and ready for start service");
  }

  void onDeactivate(DeactivateReason reason) override
  {
    (void)reason;
    RCLCPP_WARN(node_.get_logger(), "MpcModeExecutor deactivated");
    if (state_.load() == State::Landing) {
      return;
    }
    stop_requested_.store(true);
    state_.store(State::Idle);
    feedback_cv_.notify_all();
  }

private:
  static constexpr int64_t kMinimumSystemId = 1;
  static constexpr int64_t kMaximumSystemId = 255;
  static constexpr int64_t kDefaultSystemId = 2;
  static constexpr std::chrono::seconds kArmTimeout{10};
  static constexpr std::chrono::seconds kTakeoffTimeout{45};
  static constexpr std::chrono::seconds kExternalModeWaitTimeout{120};
  static constexpr double kDefaultMissionTimeoutSeconds = 300.0;
  static constexpr std::chrono::seconds kLandTimeout{60};
  static constexpr std::chrono::milliseconds kCommandRetryInterval{100};
  static constexpr std::chrono::seconds kCommandPublisherTimeout{3};
  static constexpr int kCommandAttempts = 3;

  bool waitForCondition(
    const std::function<bool()> &predicate, std::chrono::duration<double> timeout)
  {
    std::unique_lock<std::mutex> lock(feedback_mutex_);
    const bool completed = feedback_cv_.wait_for(
      lock, timeout,
      [this, &predicate] {return stop_requested_.load() || predicate();});
    return completed && !stop_requested_.load() && predicate();
  }

  bool waitForVehicleStatus(
    const std::function<bool(const px4_msgs::msg::VehicleStatus &)> &predicate,
    std::chrono::seconds timeout)
  {
    return waitForCondition(
      [this, &predicate] {
        return latest_status_.has_value() && predicate(*latest_status_);
      }, timeout);
  }

  bool waitForArmed(bool expected)
  {
    return waitForVehicleStatus(
      [expected](const px4_msgs::msg::VehicleStatus &status) {
        const bool armed = status.arming_state ==
          px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
        return armed == expected;
      }, kArmTimeout);
  }

  bool waitForTakeoffAltitude()
  {
    return waitForCondition(
      [this] {
        return latest_status_.has_value() && latest_local_position_.has_value() &&
               latest_status_->arming_state ==
               px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED &&
               latest_local_position_->z_valid && latest_local_position_->z <= -9.0f;
      }, kTakeoffTimeout);
  }

  bool waitForMissionOrLandRequest()
  {
    return waitForCondition(
      [this] {return mission_completed_.load() || land_requested_.load();},
      std::chrono::duration<double>(mission_timeout_seconds_));
  }

  bool publishCommandThrough(
    const rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr & publisher,
    uint16_t source_component,
    uint32_t command, float param1 = NAN, float param2 = NAN, float param3 = NAN,
    float param4 = NAN, float param5 = NAN, float param6 = NAN, float param7 = NAN)
  {
    px4_msgs::msg::VehicleCommand message{};
    message.command = command;
    message.param1 = param1;
    message.param2 = param2;
    message.param3 = param3;
    message.param4 = param4;
    message.param5 = param5;
    message.param6 = param6;
    message.param7 = param7;
    message.target_system = static_cast<uint8_t>(px4_system_id_);
    message.target_component = 1;
    message.source_system = static_cast<uint8_t>(px4_system_id_);
    message.source_component = source_component;
    message.from_external = true;
    message.timestamp = 0;

    const auto publisher_deadline =
      std::chrono::steady_clock::now() + kCommandPublisherTimeout;
    while (publisher->get_subscription_count() == 0 &&
           !stop_requested_.load() && std::chrono::steady_clock::now() < publisher_deadline) {
      std::this_thread::sleep_for(kCommandRetryInterval);
    }
    if (stop_requested_.load() || publisher->get_subscription_count() == 0) {
      RCLCPP_WARN(node_.get_logger(), "[Executor] PX4 command publisher is not connected");
      return false;
    }

    for (int attempt = 0; attempt < kCommandAttempts; ++attempt) {
      publisher->publish(message);
      if (attempt + 1 < kCommandAttempts) {
        std::this_thread::sleep_for(kCommandRetryInterval);
      }
    }
    return true;
  }

  bool publishOperatorCommand(
    uint32_t command, float param1 = NAN, float param2 = NAN, float param3 = NAN,
    float param4 = NAN, float param5 = NAN, float param6 = NAN, float param7 = NAN)
  {
    return publishCommandThrough(
      operator_command_publisher_, 1, command,
      param1, param2, param3, param4, param5, param6, param7);
  }

  bool sendArmAndWait()
  {
    if (!publishOperatorCommand(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f)) {
      return false;
    }
    return waitForArmed(true);
  }

  bool sendTakeoffAndWait()
  {
    if (!publishOperatorCommand(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF,
        NAN, NAN, NAN, NAN, NAN, NAN, 10.0f)) {
      return false;
    }
    return waitForTakeoffAltitude();
  }

  bool waitForExternalModeSelection()
  {
    return waitForVehicleStatus(
      [this](const px4_msgs::msg::VehicleStatus &status) {
        return status.nav_state == ownedMode().id() &&
               status.executor_in_charge == id();
      }, kExternalModeWaitTimeout);
  }

  bool sendLandAndWait()
  {
    if (!publishOperatorCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND)) {
      return false;
    }
    return waitForVehicleStatus(
      [](const px4_msgs::msg::VehicleStatus &status) {
        return status.arming_state ==
               px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED;
      }, kLandTimeout);
  }

  void reportSequenceFailure(const char *step)
  {
    if (stop_requested_.load()) {
      RCLCPP_WARN(node_.get_logger(), "[Executor] Sequence interrupted during %s", step);
    } else {
      RCLCPP_ERROR(node_.get_logger(), "[Executor] Sequence failed during %s", step);
    }
    state_.store(State::Idle);
  }

  void runAutonomousSequence()
  {
    RCLCPP_INFO(node_.get_logger(), "[Executor] Arming vehicle...");
    if (!sendArmAndWait()) {
      reportSequenceFailure("arming");
      return;
    }

    state_.store(State::TakingOff);
    RCLCPP_INFO(node_.get_logger(), "[Executor] Taking off to altitude 10.0m...");
    if (!sendTakeoffAndWait()) {
      reportSequenceFailure("takeoff");
      return;
    }

    state_.store(State::MpcRunning);
    RCLCPP_INFO(
      node_.get_logger(),
      "[Executor] Takeoff complete. Switching to External Mode (%d)...", ownedMode().id());
    // MAV_CMD_DO_SET_MODE: custom mode (1.0), AUTO (4.0), EXTERNAL1 (11.0)
    publishOperatorCommand(
      px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 4.0f, 11.0f);
    if (!waitForExternalModeSelection()) {
      reportSequenceFailure("External Mode selection");
      return;
    }

    RCLCPP_INFO(
      node_.get_logger(),
      "[Executor] External Mode selected by PX4. Holding until an explicit mission or land request");

    if (!waitForMissionOrLandRequest()) {
      reportSequenceFailure("mission wait");
      return;
    }

    runLandingSequence();
  }

  void runLandingSequence()
  {
    state_.store(State::Landing);
    RCLCPP_INFO(node_.get_logger(), "[Executor] Landing vehicle...");
    if (!sendLandAndWait()) {
      reportSequenceFailure("landing");
      return;
    }
    state_.store(State::Disarming);
    RCLCPP_INFO(node_.get_logger(), "[Executor] Vehicle disarmed. Autonomous sequence complete!");
    state_.store(State::Done);
  }

  std::atomic<State> state_{State::Idle};
  int64_t px4_system_id_{kDefaultSystemId};
  double mission_timeout_seconds_{kDefaultMissionTimeoutSeconds};
  std::atomic<bool> stop_requested_{false};
  std::atomic<bool> mission_completed_{false};
  std::atomic<bool> land_requested_{false};
  std::mutex feedback_mutex_;
  std::condition_variable feedback_cv_;
  std::optional<px4_msgs::msg::VehicleStatus> latest_status_;
  std::optional<px4_msgs::msg::VehicleLocalPosition> latest_local_position_;
  std::mutex sequence_mutex_;
  std::thread sequence_thread_;
  rclcpp::Node & node_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr operator_command_publisher_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mission_completed_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr land_service_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  while (rclcpp::ok()) {
    try {
      auto node = std::make_shared<px4_ros2::NodeWithModeExecutor<MpcModeExecutor, MpcFlightMode>>(
        kNodeName, false);
      RCLCPP_INFO(
        node->get_logger(), "MPC External Mode registered: nav_state=%u",
        static_cast<unsigned int>(node->getMode().id()));
      rclcpp::spin(node);
      break;
    } catch (const std::exception & e) {
      std::cerr << "px4_attitude_mode_node exception: " << e.what()
                << "; retrying registration" << std::endl;
      std::this_thread::sleep_for(std::chrono::seconds{1});
    }
  }
  rclcpp::shutdown();
  return 0;
}
