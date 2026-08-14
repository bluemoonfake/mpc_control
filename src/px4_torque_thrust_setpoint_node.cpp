#include "mpc_controller/msg/torque_thrust_setpoint_preview.hpp"
#include "mpc_controller/msg/force_attitude_setpoint.hpp"
#include "mpc_controller/msg/vehicle_state.hpp"
#include "mpc_controller/geometric_controller.hpp"

#include <px4_msgs/msg/hover_thrust_estimate.hpp>
#include <px4_msgs/msg/control_allocator_status.hpp>
#include <px4_msgs/msg/actuator_motors.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/timesync_status.hpp>
#include <px4_msgs/msg/vehicle_angular_velocity.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>
#include <px4_msgs/msg/vehicle_torque_setpoint.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

// Body-rate integral with conditional integration and per-axis anti-windup.
// The integrator accumulates e_omega in the FLU body frame and contributes
// tau_I = -K_I * integral(e_omega) dt to the SO(3) torque law, eliminating
// steady-state attitude trim offsets caused by CG shifts or asymmetric drag.
class RateIntegrator final
{
public:
  void configure(
    const Eigen::Vector3d &gain,
    const Eigen::Vector3d &limit) noexcept
  {
    gain_ = gain;
    limit_ = limit;
  }

  void reset() noexcept
  {
    integral_.setZero();
    last_update_.reset();
  }

  // Returns the integral torque contribution in FLU body frame.
  // Freezes accumulation when the actuator or torque output is saturated
  // (conditional integration anti-windup).
  Eigen::Vector3d update(
    const Eigen::Vector3d &rate_error,
    bool actuator_saturated,
    bool torque_saturated) noexcept
  {
    const auto current_time = std::chrono::steady_clock::now();
    if (!last_update_) {
      last_update_ = current_time;
      return -gain_.cwiseProduct(integral_);
    }
    const double dt = std::chrono::duration<double>(
      current_time - *last_update_).count();
    last_update_ = current_time;
    if (!std::isfinite(dt) || dt <= 0.0 || dt > 0.1) {
      // Reject large or invalid dt to avoid integration windup on timer jitter.
      return -gain_.cwiseProduct(integral_);
    }

    // Conditional integration: freeze when actuators are saturated.
    if (!actuator_saturated && !torque_saturated) {
      for (int axis = 0; axis < 3; ++axis) {
        integral_[axis] += rate_error[axis] * dt;
        integral_[axis] = std::clamp(
          integral_[axis], -limit_[axis], limit_[axis]);
      }
    }
    return -gain_.cwiseProduct(integral_);
  }

  const Eigen::Vector3d &integral() const noexcept {return integral_;}

private:
  Eigen::Vector3d gain_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d limit_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d integral_{Eigen::Vector3d::Zero()};
  std::optional<std::chrono::steady_clock::time_point> last_update_;
};

class Px4TorqueThrustSetpointNode final : public rclcpp::Node
{
public:
  Px4TorqueThrustSetpointNode()
  : Node("px4_torque_thrust_setpoint_node")
  {
    // Adapter parameters cover freshness, HTE thrust calibration, geometric
    // SO(3) gains/limits, and desired angular-kinematics estimation.
    declareAndGet("publish_rate_hz", publish_rate_hz_);
    declareAndGet("heartbeat_rate_hz", heartbeat_rate_hz_);
    declareAndGet("state_timeout_seconds", state_timeout_seconds_);
    declareAndGet("command_timeout_seconds", command_timeout_seconds_);
    declareAndGet("timesync_timeout_seconds", timesync_timeout_seconds_);
    declareAndGet("hover_thrust_estimate_timeout_seconds", hover_thrust_estimate_timeout_seconds_);
    declareAndGet("hover_thrust_tracking_time_constant_seconds", hover_thrust_tracking_time_constant_seconds_);
    declareAndGet("hover_thrust_max_rate_per_second", hover_thrust_max_rate_per_second_);
    declareAndGet("hover_thrust_normalized", thrust_mapping_.hover_thrust_normalized);
    declareAndGet("enable_desired_rate_tracking", enable_desired_rate_tracking_);
    declareAndGet("enable_dynamics_compensation", torque_parameters_.enable_dynamics_compensation);
    declareAndGet(
      "desired_kinematics_filter_time_constant_seconds",
      desired_kinematics_parameters_.filter_time_constant_seconds);
    declareAndGet(
      "desired_kinematics_max_sample_interval_seconds",
      desired_kinematics_parameters_.max_sample_interval_seconds);

    // Vector parameters defaults
    declare_parameter("attitude_gain", std::vector<double>{0.8, 0.8, 0.4});
    declare_parameter("rate_gain", std::vector<double>{0.15, 0.15, 0.10});
    declare_parameter("normalized_inertia", std::vector<double>{0.0, 0.0, 0.0});
    declare_parameter("max_desired_body_rate_rad_s", std::vector<double>{2.0, 2.0, 1.0});
    declare_parameter(
      "max_desired_body_acceleration_rad_s2", std::vector<double>{10.0, 10.0, 5.0});
    declare_parameter("normalized_torque_limit", std::vector<double>{0.30, 0.30, 0.20});
    declare_parameter("rate_integral_gain", std::vector<double>{0.0, 0.0, 0.0});
    declare_parameter("rate_integral_limit", std::vector<double>{0.15, 0.15, 0.10});

    const bool torque_parameters_loaded =
      getVector3Parameter("attitude_gain", torque_parameters_.attitude_gain)
      && getVector3Parameter("rate_gain", torque_parameters_.rate_gain)
      && getVector3Parameter("normalized_inertia", torque_parameters_.normalized_inertia)
      && getVector3Parameter(
        "max_desired_body_rate_rad_s", desired_kinematics_parameters_.max_body_rate_rad_s)
      && getVector3Parameter(
        "max_desired_body_acceleration_rad_s2",
        desired_kinematics_parameters_.max_body_acceleration_rad_s2)
      && getVector3Parameter("normalized_torque_limit", torque_parameters_.normalized_limit)
      && getVector3Parameter("rate_integral_gain", torque_parameters_.rate_integral_gain)
      && getVector3Parameter("rate_integral_limit", torque_parameters_.rate_integral_limit);

    config_valid_ = std::isfinite(publish_rate_hz_) && publish_rate_hz_ > 0.0
      && std::isfinite(heartbeat_rate_hz_) && heartbeat_rate_hz_ > 0.0
      && std::isfinite(state_timeout_seconds_) && state_timeout_seconds_ > 0.0
      && std::isfinite(command_timeout_seconds_) && command_timeout_seconds_ > 0.0
      && std::isfinite(timesync_timeout_seconds_) && timesync_timeout_seconds_ > 0.0
      && std::isfinite(hover_thrust_estimate_timeout_seconds_)
      && hover_thrust_estimate_timeout_seconds_ > 0.0
      && std::isfinite(hover_thrust_tracking_time_constant_seconds_)
      && hover_thrust_tracking_time_constant_seconds_ > 0.0
      && std::isfinite(hover_thrust_max_rate_per_second_)
      && hover_thrust_max_rate_per_second_ > 0.0
      && torque_parameters_loaded
      && mpc_controller::px4_control::validTorqueParameters(torque_parameters_)
      && mpc_controller::px4_control::validDesiredKinematicsParameters(
        desired_kinematics_parameters_)
      && mpc_controller::px4_thrust::valid(thrust_mapping_);
    if (!config_valid_) {
      RCLCPP_ERROR(get_logger(), "Invalid SITL torque/thrust adapter configuration");
    }
    desired_kinematics_estimator_.emplace(desired_kinematics_parameters_);
    rate_integrator_.configure(
      torque_parameters_.rate_integral_gain, torque_parameters_.rate_integral_limit);

    // PX4 sensor/output topics use best-effort QoS. Callbacks only cache data;
    // the timers below are the sole heartbeat and wrench publishers.
    const auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    setpoint_subscription_ = create_subscription<Setpoint>(
      "force_attitude_setpoint", 10,
      [this](Setpoint::SharedPtr message) {
        if (!message) return;
        std::lock_guard<std::mutex> lock(mutex_);
        setpoint_ = *message;
        setpoint_received_at_ = Clock::now();
      });
    vehicle_state_subscription_ = create_subscription<VehicleState>(
      "vehicle_state", 10,
      [this](VehicleState::SharedPtr message) {
        if (!message) return;
        std::lock_guard<std::mutex> lock(mutex_);
        vehicle_state_ = *message;
      });
    attitude_subscription_ = create_subscription<Px4Attitude>(
      "fmu/out/vehicle_attitude", sensor_qos,
      [this](Px4Attitude::SharedPtr message) {
        if (!message || message->timestamp_sample == 0U) return;
        const Eigen::Quaterniond q_frd_ned(
          message->q[0], message->q[1], message->q[2], message->q[3]);
        const auto converted = mpc_controller::px4_control::frdNedToFluEnu(q_frd_ned);
        if (!converted) return;
        std::lock_guard<std::mutex> lock(mutex_);
        measured_attitude_flu_enu_ = *converted;
        attitude_received_at_ = Clock::now();
      });
    angular_velocity_subscription_ = create_subscription<Px4AngularVelocity>(
      "fmu/out/vehicle_angular_velocity", sensor_qos,
      [this](Px4AngularVelocity::SharedPtr message) {
        if (!message || message->timestamp_sample == 0U
          || !std::isfinite(message->xyz[0]) || !std::isfinite(message->xyz[1])
          || !std::isfinite(message->xyz[2])) return;
        std::lock_guard<std::mutex> lock(mutex_);
        measured_body_rate_flu_ = Eigen::Vector3d(
          message->xyz[0], -message->xyz[1], -message->xyz[2]);
        body_rate_received_at_ = Clock::now();
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
        // PX4 may briefly clear `valid` while the numerical estimate remains
        // continuous. Retain only the latest admissible sample so a short
        // validity dropout cannot force an Offboard handover onto the fallback.
        if (!message || !message->valid
          || !std::isfinite(message->hover_thrust)
          || message->hover_thrust <= 0.0F || message->hover_thrust > 1.0F
          || !std::isfinite(message->hover_thrust_var)
          || message->hover_thrust_var < 0.0F) {
          return;
        }
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
          resetDesiredKinematics();
          rate_integrator_.reset();
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
          resetDesiredKinematics();
          rate_integrator_.reset();
          yaw_hold_enu_rad_.reset();
          latched_hover_thrust_normalized_.reset();
          latched_hover_thrust_from_estimator_ = false;
          hover_thrust_tracking_updated_at_.reset();
          state_ = State::offboard_lost;
          RCLCPP_WARN(get_logger(), "PX4 Offboard lost; torque/thrust publication stopped");
        }
      });
    allocator_status_subscription_ = create_subscription<AllocatorStatus>(
      "fmu/out/control_allocator_status", sensor_qos,
      [this](AllocatorStatus::SharedPtr message) {
        if (!message || message->timestamp == 0U) return;
        std::lock_guard<std::mutex> lock(mutex_);
        allocator_status_ = *message;
        allocator_status_received_at_ = Clock::now();
      });
    actuator_motors_subscription_ = create_subscription<ActuatorMotors>(
      "fmu/out/actuator_motors", sensor_qos,
      [this](ActuatorMotors::SharedPtr message) {
        if (!message || message->timestamp == 0U) return;
        std::lock_guard<std::mutex> lock(mutex_);
        actuator_motors_ = *message;
        actuator_motors_received_at_ = Clock::now();
      });

    thrust_publisher_ = create_publisher<Thrust>("fmu/in/vehicle_thrust_setpoint", sensor_qos);
    torque_publisher_ = create_publisher<Torque>("fmu/in/vehicle_torque_setpoint", sensor_qos);
    preview_publisher_ = create_publisher<Preview>("px4_torque_thrust_setpoint_preview", 10);
    heartbeat_publisher_ = create_publisher<Offboard>(
      "fmu/in/offboard_control_mode", sensor_qos);

    setpoint_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / publish_rate_hz_)),
      [this]() {publishTorqueThrustSetpoint();});
    heartbeat_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / heartbeat_rate_hz_)),
      [this]() {publishHeartbeat();});

    RCLCPP_INFO(
      get_logger(),
      "SITL-first torque/thrust adapter ready: WAIT_DATA -> PRESTREAM -> ACTIVE; "
      "geometric SO(3) at %.1f Hz; desired-rate=%s dynamics=%s; "
      "no arm or mode command is sent",
      publish_rate_hz_, enable_desired_rate_tracking_ ? "on" : "off",
      torque_parameters_.enable_dynamics_compensation ? "on" : "off");
  }

private:
  using Setpoint = mpc_controller::msg::ForceAttitudeSetpoint;
  using VehicleState = mpc_controller::msg::VehicleState;
  using Preview = mpc_controller::msg::TorqueThrustSetpointPreview;
  using Timesync = px4_msgs::msg::TimesyncStatus;
  using HoverThrustEstimate = px4_msgs::msg::HoverThrustEstimate;
  using Px4Attitude = px4_msgs::msg::VehicleAttitude;
  using Px4AngularVelocity = px4_msgs::msg::VehicleAngularVelocity;
  using Thrust = px4_msgs::msg::VehicleThrustSetpoint;
  using Torque = px4_msgs::msg::VehicleTorqueSetpoint;
  using Offboard = px4_msgs::msg::OffboardControlMode;
  using ControlMode = px4_msgs::msg::VehicleControlMode;
  using AllocatorStatus = px4_msgs::msg::ControlAllocatorStatus;
  using ActuatorMotors = px4_msgs::msg::ActuatorMotors;
  using Clock = std::chrono::steady_clock;

  template <typename T>
  void declareAndGet(const std::string &name, T &target)
  {
    declare_parameter(name, target);
    get_parameter(name, target);
  }

  enum class State : uint8_t
  {
    wait_data = Preview::STATE_WAIT_DATA,
    prestream = Preview::STATE_PRESTREAM,
    active = Preview::STATE_ACTIVE,
    offboard_lost = Preview::STATE_OFFBOARD_LOST,
  };

  struct Snapshot
  {
    Setpoint setpoint{};
    VehicleState vehicle{};
    Eigen::Quaterniond measured_attitude_flu_enu{Eigen::Quaterniond::Identity()};
    Eigen::Vector3d measured_body_rate_flu{};
    uint64_t px4_timestamp = 0U;
    double attitude_age_seconds = std::numeric_limits<double>::quiet_NaN();
    double body_rate_age_seconds = std::numeric_limits<double>::quiet_NaN();
    double measured_yaw_enu_rad = std::numeric_limits<double>::quiet_NaN();
    double setpoint_age_seconds = std::numeric_limits<double>::quiet_NaN();
    double timesync_age_seconds = std::numeric_limits<double>::quiet_NaN();
    double yaw_command_enu_rad = std::numeric_limits<double>::quiet_NaN();
    double hover_thrust_estimate = std::numeric_limits<double>::quiet_NaN();
    double hover_thrust_estimate_age_seconds = std::numeric_limits<double>::quiet_NaN();
    double active_hover_thrust_normalized = std::numeric_limits<double>::quiet_NaN();
    std::array<float, 3> unallocated_torque{};
    std::array<float, 3> unallocated_thrust{};
    bool attitude_fresh = false;
    bool body_rate_fresh = false;
    bool setpoint_fresh = false;
    bool timesync_fresh = false;
    bool offboard_active = false;
    bool yaw_hold_valid = false;
    bool hover_thrust_estimate_valid = false;
    bool hover_thrust_estimate_fresh = false;
    bool active_hover_thrust_from_estimator = false;
    bool allocator_status_fresh = false;
    bool torque_setpoint_achieved = false;
    bool thrust_setpoint_achieved = false;
    bool actuator_saturated = false;

    bool ready() const noexcept
    {
      return attitude_fresh && body_rate_fresh && setpoint_fresh && timesync_fresh;
    }
  };

  static bool freshAge(double age_seconds, double timeout_seconds) noexcept
  {
    return std::isfinite(age_seconds) && age_seconds >= 0.0
      && age_seconds <= timeout_seconds;
  }

  bool getVector3Parameter(const std::string &name, Eigen::Vector3d &output)
  {
    const auto values = get_parameter(name).as_double_array();
    if (values.size() != 3U) {
      RCLCPP_ERROR(get_logger(), "Parameter '%s' must contain exactly 3 values", name.c_str());
      return false;
    }
    output = {values[0], values[1], values[2]};
    return output.allFinite();
  }

  void captureYawLocked()
  {
    if (measured_attitude_flu_enu_) {
      const Eigen::Matrix3d rotation = measured_attitude_flu_enu_->toRotationMatrix();
      const double yaw = std::atan2(rotation(1, 0), rotation(0, 0));
      if (std::isfinite(yaw)) {
        yaw_hold_enu_rad_ = yaw;
        return;
      }
    }
    if (vehicle_state_ && std::isfinite(vehicle_state_->yaw)) {
      yaw_hold_enu_rad_ = vehicle_state_->yaw;
    } else {
      yaw_hold_enu_rad_.reset();
    }
  }

  void resetDesiredKinematics()
  {
    std::lock_guard<std::mutex> lock(kinematics_mutex_);
    if (desired_kinematics_estimator_) desired_kinematics_estimator_->reset();
    desired_kinematics_ = {};
    desired_kinematics_sequence_ = 0U;
    rate_integrator_.reset();
  }

  mpc_controller::px4_control::DesiredKinematics updateDesiredKinematics(
    const Eigen::Quaterniond &desired_attitude, const Setpoint &setpoint)
  {
    std::lock_guard<std::mutex> lock(kinematics_mutex_);
    if (!enable_desired_rate_tracking_ || !desired_kinematics_estimator_) {
      desired_kinematics_ = {};
      return desired_kinematics_;
    }
    if (setpoint.sequence == desired_kinematics_sequence_) return desired_kinematics_;
    desired_kinematics_sequence_ = setpoint.sequence;
    const rclcpp::Time stamp(setpoint.header.stamp);
    if (stamp.nanoseconds() <= 0) {
      desired_kinematics_estimator_->reset();
      desired_kinematics_ = {};
      return desired_kinematics_;
    }
    desired_kinematics_ = desired_kinematics_estimator_->update(
      desired_attitude, stamp.seconds());
    return desired_kinematics_;
  }

  bool hoverThrustEstimateUsableLocked(Clock::time_point current_time) const noexcept
  {
    if (!hover_thrust_estimate_ || !hover_thrust_estimate_received_at_) return false;
    const double age = std::chrono::duration<double>(
      current_time - *hover_thrust_estimate_received_at_).count();
    return freshAge(age, hover_thrust_estimate_timeout_seconds_)
      && hover_thrust_estimate_->valid
      && std::isfinite(hover_thrust_estimate_->hover_thrust)
      && hover_thrust_estimate_->hover_thrust > 0.0F
      && hover_thrust_estimate_->hover_thrust <= 1.0F
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
    hover_thrust_tracking_updated_at_ = Clock::now();
  }

  void updateActiveHoverThrustLocked(Clock::time_point current_time)
  {
    if (!offboard_active_ || !latched_hover_thrust_normalized_
      || !hoverThrustEstimateUsableLocked(current_time)) {
      return;
    }
    if (!hover_thrust_tracking_updated_at_) {
      hover_thrust_tracking_updated_at_ = current_time;
      return;
    }
    const double dt = std::chrono::duration<double>(
      current_time - *hover_thrust_tracking_updated_at_).count();
    hover_thrust_tracking_updated_at_ = current_time;
    if (!std::isfinite(dt) || dt <= 0.0) return;

    // Do not turn a callback or estimator outage into a large catch-up step.
    const double bounded_dt = std::min(dt, 0.1);
    const double target = hover_thrust_estimate_->hover_thrust;
    const double alpha = 1.0 - std::exp(
      -bounded_dt / hover_thrust_tracking_time_constant_seconds_);
    const double rate_limited_delta = std::clamp(
      alpha * (target - *latched_hover_thrust_normalized_),
      -hover_thrust_max_rate_per_second_ * bounded_dt,
      hover_thrust_max_rate_per_second_ * bounded_dt);
    *latched_hover_thrust_normalized_ = std::clamp(
      *latched_hover_thrust_normalized_ + rate_limited_delta,
      std::numeric_limits<double>::epsilon(), 1.0);
    latched_hover_thrust_from_estimator_ = true;
  }

  Snapshot snapshot()
  {
    // Build one time-consistent decision snapshot and derive every freshness
    // flag from steady-clock receipt age rather than mixed ROS/PX4 clocks.
    Snapshot output;
    const auto current_time = Clock::now();
    std::lock_guard<std::mutex> lock(mutex_);
    if (setpoint_ && setpoint_received_at_) {
      output.setpoint = *setpoint_;
      output.setpoint_age_seconds = std::chrono::duration<double>(
        current_time - *setpoint_received_at_).count();
      output.setpoint_fresh = freshAge(output.setpoint_age_seconds, command_timeout_seconds_);
    }
    if (vehicle_state_) output.vehicle = *vehicle_state_;
    if (measured_attitude_flu_enu_ && attitude_received_at_) {
      output.measured_attitude_flu_enu = *measured_attitude_flu_enu_;
      const Eigen::Matrix3d rotation = output.measured_attitude_flu_enu.toRotationMatrix();
      output.measured_yaw_enu_rad = std::atan2(rotation(1, 0), rotation(0, 0));
      output.attitude_age_seconds = std::chrono::duration<double>(
        current_time - *attitude_received_at_).count();
      output.attitude_fresh = freshAge(output.attitude_age_seconds, state_timeout_seconds_);
    }
    if (measured_body_rate_flu_ && body_rate_received_at_) {
      output.measured_body_rate_flu = *measured_body_rate_flu_;
      output.body_rate_age_seconds = std::chrono::duration<double>(
        current_time - *body_rate_received_at_).count();
      output.body_rate_fresh = freshAge(output.body_rate_age_seconds, state_timeout_seconds_);
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
      output.hover_thrust_estimate_age_seconds = std::chrono::duration<double>(
        current_time - *hover_thrust_estimate_received_at_).count();
      output.hover_thrust_estimate_fresh = freshAge(
        output.hover_thrust_estimate_age_seconds, hover_thrust_estimate_timeout_seconds_);
      output.hover_thrust_estimate_valid = hoverThrustEstimateUsableLocked(current_time);
    }
    if (allocator_status_ && allocator_status_received_at_) {
      const double allocator_age = std::chrono::duration<double>(
        current_time - *allocator_status_received_at_).count();
      output.allocator_status_fresh = freshAge(allocator_age, state_timeout_seconds_);
      output.torque_setpoint_achieved = allocator_status_->torque_setpoint_achieved;
      output.thrust_setpoint_achieved = allocator_status_->thrust_setpoint_achieved;
      output.unallocated_torque = allocator_status_->unallocated_torque;
      output.unallocated_thrust = allocator_status_->unallocated_thrust;
      if (actuator_motors_ && actuator_motors_received_at_) {
        const double motors_age = std::chrono::duration<double>(
          current_time - *actuator_motors_received_at_).count();
        if (freshAge(motors_age, state_timeout_seconds_)) {
          // PX4 exposes 16 saturation slots but only publishes finite controls
          // for configured motors. Ignore unused slots so this is portable
          // across quad, hexa, octo, and other allocator geometries.
          for (std::size_t index = 0; index < actuator_motors_->control.size(); ++index) {
            if (std::isfinite(actuator_motors_->control[index])
              && allocator_status_->actuator_saturation[index]
              != AllocatorStatus::ACTUATOR_SATURATION_OK) {
              output.actuator_saturated = true;
              break;
            }
          }
        }
      }
    }
    updateActiveHoverThrustLocked(current_time);
    output.offboard_active = offboard_active_;
    output.yaw_hold_valid = yaw_hold_enu_rad_.has_value();
    output.yaw_command_enu_rad = yaw_hold_enu_rad_.value_or(
      std::isfinite(output.measured_yaw_enu_rad)
      ? output.measured_yaw_enu_rad : output.vehicle.yaw);
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
    // Freshness-driven state machine; it never arms or changes PX4 mode.
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
    heartbeat.thrust_and_torque = true;
    heartbeat_publisher_->publish(heartbeat);
  }

  void publishTorqueThrustSetpoint()
  {
    // ACTIVE uses trajectory yaw from the setpoint. PRESTREAM replaces it with
    // captured heading to avoid a handover jump, then SO(3) produces wrench.
    auto sample = snapshot();
    if (!updateState(sample)) {
      publishPreview(sample, {});
      return;
    }

    // updateState may latch yaw after the snapshot was taken.
    sample = snapshot();
    mpc_controller::px4_control::Output converted;
    const Eigen::Quaterniond setpoint_attitude(
      sample.setpoint.desired_attitude_wxyz[0], sample.setpoint.desired_attitude_wxyz[1],
      sample.setpoint.desired_attitude_wxyz[2], sample.setpoint.desired_attitude_wxyz[3]);
    std::optional<Eigen::Quaterniond> desired_attitude;
    if (sample.offboard_active
      && mpc_controller::px4_control::finiteQuaternion(setpoint_attitude)
      && setpoint_attitude.norm() > 1.0e-9) {
      // ACTIVE follows the trajectory yaw embedded in the desired attitude.
      desired_attitude = setpoint_attitude.normalized();
    } else {
      // PRESTREAM retains the measured/captured yaw for a continuous handover.
      desired_attitude = mpc_controller::px4_control::withEnuYaw(
        setpoint_attitude, sample.yaw_command_enu_rad);
    }
    mpc_controller::px4_control::DesiredKinematics desired_kinematics;
    if (desired_attitude) {
      desired_kinematics = updateDesiredKinematics(*desired_attitude, sample.setpoint);
      mpc_controller::px4_control::Input input;
      input.desired_body_flu_to_world_enu = *desired_attitude;
      input.measured_body_flu_to_world_enu = sample.measured_attitude_flu_enu;
      input.desired_body_rate_flu_rad_s = desired_kinematics.body_rate_rad_s;
      input.desired_body_acceleration_flu_rad_s2 =
        desired_kinematics.body_acceleration_rad_s2;
      input.measured_body_rate_flu_rad_s = sample.measured_body_rate_flu;
      input.desired_collective_specific_force_m_s2 =
        sample.setpoint.desired_collective_specific_force_m_s2;
      input.valid = true;
      // VehicleThrustSetpoint is a normalized PX4 control coordinate.
      auto active_thrust_mapping = thrust_mapping_;
      active_thrust_mapping.hover_thrust_normalized = sample.active_hover_thrust_normalized;
      converted = mpc_controller::px4_control::convert(
        input, active_thrust_mapping, torque_parameters_);
    }

    publishPreview(sample, converted, desired_kinematics.valid);
    if (!converted.valid) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "SITL torque/thrust computation invalid; state remains freshness-driven (failure=%u)",
        static_cast<unsigned>(converted.failure_reason));
      return;
    }

    // Rate-error integral torque: tau_I = -K_I * integral(e_omega) in FLU.
    // Conditional integration freezes when actuators or torque are saturated.
    const Eigen::Vector3d rate_error_flu(
      converted.body_rate_error_rad_s[0],
      converted.body_rate_error_rad_s[1],
      converted.body_rate_error_rad_s[2]);
    const Eigen::Vector3d integral_torque_flu = rate_integrator_.update(
      rate_error_flu, sample.actuator_saturated, converted.torque_saturated);

    // Add integral contribution in FRD frame: FLU→FRD is diag(1, -1, -1).
    converted.torque_body_frd[0] += static_cast<float>(integral_torque_flu.x());
    converted.torque_body_frd[1] += static_cast<float>(-integral_torque_flu.y());
    converted.torque_body_frd[2] += static_cast<float>(-integral_torque_flu.z());

    // Re-clamp combined (PD+I) torque to normalized limit.
    for (int axis = 0; axis < 3; ++axis) {
      const float limit = static_cast<float>(torque_parameters_.normalized_limit[axis]);
      converted.torque_body_frd[axis] = std::clamp(
        converted.torque_body_frd[axis], -limit, limit);
    }

    // PX4's allocator is scheduled by the torque update, so publish thrust
    // first to make the matching collective command visible to that cycle.
    Thrust thrust{};
    thrust.timestamp = sample.px4_timestamp;
    thrust.timestamp_sample = sample.px4_timestamp;
    thrust.xyz = converted.thrust_body_frd;
    thrust_publisher_->publish(thrust);

    Torque torque{};
    torque.timestamp = sample.px4_timestamp;
    torque.timestamp_sample = sample.px4_timestamp;
    torque.xyz = converted.torque_body_frd;
    torque_publisher_->publish(torque);
  }

  void publishPreview(
    const Snapshot &sample, const mpc_controller::px4_control::Output &converted,
    bool desired_kinematics_valid = false)
  {
    Preview preview{};
    preview.header.stamp = now();
    preview.q_d_wxyz = converted.q_d_wxyz;
    preview.torque_body_frd = converted.torque_body_frd;
    preview.thrust_body_frd = converted.thrust_body_frd;
    preview.attitude_error = converted.attitude_error;
    preview.body_rate_error_rad_s = converted.body_rate_error_rad_s;
    preview.desired_body_rate_flu_rad_s = converted.desired_body_rate_flu_rad_s;
    preview.desired_body_acceleration_flu_rad_s2 =
      converted.desired_body_acceleration_flu_rad_s2;
    preview.feedback_torque_body_flu_normalized =
      converted.feedback_torque_body_flu_normalized;
    preview.dynamics_torque_body_flu_normalized =
      converted.dynamics_torque_body_flu_normalized;
    preview.desired_kinematics_valid = desired_kinematics_valid;
    preview.torque_saturated = converted.torque_saturated;
    preview.mpc_recovery_active = sample.setpoint.recovery_active;
    preview.allocator_status_fresh = sample.allocator_status_fresh;
    preview.torque_setpoint_achieved = sample.torque_setpoint_achieved;
    preview.thrust_setpoint_achieved = sample.thrust_setpoint_achieved;
    preview.actuator_saturated = sample.actuator_saturated;
    preview.unallocated_torque = sample.unallocated_torque;
    preview.unallocated_thrust = sample.unallocated_thrust;
    preview.valid = converted.valid;
    preview.state = static_cast<uint8_t>(stateValue());
    preview.attitude_fresh = sample.attitude_fresh;
    preview.body_rate_fresh = sample.body_rate_fresh;
    preview.setpoint_fresh = sample.setpoint_fresh;
    preview.timesync_fresh = sample.timesync_fresh;
    preview.yaw_hold_valid = sample.yaw_hold_valid;
    preview.yaw_hold_enu_rad = sample.yaw_command_enu_rad;
    preview.failure_reason = static_cast<uint8_t>(converted.failure_reason);
    preview.hover_thrust_estimate = static_cast<float>(sample.hover_thrust_estimate);
    preview.active_hover_thrust_normalized = sample.active_hover_thrust_normalized;
    preview.active_hover_thrust_from_estimator = sample.active_hover_thrust_from_estimator;
    preview_publisher_->publish(preview);
  }

  mutable std::mutex mutex_;
  bool config_valid_ = false;
  bool offboard_active_ = false;
  bool enable_desired_rate_tracking_ = true;
  State state_ = State::wait_data;
  double publish_rate_hz_ = 250.0;
  double heartbeat_rate_hz_ = 50.0;
  double state_timeout_seconds_ = 0.25;
  double command_timeout_seconds_ = 0.25;
  double timesync_timeout_seconds_ = 1.5;
  double hover_thrust_estimate_timeout_seconds_ = 1.0;
  double hover_thrust_tracking_time_constant_seconds_ = 2.0;
  double hover_thrust_max_rate_per_second_ = 0.02;
  mpc_controller::px4_thrust::Mapping thrust_mapping_{};
  mpc_controller::px4_control::TorqueParameters torque_parameters_{};
  mpc_controller::px4_control::DesiredKinematicsParameters desired_kinematics_parameters_{};
  std::optional<mpc_controller::px4_control::DesiredKinematicsEstimator>
  desired_kinematics_estimator_;
  mpc_controller::px4_control::DesiredKinematics desired_kinematics_{};
  uint64_t desired_kinematics_sequence_ = 0U;
  std::mutex kinematics_mutex_;
  RateIntegrator rate_integrator_;
  std::optional<double> yaw_hold_enu_rad_;
  std::optional<double> latched_hover_thrust_normalized_;
  bool latched_hover_thrust_from_estimator_ = false;
  std::optional<Setpoint> setpoint_;
  std::optional<VehicleState> vehicle_state_;
  std::optional<Eigen::Quaterniond> measured_attitude_flu_enu_;
  std::optional<Eigen::Vector3d> measured_body_rate_flu_;
  std::optional<HoverThrustEstimate> hover_thrust_estimate_;
  std::optional<AllocatorStatus> allocator_status_;
  std::optional<ActuatorMotors> actuator_motors_;
  std::optional<Clock::time_point> setpoint_received_at_;
  std::optional<Clock::time_point> attitude_received_at_;
  std::optional<Clock::time_point> body_rate_received_at_;
  std::optional<Clock::time_point> hover_thrust_estimate_received_at_;
  std::optional<Clock::time_point> allocator_status_received_at_;
  std::optional<Clock::time_point> actuator_motors_received_at_;
  std::optional<Clock::time_point> hover_thrust_tracking_updated_at_;
  std::optional<uint64_t> timesync_timestamp_;
  std::optional<Clock::time_point> timesync_received_at_;

  rclcpp::Subscription<Setpoint>::SharedPtr setpoint_subscription_;
  rclcpp::Subscription<VehicleState>::SharedPtr vehicle_state_subscription_;
  rclcpp::Subscription<Px4Attitude>::SharedPtr attitude_subscription_;
  rclcpp::Subscription<Px4AngularVelocity>::SharedPtr angular_velocity_subscription_;
  rclcpp::Subscription<Timesync>::SharedPtr timesync_subscription_;
  rclcpp::Subscription<HoverThrustEstimate>::SharedPtr hover_thrust_estimate_subscription_;
  rclcpp::Subscription<ControlMode>::SharedPtr control_mode_subscription_;
  rclcpp::Subscription<AllocatorStatus>::SharedPtr allocator_status_subscription_;
  rclcpp::Subscription<ActuatorMotors>::SharedPtr actuator_motors_subscription_;
  rclcpp::Publisher<Thrust>::SharedPtr thrust_publisher_;
  rclcpp::Publisher<Torque>::SharedPtr torque_publisher_;
  rclcpp::Publisher<Preview>::SharedPtr preview_publisher_;
  rclcpp::Publisher<Offboard>::SharedPtr heartbeat_publisher_;
  rclcpp::TimerBase::SharedPtr setpoint_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Px4TorqueThrustSetpointNode>());
  rclcpp::shutdown();
  return 0;
}
