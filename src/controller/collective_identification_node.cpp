#include "mpc_controller/controller/collective_force_filter.hpp"
#include "mpc_controller/msg/force_attitude_setpoint.hpp"
#include "mpc_controller/msg/vehicle_state.hpp"

#include <px4_msgs/msg/actuator_motors.hpp>
#include <px4_msgs/msg/hover_thrust_estimate.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <mutex>
#include <numeric>
#include <optional>
#include <string>
#include <utility>

namespace {

using ForceAttitudeSetpoint = mpc_controller::msg::ForceAttitudeSetpoint;
using VehicleState = mpc_controller::msg::VehicleState;
using VehicleThrustSetpoint = px4_msgs::msg::VehicleThrustSetpoint;
using ActuatorMotors = px4_msgs::msg::ActuatorMotors;

constexpr double kGravityMps2 = 9.80665;
constexpr std::size_t kLoggedMotorCount = 4;

double tiltAngle(const VehicleState & state) noexcept
{
  const Eigen::Quaterniond attitude(
    state.attitude[0], state.attitude[1], state.attitude[2], state.attitude[3]);
  if (!std::isfinite(attitude.norm()) || attitude.norm() < 1.0e-9) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return std::acos(std::clamp(
    attitude.normalized().toRotationMatrix()(2, 2), -1.0, 1.0));
}

bool measuredCollective(
  const VehicleState & state, double & collective) noexcept
{
  if (!state.valid || !state.attitude_valid || !state.acceleration_valid ||
      !std::all_of(state.attitude.begin(), state.attitude.end(),
        [](double value) { return std::isfinite(value); }) ||
      !std::all_of(state.acceleration.begin(), state.acceleration.end(),
        [](double value) { return std::isfinite(value); })) {
    return false;
  }
  Eigen::Quaterniond body_to_world(
    state.attitude[0], state.attitude[1], state.attitude[2], state.attitude[3]);
  if (!std::isfinite(body_to_world.norm()) || body_to_world.norm() < 1.0e-9) {
    return false;
  }
  body_to_world.normalize();
  const Eigen::Vector3d acceleration(
    state.acceleration[0], state.acceleration[1], state.acceleration[2]);
  const Eigen::Vector3d specific_force =
    acceleration - Eigen::Vector3d(0.0, 0.0, -kGravityMps2);
  collective = specific_force.dot(body_to_world.toRotationMatrix().col(2));
  return std::isfinite(collective) && collective > 0.0;
}

uint64_t timestampNanoseconds(const rclcpp::Time & timestamp) noexcept
{
  return timestamp.nanoseconds() > 0
    ? static_cast<uint64_t>(timestamp.nanoseconds()) : 0U;
}

struct Measurement
{
  VehicleState state;
  double collective_specific_force_m_s2 = 0.0;
  double raw_vertical_acceleration_m_s2 = 0.0;
  double tilt_rad = std::numeric_limits<double>::quiet_NaN();
  double hover_thrust = std::numeric_limits<double>::quiet_NaN();
  rclcpp::Time timestamp{0, 0, RCL_ROS_TIME};
  bool valid = false;
};

struct AppliedControl
{
  double collective_specific_force_m_s2 = std::numeric_limits<double>::quiet_NaN();
  rclcpp::Time timestamp{0, 0, RCL_ROS_TIME};
  bool valid = false;
};

struct Px4ThrustTelemetry
{
  std::array<double, 3> body_frd{};
  uint64_t timestamp_sample_us = 0;
  rclcpp::Time received_at{0, 0, RCL_ROS_TIME};
  bool valid = false;
};

struct MotorTelemetry
{
  std::array<double, kLoggedMotorCount> output{};
  double mean_output = std::numeric_limits<double>::quiet_NaN();
  uint64_t timestamp_sample_us = 0;
  rclcpp::Time received_at{0, 0, RCL_ROS_TIME};
  bool valid = false;
};

enum class SampleRejection : std::size_t
{
  none,
  missing_applied_control,
  missing_px4_thrust,
  missing_motor_output,
  stale_px4_telemetry,
  state_command_skew,
  excessive_tilt,
  acceleration_spike,
  duplicate_state
};

class CollectiveIdentificationNode final : public rclcpp::Node
{
public:
  CollectiveIdentificationNode()
  : Node("collective_identification_node")
  {
    declareParameters();
    readParameters();
    
    state_subscription_ = create_subscription<VehicleState>(
      "vehicle_state", rclcpp::QoS(10),
      [this](VehicleState::SharedPtr message) { stateCallback(std::move(message)); });
    external_mode_subscription_ = create_subscription<std_msgs::msg::Bool>(
      "mpc_external_mode_active", rclcpp::QoS(1).reliable().transient_local(),
      [this](std_msgs::msg::Bool::SharedPtr message) { externalModeCallback(std::move(message)); });
    hover_thrust_subscription_ = create_subscription<px4_msgs::msg::HoverThrustEstimate>(
      "/fmu/out/hover_thrust_estimate", rclcpp::QoS(10).best_effort(),
      [this](px4_msgs::msg::HoverThrustEstimate::SharedPtr message) {
        hoverThrustCallback(std::move(message));
      });
    applied_control_subscription_ = create_subscription<ForceAttitudeSetpoint>(
      "mpc_applied_force_attitude_setpoint", rclcpp::QoS(10),
      [this](ForceAttitudeSetpoint::SharedPtr message) {
        appliedControlCallback(std::move(message));
      });
    vehicle_thrust_subscription_ = create_subscription<VehicleThrustSetpoint>(
      "/fmu/out/vehicle_thrust_setpoint", rclcpp::QoS(10).best_effort(),
      [this](VehicleThrustSetpoint::SharedPtr message) {
        vehicleThrustCallback(std::move(message));
      });
    actuator_motors_subscription_ = create_subscription<ActuatorMotors>(
      "/fmu/out/actuator_motors", rclcpp::QoS(10).best_effort(),
      [this](ActuatorMotors::SharedPtr message) {
        actuatorMotorsCallback(std::move(message));
      });

    setpoint_publisher_ = create_publisher<ForceAttitudeSetpoint>(
      "force_attitude_setpoint", rclcpp::QoS(10));
    start_service_ = create_service<std_srvs::srv::Trigger>(
      "collective_identification/start",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        startRun(*response);
      });
    stop_service_ = create_service<std_srvs::srv::Trigger>(
      "collective_identification/stop",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        std::lock_guard<std::mutex> lock(mutex_);
        stopRunLocked("Stopped by service");
        response->success = true;
        response->message = "Collective identification stopped";
      });

    if (!validConfiguration()) {
      RCLCPP_FATAL(get_logger(), "Invalid collective identification configuration");
      return;
    }
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / sample_rate_hz_)),
      std::bind(&CollectiveIdentificationNode::update, this));
    RCLCPP_INFO(
      get_logger(),
      "Collective identification ready: amplitude=%.3f m/s2 half_period=%.2f s "
      "cycles=%d output=%s",
      excitation_amplitude_m_s2_, half_period_seconds_, cycles_, output_path_.c_str());
  }

private:
  struct ExcitationCommand
  {
    double collective_m_s2 = kGravityMps2;
    const char * phase = "settle";
  };

  void declareParameters()
  {
    declare_parameter("output_path", output_path_);
    declare_parameter("sample_rate_hz", sample_rate_hz_);
    declare_parameter("settle_seconds", settle_seconds_);
    declare_parameter("excitation_amplitude_m_s2", excitation_amplitude_m_s2_);
    declare_parameter("half_period_seconds", half_period_seconds_);
    declare_parameter("cycles", cycles_);
    declare_parameter("minimum_collective_m_s2", minimum_collective_m_s2_);
    declare_parameter("maximum_collective_m_s2", maximum_collective_m_s2_);
    declare_parameter("maximum_collective_rate_m_s3", maximum_collective_rate_m_s3_);
    declare_parameter("maximum_altitude_deviation_m", maximum_altitude_deviation_m_);
    declare_parameter("maximum_vertical_speed_m_s", maximum_vertical_speed_m_s_);
    declare_parameter("maximum_start_vertical_speed_m_s", maximum_start_vertical_speed_m_s_);
    declare_parameter("altitude_hold_position_gain", altitude_hold_position_gain_);
    declare_parameter("altitude_hold_velocity_gain", altitude_hold_velocity_gain_);
    declare_parameter("maximum_identification_tilt_rad", maximum_identification_tilt_rad_);
    declare_parameter("maximum_state_command_skew_seconds", maximum_state_command_skew_seconds_);
    declare_parameter("maximum_px4_telemetry_age_seconds", maximum_px4_telemetry_age_seconds_);
    declare_parameter("maximum_vertical_acceleration_jump_m_s2",
      maximum_vertical_acceleration_jump_m_s2_);
    declare_parameter("collective_measurement_filter_time_constant_seconds",
      collective_measurement_filter_time_constant_seconds_);
  }

  void readParameters()
  {
    get_parameter("output_path", output_path_);
    get_parameter("sample_rate_hz", sample_rate_hz_);
    get_parameter("settle_seconds", settle_seconds_);
    get_parameter("excitation_amplitude_m_s2", excitation_amplitude_m_s2_);
    get_parameter("half_period_seconds", half_period_seconds_);
    get_parameter("cycles", cycles_);
    get_parameter("minimum_collective_m_s2", minimum_collective_m_s2_);
    get_parameter("maximum_collective_m_s2", maximum_collective_m_s2_);
    get_parameter("maximum_collective_rate_m_s3", maximum_collective_rate_m_s3_);
    get_parameter("maximum_altitude_deviation_m", maximum_altitude_deviation_m_);
    get_parameter("maximum_vertical_speed_m_s", maximum_vertical_speed_m_s_);
    get_parameter("maximum_start_vertical_speed_m_s", maximum_start_vertical_speed_m_s_);
    get_parameter("altitude_hold_position_gain", altitude_hold_position_gain_);
    get_parameter("altitude_hold_velocity_gain", altitude_hold_velocity_gain_);
    get_parameter("maximum_identification_tilt_rad", maximum_identification_tilt_rad_);
    get_parameter("maximum_state_command_skew_seconds", maximum_state_command_skew_seconds_);
    get_parameter("maximum_px4_telemetry_age_seconds", maximum_px4_telemetry_age_seconds_);
    get_parameter("maximum_vertical_acceleration_jump_m_s2",
      maximum_vertical_acceleration_jump_m_s2_);
    get_parameter("collective_measurement_filter_time_constant_seconds",
      collective_measurement_filter_time_constant_seconds_);
  }

  bool validConfiguration() const noexcept
  {
    return !output_path_.empty() && std::isfinite(sample_rate_hz_) && sample_rate_hz_ > 0.0 &&
      std::isfinite(settle_seconds_) && settle_seconds_ >= 0.0 &&
      std::isfinite(excitation_amplitude_m_s2_) && excitation_amplitude_m_s2_ > 0.0 &&
      std::isfinite(half_period_seconds_) && half_period_seconds_ > 0.0 && cycles_ > 0 &&
      std::isfinite(minimum_collective_m_s2_) && std::isfinite(maximum_collective_m_s2_) &&
      minimum_collective_m_s2_ > 0.0 && minimum_collective_m_s2_ < maximum_collective_m_s2_ &&
      std::isfinite(maximum_collective_rate_m_s3_) && maximum_collective_rate_m_s3_ > 0.0 &&
      std::isfinite(maximum_altitude_deviation_m_) && maximum_altitude_deviation_m_ > 0.0 &&
      std::isfinite(maximum_vertical_speed_m_s_) && maximum_vertical_speed_m_s_ > 0.0 &&
      std::isfinite(maximum_start_vertical_speed_m_s_) &&
      maximum_start_vertical_speed_m_s_ > 0.0 &&
      std::isfinite(altitude_hold_position_gain_) && altitude_hold_position_gain_ > 0.0 &&
      std::isfinite(altitude_hold_velocity_gain_) && altitude_hold_velocity_gain_ > 0.0 &&
      std::isfinite(maximum_identification_tilt_rad_) &&
      maximum_identification_tilt_rad_ > 0.0 &&
      std::isfinite(maximum_state_command_skew_seconds_) &&
      maximum_state_command_skew_seconds_ > 0.0 &&
      std::isfinite(maximum_px4_telemetry_age_seconds_) &&
      maximum_px4_telemetry_age_seconds_ > 0.0 &&
      std::isfinite(maximum_vertical_acceleration_jump_m_s2_) &&
      maximum_vertical_acceleration_jump_m_s2_ > 0.0 &&
      std::isfinite(collective_measurement_filter_time_constant_seconds_) &&
      collective_measurement_filter_time_constant_seconds_ >= 0.0 &&
      excitation_amplitude_m_s2_ <
        0.5 * (maximum_collective_m_s2_ - minimum_collective_m_s2_);
  }

  void stateCallback(VehicleState::SharedPtr message)
  {
    if (!message) {
      return;
    }
    double collective = 0.0;
    const bool valid = measuredCollective(*message, collective);
    std::lock_guard<std::mutex> lock(mutex_);
    latest_measurement_.state = *message;
    latest_measurement_.collective_specific_force_m_s2 = collective;
    latest_measurement_.raw_vertical_acceleration_m_s2 = message->acceleration[2];
    latest_measurement_.tilt_rad = tiltAngle(*message);
    latest_measurement_.hover_thrust = latest_hover_thrust_;
    latest_measurement_.timestamp = rclcpp::Time(message->header.stamp, RCL_ROS_TIME);
    latest_measurement_.valid = valid && latest_measurement_.timestamp.nanoseconds() > 0;
  }

  void externalModeCallback(std_msgs::msg::Bool::SharedPtr message)
  {
    if (!message) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    external_mode_active_ = message->data;
    if (!external_mode_active_) {
      stopRunLocked("External Mode became inactive");
    }
  }

  void hoverThrustCallback(px4_msgs::msg::HoverThrustEstimate::SharedPtr message)
  {
    if (!message || !message->valid || !std::isfinite(message->hover_thrust) ||
        message->hover_thrust <= 0.05f) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    latest_hover_thrust_ = message->hover_thrust;
  }

  void appliedControlCallback(ForceAttitudeSetpoint::SharedPtr message)
  {
    if (!message || !std::isfinite(message->desired_collective_specific_force_m_s2)) {
      return;
    }
    const rclcpp::Time timestamp(message->header.stamp, RCL_ROS_TIME);
    if (timestamp.nanoseconds() <= 0) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    latest_applied_control_.collective_specific_force_m_s2 =
      message->desired_collective_specific_force_m_s2;
    latest_applied_control_.timestamp = timestamp;
    latest_applied_control_.valid = true;
  }

  void vehicleThrustCallback(VehicleThrustSetpoint::SharedPtr message)
  {
    if (!message || !std::all_of(message->xyz.begin(), message->xyz.end(),
        [](float value) { return std::isfinite(value); })) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    for (std::size_t index = 0; index < latest_thrust_.body_frd.size(); ++index) {
      latest_thrust_.body_frd[index] = message->xyz[index];
    }
    latest_thrust_.timestamp_sample_us = message->timestamp_sample;
    latest_thrust_.received_at = now();
    latest_thrust_.valid = message->timestamp_sample != 0U;
  }

  void actuatorMotorsCallback(ActuatorMotors::SharedPtr message)
  {
    if (!message) {
      return;
    }
    std::array<double, kLoggedMotorCount> motor_output{};
    for (std::size_t index = 0; index < motor_output.size(); ++index) {
      if (!std::isfinite(message->control[index])) {
        return;
      }
      motor_output[index] = message->control[index];
    }
    std::lock_guard<std::mutex> lock(mutex_);
    latest_motors_.output = motor_output;
    latest_motors_.mean_output =
      std::accumulate(motor_output.begin(), motor_output.end(), 0.0) /
      static_cast<double>(motor_output.size());
    latest_motors_.timestamp_sample_us = message->timestamp_sample;
    latest_motors_.received_at = now();
    latest_motors_.valid = message->timestamp_sample != 0U;
  }

  void startRun(std_srvs::srv::Trigger::Response & response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!external_mode_active_) {
      response.success = false;
      response.message = "External Mode is not active";
      return;
    }
    if (!latest_measurement_.valid) {
      response.success = false;
      response.message = "VehicleState is not valid yet";
      return;
    }
    if (std::abs(latest_measurement_.state.velocity[2]) > maximum_start_vertical_speed_m_s_) {
      response.success = false;
      response.message = "Vehicle is still moving vertically; wait for hover and retry";
      return;
    }
    if (running_) {
      response.success = false;
      response.message = "Collective identification is already running";
      return;
    }
    if (!openOutputLocked(response)) {
      return;
    }
    if (!collective_filter_.configure(
        collective_measurement_filter_time_constant_seconds_, 1.0 / sample_rate_hz_)) {
      stopRunLocked("Invalid collective measurement filter configuration");
      response.success = false;
      response.message = "Cannot configure collective measurement filter";
      return;
    }
    resetSampleStatisticsLocked();
    baseline_collective_m_s2_ = std::clamp(
      latest_measurement_.collective_specific_force_m_s2,
      minimum_collective_m_s2_ + excitation_amplitude_m_s2_,
      maximum_collective_m_s2_ - excitation_amplitude_m_s2_);
    previous_command_m_s2_ = baseline_collective_m_s2_;
    initial_altitude_m_ = latest_measurement_.state.position[2];
    run_started_at_ = now();
    last_update_at_ = run_started_at_;
    running_ = true;
    sequence_ = 0;
    response.success = true;
    response.message = "Collective identification started";
    RCLCPP_INFO(get_logger(), "Collective identification started: baseline=%.3f m/s2",
      baseline_collective_m_s2_);
  }

  bool openOutputLocked(std_srvs::srv::Trigger::Response & response)
  {
    try {
      const std::filesystem::path path(output_path_);
      if (path.has_parent_path()) {
        std::filesystem::create_directories(path.parent_path());
      }
      output_.open(path, std::ios::out | std::ios::trunc);
    } catch (const std::filesystem::filesystem_error & error) {
      response.success = false;
      response.message = std::string("Cannot create output: ") + error.what();
      return false;
    }
    if (!output_.is_open()) {
      response.success = false;
      response.message = "Cannot open identification output";
      return false;
    }
    output_ << "time_s,phase,command_collective_m_s2,applied_collective_m_s2,"
            << "state_timestamp_ns,applied_timestamp_ns,state_command_skew_ms,"
            << "thrust_setpoint_timestamp_sample_us,motors_timestamp_sample_us,"
            << "measured_collective_m_s2,filtered_collective_m_s2,hover_thrust,"
            << "raw_vertical_acceleration_m_s2,tilt_rad,position_z_m,velocity_z_m_s,"
            << "thrust_setpoint_frd_x,thrust_setpoint_frd_y,thrust_setpoint_frd_z,"
            << "motor_0,motor_1,motor_2,motor_3,motor_mean,external_mode\n";
    return true;
  }

  void stopRunLocked(const std::string & reason)
  {
    if (!running_ && !output_.is_open()) {
      return;
    }
    running_ = false;
    if (output_.is_open()) {
      output_.flush();
      output_.close();
    }
    RCLCPP_INFO(
      get_logger(),
      "Collective identification stopped: %s accepted=%lu rejected=[missing_control=%lu "
      "missing_thrust=%lu missing_motors=%lu stale=%lu skew=%lu tilt=%lu spike=%lu duplicate=%lu]",
      reason.c_str(), static_cast<unsigned long>(accepted_samples_),
      static_cast<unsigned long>(rejected_samples_[static_cast<std::size_t>(SampleRejection::missing_applied_control)]),
      static_cast<unsigned long>(rejected_samples_[static_cast<std::size_t>(SampleRejection::missing_px4_thrust)]),
      static_cast<unsigned long>(rejected_samples_[static_cast<std::size_t>(SampleRejection::missing_motor_output)]),
      static_cast<unsigned long>(rejected_samples_[static_cast<std::size_t>(SampleRejection::stale_px4_telemetry)]),
      static_cast<unsigned long>(rejected_samples_[static_cast<std::size_t>(SampleRejection::state_command_skew)]),
      static_cast<unsigned long>(rejected_samples_[static_cast<std::size_t>(SampleRejection::excessive_tilt)]),
      static_cast<unsigned long>(rejected_samples_[static_cast<std::size_t>(SampleRejection::acceleration_spike)]),
      static_cast<unsigned long>(rejected_samples_[static_cast<std::size_t>(SampleRejection::duplicate_state)]));
  }

  void resetSampleStatisticsLocked()
  {
    accepted_samples_ = 0;
    rejected_samples_.fill(0);
    last_logged_state_timestamp_ns_ = 0;
    previous_accepted_vertical_acceleration_m_s2_.reset();
  }

  void update()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!external_mode_active_ || !latest_measurement_.valid) {
      return;
    }
    const rclcpp::Time timestamp = now();
    if (!running_) {
      publishSetpointLocked(timestamp, hoverCollectiveLocked(), latest_measurement_.state.yaw);
      return;
    }
    const double elapsed = (timestamp - run_started_at_).seconds();
    const double dt = std::clamp((timestamp - last_update_at_).seconds(), 0.0, 0.2);
    last_update_at_ = timestamp;
    const auto & state = latest_measurement_.state;
    if (!withinSafetyEnvelope(state)) {
      publishSetpointLocked(timestamp, baseline_collective_m_s2_, state.yaw);
      stopRunLocked("Safety envelope exceeded; baseline hover published");
      return;
    }
    const double excitation_time = elapsed - settle_seconds_;
    const double total_excitation = 2.0 * half_period_seconds_ * cycles_;
    if (excitation_time >= total_excitation) {
      stopRunLocked("Excitation completed");
      return;
    }

    const ExcitationCommand command = nextCommand(state, excitation_time, dt);
    publishSetpointLocked(timestamp, command.collective_m_s2, state.yaw);
    appendSampleLocked(elapsed, command.phase, timestamp);
  }

  bool withinSafetyEnvelope(const VehicleState & state) const noexcept
  {
    return std::isfinite(initial_altitude_m_) &&
      std::abs(state.position[2] - initial_altitude_m_) <= maximum_altitude_deviation_m_ &&
      std::abs(state.velocity[2]) <= maximum_vertical_speed_m_s_;
  }

  ExcitationCommand nextCommand(
    const VehicleState & state, double excitation_time, double dt)
  {
    double target = baseline_collective_m_s2_ + altitude_hold_position_gain_ *
      (initial_altitude_m_ - state.position[2]) -
      altitude_hold_velocity_gain_ * state.velocity[2];
    const char * phase = "settle";
    if (excitation_time >= 0.0) {
      const int half_period = static_cast<int>(excitation_time / half_period_seconds_);
      const bool high_phase = half_period % 2 == 0;
      target += high_phase ? excitation_amplitude_m_s2_ : -excitation_amplitude_m_s2_;
      phase = high_phase ? "high" : "low";
    }
    target = std::clamp(target, minimum_collective_m_s2_, maximum_collective_m_s2_);
    const double maximum_change = maximum_collective_rate_m_s3_ * dt;
    previous_command_m_s2_ = std::clamp(
      target, previous_command_m_s2_ - maximum_change,
      previous_command_m_s2_ + maximum_change);
    return {previous_command_m_s2_, phase};
  }

  double hoverCollectiveLocked() const noexcept
  {
    return std::clamp(
      latest_measurement_.collective_specific_force_m_s2,
      minimum_collective_m_s2_, maximum_collective_m_s2_);
  }

  SampleRejection sampleRejectionLocked(const rclcpp::Time & timestamp) const
  {
    if (!latest_applied_control_.valid) {
      return SampleRejection::missing_applied_control;
    }
    if (!latest_thrust_.valid) {
      return SampleRejection::missing_px4_thrust;
    }
    if (!latest_motors_.valid) {
      return SampleRejection::missing_motor_output;
    }
    if ((timestamp - latest_thrust_.received_at).seconds() > maximum_px4_telemetry_age_seconds_ ||
        (timestamp - latest_motors_.received_at).seconds() > maximum_px4_telemetry_age_seconds_) {
      return SampleRejection::stale_px4_telemetry;
    }
    const double state_command_skew = std::abs(
      (latest_measurement_.timestamp - latest_applied_control_.timestamp).seconds());
    if (!std::isfinite(state_command_skew) ||
        state_command_skew > maximum_state_command_skew_seconds_) {
      return SampleRejection::state_command_skew;
    }
    if (!std::isfinite(latest_measurement_.tilt_rad) ||
        latest_measurement_.tilt_rad > maximum_identification_tilt_rad_) {
      return SampleRejection::excessive_tilt;
    }
    if (previous_accepted_vertical_acceleration_m_s2_ &&
        std::abs(latest_measurement_.raw_vertical_acceleration_m_s2 -
          *previous_accepted_vertical_acceleration_m_s2_) >
          maximum_vertical_acceleration_jump_m_s2_) {
      return SampleRejection::acceleration_spike;
    }
    if (timestampNanoseconds(latest_measurement_.timestamp) <= last_logged_state_timestamp_ns_) {
      return SampleRejection::duplicate_state;
    }
    return SampleRejection::none;
  }

  void appendSampleLocked(
    double elapsed, const char * phase, const rclcpp::Time & timestamp)
  {
    const SampleRejection rejection = sampleRejectionLocked(timestamp);
    if (rejection != SampleRejection::none) {
      ++rejected_samples_[static_cast<std::size_t>(rejection)];
      return;
    }
    const auto filtered_collective = collective_filter_.update(
      latest_measurement_.collective_specific_force_m_s2);
    if (!filtered_collective) {
      ++rejected_samples_[static_cast<std::size_t>(SampleRejection::acceleration_spike)];
      return;
    }
    const uint64_t state_timestamp_ns = timestampNanoseconds(latest_measurement_.timestamp);
    const uint64_t applied_timestamp_ns = timestampNanoseconds(latest_applied_control_.timestamp);
    const double state_command_skew_ms = std::abs(
      (latest_measurement_.timestamp - latest_applied_control_.timestamp).seconds()) * 1.0e3;
    const auto & state = latest_measurement_.state;
    output_ << std::fixed << std::setprecision(9)
            << elapsed << ',' << phase << ',' << previous_command_m_s2_ << ','
            << latest_applied_control_.collective_specific_force_m_s2 << ','
            << state_timestamp_ns << ',' << applied_timestamp_ns << ','
            << state_command_skew_ms << ',' << latest_thrust_.timestamp_sample_us << ','
            << latest_motors_.timestamp_sample_us << ','
            << latest_measurement_.collective_specific_force_m_s2 << ','
            << *filtered_collective << ',' << latest_measurement_.hover_thrust << ','
            << latest_measurement_.raw_vertical_acceleration_m_s2 << ','
            << latest_measurement_.tilt_rad << ',' << state.position[2] << ','
            << state.velocity[2] << ',' << latest_thrust_.body_frd[0] << ','
            << latest_thrust_.body_frd[1] << ',' << latest_thrust_.body_frd[2];
    for (const double output : latest_motors_.output) {
      output_ << ',' << output;
    }
    output_ << ',' << latest_motors_.mean_output << ','
            << (external_mode_active_ ? 1 : 0) << '\n';
    ++accepted_samples_;
    last_logged_state_timestamp_ns_ = state_timestamp_ns;
    previous_accepted_vertical_acceleration_m_s2_ =
      latest_measurement_.raw_vertical_acceleration_m_s2;
  }

  void publishSetpointLocked(
    const rclcpp::Time & timestamp, double collective, double yaw)
  {
    const Eigen::Quaterniond attitude(
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
    const Eigen::Vector3d force = attitude.toRotationMatrix().col(2) * collective;
    ForceAttitudeSetpoint setpoint;
    setpoint.header.stamp = timestamp;
    setpoint.header.frame_id = "map";
    setpoint.sequence = ++sequence_;
    setpoint.desired_specific_force_world_m_s2 = {force.x(), force.y(), force.z()};
    setpoint.desired_acceleration_m_s2 = {
      force.x(), force.y(), force.z() - kGravityMps2};
    setpoint.desired_attitude_wxyz = {
      attitude.w(), attitude.x(), attitude.y(), attitude.z()};
    setpoint.desired_collective_specific_force_m_s2 = collective;
    setpoint.tilt_angle_rad = 0.0;
    setpoint.recovery_active = true;
    setpoint_publisher_->publish(setpoint);
  }

  std::mutex mutex_;
  std::string output_path_ = "/tmp/mpc_controller_sim/collective_identification.csv";
  double sample_rate_hz_ = 50.0;
  double settle_seconds_ = 5.0;
  double excitation_amplitude_m_s2_ = 0.22;
  double half_period_seconds_ = 1.5;
  int cycles_ = 8;
  double minimum_collective_m_s2_ = 8.0;
  double maximum_collective_m_s2_ = 12.0;
  double maximum_collective_rate_m_s3_ = 2.0;
  double maximum_altitude_deviation_m_ = 0.75;
  double maximum_vertical_speed_m_s_ = 1.0;
  double maximum_start_vertical_speed_m_s_ = 0.15;
  double altitude_hold_position_gain_ = 0.8;
  double altitude_hold_velocity_gain_ = 1.2;
  double maximum_identification_tilt_rad_ = 0.08;
  double maximum_state_command_skew_seconds_ = 0.08;
  double maximum_px4_telemetry_age_seconds_ = 0.10;
  double maximum_vertical_acceleration_jump_m_s2_ = 2.5;
  double collective_measurement_filter_time_constant_seconds_ = 0.08;
  bool external_mode_active_ = false;
  bool running_ = false;
  double baseline_collective_m_s2_ = kGravityMps2;
  double previous_command_m_s2_ = kGravityMps2;
  double initial_altitude_m_ = std::numeric_limits<double>::quiet_NaN();
  double latest_hover_thrust_ = std::numeric_limits<double>::quiet_NaN();
  std::uint64_t sequence_ = 0;
  std::uint64_t accepted_samples_ = 0;
  std::array<std::uint64_t, static_cast<std::size_t>(SampleRejection::duplicate_state) + 1>
    rejected_samples_{};
  std::uint64_t last_logged_state_timestamp_ns_ = 0;
  std::optional<double> previous_accepted_vertical_acceleration_m_s2_;
  rclcpp::Time run_started_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_update_at_{0, 0, RCL_ROS_TIME};
  Measurement latest_measurement_;
  AppliedControl latest_applied_control_;
  Px4ThrustTelemetry latest_thrust_;
  MotorTelemetry latest_motors_;
  mpc_controller::state_estimation::CollectiveForceFilter collective_filter_;
  std::ofstream output_;
  rclcpp::Subscription<VehicleState>::SharedPtr state_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr external_mode_subscription_;
  rclcpp::Subscription<px4_msgs::msg::HoverThrustEstimate>::SharedPtr
    hover_thrust_subscription_;
  rclcpp::Subscription<ForceAttitudeSetpoint>::SharedPtr applied_control_subscription_;
  rclcpp::Subscription<VehicleThrustSetpoint>::SharedPtr vehicle_thrust_subscription_;
  rclcpp::Subscription<ActuatorMotors>::SharedPtr actuator_motors_subscription_;
  rclcpp::Publisher<ForceAttitudeSetpoint>::SharedPtr setpoint_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CollectiveIdentificationNode>());
  rclcpp::shutdown();
  return 0;
}
