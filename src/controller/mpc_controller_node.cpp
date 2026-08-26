#include "mpc_controller/solver/acados_tpmc_solver.hpp"
#include "mpc_controller/controller/collective_force_filter.hpp"
#include "mpc_controller/controller/force_attitude_mapping.hpp"
#include "mpc_controller/msg/force_attitude_setpoint.hpp"
#include "mpc_controller/msg/mpc_translational_output.hpp"
#include "mpc_controller/msg/reference_trajectory.hpp"
#include "mpc_controller/msg/vehicle_state.hpp"
#include "mpc_controller/solver/tpmc_constraints.hpp"
#include "mpc_controller/solver/tpmc_reference.hpp"

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace {

using ModelState = mpc_controller::tpmc::State;
using ModelInput = mpc_controller::tpmc::Input;
using ModelReference = mpc_controller::tpmc::TpmcReference;
using SolverStatus = mpc_controller::tpmc::SolverStatus;
using SolverConfiguration = mpc_controller::tpmc::Configuration;

constexpr double kLargeBound = 1.0e6;

template <typename Value>
bool finiteArray(const std::vector<Value> &values) noexcept {
  return std::all_of(values.begin(), values.end(),
                     [](Value value) { return std::isfinite(value); });
}

Eigen::Quaterniond quaternionFromEuler(double roll, double pitch, double yaw) {
  Eigen::Quaterniond quaternion =
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
  return quaternion.normalized();
}

std::array<double, 3> vectorToArray(const Eigen::Vector3d &value) noexcept {
  return {value.x(), value.y(), value.z()};
}

const char *statusName(SolverStatus status) noexcept {
  switch (status) {
  case SolverStatus::success:
    return "success";
  case SolverStatus::invalid_input:
    return "invalid_input";
  case SolverStatus::dependency_unavailable:
    return "dependency_unavailable";
  case SolverStatus::not_initialized:
    return "not_initialized";
  case SolverStatus::infeasible:
    return "infeasible";
  case SolverStatus::deadline_exceeded:
    return "deadline_exceeded";
  case SolverStatus::numerical_failure:
    return "numerical_failure";
  case SolverStatus::maximum_iterations:
    return "maximum_iterations";
  case SolverStatus::minimum_step:
    return "minimum_step";
  }
  return "unknown";
}

std::string
sqpStatisticsTrace(const mpc_controller::tpmc::SolveResult &result) {
  if (result.sqp_statistics_count == 0) {
    return "unavailable";
  }

  std::ostringstream trace;
  trace << std::scientific << std::setprecision(2);
  for (std::size_t index = 0; index < result.sqp_statistics_count; ++index) {
    const auto &iteration = result.sqp_statistics[index];
    if (index != 0) {
      trace << ' ';
    }
    trace << "[i=" << iteration.iteration << " qp=" << iteration.qp_status
          << '/' << iteration.qp_iterations;
    if (std::all_of(iteration.nlp_residuals.begin(),
                    iteration.nlp_residuals.end(),
                    [](double value) { return std::isfinite(value); })) {
      trace << " nlp=" << iteration.nlp_residuals[0] << ','
            << iteration.nlp_residuals[1] << ',' << iteration.nlp_residuals[2]
            << ',' << iteration.nlp_residuals[3];
    }
    if (std::isfinite(iteration.step_length)) {
      trace << " alpha=" << iteration.step_length;
    }
    if (std::all_of(iteration.qp_residuals.begin(),
                    iteration.qp_residuals.end(),
                    [](double value) { return std::isfinite(value); })) {
      trace << " qpres=" << iteration.qp_residuals[0] << ','
            << iteration.qp_residuals[1] << ','
            << iteration.qp_residuals[2] << ','
            << iteration.qp_residuals[3];
    }
    trace << ']';
  }
  return trace.str();
}

std::string kktResidualTrace(
    const mpc_controller::tpmc::SolveResult &result) {
  if (!std::all_of(result.kkt_residuals.begin(), result.kkt_residuals.end(),
                   [](double value) { return std::isfinite(value); })) {
    return "not_evaluated";
  }
  std::ostringstream trace;
  trace << std::scientific << std::setprecision(2) << '['
        << result.kkt_residuals[0] << ' ' << result.kkt_residuals[1] << ' '
        << result.kkt_residuals[2] << ' ' << result.kkt_residuals[3] << ']';
  return trace.str();
}

SolverConfiguration defaultConfiguration() {
  SolverConfiguration configuration;
  configuration.sample_time_seconds = 0.05;
  configuration.solver_deadline_seconds = 0.018;
  configuration.max_tilt_rad = mpc_controller::tpmc::kDefaultMaximumTiltRad;
  configuration.max_yaw_command_rad = 1.0e6;
  configuration.min_collective_specific_force_m_s2 =
      mpc_controller::tpmc::kDefaultMinimumCollectiveSpecificForceMps2;
  configuration.max_collective_specific_force_m_s2 =
      mpc_controller::tpmc::kDefaultMaximumCollectiveSpecificForceMps2;

  configuration.stage_weights = {60.0, 60.0, 120.0, 25.0, 25.0, 120.0,
                                 35.0, 35.0, 15.0, 20.0, 25.0};
  configuration.terminal_weights = {80.0, 80.0, 180.0, 35.0, 35.0, 160.0,
                                    50.0, 50.0, 20.0, 30.0, 40.0};
  configuration.input_weights = {8.0, 8.0, 10.0, 20.0};
  configuration.yaw_command_delta_weight = 30.0;

  configuration.state_lower.fill(-kLargeBound);
  configuration.state_upper.fill(kLargeBound);
  configuration.input_lower.fill(-kLargeBound);
  configuration.input_upper.fill(kLargeBound);

  configuration.state_lower[mpc_controller::tpmc::roll] =
      -configuration.max_tilt_rad;
  configuration.state_upper[mpc_controller::tpmc::roll] =
      configuration.max_tilt_rad;
  configuration.state_lower[mpc_controller::tpmc::pitch] =
      -configuration.max_tilt_rad;
  configuration.state_upper[mpc_controller::tpmc::pitch] =
      configuration.max_tilt_rad;
  configuration.state_lower[mpc_controller::tpmc::yaw_rate] =
      -configuration.max_yaw_rate_rad_s;
  configuration.state_upper[mpc_controller::tpmc::yaw_rate] =
      configuration.max_yaw_rate_rad_s;
  configuration.state_lower[mpc_controller::tpmc::collective_specific_force] =
      configuration.min_collective_specific_force_m_s2;
  configuration.state_upper[mpc_controller::tpmc::collective_specific_force] =
      configuration.max_collective_specific_force_m_s2;
  configuration.input_lower[mpc_controller::tpmc::roll_command] =
      -configuration.max_tilt_rad;
  configuration.input_upper[mpc_controller::tpmc::roll_command] =
      configuration.max_tilt_rad;
  configuration.input_lower[mpc_controller::tpmc::pitch_command] =
      -configuration.max_tilt_rad;
  configuration.input_upper[mpc_controller::tpmc::pitch_command] =
      configuration.max_tilt_rad;
  configuration.input_lower[mpc_controller::tpmc::yaw_command] =
      -configuration.max_yaw_command_rad;
  configuration.input_upper[mpc_controller::tpmc::yaw_command] =
      configuration.max_yaw_command_rad;
  configuration
      .input_lower[mpc_controller::tpmc::collective_specific_force_command] =
      configuration.min_collective_specific_force_m_s2;
  configuration
      .input_upper[mpc_controller::tpmc::collective_specific_force_command] =
      configuration.max_collective_specific_force_m_s2;
  return configuration;
}

class MpcControllerNode final : public rclcpp::Node {
public:
  MpcControllerNode()
      : Node("mpc_controller_node"),
        solver_configuration_(defaultConfiguration()) {
    declareAndGet("update_rate_hz", update_rate_hz_);
    declareAndGet("reference_timeout_seconds", reference_timeout_seconds_);
    declareAndGet("state_timeout_seconds", state_timeout_seconds_);
    declareAndGet("strict_validation", strict_validation_);
    declareAndGet("output_frame_id", output_frame_id_);
    declareAndGet("sample_time_seconds",
                  solver_configuration_.sample_time_seconds);
    declareAndGet("solver_deadline_seconds",
                  solver_configuration_.solver_deadline_seconds);
    declareAndGet("gravity_m_s2", solver_configuration_.model.gravity_m_s2);
    declareAndGet("max_tilt", solver_configuration_.max_tilt_rad);
    declareAndGet("max_tilt_rate_rad_s", solver_configuration_.max_tilt_rate_rad_s);
    declareAndGet("max_yaw_command_rad",
                  solver_configuration_.max_yaw_command_rad);
    declareAndGet("max_yaw_command_rate_rad_s",
                  solver_configuration_.max_yaw_command_rate_rad_s);
    declareAndGet("max_yaw_rate_rad_s",
                  solver_configuration_.max_yaw_rate_rad_s);
    declareAndGet("min_collective_specific_force_m_s2",
                  solver_configuration_.min_collective_specific_force_m_s2);
    declareAndGet("max_collective_specific_force_m_s2",
                  solver_configuration_.max_collective_specific_force_m_s2);
    declareAndGet("max_collective_rate_m_s3",
                  solver_configuration_.max_collective_rate_m_s3);
    declareAndGet("command_filter_alpha", command_filter_alpha_);
    declareAndGet("collective_handover_valid_samples",
                  collective_handover_valid_samples_);
    declareAndGet("handover_minimum_duration_seconds",
                  handover_minimum_duration_seconds_);
    declareAndGet("handover_maximum_yaw_rate_rad_s",
                  handover_maximum_yaw_rate_rad_s_);
    declareAndGet("collective_measurement_filter_time_constant_seconds",
                  collective_measurement_filter_time_constant_seconds_);
    declareAndGet("recovery_velocity_gain", recovery_velocity_gain_);
    declareAndGet("recovery_position_gain", recovery_position_gain_);
    declareAndGet("recovery_max_acceleration_xy",
                  recovery_max_acceleration_xy_);
    declareAndGet("recovery_max_acceleration_z", recovery_max_acceleration_z_);

    declareAndGet("roll_time_constant_seconds",
                  solver_configuration_.model.roll_time_constant_seconds);
    declareAndGet("pitch_time_constant_seconds",
                  solver_configuration_.model.pitch_time_constant_seconds);
    declareAndGet("yaw_natural_frequency_rad_s",
                  solver_configuration_.model.yaw_natural_frequency_rad_s);
    declareAndGet("yaw_damping_ratio",
                  solver_configuration_.model.yaw_damping_ratio);
    declareAndGet("collective_time_constant_seconds",
                  solver_configuration_.model.collective_time_constant_seconds);
    declare_parameter("stage_weights",
                      std::vector<double>{60.0, 60.0, 120.0, 25.0, 25.0, 120.0,
                                          35.0, 35.0, 15.0, 20.0, 25.0});
    declare_parameter("terminal_weights",
                      std::vector<double>{80.0, 80.0, 180.0, 35.0, 35.0, 160.0,
                                          50.0, 50.0, 20.0, 30.0, 40.0});
    declare_parameter("input_weights",
                      std::vector<double>{8.0, 8.0, 10.0, 20.0});
    declareAndGet("yaw_command_delta_weight",
                  solver_configuration_.yaw_command_delta_weight);

    const bool parameters_valid =
        readArrayParameter("stage_weights",
                           solver_configuration_.stage_weights) &&
        readArrayParameter("terminal_weights",
                           solver_configuration_.terminal_weights) &&
        readArrayParameter("input_weights",
                           solver_configuration_.input_weights);
    rebuildBounds();
    previous_input_[mpc_controller::tpmc::collective_specific_force_command] =
        solver_configuration_.model.gravity_m_s2;
    const bool collective_filter_valid = collective_force_filter_.configure(
        collective_measurement_filter_time_constant_seconds_,
        1.0 / update_rate_hz_);
    config_valid_ =
        parameters_valid && std::isfinite(update_rate_hz_) &&
        update_rate_hz_ > 0.0 && std::isfinite(reference_timeout_seconds_) &&
        reference_timeout_seconds_ > 0.0 &&
        std::isfinite(state_timeout_seconds_) && state_timeout_seconds_ > 0.0 &&
        std::isfinite(recovery_velocity_gain_) &&
        recovery_velocity_gain_ >= 0.0 &&
        std::isfinite(recovery_position_gain_) &&
        recovery_position_gain_ >= 0.0 &&
        std::isfinite(recovery_max_acceleration_xy_) &&
        recovery_max_acceleration_xy_ > 0.0 &&
        std::isfinite(recovery_max_acceleration_z_) &&
        recovery_max_acceleration_z_ > 0.0 && !output_frame_id_.empty() &&
        collective_handover_valid_samples_ > 0 &&
        std::isfinite(handover_minimum_duration_seconds_) &&
        handover_minimum_duration_seconds_ >= 0.0 &&
        std::isfinite(handover_maximum_yaw_rate_rad_s_) &&
        handover_maximum_yaw_rate_rad_s_ > 0.0 && collective_filter_valid &&
        mpc_controller::tpmc::validConfiguration(solver_configuration_) &&
        solver_configuration_.solver_deadline_seconds < 1.0 / update_rate_hz_;

    reference_subscription_ = create_subscription<ReferenceMessage>(
        "reference_trajectory", 10,
        [this](ReferenceMessage::SharedPtr message) {
          referenceCallback(std::move(message));
        });
    state_subscription_ = create_subscription<StateMessage>(
        "vehicle_state", 10, [this](StateMessage::SharedPtr message) {
          stateCallback(std::move(message));
        });
    external_mode_subscription_ = create_subscription<std_msgs::msg::Bool>(
        "mpc_external_mode_active", rclcpp::QoS(1).reliable().transient_local(),
        [this](std_msgs::msg::Bool::SharedPtr message) {
          externalModeCallback(std::move(message));
        });
    applied_setpoint_subscription_ = create_subscription<SetpointMessage>(
        "mpc_applied_force_attitude_setpoint", rclcpp::QoS(10),
        [this](SetpointMessage::SharedPtr message) {
          appliedSetpointCallback(std::move(message));
        });
    output_publisher_ =
        create_publisher<OutputMessage>("mpc_translational_output", 10);
    setpoint_publisher_ =
        create_publisher<SetpointMessage>("force_attitude_setpoint", 10);

    solver_ = std::make_unique<mpc_controller::tpmc::Application>(
        std::make_unique<mpc_controller::tpmc::AcadosTpmcSolver>(
            solver_configuration_));

    if (!config_valid_) {
      RCLCPP_ERROR(get_logger(),
                   "Invalid TMPC configuration; updates are disabled");
      return;
    }
    if (solver_->configured()) {
      RCLCPP_INFO(get_logger(),
                  "TMPC uses generated Acados solver %s with HPIPM QP "
                  "feedback",
                  solver_->backendName());
    } else {
      RCLCPP_ERROR(get_logger(),
                   "TMPC acados solver is unavailable; updates will use "
                   "recovery control");
    }
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / update_rate_hz_)),
        std::bind(&MpcControllerNode::update, this));
  }

private:
  using ReferenceMessage = mpc_controller::msg::ReferenceTrajectory;
  using StateMessage = mpc_controller::msg::VehicleState;
  using OutputMessage = mpc_controller::msg::MpcTranslationalOutput;
  using SetpointMessage = mpc_controller::msg::ForceAttitudeSetpoint;
  using ReferenceTrajectory = mpc_controller::tpmc::ReferenceTrajectory;

  enum class SetpointSource { solver, recovery, collective_handover };

  struct SetpointPublication {
    ModelInput input{};
    bool published = false;
    std::string rejection_reason;
  };

  struct AppliedCommand {
    ModelInput input{};
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  };

  template <typename Value>
  void declareAndGet(const std::string &name, Value &target) {
    declare_parameter(name, target);
    get_parameter(name, target);
  }

  template <typename Value, std::size_t Size>
  bool readArrayParameter(const std::string &name,
                          std::array<Value, Size> &target) {
    const auto values = get_parameter(name).as_double_array();
    if (values.size() != Size || !finiteArray(values)) {
      RCLCPP_ERROR(get_logger(),
                   "Parameter '%s' must contain %zu finite values",
                   name.c_str(), Size);
      return false;
    }
    std::copy(values.begin(), values.end(), target.begin());
    return true;
  }

  void rebuildBounds() {
    solver_configuration_.state_lower.fill(-kLargeBound);
    solver_configuration_.state_upper.fill(kLargeBound);
    solver_configuration_.input_lower.fill(-kLargeBound);
    solver_configuration_.input_upper.fill(kLargeBound);

    solver_configuration_.state_lower[mpc_controller::tpmc::roll] =
        -solver_configuration_.max_tilt_rad;
    solver_configuration_.state_upper[mpc_controller::tpmc::roll] =
        solver_configuration_.max_tilt_rad;
    solver_configuration_.state_lower[mpc_controller::tpmc::pitch] =
        -solver_configuration_.max_tilt_rad;
    solver_configuration_.state_upper[mpc_controller::tpmc::pitch] =
        solver_configuration_.max_tilt_rad;
    solver_configuration_.state_lower[mpc_controller::tpmc::yaw_rate] =
        -solver_configuration_.max_yaw_rate_rad_s;
    solver_configuration_.state_upper[mpc_controller::tpmc::yaw_rate] =
        solver_configuration_.max_yaw_rate_rad_s;
    solver_configuration_
        .state_lower[mpc_controller::tpmc::collective_specific_force] =
        solver_configuration_.min_collective_specific_force_m_s2;
    solver_configuration_
        .state_upper[mpc_controller::tpmc::collective_specific_force] =
        solver_configuration_.max_collective_specific_force_m_s2;
    solver_configuration_.input_lower[mpc_controller::tpmc::roll_command] =
        -solver_configuration_.max_tilt_rad;
    solver_configuration_.input_upper[mpc_controller::tpmc::roll_command] =
        solver_configuration_.max_tilt_rad;
    solver_configuration_.input_lower[mpc_controller::tpmc::pitch_command] =
        -solver_configuration_.max_tilt_rad;
    solver_configuration_.input_upper[mpc_controller::tpmc::pitch_command] =
        solver_configuration_.max_tilt_rad;
    solver_configuration_.input_lower[mpc_controller::tpmc::yaw_command] =
        -solver_configuration_.max_yaw_command_rad;
    solver_configuration_.input_upper[mpc_controller::tpmc::yaw_command] =
        solver_configuration_.max_yaw_command_rad;
    solver_configuration_
        .input_lower[mpc_controller::tpmc::collective_specific_force_command] =
        solver_configuration_.min_collective_specific_force_m_s2;
    solver_configuration_
        .input_upper[mpc_controller::tpmc::collective_specific_force_command] =
        solver_configuration_.max_collective_specific_force_m_s2;
  }

  static double
  durationSeconds(const builtin_interfaces::msg::Duration &duration) noexcept {
    return static_cast<double>(duration.sec) +
           static_cast<double>(duration.nanosec) * 1.0e-9;
  }

  static bool convertReference(const ReferenceMessage &message,
                               ReferenceTrajectory &output) noexcept {
    output.header_time_seconds = rclcpp::Time(message.header.stamp).seconds();
    output.hold_after_end = message.hold_after_end;
    output.points.clear();
    output.points.reserve(message.points.size());
    for (const auto &input : message.points) {
      mpc_controller::tpmc::ReferencePoint point;
      point.time_from_start = durationSeconds(input.time_from_start);
      point.position = input.position;
      point.velocity = input.velocity;
      point.acceleration = input.acceleration;
      point.yaw = input.yaw;
      point.yaw_rate = input.yaw_rate;
      output.points.push_back(point);
    }
    return mpc_controller::tpmc::validTrajectory(output);
  }

  static std::optional<ModelState> convertState(const StateMessage &message,
                                                double gravity_m_s2) noexcept {
    if (!std::all_of(message.position.begin(), message.position.end(),
                     [](double value) { return std::isfinite(value); }) ||
        !std::all_of(message.velocity.begin(), message.velocity.end(),
                     [](double value) { return std::isfinite(value); }) ||
        !std::all_of(message.acceleration.begin(), message.acceleration.end(),
                     [](double value) { return std::isfinite(value); }) ||
        !std::all_of(message.attitude.begin(), message.attitude.end(),
                     [](double value) { return std::isfinite(value); }) ||
        !std::isfinite(message.yaw_rate)) {
      return std::nullopt;
    }

    Eigen::Quaterniond body_to_world(message.attitude[0], message.attitude[1],
                                     message.attitude[2], message.attitude[3]);
    if (!std::isfinite(body_to_world.norm()) || body_to_world.norm() < 1.0e-9) {
      return std::nullopt;
    }
    body_to_world.normalize();
    const Eigen::Matrix3d rotation = body_to_world.toRotationMatrix();
    const double roll = std::atan2(rotation(2, 1), rotation(2, 2));
    const double pitch = std::asin(std::clamp(-rotation(2, 0), -1.0, 1.0));
    const double yaw = std::atan2(rotation(1, 0), rotation(0, 0));
    const Eigen::Vector3d acceleration(message.acceleration[0],
                                       message.acceleration[1],
                                       message.acceleration[2]);
    const Eigen::Vector3d gravity(0.0, 0.0, -gravity_m_s2);
    const double collective_force = rotation.col(2).dot(acceleration - gravity);

    ModelState output{message.position[0],
                      message.position[1],
                      message.position[2],
                      message.velocity[0],
                      message.velocity[1],
                      message.velocity[2],
                      roll,
                      pitch,
                      yaw,
                      message.yaw_rate,
                      collective_force};
    if (!mpc_controller::tpmc::finite(output) ||
        !std::isfinite(collective_force)) {
      return std::nullopt;
    }
    return output;
  }

  static void
  unwrapReferenceYaw(mpc_controller::tpmc::ReferenceHorizon &horizon,
                     double initial_yaw) noexcept {
    double previous_yaw = initial_yaw;
    for (auto &reference : horizon) {
      const double delta = mpc_controller::tpmc::shortestAngle(
          previous_yaw, reference.state[mpc_controller::tpmc::yaw]);
      previous_yaw += delta;
      reference.state[mpc_controller::tpmc::yaw] = previous_yaw;
      reference.input[mpc_controller::tpmc::yaw_command] = previous_yaw;
    }
  }

  ModelInput rebaseAppliedInputYaw(ModelInput input,
                                   double yaw_reference) const noexcept {
    input[mpc_controller::tpmc::yaw_command] =
        mpc_controller::tpmc::rebaseAngle(
            yaw_reference, input[mpc_controller::tpmc::yaw_command]);
    return input;
  }

  void referenceCallback(ReferenceMessage::SharedPtr message) {
    if (!message || message->header.frame_id.empty() ||
        message->header.stamp.sec == 0 && message->header.stamp.nanosec == 0) {
      return;
    }
    ReferenceTrajectory converted;
    if (!convertReference(*message, converted)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "TMPC reference rejected: invalid trajectory");
      return;
    }
    const rclcpp::Time stamp(message->header.stamp);
    std::lock_guard<std::mutex> lock(mutex_);
    if (reference_stamp_.nanoseconds() != 0 && stamp < reference_stamp_) {
      return;
    }
    reference_ = std::move(converted);
    reference_stamp_ = stamp;
    reference_trajectory_id_ = message->trajectory_id;
    reference_received_at_ = get_clock()->now();
  }

  void stateCallback(StateMessage::SharedPtr message) {
    if (!message) {
      return;
    }
    const rclcpp::Time stamp(message->header.stamp);
    if (stamp.nanoseconds() == 0) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_stamp_.nanoseconds() != 0 && stamp < state_stamp_) {
      return;
    }
    state_ = std::move(*message);
    state_stamp_ = stamp;
    state_received_at_ = get_clock()->now();
  }

  void appliedSetpointCallback(SetpointMessage::SharedPtr message) {
    if (!message) {
      return;
    }
    const auto input = modelInputFromSetpoint(*message);
    const rclcpp::Time stamp(message->header.stamp);
    if (!input || stamp.nanoseconds() == 0) {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (latest_applied_command_ && stamp < latest_applied_command_->stamp) {
      return;
    }
    latest_applied_command_ = AppliedCommand{*input, stamp};
  }

  void externalModeCallback(std_msgs::msg::Bool::SharedPtr message) {
    if (!message) {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (message->data == external_mode_active_) {
      return;
    }

    external_mode_active_ = message->data;
    latest_applied_command_.reset();
    continuous_measured_yaw_.reset();
    collective_handover_active_ = true;
    collective_handover_valid_count_ = 0;
    handover_started_at_ = get_clock()->now();
    collective_force_filter_.reset();
    previous_input_initialized_for_external_mode_ = false;
    solver_->reset();
    if (external_mode_active_) {
      activation_reference_trajectory_id_ = reference_trajectory_id_;
      RCLCPP_INFO(
          get_logger(),
          "TMPC External Mode gate opened; waiting for a post-activation "
          "reference before collective handover");
      return;
    }

    previous_input_ = {};
    handover_started_at_ = rclcpp::Time{0, 0, RCL_ROS_TIME};
    previous_input_[mpc_controller::tpmc::collective_specific_force_command] =
        solver_configuration_.model.gravity_m_s2;
    RCLCPP_INFO(get_logger(),
                "TMPC External Mode gate closed; solver state reset");
  }

  bool referenceReadyForExternalMode() const noexcept {
    return external_mode_active_ &&
           reference_trajectory_id_ > activation_reference_trajectory_id_;
  }

  void
  initializePreviousInputForExternalMode(const ModelState &measured) noexcept {
    previous_input_ = {
        std::clamp(measured[mpc_controller::tpmc::roll],
                   solver_configuration_
                       .input_lower[mpc_controller::tpmc::roll_command],
                   solver_configuration_
                       .input_upper[mpc_controller::tpmc::roll_command]),
        std::clamp(measured[mpc_controller::tpmc::pitch],
                   solver_configuration_
                       .input_lower[mpc_controller::tpmc::pitch_command],
                   solver_configuration_
                       .input_upper[mpc_controller::tpmc::pitch_command]),
        std::clamp(measured[mpc_controller::tpmc::yaw],
                   solver_configuration_
                       .input_lower[mpc_controller::tpmc::yaw_command],
                   solver_configuration_
                       .input_upper[mpc_controller::tpmc::yaw_command]),
        std::clamp(
            measured[mpc_controller::tpmc::collective_specific_force],
            solver_configuration_.input_lower
                [mpc_controller::tpmc::collective_specific_force_command],
            solver_configuration_.input_upper
                [mpc_controller::tpmc::collective_specific_force_command])};
    previous_input_initialized_for_external_mode_ = true;
    smoothed_roll_command_ = previous_input_[mpc_controller::tpmc::roll_command];
    smoothed_pitch_command_ = previous_input_[mpc_controller::tpmc::pitch_command];
    solver_->reset();
    RCLCPP_INFO(get_logger(),
                "TMPC External Mode input reset: roll=%.3f pitch=%.3f yaw=%.3f "
                "collective=%.3f",
                previous_input_[mpc_controller::tpmc::roll_command],
                previous_input_[mpc_controller::tpmc::pitch_command],
                previous_input_[mpc_controller::tpmc::yaw_command],
                previous_input_
                    [mpc_controller::tpmc::collective_specific_force_command]);
  }

  void update() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!config_valid_ || !solver_) {
      return;
    }
    if (!external_mode_active_) {
      return;
    }
    if (!referenceReadyForExternalMode()) {
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "TMPC waiting for a post-activation reference before solving");
      return;
    }
    if (!reference_ || !state_) {
      return;
    }

    const auto now = get_clock()->now();
    const double reference_age = (now - reference_received_at_).seconds();
    const double state_age = (now - state_received_at_).seconds();
    if (!std::isfinite(reference_age) || reference_age < 0.0 ||
        reference_age > reference_timeout_seconds_ ||
        !std::isfinite(state_age) || state_age < 0.0 ||
        state_age > state_timeout_seconds_) {
      if (!stale_input_active_) {
        solver_->reset();
        stale_input_active_ = true;
      }
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "TMPC update rejected: stale state or reference");
      return;
    }
    stale_input_active_ = false;

    if (strict_validation_ &&
        (!state_->valid || !state_->position_valid || !state_->velocity_valid ||
         !state_->acceleration_valid || !state_->attitude_valid)) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "TMPC update rejected: VehicleState validity flags are not ready");
      return;
    }

    auto measured =
        convertState(*state_, solver_configuration_.model.gravity_m_s2);
    if (!measured) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "TMPC update rejected: measured state is non-finite or invalid");
      return;
    }
    const double yaw_reference = continuous_measured_yaw_.value_or(
        previous_input_[mpc_controller::tpmc::yaw_command]);
    (*measured)[mpc_controller::tpmc::yaw] =
        mpc_controller::tpmc::rebaseAngle(
            yaw_reference, (*measured)[mpc_controller::tpmc::yaw]);
    continuous_measured_yaw_ = (*measured)[mpc_controller::tpmc::yaw];
    const auto filtered_collective = collective_force_filter_.update(
        (*measured)[mpc_controller::tpmc::collective_specific_force]);
    if (!filtered_collective) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "TMPC update rejected: collective force filter is not ready");
      return;
    }
    (*measured)[mpc_controller::tpmc::collective_specific_force] =
        *filtered_collective;

    if (!previous_input_initialized_for_external_mode_) {
      if (!mpc_controller::tpmc::hasValidCollectiveSpecificForce(
              *measured, solver_configuration_)) {
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "TMPC waiting for valid measured collective before input reset");
        return;
      }
      initializePreviousInputForExternalMode(*measured);
    }

    mpc_controller::tpmc::ReferenceHorizon horizon;
    const double trajectory_age =
        now.seconds() - reference_->header_time_seconds;
    if (!mpc_controller::tpmc::buildReferenceHorizon(
            *reference_, trajectory_age,
            solver_configuration_.sample_time_seconds,
            solver_configuration_.max_tilt_rad,
            solver_configuration_.model.gravity_m_s2, horizon)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "TMPC update rejected: reference cannot be "
                           "converted to TMPC state/input");
      return;
    }
    unwrapReferenceYaw(horizon, (*measured)[mpc_controller::tpmc::yaw]);

    OutputMessage output;
    output.header.stamp = now;
    output.header.frame_id = output_frame_id_;
    output.trajectory_id = reference_trajectory_id_;
    output.sequence = ++sequence_;
    output.measured_state = *measured;
    output.first_reference_state = horizon[0].state;
    output.first_reference_input = horizon[0].input;
    output.solver_available = solver_->configured();
    output.setpoint_age_seconds =
        std::isfinite(state_age) ? state_age : 0.0;
    output.reference_age_seconds =
        std::isfinite(reference_age) ? reference_age : 0.0;
    populateAppliedCommandTelemetry(output, now);

    if (!collectiveHandoverReady(*measured)) {
      std::optional<ModelInput> handover_input =
          publishRecoverySetpoint(*measured, horizon[0], now);
      std::string handover_rejection;
      if (!handover_input) {
        const ModelInput level_hover_input = makeHandoverInput(*measured);
        const SetpointPublication publication = publishSetpoint(
            level_hover_input, SetpointSource::collective_handover, now);
        if (publication.published) {
          handover_input = level_hover_input;
        } else {
          handover_rejection = publication.rejection_reason;
        }
      }
      if (handover_input) {
        previous_input_ = *handover_input;
      }
      output.solver_status =
          static_cast<std::uint8_t>(SolverStatus::invalid_input);
      output.valid = false;
      output.recovery_active = true;
      output.control_input = handover_input.value_or(makeHandoverInput(*measured));
      output.raw_control_input = output.control_input;
      output.filtered_control_input = output.control_input;
      output.failure_reason = collectiveHandoverDetail(*measured);
      if (handover_input) {
        output.failure_reason += "; recovery_position_hold_published";
      } else {
        output.failure_reason +=
            "; handover_setpoint_rejected: " + handover_rejection;
      }
      output_publisher_->publish(output);
      return;
    }

    synchronizePreviousInputWithAppliedCommand(now);

    mpc_controller::tpmc::SolveRequest request;
    request.initial_state = *measured;
    request.reference = horizon;
    request.previous_input = previous_input_;
    request.deadline =
        mpc_controller::tpmc::Clock::now() +
        std::chrono::duration_cast<mpc_controller::tpmc::Clock::duration>(
            std::chrono::duration<double>(
                solver_configuration_.solver_deadline_seconds));
    const auto result = solver_->solve(request);

    output.solver_status = static_cast<std::uint8_t>(result.status);
    output.solver_iterations = result.iterations;
    output.solve_time_seconds = result.solve_time_seconds;
    output.preparation_time_seconds = result.preparation_time_seconds;
    output.acados_wall_time_seconds = result.acados_wall_time_seconds;
    output.postprocessing_time_seconds = result.postprocessing_time_seconds;
    output.acados_metadata_time_seconds = result.acados_metadata_time_seconds;
    output.diagnostics_time_seconds = result.diagnostics_time_seconds;
    output.sqp_statistics_time_seconds = result.sqp_statistics_time_seconds;
    output.prediction_read_time_seconds = result.prediction_read_time_seconds;
    output.constraint_validation_time_seconds =
        result.constraint_validation_time_seconds;
    output.result_finalization_time_seconds =
        result.result_finalization_time_seconds;
    output.postprocessing_unattributed_time_seconds =
        result.postprocessing_unattributed_time_seconds;
    output.end_to_end_time_seconds = result.end_to_end_time_seconds;
    output.max_constraint_violation = result.max_constraint_violation;
    output.valid = result.valid;
    output.deadline_missed = result.deadline_missed;
    output.failure_reason =
        result.detail.empty() ? statusName(result.status) : result.detail;
    output.predicted_states.reserve((mpc_controller::tpmc::kHorizonLength + 1) *
                                    mpc_controller::tpmc::kStateDimension);
    for (const auto &state : result.predicted_states) {
      output.predicted_states.insert(output.predicted_states.end(),
                                     state.begin(), state.end());
    }
    output.predicted_inputs.reserve(mpc_controller::tpmc::kHorizonLength *
                                    mpc_controller::tpmc::kInputDimension);
    for (const auto &input : result.predicted_inputs) {
      output.predicted_inputs.insert(output.predicted_inputs.end(),
                                     input.begin(), input.end());
    }

    const SetpointPublication solver_publication =
        result.valid
            ? publishSetpoint(result.first_input, SetpointSource::solver, now)
            : SetpointPublication{};
    if (solver_publication.published) {
      output.raw_control_input = result.first_input;
      output.filtered_control_input = solver_publication.input;
      output.control_input = solver_publication.input;
      output.recovery_active = false;
      // The applied-command callback will replace this local fallback before
      // the next solve. It prevents a missing DDS sample from reverting to
      // the raw, pre-filter command.
      previous_input_ = solver_publication.input;
    } else {
      output.valid = false;
      output.recovery_active = true;
      const std::string solver_failure =
          result.valid
              ? "solver_output_rejected: " + solver_publication.rejection_reason
              : result.detail;
      const auto recovery_input =
          publishRecoverySetpoint(*measured, horizon[0], now);
      if (recovery_input) {
        output.control_input = *recovery_input;
        output.raw_control_input = *recovery_input;
        output.filtered_control_input = *recovery_input;
        previous_input_ = *recovery_input;
        output.failure_reason =
            solver_failure.empty()
                ? "recovery_setpoint_published"
                : solver_failure + "; recovery_setpoint_published";
      } else {
        output.control_input = {0.0, 0.0,
                                horizon[0].state[mpc_controller::tpmc::yaw],
                                solver_configuration_.model.gravity_m_s2};
        output.raw_control_input = output.control_input;
        output.filtered_control_input = output.control_input;
        output.failure_reason =
            solver_failure.empty()
                ? "recovery_setpoint_rejected"
                : solver_failure + "; recovery_setpoint_rejected";
      }
      const std::string sqp_trace = sqpStatisticsTrace(result);
      const std::string kkt_trace = kktResidualTrace(result);
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "TMPC fallback: status=%s detail=%s iterations=%d solve_ms=%.3f "
          "kkt=%s "
          "timing_ms=[prepare=%.3f acados_wall=%.3f post=%.3f total=%.3f "
          "lin=%.3f qp=%.3f reg=%.3f] "
          "post_ms=[meta=%.3f diag=%.3f sqp_stat=%.3f prediction=%.3f "
          "constraints=%.3f final=%.3f other=%.3f] "
          "recovery=%s measured_z=%.3f reference_z=%.3f sqp_trace=%s",
          statusName(result.status), solver_failure.c_str(), result.iterations,
          result.solve_time_seconds * 1.0e3, kkt_trace.c_str(),
          result.preparation_time_seconds * 1.0e3,
          result.acados_wall_time_seconds * 1.0e3,
          result.postprocessing_time_seconds * 1.0e3,
          result.end_to_end_time_seconds * 1.0e3,
          result.linearization_time_seconds * 1.0e3,
          result.qp_time_seconds * 1.0e3,
          result.regularization_time_seconds * 1.0e3,
          result.acados_metadata_time_seconds * 1.0e3,
          result.diagnostics_time_seconds * 1.0e3,
          result.sqp_statistics_time_seconds * 1.0e3,
          result.prediction_read_time_seconds * 1.0e3,
          result.constraint_validation_time_seconds * 1.0e3,
          result.result_finalization_time_seconds * 1.0e3,
          result.postprocessing_unattributed_time_seconds * 1.0e3,
          recovery_input ? "published" : "rejected",
          (*measured)[mpc_controller::tpmc::position_z],
          horizon[0].state[mpc_controller::tpmc::position_z],
          sqp_trace.c_str());
    }
    output_publisher_->publish(output);
  }

  bool collectiveHandoverReady(const ModelState &measured) {
    const bool collective_is_valid =
        mpc_controller::tpmc::hasValidCollectiveSpecificForce(
            measured, solver_configuration_);
    const double measured_yaw_rate =
        measured[mpc_controller::tpmc::yaw_rate];
    if (collective_is_valid && !collective_handover_active_) {
      return true;
    }
    const bool yaw_rate_is_stable = std::isfinite(measured_yaw_rate) &&
        std::abs(measured_yaw_rate) <= handover_maximum_yaw_rate_rad_s_;
    const double handover_age_seconds = handover_started_at_.nanoseconds() > 0
        ? (get_clock()->now() - handover_started_at_).seconds() : 0.0;
    const bool minimum_duration_elapsed =
        std::isfinite(handover_age_seconds) &&
        handover_age_seconds >= handover_minimum_duration_seconds_;
    if (!collective_is_valid || !yaw_rate_is_stable) {
      if (!collective_handover_active_) {
        solver_->reset();
      }
      collective_handover_active_ = true;
      collective_handover_valid_count_ = 0;
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "TMPC state handover active: collective=%.3f expected=[%.3f, %.3f] "
          "yaw_rate=%.3f limit=%.3f; streaming level hover",
          measured[mpc_controller::tpmc::collective_specific_force],
          solver_configuration_
              .state_lower[mpc_controller::tpmc::collective_specific_force],
          solver_configuration_
              .state_upper[mpc_controller::tpmc::collective_specific_force],
          measured_yaw_rate, handover_maximum_yaw_rate_rad_s_);
      return false;
    }

    if (collective_handover_valid_count_ < collective_handover_valid_samples_) {
      ++collective_handover_valid_count_;
    }
    if (!minimum_duration_elapsed ||
        collective_handover_valid_count_ < collective_handover_valid_samples_) {
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "TMPC state handover waiting: collective=%.3f yaw_rate=%.3f "
          "sample=%d/%d age=%.3f/%.3f s",
          measured[mpc_controller::tpmc::collective_specific_force],
          measured_yaw_rate, collective_handover_valid_count_,
          collective_handover_valid_samples_, handover_age_seconds,
          handover_minimum_duration_seconds_);
      return false;
    }

    if (collective_handover_active_) {
      collective_handover_active_ = false;
      initializePreviousInputForExternalMode(measured);
      RCLCPP_INFO(
          get_logger(),
          "TMPC state handover complete: collective=%.3f yaw_rate=%.3f "
          "valid_samples=%d age=%.3f s",
          measured[mpc_controller::tpmc::collective_specific_force],
          measured_yaw_rate, collective_handover_valid_count_,
          handover_age_seconds);
    }
    return true;
  }

  std::string collectiveHandoverDetail(const ModelState &measured) const {
    return "collective_handover_waiting: state[collective_specific_force]=" +
           std::to_string(
               measured[mpc_controller::tpmc::collective_specific_force]) +
           " expected=[" +
           std::to_string(
               solver_configuration_.state_lower
                   [mpc_controller::tpmc::collective_specific_force]) +
           ", " +
           std::to_string(
               solver_configuration_.state_upper
                   [mpc_controller::tpmc::collective_specific_force]) +
           "] state[yaw_rate]=" +
           std::to_string(measured[mpc_controller::tpmc::yaw_rate]) +
           " max_abs=" + std::to_string(handover_maximum_yaw_rate_rad_s_);
  }

  ModelInput makeHandoverInput(const ModelState &measured) const noexcept {
    const double yaw = std::clamp(
        measured[mpc_controller::tpmc::yaw],
        solver_configuration_.input_lower[mpc_controller::tpmc::yaw_command],
        solver_configuration_.input_upper[mpc_controller::tpmc::yaw_command]);
    return {0.0, 0.0, yaw, solver_configuration_.model.gravity_m_s2};
  }

  std::optional<ModelInput>
  modelInputFromSetpoint(const SetpointMessage &setpoint) const noexcept {
    const auto &raw_quaternion = setpoint.desired_attitude_wxyz;
    Eigen::Quaterniond quaternion(raw_quaternion[0], raw_quaternion[1],
                                  raw_quaternion[2], raw_quaternion[3]);
    if (!quaternion.coeffs().allFinite() || quaternion.norm() < 1.0e-9 ||
        !std::isfinite(setpoint.desired_collective_specific_force_m_s2)) {
      return std::nullopt;
    }
    quaternion.normalize();
    const Eigen::Matrix3d rotation = quaternion.toRotationMatrix();
    const ModelInput input{
        std::atan2(rotation(2, 1), rotation(2, 2)),
        std::asin(std::clamp(-rotation(2, 0), -1.0, 1.0)),
        std::atan2(rotation(1, 0), rotation(0, 0)),
        setpoint.desired_collective_specific_force_m_s2};
    if (!mpc_controller::tpmc::finite(input) ||
        !mpc_controller::tpmc::describeInputViolation(input,
                                                       solver_configuration_)
             .empty()) {
      return std::nullopt;
    }
    return input;
  }

  std::optional<AppliedCommand>
  freshAppliedCommand(const rclcpp::Time &now) const noexcept {
    if (!latest_applied_command_) {
      return std::nullopt;
    }
    const double age_seconds = (now - latest_applied_command_->stamp).seconds();
    if (!std::isfinite(age_seconds) || age_seconds < 0.0 ||
        age_seconds > state_timeout_seconds_) {
      return std::nullopt;
    }
    return latest_applied_command_;
  }

  void populateAppliedCommandTelemetry(OutputMessage &output,
                                       const rclcpp::Time &now) const noexcept {
    const auto applied = freshAppliedCommand(now);
    if (!applied) {
      return;
    }
    output.applied_control_input = rebaseAppliedInputYaw(
        applied->input, continuous_measured_yaw_.value_or(
                            previous_input_[mpc_controller::tpmc::yaw_command]));
    output.applied_command_age_seconds =
        std::max(0.0, (now - applied->stamp).seconds());
    output.applied_command_available = true;
  }

  void synchronizePreviousInputWithAppliedCommand(
      const rclcpp::Time &now) noexcept {
    const auto applied = freshAppliedCommand(now);
    if (!applied) {
      return;
    }
    previous_input_ = rebaseAppliedInputYaw(
        applied->input, continuous_measured_yaw_.value_or(
                            previous_input_[mpc_controller::tpmc::yaw_command]));
    // Start the local filter at the command the adapter actually emitted. If
    // SLERP has clipped tilt, the next filtered command evolves from that
    // clipped state instead of an unattainable request.
    smoothed_roll_command_ =
        previous_input_[mpc_controller::tpmc::roll_command];
    smoothed_pitch_command_ =
        previous_input_[mpc_controller::tpmc::pitch_command];
  }

  SetpointPublication publishSetpoint(const ModelInput &input,
                                      SetpointSource source,
                                      const rclcpp::Time &stamp) {
    SetpointPublication publication;
    publication.input = input;
    publication.rejection_reason = mpc_controller::tpmc::describeInputViolation(
        input, solver_configuration_);
    if (!publication.rejection_reason.empty()) {
      return publication;
    }

    const double raw_roll = input[mpc_controller::tpmc::roll_command];
    const double raw_pitch = input[mpc_controller::tpmc::pitch_command];
    if (source == SetpointSource::solver) {
      smoothed_roll_command_ =
          (1.0 - command_filter_alpha_) * smoothed_roll_command_ +
          command_filter_alpha_ * raw_roll;
      smoothed_pitch_command_ =
          (1.0 - command_filter_alpha_) * smoothed_pitch_command_ +
          command_filter_alpha_ * raw_pitch;
    } else {
      smoothed_roll_command_ = raw_roll;
      smoothed_pitch_command_ = raw_pitch;
    }
    const double roll = smoothed_roll_command_;
    const double pitch = smoothed_pitch_command_;
    const double yaw = input[mpc_controller::tpmc::yaw_command];
    const double collective =
        input[mpc_controller::tpmc::collective_specific_force_command];
    const Eigen::Quaterniond quaternion = quaternionFromEuler(roll, pitch, yaw);
    const Eigen::Vector3d force =
        quaternion.toRotationMatrix() * Eigen::Vector3d(0.0, 0.0, collective);
    const Eigen::Vector3d acceleration =
        force +
        Eigen::Vector3d(0.0, 0.0, -solver_configuration_.model.gravity_m_s2);

    SetpointMessage output;
    output.header.stamp = stamp;
    output.header.frame_id = output_frame_id_;
    output.sequence = ++setpoint_sequence_;
    output.desired_acceleration_m_s2 = vectorToArray(acceleration);
    output.desired_specific_force_world_m_s2 = vectorToArray(force);
    output.desired_attitude_wxyz = {quaternion.w(), quaternion.x(),
                                    quaternion.y(), quaternion.z()};
    output.desired_collective_specific_force_m_s2 = collective;
    output.tilt_angle_rad =
        std::acos(std::clamp(quaternion.toRotationMatrix()(2, 2), -1.0, 1.0));
    output.recovery_active = source != SetpointSource::solver;
    setpoint_publisher_->publish(output);
    publication.input = {roll, pitch, yaw, collective};
    publication.published = true;
    return publication;
  }

  std::optional<ModelInput>
  publishRecoverySetpoint(const ModelState &measured,
                          const ModelReference &reference,
                          const rclcpp::Time &stamp) {
    Eigen::Vector3d acceleration;
    acceleration.x() =
        recovery_position_gain_ *
            (reference.state[mpc_controller::tpmc::position_x] -
             measured[mpc_controller::tpmc::position_x]) -
        recovery_velocity_gain_ * measured[mpc_controller::tpmc::velocity_x];
    acceleration.y() =
        recovery_position_gain_ *
            (reference.state[mpc_controller::tpmc::position_y] -
             measured[mpc_controller::tpmc::position_y]) -
        recovery_velocity_gain_ * measured[mpc_controller::tpmc::velocity_y];
    acceleration.z() =
        recovery_position_gain_ *
            (reference.state[mpc_controller::tpmc::position_z] -
             measured[mpc_controller::tpmc::position_z]) -
        recovery_velocity_gain_ * measured[mpc_controller::tpmc::velocity_z];

    const double horizontal_norm =
        std::hypot(acceleration.x(), acceleration.y());
    if (horizontal_norm > recovery_max_acceleration_xy_) {
      const double scale = recovery_max_acceleration_xy_ / horizontal_norm;
      acceleration.x() *= scale;
      acceleration.y() *= scale;
    }
    acceleration.z() =
        std::clamp(acceleration.z(), -recovery_max_acceleration_z_,
                   recovery_max_acceleration_z_);

    mpc_controller::force_attitude::Parameters parameters;
    parameters.gravity_m_s2 = solver_configuration_.model.gravity_m_s2;
    parameters.max_tilt_rad = solver_configuration_.max_tilt_rad;
    mpc_controller::force_attitude::Input input;
    input.desired_acceleration_m_s2 = acceleration;
    input.desired_yaw_rad = reference.state[mpc_controller::tpmc::yaw];
    input.valid = true;
    const auto mapped =
        mpc_controller::force_attitude::compute(parameters, input);
    if (!mapped.valid) {
      return std::nullopt;
    }

    const Eigen::Matrix3d rotation =
        mapped.desired_body_to_world.toRotationMatrix();
    ModelInput recovery_input{std::atan2(rotation(2, 1), rotation(2, 2)),
                              std::asin(std::clamp(-rotation(2, 0), -1.0, 1.0)),
                              reference.state[mpc_controller::tpmc::yaw],
                              mapped.desired_collective_specific_force_m_s2};
    const SetpointPublication publication =
        publishSetpoint(recovery_input, SetpointSource::recovery, stamp);
    if (!publication.published) {
      return std::nullopt;
    }
    return publication.input;
  }

  std::mutex mutex_;
  bool config_valid_ = false;
  bool strict_validation_ = true;
  bool stale_input_active_ = false;
  bool external_mode_active_ = false;
  bool previous_input_initialized_for_external_mode_ = false;
  bool collective_handover_active_ = true;
  double update_rate_hz_ = 50.0;
  double reference_timeout_seconds_ = 1.5;
  double state_timeout_seconds_ = 0.25;
  double recovery_velocity_gain_ = 1.0;
  double recovery_position_gain_ = 0.5;
  double recovery_max_acceleration_xy_ = 2.5;
  double recovery_max_acceleration_z_ = 1.5;
  double command_filter_alpha_ = 0.35;
  double smoothed_roll_command_ = 0.0;
  double smoothed_pitch_command_ = 0.0;
  int collective_handover_valid_samples_ = 5;
  double handover_minimum_duration_seconds_ = 1.0;
  double handover_maximum_yaw_rate_rad_s_ = 0.15;
  double collective_measurement_filter_time_constant_seconds_ = 0.15;
  mpc_controller::state_estimation::CollectiveForceFilter
      collective_force_filter_;
  int collective_handover_valid_count_ = 0;
  rclcpp::Time handover_started_at_{0, 0, RCL_ROS_TIME};
  std::string output_frame_id_ = "map";
  SolverConfiguration solver_configuration_{};
  ModelInput previous_input_{};
  std::unique_ptr<mpc_controller::tpmc::Application> solver_;
  std::optional<ReferenceTrajectory> reference_;
  std::optional<StateMessage> state_;
  std::optional<AppliedCommand> latest_applied_command_;
  std::optional<double> continuous_measured_yaw_;
  rclcpp::Time reference_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time state_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time reference_received_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Time state_received_at_{0, 0, RCL_ROS_TIME};
  std::uint64_t reference_trajectory_id_ = 0;
  std::uint64_t activation_reference_trajectory_id_ = 0;
  std::uint64_t sequence_ = 0;
  std::uint64_t setpoint_sequence_ = 0;
  rclcpp::Subscription<ReferenceMessage>::SharedPtr reference_subscription_;
  rclcpp::Subscription<StateMessage>::SharedPtr state_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
      external_mode_subscription_;
  rclcpp::Subscription<SetpointMessage>::SharedPtr
      applied_setpoint_subscription_;
  rclcpp::Publisher<OutputMessage>::SharedPtr output_publisher_;
  rclcpp::Publisher<SetpointMessage>::SharedPtr setpoint_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MpcControllerNode>());
  rclcpp::shutdown();
  return 0;
}
