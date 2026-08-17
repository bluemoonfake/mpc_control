#include "mpc_controller/msg/mpc_translational_output.hpp"
#include "mpc_controller/msg/force_attitude_setpoint.hpp"
#include "mpc_controller/msg/reference_trajectory.hpp"
#include "mpc_controller/msg/vehicle_state.hpp"
#include "mpc_controller/force_attitude_mapping.hpp"
#include "mpc_controller/translational_mpc.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace
{

// Strict-mode gate for the measured Z acceleration used in x0=[p,v,a].
// It rejects bad samples; it never clamps or filters controller feedback.
struct AccelGate
{
  double max_z = 2.0;

  bool valid() const noexcept
  {
    return std::isfinite(max_z) && max_z > 0.0;
  }

  bool accept(const std::array<double, 3> &a) const noexcept
  {
    return valid() && std::isfinite(a[0]) && std::isfinite(a[1])
      && std::isfinite(a[2]) && std::abs(a[2]) <= max_z;
  }
};

// Model-consistent XY acceleration observer. The plant prediction supplies
// the fast response while the PX4 estimate corrects drift and model error:
//   a- = alpha*a_hat + (1-alpha)*u_previous
//   a_hat = a- + (1-alpha)*(a_measured-a-), alpha=exp(-dt/tau).
// Z stays measured until its separately tuned baseline is revisited.
struct AccelObserver
{
  std::array<double, 3> value{};
  std::array<double, 3> command{};
  int64_t stamp_ns = 0;
  bool initialized = false;

  void reset() noexcept
  {
    value = {};
    command = {};
    stamp_ns = 0;
    initialized = false;
  }

  std::array<double, 3> update(
    const std::array<double, 3> &measured,
    const std::array<double, 3> &time_constant,
    const rclcpp::Time &stamp, double max_gap) noexcept
  {
    const int64_t next_stamp_ns = stamp.nanoseconds();
    if (initialized && next_stamp_ns == stamp_ns) {
      return value;
    }
    const double dt = static_cast<double>(next_stamp_ns - stamp_ns) * 1.0e-9;
    if (!initialized || next_stamp_ns < stamp_ns || !std::isfinite(dt)
      || dt > max_gap) {
      value = measured;
      stamp_ns = next_stamp_ns;
      initialized = true;
      return value;
    }

    for (std::size_t axis = 0; axis < 2; ++axis) {
      const double tau = time_constant[axis];
      if (!(std::isfinite(tau) && tau > 0.0)) {
        value[axis] = measured[axis];
        continue;
      }
      const double alpha = std::exp(-dt / tau);
      const double predicted = alpha * value[axis] + (1.0 - alpha) * command[axis];
      value[axis] = predicted + (1.0 - alpha) * (measured[axis] - predicted);
    }
    value[2] = measured[2];
    stamp_ns = next_stamp_ns;
    return value;
  }

  void setCommand(const std::array<double, 3> &input) noexcept
  {
    command = input;
  }
};

}  // namespace

class MpcControllerNode final : public rclcpp::Node
{
public:
  MpcControllerNode()
  : Node("mpc_controller_node")
  {
    // Parameters are split into solver/model limits, acceleration validation,
    // recovery braking, and force-to-attitude mapping.
    declareAndGet("update_rate_hz", update_rate_hz_);
    declareAndGet("reference_timeout_seconds", reference_timeout_seconds_);
    declareAndGet("state_timeout_seconds", state_timeout_seconds_);
    declareAndGet("strict_validation", strict_validation_);
    declareAndGet("output_frame_id", output_frame_id_);
    declareAndGet("dt_first", config_.dt_first);
    declareAndGet("dt_later", config_.dt_later);
    declareAndGet("solver_deadline_seconds", config_.solver_deadline_seconds);
    declareAndGet("max_iterations", config_.max_iterations);
    declareAndGet("admm_rho", config_.admm_rho);
    declareAndGet("coupled_admm_rho", config_.coupled_admm_rho);
    declareAndGet("solver_absolute_tolerance", config_.solver_absolute_tolerance);
    declareAndGet("solver_relative_tolerance", config_.solver_relative_tolerance);
    declareAndGet("controller_backend", controller_backend_);
    declareAndGet("control_weight_xy", config_.control_weight_xy);
    declareAndGet("control_rate_weight_xy", config_.control_rate_weight_xy);
    declareAndGet("control_weight_z", config_.control_weight_z);
    declareAndGet("control_rate_weight_z", config_.control_rate_weight_z);
    declareAndGet("max_speed_xy", config_.max_speed_xy);
    declareAndGet("max_acceleration_xy", config_.max_acceleration_xy);
    declareAndGet("max_control_xy", config_.max_control_xy);
    declareAndGet("max_control_rate_xy", config_.max_control_rate_xy);
    declareAndGet("max_speed_z", config_.max_speed_z);
    declareAndGet("max_acceleration_z", config_.max_acceleration_z);
    declareAndGet("max_measured_acceleration_z_m_s2", accel_gate_.max_z);
    declareAndGet("max_control_z", config_.max_control_z);
    declareAndGet("max_control_rate_z", config_.max_control_rate_z);
    declareAndGet("min_collective_specific_force_m_s2", config_.min_collective_specific_force_m_s2);
    declareAndGet("max_collective_specific_force_m_s2", config_.max_collective_specific_force_m_s2);
    declareAndGet("constraint_slack_weight", config_.constraint_slack_weight);
    declareAndGet("max_constraint_slack", config_.max_constraint_slack);
    declareAndGet("recovery_velocity_gain", recovery_velocity_gain_);
    declareAndGet("recovery_position_gain_z", recovery_position_gain_z_);
    declareAndGet("recovery_max_acceleration_xy", recovery_max_acceleration_xy_);
    declareAndGet("recovery_max_acceleration_z", recovery_max_acceleration_z_);
    declareAndGet("max_tilt", mapping_config_.max_tilt_rad);

    const bool backend_valid = parseBackend(controller_backend_, config_.backend);
    config_.gravity_m_s2 = mapping_config_.gravity_m_s2;
    config_.max_tilt_rad = mapping_config_.max_tilt_rad;

    // Fixed-size vector parameters use explicit defaults and shape checks.
    declare_parameter("model_time_constant_xyz", std::vector<double>{0.0, 0.0, 0.0});
    declare_parameter("q_xy", std::vector<double>{500.0, 100.0, 100.0});
    declare_parameter("s_xy", std::vector<double>{1000.0, 300.0, 300.0});
    declare_parameter("q_z", std::vector<double>{100.0, 30.0, 10.0});
    declare_parameter("s_z", std::vector<double>{100.0, 30.0, 10.0});
    const bool vector_parameters_valid = getArrayParameter("q_xy", config_.q_xy)
      && getArrayParameter("s_xy", config_.s_xy)
      && getArrayParameter("q_z", config_.q_z)
      && getArrayParameter("s_z", config_.s_z)
      && getArrayParameter("model_time_constant_xyz", config_.model_time_constant_xyz);
    const bool update_rate_valid = std::isfinite(update_rate_hz_) && update_rate_hz_ > 0.0;
    config_valid_ = vector_parameters_valid && backend_valid
      && update_rate_valid
      && std::isfinite(reference_timeout_seconds_) && reference_timeout_seconds_ > 0.0
      && std::isfinite(state_timeout_seconds_) && state_timeout_seconds_ > 0.0
      && !output_frame_id_.empty() && mpc_controller::translational::validConfig(config_)
      && config_.solver_deadline_seconds < 1.0 / update_rate_hz_
      && accel_gate_.valid()
      && std::isfinite(recovery_velocity_gain_) && recovery_velocity_gain_ > 0.0
      && std::isfinite(recovery_position_gain_z_) && recovery_position_gain_z_ >= 0.0
      && std::isfinite(recovery_max_acceleration_xy_) && recovery_max_acceleration_xy_ > 0.0
      && std::isfinite(recovery_max_acceleration_z_) && recovery_max_acceleration_z_ > 0.0;
    mapping_config_valid_ = mpc_controller::force_attitude::validParameters(mapping_config_);

    // The controller consumes measured state and sampled reference streams,
    // then publishes solver diagnostics and a compact force/attitude setpoint.
    reference_subscription_ = create_subscription<Reference>(
      "reference_trajectory", 10,
      [this](Reference::SharedPtr message) {referenceCallback(std::move(message));});
    state_subscription_ = create_subscription<State>(
      "vehicle_state", 10,
      [this](State::SharedPtr message) {stateCallback(std::move(message));});
    output_publisher_ = create_publisher<Output>("mpc_translational_output", 10);
    setpoint_publisher_ = create_publisher<Setpoint>("force_attitude_setpoint", 10);

    if (!config_valid_) {
      RCLCPP_ERROR(get_logger(), "Invalid MPC configuration; controller updates disabled");
      return;
    }
    if (!mapping_config_valid_) {
      RCLCPP_ERROR(get_logger(), "Invalid force/attitude control configuration; output disabled");
    }

    controller_.emplace(config_);
    RCLCPP_INFO(
      get_logger(), "MPC backend selected at startup: %s", controller_backend_.c_str());
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / update_rate_hz_)),
      std::bind(&MpcControllerNode::update, this));
  }

private:
  template <typename T>
  void declareAndGet(const std::string &name, T &target)
  {
    declare_parameter(name, target);
    get_parameter(name, target);
  }

  using Reference = mpc_controller::msg::ReferenceTrajectory;
  using State = mpc_controller::msg::VehicleState;
  using Output = mpc_controller::msg::MpcTranslationalOutput;
  using Setpoint = mpc_controller::msg::ForceAttitudeSetpoint;
  using ReferenceData = mpc_controller::translational::ReferenceTrajectoryData;
  using ReferencePoint = mpc_controller::translational::ReferencePoint;
  using MeasuredState = mpc_controller::translational::MeasuredState;
  using Horizon = mpc_controller::translational::ReferenceHorizon;
  using Sampler = mpc_controller::translational::ReferenceSampler;
  using Controller = mpc_controller::translational::TranslationalMpc;
  using FailureReason = mpc_controller::translational::FailureReason;

  static bool parseBackend(
    const std::string & /*name*/, mpc_controller::translational::Backend &backend) noexcept
  {
    backend = mpc_controller::translational::Backend::coupled;
    return true;
  }

  static double durationSeconds(const builtin_interfaces::msg::Duration &duration) noexcept
  {
    return static_cast<double>(duration.sec) + static_cast<double>(duration.nanosec) * 1.0e-9;
  }

  bool getArrayParameter(const std::string &name, std::array<double, 3> &output)
  {
    const auto values = get_parameter(name).as_double_array();
    if (values.size() != output.size()) {
      RCLCPP_ERROR(get_logger(), "Parameter '%s' must contain exactly 3 values", name.c_str());
      config_valid_ = false;
      return false;
    }
    std::copy(values.begin(), values.end(), output.begin());
    return true;
  }

  static bool convertReference(const Reference &message, ReferenceData &output) noexcept
  {
    const rclcpp::Time stamp(message.header.stamp);
    output.header_time_seconds = stamp.seconds();
    output.hold_after_end = message.hold_after_end;
    output.points.clear();
    output.points.reserve(message.points.size());
    for (const auto &input : message.points) {
      ReferencePoint point;
      point.time_from_start = durationSeconds(input.time_from_start);
      point.position = input.position;
      point.velocity = input.velocity;
      point.acceleration = input.acceleration;
      point.yaw = input.yaw;
      point.yaw_rate = input.yaw_rate;
      output.points.push_back(point);
    }
    return Sampler::validTrajectory(output);
  }

  void referenceCallback(Reference::SharedPtr message)
  {
    if (!message) {
      return;
    }
    if (message->header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "MPC reference rejected: frame_id is empty");
      return;
    }
    ReferenceData converted;
    if ((message->header.stamp.sec == 0 && message->header.stamp.nanosec == 0)
      || !convertReference(*message, converted)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "MPC reference rejected: malformed, empty, zero-timestamp or non-finite trajectory");
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    const rclcpp::Time stamp(message->header.stamp);
    if (reference_stamp_.nanoseconds() != 0 && stamp < reference_stamp_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "MPC reference rejected: timestamp moved backwards");
      return;
    }
    reference_ = std::move(converted);
    reference_stamp_ = stamp;
    reference_trajectory_id_ = message->trajectory_id;
    reference_received_at_ = get_clock()->now();
  }

  void stateCallback(State::SharedPtr message)
  {
    if (!message) {
      return;
    }
    const rclcpp::Time stamp(message->header.stamp);
    if (stamp.nanoseconds() == 0) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_stamp_.nanoseconds() != 0 && stamp < state_stamp_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "MPC measured state rejected: timestamp moved backwards");
      return;
    }
    state_ = std::move(*message);
    state_stamp_ = stamp;
    state_received_at_ = get_clock()->now();
  }

  void update()
  {
    // One callback performs admission, horizon sampling, the selected MPC
    // backend, bounded recovery on failure, and force/attitude conversion.
    std::lock_guard<std::mutex> lock(mutex_);
    if (!config_valid_ || !controller_ || !reference_ || !state_) {
      return;
    }
    const auto now = get_clock()->now();
    const double reference_age = (now - reference_received_at_).seconds();
    const double state_age = (now - state_received_at_).seconds();
    const bool stale_input = !std::isfinite(reference_age) || reference_age < 0.0
      || reference_age > reference_timeout_seconds_
      || !std::isfinite(state_age) || state_age < 0.0 || state_age > state_timeout_seconds_;
    if (stale_input) {
      if (!stale_input_active_) {
        // A stale interval breaks the continuity assumed by the warm start and
        // by the first-input rate constraint.  Restart from neutral acceleration
        // when fresh measurements return instead of reusing the pre-gap state.
        controller_->reset();
        accel_observer_.reset();
        stale_input_active_ = true;
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "MPC update rejected: stale input: reference_age=%.3f s (limit %.3f s), "
        "state_age=%.3f s (limit %.3f s)",
        reference_age, reference_timeout_seconds_, state_age, state_timeout_seconds_);
      return;
    }
    if (stale_input_active_) {
      RCLCPP_INFO(
        get_logger(),
        "MPC input recovered; solver warm start and input-rate memory were reset: "
        "reference_age=%.3f s, state_age=%.3f s",
        reference_age, state_age);
      stale_input_active_ = false;
    }
    if (strict_validation_ && (!state_->valid || !state_->position_valid || !state_->velocity_valid
      || !state_->acceleration_valid || state_->header.frame_id.empty())) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "MPC update rejected: measured translational state is invalid");
      return;
    }
    MeasuredState measured;
    measured.position = state_->position;
    measured.velocity = state_->velocity;
    const std::array<double, 3> raw_acceleration = state_->acceleration;
    measured.acceleration = raw_acceleration;
    if (!mpc_controller::translational::finite(measured.position)
      || !mpc_controller::translational::finite(measured.velocity)
      || !mpc_controller::translational::finite(measured.acceleration)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "MPC update rejected: measured state contains NaN or Inf");
      return;
    }
    if (strict_validation_) {
      if (!accel_gate_.accept(raw_acceleration)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "MPC update rejected: acceleration is non-finite or |az|=%.3f exceeds %.3f m/s^2",
          std::abs(raw_acceleration[2]), accel_gate_.max_z);
        return;
      }
    }
    measured.acceleration = accel_observer_.update(
      raw_acceleration, config_.model_time_constant_xyz, state_stamp_, state_timeout_seconds_);

    const double elapsed = now.seconds() - reference_->header_time_seconds;
    Horizon horizon;
    if (!Sampler::buildHorizon(
        *reference_, elapsed, config_.dt_first, config_.dt_later, horizon)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "MPC update rejected: reference cannot be sampled on the solver grid");
      return;
    }

    const auto result = controller_->update(measured, horizon);
    Output output;
    output.header.stamp = now;
    output.header.frame_id = output_frame_id_;
    output.trajectory_id = reference_trajectory_id_;
    output.sequence = ++sequence_;
    output.measured_position = measured.position;
    output.measured_velocity = measured.velocity;
    output.measured_acceleration = measured.acceleration;
    output.sampled_reference_position = horizon.points.front().position;
    output.sampled_reference_velocity = horizon.points.front().velocity;
    output.sampled_reference_acceleration = horizon.points.front().acceleration;
    output.first_predicted_acceleration = result.first_predicted_acceleration;
    output.solve_time_seconds = result.solve_time_seconds;
    output.solver_deadline_missed = result.deadline_missed;
    output.failure_reason = static_cast<uint8_t>(result.failure_reason);
    output.active_backend = static_cast<uint8_t>(config_.backend);
    for (std::size_t axis = 0; axis < 3; ++axis) {
      output.solver_iterations[axis] = result.coupled.iterations;
      output.solver_status[axis] = static_cast<uint8_t>(result.coupled.status);
      output.solver_primal_residual[axis] = result.coupled.primal_residual;
      output.solver_dual_residual[axis] = result.coupled.dual_residual;
      output.solver_primal_tolerance[axis] = result.coupled.primal_tolerance;
      output.solver_dual_tolerance[axis] = result.coupled.dual_tolerance;
      output.recovery_constraint_active[axis] = result.coupled.recovery_constraint_active;
    }
    output.coupled_solver_ran = result.coupled_solver_ran;
    output.coupled_solver_valid = result.coupled.valid;
    output.coupled_control_active = result.coupled_control_active;
    output.coupled_solver_iterations = result.coupled.iterations;
    output.coupled_solver_status = static_cast<uint8_t>(result.coupled.status);
    output.coupled_primal_residual = result.coupled.primal_residual;
    output.coupled_dual_residual = result.coupled.dual_residual;
    output.coupled_primal_tolerance = result.coupled.primal_tolerance;
    output.coupled_dual_tolerance = result.coupled.dual_tolerance;
    output.coupled_velocity_slack = result.coupled.max_velocity_slack;
    output.coupled_acceleration_slack = result.coupled.max_acceleration_slack;
    output.coupled_max_constraint_violation = result.coupled.max_constraint_violation;
    output.coupled_max_speed_xy = result.coupled.max_predicted_speed_xy;
    output.coupled_max_acceleration_xy = result.coupled.max_predicted_acceleration_xy;
    output.coupled_max_tilt_rad = result.coupled.max_predicted_tilt_rad;
    output.coupled_max_collective_specific_force_m_s2 =
      result.coupled.max_predicted_collective_specific_force_m_s2;
    output.coupled_solve_time_seconds = result.coupled_solve_time_seconds;
    output.shadow_control_difference_norm = result.shadow_control_difference_norm;
    updateShadowAdmission(result);
    output.shadow_admission_samples = shadow_admission_samples_;
    output.shadow_admission_ready = shadow_admission_ready_;
    ++solve_count_;
    solve_time_total_seconds_ += result.solve_time_seconds;
    solve_time_max_seconds_ = std::max(solve_time_max_seconds_, result.solve_time_seconds);
    if (!result.valid) {
      const auto recovery = recoveryAcceleration(measured, horizon.points.front());
      accel_observer_.setCommand(recovery);
      output.control_input = recovery;
      output.recovery_command_active = true;
      output.valid = false;
      output_publisher_->publish(output);
      publishSetpoint(output.header, output.sequence, recovery, state_->yaw, true);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "MPC failed: %s coupled_status=%u "
        "iter=%d primal=%.3g dual=%.3g solve=%.3f ms; "
        "recovery a=[%.2f %.2f %.2f]",
        failureReasonString(result.failure_reason),
        static_cast<unsigned>(result.coupled.status),
        result.coupled.iterations,
        result.coupled.primal_residual,
        result.coupled.dual_residual,
        result.solve_time_seconds * 1000.0,
        recovery[0], recovery[1], recovery[2]);
      return;
    }

    output.predicted_states.resize(
      mpc_controller::translational::kHorizonLength * 9);
    for (std::size_t step = 0; step < mpc_controller::translational::kHorizonLength; ++step) {
      for (std::size_t axis = 0; axis < 3; ++axis) {
        for (std::size_t state = 0; state < 3; ++state) {
          output.predicted_states[step * 9 + axis * 3 + state] =
            result.coupled.prediction[step](3 * state + axis);
        }
      }
    }
    output.recovery_command_active = false;
    output.valid = true;

    const auto &command_reference = horizon.points.front();
    output.control_input = result.control;
    accel_observer_.setCommand(result.control);
    output_publisher_->publish(output);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "MPC update seq=%lu measured_x0=[p %.3f %.3f %.3f v %.3f %.3f %.3f "
      "a_hat %.3f %.3f %.3f a_raw %.3f %.3f %.3f] "
      "u=[%.3f %.3f %.3f] ref_age=%.1f ms state_age=%.1f ms "
      "solve=%.3f ms mean=%.3f ms max=%.3f ms coupled_valid=%s "
      "tilt_max=%.3f thrust_max=%.3f",
      static_cast<unsigned long>(output.sequence),
      measured.position[0], measured.position[1], measured.position[2],
      measured.velocity[0], measured.velocity[1], measured.velocity[2],
      measured.acceleration[0], measured.acceleration[1], measured.acceleration[2],
      raw_acceleration[0], raw_acceleration[1], raw_acceleration[2],
      result.control[0], result.control[1], result.control[2],
      reference_age * 1.0e3, state_age * 1.0e3,
      output.solve_time_seconds * 1000.0,
      (solve_time_total_seconds_ / static_cast<double>(solve_count_)) * 1000.0,
      solve_time_max_seconds_ * 1000.0,
      result.coupled.valid ? "true" : "false",
      result.coupled.max_predicted_tilt_rad,
      result.coupled.max_predicted_collective_specific_force_m_s2);
    publishSetpoint(
      output.header, output.sequence, result.control, command_reference.yaw, false);
  }

  std::array<double, 3> recoveryAcceleration(
    const MeasuredState &measured, const ReferencePoint &reference) const noexcept
  {
    // Bounded fallback: a_xy=-k_v*v_xy and
    // a_z=k_p(z_ref-z)-k_v*v_z. It brakes rather than holding the last tilt.
    std::array<double, 3> command{
      -recovery_velocity_gain_ * measured.velocity[0],
      -recovery_velocity_gain_ * measured.velocity[1],
      recovery_position_gain_z_ * (reference.position[2] - measured.position[2])
        - recovery_velocity_gain_ * measured.velocity[2]};
    const double horizontal = std::hypot(command[0], command[1]);
    if (horizontal > recovery_max_acceleration_xy_) {
      const double scale = recovery_max_acceleration_xy_ / horizontal;
      command[0] *= scale;
      command[1] *= scale;
    }
    command[2] = std::clamp(
      command[2], -recovery_max_acceleration_z_, recovery_max_acceleration_z_);
    return command;
  }

  void updateShadowAdmission(
    const mpc_controller::translational::UpdateResult &result) noexcept
  {
    if (!result.coupled_solver_ran) {
      shadow_admission_samples_ = 0;
      shadow_admission_ready_ = false;
      return;
    }
    const bool stable = result.coupled.valid && !result.deadline_missed
      && std::isfinite(result.coupled_solve_time_seconds)
      && result.coupled_solve_time_seconds <= config_.solver_deadline_seconds
      && std::isfinite(result.coupled.max_constraint_violation)
      && result.coupled.max_constraint_violation
      <= 10.0 * (config_.solver_absolute_tolerance + config_.solver_relative_tolerance)
      && result.coupled.max_predicted_tilt_rad <= config_.max_tilt_rad + 1.0e-6
      && result.coupled.max_predicted_collective_specific_force_m_s2
      <= config_.max_collective_specific_force_m_s2 + 1.0e-6;
    if (!stable && shadow_admission_samples_ > 0U) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Coupled shadow admission reset: valid=%s deadline=%s status=%u iter=%d "
        "solve=%.3f ms primal=%.3g/%.3g dual=%.3g/%.3g violation=%.3g "
        "tilt=%.3f thrust=%.3f samples=%u",
        result.coupled.valid ? "true" : "false",
        result.deadline_missed ? "true" : "false",
        static_cast<unsigned>(result.coupled.status), result.coupled.iterations,
        result.coupled_solve_time_seconds * 1.0e3,
        result.coupled.primal_residual, result.coupled.primal_tolerance,
        result.coupled.dual_residual, result.coupled.dual_tolerance,
        result.coupled.max_constraint_violation,
        result.coupled.max_predicted_tilt_rad,
        result.coupled.max_predicted_collective_specific_force_m_s2,
        shadow_admission_samples_);
    }
    shadow_admission_samples_ = stable ? std::min(
      shadow_admission_samples_ + 1U, shadow_admission_required_samples_) : 0U;
    shadow_admission_ready_ = shadow_admission_samples_ >= shadow_admission_required_samples_;
  }

  void publishSetpoint(
    const std_msgs::msg::Header &header, uint64_t sequence,
    const std::array<double, 3> &acceleration,
    double yaw, bool recovery)
  {
    // Convert desired ENU acceleration and yaw into desired SO(3) attitude
    // plus mass-normalized collective force for the 250 Hz adapter.
    if (!mapping_config_valid_ || !state_) return;
    mpc_controller::force_attitude::Input mapping_input;
    mapping_input.desired_acceleration_m_s2 = Eigen::Vector3d(
      acceleration[0], acceleration[1], acceleration[2]);
    mapping_input.desired_yaw_rad = std::isfinite(yaw) ? yaw : state_->yaw;
    mapping_input.valid = strict_validation_ ? state_->attitude_valid : true;
    const auto mapping = mpc_controller::force_attitude::compute(mapping_config_, mapping_input);
    if (!mapping.valid) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Force/attitude output rejected: %s",
        mpc_controller::force_attitude::failureReasonName(mapping.failure_reason));
      return;
    }

    Setpoint setpoint;
    setpoint.header = header;
    setpoint.sequence = sequence;
    for (int i = 0; i < 3; ++i) {
      setpoint.desired_acceleration_m_s2[i] = mapping.desired_acceleration_m_s2[i];
      setpoint.desired_specific_force_world_m_s2[i] =
        mapping.desired_specific_force_world_m_s2[i];
    }
    setpoint.desired_attitude_wxyz = {
      mapping.desired_body_to_world.w(), mapping.desired_body_to_world.x(),
      mapping.desired_body_to_world.y(), mapping.desired_body_to_world.z()};
    setpoint.desired_collective_specific_force_m_s2 =
      mapping.desired_collective_specific_force_m_s2;
    setpoint.tilt_angle_rad = mapping.tilt_angle_rad;
    setpoint.recovery_active = recovery;
    setpoint_publisher_->publish(setpoint);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Force/attitude setpoint seq=%lu a_des=[%.3f %.3f %.3f] "
      "specific_force=[%.3f %.3f %.3f] "
      "collective=%.3f m/s^2 tilt=%.3f rad mode=%s",
      static_cast<unsigned long>(setpoint.sequence),
      setpoint.desired_acceleration_m_s2[0], setpoint.desired_acceleration_m_s2[1],
      setpoint.desired_acceleration_m_s2[2],
      setpoint.desired_specific_force_world_m_s2[0],
      setpoint.desired_specific_force_world_m_s2[1],
      setpoint.desired_specific_force_world_m_s2[2],
      setpoint.desired_collective_specific_force_m_s2,
      setpoint.tilt_angle_rad, recovery ? "recovery" : "tracking");
  }

  static const char * failureReasonString(FailureReason reason) noexcept
  {
    switch (reason) {
      case FailureReason::invalid_configuration: return "invalid configuration";
      case FailureReason::invalid_measured_state: return "invalid measured state";
      case FailureReason::invalid_reference: return "invalid reference";
      case FailureReason::solver_not_converged: return "solver not converged";
      case FailureReason::non_finite_solver_output: return "non-finite solver output";
      case FailureReason::none: return "none";
    }
    return "unknown";
  }

  std::mutex mutex_;
  bool config_valid_ = true;
  bool mapping_config_valid_ = true;
  bool stale_input_active_ = false;
  bool strict_validation_ = true;
  double update_rate_hz_ = 50.0;
  double reference_timeout_seconds_ = 1.5;
  double state_timeout_seconds_ = 0.25;
  double recovery_velocity_gain_ = 1.0;
  double recovery_position_gain_z_ = 0.5;
  double recovery_max_acceleration_xy_ = 2.5;
  double recovery_max_acceleration_z_ = 1.5;
  std::string controller_backend_ = "coupled_shadow";
  std::string output_frame_id_ = "map";
  mpc_controller::translational::Config config_{};
  AccelGate accel_gate_{};
  AccelObserver accel_observer_{};
  mpc_controller::force_attitude::Parameters mapping_config_{};
  std::optional<Controller> controller_;
  std::optional<ReferenceData> reference_;
  std::optional<State> state_;
  rclcpp::Time reference_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time state_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time reference_received_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Time state_received_at_{0, 0, RCL_ROS_TIME};
  uint64_t reference_trajectory_id_ = 0;
  uint64_t sequence_ = 0;
  uint64_t solve_count_ = 0;
  uint32_t shadow_admission_samples_ = 0;
  static constexpr uint32_t shadow_admission_required_samples_ = 250;
  bool shadow_admission_ready_ = false;
  double solve_time_total_seconds_ = 0.0;
  double solve_time_max_seconds_ = 0.0;
  rclcpp::Subscription<Reference>::SharedPtr reference_subscription_;
  rclcpp::Subscription<State>::SharedPtr state_subscription_;
  rclcpp::Publisher<Output>::SharedPtr output_publisher_;
  rclcpp::Publisher<Setpoint>::SharedPtr setpoint_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MpcControllerNode>());
  rclcpp::shutdown();
  return 0;
}
