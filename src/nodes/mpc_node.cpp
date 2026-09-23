#include "mpc_controller/application/control_runtime.hpp"
#include "mpc_controller/msg/force_attitude_setpoint.hpp"
#include "mpc_controller/msg/mpc_translational_output.hpp"
#include "mpc_controller/msg/reference_trajectory.hpp"
#include "mpc_controller/msg/vehicle_state.hpp"
#include "mpc_controller/adapters/geometry_mapper.hpp"

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

namespace {

// Strict-mode gate for the measured Z acceleration used in x0=[p,v,a].
// It rejects bad samples; it never clamps or filters controller feedback.
struct AccelGate {
  double max_z = 2.0;

  bool valid() const noexcept { return std::isfinite(max_z) && max_z > 0.0; }

  bool accept(const std::array<double, 3> &a) const noexcept {
    return valid() && std::isfinite(a[0]) && std::isfinite(a[1]) && std::isfinite(a[2]) && std::abs(a[2]) <= max_z;
  }
};

// Model-consistent XY acceleration observer. The plant prediction supplies
// the fast response while the PX4 estimate corrects drift and model error:
//   a- = alpha*a_hat + (1-alpha)*u_previous
//   a_hat = a- + (1-alpha)*(a_measured-a-), alpha=exp(-dt/tau).
// Z stays measured until its separately tuned baseline is revisited.
struct AccelObserver {
  std::array<double, 3> value{};
  std::array<double, 3> command{};
  int64_t stamp_ns = 0;
  bool initialized = false;

  void reset() noexcept {
    value = {};
    command = {};
    stamp_ns = 0;
    initialized = false;
  }

  std::array<double, 3> update(const std::array<double, 3> &measured,
                               const std::array<double, 3> &time_constant,
                               const rclcpp::Time &stamp, double max_gap) noexcept {
    const int64_t next_stamp_ns = stamp.nanoseconds();
    if (initialized && next_stamp_ns == stamp_ns) {
      return value;
    }
    const double dt = static_cast<double>(next_stamp_ns - stamp_ns) * 1.0e-9;
    if (!initialized || next_stamp_ns < stamp_ns || !std::isfinite(dt) || dt > max_gap) {
      value = measured;
      stamp_ns = next_stamp_ns;
      initialized = true;
      return value;
    }
    // acceleration observer
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

  void setCommand(const std::array<double, 3> &input) noexcept {
    command = input;
  }
};

} // namespace

class MpcControllerNode final : public rclcpp::Node {
public:
  MpcControllerNode() : Node("mpc_controller_node") {
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
    declareAndGet("solver_absolute_tolerance", config_.solver_absolute_tolerance);
    declareAndGet("solver_relative_tolerance", config_.solver_relative_tolerance);
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
    declareAndGet("recovery_velocity_gain", recovery_velocity_gain_);
    declareAndGet("recovery_position_gain_z", recovery_position_gain_z_);
    declareAndGet("recovery_max_acceleration_xy", recovery_max_acceleration_xy_);
    declareAndGet("recovery_max_acceleration_z", recovery_max_acceleration_z_);
    declareAndGet("max_tilt", mapping_config_.max_tilt_rad);

    config_.gravity_m_s2 = mapping_config_.gravity_m_s2;
    config_.max_tilt_rad = mapping_config_.max_tilt_rad;

    // Fixed-size vector parameters use explicit defaults and shape checks.
    declare_parameter("model_time_constant_xyz", std::vector<double>{0.0, 0.0, 0.0});
    declare_parameter("q_xy", std::vector<double>{500.0, 100.0, 100.0});
    declare_parameter("s_xy", std::vector<double>{1000.0, 300.0, 300.0});
    declare_parameter("q_z", std::vector<double>{100.0, 30.0, 10.0});
    declare_parameter("s_z", std::vector<double>{100.0, 30.0, 10.0});
    const bool vector_parameters_valid =
        getArrayParameter("q_xy", config_.q_xy) &&
        getArrayParameter("s_xy", config_.s_xy) &&
        getArrayParameter("q_z", config_.q_z) &&
        getArrayParameter("s_z", config_.s_z) &&
        getArrayParameter("model_time_constant_xyz", config_.model_time_constant_xyz);
    const bool update_rate_valid = std::isfinite(update_rate_hz_) && update_rate_hz_ > 0.0;
    config_valid_ = vector_parameters_valid && update_rate_valid &&
        std::isfinite(reference_timeout_seconds_) &&
        reference_timeout_seconds_ > 0.0 &&
        std::isfinite(state_timeout_seconds_) && state_timeout_seconds_ > 0.0 &&
        !output_frame_id_.empty() &&
        mpc_controller::translational::validConfig(config_) &&
        config_.solver_deadline_seconds < 1.0 / update_rate_hz_ &&
        accel_gate_.valid() && std::isfinite(recovery_velocity_gain_) &&
        recovery_velocity_gain_ > 0.0 &&
        std::isfinite(recovery_position_gain_z_) &&
        recovery_position_gain_z_ >= 0.0 &&
        std::isfinite(recovery_max_acceleration_xy_) &&
        recovery_max_acceleration_xy_ > 0.0 &&
        std::isfinite(recovery_max_acceleration_z_) &&
        recovery_max_acceleration_z_ > 0.0;

    mapping_config_valid_ = mpc_controller::force_attitude::validParameters(mapping_config_);

    // The controller consumes measured state and sampled reference streams,
    // then publishes solver diagnostics and a compact force/attitude setpoint.
    reference_subscription_ = create_subscription<Reference>(
        "reference_trajectory", 10, [this](Reference::SharedPtr message) {
          referenceCallback(std::move(message));
        });
    state_subscription_ = create_subscription<State>(
        "vehicle_state", 10, [this](State::SharedPtr message) {
          stateCallback(std::move(message));
        });
    output_publisher_ = create_publisher<Output>("mpc_translational_output", 10);
    setpoint_publisher_ = create_publisher<Setpoint>("force_attitude_setpoint", 10);

    if (!config_valid_) {
      RCLCPP_ERROR(get_logger(), "Invalid MPC configuration; controller updates disabled");
      return;
    }
    if (!mapping_config_valid_) {
      RCLCPP_ERROR(get_logger(), "Invalid force/attitude control configuration; output disabled");
    }

    controller_.emplace(config_, mpc_controller::application::RecoveryConfig{
                                recovery_velocity_gain_, recovery_position_gain_z_,
                                recovery_max_acceleration_xy_, recovery_max_acceleration_z_});
    RCLCPP_INFO(get_logger(), "HPIPM coupled MPC initialized");
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / update_rate_hz_)), std::bind(&MpcControllerNode::update, this));
  }

private:
  template <typename T> void declareAndGet(const std::string &name, T &target) {
    declare_parameter(name, target);
    get_parameter(name, target);
  }

  using Reference = mpc_controller::msg::ReferenceTrajectory;
  using State = mpc_controller::msg::VehicleState;
  using Output = mpc_controller::msg::MpcTranslationalOutput;
  using Setpoint = mpc_controller::msg::ForceAttitudeSetpoint;
  using ReferenceData = mpc_controller::translational::ReferenceTrajectoryData;
  using ValidatedReference = mpc_controller::translational::ValidatedTrajectory;
  using ReferencePoint = mpc_controller::translational::ReferencePoint;
  using MeasuredState = mpc_controller::translational::MeasuredState;
  using Horizon = mpc_controller::translational::ReferenceHorizon;
  using Sampler = mpc_controller::translational::ReferenceSampler;
  using FailureReason = mpc_controller::translational::FailureReason;

  static double
  durationSeconds(const builtin_interfaces::msg::Duration &duration) noexcept {
    return static_cast<double>(duration.sec) + static_cast<double>(duration.nanosec) * 1.0e-9;
  }

  bool getArrayParameter(const std::string &name, std::array<double, 3> &output) {
    const auto values = get_parameter(name).as_double_array();
    if (values.size() != output.size()) {
      RCLCPP_ERROR(get_logger(), "Parameter '%s' must contain exactly 3 values",name.c_str());
      config_valid_ = false;
      return false;
    }
    std::copy(values.begin(), values.end(), output.begin());
    return true;
  }

  static bool convertReference(const Reference &message, ValidatedReference &output) {
    const rclcpp::Time stamp(message.header.stamp);
    ReferenceData converted;
    converted.header_time_seconds = stamp.seconds();
    converted.hold_after_end = message.hold_after_end;
    converted.limits.max_speed_xy = message.max_speed_xy;
    converted.limits.max_speed_z = message.max_speed_z;
    converted.limits.max_acceleration_xy = message.max_acceleration_xy;
    converted.limits.max_acceleration_z = message.max_acceleration_z;
    converted.limits.max_control_rate_xy = message.max_control_rate_xy;
    converted.limits.max_control_rate_z = message.max_control_rate_z;
    converted.points.reserve(message.points.size());
    for (const auto &input : message.points) {
      ReferencePoint point;
      point.time_from_start = durationSeconds(input.time_from_start);
      point.position = input.position;
      point.velocity = input.velocity;
      point.acceleration = input.acceleration;
      point.yaw = input.yaw;
      point.yaw_rate = input.yaw_rate;
      converted.points.push_back(point);
    }
    return output.assign(std::move(converted));
  }

  void referenceCallback(Reference::SharedPtr message) {
    if (!message) {
      return;
    }
    if (message->header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "MPC reference rejected: frame_id is empty");
      return;
    }
    ValidatedReference converted;
    if ((message->header.stamp.sec == 0 && message->header.stamp.nanosec == 0) || !convertReference(*message, converted)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "MPC reference rejected: ");
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    const rclcpp::Time stamp(message->header.stamp);
    if (reference_stamp_.nanoseconds() != 0 && stamp < reference_stamp_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "MPC reference rejected: timestamp moved backwards");
      return;
    }
    reference_ = std::move(converted);
    reference_stamp_ = stamp;
    reference_trajectory_id_ = message->trajectory_id;
    reference_received_at_ = get_clock()->now();
  }

  void stateCallback(State::SharedPtr message) {
    if (!message) {
      return;
    }
    const rclcpp::Time stamp(message->header.stamp);
    if (stamp.nanoseconds() == 0) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_stamp_.nanoseconds() != 0 && stamp < state_stamp_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "MPC measured state rejected: timestamp moved backwards");
      return;
    }
    state_ = std::move(*message);
    state_stamp_ = stamp;
    state_received_at_ = get_clock()->now();
  }

  void update() {
    // One callback performs admission, horizon sampling, the selected MPC
    // backend, bounded recovery on failure, and force/attitude conversion.
    const auto callback_start = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(mutex_);
    const auto mutex_acquired = std::chrono::steady_clock::now();
    const double mutex_wait_seconds = std::chrono::duration<double>(mutex_acquired - callback_start).count();
    const double callback_period_seconds = callback_timing_initialized_ ? std::chrono::duration<double>(callback_start - last_callback_start_).count() : 0.0;
    last_callback_start_ = callback_start;
    callback_timing_initialized_ = true;
    if (!config_valid_ || !controller_ || !state_) {
      return;
    }
    if (!reference_) {
      return;
    }
    const auto now = get_clock()->now();
    const double reference_age = (now - reference_received_at_).seconds();
    const double state_age = (now - state_received_at_).seconds();
    const bool stale_input = !std::isfinite(reference_age) || reference_age < 0.0 || reference_age > reference_timeout_seconds_ ||
                             !std::isfinite(state_age) || state_age < 0.0 || state_age > state_timeout_seconds_;
    if (stale_input) {
      if (!stale_input_active_) {
        // A stale interval breaks the continuity assumed by the warm start and by the first-input rate constraint.
        controller_->reset();
        accel_observer_.reset();
        stale_input_active_ = true;
      }
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "MPC update rejected: stale input: reference_age=%.3f s (limit %.3f s), state_age=%.3f s (limit %.3f s)",
          reference_age, reference_timeout_seconds_, state_age, state_timeout_seconds_);
      return;
    }
    if (stale_input_active_) {
      RCLCPP_INFO(get_logger(),
                  "Input-rate memory were reset: reference_age=%.3f s, state_age=%.3f s",reference_age, state_age);
      stale_input_active_ = false;
    }
    if (strict_validation_ &&
        (!state_->valid || !state_->position_valid || !state_->velocity_valid ||
         !state_->acceleration_valid || state_->header.frame_id.empty())) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "MPC update rejected: measured translational state is invalid");
      return;
    }
    MeasuredState measured;
    measured.position = state_->position;
    measured.velocity = state_->velocity;
    const std::array<double, 3> raw_acceleration = state_->acceleration;
    measured.acceleration = raw_acceleration;
    if (!mpc_controller::translational::finite(measured.position) ||
        !mpc_controller::translational::finite(measured.velocity) ||
        !mpc_controller::translational::finite(measured.acceleration)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "MPC update rejected: non-finite values");
      return;
    }
    if (strict_validation_) {
      if (!accel_gate_.accept(raw_acceleration)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "MPC update rejected: acceleration is non-finite or |az|=%.3f exceeds %.3f m/s^2",
                             std::abs(raw_acceleration[2]), accel_gate_.max_z);
        return;
      }
    }
    measured.acceleration = accel_observer_.update(raw_acceleration, config_.model_time_constant_xyz, state_stamp_, state_timeout_seconds_);

    const double elapsed = now.seconds() - reference_->get().header_time_seconds;
    Horizon horizon;
    if (!Sampler::buildHorizon(*reference_, elapsed, config_.dt_first, config_.dt_later, horizon)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "MPC update rejected: reference cannot be sampled on the solver grid");
      return;
    }
    const ReferencePoint command_reference = horizon.points.front();
    const auto runtime = controller_->update(measured, horizon, command_reference);
    const auto &result = runtime.result;
    Output output;
    output.header.stamp = now;
    output.header.frame_id = output_frame_id_;
    output.trajectory_id = reference_trajectory_id_;
    output.sequence = ++sequence_;
    output.measured_position = measured.position;
    output.measured_velocity = measured.velocity;
    output.measured_acceleration = measured.acceleration;
    output.sampled_reference_position = command_reference.position;
    output.sampled_reference_velocity = command_reference.velocity;
    output.sampled_reference_acceleration = command_reference.acceleration;
    output.first_predicted_acceleration = result.first_predicted_acceleration;
    output.solve_time_seconds = result.solve_time_seconds;
    output.solver_deadline_missed = result.deadline_missed;
    output.failure_reason = static_cast<uint8_t>(result.failure_reason);
    output.coupled_solver_ran = result.coupled_solver_ran;
    output.coupled_solver_valid = result.coupled.valid;
    output.coupled_control_active = result.coupled_control_active;
    output.coupled_solver_iterations = result.coupled.iterations;
    output.coupled_solver_status = static_cast<uint8_t>(result.coupled.status);
    output.coupled_primal_residual = result.coupled.primal_residual;
    output.coupled_dual_residual = result.coupled.dual_residual;
    output.coupled_primal_tolerance = result.coupled.primal_tolerance;
    output.coupled_dual_tolerance = result.coupled.dual_tolerance;
    output.coupled_max_constraint_violation = result.coupled.max_constraint_violation;
    output.coupled_max_speed_xy = result.coupled.max_predicted_speed_xy;
    output.coupled_max_acceleration_xy = result.coupled.max_predicted_acceleration_xy;
    output.coupled_max_tilt_rad = result.coupled.max_predicted_tilt_rad;
    output.coupled_max_collective_specific_force_m_s2 = result.coupled.max_predicted_collective_specific_force_m_s2;
    output.coupled_solve_time_seconds = result.coupled_solve_time_seconds;
    output.problem_update_seconds = result.problem_update_seconds;
    output.vector_copy_seconds = result.vector_copy_seconds;
    output.warm_start_seconds = result.warm_start_seconds;
    output.warm_start_prepare_seconds = result.warm_start_prepare_seconds;
    output.solve_seconds = result.solve_seconds;
    output.result_postprocess_seconds = result.result_postprocess_seconds;
    output.remaining_budget_seconds = result.remaining_budget_seconds;
    output.coupled_solve_cpu_seconds = result.coupled_solve_cpu_seconds;
    output.warm_start_cpu_seconds = result.warm_start_cpu_seconds;
    output.solve_cpu_seconds = result.solve_cpu_seconds;
    ++solve_count_;
    solve_time_total_seconds_ += result.solve_time_seconds;
    solve_time_max_seconds_ = std::max(solve_time_max_seconds_, result.solve_time_seconds);
    if (!result.valid) {
      accel_observer_.setCommand(runtime.command);
      output.control_input = runtime.command;
      output.recovery_command_active = true;
      output.valid = false;
      recordCallbackTiming(output, callback_start, callback_period_seconds, mutex_wait_seconds);
      output_publisher_->publish(output);
      publishSetpoint(output.header, output.sequence, runtime.command, state_->yaw, 0.0, true);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "MPC failed: %s coupled_status=%u iter=%d primal=%.3g dual=%.3g solve=%.3f ms; recovery a=[%.2f %.2f %.2f]",
          failureReasonString(result.failure_reason),
          static_cast<unsigned>(result.coupled.status),
          result.coupled.iterations, result.coupled.primal_residual,
          result.coupled.dual_residual, result.solve_time_seconds * 1000.0,
          runtime.command[0], runtime.command[1], runtime.command[2]);
      return;
    }

    output.predicted_states.resize(mpc_controller::translational::kHorizonLength * 9);
    for (std::size_t step = 0;
         step < mpc_controller::translational::kHorizonLength; ++step) {
      for (std::size_t axis = 0; axis < 3; ++axis) {
        for (std::size_t state = 0; state < 3; ++state) {
          output.predicted_states[step * 9 + axis * 3 + state] = result.coupled.prediction[step](3 * state + axis);
        }
      }
    }
    output.recovery_command_active = false;
    output.valid = true;

    output.control_input = runtime.command;
    accel_observer_.setCommand(runtime.command);
    recordCallbackTiming(output, callback_start, callback_period_seconds, mutex_wait_seconds);
    output_publisher_->publish(output);
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "MPC update seq=%lu measured_x0=[p %.3f %.3f %.3f v %.3f %.3f %.3f "
        "a_hat %.3f %.3f %.3f a_raw %.3f %.3f %.3f] "
        "u=[%.3f %.3f %.3f] ref_age=%.1f ms state_age=%.1f ms "
        "solve=%.3f ms mean=%.3f ms max=%.3f ms coupled_valid=%s "
        "tilt_max=%.3f thrust_max=%.3f",
        static_cast<unsigned long>(output.sequence), measured.position[0],
        measured.position[1], measured.position[2], measured.velocity[0],
        measured.velocity[1], measured.velocity[2], measured.acceleration[0],
        measured.acceleration[1], measured.acceleration[2], raw_acceleration[0],
        raw_acceleration[1], raw_acceleration[2], runtime.command[0],
        runtime.command[1], runtime.command[2], reference_age * 1.0e3,
        state_age * 1.0e3, output.solve_time_seconds * 1000.0,
        (solve_time_total_seconds_ / static_cast<double>(solve_count_)) *  1000.0,
        solve_time_max_seconds_ * 1000.0,
        result.coupled.valid ? "true" : "false",
        result.coupled.max_predicted_tilt_rad,
        result.coupled.max_predicted_collective_specific_force_m_s2);
    publishSetpoint(output.header, output.sequence, runtime.command, command_reference.yaw, command_reference.yaw_rate, false);
  }

  void recordCallbackTiming(
      Output &output, std::chrono::steady_clock::time_point callback_start,
      double callback_period_seconds, double mutex_wait_seconds) const noexcept {
    output.callback_period_seconds = callback_period_seconds;
    output.callback_period_jitter_seconds = callback_period_seconds > 0.0 ? callback_period_seconds - 1.0 / update_rate_hz_ : 0.0;
    output.callback_execution_seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - callback_start).count();
    output.mutex_wait_seconds = mutex_wait_seconds;
  }

  void publishSetpoint(const std_msgs::msg::Header &header, uint64_t sequence,
                       const std::array<double, 3> &acceleration, double yaw,
                       double yaw_rate, bool recovery) {
    // Convert desired ENU acceleration and yaw into desired SO(3) attitude
    // plus mass-normalized collective force for the 250 Hz adapter.
    if (!mapping_config_valid_ || !state_)
      return;
    mpc_controller::force_attitude::Input mapping_input;
    mapping_input.desired_acceleration_m_s2 = Eigen::Vector3d(acceleration[0], acceleration[1], acceleration[2]);
    mapping_input.desired_yaw_rad = std::isfinite(yaw) ? yaw : state_->yaw;
    mapping_input.valid = strict_validation_ ? state_->attitude_valid : true;
    const auto mapping = mpc_controller::force_attitude::compute(mapping_config_, mapping_input);
    if (!mapping.valid) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,"Force/attitude output rejected: %s",
                           mpc_controller::force_attitude::failureReasonName(mapping.failure_reason));
      return;
    }

    Setpoint setpoint;
    setpoint.header = header;
    setpoint.sequence = sequence;
    for (int i = 0; i < 3; ++i) {
      setpoint.desired_acceleration_m_s2[i] = mapping.desired_acceleration_m_s2[i];
      setpoint.desired_specific_force_world_m_s2[i] = mapping.desired_specific_force_world_m_s2[i];
    }
    setpoint.desired_attitude_wxyz = {
        mapping.desired_body_to_world.w(), mapping.desired_body_to_world.x(),
        mapping.desired_body_to_world.y(), mapping.desired_body_to_world.z()};
    // A malformed reference never reaches PX4. Recovery deliberately has a
    // zero yaw-rate feed-forward, so it holds the measured/current heading.
    if (!std::isfinite(yaw_rate)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,"Force/attitude output rejected: non-finite yaw rate");
      return;
    }
    setpoint.desired_yaw_rate_rad_s = yaw_rate;
    setpoint.desired_collective_specific_force_m_s2 =mapping.desired_collective_specific_force_m_s2;
    setpoint.tilt_angle_rad = mapping.tilt_angle_rad;
    setpoint.recovery_active = recovery;
    setpoint_publisher_->publish(setpoint);
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Force/attitude setpoint seq=%lu a_des=[%.3f %.3f %.3f] "
        "specific_force=[%.3f %.3f %.3f] "
        "collective=%.3f m/s^2 tilt=%.3f rad mode=%s",
        static_cast<unsigned long>(setpoint.sequence),
        setpoint.desired_acceleration_m_s2[0],
        setpoint.desired_acceleration_m_s2[1],
        setpoint.desired_acceleration_m_s2[2],
        setpoint.desired_specific_force_world_m_s2[0],
        setpoint.desired_specific_force_world_m_s2[1],
        setpoint.desired_specific_force_world_m_s2[2],
        setpoint.desired_collective_specific_force_m_s2,
        setpoint.tilt_angle_rad, recovery ? "recovery" : "tracking");
  }

  static const char *failureReasonString(FailureReason reason) noexcept {
    switch (reason) {
    case FailureReason::invalid_configuration:
      return "invalid configuration";
    case FailureReason::invalid_measured_state:
      return "invalid measured state";
    case FailureReason::invalid_reference:
      return "invalid reference";
    case FailureReason::solver_not_converged:
      return "solver not converged";
    case FailureReason::non_finite_solver_output:
      return "non-finite solver output";
    case FailureReason::none:
      return "none";
    }
    return "unknown";
  }

  std::mutex mutex_;
  bool config_valid_ = true;
  bool mapping_config_valid_ = true;
  bool stale_input_active_ = false;
  bool strict_validation_ = true;
  double update_rate_hz_ = 100.0;
  double reference_timeout_seconds_ = 1.5;
  double state_timeout_seconds_ = 0.25;
  double recovery_velocity_gain_ = 1.0;
  double recovery_position_gain_z_ = 0.5;
  double recovery_max_acceleration_xy_ = 2.5;
  double recovery_max_acceleration_z_ = 1.5;
  std::string output_frame_id_ = "map";
  mpc_controller::translational::Config config_{};
  AccelGate accel_gate_{};
  AccelObserver accel_observer_{};
  mpc_controller::force_attitude::Parameters mapping_config_{};
  std::optional<mpc_controller::application::ControlRuntime> controller_;
  std::optional<ValidatedReference> reference_;
  std::optional<State> state_;
  rclcpp::Time reference_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time state_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time reference_received_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Time state_received_at_{0, 0, RCL_ROS_TIME};
  uint64_t reference_trajectory_id_ = 0;
  uint64_t sequence_ = 0;
  uint64_t solve_count_ = 0;
  double solve_time_total_seconds_ = 0.0;
  double solve_time_max_seconds_ = 0.0;
  std::chrono::steady_clock::time_point last_callback_start_{};
  bool callback_timing_initialized_ = false;
  rclcpp::Subscription<Reference>::SharedPtr reference_subscription_;
  rclcpp::Subscription<State>::SharedPtr state_subscription_;
  rclcpp::Publisher<Output>::SharedPtr output_publisher_;
  rclcpp::Publisher<Setpoint>::SharedPtr setpoint_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MpcControllerNode>());
  rclcpp::shutdown();
  return 0;
}
