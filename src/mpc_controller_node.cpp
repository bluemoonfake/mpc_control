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

}  // namespace

class MpcControllerNode final : public rclcpp::Node
{
public:
  MpcControllerNode()
  : Node("mpc_controller_node")
  {
    // Parameters are split into solver/model limits, acceleration validation,
    // recovery braking, and force-to-attitude mapping.
    declare_parameter("update_rate_hz", update_rate_hz_);
    declare_parameter("reference_timeout_seconds", reference_timeout_seconds_);
    declare_parameter("state_timeout_seconds", state_timeout_seconds_);
    declare_parameter("strict_validation", strict_validation_);
    declare_parameter("output_frame_id", output_frame_id_);
    declare_parameter("dt_first", config_.dt_first);
    declare_parameter("dt_later", config_.dt_later);
    declare_parameter("solver_deadline_seconds", config_.solver_deadline_seconds);
    declare_parameter("max_iterations", config_.max_iterations);
    declare_parameter("admm_rho", config_.admm_rho);
    declare_parameter("solver_absolute_tolerance", config_.solver_absolute_tolerance);
    declare_parameter("solver_relative_tolerance", config_.solver_relative_tolerance);
    declare_parameter("max_speed_xy", config_.max_speed_xy);
    declare_parameter("max_acceleration_xy", config_.max_acceleration_xy);
    declare_parameter("max_control_xy", config_.max_control_xy);
    declare_parameter("max_control_rate_xy", config_.max_control_rate_xy);
    declare_parameter("max_speed_z", config_.max_speed_z);
    declare_parameter("max_acceleration_z", config_.max_acceleration_z);
    declare_parameter("max_measured_acceleration_z_m_s2", accel_gate_.max_z);
    declare_parameter("max_control_z", config_.max_control_z);
    declare_parameter("max_control_rate_z", config_.max_control_rate_z);
    declare_parameter("recovery_velocity_gain", recovery_velocity_gain_);
    declare_parameter("recovery_position_gain_z", recovery_position_gain_z_);
    declare_parameter("recovery_max_acceleration_xy", recovery_max_acceleration_xy_);
    declare_parameter("recovery_max_acceleration_z", recovery_max_acceleration_z_);
    declare_parameter("max_tilt", mapping_config_.max_tilt_rad);

    get_parameter("update_rate_hz", update_rate_hz_);
    get_parameter("reference_timeout_seconds", reference_timeout_seconds_);
    get_parameter("state_timeout_seconds", state_timeout_seconds_);
    get_parameter("strict_validation", strict_validation_);
    get_parameter("output_frame_id", output_frame_id_);
    get_parameter("dt_first", config_.dt_first);
    get_parameter("dt_later", config_.dt_later);
    get_parameter("solver_deadline_seconds", config_.solver_deadline_seconds);
    get_parameter("max_iterations", config_.max_iterations);
    get_parameter("admm_rho", config_.admm_rho);
    get_parameter("solver_absolute_tolerance", config_.solver_absolute_tolerance);
    get_parameter("solver_relative_tolerance", config_.solver_relative_tolerance);
    get_parameter("max_speed_xy", config_.max_speed_xy);
    get_parameter("max_acceleration_xy", config_.max_acceleration_xy);
    get_parameter("max_control_xy", config_.max_control_xy);
    get_parameter("max_control_rate_xy", config_.max_control_rate_xy);
    get_parameter("max_speed_z", config_.max_speed_z);
    get_parameter("max_acceleration_z", config_.max_acceleration_z);
    get_parameter("max_measured_acceleration_z_m_s2", accel_gate_.max_z);
    get_parameter("max_control_z", config_.max_control_z);
    get_parameter("max_control_rate_z", config_.max_control_rate_z);
    get_parameter("recovery_velocity_gain", recovery_velocity_gain_);
    get_parameter("recovery_position_gain_z", recovery_position_gain_z_);
    get_parameter("recovery_max_acceleration_xy", recovery_max_acceleration_xy_);
    get_parameter("recovery_max_acceleration_z", recovery_max_acceleration_z_);
    get_parameter("max_tilt", mapping_config_.max_tilt_rad);

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
    config_valid_ = vector_parameters_valid
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
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / update_rate_hz_)),
      std::bind(&MpcControllerNode::update, this));
  }

private:
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
    // One callback performs admission, horizon sampling, three scalar QPs,
    // bounded recovery on failure, and the force/attitude conversion.
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
    measured.acceleration = state_->acceleration;
    if (!mpc_controller::translational::finite(measured.position)
      || !mpc_controller::translational::finite(measured.velocity)
      || !mpc_controller::translational::finite(measured.acceleration)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "MPC update rejected: measured state contains NaN or Inf");
      return;
    }
    if (strict_validation_) {
      if (!accel_gate_.accept(measured.acceleration)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "MPC update rejected: acceleration is non-finite or |az|=%.3f exceeds %.3f m/s^2",
          std::abs(measured.acceleration[2]), accel_gate_.max_z);
        return;
      }
    }

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
    for (std::size_t axis = 0; axis < 3; ++axis) {
      const auto &axis_result = result.axes[axis];
      output.solver_iterations[axis] = axis_result.iterations;
      output.solver_status[axis] = static_cast<uint8_t>(axis_result.status);
      output.solver_primal_residual[axis] = axis_result.primal_residual;
      output.solver_dual_residual[axis] = axis_result.dual_residual;
      output.solver_primal_tolerance[axis] = axis_result.primal_tolerance;
      output.solver_dual_tolerance[axis] = axis_result.dual_tolerance;
      output.recovery_constraint_active[axis] = axis_result.recovery_constraint_active;
    }
    ++solve_count_;
    solve_time_total_seconds_ += result.solve_time_seconds;
    solve_time_max_seconds_ = std::max(solve_time_max_seconds_, result.solve_time_seconds);
    if (!result.valid) {
      const auto recovery = recoveryAcceleration(measured, horizon.points.front());
      output.control_input = recovery;
      output.recovery_command_active = true;
      output.valid = false;
      output_publisher_->publish(output);
      publishSetpoint(output.header, output.sequence, recovery, state_->yaw, true);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "MPC failed: %s status=[%u %u %u] iter=[%d %d %d] "
        "primal=[%.3g %.3g %.3g] dual=[%.3g %.3g %.3g] solve=%.3f ms; "
        "recovery a=[%.2f %.2f %.2f]",
        failureReasonString(result.failure_reason),
        static_cast<unsigned>(result.axes[0].status),
        static_cast<unsigned>(result.axes[1].status),
        static_cast<unsigned>(result.axes[2].status),
        result.axes[0].iterations, result.axes[1].iterations, result.axes[2].iterations,
        result.axes[0].primal_residual, result.axes[1].primal_residual,
        result.axes[2].primal_residual,
        result.axes[0].dual_residual, result.axes[1].dual_residual,
        result.axes[2].dual_residual,
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
            result.axes[axis].prediction[step](state);
        }
      }
    }
    output.recovery_command_active = false;
    output.valid = true;

    const auto &command_reference = horizon.points.front();
    output.control_input = result.control;
    output_publisher_->publish(output);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "MPC update seq=%lu measured_x0=[p %.3f %.3f %.3f v %.3f %.3f %.3f "
      "a %.3f %.3f %.3f] u=[%.3f %.3f %.3f] ref_age=%.1f ms state_age=%.1f ms "
      "solve=%.3f ms mean=%.3f ms max=%.3f ms",
      static_cast<unsigned long>(output.sequence),
      measured.position[0], measured.position[1], measured.position[2],
      measured.velocity[0], measured.velocity[1], measured.velocity[2],
      measured.acceleration[0], measured.acceleration[1], measured.acceleration[2],
      result.control[0], result.control[1], result.control[2],
      reference_age * 1.0e3, state_age * 1.0e3,
      output.solve_time_seconds * 1000.0,
      (solve_time_total_seconds_ / static_cast<double>(solve_count_)) * 1000.0,
      solve_time_max_seconds_ * 1000.0);
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
  std::string output_frame_id_ = "map";
  mpc_controller::translational::Config config_{};
  AccelGate accel_gate_{};
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
