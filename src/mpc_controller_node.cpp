#include "mpc_controller/msg/mpc_translational_output.hpp"
#include "mpc_controller/msg/m3_control_output.hpp"
#include "mpc_controller/msg/reference_trajectory.hpp"
#include "mpc_controller/msg/vehicle_state.hpp"
#include "mpc_controller/mrs_control_math.hpp"
#include "mpc_controller/translational_mpc.hpp"

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

class MpcControllerNode final : public rclcpp::Node
{
public:
  MpcControllerNode()
  : Node("mpc_controller_node")
  {
    declare_parameter("update_rate_hz", update_rate_hz_);
    declare_parameter("reference_timeout_seconds", reference_timeout_seconds_);
    declare_parameter("state_timeout_seconds", state_timeout_seconds_);
    declare_parameter("output_frame_id", output_frame_id_);
    declare_parameter("dt_first", config_.dt_first);
    declare_parameter("dt_later", config_.dt_later);
    declare_parameter("max_iterations", config_.max_iterations);
    declare_parameter("solver_verbose", config_.solver_verbose);
    declare_parameter("q_xy", std::vector<double>{500.0, 100.0, 100.0});
    declare_parameter("s_xy", std::vector<double>{1000.0, 300.0, 300.0});
    declare_parameter("q_z", std::vector<double>{100.0, 10.0, 10.0});
    declare_parameter("s_z", std::vector<double>{100.0, 10.0, 10.0});
    declare_parameter("max_speed_xy", config_.max_speed_xy);
    declare_parameter("max_control_xy", config_.max_control_xy);
    declare_parameter("max_control_rate_xy", config_.max_control_rate_xy);
    declare_parameter("max_speed_z", config_.max_speed_z);
    declare_parameter("max_acceleration_z", config_.max_acceleration_z);
    declare_parameter("max_control_z", config_.max_control_z);
    declare_parameter("max_control_rate_z", config_.max_control_rate_z);

    declare_parameter("vehicle_mass", m3_config_.mass_kg);
    declare_parameter("inertia_xx", m3_config_.inertia_kg_m2.x());
    declare_parameter("inertia_yy", m3_config_.inertia_kg_m2.y());
    declare_parameter("inertia_zz", m3_config_.inertia_kg_m2.z());
    declare_parameter("gravity", m3_config_.gravity_m_s2);
    declare_parameter("attitude_gain_roll_pitch", 5.0);
    declare_parameter("attitude_gain_yaw", 1.0);
    declare_parameter("rate_gain_roll_pitch", 4.0);
    declare_parameter("rate_gain_yaw", 4.0);
    declare_parameter("normalized_rate_gain_roll_pitch", 0.15);
    declare_parameter("normalized_rate_gain_yaw", 0.20);
    declare_parameter("max_normalized_torque_roll", 1.0);
    declare_parameter("max_normalized_torque_pitch", 1.0);
    declare_parameter("max_normalized_torque_yaw", 1.0);
    declare_parameter("max_roll_rate", 4.0);
    declare_parameter("max_pitch_rate", 4.0);
    declare_parameter("max_yaw_rate", 4.0);
    declare_parameter("max_tilt", m3_config_.max_tilt_rad);

    get_parameter("update_rate_hz", update_rate_hz_);
    get_parameter("reference_timeout_seconds", reference_timeout_seconds_);
    get_parameter("state_timeout_seconds", state_timeout_seconds_);
    get_parameter("output_frame_id", output_frame_id_);
    get_parameter("dt_first", config_.dt_first);
    get_parameter("dt_later", config_.dt_later);
    get_parameter("max_iterations", config_.max_iterations);
    get_parameter("solver_verbose", config_.solver_verbose);
    const bool vector_parameters_valid = getArrayParameter("q_xy", config_.q_xy)
      && getArrayParameter("s_xy", config_.s_xy)
      && getArrayParameter("q_z", config_.q_z)
      && getArrayParameter("s_z", config_.s_z);
    get_parameter("max_speed_xy", config_.max_speed_xy);
    get_parameter("max_control_xy", config_.max_control_xy);
    get_parameter("max_control_rate_xy", config_.max_control_rate_xy);
    get_parameter("max_speed_z", config_.max_speed_z);
    get_parameter("max_acceleration_z", config_.max_acceleration_z);
    get_parameter("max_control_z", config_.max_control_z);
    get_parameter("max_control_rate_z", config_.max_control_rate_z);

    double attitude_gain_roll_pitch = 0.0;
    double attitude_gain_yaw = 0.0;
    double rate_gain_roll_pitch = 0.0;
    double rate_gain_yaw = 0.0;
    double normalized_rate_gain_roll_pitch = 0.0;
    double normalized_rate_gain_yaw = 0.0;
    double max_normalized_torque_roll = 0.0;
    double max_normalized_torque_pitch = 0.0;
    double max_normalized_torque_yaw = 0.0;
    double max_roll_rate = 0.0;
    double max_pitch_rate = 0.0;
    double max_yaw_rate = 0.0;
    get_parameter("vehicle_mass", m3_config_.mass_kg);
    get_parameter("inertia_xx", m3_config_.inertia_kg_m2.x());
    get_parameter("inertia_yy", m3_config_.inertia_kg_m2.y());
    get_parameter("inertia_zz", m3_config_.inertia_kg_m2.z());
    get_parameter("gravity", m3_config_.gravity_m_s2);
    get_parameter("attitude_gain_roll_pitch", attitude_gain_roll_pitch);
    get_parameter("attitude_gain_yaw", attitude_gain_yaw);
    get_parameter("rate_gain_roll_pitch", rate_gain_roll_pitch);
    get_parameter("rate_gain_yaw", rate_gain_yaw);
    get_parameter("normalized_rate_gain_roll_pitch", normalized_rate_gain_roll_pitch);
    get_parameter("normalized_rate_gain_yaw", normalized_rate_gain_yaw);
    get_parameter("max_normalized_torque_roll", max_normalized_torque_roll);
    get_parameter("max_normalized_torque_pitch", max_normalized_torque_pitch);
    get_parameter("max_normalized_torque_yaw", max_normalized_torque_yaw);
    get_parameter("max_roll_rate", max_roll_rate);
    get_parameter("max_pitch_rate", max_pitch_rate);
    get_parameter("max_yaw_rate", max_yaw_rate);
    get_parameter("max_tilt", m3_config_.max_tilt_rad);
    m3_config_.attitude_gain_rad_s =
      Eigen::Vector3d(attitude_gain_roll_pitch, attitude_gain_roll_pitch, attitude_gain_yaw);
    m3_config_.rate_gain_s_inv =
      Eigen::Vector3d(rate_gain_roll_pitch, rate_gain_roll_pitch, rate_gain_yaw);
    m3_config_.normalized_rate_gain_s_inv = Eigen::Vector3d(
      normalized_rate_gain_roll_pitch, normalized_rate_gain_roll_pitch,
      normalized_rate_gain_yaw);
    m3_config_.normalized_torque_limit = Eigen::Vector3d(
      max_normalized_torque_roll, max_normalized_torque_pitch, max_normalized_torque_yaw);
    m3_config_.max_body_rate_rad_s = Eigen::Vector3d(max_roll_rate, max_pitch_rate, max_yaw_rate);

    config_valid_ = vector_parameters_valid
      && std::isfinite(update_rate_hz_) && update_rate_hz_ > 0.0
      && std::isfinite(reference_timeout_seconds_) && reference_timeout_seconds_ > 0.0
      && std::isfinite(state_timeout_seconds_) && state_timeout_seconds_ > 0.0
      && !output_frame_id_.empty() && mpc_controller::translational::validConfig(config_);
    m3_config_valid_ = mpc_controller::mrs_control::validParameters(m3_config_);

    reference_subscription_ = create_subscription<Reference>(
      "reference_trajectory", 10,
      [this](Reference::SharedPtr message) {referenceCallback(std::move(message));});
    state_subscription_ = create_subscription<State>(
      "vehicle_state", 10,
      [this](State::SharedPtr message) {stateCallback(std::move(message));});
    output_publisher_ = create_publisher<Output>("mpc_translational_output", 10);
    m3_output_publisher_ = create_publisher<M3Output>("m3_control_output", 10);

    if (!config_valid_) {
      RCLCPP_ERROR(get_logger(), "Invalid M2 MPC configuration; controller updates disabled");
      return;
    }
    if (!m3_config_valid_) {
      RCLCPP_ERROR(get_logger(), "Invalid M3 control configuration; M3 shadow output disabled");
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
  using M3Output = mpc_controller::msg::M3ControlOutput;
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
        "M2 reference rejected: frame_id is empty");
      return;
    }
    ReferenceData converted;
    if ((message->header.stamp.sec == 0 && message->header.stamp.nanosec == 0)
      || !convertReference(*message, converted)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "M2 reference rejected: malformed, empty, zero-timestamp or non-finite trajectory");
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    const rclcpp::Time stamp(message->header.stamp);
    if (reference_stamp_.nanoseconds() != 0 && stamp < reference_stamp_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "M2 reference rejected: timestamp moved backwards");
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
        "M2 measured state rejected: timestamp moved backwards");
      return;
    }
    state_ = std::move(*message);
    state_stamp_ = stamp;
    state_received_at_ = get_clock()->now();
  }

  void update()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!config_valid_ || !controller_ || !reference_ || !state_) {
      return;
    }
    const auto now = get_clock()->now();
    const double reference_age = (now - reference_received_at_).seconds();
    const double state_age = (now - state_received_at_).seconds();
    if (!std::isfinite(reference_age) || reference_age < 0.0
      || reference_age > reference_timeout_seconds_
      || !std::isfinite(state_age) || state_age < 0.0 || state_age > state_timeout_seconds_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "M2 update rejected: reference or measured state is stale");
      return;
    }
    if (!state_->valid || !state_->position_valid || !state_->velocity_valid
      || !state_->acceleration_valid || state_->header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "M2 update rejected: measured translational state is invalid");
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
        "M2 update rejected: measured state contains NaN or Inf");
      return;
    }

    const double elapsed = now.seconds() - reference_->header_time_seconds;
    Horizon horizon;
    if (!Sampler::buildHorizon(
        *reference_, elapsed, config_.dt_first, config_.dt_later, horizon)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "M2 update rejected: reference cannot be sampled on the solver grid");
      return;
    }

    const auto result = controller_->update(measured, horizon);
    if (!result.valid) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "M2 solver update failed: %s", failureReasonString(result.failure_reason));
      return;
    }

    Output output;
    output.header.stamp = now;
    output.header.frame_id = output_frame_id_;
    output.trajectory_id = reference_trajectory_id_;
    output.sequence = ++sequence_;
    output.control_input = result.control;
    output.first_predicted_acceleration = result.first_predicted_acceleration;
    output.predicted_states.assign(result.prediction.begin(), result.prediction.end());
    output.solver_iterations = result.iterations;
    output.solve_time_seconds = result.solve_time_seconds;
    output.solver_success = true;
    output.valid = true;
    output_publisher_->publish(output);
    ++solve_count_;
    solve_time_total_seconds_ += result.solve_time_seconds;
    solve_time_max_seconds_ = std::max(solve_time_max_seconds_, result.solve_time_seconds);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "M2 shadow update seq=%lu measured_x0=[p %.3f %.3f %.3f v %.3f %.3f %.3f a %.3f %.3f %.3f] "
      "u=[%.3f %.3f %.3f] ref_age=%.1f ms state_age=%.1f ms "
      "solve=%.3f ms mean=%.3f ms max=%.3f ms",
      static_cast<unsigned long>(output.sequence),
      measured.position[0], measured.position[1], measured.position[2],
      measured.velocity[0], measured.velocity[1], measured.velocity[2],
      measured.acceleration[0], measured.acceleration[1], measured.acceleration[2],
      output.control_input[0], output.control_input[1], output.control_input[2],
      reference_age * 1.0e3, state_age * 1.0e3,
      output.solve_time_seconds * 1000.0,
      (solve_time_total_seconds_ / static_cast<double>(solve_count_)) * 1000.0,
      solve_time_max_seconds_ * 1000.0);

    if (!m3_config_valid_) {
      return;
    }
    ReferencePoint yaw_reference;
    if (!Sampler::sampleAt(*reference_, elapsed, yaw_reference)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "M3 output rejected: reference yaw cannot be sampled");
      return;
    }

    mpc_controller::mrs_control::Input m3_input;
    m3_input.desired_acceleration_m_s2 = Eigen::Vector3d(
      result.control[0], result.control[1], result.control[2]);
    m3_input.measured_body_rate_rad_s = Eigen::Vector3d(
      state_->body_rate[0], state_->body_rate[1], state_->body_rate[2]);
    m3_input.measured_body_to_world = Eigen::Quaterniond(
      state_->attitude[0], state_->attitude[1], state_->attitude[2], state_->attitude[3]);
    m3_input.desired_yaw_rad = yaw_reference.yaw;
    m3_input.desired_yaw_rate_rad_s = yaw_reference.yaw_rate;
    m3_input.desired_yaw_rate_valid = std::isfinite(yaw_reference.yaw_rate);
    m3_input.attitude_valid = state_->attitude_valid;
    m3_input.body_rate_valid = state_->body_rate_valid;
    m3_input.heading_valid = state_->heading_valid;
    m3_input.control_ready = state_->control_ready;

    const auto m3_result = mpc_controller::mrs_control::compute(m3_config_, m3_input);
    if (!m3_result.valid) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "M3 shadow update rejected: invalid control math");
      return;
    }

    M3Output m3_output;
    m3_output.header = output.header;
    m3_output.trajectory_id = output.trajectory_id;
    m3_output.sequence = output.sequence;
    for (int i = 0; i < 3; ++i) {
      m3_output.desired_acceleration_m_s2[i] = m3_result.desired_acceleration_m_s2[i];
      m3_output.desired_force_world_n[i] = m3_result.desired_force_world_n[i];
      m3_output.orientation_error[i] = m3_result.orientation_error[i];
      m3_output.desired_body_rate_rad_s[i] = m3_result.desired_body_rate_rad_s[i];
      m3_output.body_rate_error_rad_s[i] = m3_result.body_rate_error_rad_s[i];
      m3_output.control_group_action[i] = m3_result.control_group_action[i];
      m3_output.normalized_torque_command_flu[i] =
        m3_result.normalized_torque_command_flu[i];
    }
    m3_output.normalized_torque_saturated = m3_result.normalized_torque_saturated;
    m3_output.desired_attitude_wxyz = {
      m3_result.desired_body_to_world.w(), m3_result.desired_body_to_world.x(),
      m3_result.desired_body_to_world.y(), m3_result.desired_body_to_world.z()};
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) {
        m3_output.desired_rotation_world_from_body[row * 3 + column] =
          m3_result.desired_rotation_world_from_body(row, column);
      }
    }
    m3_output.desired_thrust_force_n = m3_result.desired_thrust_force_n;
    m3_output.tilt_angle_rad = m3_result.tilt_angle_rad;
    m3_output.math_valid = m3_result.valid;
    m3_output.active_control_ready = m3_result.active_control_ready;
    m3_output.valid = m3_result.valid;
    m3_output_publisher_->publish(m3_output);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "M3 shadow output seq=%lu a_des=[%.3f %.3f %.3f] force=[%.3f %.3f %.3f] "
      "thrust=%.3f N tilt=%.3f rad omega=[%.3f %.3f %.3f] "
      "norm_torque_flu=[%.4f %.4f %.4f] saturated=%s active=%s",
      static_cast<unsigned long>(m3_output.sequence),
      m3_output.desired_acceleration_m_s2[0], m3_output.desired_acceleration_m_s2[1],
      m3_output.desired_acceleration_m_s2[2],
      m3_output.desired_force_world_n[0], m3_output.desired_force_world_n[1],
      m3_output.desired_force_world_n[2], m3_output.desired_thrust_force_n,
      m3_output.tilt_angle_rad, m3_output.desired_body_rate_rad_s[0],
      m3_output.desired_body_rate_rad_s[1], m3_output.desired_body_rate_rad_s[2],
      m3_output.normalized_torque_command_flu[0],
      m3_output.normalized_torque_command_flu[1],
      m3_output.normalized_torque_command_flu[2],
      m3_output.normalized_torque_saturated ? "true" : "false",
      m3_output.active_control_ready ? "true" : "false");
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
  bool m3_config_valid_ = true;
  double update_rate_hz_ = 50.0;
  double reference_timeout_seconds_ = 1.5;
  double state_timeout_seconds_ = 0.25;
  std::string output_frame_id_ = "map";
  mpc_controller::translational::Config config_{};
  mpc_controller::mrs_control::Parameters m3_config_{};
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
  rclcpp::Publisher<M3Output>::SharedPtr m3_output_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MpcControllerNode>());
  rclcpp::shutdown();
  return 0;
}
