#include "mpc_control_ros/mpc_trajectory_node.hpp"

#include <builtin_interfaces/msg/duration.hpp>
#include <lifecycle_msgs/msg/state.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <sstream>
#include <utility>
#include <vector>

namespace mpc_control_ros
{

namespace
{

using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

constexpr double kTimeEpsilon = 1.0e-9;

bool finiteArray(const std::array<double, 3>& values) noexcept
{
  return std::all_of(
      values.begin(), values.end(),
      [](const double value) { return std::isfinite(value); });
}

double finiteOrZero(const double value) noexcept
{
  return std::isfinite(value) ? value : 0.0;
}

}  // namespace

MpcTrajectoryNode::MpcTrajectoryNode(const rclcpp::NodeOptions& options)
: rclcpp_lifecycle::LifecycleNode("mpc_trajectory_node", options)
{
  declare_parameter("frame_id", frame_id_);
  declare_parameter("reference_topic", reference_topic_);
  declare_parameter("state_topic", state_topic_);
  declare_parameter("command_topic", command_topic_);
  declare_parameter("prediction_topic", prediction_topic_);
  declare_parameter("diagnostics_topic", diagnostics_topic_);
  declare_parameter("update_rate_hz", update_rate_hz_);
  declare_parameter("reference_timeout_seconds", reference_timeout_seconds_);
  declare_parameter("state_timeout_seconds", state_timeout_seconds_);
  declare_parameter("clock_stall_timeout_seconds", clock_stall_timeout_seconds_);
  declare_parameter("clock_jump_threshold_seconds", clock_jump_threshold_seconds_);
  declare_parameter(
      "check_derivative_consistency", check_derivative_consistency_);
  declare_parameter(
      "derivative_position_tolerance", derivative_position_tolerance_);
  declare_parameter(
      "derivative_velocity_tolerance", derivative_velocity_tolerance_);
  declare_parameter(
      "use_measured_acceleration_on_activate",
      use_measured_acceleration_on_activate_);
}

CallbackReturn MpcTrajectoryNode::on_configure(
    const rclcpp_lifecycle::State& /*state*/)
{
  try {
    get_parameter("frame_id", frame_id_);
    get_parameter("reference_topic", reference_topic_);
    get_parameter("state_topic", state_topic_);
    get_parameter("command_topic", command_topic_);
    get_parameter("prediction_topic", prediction_topic_);
    get_parameter("diagnostics_topic", diagnostics_topic_);
    get_parameter("update_rate_hz", update_rate_hz_);
    get_parameter("reference_timeout_seconds", reference_timeout_seconds_);
    get_parameter("state_timeout_seconds", state_timeout_seconds_);
    get_parameter("clock_stall_timeout_seconds", clock_stall_timeout_seconds_);
    get_parameter("clock_jump_threshold_seconds", clock_jump_threshold_seconds_);
    get_parameter(
        "check_derivative_consistency", check_derivative_consistency_);
    get_parameter(
        "derivative_position_tolerance", derivative_position_tolerance_);
    get_parameter(
        "derivative_velocity_tolerance", derivative_velocity_tolerance_);
    get_parameter(
        "use_measured_acceleration_on_activate",
        use_measured_acceleration_on_activate_);

    if (frame_id_.empty() || reference_topic_.empty() || state_topic_.empty()
        || command_topic_.empty() || prediction_topic_.empty()
        || diagnostics_topic_.empty()
        || !std::isfinite(update_rate_hz_) || update_rate_hz_ <= 0.0
        || !std::isfinite(reference_timeout_seconds_)
        || reference_timeout_seconds_ <= 0.0
        || !std::isfinite(state_timeout_seconds_)
        || state_timeout_seconds_ <= 0.0
        || !std::isfinite(clock_stall_timeout_seconds_)
        || clock_stall_timeout_seconds_ <= 0.0
        || !std::isfinite(clock_jump_threshold_seconds_)
        || clock_jump_threshold_seconds_ <= 0.0
        || !std::isfinite(derivative_position_tolerance_)
        || derivative_position_tolerance_ < 0.0
        || !std::isfinite(derivative_velocity_tolerance_)
        || derivative_velocity_tolerance_ < 0.0) {
      RCLCPP_ERROR(get_logger(), "Invalid mpc_control_ros parameter configuration");
      return CallbackReturn::FAILURE;
    }

    core_configuration_.use_measured_acceleration_on_activate =
        use_measured_acceleration_on_activate_;
    // The core advances its virtual state with the first prediction step on
    // every update.  Therefore that step must equal the wrapper's update
    // period; otherwise a 50 Hz wrapper with dt_first=0.01 would make the
    // virtual trajectory progress at half real time and create phase lag.
    const double update_period_seconds = 1.0 / update_rate_hz_;
    core_configuration_.horizontal_solver.dt_first = update_period_seconds;
    core_configuration_.vertical_solver.dt_first = update_period_seconds;
    sampler_configuration_.dt_first =
        core_configuration_.horizontal_solver.dt_first;
    sampler_configuration_.dt_later =
        core_configuration_.horizontal_solver.dt_later;
    sampler_configuration_.validation.check_derivative_consistency =
        check_derivative_consistency_;
    sampler_configuration_.validation.position_derivative_tolerance =
        derivative_position_tolerance_;
    sampler_configuration_.validation.velocity_derivative_tolerance =
        derivative_velocity_tolerance_;
    sampler_configuration_.validation.limits.has_max_speed = true;
    sampler_configuration_.validation.limits.max_speed = std::min(
        core_configuration_.horizontal_solver.max_speed,
        core_configuration_.vertical_solver.max_speed);
    sampler_configuration_.validation.limits.has_max_acceleration = true;
    sampler_configuration_.validation.limits.max_acceleration = std::min(
        core_configuration_.horizontal_solver.max_acceleration,
        core_configuration_.vertical_solver.max_acceleration);

    core_ = std::make_unique<mpc_control::MpcTrajectoryCore>(core_configuration_);
    if (!core_->configured()) {
      RCLCPP_ERROR(get_logger(), "MPC core configuration failed");
      core_.reset();
      return CallbackReturn::FAILURE;
    }

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    reference_subscription_ = create_subscription<ReferenceMessage>(
        reference_topic_, qos,
        std::bind(&MpcTrajectoryNode::referenceCallback, this,
                  std::placeholders::_1));
    state_subscription_ = create_subscription<StateMessage>(
        state_topic_, qos,
        std::bind(&MpcTrajectoryNode::stateCallback, this,
                  std::placeholders::_1));

    command_publisher_ = create_publisher<CommandMessage>(command_topic_, qos);
    prediction_publisher_ = create_publisher<PredictionMessage>(prediction_topic_, qos);
    diagnostics_publisher_ = create_publisher<DiagnosticsMessage>(diagnostics_topic_, qos);

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / update_rate_hz_));
    update_timer_ = create_wall_timer(
        period, std::bind(&MpcTrajectoryNode::updateCallback, this));
    update_timer_->cancel();

    resetRuntimeState();
    publishDiagnostics(
        get_clock()->now(), DiagnosticsMessage::FAILURE_NONE,
        "configured", false, false, 0.0, 0.0, 0.0);
    return CallbackReturn::SUCCESS;
  } catch (const std::exception& exception) {
    RCLCPP_ERROR(
        get_logger(), "Configure failed: %s", exception.what());
    core_.reset();
    return CallbackReturn::FAILURE;
  } catch (...) {
    RCLCPP_ERROR(get_logger(), "Configure failed with an unknown exception");
    core_.reset();
    return CallbackReturn::FAILURE;
  }
}

CallbackReturn MpcTrajectoryNode::on_activate(
    const rclcpp_lifecycle::State& /*state*/)
{
  if (!command_publisher_ || !prediction_publisher_ || !update_timer_ || !core_) {
    return CallbackReturn::FAILURE;
  }

  command_publisher_->on_activate();
  prediction_publisher_->on_activate();
  update_timer_->reset();
  publishDiagnostics(
      get_clock()->now(), DiagnosticsMessage::FAILURE_NONE,
      "active; waiting for valid reference and vehicle state", false, false,
      0.0, 0.0, 0.0);
  return CallbackReturn::SUCCESS;
}

CallbackReturn MpcTrajectoryNode::on_deactivate(
    const rclcpp_lifecycle::State& /*state*/)
{
  if (update_timer_) {
    update_timer_->cancel();
  }
  if (command_publisher_) {
    command_publisher_->on_deactivate();
  }
  if (prediction_publisher_) {
    prediction_publisher_->on_deactivate();
  }
  if (core_) {
    const auto configure_result = core_->configure(core_configuration_);
    if (!configure_result.valid) {
      RCLCPP_ERROR(get_logger(), "Failed to reset core during deactivation");
      return CallbackReturn::FAILURE;
    }
  }
  last_update_time_.reset();
  publishDiagnostics(
      get_clock()->now(), DiagnosticsMessage::FAILURE_NONE,
      "deactivated; command publishing stopped", false, false, 0.0, 0.0,
      0.0);
  return CallbackReturn::SUCCESS;
}

CallbackReturn MpcTrajectoryNode::on_cleanup(
    const rclcpp_lifecycle::State& /*state*/)
{
  if (update_timer_) {
    update_timer_->cancel();
  }
  update_timer_.reset();
  reference_subscription_.reset();
  state_subscription_.reset();
  command_publisher_.reset();
  prediction_publisher_.reset();
  diagnostics_publisher_.reset();
  core_.reset();
  reference_.reset();
  state_.reset();
  resetRuntimeState();
  return CallbackReturn::SUCCESS;
}

CallbackReturn MpcTrajectoryNode::on_shutdown(
    const rclcpp_lifecycle::State& /*state*/)
{
  if (update_timer_) {
    update_timer_->cancel();
  }
  if (command_publisher_) {
    command_publisher_->on_deactivate();
  }
  if (prediction_publisher_) {
    prediction_publisher_->on_deactivate();
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn MpcTrajectoryNode::on_error(
    const rclcpp_lifecycle::State& /*state*/)
{
  if (update_timer_) {
    update_timer_->cancel();
  }
  if (command_publisher_) {
    command_publisher_->on_deactivate();
  }
  if (prediction_publisher_) {
    prediction_publisher_->on_deactivate();
  }
  publishDiagnostics(
      get_clock()->now(), DiagnosticsMessage::FAILURE_INVALID_CONFIGURATION,
      "lifecycle error; command publishing stopped", false, false, 0.0, 0.0,
      0.0);
  return CallbackReturn::SUCCESS;
}

void MpcTrajectoryNode::referenceCallback(const ReferenceMessage::SharedPtr message)
{
  if (!message) {
    return;
  }

  const auto now = get_clock()->now();
  CachedReference converted;
  std::string error;
  const bool valid = convertReference(*message, converted, error);

  std::lock_guard<std::mutex> lock(mutex_);
  if (!valid) {
    reference_.reset();
    reference_valid_ = false;
    last_failure_reason_ = DiagnosticsMessage::FAILURE_INVALID_REFERENCE;
    last_failure_detail_ = error;
    return;
  }

  converted.received_at = now;
  reference_ = std::move(converted);
  reference_valid_ = true;
  last_failure_reason_ = DiagnosticsMessage::FAILURE_NONE;
  last_failure_detail_.clear();
}

void MpcTrajectoryNode::stateCallback(const StateMessage::SharedPtr message)
{
  if (!message) {
    return;
  }

  const auto now = get_clock()->now();
  CachedState converted;
  std::string error;
  const bool valid = convertState(*message, converted, error);
  converted.received_at = now;
  converted.measured_at = now;
  if (message->header.stamp.sec >= 0
      && !(message->header.stamp.sec == 0
           && message->header.stamp.nanosec == 0)) {
    converted.measured_at = rclcpp::Time(message->header.stamp, RCL_ROS_TIME);
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (last_state_timestamp_ && converted.measured_at < *last_state_timestamp_) {
    state_valid_ = false;
    last_failure_reason_ = DiagnosticsMessage::FAILURE_OUT_OF_ORDER;
    last_failure_detail_ = "vehicle state timestamp moved backwards";
    return;
  }
  if (!valid) {
    state_.reset();
    state_valid_ = false;
    last_failure_reason_ = DiagnosticsMessage::FAILURE_INVALID_STATE;
    last_failure_detail_ = error;
    return;
  }

  last_state_timestamp_ = converted.measured_at;
  state_ = std::move(converted);
  state_valid_ = true;
  last_failure_reason_ = DiagnosticsMessage::FAILURE_NONE;
  last_failure_detail_.clear();
}

void MpcTrajectoryNode::updateCallback()
{
  const auto now = get_clock()->now();
  const auto wall_now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(mutex_);
  reset_detected_ = false;

  double reference_age = 0.0;
  double state_age = 0.0;
  auto fail = [&](const std::uint8_t reason, const std::string& detail) {
      ++consecutive_failures_;
      last_failure_reason_ = reason;
      last_failure_detail_ = detail;
      publishDiagnostics(
          now, reason, detail, false, false, reference_age, state_age, 0.0,
          nullptr, reset_detected_, false);
    };

  if (!reference_ || !reference_valid_) {
    fail(DiagnosticsMessage::FAILURE_INVALID_REFERENCE, "no valid reference");
    return;
  }
  if (!state_ || !state_valid_) {
    const auto reason = last_failure_reason_ == DiagnosticsMessage::FAILURE_OUT_OF_ORDER
      ? DiagnosticsMessage::FAILURE_OUT_OF_ORDER
      : DiagnosticsMessage::FAILURE_INVALID_STATE;
    const auto detail = last_failure_detail_.empty()
      ? "no valid vehicle state"
      : last_failure_detail_;
    fail(reason, detail);
    return;
  }

  if (last_update_time_ && now < *last_update_time_) {
    fail(DiagnosticsMessage::FAILURE_TIME_JUMP, "ROS clock moved backwards");
    return;
  }
  if (last_update_time_ && last_clock_progress_wall_time_) {
    const double clock_delta_seconds = (now - *last_update_time_).seconds();
    const double wall_delta_seconds = std::chrono::duration<double>(
        wall_now - *last_clock_progress_wall_time_).count();
    if (clock_delta_seconds > clock_jump_threshold_seconds_) {
      fail(DiagnosticsMessage::FAILURE_TIME_JUMP, "ROS clock moved forward discontinuously");
      return;
    }
    if (clock_delta_seconds <= kTimeEpsilon
        && wall_delta_seconds > clock_stall_timeout_seconds_) {
      fail(DiagnosticsMessage::FAILURE_TIME_JUMP, "ROS clock is paused");
      return;
    }
  }

  const bool reference_is_fresh = referenceFresh(now, reference_age);
  const bool state_is_fresh = stateFresh(now, state_age);
  if (reference_age < -kTimeEpsilon || state_age < -kTimeEpsilon) {
    fail(DiagnosticsMessage::FAILURE_TIME_JUMP, "input timestamp is in the future");
    return;
  }
  if (!reference_is_fresh) {
    fail(DiagnosticsMessage::FAILURE_STALE_REFERENCE, "reference timeout");
    return;
  }
  if (!state_is_fresh) {
    fail(DiagnosticsMessage::FAILURE_STALE_STATE, "vehicle state timeout");
    return;
  }

  const double reference_time_seconds =
      (now - reference_->epoch).seconds();
  if (!std::isfinite(reference_time_seconds) || reference_time_seconds < 0.0) {
    fail(DiagnosticsMessage::FAILURE_TIME_JUMP, "reference epoch is in the future");
    return;
  }

  sampler_configuration_.hold_after_end = reference_->hold_after_end;
  const auto sampled = sampler_.sample(
      reference_->trajectory, reference_time_seconds, sampler_configuration_);
  if (!sampled.valid) {
    fail(DiagnosticsMessage::FAILURE_INVALID_REFERENCE, "reference sampling failed");
    return;
  }

  std::string core_error;
  if (!activateCoreIfNeeded(reference_time_seconds, *state_, core_error)) {
    fail(DiagnosticsMessage::FAILURE_INVALID_STATE, core_error);
    return;
  }
  if (!resetCoreIfNeeded(reference_time_seconds, *state_)) {
    fail(DiagnosticsMessage::FAILURE_RESET, "failed to re-anchor after state reset");
    return;
  }

  const auto solve_start = std::chrono::steady_clock::now();
  const auto result = core_->update(sampled.horizon, reference_time_seconds);
  const auto solve_end = std::chrono::steady_clock::now();
  const double solve_time_seconds = std::chrono::duration<double>(
      solve_end - solve_start).count();
  const double update_period_seconds = 1.0 / update_rate_hz_;
  const bool deadline_missed = solve_time_seconds > update_period_seconds;

  if (!result.valid) {
    fail(
        mapCoreFailure(result.failure_reason),
        "MPC core update failed: " + std::to_string(
            static_cast<int>(result.failure_reason)));
    publishDiagnostics(
        now, mapCoreFailure(result.failure_reason), last_failure_detail_, false,
        false, reference_age, state_age, solve_time_seconds, &result,
        reset_detected_, deadline_missed);
    const bool clock_progressed = !last_update_time_ || now > *last_update_time_;
    last_update_time_ = now;
    if (clock_progressed) {
      last_clock_progress_wall_time_ = wall_now;
    }
    return;
  }

  publishCommand(result, sampled.horizon, now);
  publishPrediction(result, sampled.horizon, now);
  ++published_sequence_;
  const bool clock_progressed = !last_update_time_ || now > *last_update_time_;
  last_update_time_ = now;
  if (clock_progressed) {
    last_clock_progress_wall_time_ = wall_now;
  }
  consecutive_failures_ = 0;
  last_failure_reason_ = deadline_missed
    ? DiagnosticsMessage::FAILURE_DEADLINE
    : DiagnosticsMessage::FAILURE_NONE;
  last_failure_detail_ = deadline_missed
    ? "command generated after update deadline"
    : "ok";
  publishDiagnostics(
      now, last_failure_reason_, last_failure_detail_, true, true,
      reference_age, state_age, solve_time_seconds, &result, reset_detected_,
      deadline_missed);
}

bool MpcTrajectoryNode::convertReference(
    const ReferenceMessage& message,
    CachedReference& converted,
    std::string& error) const
{
  if (message.header.frame_id != frame_id_) {
    error = "reference frame_id does not match configured frame_id";
    return false;
  }
  if (message.header.stamp.sec < 0
      || (message.header.stamp.sec == 0 && message.header.stamp.nanosec == 0)) {
    error = "reference header.stamp must define a non-zero epoch";
    return false;
  }
  if (message.points.empty()) {
    error = "reference contains no points";
    return false;
  }

  std::vector<mpc_control::ReferencePoint> points;
  points.reserve(message.points.size());
  bool all_yaw_valid = true;
  double previous_time = -std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index < message.points.size(); ++index) {
    const auto& point = message.points[index];
    if (!finitePoint(point)) {
      error = "reference contains non-finite point data";
      return false;
    }
    const double time_seconds = durationSeconds(point.time_from_start);
    if (!std::isfinite(time_seconds) || time_seconds < 0.0
        || (index > 0 && time_seconds <= previous_time)) {
      error = "reference point times must be finite, non-negative and increasing";
      return false;
    }
    previous_time = time_seconds;
    all_yaw_valid = all_yaw_valid && point.yaw_valid;

    mpc_control::ReferencePoint converted_point;
    converted_point.time_seconds = time_seconds;
    converted_point.position = Eigen::Vector3d(
        point.position[0], point.position[1], point.position[2]);
    converted_point.velocity = Eigen::Vector3d(
        point.velocity[0], point.velocity[1], point.velocity[2]);
    converted_point.acceleration = Eigen::Vector3d(
        point.acceleration[0], point.acceleration[1], point.acceleration[2]);
    converted_point.yaw = point.yaw_valid ? point.yaw : 0.0;
    converted_point.yaw_rate = point.yaw_valid ? point.yaw_rate : 0.0;
    points.push_back(converted_point);
  }

  mpc_control::ReferenceValidationOptions options =
      sampler_configuration_.validation;
  const mpc_control::ReferenceTrajectory trajectory(std::move(points));
  const auto validation = trajectory.validate(options);
  if (!validation.valid) {
    error = "reference validation failed at point "
        + std::to_string(validation.index);
    return false;
  }

  converted.trajectory = trajectory;
  converted.epoch = rclcpp::Time(message.header.stamp, RCL_ROS_TIME);
  converted.trajectory_id = message.trajectory_id;
  converted.hold_after_end = message.hold_after_end;
  converted.yaw_valid = all_yaw_valid;
  return true;
}

bool MpcTrajectoryNode::convertState(
    const StateMessage& message,
    CachedState& converted,
    std::string& error) const
{
  if (message.header.frame_id != frame_id_) {
    error = "vehicle state frame_id does not match configured frame_id";
    return false;
  }
  if (message.header.stamp.sec < 0) {
    error = "vehicle state header.stamp cannot be negative";
    return false;
  }
  if (!finiteState(message)) {
    error = "vehicle state contains non-finite data";
    return false;
  }
  if (!message.valid || !message.position_valid || !message.velocity_valid
      || !message.yaw_valid
      || (use_measured_acceleration_on_activate_
          && !message.acceleration_valid)) {
    error = "vehicle state validity flags are not sufficient for activation";
    return false;
  }

  converted.state.position = Eigen::Vector3d(
      message.position[0], message.position[1], message.position[2]);
  converted.state.velocity = Eigen::Vector3d(
      message.velocity[0], message.velocity[1], message.velocity[2]);
  converted.state.acceleration = message.acceleration_valid
    ? Eigen::Vector3d(
        message.acceleration[0], message.acceleration[1], message.acceleration[2])
    : Eigen::Vector3d::Zero();
  converted.state.yaw = message.yaw;
  converted.state.yaw_rate = message.yaw_rate;
  converted.sequence = message.sequence;
  converted.reset_counter = message.reset_counter;
  converted.reset_counter_valid = message.reset_counter_valid;
  return true;
}

bool MpcTrajectoryNode::referenceFresh(
    const rclcpp::Time& now, double& age_seconds) const
{
  if (!reference_) {
    age_seconds = 0.0;
    return false;
  }
  age_seconds = (now - reference_->received_at).seconds();
  return std::isfinite(age_seconds)
      && age_seconds >= 0.0
      && age_seconds <= reference_timeout_seconds_;
}

bool MpcTrajectoryNode::stateFresh(
    const rclcpp::Time& now, double& age_seconds) const
{
  if (!state_) {
    age_seconds = 0.0;
    return false;
  }
  age_seconds = (now - state_->measured_at).seconds();
  return std::isfinite(age_seconds)
      && age_seconds >= 0.0
      && age_seconds <= state_timeout_seconds_;
}

bool MpcTrajectoryNode::activateCoreIfNeeded(
    const double reference_time_seconds,
    const CachedState& state,
    std::string& error)
{
  if (core_->active()) {
    return true;
  }
  const auto activation = core_->activate(
      state.state, reference_time_seconds);
  if (!activation.valid) {
    error = "MPC core activation failed: "
        + std::to_string(static_cast<int>(activation.failure_reason));
    return false;
  }
  last_reset_counter_ = state.reset_counter;
  last_reset_counter_valid_ = state.reset_counter_valid;
  reset_detected_ = true;
  return true;
}

bool MpcTrajectoryNode::resetCoreIfNeeded(
    const double reference_time_seconds,
    const CachedState& state)
{
  if (!state.reset_counter_valid) {
    return true;
  }
  if (!last_reset_counter_valid_) {
    last_reset_counter_ = state.reset_counter;
    last_reset_counter_valid_ = true;
    return true;
  }
  if (state.reset_counter == last_reset_counter_) {
    return true;
  }

  core_->reset(state.state, reference_time_seconds);
  if (!core_->active()) {
    return false;
  }
  last_reset_counter_ = state.reset_counter;
  reset_detected_ = true;
  return true;
}

void MpcTrajectoryNode::publishCommand(
    const mpc_control::MpcUpdateResult& result,
    const mpc_control::ReferenceHorizon& horizon,
    const rclcpp::Time& now)
{
  if (!command_publisher_ || !command_publisher_->is_activated()) {
    return;
  }
  CommandMessage message;
  message.header.stamp = static_cast<builtin_interfaces::msg::Time>(now);
  message.header.frame_id = frame_id_;
  message.trajectory_id = reference_->trajectory_id;
  message.sequence = published_sequence_;
  message.reference_time_from_start = durationMessage(
      horizon.current_time_seconds + horizon.relative_times[0]);
  for (std::size_t axis = 0; axis < 3; ++axis) {
    message.position[axis] = result.command.position(axis);
    message.velocity[axis] = result.command.velocity(axis);
    message.acceleration[axis] = result.command.acceleration(axis);
  }
  message.yaw = reference_->yaw_valid ? result.command.yaw : 0.0;
  message.yaw_rate = reference_->yaw_valid ? result.command.yaw_rate : 0.0;
  message.yaw_valid = reference_->yaw_valid;
  command_publisher_->publish(message);
}

void MpcTrajectoryNode::publishPrediction(
    const mpc_control::MpcUpdateResult& result,
    const mpc_control::ReferenceHorizon& horizon,
    const rclcpp::Time& now)
{
  if (!prediction_publisher_ || !prediction_publisher_->is_activated()) {
    return;
  }
  PredictionMessage message;
  message.header.stamp = static_cast<builtin_interfaces::msg::Time>(now);
  message.header.frame_id = frame_id_;
  message.trajectory_id = reference_->trajectory_id;
  message.source_sequence = published_sequence_;
  message.points.resize(mpc_control::kReferenceHorizonLength);
  for (std::size_t index = 0;
       index < mpc_control::kReferenceHorizonLength; ++index) {
    auto& point = message.points[index];
    point.time_from_start = durationMessage(horizon.relative_times[index]);
    for (std::size_t axis = 0; axis < 3; ++axis) {
      point.position[axis] = result.prediction[index].position(axis);
      point.velocity[axis] = result.prediction[index].velocity(axis);
      point.acceleration[axis] = result.prediction[index].acceleration(axis);
    }
    point.yaw = reference_->yaw_valid ? result.prediction[index].yaw : 0.0;
    point.yaw_rate = reference_->yaw_valid
      ? result.prediction[index].yaw_rate : 0.0;
    point.yaw_valid = reference_->yaw_valid;
  }
  prediction_publisher_->publish(message);
}

void MpcTrajectoryNode::publishDiagnostics(
    const rclcpp::Time& now,
    const std::uint8_t failure_reason,
    const std::string& detail,
    const bool command_published,
    const bool prediction_valid,
    const double reference_age_seconds,
    const double state_age_seconds,
    const double solve_time_seconds,
    const mpc_control::MpcUpdateResult* result,
    const bool reset_detected,
    const bool deadline_missed)
{
  if (!diagnostics_publisher_) {
    return;
  }
  DiagnosticsMessage message;
  message.header.stamp = static_cast<builtin_interfaces::msg::Time>(now);
  message.header.frame_id = frame_id_;
  message.lifecycle_state = lifecycleStateId(get_current_state().id());
  message.failure_reason = failure_reason;
  message.trajectory_id = reference_ ? reference_->trajectory_id : 0U;
  message.sequence = published_sequence_;
  message.command_published = command_published;
  message.reference_valid = reference_valid_;
  message.vehicle_state_valid = state_valid_;
  message.prediction_valid = prediction_valid;
  message.reference_age_seconds = finiteOrZero(reference_age_seconds);
  message.vehicle_state_age_seconds = finiteOrZero(state_age_seconds);
  message.update_period_seconds = finiteOrZero(1.0 / update_rate_hz_);
  message.solve_time_seconds = finiteOrZero(solve_time_seconds);
  message.prediction_size = prediction_valid
    ? static_cast<std::uint32_t>(mpc_control::kReferenceHorizonLength) : 0U;
  message.consecutive_failures = consecutive_failures_;
  message.deadline_missed = deadline_missed;
  message.reset_detected = reset_detected;
  message.tracking_error_valid = false;
  message.tracking_error_position = {0.0, 0.0, 0.0};
  message.tracking_error_norm = 0.0;
  message.detail = detail;
  if (result) {
    message.reset_detected = message.reset_detected
      || result->diagnostics.state_reset;
    for (std::size_t axis = 0; axis < 3; ++axis) {
      message.solver_iterations[axis] = result->diagnostics.solver_iterations[axis];
    }
  }
  diagnostics_publisher_->publish(message);
}

std::uint8_t MpcTrajectoryNode::lifecycleStateId(
    const std::uint8_t lifecycle_id) noexcept
{
  switch (lifecycle_id) {
    case lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED:
      return DiagnosticsMessage::LIFECYCLE_UNCONFIGURED;
    case lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE:
      return DiagnosticsMessage::LIFECYCLE_INACTIVE;
    case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE:
      return DiagnosticsMessage::LIFECYCLE_ACTIVE;
    default:
      return DiagnosticsMessage::LIFECYCLE_ERROR;
  }
}

std::uint8_t MpcTrajectoryNode::mapCoreFailure(
    const mpc_control::MpcFailureReason reason) noexcept
{
  switch (reason) {
    case mpc_control::MpcFailureReason::SolverFailureX:
      return DiagnosticsMessage::FAILURE_SOLVER_X;
    case mpc_control::MpcFailureReason::SolverFailureY:
      return DiagnosticsMessage::FAILURE_SOLVER_Y;
    case mpc_control::MpcFailureReason::SolverFailureZ:
      return DiagnosticsMessage::FAILURE_SOLVER_Z;
    case mpc_control::MpcFailureReason::NonFiniteOutput:
      return DiagnosticsMessage::FAILURE_NONFINITE_OUTPUT;
    case mpc_control::MpcFailureReason::InvalidReferenceHorizon:
    case mpc_control::MpcFailureReason::TimeMismatch:
      return DiagnosticsMessage::FAILURE_INVALID_REFERENCE;
    case mpc_control::MpcFailureReason::InvalidVehicleState:
    case mpc_control::MpcFailureReason::NotActive:
      return DiagnosticsMessage::FAILURE_INVALID_STATE;
    case mpc_control::MpcFailureReason::None:
      return DiagnosticsMessage::FAILURE_NONE;
    default:
      return DiagnosticsMessage::FAILURE_INVALID_CONFIGURATION;
  }
}

bool MpcTrajectoryNode::finitePoint(
    const ReferenceMessage::_points_type::value_type& point) noexcept
{
  return finiteArray(point.position)
      && finiteArray(point.velocity)
      && finiteArray(point.acceleration)
      && (!point.yaw_valid
          || (std::isfinite(point.yaw) && std::isfinite(point.yaw_rate)));
}

bool MpcTrajectoryNode::finiteState(const StateMessage& message) noexcept
{
  return finiteArray(message.position)
      && finiteArray(message.velocity)
      && std::isfinite(message.yaw_rate)
      && std::isfinite(message.yaw)
      && (!message.acceleration_valid || finiteArray(message.acceleration));
}

double MpcTrajectoryNode::durationSeconds(
    const builtin_interfaces::msg::Duration& duration) noexcept
{
  return static_cast<double>(duration.sec)
      + static_cast<double>(duration.nanosec) * 1.0e-9;
}

builtin_interfaces::msg::Duration MpcTrajectoryNode::durationMessage(
    const double seconds) noexcept
{
  builtin_interfaces::msg::Duration duration;
  if (!std::isfinite(seconds) || seconds < 0.0) {
    return duration;
  }
  const auto whole_seconds = static_cast<std::int64_t>(std::floor(seconds));
  const auto nanoseconds = static_cast<std::int64_t>(std::llround(
      (seconds - static_cast<double>(whole_seconds)) * 1.0e9));
  duration.sec = static_cast<std::int32_t>(whole_seconds + nanoseconds / 1000000000LL);
  duration.nanosec = static_cast<std::uint32_t>(nanoseconds % 1000000000LL);
  return duration;
}

void MpcTrajectoryNode::resetRuntimeState() noexcept
{
  reference_valid_ = false;
  state_valid_ = false;
  reset_detected_ = false;
  last_failure_reason_ = DiagnosticsMessage::FAILURE_NONE;
  last_failure_detail_.clear();
  consecutive_failures_ = 0;
  published_sequence_ = 0;
  last_update_time_.reset();
  last_clock_progress_wall_time_.reset();
  last_state_timestamp_.reset();
  last_reset_counter_ = 0;
  last_reset_counter_valid_ = false;
}

}  // namespace mpc_control_ros
