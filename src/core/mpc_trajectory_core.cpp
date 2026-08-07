#include <mpc_control/mpc_trajectory_core.hpp>

#include <algorithm>
#include <cmath>

namespace mpc_control
{

namespace
{

bool finiteSolverConfiguration(const SolverConfiguration& configuration) noexcept
{
  if (configuration.max_iterations <= 0
      || !std::isfinite(configuration.dt_first)
      || !std::isfinite(configuration.dt_later)
      || configuration.dt_first <= 0.0
      || configuration.dt_later <= 0.0
      || !std::isfinite(configuration.model_p1)
      || !std::isfinite(configuration.model_p2)) {
    return false;
  }

  for (const double weight : configuration.stage_weights) {
    if (!std::isfinite(weight) || weight < 0.0) {
      return false;
    }
  }
  for (const double weight : configuration.terminal_weights) {
    if (!std::isfinite(weight) || weight < 0.0) {
      return false;
    }
  }

  return std::isfinite(configuration.max_speed)
      && configuration.max_speed >= 0.0
      && std::isfinite(configuration.max_acceleration)
      && configuration.max_acceleration >= 0.0
      && std::isfinite(configuration.max_control)
      && configuration.max_control >= 0.0
      && std::isfinite(configuration.max_control_rate)
      && configuration.max_control_rate >= 0.0;
}

}  // namespace

MpcTrajectoryCore::MpcTrajectoryCore()
{
  configure(configuration_);
}

MpcTrajectoryCore::MpcTrajectoryCore(
    const MpcCoreConfiguration& configuration)
{
  configure(configuration);
}

MpcTrajectoryCore::MpcTrajectoryCore(
    const MpcCoreConfiguration& configuration,
    const std::array<SolverBackend*, 3>& external_backends)
: external_backends_(true), backends_(external_backends)
{
  configure(configuration);
}

MpcConfigureResult MpcTrajectoryCore::configure(
    const MpcCoreConfiguration& configuration) noexcept
{
  MpcConfigureResult result;
  clearRuntimeState();
  configured_ = false;
  configuration_ = configuration;

  if (!validConfiguration(configuration_)) {
    result.failure_reason = MpcFailureReason::InvalidConfiguration;
    return result;
  }

  try {
    if (!external_backends_) {
      owned_backends_[0] = std::make_unique<MrsSolverBackend>(
          configuration_.horizontal_solver);
      owned_backends_[1] = std::make_unique<MrsSolverBackend>(
          configuration_.horizontal_solver);
      owned_backends_[2] = std::make_unique<MrsSolverBackend>(
          configuration_.vertical_solver);
      backends_[0] = owned_backends_[0].get();
      backends_[1] = owned_backends_[1].get();
      backends_[2] = owned_backends_[2].get();
    }

    if (std::any_of(
            backends_.begin(), backends_.end(),
            [](const SolverBackend* backend) { return backend == nullptr; })) {
      result.failure_reason = MpcFailureReason::InvalidConfiguration;
      return result;
    }

    if (!external_backends_
        && std::any_of(
            owned_backends_.begin(), owned_backends_.end(),
            [](const std::unique_ptr<SolverBackend>& backend) {
              const auto* mrs_backend =
                  dynamic_cast<const MrsSolverBackend*>(backend.get());
              return mrs_backend == nullptr || !mrs_backend->isConfigured();
            })) {
      result.failure_reason = MpcFailureReason::InvalidConfiguration;
      return result;
    }

    configured_ = true;
    result.valid = true;
    result.failure_reason = MpcFailureReason::None;
    return result;
  } catch (...) {
    owned_backends_ = {};
    backends_ = {nullptr, nullptr, nullptr};
    result.failure_reason = MpcFailureReason::BackendException;
    return result;
  }
}

MpcActivationResult MpcTrajectoryCore::activate(
    const VehicleState& measured_state,
    const double reference_time_seconds) noexcept
{
  MpcActivationResult result;
  if (!configured_) {
    result.failure_reason = MpcFailureReason::NotConfigured;
    return result;
  }
  if (active_) {
    result.failure_reason = MpcFailureReason::AlreadyActive;
    return result;
  }
  if (!std::isfinite(reference_time_seconds)
      || !finiteVehicleState(
          measured_state,
          configuration_.use_measured_acceleration_on_activate)) {
    result.failure_reason = MpcFailureReason::InvalidVehicleState;
    return result;
  }

  if (!initializeVirtualState(measured_state, reference_time_seconds)) {
    result.failure_reason = MpcFailureReason::InvalidVehicleState;
    return result;
  }

  active_ = true;
  state_reset_pending_ = true;
  result.valid = true;
  result.failure_reason = MpcFailureReason::None;
  return result;
}

MpcUpdateResult MpcTrajectoryCore::update(
    const ReferenceHorizon& reference_horizon,
    const double update_time_seconds) noexcept
{
  MpcUpdateResult result;
  result.diagnostics.update_count = update_count_;
  result.diagnostics.state_reset = state_reset_pending_;

  if (!configured_) {
    result.failure_reason = MpcFailureReason::NotConfigured;
    result.command.failure_reason = result.failure_reason;
    return result;
  }
  if (!active_) {
    result.failure_reason = MpcFailureReason::NotActive;
    result.command.failure_reason = result.failure_reason;
    return result;
  }
  if (!std::isfinite(update_time_seconds)
      || !validReferenceHorizon(reference_horizon)) {
    result.failure_reason = MpcFailureReason::InvalidReferenceHorizon;
    result.command.failure_reason = result.failure_reason;
    return result;
  }
  if (std::abs(
          reference_horizon.current_time_seconds - update_time_seconds)
      > configuration_.time_tolerance_seconds) {
    result.failure_reason = MpcFailureReason::TimeMismatch;
    result.command.failure_reason = result.failure_reason;
    return result;
  }

  std::array<SolverResult, 3> solver_results{};
  try {
    for (std::size_t axis = 0; axis < 3; ++axis) {
      SolverRequest request;
      request.initial_state = axisState(virtual_state_.position, axis);
      request.initial_state.y() = virtual_state_.velocity(axis);
      request.initial_state.z() = virtual_state_.acceleration(axis);
      request.last_control = last_controls_[axis];

      for (std::size_t step = 0; step < kReferenceHorizonLength; ++step) {
        request.reference[step] = Eigen::Vector3d(
            reference_horizon.samples[step].position(axis),
            reference_horizon.samples[step].velocity(axis),
            reference_horizon.samples[step].acceleration(axis));
      }

      solver_results[axis] = backends_[axis]->solve(request);
      result.diagnostics.solver_iterations[axis] = solver_results[axis].iterations;
      result.diagnostics.solver_failure_reasons[axis] =
          solver_results[axis].failure_reason;
      if (!solver_results[axis].valid) {
        result.failure_reason = axisFailure(axis);
        result.command.failure_reason = result.failure_reason;
        return result;
      }
    }
  } catch (...) {
    result.failure_reason = MpcFailureReason::BackendException;
    result.command.failure_reason = result.failure_reason;
    return result;
  }

  std::array<double, 3> next_last_controls = last_controls_;
  for (std::size_t axis = 0; axis < 3; ++axis) {
    if (!std::isfinite(solver_results[axis].first_control)) {
      result.failure_reason = MpcFailureReason::NonFiniteOutput;
      result.command.failure_reason = result.failure_reason;
      return result;
    }
    next_last_controls[axis] = solver_results[axis].first_control;
  }

  VirtualTrajectoryState next_virtual_state = virtual_state_;
  for (std::size_t axis = 0; axis < 3; ++axis) {
    const SolverState& first_prediction = solver_results[axis].prediction[0];
    next_virtual_state.position(axis) = first_prediction.x();
    next_virtual_state.velocity(axis) = first_prediction.y();
    next_virtual_state.acceleration(axis) = first_prediction.z();
  }
  next_virtual_state.yaw = reference_horizon.samples[0].yaw;
  next_virtual_state.yaw_rate = reference_horizon.samples[0].yaw_rate;
  next_virtual_state.trajectory_time_seconds =
      update_time_seconds + reference_horizon.relative_times[0];

  for (std::size_t step = 0; step < kReferenceHorizonLength; ++step) {
    auto& command = result.prediction[step];
    for (std::size_t axis = 0; axis < 3; ++axis) {
      const SolverState& prediction = solver_results[axis].prediction[step];
      command.position(axis) = prediction.x();
      command.velocity(axis) = prediction.y();
      command.acceleration(axis) = prediction.z();
    }
    command.yaw = reference_horizon.samples[step].yaw;
    command.yaw_rate = reference_horizon.samples[step].yaw_rate;
    command.valid = command.position.allFinite()
        && command.velocity.allFinite()
        && command.acceleration.allFinite()
        && std::isfinite(command.yaw)
        && std::isfinite(command.yaw_rate);
    command.failure_reason = command.valid
        ? MpcFailureReason::None
        : MpcFailureReason::NonFiniteOutput;
    if (!command.valid) {
      result.failure_reason = MpcFailureReason::NonFiniteOutput;
      result.command.failure_reason = result.failure_reason;
      return result;
    }
  }

  last_controls_ = next_last_controls;
  virtual_state_ = next_virtual_state;
  result.command = result.prediction[0];
  result.valid = true;
  result.failure_reason = MpcFailureReason::None;
  result.command.failure_reason = MpcFailureReason::None;
  ++update_count_;
  result.diagnostics.update_count = update_count_;
  state_reset_pending_ = false;
  return result;
}

void MpcTrajectoryCore::reset(
    const VehicleState& measured_state,
    const double reference_time_seconds) noexcept
{
  active_ = false;
  clearRuntimeState();
  if (configured_
      && std::isfinite(reference_time_seconds)
      && finiteVehicleState(
          measured_state,
          configuration_.use_measured_acceleration_on_activate)
      && initializeVirtualState(measured_state, reference_time_seconds)) {
    active_ = true;
    state_reset_pending_ = true;
  }
}

bool MpcTrajectoryCore::configured() const noexcept
{
  return configured_;
}

bool MpcTrajectoryCore::active() const noexcept
{
  return active_;
}

const VirtualTrajectoryState& MpcTrajectoryCore::virtualState() const noexcept
{
  return virtual_state_;
}

bool MpcTrajectoryCore::validConfiguration(
    const MpcCoreConfiguration& configuration) noexcept
{
  return std::isfinite(configuration.time_tolerance_seconds)
      && configuration.time_tolerance_seconds >= 0.0
      && finiteSolverConfiguration(configuration.horizontal_solver)
      && finiteSolverConfiguration(configuration.vertical_solver);
}

bool MpcTrajectoryCore::finiteVehicleState(
    const VehicleState& state,
    const bool require_acceleration) noexcept
{
  return state.position.allFinite()
      && state.velocity.allFinite()
      && (!require_acceleration || state.acceleration.allFinite())
      && std::isfinite(state.yaw)
      && std::isfinite(state.yaw_rate);
}

bool MpcTrajectoryCore::validReferenceHorizon(
    const ReferenceHorizon& horizon) noexcept
{
  if (!std::isfinite(horizon.current_time_seconds)
      || !std::isfinite(horizon.relative_times[0])
      || horizon.relative_times[0] <= 0.0) {
    return false;
  }
  for (std::size_t index = 0; index < kReferenceHorizonLength; ++index) {
    const auto& sample = horizon.samples[index];
    if (!std::isfinite(horizon.relative_times[index])
        || !sample.position.allFinite()
        || !sample.velocity.allFinite()
        || !sample.acceleration.allFinite()
        || !std::isfinite(sample.time_seconds)
        || !std::isfinite(sample.yaw)
        || !std::isfinite(sample.yaw_rate)) {
      return false;
    }
    if (index > 0
        && horizon.relative_times[index] <= horizon.relative_times[index - 1]) {
      return false;
    }
  }
  return true;
}

Eigen::Vector3d MpcTrajectoryCore::axisState(
    const Eigen::Vector3d& vector,
    const std::size_t axis) noexcept
{
  Eigen::Vector3d result = Eigen::Vector3d::Zero();
  result.x() = vector(axis);
  return result;
}

MpcFailureReason MpcTrajectoryCore::axisFailure(const std::size_t axis) noexcept
{
  if (axis == 0) {
    return MpcFailureReason::SolverFailureX;
  }
  if (axis == 1) {
    return MpcFailureReason::SolverFailureY;
  }
  return MpcFailureReason::SolverFailureZ;
}

void MpcTrajectoryCore::clearRuntimeState() noexcept
{
  active_ = false;
  last_controls_ = {0.0, 0.0, 0.0};
  virtual_state_ = {};
  update_count_ = 0;
  state_reset_pending_ = false;
}

bool MpcTrajectoryCore::initializeVirtualState(
    const VehicleState& measured_state,
    const double reference_time_seconds) noexcept
{
  virtual_state_.position = measured_state.position;
  virtual_state_.velocity = measured_state.velocity;
  virtual_state_.acceleration = configuration_.use_measured_acceleration_on_activate
      ? measured_state.acceleration
      : Eigen::Vector3d::Zero();
  virtual_state_.yaw = measured_state.yaw;
  virtual_state_.yaw_rate = measured_state.yaw_rate;
  virtual_state_.trajectory_time_seconds = reference_time_seconds;
  return virtual_state_.position.allFinite()
      && virtual_state_.velocity.allFinite()
      && virtual_state_.acceleration.allFinite()
      && std::isfinite(virtual_state_.yaw)
      && std::isfinite(virtual_state_.yaw_rate);
}

}  // namespace mpc_control
