#include <mpc_control/solver_backend.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

namespace mpc_control
{

namespace
{

constexpr double kNumericalTolerance = 1.0e-6;

bool finiteNonNegative(const double value) noexcept
{
  return std::isfinite(value) && value >= 0.0;
}

bool finitePositive(const double value) noexcept
{
  return std::isfinite(value) && value > 0.0;
}

bool withinLimit(const double value, const double limit) noexcept
{
  return std::abs(value) <= limit + kNumericalTolerance * std::max(1.0, limit);
}

std::vector<double> toVector(const std::array<double, kStateDimension>& values)
{
  return {values.begin(), values.end()};
}

}  // namespace

struct MrsSolverBackend::Workspace
{
  Eigen::MatrixXd state;
  Eigen::MatrixXd reference;
  Eigen::MatrixXd prediction;

  Workspace()
  : state(static_cast<Eigen::Index>(kStateDimension), 1),
    reference(static_cast<Eigen::Index>(kSolverHorizon * kStateDimension), 1),
    prediction(static_cast<Eigen::Index>(kSolverHorizon * kStateDimension), 1)
  {
    state.setZero();
    reference.setZero();
    prediction.setZero();
  }
};

MrsSolverBackend::MrsSolverBackend(const SolverConfiguration& configuration)
: configuration_(configuration),
  configured_(isConfigurationValid(configuration))
{
  if (!configured_) {
    return;
  }

  try {
    workspace_ = std::make_unique<Workspace>();
    solver_ = std::make_unique<mrs_mpc_solvers::mpc_controller::Solver>(
        "mpc_control_mrs_backend",
        false,
        configuration_.max_iterations,
        toVector(configuration_.stage_weights),
        toVector(configuration_.terminal_weights),
        configuration_.dt_first,
        configuration_.dt_later,
        configuration_.model_p1,
        configuration_.model_p2);

    solver_->setLimits(
        configuration_.max_speed,
        configuration_.max_acceleration,
        configuration_.max_control,
        configuration_.max_control_rate,
        configuration_.dt_first,
        configuration_.dt_later);

  } catch (...) {
    solver_.reset();
    configured_ = false;
    initialization_failed_ = true;
  }
}

MrsSolverBackend::~MrsSolverBackend() = default;

bool MrsSolverBackend::isConfigured() const noexcept
{
  return configured_ && solver_ != nullptr && !initialization_failed_;
}

const SolverConfiguration& MrsSolverBackend::configuration() const noexcept
{
  return configuration_;
}

bool MrsSolverBackend::isConfigurationValid(
    const SolverConfiguration& configuration) noexcept
{
  if (configuration.max_iterations <= 0
      || !finitePositive(configuration.dt_first)
      || !finitePositive(configuration.dt_later)
      || !std::isfinite(configuration.model_p1)
      || !std::isfinite(configuration.model_p2)) {
    return false;
  }

  for (const double weight : configuration.stage_weights) {
    if (!finiteNonNegative(weight)) {
      return false;
    }
  }
  for (const double weight : configuration.terminal_weights) {
    if (!finiteNonNegative(weight)) {
      return false;
    }
  }

  return finiteNonNegative(configuration.max_speed)
      && finiteNonNegative(configuration.max_acceleration)
      && finiteNonNegative(configuration.max_control)
      && finiteNonNegative(configuration.max_control_rate);
}

bool MrsSolverBackend::isRequestFinite(const SolverRequest& request) noexcept
{
  if (!request.initial_state.allFinite() || !std::isfinite(request.last_control)) {
    return false;
  }

  for (const SolverState& state : request.reference) {
    if (!state.allFinite()) {
      return false;
    }
  }

  return true;
}

bool MrsSolverBackend::predictionWithinLimits(
    const PredictionHorizon& prediction) const noexcept
{
  for (const SolverState& state : prediction) {
    if (!state.allFinite()
        || !withinLimit(state.y(), configuration_.max_speed)
        || !withinLimit(state.z(), configuration_.max_acceleration)) {
      return false;
    }
  }
  return true;
}

bool MrsSolverBackend::firstControlWithinLimits(
    const double control, const double last_control) const noexcept
{
  return std::isfinite(control)
      && withinLimit(control, configuration_.max_control)
      && withinLimit(
          control - last_control,
          configuration_.max_control_rate * configuration_.dt_first);
}

SolverResult MrsSolverBackend::solve(const SolverRequest& request) noexcept
{
  SolverResult result;

  if (!isConfigured()) {
    result.failure_reason = initialization_failed_
        ? SolverFailureReason::InitializationFailure
        : SolverFailureReason::InvalidConfiguration;
    return result;
  }

  if (!isRequestFinite(request)) {
    result.failure_reason = SolverFailureReason::NonFiniteInput;
    return result;
  }

  try {
    for (int component = 0; component < static_cast<int>(kStateDimension); ++component) {
      workspace_->state(component, 0) = request.initial_state(component);
    }

    for (std::size_t step = 0; step < kSolverHorizon; ++step) {
      for (std::size_t component = 0; component < kStateDimension; ++component) {
        workspace_->reference(
            static_cast<Eigen::Index>(step * kStateDimension + component), 0)
          = request.reference[step](static_cast<Eigen::Index>(component));
      }
    }

    solver_->setLastInput(request.last_control);
    solver_->setInitialState(workspace_->state);
    solver_->loadReference(workspace_->reference);

    result.iterations = solver_->solveMPC();

    // The upstream API has no public convergence flag. Reject the iteration
    // budget boundary conservatively instead of publishing an uncertain QP.
    if (result.iterations <= 0
        || result.iterations >= configuration_.max_iterations) {
      result.failure_reason = SolverFailureReason::SolverDidNotConverge;
      return result;
    }

    result.first_control = solver_->getFirstControlInput();
    solver_->getStates(workspace_->prediction);

    for (std::size_t step = 0; step < kSolverHorizon; ++step) {
      result.prediction[step] = SolverState(
          workspace_->prediction(static_cast<Eigen::Index>(step * kStateDimension + 0), 0),
          workspace_->prediction(static_cast<Eigen::Index>(step * kStateDimension + 1), 0),
          workspace_->prediction(static_cast<Eigen::Index>(step * kStateDimension + 2), 0));
    }

    bool prediction_finite = true;
    for (const SolverState& state : result.prediction) {
      prediction_finite = prediction_finite && state.allFinite();
    }

    if (!std::isfinite(result.first_control) || !prediction_finite) {
      result.failure_reason = SolverFailureReason::NonFiniteOutput;
      return result;
    }

    if (!predictionWithinLimits(result.prediction)
        || !firstControlWithinLimits(result.first_control, request.last_control)) {
      result.failure_reason = SolverFailureReason::OutputOutOfBounds;
      return result;
    }

    result.valid = true;
    result.failure_reason = SolverFailureReason::None;
    return result;
  } catch (...) {
    result.valid = false;
    result.failure_reason = SolverFailureReason::BackendException;
    return result;
  }
}

}  // namespace mpc_control
