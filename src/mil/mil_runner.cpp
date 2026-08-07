#include <mpc_control/mil_runner.hpp>

#include <mpc_control/discrete_virtual_axis_model.hpp>

#include <algorithm>
#include <array>
#include <cmath>

namespace mpc_control
{

namespace
{

constexpr double kTimeComparisonTolerance = 1.0e-9;

class RecordingSolverBackend final : public SolverBackend
{
public:
  explicit RecordingSolverBackend(const SolverConfiguration& configuration)
  : backend_(configuration)
  {
  }

  SolverResult solve(const SolverRequest& request) noexcept override
  {
    last_request_ = request;
    last_result_ = backend_.solve(request);
    return last_result_;
  }

  bool configured() const noexcept
  {
    return backend_.isConfigured();
  }

  const SolverRequest& lastRequest() const noexcept
  {
    return last_request_;
  }

  const SolverResult& lastResult() const noexcept
  {
    return last_result_;
  }

private:
  MrsSolverBackend backend_;
  SolverRequest last_request_{};
  SolverResult last_result_{};
};

bool closeEnough(const double lhs, const double rhs, const double tolerance)
{
  return std::abs(lhs - rhs) <= tolerance;
}

bool compatibleTimeGrid(const MilRunConfiguration& configuration) noexcept
{
  const auto& horizontal = configuration.core.horizontal_solver;
  const auto& vertical = configuration.core.vertical_solver;
  return closeEnough(
             configuration.sampler.dt_first,
             horizontal.dt_first,
             kTimeComparisonTolerance)
      && closeEnough(
             configuration.sampler.dt_later,
             horizontal.dt_later,
             kTimeComparisonTolerance)
      && closeEnough(
             vertical.dt_first,
             horizontal.dt_first,
             kTimeComparisonTolerance)
      && closeEnough(
             vertical.dt_later,
             horizontal.dt_later,
             kTimeComparisonTolerance);
}

bool finiteMetrics(const MilMetrics& metrics) noexcept
{
  return std::isfinite(metrics.shaping_position_rmse)
      && std::isfinite(metrics.shaping_position_max)
      && std::isfinite(metrics.final_position_error)
      && std::isfinite(metrics.final_velocity_norm)
      && std::isfinite(metrics.final_acceleration_norm)
      && std::isfinite(metrics.max_virtual_velocity)
      && std::isfinite(metrics.max_virtual_acceleration)
      && std::isfinite(metrics.max_jerk)
      && std::isfinite(metrics.max_command_total_variation)
      && std::isfinite(metrics.max_model_consistency_error);
}

}  // namespace

MilRunResult MilRunner::run(
    const ReferenceTrajectory& reference,
    const VehicleState& initial_state,
    const MilRunConfiguration& configuration) const noexcept
{
  MilRunResult result;
  if (!validConfiguration(configuration)) {
    result.failure_reason = MilFailureReason::InvalidConfiguration;
    return result;
  }

  try {
    const auto reference_validation =
        reference.validate(configuration.sampler.validation);
    if (!reference_validation.valid) {
      result.failure_reason = MilFailureReason::InvalidReference;
      result.reference_error = reference_validation.error;
      return result;
    }

    RecordingSolverBackend x_backend(configuration.core.horizontal_solver);
    RecordingSolverBackend y_backend(configuration.core.horizontal_solver);
    RecordingSolverBackend z_backend(configuration.core.vertical_solver);
    if (!x_backend.configured()
        || !y_backend.configured()
        || !z_backend.configured()) {
      result.failure_reason = MilFailureReason::InvalidConfiguration;
      return result;
    }

    const std::array<SolverBackend*, 3> backends{
        &x_backend, &y_backend, &z_backend};
    MpcTrajectoryCore core(configuration.core, backends);
    if (!core.configured()) {
      result.failure_reason = MilFailureReason::InvalidConfiguration;
      return result;
    }

    const auto activation =
        core.activate(initial_state, configuration.start_time_seconds);
    if (!activation.valid) {
      result.failure_reason = MilFailureReason::CoreFailure;
      result.core_failure_reason = activation.failure_reason;
      return result;
    }

    const std::array<DiscreteVirtualAxisModel, 3> models{
        DiscreteVirtualAxisModel({
            configuration.core.horizontal_solver.dt_first,
            configuration.core.horizontal_solver.model_p1,
            configuration.core.horizontal_solver.model_p2}),
        DiscreteVirtualAxisModel({
            configuration.core.horizontal_solver.dt_first,
            configuration.core.horizontal_solver.model_p1,
            configuration.core.horizontal_solver.model_p2}),
        DiscreteVirtualAxisModel({
            configuration.core.vertical_solver.dt_first,
            configuration.core.vertical_solver.model_p1,
            configuration.core.vertical_solver.model_p2})};
    if (!models[0].valid() || !models[1].valid() || !models[2].valid()) {
      result.failure_reason = MilFailureReason::InvalidConfiguration;
      return result;
    }

    const auto step_count = static_cast<std::size_t>(std::floor(
        configuration.duration_seconds
            / configuration.update_period_seconds
            + kTimeComparisonTolerance)) + 1U;
    const ReferenceSampler sampler;
    double squared_shaping_error = 0.0;
    TrajectoryCommand previous_command;

    for (std::size_t step = 0; step < step_count; ++step) {
      const double current_time = configuration.start_time_seconds
          + static_cast<double>(step) * configuration.update_period_seconds;
      const auto sampled = sampler.sample(
          reference, current_time, configuration.sampler);
      if (!sampled.valid) {
        result.failure_reason = MilFailureReason::SamplingFailure;
        result.reference_error = sampled.error;
        return result;
      }

      const auto update = core.update(sampled.horizon, current_time);
      if (!update.valid) {
        result.failure_reason = MilFailureReason::CoreFailure;
        result.core_failure_reason = update.failure_reason;
        return result;
      }

      const Eigen::Vector3d shaping_error =
          sampled.horizon.samples[0].position - update.command.position;
      const double shaping_norm = shaping_error.norm();
      squared_shaping_error += shaping_norm * shaping_norm;
      result.metrics.shaping_position_max = std::max(
          result.metrics.shaping_position_max, shaping_norm);
      result.metrics.max_virtual_velocity = std::max(
          result.metrics.max_virtual_velocity,
          update.command.velocity.norm());
      result.metrics.max_virtual_acceleration = std::max(
          result.metrics.max_virtual_acceleration,
          update.command.acceleration.norm());

      if (step > 0U) {
        result.metrics.max_command_total_variation = std::max(
            result.metrics.max_command_total_variation,
            (update.command.position - previous_command.position).norm());
      }
      previous_command = update.command;

      const std::array<const RecordingSolverBackend*, 3> recordings{
          &x_backend, &y_backend, &z_backend};
      const std::array<const SolverConfiguration*, 3> solver_configurations{
          &configuration.core.horizontal_solver,
          &configuration.core.horizontal_solver,
          &configuration.core.vertical_solver};

      for (std::size_t axis = 0; axis < 3U; ++axis) {
        const auto& recording = *recordings[axis];
        const auto& solver_configuration = *solver_configurations[axis];
        const auto& solver_result = recording.lastResult();
        const auto& request = recording.lastRequest();
        const auto expected = models[axis].step(
            request.initial_state, solver_result.first_control);
        const Eigen::Vector3d actual(
            update.command.position(static_cast<Eigen::Index>(axis)),
            update.command.velocity(static_cast<Eigen::Index>(axis)),
            update.command.acceleration(static_cast<Eigen::Index>(axis)));
        const double model_error =
            (expected.state - actual).cwiseAbs().maxCoeff();
        result.metrics.max_model_consistency_error = std::max(
            result.metrics.max_model_consistency_error, model_error);
        if (!expected.valid || model_error > configuration.model_tolerance) {
          result.failure_reason = MilFailureReason::ModelMismatch;
          return result;
        }

        result.metrics.max_jerk = std::max(
            result.metrics.max_jerk,
            std::abs(solver_result.first_control - request.last_control)
                / solver_configuration.dt_first);
        for (const auto& prediction : update.prediction) {
          if (std::abs(prediction.velocity(static_cast<Eigen::Index>(axis)))
                  > solver_configuration.max_speed
                      + configuration.constraint_tolerance
              || std::abs(prediction.acceleration(
                     static_cast<Eigen::Index>(axis)))
                  > solver_configuration.max_acceleration
                      + configuration.constraint_tolerance
              || std::abs(solver_result.first_control)
                  > solver_configuration.max_control
                      + configuration.constraint_tolerance
              || std::abs(solver_result.first_control - request.last_control)
                  > solver_configuration.max_control_rate
                      * solver_configuration.dt_first
                      + configuration.constraint_tolerance) {
            ++result.metrics.constraint_violations;
          }
        }
      }

      result.metrics.update_count = step + 1U;
    }

    result.metrics.shaping_position_rmse = std::sqrt(
        squared_shaping_error
        / static_cast<double>(result.metrics.update_count));
    result.metrics.final_position_error =
        (previous_command.position - reference.points().back().position).norm();
    result.metrics.final_velocity_norm = previous_command.velocity.norm();
    result.metrics.final_acceleration_norm =
        previous_command.acceleration.norm();
    result.final_virtual_state = core.virtualState();

    if (!finiteMetrics(result.metrics)) {
      result.failure_reason = MilFailureReason::NonFiniteMetric;
      return result;
    }
    if (result.metrics.constraint_violations != 0U) {
      result.failure_reason = MilFailureReason::ConstraintViolation;
      return result;
    }

    result.valid = true;
    result.failure_reason = MilFailureReason::None;
    return result;
  } catch (...) {
    result.failure_reason = MilFailureReason::CoreFailure;
    result.core_failure_reason = MpcFailureReason::BackendException;
    return result;
  }
}

bool MilRunner::validConfiguration(
    const MilRunConfiguration& configuration) noexcept
{
  return std::isfinite(configuration.start_time_seconds)
      && std::isfinite(configuration.duration_seconds)
      && configuration.duration_seconds >= 0.0
      && std::isfinite(configuration.update_period_seconds)
      && configuration.update_period_seconds > 0.0
      && std::isfinite(configuration.model_tolerance)
      && configuration.model_tolerance >= 0.0
      && std::isfinite(configuration.constraint_tolerance)
      && configuration.constraint_tolerance >= 0.0
      && std::isfinite(configuration.sampler.dt_first)
      && std::isfinite(configuration.sampler.dt_later)
      && configuration.sampler.dt_first > 0.0
      && configuration.sampler.dt_later > 0.0
      && compatibleTimeGrid(configuration);
}

}  // namespace mpc_control
