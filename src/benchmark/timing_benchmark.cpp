#include <mpc_control/timing_benchmark.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <numeric>

namespace mpc_control
{

namespace
{

constexpr double kGridTolerance = 1.0e-12;

bool closeEnough(const double lhs, const double rhs) noexcept
{
  return std::abs(lhs - rhs) <= kGridTolerance;
}

double nearestRank(
    const std::vector<double>& sorted_samples,
    const double quantile) noexcept
{
  if (sorted_samples.empty()) {
    return 0.0;
  }
  const auto rank = static_cast<std::size_t>(std::ceil(
      quantile * static_cast<double>(sorted_samples.size())));
  const auto index = std::min(
      sorted_samples.size() - 1U,
      rank == 0U ? 0U : rank - 1U);
  return sorted_samples[index];
}

bool finiteCoreConfiguration(const MpcCoreConfiguration& configuration) noexcept
{
  const auto finite_solver = [](const SolverConfiguration& solver) noexcept {
    return solver.max_iterations > 0
        && std::isfinite(solver.dt_first)
        && solver.dt_first > 0.0
        && std::isfinite(solver.dt_later)
        && solver.dt_later > 0.0;
  };
  return finite_solver(configuration.horizontal_solver)
      && finite_solver(configuration.vertical_solver);
}

}  // namespace

TimingStatistics summarizeTimingSamples(
    const std::vector<double>& samples_microseconds) noexcept
{
  TimingStatistics statistics;
  if (samples_microseconds.empty()) {
    return statistics;
  }

  std::vector<double> sorted_samples = samples_microseconds;
  if (!std::all_of(
          sorted_samples.cbegin(), sorted_samples.cend(),
          [](const double sample) { return std::isfinite(sample) && sample >= 0.0; })) {
    return statistics;
  }
  std::sort(sorted_samples.begin(), sorted_samples.end());

  const double sum = std::accumulate(
      sorted_samples.cbegin(), sorted_samples.cend(), 0.0);
  statistics.valid = std::isfinite(sum);
  if (!statistics.valid) {
    return statistics;
  }
  statistics.sample_count = sorted_samples.size();
  statistics.mean_microseconds = sum / static_cast<double>(sorted_samples.size());
  statistics.p95_microseconds = nearestRank(sorted_samples, 0.95);
  statistics.p99_microseconds = nearestRank(sorted_samples, 0.99);
  statistics.max_microseconds = sorted_samples.back();
  statistics.valid = std::isfinite(statistics.mean_microseconds)
      && std::isfinite(statistics.p95_microseconds)
      && std::isfinite(statistics.p99_microseconds)
      && std::isfinite(statistics.max_microseconds);
  return statistics;
}

bool validTimingBenchmarkConfiguration(
    const TimingBenchmarkConfiguration& configuration) noexcept
{
  const auto& horizontal = configuration.core.horizontal_solver;
  const auto& vertical = configuration.core.vertical_solver;
  return finiteCoreConfiguration(configuration.core)
      && std::isfinite(configuration.start_time_seconds)
      && std::isfinite(configuration.update_period_seconds)
      && configuration.update_period_seconds > 0.0
      && configuration.measured_samples > 0U
      && std::isfinite(configuration.p99_budget_fraction)
      && configuration.p99_budget_fraction > 0.0
      && configuration.p99_budget_fraction <= 1.0
      && std::isfinite(configuration.max_budget_fraction)
      && configuration.max_budget_fraction > 0.0
      && configuration.max_budget_fraction <= 1.0
      && configuration.warmup_samples
          <= std::numeric_limits<std::size_t>::max()
              - configuration.measured_samples
      && std::isfinite(configuration.sampler.dt_first)
      && configuration.sampler.dt_first > 0.0
      && std::isfinite(configuration.sampler.dt_later)
      && configuration.sampler.dt_later > 0.0
      && closeEnough(configuration.sampler.dt_first, horizontal.dt_first)
      && closeEnough(configuration.sampler.dt_later, horizontal.dt_later)
      && closeEnough(configuration.sampler.dt_first, vertical.dt_first)
      && closeEnough(configuration.sampler.dt_later, vertical.dt_later);
}

TimingBenchmarkResult runTimingBenchmark(
    const ReferenceTrajectory& reference,
    const VehicleState& initial_state,
    const TimingBenchmarkConfiguration& configuration) noexcept
{
  TimingBenchmarkResult result;
  result.period_microseconds = configuration.update_period_seconds * 1.0e6;
  result.p99_budget_microseconds =
      result.period_microseconds * configuration.p99_budget_fraction;
  result.max_budget_microseconds =
      result.period_microseconds * configuration.max_budget_fraction;

  if (!validTimingBenchmarkConfiguration(configuration)) {
    result.failure_reason = TimingFailureReason::InvalidConfiguration;
    return result;
  }

  try {
    const auto reference_validation = reference.validate(
        configuration.sampler.validation);
    if (!reference_validation.valid) {
      result.failure_reason = TimingFailureReason::InvalidReference;
      result.reference_error = reference_validation.error;
      return result;
    }

    const std::size_t total_samples =
        configuration.warmup_samples + configuration.measured_samples;
    std::vector<ReferenceHorizon> horizons;
    horizons.reserve(total_samples);
    ReferenceSampler sampler;
    for (std::size_t index = 0; index < total_samples; ++index) {
      const double current_time = configuration.start_time_seconds
          + static_cast<double>(index) * configuration.update_period_seconds;
      const auto sampled = sampler.sample(
          reference, current_time, configuration.sampler);
      if (!sampled.valid) {
        result.failure_reason = TimingFailureReason::InvalidReference;
        result.reference_error = sampled.error;
        return result;
      }
      horizons.push_back(sampled.horizon);
    }

    MpcTrajectoryCore core(configuration.core);
    if (!core.configured()) {
      result.failure_reason = TimingFailureReason::InvalidConfiguration;
      return result;
    }
    const auto activation = core.activate(
        initial_state, configuration.start_time_seconds);
    if (!activation.valid) {
      result.failure_reason = TimingFailureReason::ActivationFailure;
      result.core_failure_reason = activation.failure_reason;
      return result;
    }

    std::vector<double> samples_microseconds;
    samples_microseconds.reserve(configuration.measured_samples);
    for (std::size_t index = 0; index < total_samples; ++index) {
      const double current_time = configuration.start_time_seconds
          + static_cast<double>(index) * configuration.update_period_seconds;
      const auto start = std::chrono::steady_clock::now();
      const auto update = core.update(horizons[index], current_time);
      const auto finish = std::chrono::steady_clock::now();
      if (!update.valid) {
        result.failure_reason = TimingFailureReason::UpdateFailure;
        result.core_failure_reason = update.failure_reason;
        result.completed_updates = index;
        return result;
      }

      ++result.completed_updates;
      if (index >= configuration.warmup_samples) {
        const auto elapsed = std::chrono::duration<double, std::micro>(
            finish - start).count();
        samples_microseconds.push_back(elapsed);
      }
    }

    result.statistics = summarizeTimingSamples(samples_microseconds);
    if (!result.statistics.valid) {
      result.failure_reason = TimingFailureReason::UpdateFailure;
      return result;
    }
    result.p99_within_budget = result.statistics.p99_microseconds
        < result.p99_budget_microseconds;
    result.max_within_budget = result.statistics.max_microseconds
        < result.max_budget_microseconds;
    result.valid = true;
    result.failure_reason = TimingFailureReason::None;
    return result;
  } catch (...) {
    result.failure_reason = TimingFailureReason::UpdateFailure;
    result.core_failure_reason = MpcFailureReason::BackendException;
    return result;
  }
}

const char* timingFailureReasonName(const TimingFailureReason reason) noexcept
{
  switch (reason) {
    case TimingFailureReason::None:
      return "none";
    case TimingFailureReason::InvalidConfiguration:
      return "invalid_configuration";
    case TimingFailureReason::InvalidReference:
      return "invalid_reference";
    case TimingFailureReason::ActivationFailure:
      return "activation_failure";
    case TimingFailureReason::UpdateFailure:
      return "update_failure";
  }
  return "unknown";
}

}  // namespace mpc_control
