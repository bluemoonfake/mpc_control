#pragma once

#include "mpc_trajectory_core.hpp"

#include <cstddef>
#include <vector>

namespace mpc_control
{

enum class TimingFailureReason
{
  None,
  InvalidConfiguration,
  InvalidReference,
  ActivationFailure,
  UpdateFailure,
};

struct TimingBenchmarkConfiguration
{
  MpcCoreConfiguration core{};
  ReferenceSamplerConfig sampler{};
  double start_time_seconds = 0.0;
  double update_period_seconds = 0.02;
  std::size_t warmup_samples = 100;
  std::size_t measured_samples = 1000;
  double p99_budget_fraction = 0.50;
  double max_budget_fraction = 0.80;
};

struct TimingStatistics
{
  bool valid = false;
  std::size_t sample_count = 0;
  double mean_microseconds = 0.0;
  double p95_microseconds = 0.0;
  double p99_microseconds = 0.0;
  double max_microseconds = 0.0;
};

struct TimingBenchmarkResult
{
  bool valid = false;
  TimingFailureReason failure_reason = TimingFailureReason::InvalidConfiguration;
  MpcFailureReason core_failure_reason = MpcFailureReason::None;
  ReferenceError reference_error = ReferenceError::None;
  std::size_t completed_updates = 0;
  TimingStatistics statistics{};
  double period_microseconds = 0.0;
  double p99_budget_microseconds = 0.0;
  double max_budget_microseconds = 0.0;
  bool p99_within_budget = false;
  bool max_within_budget = false;
};

TimingStatistics summarizeTimingSamples(
    const std::vector<double>& samples_microseconds) noexcept;

bool validTimingBenchmarkConfiguration(
    const TimingBenchmarkConfiguration& configuration) noexcept;

TimingBenchmarkResult runTimingBenchmark(
    const ReferenceTrajectory& reference,
    const VehicleState& initial_state,
    const TimingBenchmarkConfiguration& configuration = {}) noexcept;

const char* timingFailureReasonName(TimingFailureReason reason) noexcept;

}  // namespace mpc_control
