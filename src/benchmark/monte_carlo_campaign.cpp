#include <mpc_control/monte_carlo_campaign.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <numeric>
#include <random>
#include <sstream>
#include <thread>
#include <utility>
#include <vector>

namespace mpc_control
{

namespace
{

constexpr double kMaximumReferenceDelaySeconds = 0.10;
constexpr double kConstraintTolerance = 1.0e-6;
constexpr double kPi = 3.14159265358979323846;

struct TrialVariation
{
  MpcCoreConfiguration core{};
  double reference_delay_seconds = 0.0;
  Eigen::Vector3d finish = Eigen::Vector3d::Zero();
  VehicleState initial_state{};
};

struct TrialOutcome
{
  bool execution_valid = false;
  bool constraint_valid = false;
  bool non_finite = false;
  MpcFailureReason failure_reason = MpcFailureReason::None;
  std::size_t completed_updates = 0U;
  std::size_t constraint_violations = 0U;
  double final_position_error = 0.0;
  double final_velocity_norm = 0.0;
  double final_acceleration_norm = 0.0;
  std::size_t timing_overrun_updates = 0U;
};

double randomValue(
    std::mt19937_64& generator,
    const double lower,
    const double upper)
{
  std::uniform_real_distribution<double> distribution(lower, upper);
  return distribution(generator);
}

CampaignMetricSummary summarize(std::vector<double> samples)
{
  CampaignMetricSummary summary;
  if (samples.empty()) {
    return summary;
  }
  std::sort(samples.begin(), samples.end());
  const double sum = std::accumulate(samples.cbegin(), samples.cend(), 0.0);
  const auto nearest_rank = [&samples](const double quantile) {
    const auto rank = static_cast<std::size_t>(std::ceil(
        quantile * static_cast<double>(samples.size())));
    const auto index = std::min(
        samples.size() - 1U, rank == 0U ? 0U : rank - 1U);
    return samples[index];
  };
  summary.sample_count = samples.size();
  summary.mean = sum / static_cast<double>(samples.size());
  summary.p95 = nearest_rank(0.95);
  summary.p99 = nearest_rank(0.99);
  summary.maximum = samples.back();
  return summary;
}

Eigen::Vector3d randomVector(
    std::mt19937_64& generator,
    const double lower,
    const double upper)
{
  return Eigen::Vector3d(
      randomValue(generator, lower, upper),
      randomValue(generator, lower, upper),
      randomValue(generator, lower, upper));
}

ReferencePoint minimumJerkPoint(
    const double time,
    const Eigen::Vector3d& finish)
{
  const double s = time;
  const double s2 = s * s;
  const double s3 = s2 * s;
  const double s4 = s3 * s;
  const double s5 = s4 * s;
  const double position_scale = 10.0 * s3 - 15.0 * s4 + 6.0 * s5;
  const double velocity_scale = 30.0 * s2 - 60.0 * s3 + 30.0 * s4;
  const double acceleration_scale = 60.0 * s - 180.0 * s2 + 120.0 * s3;

  ReferencePoint point;
  point.time_seconds = time;
  point.position = finish * position_scale;
  point.velocity = finish * velocity_scale;
  point.acceleration = finish * acceleration_scale;
  return point;
}

ReferenceTrajectory makeReference(
    const Eigen::Vector3d& finish,
    const double delay_seconds)
{
  std::vector<ReferencePoint> points;
  points.reserve(delay_seconds > 0.0 ? 22U : 21U);
  if (delay_seconds > 0.0) {
    ReferencePoint pre_hold;
    pre_hold.time_seconds = -delay_seconds;
    points.push_back(pre_hold);
  }
  for (int index = 0; index <= 20; ++index) {
    points.push_back(minimumJerkPoint(
        static_cast<double>(index) * 0.05, finish));
  }
  return ReferenceTrajectory(std::move(points));
}

VehicleState randomInitialState(std::mt19937_64& generator)
{
  VehicleState state;
  state.position = randomVector(generator, -0.15, 0.15);
  state.velocity = randomVector(generator, -0.15, 0.15);
  state.acceleration = randomVector(generator, -0.10, 0.10);
  state.yaw = randomValue(generator, -kPi, kPi);
  state.yaw_rate = randomValue(generator, -0.20, 0.20);
  return state;
}

MpcCoreConfiguration randomCoreConfiguration(std::mt19937_64& generator)
{
  MpcCoreConfiguration configuration;
  configuration.use_measured_acceleration_on_activate = true;
  configuration.horizontal_solver.dt_first =
      randomValue(generator, 0.009, 0.011);
  configuration.horizontal_solver.dt_later =
      randomValue(generator, 0.18, 0.22);
  configuration.vertical_solver.dt_first =
      configuration.horizontal_solver.dt_first;
  configuration.vertical_solver.dt_later =
      configuration.horizontal_solver.dt_later;
  configuration.horizontal_solver.model_p1 =
      randomValue(generator, -0.02, 0.02);
  configuration.horizontal_solver.model_p2 =
      randomValue(generator, 0.98, 1.02);
  configuration.vertical_solver.model_p1 =
      randomValue(generator, 0.48, 0.52);
  configuration.vertical_solver.model_p2 =
      randomValue(generator, 0.48, 0.52);

  const double max_speed = randomValue(generator, 1.85, 2.0);
  const double max_acceleration = randomValue(generator, 3.0, 5.0);
  const double max_control = randomValue(generator, 1.8, 2.0);
  const double max_control_rate = randomValue(generator, 4.5, 5.0);
  configuration.horizontal_solver.max_speed = max_speed;
  configuration.vertical_solver.max_speed = max_speed;
  configuration.horizontal_solver.max_acceleration = max_acceleration;
  configuration.vertical_solver.max_acceleration = max_acceleration;
  configuration.horizontal_solver.max_control = max_control;
  configuration.vertical_solver.max_control = max_control;
  configuration.horizontal_solver.max_control_rate = max_control_rate;
  configuration.vertical_solver.max_control_rate = max_control_rate;
  return configuration;
}

TrialVariation randomVariation(std::mt19937_64& generator)
{
  TrialVariation variation;
  variation.core = randomCoreConfiguration(generator);
  variation.reference_delay_seconds = randomValue(
      generator, 0.0, kMaximumReferenceDelaySeconds);
  variation.finish = Eigen::Vector3d(1.0, -0.5, 0.5) + Eigen::Vector3d(
      randomValue(generator, -0.20, 0.20),
      randomValue(generator, -0.20, 0.20),
      randomValue(generator, -0.10, 0.10));
  variation.initial_state = randomInitialState(generator);
  return variation;
}

std::vector<int> jitterSchedule(
    std::mt19937_64& generator,
    const std::size_t count,
    const int maximum_jitter_microseconds)
{
  std::uniform_int_distribution<int> distribution(
      0, maximum_jitter_microseconds);
  std::vector<int> schedule;
  schedule.reserve(count);
  for (std::size_t index = 0; index < count; ++index) {
    schedule.push_back(distribution(generator));
  }
  return schedule;
}

class JitteredBackend final : public SolverBackend
{
public:
  JitteredBackend(
      const SolverConfiguration& configuration,
      std::vector<int> jitter_microseconds)
  : backend_(configuration), jitter_microseconds_(std::move(jitter_microseconds))
  {
  }

  SolverResult solve(const SolverRequest& request) noexcept override
  {
    if (!jitter_microseconds_.empty()) {
      const int delay = jitter_microseconds_[next_jitter_index_
          % jitter_microseconds_.size()];
      ++next_jitter_index_;
      std::this_thread::sleep_for(std::chrono::microseconds(delay));
    }
    return backend_.solve(request);
  }

  bool configured() const noexcept
  {
    return backend_.isConfigured();
  }

private:
  MrsSolverBackend backend_;
  std::vector<int> jitter_microseconds_;
  std::size_t next_jitter_index_ = 0U;
};

TrialOutcome runTrial(
    const TrialVariation& variation,
    const MonteCarloCampaignConfiguration& configuration,
    std::mt19937_64& generator,
    std::vector<double>* timing_samples = nullptr)
{
  TrialOutcome outcome;
  const auto reference = makeReference(
      variation.finish, variation.reference_delay_seconds);
  const std::size_t solve_count = configuration.updates_per_trial * 3U;
  JitteredBackend x_backend(
      variation.core.horizontal_solver,
      jitterSchedule(
          generator, solve_count, configuration.maximum_jitter_microseconds));
  JitteredBackend y_backend(
      variation.core.horizontal_solver,
      jitterSchedule(
          generator, solve_count, configuration.maximum_jitter_microseconds));
  JitteredBackend z_backend(
      variation.core.vertical_solver,
      jitterSchedule(
          generator, solve_count, configuration.maximum_jitter_microseconds));
  if (!x_backend.configured() || !y_backend.configured()
      || !z_backend.configured()) {
    outcome.failure_reason = MpcFailureReason::InvalidConfiguration;
    return outcome;
  }

  const std::array<SolverBackend*, 3> backends{
      &x_backend, &y_backend, &z_backend};
  MpcTrajectoryCore core(variation.core, backends);
  if (!core.configured()) {
    outcome.failure_reason = MpcFailureReason::InvalidConfiguration;
    return outcome;
  }
  const auto activation = core.activate(variation.initial_state, 0.0);
  if (!activation.valid) {
    outcome.failure_reason = activation.failure_reason;
    return outcome;
  }

  ReferenceSamplerConfig sampler_configuration;
  sampler_configuration.dt_first = variation.core.horizontal_solver.dt_first;
  sampler_configuration.dt_later = variation.core.horizontal_solver.dt_later;
  const ReferenceSampler sampler;

  for (std::size_t step = 0; step < configuration.updates_per_trial; ++step) {
    const double current_time =
        static_cast<double>(step) * configuration.update_period_seconds;
    auto sampled = sampler.sample(
        reference,
        current_time - variation.reference_delay_seconds,
        sampler_configuration);
    if (!sampled.valid) {
      outcome.failure_reason = MpcFailureReason::InvalidReferenceHorizon;
      return outcome;
    }
    sampled.horizon.current_time_seconds = current_time;
    for (std::size_t index = 0; index < kReferenceHorizonLength; ++index) {
      sampled.horizon.samples[index].time_seconds =
          current_time + sampled.horizon.relative_times[index];
    }

    const auto start = timing_samples == nullptr
        ? std::chrono::steady_clock::time_point{}
        : std::chrono::steady_clock::now();
    const auto update = core.update(sampled.horizon, current_time);
    if (timing_samples != nullptr) {
      const auto finish = std::chrono::steady_clock::now();
      const double elapsed_microseconds =
          std::chrono::duration<double, std::micro>(finish - start).count();
      timing_samples->push_back(elapsed_microseconds);
      if (elapsed_microseconds
          > configuration.update_period_seconds * 1.0e6) {
        ++outcome.timing_overrun_updates;
      }
    }
    if (!update.valid) {
      outcome.failure_reason = update.failure_reason;
      return outcome;
    }
    ++outcome.completed_updates;

    for (const auto& prediction : update.prediction) {
      if (!prediction.valid || !prediction.position.allFinite()
          || !prediction.velocity.allFinite()
          || !prediction.acceleration.allFinite()) {
        outcome.non_finite = true;
        outcome.failure_reason = MpcFailureReason::NonFiniteOutput;
        return outcome;
      }
      for (std::size_t axis = 0; axis < 3U; ++axis) {
        const auto& solver = axis == 2U
            ? variation.core.vertical_solver
            : variation.core.horizontal_solver;
        if (std::abs(prediction.velocity(axis))
                > solver.max_speed + kConstraintTolerance
            || std::abs(prediction.acceleration(axis))
                > solver.max_acceleration + kConstraintTolerance) {
          ++outcome.constraint_violations;
        }
      }
    }
  }

  const auto& final_state = core.virtualState();
  outcome.final_position_error =
      (final_state.position - variation.finish).norm();
  outcome.final_velocity_norm = final_state.velocity.norm();
  outcome.final_acceleration_norm = final_state.acceleration.norm();
  outcome.constraint_valid = outcome.constraint_violations == 0U;
  outcome.execution_valid = outcome.completed_updates
      == configuration.updates_per_trial;
  return outcome;
}

bool validConfiguration(
    const MonteCarloCampaignConfiguration& configuration) noexcept
{
  return configuration.trial_count > 0U
      && configuration.updates_per_trial > 0U
      && std::isfinite(configuration.update_period_seconds)
      && configuration.update_period_seconds > 0.0
      && configuration.maximum_jitter_microseconds >= 0;
}

bool validFullHorizonConfiguration(
    const FullHorizonLoadConfiguration& configuration) noexcept
{
  return configuration.trial_count > 0U
      && configuration.updates_per_trial > 0U
      && std::isfinite(configuration.update_period_seconds)
      && configuration.update_period_seconds > 0.0
      && configuration.maximum_jitter_microseconds >= 0
      && configuration.cpu_load_threads > 0U
      && std::isfinite(configuration.p99_budget_fraction)
      && configuration.p99_budget_fraction > 0.0
      && configuration.p99_budget_fraction <= 1.0
      && std::isfinite(configuration.max_budget_fraction)
      && configuration.max_budget_fraction > 0.0
      && configuration.max_budget_fraction <= 1.0
      && std::isfinite(configuration.final_position_error_limit)
      && configuration.final_position_error_limit >= 0.0
      && std::isfinite(configuration.final_velocity_norm_limit)
      && configuration.final_velocity_norm_limit >= 0.0
      && std::isfinite(configuration.final_acceleration_norm_limit)
      && configuration.final_acceleration_norm_limit >= 0.0;
}

class CpuLoadGuard final
{
public:
  CpuLoadGuard() = default;
  CpuLoadGuard(const CpuLoadGuard&) = delete;
  CpuLoadGuard& operator=(const CpuLoadGuard&) = delete;

  ~CpuLoadGuard()
  {
    stop_.store(true, std::memory_order_relaxed);
    for (auto& worker : workers_) {
      if (worker.joinable()) {
        worker.join();
      }
    }
  }

  bool start(const std::size_t thread_count) noexcept
  {
    try {
      workers_.reserve(thread_count);
      for (std::size_t index = 0U; index < thread_count; ++index) {
        workers_.emplace_back([this]() {
          std::uint64_t value = 0x9e3779b97f4a7c15ULL;
          while (!stop_.load(std::memory_order_relaxed)) {
            for (std::size_t iteration = 0U; iteration < 4096U; ++iteration) {
              value = value * 1664525ULL + 1013904223ULL;
              value ^= value >> 13U;
            }
          }
          sink_.fetch_xor(value, std::memory_order_relaxed);
        });
      }
      return true;
    } catch (...) {
      stop_.store(true, std::memory_order_relaxed);
      for (auto& worker : workers_) {
        if (worker.joinable()) {
          worker.join();
        }
      }
      workers_.clear();
      return false;
    }
  }

private:
  std::atomic<bool> stop_{false};
  std::atomic<std::uint64_t> sink_{0U};
  std::vector<std::thread> workers_;
};

void appendMetric(
    std::ostringstream& output,
    const char* name,
    const CampaignMetricSummary& metric,
    const bool trailing_comma)
{
  output << "\"" << name << "\":{"
         << "\"sample_count\":" << metric.sample_count << ","
         << std::setprecision(17)
         << "\"mean\":" << metric.mean << ","
         << "\"p95\":" << metric.p95 << ","
         << "\"p99\":" << metric.p99 << ","
         << "\"max\":" << metric.maximum << "}"
         << (trailing_comma ? "," : "");
}

void appendTiming(
    std::ostringstream& output,
    const FullHorizonTimingSummary& timing)
{
  output << "\"timing\":{";
  output << "\"sample_count\":" << timing.sample_count << ",";
  output << std::setprecision(17)
         << "\"mean_microseconds\":" << timing.mean_microseconds << ","
         << "\"p95_microseconds\":" << timing.p95_microseconds << ","
         << "\"p99_microseconds\":" << timing.p99_microseconds << ","
         << "\"maximum_microseconds\":" << timing.maximum_microseconds << ","
         << "\"overrun_updates\":" << timing.overrun_updates << ","
         << "\"overrun_trials\":" << timing.overrun_trials << ","
         << "\"p99_budget_microseconds\":"
         << timing.p99_budget_microseconds << ","
         << "\"max_budget_microseconds\":"
         << timing.max_budget_microseconds << ","
         << "\"p99_within_budget\":" << timing.p99_within_budget << ","
         << "\"max_within_budget\":" << timing.max_within_budget << "}";
}

}  // namespace

MonteCarloCampaignReport runMonteCarloCampaign(
    const MonteCarloCampaignConfiguration& configuration) noexcept
{
  MonteCarloCampaignReport report;
  report.seed = configuration.seed;
  report.trial_count = configuration.trial_count;
  report.updates_per_trial = configuration.updates_per_trial;
  if (!validConfiguration(configuration)) {
    return report;
  }

  try {
    std::mt19937_64 generator(configuration.seed);
    std::vector<double> position_errors;
    std::vector<double> velocity_norms;
    std::vector<double> acceleration_norms;
    position_errors.reserve(configuration.trial_count);
    velocity_norms.reserve(configuration.trial_count);
    acceleration_norms.reserve(configuration.trial_count);

    for (std::size_t trial = 0; trial < configuration.trial_count; ++trial) {
      const auto variation = randomVariation(generator);
      const auto outcome = runTrial(variation, configuration, generator);
      report.completed_updates += outcome.completed_updates;
      if (outcome.execution_valid && outcome.constraint_valid
          && !outcome.non_finite) {
        ++report.successful_trials;
      }
      if (!outcome.execution_valid) {
        ++report.failure_transitions;
        if (outcome.failure_reason == MpcFailureReason::SolverFailureX) {
          ++report.solver_failure_counts[0];
        } else if (outcome.failure_reason == MpcFailureReason::SolverFailureY) {
          ++report.solver_failure_counts[1];
        } else if (outcome.failure_reason == MpcFailureReason::SolverFailureZ) {
          ++report.solver_failure_counts[2];
        }
      }
      if (outcome.constraint_violations > 0U) {
        ++report.constraint_violation_trials;
      }
      if (outcome.non_finite) {
        ++report.non_finite_trials;
      }
      if (outcome.execution_valid) {
        position_errors.push_back(outcome.final_position_error);
        velocity_norms.push_back(outcome.final_velocity_norm);
        acceleration_norms.push_back(outcome.final_acceleration_norm);
      }
    }

    report.final_position_error = summarize(std::move(position_errors));
    report.final_velocity_norm = summarize(std::move(velocity_norms));
    report.final_acceleration_norm = summarize(std::move(acceleration_norms));
    report.valid = report.successful_trials == report.trial_count
        && report.failure_transitions == 0U
        && report.constraint_violation_trials == 0U
        && report.non_finite_trials == 0U;
  } catch (...) {
    report.valid = false;
  }
  return report;
}

FullHorizonLoadReport runFullHorizonLoadCampaign(
    const FullHorizonLoadConfiguration& configuration) noexcept
{
  FullHorizonLoadReport report;
  report.seed = configuration.seed;
  report.trial_count = configuration.trial_count;
  report.updates_per_trial = configuration.updates_per_trial;
  report.cpu_load_threads = configuration.cpu_load_threads;
  report.update_period_seconds = configuration.update_period_seconds;
  report.maximum_jitter_microseconds =
      configuration.maximum_jitter_microseconds;
  report.p99_budget_fraction = configuration.p99_budget_fraction;
  report.max_budget_fraction = configuration.max_budget_fraction;
  report.final_position_error_limit = configuration.final_position_error_limit;
  report.final_velocity_norm_limit = configuration.final_velocity_norm_limit;
  report.final_acceleration_norm_limit =
      configuration.final_acceleration_norm_limit;
  if (!validFullHorizonConfiguration(configuration)) {
    return report;
  }

  try {
    CpuLoadGuard cpu_load;
    if (!cpu_load.start(configuration.cpu_load_threads)) {
      return report;
    }

    MonteCarloCampaignConfiguration trial_configuration;
    trial_configuration.trial_count = 1U;
    trial_configuration.updates_per_trial = configuration.updates_per_trial;
    trial_configuration.update_period_seconds =
        configuration.update_period_seconds;
    trial_configuration.maximum_jitter_microseconds =
        configuration.maximum_jitter_microseconds;

    std::mt19937_64 generator(configuration.seed);
    std::vector<double> position_errors;
    std::vector<double> velocity_norms;
    std::vector<double> acceleration_norms;
    std::vector<double> timing_samples;
    position_errors.reserve(configuration.trial_count);
    velocity_norms.reserve(configuration.trial_count);
    acceleration_norms.reserve(configuration.trial_count);
    timing_samples.reserve(
        configuration.trial_count * configuration.updates_per_trial);

    for (std::size_t trial = 0U; trial < configuration.trial_count; ++trial) {
      const auto variation = randomVariation(generator);
      const auto outcome = runTrial(
          variation, trial_configuration, generator, &timing_samples);
      report.completed_updates += outcome.completed_updates;
      report.timing.overrun_updates += outcome.timing_overrun_updates;
      if (outcome.timing_overrun_updates > 0U) {
        ++report.timing.overrun_trials;
      }

      if (outcome.execution_valid && outcome.constraint_valid
          && !outcome.non_finite) {
        position_errors.push_back(outcome.final_position_error);
        velocity_norms.push_back(outcome.final_velocity_norm);
        acceleration_norms.push_back(outcome.final_acceleration_norm);
        const bool converged =
            outcome.final_position_error <= configuration.final_position_error_limit
            && outcome.final_velocity_norm
                <= configuration.final_velocity_norm_limit
            && outcome.final_acceleration_norm
                <= configuration.final_acceleration_norm_limit;
        if (converged) {
          ++report.successful_trials;
        } else {
          ++report.convergence_limit_misses;
        }
      } else {
        if (!outcome.execution_valid) {
          ++report.failure_transitions;
          if (outcome.failure_reason == MpcFailureReason::SolverFailureX) {
            ++report.solver_failure_counts[0];
          } else if (outcome.failure_reason == MpcFailureReason::SolverFailureY) {
            ++report.solver_failure_counts[1];
          } else if (outcome.failure_reason == MpcFailureReason::SolverFailureZ) {
            ++report.solver_failure_counts[2];
          }
        }
        if (outcome.constraint_violations > 0U) {
          ++report.constraint_violation_trials;
        }
        if (outcome.non_finite) {
          ++report.non_finite_trials;
        }
      }
    }

    report.final_position_error = summarize(std::move(position_errors));
    report.final_velocity_norm = summarize(std::move(velocity_norms));
    report.final_acceleration_norm = summarize(std::move(acceleration_norms));
    const auto timing = summarize(std::move(timing_samples));
    report.timing.sample_count = timing.sample_count;
    report.timing.mean_microseconds = timing.mean;
    report.timing.p95_microseconds = timing.p95;
    report.timing.p99_microseconds = timing.p99;
    report.timing.maximum_microseconds = timing.maximum;
    report.timing.p99_budget_microseconds =
        configuration.update_period_seconds * 1.0e6
        * configuration.p99_budget_fraction;
    report.timing.max_budget_microseconds =
        configuration.update_period_seconds * 1.0e6
        * configuration.max_budget_fraction;
    report.timing.p99_within_budget =
        report.timing.sample_count > 0U
        && report.timing.p99_microseconds
            <= report.timing.p99_budget_microseconds;
    report.timing.max_within_budget =
        report.timing.sample_count > 0U
        && report.timing.maximum_microseconds
            <= report.timing.max_budget_microseconds;
    report.valid = report.successful_trials == report.trial_count
        && report.failure_transitions == 0U
        && report.convergence_limit_misses == 0U
        && report.constraint_violation_trials == 0U
        && report.non_finite_trials == 0U
        && report.timing.p99_within_budget
        && report.timing.max_within_budget;
  } catch (...) {
    report.valid = false;
  }
  return report;
}

std::string MonteCarloCampaignReport::toJson() const
{
  std::ostringstream output;
  output << std::boolalpha << "{"
         << "\"schema\":\"mpc_control.monte_carlo_campaign.v1\","
         << "\"valid\":" << valid << ","
         << "\"seed\":" << seed << ","
         << "\"trial_count\":" << trial_count << ","
         << "\"updates_per_trial\":" << updates_per_trial << ","
         << "\"successful_trials\":" << successful_trials << ","
         << "\"failure_transitions\":" << failure_transitions << ","
         << "\"constraint_violation_trials\":"
         << constraint_violation_trials << ","
         << "\"non_finite_trials\":" << non_finite_trials << ","
         << "\"completed_updates\":" << completed_updates << ","
         << "\"solver_failure_counts\":{"
         << "\"x\":" << solver_failure_counts[0] << ","
         << "\"y\":" << solver_failure_counts[1] << ","
         << "\"z\":" << solver_failure_counts[2] << "},";
  appendMetric(output, "final_position_error", final_position_error, true);
  appendMetric(output, "final_velocity_norm", final_velocity_norm, true);
  appendMetric(output, "final_acceleration_norm", final_acceleration_norm, false);
  output << "}";
  return output.str();
}

bool writeMonteCarloCampaignReport(
    const MonteCarloCampaignReport& report,
    const std::string& output_path) noexcept
{
  try {
    std::ofstream output(output_path);
    if (!output.is_open()) {
      return false;
    }
    output << report.toJson() << '\n';
    return output.good();
  } catch (...) {
    return false;
  }
}

std::string FullHorizonLoadReport::toJson() const
{
  std::ostringstream output;
  output << std::boolalpha << "{"
         << "\"schema\":\"mpc_control.full_horizon_load.v1\","
         << "\"valid\":" << valid << ","
         << "\"seed\":" << seed << ","
         << "\"trial_count\":" << trial_count << ","
         << "\"updates_per_trial\":" << updates_per_trial << ","
         << "\"cpu_load_threads\":" << cpu_load_threads << ","
         << std::setprecision(17)
         << "\"update_period_seconds\":" << update_period_seconds << ","
         << "\"maximum_jitter_microseconds\":"
         << maximum_jitter_microseconds << ","
         << "\"p99_budget_fraction\":" << p99_budget_fraction << ","
         << "\"max_budget_fraction\":" << max_budget_fraction << ","
         << "\"final_position_error_limit\":"
         << final_position_error_limit << ","
         << "\"final_velocity_norm_limit\":"
         << final_velocity_norm_limit << ","
         << "\"final_acceleration_norm_limit\":"
         << final_acceleration_norm_limit << ","
         << "\"successful_trials\":" << successful_trials << ","
         << "\"failure_transitions\":" << failure_transitions << ","
         << "\"convergence_limit_misses\":" << convergence_limit_misses << ","
         << "\"constraint_violation_trials\":"
         << constraint_violation_trials << ","
         << "\"non_finite_trials\":" << non_finite_trials << ","
         << "\"completed_updates\":" << completed_updates << ","
         << "\"solver_failure_counts\":{"
         << "\"x\":" << solver_failure_counts[0] << ","
         << "\"y\":" << solver_failure_counts[1] << ","
         << "\"z\":" << solver_failure_counts[2] << "},";
  appendMetric(output, "final_position_error", final_position_error, true);
  appendMetric(output, "final_velocity_norm", final_velocity_norm, true);
  appendMetric(output, "final_acceleration_norm", final_acceleration_norm, true);
  appendTiming(output, timing);
  output << "}";
  return output.str();
}

bool writeFullHorizonLoadReport(
    const FullHorizonLoadReport& report,
    const std::string& output_path) noexcept
{
  try {
    std::ofstream output(output_path);
    if (!output.is_open()) {
      return false;
    }
    output << report.toJson() << '\n';
    return output.good();
  } catch (...) {
    return false;
  }
}

}  // namespace mpc_control
