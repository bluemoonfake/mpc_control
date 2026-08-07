#pragma once

#include "mpc_trajectory_core.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

namespace mpc_control
{

struct CampaignMetricSummary
{
  std::size_t sample_count = 0U;
  double mean = 0.0;
  double p95 = 0.0;
  double p99 = 0.0;
  double maximum = 0.0;
};

struct MonteCarloCampaignConfiguration
{
  std::uint64_t seed = 20260807U;
  std::size_t trial_count = 10000U;
  std::size_t updates_per_trial = 3U;
  double update_period_seconds = 0.02;
  int maximum_jitter_microseconds = 200;
};

struct MonteCarloCampaignReport
{
  bool valid = false;
  std::uint64_t seed = 0U;
  std::size_t trial_count = 0U;
  std::size_t updates_per_trial = 0U;
  std::size_t successful_trials = 0U;
  std::size_t failure_transitions = 0U;
  std::size_t constraint_violation_trials = 0U;
  std::size_t non_finite_trials = 0U;
  std::size_t completed_updates = 0U;
  std::array<std::size_t, 3> solver_failure_counts{0U, 0U, 0U};
  CampaignMetricSummary final_position_error{};
  CampaignMetricSummary final_velocity_norm{};
  CampaignMetricSummary final_acceleration_norm{};

  std::string toJson() const;
};

struct FullHorizonLoadConfiguration
{
  std::uint64_t seed = 20260811U;
  std::size_t trial_count = 50U;
  std::size_t updates_per_trial = 301U;
  double update_period_seconds = 0.02;
  int maximum_jitter_microseconds = 200;
  std::size_t cpu_load_threads = 1U;
  double p99_budget_fraction = 0.50;
  double max_budget_fraction = 0.80;
  double final_position_error_limit = 0.25;
  double final_velocity_norm_limit = 0.60;
  double final_acceleration_norm_limit = 1.00;
};

struct FullHorizonTimingSummary
{
  std::size_t sample_count = 0U;
  double mean_microseconds = 0.0;
  double p95_microseconds = 0.0;
  double p99_microseconds = 0.0;
  double maximum_microseconds = 0.0;
  std::size_t overrun_updates = 0U;
  std::size_t overrun_trials = 0U;
  double p99_budget_microseconds = 0.0;
  double max_budget_microseconds = 0.0;
  bool p99_within_budget = false;
  bool max_within_budget = false;
};

struct FullHorizonLoadReport
{
  bool valid = false;
  std::uint64_t seed = 0U;
  std::size_t trial_count = 0U;
  std::size_t updates_per_trial = 0U;
  std::size_t cpu_load_threads = 0U;
  double update_period_seconds = 0.0;
  int maximum_jitter_microseconds = 0;
  double p99_budget_fraction = 0.0;
  double max_budget_fraction = 0.0;
  double final_position_error_limit = 0.0;
  double final_velocity_norm_limit = 0.0;
  double final_acceleration_norm_limit = 0.0;
  std::size_t successful_trials = 0U;
  std::size_t failure_transitions = 0U;
  std::size_t convergence_limit_misses = 0U;
  std::size_t constraint_violation_trials = 0U;
  std::size_t non_finite_trials = 0U;
  std::size_t completed_updates = 0U;
  std::array<std::size_t, 3> solver_failure_counts{0U, 0U, 0U};
  CampaignMetricSummary final_position_error{};
  CampaignMetricSummary final_velocity_norm{};
  CampaignMetricSummary final_acceleration_norm{};
  FullHorizonTimingSummary timing{};

  std::string toJson() const;
};

MonteCarloCampaignReport runMonteCarloCampaign(
    const MonteCarloCampaignConfiguration& configuration = {}) noexcept;

bool writeMonteCarloCampaignReport(
    const MonteCarloCampaignReport& report,
    const std::string& output_path) noexcept;

FullHorizonLoadReport runFullHorizonLoadCampaign(
    const FullHorizonLoadConfiguration& configuration = {}) noexcept;

bool writeFullHorizonLoadReport(
    const FullHorizonLoadReport& report,
    const std::string& output_path) noexcept;

}  // namespace mpc_control
