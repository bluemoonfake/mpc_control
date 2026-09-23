#include "mpc_controller/ports/solver.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

using mpc_controller::coupled_mpc::Clock;
using mpc_controller::coupled_mpc::Configuration;
using mpc_controller::coupled_mpc::Input;
using mpc_controller::coupled_mpc::Limits;
using mpc_controller::coupled_mpc::Reference;
using mpc_controller::coupled_mpc::Solver;
using mpc_controller::coupled_mpc::State;

namespace {

struct Sample {
  double wall = 0.0;
  double problem = 0.0;
  double vector_copy = 0.0;
  double warm_start = 0.0;
  double solve = 0.0;
  double postprocess = 0.0;
  double solve_cpu = 0.0;
  int iterations = 0;
  int status = 0;
  bool valid = false;
};

double percentile(std::vector<double> values, double fraction)
{
  if (values.empty()) return std::numeric_limits<double>::quiet_NaN();
  std::sort(values.begin(), values.end());
  const double index = fraction * static_cast<double>(values.size() - 1);
  const auto lower = static_cast<std::size_t>(std::floor(index));
  const auto upper = static_cast<std::size_t>(std::ceil(index));
  const double weight = index - static_cast<double>(lower);
  return values[lower] * (1.0 - weight) + values[upper] * weight;
}

Configuration makeConfiguration()
{
  Configuration config;
  config.dt_first = 0.20;
  config.dt_later = 0.20;
  config.model_time_constant_xyz = {0.25, 0.25, 0.08};
  config.stage_weights_xy = {80.0, 550.0, 2.0};
  config.terminal_weights_xy = {100.0, 600.0, 4.0};
  config.stage_weights_z = {200.0, 350.0, 2.0};
  config.terminal_weights_z = {300.0, 400.0, 4.0};
  config.control_weights = {1.5, 1.5, 4.0};
  config.control_rate_weights = {40.0, 40.0, 10.0};
  config.max_speed_xy = 18.0;
  config.max_speed_z = 3.0;
  config.max_acceleration_xy = 6.0;
  config.max_acceleration_z = 5.0;
  config.max_control_xy = 6.0;
  config.max_control_z = 3.0;
  config.max_control_rate_xy = 8.0;
  config.max_control_rate_z = 4.0;
  config.gravity_m_s2 = 9.80665;
  config.max_tilt_rad = 0.6108652381980153;
  config.min_collective_specific_force_m_s2 = 1.0;
  config.max_collective_specific_force_m_s2 = 16.0;
  // The IPM solve is synchronous and HPIPM has no wall-clock cancellation.
  // Keep this finite cap aligned with the first 100 Hz timing qualification.
  config.max_iterations = 50;
  config.absolute_tolerance = 0.0003;
  config.relative_tolerance = 0.001;
  return config;
}

Reference makeReference(double offset, bool aggressive)
{
  Reference reference{};
  for (std::size_t step = 0; step < reference.size(); ++step) {
    State state = State::Zero();
    state(0) = (aggressive ? 30.0 : 5.0) + offset;
    state(1) = (aggressive ? -30.0 : 2.0) - 0.5 * offset;
    state(2) = aggressive ? 10.0 : 3.0;
    state(3) = aggressive ? 18.0 : 1.0;
    state(4) = aggressive ? -18.0 : -0.2;
    state(5) = aggressive ? 3.0 : 0.0;
    state(6) = aggressive ? 6.0 : 0.0;
    state(7) = aggressive ? -6.0 : 0.0;
    state(8) = aggressive ? 5.0 : 0.0;
    reference[step] = state;
  }
  return reference;
}

Reference makeBoundaryReference(int sample)
{
  Reference reference{};
  const std::size_t boundary = 8U + static_cast<std::size_t>(sample % 4);
  const int pattern = (sample / 4) % 4;
  for (std::size_t step = 0; step < reference.size(); ++step) {
    const bool before = step < boundary;
    const double sign = before ? 1.0 : -1.0;
    State state = State::Zero();
    if (pattern == 0) {
      state(0) = before ? 30.0 : -30.0;
      state(3) = before ? 18.0 : -18.0;
    } else if (pattern == 1) {
      state(0) = before ? 30.0 : -30.0;
      state(1) = before ? -30.0 : 30.0;
      state(3) = before ? 18.0 : -18.0;
      state(4) = before ? -18.0 : 18.0;
    } else if (pattern == 2) {
      state(0) = 30.0;
      state(3) = before ? 18.0 : 0.0;
    } else {
      state(2) = before ? 10.0 : -10.0;
      state(5) = before ? 3.0 : -3.0;
    }
    state(6) = pattern == 3 ? 0.0 : 6.0 * sign;
    state(7) = pattern == 1 ? -6.0 * sign : 0.0;
    state(8) = pattern == 3 ? 5.0 * sign : 0.0;
    reference[step] = state;
  }
  return reference;
}

std::array<Input, mpc_controller::coupled_mpc::kHorizonLength>
reconstructControls(
  const State &initial, const mpc_controller::coupled_mpc::Result &result,
  const Configuration &config)
{
  std::array<Input, mpc_controller::coupled_mpc::kHorizonLength> controls{};
  Eigen::Vector3d previous_acceleration = initial.segment<3>(6);
  for (std::size_t step = 0; step < controls.size(); ++step) {
    const double dt = step == 0U ? config.dt_first : config.dt_later;
    Eigen::Vector3d alpha;
    for (std::size_t axis = 0; axis < 3U; ++axis) {
      alpha(static_cast<Eigen::Index>(axis)) = std::exp(
        -dt / config.model_time_constant_xyz[axis]);
    }
    const Eigen::Vector3d acceleration = result.prediction[step].segment<3>(6);
    controls[step] = (acceleration - alpha.cwiseProduct(previous_acceleration))
      .cwiseQuotient(Eigen::Vector3d::Ones() - alpha);
    previous_acceleration = acceleration;
  }
  return controls;
}

double normalizedRateRatio(
  const Input &current, const Input &previous, std::size_t step,
  const Configuration &config)
{
  const double dt = step == 0U ? config.dt_first : config.dt_later;
  const double polygon_scale = std::cos(M_PI / 6.0);
  const double xy_limit = config.max_control_rate_xy * dt * polygon_scale;
  const double z_limit = config.max_control_rate_z * dt;
  double ratio = std::abs(current.z() - previous.z()) / z_limit;
  for (std::size_t side = 0; side < 3U; ++side) {
    const double angle = 2.0 * M_PI * static_cast<double>(side) / 6.0;
    const double projection = std::cos(angle) * (current.x() - previous.x())
      + std::sin(angle) * (current.y() - previous.y());
    ratio = std::max(ratio, std::abs(projection) / xy_limit);
  }
  return ratio;
}

void runBoundaryStress(Solver &solver, const Configuration &config, const Limits &limits)
{
  const State initial = State::Zero();
  const Input last_control = Input::Zero();
  for (int sample = 0; sample < 256; ++sample) {
    solver.reset();
    const auto reference = makeBoundaryReference(sample);
    const auto result = solver.solve(
      initial, reference, last_control, limits, Clock::now() + std::chrono::seconds(1));
    if (!result.valid) {
      std::cout << "STRESS sample=" << sample << " valid=0\n";
      continue;
    }
    const auto controls = reconstructControls(
      initial, result, config);
    double omitted_max_ratio = 0.0;
    double first_omitted_ratio = 0.0;
    for (std::size_t step = 0; step < controls.size(); ++step) {
      const Input previous = step == 0U ? last_control : controls[step - 1U];
      if (step >= mpc_controller::coupled_mpc::kRateConstraintHorizonLength) {
        const double ratio = normalizedRateRatio(controls[step], previous, step, config);
        omitted_max_ratio = std::max(omitted_max_ratio, ratio);
        if (step == mpc_controller::coupled_mpc::kRateConstraintHorizonLength) {
          first_omitted_ratio = ratio;
        }
      }
    }
    double first_ten_state_l1 = 0.0;
    for (std::size_t step = 0; step < 10U; ++step) {
      first_ten_state_l1 += result.prediction[step].lpNorm<1>();
    }
    std::cout << "STRESS sample=" << sample << " valid=1"
              << " u0=" << result.first_control.x() << ','
              << result.first_control.y() << ',' << result.first_control.z()
              << " omitted_first=" << first_omitted_ratio
              << " omitted_max=" << omitted_max_ratio
              << " first10_state_l1=" << first_ten_state_l1
              << " iterations=" << result.iterations << '\n';
  }
}

void printStats(const std::string &name, const std::vector<Sample> &samples)
{
  const auto report = [&](const std::string &field, auto getter) {
    std::vector<double> values;
    values.reserve(samples.size());
    for (const auto &sample : samples) values.push_back(getter(sample) * 1.0e3);
    std::cout << "  " << field << "_ms p50=" << percentile(values, 0.50)
              << " p95=" << percentile(values, 0.95)
              << " p99=" << percentile(values, 0.99)
              << " p99.9=" << percentile(values, 0.999)
              << " max=" << *std::max_element(values.begin(), values.end()) << '\n';
  };
  std::vector<double> iterations;
  std::array<std::size_t, 8> status_counts{};
  std::size_t valid = 0;
  for (const auto &sample : samples) {
    iterations.push_back(static_cast<double>(sample.iterations));
    if (sample.valid) ++valid;
    if (sample.status >= 0 &&
        static_cast<std::size_t>(sample.status) < status_counts.size()) {
      ++status_counts[static_cast<std::size_t>(sample.status)];
    }
  }
  std::cout << "SCENARIO " << name << " samples=" << samples.size()
            << " valid=" << valid << '\n';
  report("wall", [](const Sample &sample) {return sample.wall;});
  report("problem", [](const Sample &sample) {return sample.problem;});
  report("vector_copy", [](const Sample &sample) {return sample.vector_copy;});
  report("warm_start", [](const Sample &sample) {return sample.warm_start;});
  report("solve", [](const Sample &sample) {return sample.solve;});
  report("postprocess", [](const Sample &sample) {return sample.postprocess;});
  report("solve_cpu", [](const Sample &sample) {return sample.solve_cpu;});
  std::cout << "  iterations p50=" << percentile(iterations, 0.50)
            << " p95=" << percentile(iterations, 0.95)
            << " p99=" << percentile(iterations, 0.99)
            << " max=" << *std::max_element(iterations.begin(), iterations.end()) << '\n';
  std::cout << "  status_counts success=" << status_counts[0]
            << " invalid_input=" << status_counts[1]
            << " infeasible_bounds=" << status_counts[2]
            << " factorization_failure=" << status_counts[3]
            << " deadline_exceeded=" << status_counts[4]
            << " max_iterations=" << status_counts[5]
            << " non_finite_output=" << status_counts[6]
            << " output_limit=" << status_counts[7] << '\n';
}

}  // namespace

int main(int argc, char **argv)
{
  std::cout << std::fixed << std::setprecision(6);
  const auto config = makeConfiguration();
  const Limits limits{
    config.max_speed_xy, config.max_speed_z, config.max_acceleration_xy,
    config.max_acceleration_z, config.max_control_rate_xy, config.max_control_rate_z};
  Solver solver(config);
  if (!solver.configured()) {
    std::cerr << "solver not configured\n";
    return 2;
  }
  if (argc > 1 && std::string(argv[1]) == "--boundary-stress") {
    runBoundaryStress(solver, config, limits);
    return 0;
  }
  std::cout << "FORMULATION state="
            << mpc_controller::coupled_mpc::kStateDimension
            << " input=" << mpc_controller::coupled_mpc::kInputDimension
            << " horizon=" << mpc_controller::coupled_mpc::kHorizonLength
            << " decisions=" << mpc_controller::coupled_mpc::kDecisionVariableCount
            << " constraints=" << mpc_controller::coupled_mpc::kConstraintRowCount
            << " ocp_nx=" << mpc_controller::coupled_mpc::kOcpStateDimension
            << " ocp_nu=" << mpc_controller::coupled_mpc::kInputDimension
            << " ocp_ng=" << mpc_controller::coupled_mpc::kOcpGeneralConstraintCount
            << " ocp_ns=0"
            << " ocp_total_stage_variables="
            << mpc_controller::coupled_mpc::kOcpStageVariableCount
            << " polygon_sides=" << mpc_controller::coupled_mpc::kPolygonSides
            << " dt_first=" << config.dt_first
            << " dt_later=" << config.dt_later << '\n';

  const auto run = [&](const std::string &name, bool cold, bool moving, bool aggressive) {
    std::vector<Sample> samples;
    samples.reserve(10000);
    State initial = State::Zero();
    Input last_control = Input::Zero();
    for (int index = 0; index < 200; ++index) {
      const auto result = solver.solve(
        initial, makeReference(moving ? 0.0005 * index : 0.0, aggressive), last_control,
        limits, Clock::now() + std::chrono::seconds(1));
      if (result.valid) last_control = result.first_control;
    }
    for (int index = 0; index < 10000; ++index) {
      if (cold) solver.reset();
      const auto start = Clock::now();
      const auto result = solver.solve(
        initial, makeReference(moving ? 0.0005 * index : 0.0, aggressive), last_control,
        limits, Clock::now() + std::chrono::milliseconds(9));
      const double wall = std::chrono::duration<double>(Clock::now() - start).count();
      samples.push_back({
        wall, result.problem_update_seconds, result.vector_copy_seconds,
        result.warm_start_seconds, result.solve_seconds,
        result.result_postprocess_seconds, result.solve_cpu_seconds,
        result.iterations, static_cast<int>(result.status), result.valid});
      if (result.valid) last_control = result.first_control;
    }
    printStats(name, samples);
  };

  run("warm_hover", false, false, false);
  run("warm_moving_reference", false, true, false);
  run("warm_aggressive_active_bounds", false, true, true);
  run("cold_start", true, true, true);
  return 0;
}
