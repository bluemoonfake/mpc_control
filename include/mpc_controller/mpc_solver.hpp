#pragma once

#include <Eigen/Core>

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>

namespace mpc_controller::axis_mpc
{

inline constexpr std::size_t kStateDimension = 3;
inline constexpr std::size_t kHorizonLength = 26;

using Clock = std::chrono::steady_clock;
using State = Eigen::Matrix<double, 3, 1>;
using Reference = std::array<State, kHorizonLength>;
using Prediction = std::array<State, kHorizonLength>;

enum class Status : uint8_t
{
  success = 0,
  invalid_input = 1,
  infeasible_bounds = 2,
  factorization_failure = 3,
  deadline_exceeded = 4,
  max_iterations = 5,
  non_finite_output = 6,
  output_limit = 7
};

struct Configuration
{
  double dt_first = 0.01;
  double dt_later = 0.20;
  double model_time_constant = 0.0;
  std::array<double, 3> stage_weights{};
  std::array<double, 3> terminal_weights{};
  double max_speed = 0.0;
  double max_acceleration = 0.0;
  double max_control = 0.0;
  double max_control_rate = 0.0;
  int max_iterations = 400;
  double admm_rho = 0.02;
  double absolute_tolerance = 1.0e-5;
  double relative_tolerance = 1.0e-5;
};

struct Result
{
  bool valid = false;
  bool recovery_constraint_active = false;
  Status status = Status::invalid_input;
  int iterations = 0;
  double primal_residual = std::numeric_limits<double>::infinity();
  double dual_residual = std::numeric_limits<double>::infinity();
  double primal_tolerance = 0.0;
  double dual_tolerance = 0.0;
  double first_control = 0.0;
  Prediction prediction{};
};

class Solver final
{
public:
  explicit Solver(const Configuration &configuration);
  ~Solver();

  Solver(const Solver &) = delete;
  Solver &operator=(const Solver &) = delete;

  bool configured() const noexcept;
  void reset() noexcept;
  Result solve(
    const State &initial_state, const Reference &reference,
    double last_control, Clock::time_point deadline) noexcept;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace mpc_controller::axis_mpc
