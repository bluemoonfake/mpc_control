#pragma once

#include <Eigen/Core>

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>

namespace mpc_controller::coupled_mpc
{

inline constexpr std::size_t kStateDimension = 9;
inline constexpr std::size_t kInputDimension = 3;
inline constexpr std::size_t kHorizonLength = 26;
inline constexpr std::size_t kPolygonSides = 8;

using Clock = std::chrono::steady_clock;
using State = Eigen::Matrix<double, kStateDimension, 1>;
using Input = Eigen::Matrix<double, kInputDimension, 1>;
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
  std::array<double, 3> model_time_constant_xyz{};
  std::array<double, 3> stage_weights_xy{};
  std::array<double, 3> terminal_weights_xy{};
  std::array<double, 3> stage_weights_z{};
  std::array<double, 3> terminal_weights_z{};
  std::array<double, 3> control_weights{};
  std::array<double, 3> control_rate_weights{};
  double max_speed_xy = 0.0;
  double max_speed_z = 0.0;
  double max_acceleration_xy = 0.0;
  double max_acceleration_z = 0.0;
  double max_control_xy = 0.0;
  double max_control_z = 0.0;
  double max_control_rate_xy = 0.0;
  double max_control_rate_z = 0.0;
  double gravity_m_s2 = 9.80665;
  double max_tilt_rad = 0.7853981633974483;   //45 degree
  double min_collective_specific_force_m_s2 = 1.0;
  double max_collective_specific_force_m_s2 = 16.0;
  double constraint_slack_weight = 1.0e4;
  double max_constraint_slack = 20.0;
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
  double max_velocity_slack = 0.0;
  double max_acceleration_slack = 0.0;
  double max_constraint_violation = std::numeric_limits<double>::infinity();
  double max_predicted_speed_xy = 0.0;
  double max_predicted_acceleration_xy = 0.0;
  double max_predicted_tilt_rad = 0.0;
  double max_predicted_collective_specific_force_m_s2 = 0.0;
  Input first_control = Input::Zero();
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
    const Input &last_control, Clock::time_point deadline) noexcept;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace mpc_controller::coupled_mpc
