#pragma once

#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace mpc_controller::tpmc {

inline constexpr std::size_t kStateDimension = 11; // p, v, roll, pitch, yaw,
                                                   // yaw rate, collective force
inline constexpr std::size_t kInputDimension =
    4; // inputs: roll pitch yaw collective
inline constexpr std::size_t kHorizonLength = 10; // Chan troi du doan 10 buoc
inline constexpr std::size_t kMaximumSqpStatisticsRows = 64;
inline constexpr double kDefaultSampleTimeSeconds = 0.05;
inline constexpr double kDefaultMaximumTiltRad = 0.7853981633974483;
inline constexpr double kDefaultMinimumCollectiveSpecificForceMps2 = 7.0;
inline constexpr double kDefaultMaximumCollectiveSpecificForceMps2 = 14.0;

enum StateIndex : std::size_t {
  position_x = 0,
  position_y,
  position_z,
  velocity_x,
  velocity_y,
  velocity_z,
  roll,
  pitch,
  yaw,
  yaw_rate,
  collective_specific_force
};

enum InputIndex : std::size_t {
  roll_command = 0,
  pitch_command,
  yaw_command,
  collective_specific_force_command
};

using Vector3 = std::array<double, 3>;
using State = std::array<double, kStateDimension>;
using Input = std::array<double, kInputDimension>;
using StateBounds = std::array<double, kStateDimension>;
using InputBounds = std::array<double, kInputDimension>;
using Clock = std::chrono::steady_clock;

struct ModelParameters {
  double roll_time_constant_seconds = 0.25;
  double pitch_time_constant_seconds = 0.25;
  double yaw_natural_frequency_rad_s = 3.42;
  double yaw_damping_ratio = 0.102;
  double collective_time_constant_seconds = 0.0932;
  double gravity_m_s2 = 9.80665;
};

struct Configuration {
  double sample_time_seconds = kDefaultSampleTimeSeconds;
  double solver_deadline_seconds = 0.018;
  ModelParameters model{};

  State stage_weights{};
  State terminal_weights{};
  Input input_weights{};
  double yaw_command_delta_weight = 30.0;

  StateBounds state_lower{};
  StateBounds state_upper{};
  InputBounds input_lower{};
  InputBounds input_upper{};

  double max_tilt_rad = kDefaultMaximumTiltRad;
  double max_tilt_rate_rad_s = 2.0;
  double max_yaw_command_rad = 1.0e6;
  double max_yaw_command_rate_rad_s = 2.0;
  double max_yaw_rate_rad_s = 2.0;
  double min_collective_specific_force_m_s2 =
      kDefaultMinimumCollectiveSpecificForceMps2;
  double max_collective_specific_force_m_s2 =
      kDefaultMaximumCollectiveSpecificForceMps2;
  double max_collective_rate_m_s3 = 25.0;
};

struct ReferencePoint {
  double time_from_start = 0.0;
  Vector3 position{};
  Vector3 velocity{};
  Vector3 acceleration{};
  double yaw = 0.0;
  double yaw_rate = 0.0;
};

struct ReferenceTrajectory {
  double header_time_seconds = 0.0;
  bool hold_after_end = false;
  std::vector<ReferencePoint> points;
};

struct TpmcReference {
  State state{};
  Input input{};
};

using ReferenceHorizon = std::array<TpmcReference, kHorizonLength + 1>;
using StatePrediction = std::array<State, kHorizonLength + 1>;
using InputPrediction = std::array<Input, kHorizonLength>;

struct SolveRequest {
  State initial_state{};
  ReferenceHorizon reference{};
  Input previous_input{};
  Clock::time_point deadline{};
};

enum class SolverStatus : std::uint8_t {
  success = 0,
  invalid_input,
  dependency_unavailable,
  not_initialized,
  infeasible,
  deadline_exceeded,
  numerical_failure,
  maximum_iterations,
  minimum_step
};

struct SqpIterationDiagnostics {
  int iteration = -1;
  std::array<double, 4> nlp_residuals{std::numeric_limits<double>::infinity(),
                                      std::numeric_limits<double>::infinity(),
                                      std::numeric_limits<double>::infinity(),
                                      std::numeric_limits<double>::infinity()};
  int qp_status = -1;
  int qp_iterations = -1;
  double step_length = std::numeric_limits<double>::quiet_NaN();
  std::array<double, 4> qp_residuals{std::numeric_limits<double>::infinity(),
                                     std::numeric_limits<double>::infinity(),
                                     std::numeric_limits<double>::infinity(),
                                     std::numeric_limits<double>::infinity()};
};

struct SolveResult {
  bool valid = false;
  SolverStatus status = SolverStatus::not_initialized;
  bool deadline_missed = false;
  int iterations = 0;
  // Acados-reported time spent inside the NLP solve. It excludes controller
  // input preparation and result extraction.
  double solve_time_seconds = 0.0;
  // Wall-clock timing splits for one call to Solver::solve(). The deadline is
  // evaluated after preparation plus the generated Acados solve, before the
  // diagnostic/result-extraction phase.
  double preparation_time_seconds = 0.0;
  double acados_wall_time_seconds = 0.0;
  double postprocessing_time_seconds = 0.0;
  // Timed components of postprocessing. The unattributed value is the small
  // remainder after subtracting these components from postprocessing time.
  double acados_metadata_time_seconds = 0.0;
  double diagnostics_time_seconds = 0.0;
  double sqp_statistics_time_seconds = 0.0;
  double prediction_read_time_seconds = 0.0;
  double constraint_validation_time_seconds = 0.0;
  double result_finalization_time_seconds = 0.0;
  double postprocessing_unattributed_time_seconds = 0.0;
  double end_to_end_time_seconds = 0.0;
  std::array<double, 4> kkt_residuals{std::numeric_limits<double>::infinity(),
                                      std::numeric_limits<double>::infinity(),
                                      std::numeric_limits<double>::infinity(),
                                      std::numeric_limits<double>::infinity()};
  double linearization_time_seconds = 0.0;
  double qp_time_seconds = 0.0;
  double regularization_time_seconds = 0.0;
  std::size_t sqp_statistics_count = 0;
  std::array<SqpIterationDiagnostics, kMaximumSqpStatisticsRows>sqp_statistics{};
  double max_constraint_violation = std::numeric_limits<double>::infinity();
  Input first_input{};
  StatePrediction predicted_states{};
  InputPrediction predicted_inputs{};
  std::string detail;
};

inline bool finite(const Vector3 &value) noexcept {
  for (const double item : value) {
    if (!std::isfinite(item)) {
      return false;
    }
  }
  return true;
}

template <typename Value, std::size_t Size>
inline bool finite(const std::array<Value, Size> &value) noexcept {
  for (const Value item : value) {
    if (!std::isfinite(item)) {
      return false;
    }
  }
  return true;
}

inline bool finite(const State &value) noexcept {
  for (const double item : value) {
    if (!std::isfinite(item)) {
      return false;
    }
  }
  return true;
}

inline bool finite(const Input &value) noexcept {
  for (const double item : value) {
    if (!std::isfinite(item)) {
      return false;
    }
  }
  return true;
}

inline bool withinBounds(const State &value, const StateBounds &lower,
                         const StateBounds &upper) noexcept {
  if (!finite(value)) {
    return false;
  }
  for (std::size_t index = 0; index < kStateDimension; ++index) {
    if (value[index] < lower[index] || value[index] > upper[index]) {
      return false;
    }
  }
  return true;
}

inline bool withinBounds(const Input &value, const InputBounds &lower,
                         const InputBounds &upper) noexcept {
  if (!finite(value)) {
    return false;
  }
  for (std::size_t index = 0; index < kInputDimension; ++index) {
    if (value[index] < lower[index] || value[index] > upper[index]) {
      return false;
    }
  }
  return true;
}

inline bool validConfiguration(const Configuration &configuration) noexcept {
  if (!std::isfinite(configuration.sample_time_seconds) ||
      configuration.sample_time_seconds <= 0.0 ||
      !std::isfinite(configuration.solver_deadline_seconds) ||
      configuration.solver_deadline_seconds <= 0.0 ||
      !std::isfinite(configuration.max_tilt_rad) ||
      configuration.max_tilt_rad <= 0.0 ||
      configuration.max_tilt_rad >= 0.5 * 3.14159265358979323846 ||
      !std::isfinite(configuration.max_yaw_command_rad) ||
      configuration.max_yaw_command_rad <= 0.0 ||
      !std::isfinite(configuration.max_yaw_command_rate_rad_s) ||
      configuration.max_yaw_command_rate_rad_s <= 0.0 ||
      !std::isfinite(configuration.max_yaw_rate_rad_s) ||
      configuration.max_yaw_rate_rad_s <= 0.0) {
    return false;
  }
  if (!finite(configuration.stage_weights) ||
      !finite(configuration.terminal_weights) ||
      !finite(configuration.input_weights) ||
      !std::isfinite(configuration.yaw_command_delta_weight) ||
      configuration.yaw_command_delta_weight < 0.0 ||
      !finite(configuration.state_lower) ||
      !finite(configuration.state_upper) ||
      !finite(configuration.input_lower) ||
      !finite(configuration.input_upper)) {
    return false;
  }
  for (std::size_t index = 0; index < kStateDimension; ++index) {
    if (configuration.state_lower[index] > configuration.state_upper[index]) {
      return false;
    }
  }
  for (std::size_t index = 0; index < kInputDimension; ++index) {
    if (configuration.input_lower[index] > configuration.input_upper[index]) {
      return false;
    }
  }
  return std::isfinite(configuration.model.roll_time_constant_seconds) &&
         configuration.model.roll_time_constant_seconds > 0.0 &&
         std::isfinite(configuration.model.pitch_time_constant_seconds) &&
         configuration.model.pitch_time_constant_seconds > 0.0 &&
         std::isfinite(configuration.model.yaw_natural_frequency_rad_s) &&
         configuration.model.yaw_natural_frequency_rad_s > 0.0 &&
         std::isfinite(configuration.model.yaw_damping_ratio) &&
         configuration.model.yaw_damping_ratio > 0.0 &&
         std::isfinite(configuration.model.collective_time_constant_seconds) &&
         configuration.model.collective_time_constant_seconds > 0.0 &&
         std::isfinite(configuration.model.gravity_m_s2) &&
         configuration.model.gravity_m_s2 > 0.0 &&
         std::isfinite(configuration.min_collective_specific_force_m_s2) &&
         std::isfinite(configuration.max_collective_specific_force_m_s2) &&
         configuration.min_collective_specific_force_m_s2 > 0.0 &&
         configuration.min_collective_specific_force_m_s2 <
             configuration.max_collective_specific_force_m_s2;
}

} // namespace mpc_controller::tpmc
