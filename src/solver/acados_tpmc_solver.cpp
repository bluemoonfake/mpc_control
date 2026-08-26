#include "mpc_controller/solver/acados_tpmc_solver.hpp"

#include "acados_c/ocp_nlp_interface.h"
#include "acados/utils/types.h"
#include "mpc_controller/solver/tpmc_constraints.hpp"
#include "mpc_controller/solver/tpmc_model.hpp"
#include "tpmc_generated_solver_bridge.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <utility>

namespace mpc_controller::tpmc {
namespace {

constexpr double kConstraintTolerance = 5.0e-3;
constexpr std::size_t kParameterDimension = 6;
constexpr std::size_t kGeneratedStateDimension =
    kStateDimension + kInputDimension;
constexpr std::size_t kStageNonlinearConstraintDimension = 5;
constexpr std::size_t kStateCostDimension = kStateDimension + 1;
constexpr std::size_t kInputCostDimension = kInputDimension + 3;
constexpr std::size_t kStageCostDimension =
    kStateCostDimension + kInputCostDimension;
// The state residual is
// [p, v, roll, pitch, yaw_rate, collective, sin(yaw), cos(yaw)].
constexpr std::size_t kYawRateStateCostIndex = yaw;
constexpr std::size_t kCollectiveStateCostIndex = kYawRateStateCostIndex + 1;
constexpr std::size_t kYawStateSinCostIndex = kCollectiveStateCostIndex + 1;
constexpr std::size_t kYawStateCosCostIndex = kYawStateSinCostIndex + 1;
static_assert(kYawStateCosCostIndex + 1 == kStateCostDimension);
// The input residual is [roll, pitch, collective, sin/cos(yaw target),
// sin/cos(yaw delta)], which differs from Input.
constexpr std::size_t kRollCommandCostIndex = kStateCostDimension;
constexpr std::size_t kPitchCommandCostIndex = kRollCommandCostIndex + 1;
constexpr std::size_t kCollectiveCommandCostIndex = kPitchCommandCostIndex + 1;
constexpr std::size_t kYawCommandSinCostIndex = kCollectiveCommandCostIndex + 1;
constexpr std::size_t kYawCommandCosCostIndex = kYawCommandSinCostIndex + 1;
constexpr std::size_t kYawCommandDeltaSinCostIndex =
    kYawCommandCosCostIndex + 1;
constexpr std::size_t kYawCommandDeltaCosCostIndex =
    kYawCommandDeltaSinCostIndex + 1;
static_assert(kYawCommandDeltaCosCostIndex + 1 == kStageCostDimension);
constexpr std::size_t kMaximumSqpStatisticsColumns = 16;
constexpr std::size_t kSqpResidualColumnOffset = 1;
constexpr std::size_t kQpStatusColumn = 5;
constexpr std::size_t kQpIterationsColumn = 6;
constexpr std::size_t kStepLengthColumn = 7;
constexpr std::size_t kQpResidualColumnOffset = 8;
constexpr int kRtiBaseStatisticsDataColumns = 2;
constexpr std::size_t kRtiQpStatusColumn = 1;
constexpr std::size_t kRtiQpIterationsColumn = 2;
constexpr std::size_t kRtiQpResidualColumnOffset = 3;

using StageCostMatrix =
    std::array<double, kStageCostDimension * kStageCostDimension>;
using TerminalCostMatrix =
    std::array<double, kStateCostDimension * kStateCostDimension>;
using GeneratedState = std::array<double, kGeneratedStateDimension>;
using StageNonlinearBounds =
    std::array<double, kStageNonlinearConstraintDimension>;

GeneratedState generatedState(const State &physical_state,
                              const Input &previous_input) noexcept {
  GeneratedState output{};
  std::copy(physical_state.begin(), physical_state.end(), output.begin());
  std::copy(previous_input.begin(), previous_input.end(),
            output.begin() + static_cast<std::ptrdiff_t>(kStateDimension));
  return output;
}

const char *statusDetail(int status) noexcept {
  switch (status) {
  case 0:
    return "acados_success";
  case 1:
    return "acados_nan_detected";
  case 2:
    return "acados_maximum_iterations";
  case 3:
    return "acados_minimum_step";
  case 4:
    return "acados_qp_failure";
  default:
    return "acados_unknown_status";
  }
}

SolverStatus mapStatus(int status) noexcept {
  switch (status) {
  case 0:
    return SolverStatus::success;
  case 2:
    return SolverStatus::maximum_iterations;
  case 3:
    return SolverStatus::minimum_step;
  case 4:
    return SolverStatus::infeasible;
  case 1:
  default:
    return SolverStatus::numerical_failure;
  }
}

std::string describeReferenceViolation(const ReferenceHorizon &reference,
                                       const Configuration &configuration) {
  for (std::size_t stage = 0; stage < reference.size(); ++stage) {
    const std::string state_violation =
        describeStateViolation(reference[stage].state, configuration);
    if (!state_violation.empty()) {
      return "reference[" + std::to_string(stage) + "]: " + state_violation;
    }
    const std::string input_violation =
        describeInputViolation(reference[stage].input, configuration);
    if (!input_violation.empty()) {
      return "reference[" + std::to_string(stage) + "]: " + input_violation;
    }
  }
  return {};
}

std::string describePredictionViolation(const StatePrediction &states,
                                        const InputPrediction &inputs,
                                        const Input &previous_input,
                                        const Configuration &configuration) {
  Input predecessor = previous_input;
  for (std::size_t stage = 0; stage <= kHorizonLength; ++stage) {
    if (stage > 0) {
      const std::string state_violation =
          describeStateViolation(states[stage], configuration);
      if (!state_violation.empty()) {
        return "prediction[" + std::to_string(stage) + "]: " + state_violation;
      }
    }
    if (stage == kHorizonLength) {
      continue;
    }
    const std::string transition_violation = describeInputTransitionViolation(
        predecessor, inputs[stage], configuration);
    if (!transition_violation.empty()) {
      return "prediction[" + std::to_string(stage) +
             "]: " + transition_violation;
    }
    predecessor = inputs[stage];
  }
  return "prediction constraint violation exceeds tolerance";
}

template <std::size_t Size>
void setDiagonal(std::array<double, Size * Size> &matrix,
                 const std::array<double, Size> &diagonal) noexcept {
  matrix.fill(0.0);
  for (std::size_t index = 0; index < Size; ++index) {
    matrix[index * Size + index] = diagonal[index];
  }
}

template <typename RuntimeType>
bool setCostMatrix(RuntimeType &runtime,
                   const Configuration &configuration) noexcept {
  StageCostMatrix stage_matrix{};
  TerminalCostMatrix terminal_matrix{};
  setDiagonal(stage_matrix, [&configuration] {
    std::array<double, kStageCostDimension> diagonal{};
    std::copy_n(configuration.stage_weights.begin(), yaw, diagonal.begin());
    diagonal[kYawRateStateCostIndex] = configuration.stage_weights[yaw_rate];
    diagonal[kCollectiveStateCostIndex] =
        configuration.stage_weights[collective_specific_force];
    diagonal[kYawStateSinCostIndex] = configuration.stage_weights[yaw];
    diagonal[kYawStateCosCostIndex] = configuration.stage_weights[yaw];
    diagonal[kRollCommandCostIndex] = configuration.input_weights[roll_command];
    diagonal[kPitchCommandCostIndex] = configuration.input_weights[pitch_command];
    diagonal[kCollectiveCommandCostIndex] = configuration.input_weights[collective_specific_force_command];
    diagonal[kYawCommandSinCostIndex] = configuration.input_weights[yaw_command];
    diagonal[kYawCommandCosCostIndex] = configuration.input_weights[yaw_command];
    diagonal[kYawCommandDeltaSinCostIndex] =
        configuration.yaw_command_delta_weight;
    diagonal[kYawCommandDeltaCosCostIndex] =
        configuration.yaw_command_delta_weight;
    return diagonal;
  }());
  setDiagonal(terminal_matrix, [&configuration] {
    std::array<double, kStateCostDimension> diagonal{};
    std::copy_n(configuration.terminal_weights.begin(), yaw, diagonal.begin());
    diagonal[kYawRateStateCostIndex] = configuration.terminal_weights[yaw_rate];
    diagonal[kCollectiveStateCostIndex] =
        configuration.terminal_weights[collective_specific_force];
    diagonal[kYawStateSinCostIndex] = configuration.terminal_weights[yaw];
    diagonal[kYawStateCosCostIndex] = configuration.terminal_weights[yaw];
    return diagonal;
  }());

  for (std::size_t stage = 0; stage < kHorizonLength; ++stage) {
    if (ocp_nlp_cost_model_set(runtime.config, runtime.dims, runtime.input,
                               static_cast<int>(stage), "W",
                               stage_matrix.data()) != 0) {
      return false;
    }
  }
  return ocp_nlp_cost_model_set(runtime.config, runtime.dims, runtime.input,
                                static_cast<int>(kHorizonLength), "W",
                                terminal_matrix.data()) == 0;
}

template <typename RuntimeType>
bool setParameters(RuntimeType &runtime,
                   const Configuration &configuration) noexcept {
  std::array<double, kParameterDimension> parameters{};
  parameters = {configuration.model.roll_time_constant_seconds,
                configuration.model.pitch_time_constant_seconds,
                configuration.model.yaw_natural_frequency_rad_s,
                configuration.model.yaw_damping_ratio,
                configuration.model.collective_time_constant_seconds,
                configuration.model.gravity_m_s2};
  for (std::size_t stage = 0; stage <= kHorizonLength; ++stage) {
    if (generated::updateParameters(runtime.capsule, static_cast<int>(stage),
                                    parameters.data(),
                                    static_cast<int>(parameters.size())) != 0) {
      return false;
    }
  }
  return true;
}

template <typename RuntimeType>
bool setBounds(RuntimeType &runtime, const Configuration &configuration,
               const State &initial_state,
               const Input &previous_input) noexcept {
  double tilt_lower_bound = std::cos(configuration.max_tilt_rad);
  const double maximum_tilt_change = configuration.max_tilt_rate_rad_s *
                                    configuration.sample_time_seconds;
  const double maximum_yaw_change = configuration.max_yaw_command_rate_rad_s *
                                    configuration.sample_time_seconds;
  const double maximum_collective_change =
      configuration.max_collective_rate_m_s3 * configuration.sample_time_seconds;
  StageNonlinearBounds nonlinear_upper{
      ACADOS_INFTY, maximum_tilt_change, maximum_tilt_change,
      maximum_yaw_change, maximum_collective_change};
  StageNonlinearBounds nonlinear_lower{
      tilt_lower_bound, -maximum_tilt_change, -maximum_tilt_change,
      -maximum_yaw_change, -maximum_collective_change};

  for (std::size_t stage = 0; stage <= kHorizonLength; ++stage) {
    GeneratedState state_lower = generatedState(configuration.state_lower,
                                                 configuration.input_lower);
    GeneratedState state_upper = generatedState(configuration.state_upper,
                                                 configuration.input_upper);
    if (stage == 0) {
      state_lower = generatedState(initial_state, previous_input);
      state_upper = state_lower;
    }
    StageNonlinearBounds stage_nonlinear_lower = nonlinear_lower;
    if (stage == 0) {
      stage_nonlinear_lower[0] = -ACADOS_INFTY;
    }
    if (ocp_nlp_constraints_model_set(
            runtime.config, runtime.dims, runtime.input, runtime.output,
            static_cast<int>(stage), "lbx", state_lower.data()) != 0 ||
        ocp_nlp_constraints_model_set(
            runtime.config, runtime.dims, runtime.input, runtime.output,
            static_cast<int>(stage), "ubx", state_upper.data()) != 0 ||
        (stage < kHorizonLength &&
         (ocp_nlp_constraints_model_set(
              runtime.config, runtime.dims, runtime.input, runtime.output,
              static_cast<int>(stage), "lh", stage_nonlinear_lower.data()) != 0 ||
          ocp_nlp_constraints_model_set(
              runtime.config, runtime.dims, runtime.input, runtime.output,
              static_cast<int>(stage), "uh", nonlinear_upper.data()) != 0)) ||
        (stage == kHorizonLength &&
         (ocp_nlp_constraints_model_set(
              runtime.config, runtime.dims, runtime.input, runtime.output,
              static_cast<int>(stage), "lh", &tilt_lower_bound) != 0 ||
          ocp_nlp_constraints_model_set(
              runtime.config, runtime.dims, runtime.input, runtime.output,
              static_cast<int>(stage), "uh", &nonlinear_upper[0]) != 0))) {
      return false;
    }
  }

  for (std::size_t stage = 0; stage < kHorizonLength; ++stage) {
    Input input_lower = configuration.input_lower;
    Input input_upper = configuration.input_upper;
    if (stage == 0) {
      input_lower[roll_command] = std::max(
          input_lower[roll_command], previous_input[roll_command] - maximum_tilt_change);
      input_upper[roll_command] = std::min(
          input_upper[roll_command], previous_input[roll_command] + maximum_tilt_change);
      input_lower[pitch_command] = std::max(
          input_lower[pitch_command], previous_input[pitch_command] - maximum_tilt_change);
      input_upper[pitch_command] = std::min(
          input_upper[pitch_command], previous_input[pitch_command] + maximum_tilt_change);
      input_lower[yaw_command] = std::max(
          input_lower[yaw_command], previous_input[yaw_command] - maximum_yaw_change);
      input_upper[yaw_command] = std::min(
          input_upper[yaw_command], previous_input[yaw_command] + maximum_yaw_change);
      input_lower[collective_specific_force_command] = std::max(
          input_lower[collective_specific_force_command],
          previous_input[collective_specific_force_command] - maximum_collective_change);
      input_upper[collective_specific_force_command] = std::min(
          input_upper[collective_specific_force_command],
          previous_input[collective_specific_force_command] + maximum_collective_change);
    }
    for (std::size_t index = 0; index < kInputDimension; ++index) {
      if (input_lower[index] > input_upper[index]) {
        return false;
      }
    }
    if (ocp_nlp_constraints_model_set(
            runtime.config, runtime.dims, runtime.input, runtime.output,
            static_cast<int>(stage), "lbu", input_lower.data()) != 0 ||
        ocp_nlp_constraints_model_set(
            runtime.config, runtime.dims, runtime.input, runtime.output,
            static_cast<int>(stage), "ubu", input_upper.data()) != 0) {
      return false;
    }
  }
  return true;
}

template <typename RuntimeType>
void initializeTrajectory(RuntimeType &runtime, const SolveRequest &request,
                          const Configuration &configuration,
                          const StatePrediction &warm_states,
                          const InputPrediction &warm_inputs,
                          bool has_warm_start) noexcept {
  State cold_start_state = request.initial_state;
  Input cold_start_previous_input = request.previous_input;
  for (std::size_t stage = 0; stage <= kHorizonLength; ++stage) {
    const Input state_previous_input = has_warm_start
        ? (stage == 0 ? request.previous_input : warm_inputs[stage - 1])
        : cold_start_previous_input;
    GeneratedState state = generatedState(
        has_warm_start ? warm_states[stage] : cold_start_state,
        state_previous_input);
    ocp_nlp_out_set(runtime.config, runtime.dims, runtime.output, runtime.input,
                    static_cast<int>(stage), "x", state.data());
    if (stage < kHorizonLength) {
      Input input = has_warm_start ? warm_inputs[stage]
                                   : cold_start_previous_input;
      ocp_nlp_out_set(runtime.config, runtime.dims, runtime.output,
                      runtime.input, static_cast<int>(stage), "u",
                      input.data());
      if (!has_warm_start) {
        cold_start_state = integrateErk4(
            cold_start_state, input, configuration.sample_time_seconds,
            configuration.model);
        cold_start_previous_input = input;
      }
    }
  }
}

template <typename RuntimeType>
bool setReferences(RuntimeType &runtime, const ReferenceHorizon &reference,
                   const Input &previous_input) noexcept {
  std::array<double, kStageCostDimension> stage_reference{};
  std::array<double, kStateCostDimension> terminal_reference{};
  for (std::size_t stage = 0; stage < kHorizonLength; ++stage) {
    const State &state = reference[stage].state;
    const Input &input = reference[stage].input;
    std::copy_n(state.begin(), yaw, stage_reference.begin());
    stage_reference[kYawRateStateCostIndex] = state[yaw_rate];
    stage_reference[kCollectiveStateCostIndex] =
        state[collective_specific_force];
    stage_reference[kYawStateSinCostIndex] = std::sin(state[yaw]);
    stage_reference[kYawStateCosCostIndex] = std::cos(state[yaw]);
    stage_reference[kRollCommandCostIndex] = input[roll_command];
    stage_reference[kPitchCommandCostIndex] = input[pitch_command];
    stage_reference[kCollectiveCommandCostIndex] =
        input[collective_specific_force_command];
    stage_reference[kYawCommandSinCostIndex] =
        std::sin(input[yaw_command]);
    stage_reference[kYawCommandCosCostIndex] =
        std::cos(input[yaw_command]);
    // The generated state stores the predecessor command at every stage, so
    // this residual is the actual sequential yaw-command change.
    stage_reference[kYawCommandDeltaSinCostIndex] = 0.0;
    stage_reference[kYawCommandDeltaCosCostIndex] = 1.0;
    if (ocp_nlp_cost_model_set(runtime.config, runtime.dims, runtime.input,
                               static_cast<int>(stage), "yref",
                               stage_reference.data()) != 0) {
      return false;
    }
  }
  const State &terminal_state = reference[kHorizonLength].state;
  std::copy_n(terminal_state.begin(), yaw, terminal_reference.begin());
  terminal_reference[kYawRateStateCostIndex] = terminal_state[yaw_rate];
  terminal_reference[kCollectiveStateCostIndex] =
      terminal_state[collective_specific_force];
  terminal_reference[kYawStateSinCostIndex] = std::sin(terminal_state[yaw]);
  terminal_reference[kYawStateCosCostIndex] = std::cos(terminal_state[yaw]);
  return ocp_nlp_cost_model_set(runtime.config, runtime.dims, runtime.input,
                                static_cast<int>(kHorizonLength), "yref",
                                terminal_reference.data()) == 0;
}

template <typename RuntimeType>
bool readPrediction(RuntimeType &runtime, SolveResult &result) noexcept {
  for (std::size_t stage = 0; stage <= kHorizonLength; ++stage) {
    GeneratedState generated_state{};
    ocp_nlp_out_get(runtime.config, runtime.dims, runtime.output,
                    static_cast<int>(stage), "x", generated_state.data());
    std::copy_n(generated_state.begin(), kStateDimension,
                result.predicted_states[stage].begin());
    if (!finite(result.predicted_states[stage])) {
      return false;
    }
    if (stage > 0) {
      for (std::size_t index = 0; index < kInputDimension; ++index) {
        const double memory_error = std::abs(
            generated_state[kStateDimension + index] -
            result.predicted_inputs[stage - 1][index]);
        if (!std::isfinite(memory_error) || memory_error > 1.0e-6) {
          result.detail = "generated command memory is inconsistent with the preceding input";
          return false;
        }
      }
    }
    if (stage < kHorizonLength) {
      ocp_nlp_out_get(runtime.config, runtime.dims, runtime.output,
                      static_cast<int>(stage), "u",
                      result.predicted_inputs[stage].data());
      if (!finite(result.predicted_inputs[stage])) {
        return false;
      }
    }
  }
  result.first_input = result.predicted_inputs[0];
  return true;
}

template <typename RuntimeType>
void readDiagnostics(RuntimeType &runtime, SolveResult &result) noexcept {
  if constexpr (!generated::kUsesSqpRti || generated::kRtiLogsResiduals) {
    ocp_nlp_get(runtime.solver, "res_stat", &result.kkt_residuals[0]);
    ocp_nlp_get(runtime.solver, "res_eq", &result.kkt_residuals[1]);
    ocp_nlp_get(runtime.solver, "res_ineq", &result.kkt_residuals[2]);
    ocp_nlp_get(runtime.solver, "res_comp", &result.kkt_residuals[3]);
  } else {
    // Computing nonlinear KKT residuals in standard RTI requires an extra
    // nonlinear-function evaluation. Keep the 50 Hz solve path deterministic
    // and expose these values as unavailable instead of stale diagnostics.
    result.kkt_residuals.fill(std::numeric_limits<double>::quiet_NaN());
  }
  ocp_nlp_get(runtime.solver, "time_lin", &result.linearization_time_seconds);
  ocp_nlp_get(runtime.solver, "time_qp", &result.qp_time_seconds);
  ocp_nlp_get(runtime.solver, "time_reg", &result.regularization_time_seconds);
}

int integerStatistic(double value) noexcept {
  if (!std::isfinite(value) ||
      value < static_cast<double>(std::numeric_limits<int>::min()) ||
      value > static_cast<double>(std::numeric_limits<int>::max())) {
    return -1;
  }
  return static_cast<int>(value);
}

template <typename RuntimeType>
void readSqpStatistics(RuntimeType &runtime, SolveResult &result) noexcept {
  int statistic_columns = 0;
  int statistic_rows = 0;
  ocp_nlp_get(runtime.solver, "stat_n", &statistic_columns);
  ocp_nlp_get(runtime.solver, "stat_m", &statistic_rows);
  if (statistic_columns <= 0 || statistic_rows <= 0 ||
      statistic_columns + 1 > static_cast<int>(kMaximumSqpStatisticsColumns) ||
      statistic_rows > static_cast<int>(kMaximumSqpStatisticsRows)) {
    return;
  }

  const std::size_t row_count =
      std::min({static_cast<std::size_t>(statistic_rows),
                static_cast<std::size_t>(std::max(result.iterations, 0) + 1),
                result.sqp_statistics.size()});
  if (row_count == 0) {
    return;
  }

  std::array<double, kMaximumSqpStatisticsColumns * kMaximumSqpStatisticsRows>
      statistics{};
  ocp_nlp_get(runtime.solver, "statistics", statistics.data());
  const auto value = [&statistics, row_count](std::size_t row,
                                              std::size_t column) {
    return statistics[row + column * row_count];
  };

  if constexpr (generated::kUsesSqpRti) {
    const int expected_columns =
        kRtiBaseStatisticsDataColumns +
        (generated::kHasExternalQpResiduals ? 4 : 0) +
        (generated::kRtiLogsResiduals ? 4 : 0);
    if (statistic_columns != expected_columns) {
      return;
    }

    for (std::size_t row = 0; row < row_count; ++row) {
      SqpIterationDiagnostics &diagnostic = result.sqp_statistics[row];
      diagnostic.iteration = integerStatistic(value(row, 0));
      diagnostic.qp_status =
          integerStatistic(value(row, kRtiQpStatusColumn));
      diagnostic.qp_iterations =
          integerStatistic(value(row, kRtiQpIterationsColumn));
      if constexpr (generated::kHasExternalQpResiduals) {
        for (std::size_t residual = 0;
             residual < diagnostic.qp_residuals.size(); ++residual) {
          diagnostic.qp_residuals[residual] =
              value(row, kRtiQpResidualColumnOffset + residual);
        }
      }
      if constexpr (generated::kRtiLogsResiduals) {
        const std::size_t rti_residual_offset =
            kRtiQpResidualColumnOffset +
            (generated::kHasExternalQpResiduals ? 4 : 0);
        for (std::size_t residual = 0;
             residual < diagnostic.nlp_residuals.size(); ++residual) {
          diagnostic.nlp_residuals[residual] =
              value(row, rti_residual_offset + residual);
        }
      }
    }
    result.sqp_statistics_count = row_count;
    return;
  }

  if (statistic_columns < 7) {
    return;
  }

  for (std::size_t row = 0; row < row_count; ++row) {
    SqpIterationDiagnostics &diagnostic = result.sqp_statistics[row];
    diagnostic.iteration = integerStatistic(value(row, 0));
    for (std::size_t residual = 0; residual < diagnostic.nlp_residuals.size();
         ++residual) {
      diagnostic.nlp_residuals[residual] =
          value(row, kSqpResidualColumnOffset + residual);
    }
    diagnostic.qp_status = integerStatistic(value(row, kQpStatusColumn));
    diagnostic.qp_iterations =
        integerStatistic(value(row, kQpIterationsColumn));
    diagnostic.step_length = value(row, kStepLengthColumn);
    if (statistic_columns >= 11) {
      for (std::size_t residual = 0; residual < diagnostic.qp_residuals.size();
           ++residual) {
        diagnostic.qp_residuals[residual] =
            value(row, kQpResidualColumnOffset + residual);
      }
    }
  }
  result.sqp_statistics_count = row_count;
}

std::string diagnosticsDetail(const SolveResult &result) {
  const std::string kkt = finite(result.kkt_residuals)
                              ? " kkt=[stat=" +
                                    std::to_string(result.kkt_residuals[0]) +
                                    ",eq=" +
                                    std::to_string(result.kkt_residuals[1]) +
                                    ",ineq=" +
                                    std::to_string(result.kkt_residuals[2]) +
                                    ",comp=" +
                                    std::to_string(result.kkt_residuals[3]) +
                                    "]"
                              : " kkt=not_evaluated";
  return kkt + " time_ms=[lin=" +
         std::to_string(result.linearization_time_seconds * 1.0e3) +
         ",qp=" + std::to_string(result.qp_time_seconds * 1.0e3) +
         ",reg=" + std::to_string(result.regularization_time_seconds * 1.0e3) +
         "]";
}

} // namespace

struct AcadosTpmcSolver::Runtime {
  generated::SolverCapsule *capsule = nullptr;
  ocp_nlp_config *config = nullptr;
  ocp_nlp_dims *dims = nullptr;
  ocp_nlp_in *input = nullptr;
  ocp_nlp_out *output = nullptr;
  ocp_nlp_solver *solver = nullptr;
  bool created = false;

  ~Runtime() {
    if (capsule == nullptr) {
      return;
    }
    if (created) {
      generated::freeSolver(capsule);
    }
    generated::freeCapsule(capsule);
  }
};

AcadosTpmcSolver::AcadosTpmcSolver(const Configuration &configuration)
    : configuration_(configuration) {
  if (!validConfiguration(configuration_)) {
    dependency_status_ = "invalid TMPC configuration";
    return;
  }

  runtime_ = std::make_unique<Runtime>();
  runtime_->capsule = generated::createCapsule();
  if (runtime_->capsule == nullptr ||
      generated::create(runtime_->capsule) != 0) {
    dependency_status_ = "acados generated solver failed to initialize";
    return;
  }
  runtime_->created = true;
  runtime_->config = generated::config(runtime_->capsule);
  runtime_->dims = generated::dimensions(runtime_->capsule);
  runtime_->input = generated::input(runtime_->capsule);
  runtime_->output = generated::output(runtime_->capsule);
  runtime_->solver = generated::solver(runtime_->capsule);
  configured_ = runtime_->config != nullptr && runtime_->dims != nullptr &&
                runtime_->input != nullptr && runtime_->output != nullptr &&
                runtime_->solver != nullptr &&
                setCostMatrix(*runtime_, configuration_) &&
                setParameters(*runtime_, configuration_);
  if (!configured_) {
    dependency_status_ = "acados generated solver has invalid runtime objects";
  } else {
    dependency_status_ = "acados generated solver ready";
  }
}

AcadosTpmcSolver::~AcadosTpmcSolver() = default;

bool AcadosTpmcSolver::configured() const noexcept { return configured_; }

const char *AcadosTpmcSolver::backendName() const noexcept {
  return generated::kNlpSolverType;
}

void AcadosTpmcSolver::reset() noexcept {
  has_warm_start_ = false;
  if (configured_ && runtime_ != nullptr) {
    generated::resetSolverState(runtime_->capsule);
  }
}

SolveResult AcadosTpmcSolver::solve(const SolveRequest &request) noexcept {
  const auto start = Clock::now();
  SolveResult result;
  const auto finishBeforeAcados = [&result, start] {
    const double elapsed_seconds =
        std::chrono::duration<double>(Clock::now() - start).count();
    result.preparation_time_seconds = elapsed_seconds;
    result.end_to_end_time_seconds = elapsed_seconds;
  };
  result.status = SolverStatus::not_initialized;
  result.detail = dependency_status_;
  if (!configured_ || runtime_ == nullptr) {
    finishBeforeAcados();
    return result;
  }
  const std::string initial_state_violation =
      describeMeasuredStateViolation(request.initial_state, configuration_);
  const std::string previous_input_violation =
      describeInputViolation(request.previous_input, configuration_);
  const std::string reference_violation =
      describeReferenceViolation(request.reference, configuration_);
  if (!initial_state_violation.empty() || !previous_input_violation.empty() ||
      !reference_violation.empty()) {
    result.status = SolverStatus::invalid_input;
    if (!initial_state_violation.empty()) {
      result.detail = "initial_state: " + initial_state_violation;
    } else if (!previous_input_violation.empty()) {
      result.detail = "previous_input: " + previous_input_violation;
    } else {
      result.detail = reference_violation;
    }
    finishBeforeAcados();
    return result;
  }

  const auto resetAfterFailure = [this] {
    has_warm_start_ = false;
    if (runtime_ != nullptr) {
      generated::resetSolverState(runtime_->capsule);
    }
  };

  if (request.deadline != Clock::time_point{} &&
      Clock::now() > request.deadline) {
    result.status = SolverStatus::deadline_exceeded;
    result.deadline_missed = true;
    result.detail = "TMPC deadline expired before acados solve";
    finishBeforeAcados();
    return result;
  }

  if (!setParameters(*runtime_, configuration_) ||
      !setBounds(*runtime_, configuration_, request.initial_state,
                 request.previous_input) ||
      !setReferences(*runtime_, request.reference, request.previous_input)) {
    resetAfterFailure();
    result.status = SolverStatus::numerical_failure;
    result.detail = "Failed to update acados runtime data";
    finishBeforeAcados();
    return result;
  }

  initializeTrajectory(*runtime_, request, configuration_, warm_states_,
                       warm_inputs_, has_warm_start_);
  const auto acados_started_at = Clock::now();
  result.preparation_time_seconds =
      std::chrono::duration<double>(acados_started_at - start).count();
  const int acados_status = generated::solve(runtime_->capsule);
  const auto acados_finished_at = Clock::now();
  result.acados_wall_time_seconds =
      std::chrono::duration<double>(acados_finished_at - acados_started_at)
          .count();
  const auto finishPostprocessing = [&result, start, acados_finished_at] {
    const auto finished_at = Clock::now();
    result.postprocessing_time_seconds =
        std::chrono::duration<double>(finished_at - acados_finished_at)
            .count();
    const double attributed_postprocessing_seconds =
        result.acados_metadata_time_seconds + result.diagnostics_time_seconds +
        result.sqp_statistics_time_seconds + result.prediction_read_time_seconds +
        result.constraint_validation_time_seconds +
        result.result_finalization_time_seconds;
    result.postprocessing_unattributed_time_seconds = std::max(
        0.0, result.postprocessing_time_seconds -
                 attributed_postprocessing_seconds);
    result.end_to_end_time_seconds =
        std::chrono::duration<double>(finished_at - start).count();
  };

  const auto metadata_started_at = Clock::now();
  double acados_time_seconds = 0.0;
  int iterations = 0;
  ocp_nlp_get(runtime_->solver, "time_tot", &acados_time_seconds);
  ocp_nlp_get(runtime_->solver, "sqp_iter", &iterations);

  result.status = mapStatus(acados_status);
  result.iterations = iterations;
  result.detail = statusDetail(acados_status);
  result.solve_time_seconds = result.acados_wall_time_seconds;
  if (std::isfinite(acados_time_seconds) && acados_time_seconds > 0.0) {
    result.solve_time_seconds = acados_time_seconds;
  }
  const bool deadline_missed_during_solve =
      request.deadline != Clock::time_point{} &&
      acados_finished_at > request.deadline;
  result.deadline_missed = deadline_missed_during_solve;
  result.acados_metadata_time_seconds =
      std::chrono::duration<double>(Clock::now() - metadata_started_at).count();

  const auto diagnostics_started_at = Clock::now();
  readDiagnostics(*runtime_, result);
  result.diagnostics_time_seconds =
      std::chrono::duration<double>(Clock::now() - diagnostics_started_at)
          .count();

  const auto sqp_statistics_started_at = Clock::now();
  readSqpStatistics(*runtime_, result);
  result.sqp_statistics_time_seconds =
      std::chrono::duration<double>(Clock::now() - sqp_statistics_started_at)
          .count();

  const auto prediction_read_started_at = Clock::now();
  const bool prediction_is_finite = readPrediction(*runtime_, result);
  result.prediction_read_time_seconds =
      std::chrono::duration<double>(Clock::now() - prediction_read_started_at)
          .count();
  if (!prediction_is_finite) {
    const auto finalization_started_at = Clock::now();
    resetAfterFailure();
    result.status = SolverStatus::numerical_failure;
    result.detail = "acados returned a non-finite prediction";
    result.valid = false;
    result.result_finalization_time_seconds =
        std::chrono::duration<double>(Clock::now() - finalization_started_at)
            .count();
    finishPostprocessing();
    return result;
  }

  const auto constraint_validation_started_at = Clock::now();
  result.max_constraint_violation = 0.0;
  Input predecessor = request.previous_input;
  for (std::size_t stage = 0; stage <= kHorizonLength; ++stage) {
    const Input &input = stage < kHorizonLength ? result.predicted_inputs[stage]
                                                : result.first_input;
    const double stage_violation =
        stage == 0
            ? inputConstraintViolation(input, configuration_)
            : constraintViolation(result.predicted_states[stage], input,
                                  configuration_);
    result.max_constraint_violation =
        std::max(result.max_constraint_violation, stage_violation);
    if (stage < kHorizonLength) {
      result.max_constraint_violation =
          std::max(result.max_constraint_violation,
                   inputTransitionConstraintViolation(
                       predecessor, result.predicted_inputs[stage],
                       configuration_));
      predecessor = result.predicted_inputs[stage];
    }
  }

  if (result.status == SolverStatus::success && result.deadline_missed) {
    result.status = SolverStatus::deadline_exceeded;
    result.detail = "TMPC deadline exceeded during the Acados solve path";
  }
  result.valid = result.status == SolverStatus::success &&
                 result.max_constraint_violation <= kConstraintTolerance;

  result.constraint_validation_time_seconds =
      std::chrono::duration<double>(Clock::now() -
                                    constraint_validation_started_at)
          .count();

  const auto finalization_started_at = Clock::now();
  if (!result.valid) {
    if (result.status != SolverStatus::success) {
      result.detail += "; solver_output_rejected: status_not_success;" +
                       diagnosticsDetail(result);
    } else if (result.max_constraint_violation > kConstraintTolerance) {
      result.detail += "; solver_output_rejected: " +
                       describePredictionViolation(
                           result.predicted_states, result.predicted_inputs,
                           request.previous_input, configuration_) +
                       ";" + diagnosticsDetail(result);
    }
  }
  if (result.valid) {
    warm_states_ = result.predicted_states;
    warm_inputs_ = result.predicted_inputs;
    has_warm_start_ = true;
  } else {
    // The vehicle follows recovery after every rejected iterate. Keeping that
    // iterate would make the next SQP initialization inconsistent with the
    // measured recovery state and applied input.
    resetAfterFailure();
  }
  result.result_finalization_time_seconds =
      std::chrono::duration<double>(Clock::now() - finalization_started_at)
          .count();
  finishPostprocessing();
  return result;
}

const Configuration &AcadosTpmcSolver::configuration() const noexcept {
  return configuration_;
}

const std::string &AcadosTpmcSolver::dependencyStatus() const noexcept {
  return dependency_status_;
}

} // namespace mpc_controller::tpmc
