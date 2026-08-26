#include "mpc_controller/solver/acados_tpmc_solver.hpp"
#include "mpc_controller/controller/command_safety_limiter.hpp"
#include "mpc_controller/controller/collective_force_filter.hpp"
#include "mpc_controller/controller/geometric_controller.hpp"
#include "mpc_controller/mission/minimum_time_trajectory.hpp"
#include "mpc_controller/bridge/state_bridge.hpp"
#include "mpc_controller/solver/tpmc_constraints.hpp"
#include "mpc_controller/solver/tpmc_model.hpp"
#include "mpc_controller/solver/tpmc_reference.hpp"

#include <array>
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

namespace {

using namespace mpc_controller::tpmc;
constexpr double kPi = 3.14159265358979323846;

void require(bool condition, const char *message) {
  if (!condition) {
    std::cerr << "TMPC test failure: " << message << '\n';
    std::abort();
  }
}

bool near(double left, double right, double tolerance) {
  return std::abs(left - right) <= tolerance;
}

std::array<double, 8>
postprocessingTimings(const SolveResult &result) noexcept {
  return {result.postprocessing_time_seconds,
          result.acados_metadata_time_seconds,
          result.diagnostics_time_seconds,
          result.sqp_statistics_time_seconds,
          result.prediction_read_time_seconds,
          result.constraint_validation_time_seconds,
          result.result_finalization_time_seconds,
          result.postprocessing_unattributed_time_seconds};
}

Configuration makeConfiguration(const ModelParameters &model) {
  Configuration configuration;
  configuration.model = model;
  configuration.state_lower.fill(-1.0e6);
  configuration.state_upper.fill(1.0e6);
  configuration.input_lower.fill(-1.0e6);
  configuration.input_upper.fill(1.0e6);
  configuration.stage_weights.fill(1.0);
  configuration.terminal_weights.fill(1.0);
  configuration.input_weights.fill(0.1);
  configuration.state_lower[roll] = -configuration.max_tilt_rad;
  configuration.state_upper[roll] = configuration.max_tilt_rad;
  configuration.state_lower[pitch] = -configuration.max_tilt_rad;
  configuration.state_upper[pitch] = configuration.max_tilt_rad;
  configuration.state_lower[yaw_rate] = -configuration.max_yaw_rate_rad_s;
  configuration.state_upper[yaw_rate] = configuration.max_yaw_rate_rad_s;
  configuration.input_lower[roll_command] = -configuration.max_tilt_rad;
  configuration.input_upper[roll_command] = configuration.max_tilt_rad;
  configuration.input_lower[pitch_command] = -configuration.max_tilt_rad;
  configuration.input_upper[pitch_command] = configuration.max_tilt_rad;
  configuration.input_lower[yaw_command] = -configuration.max_yaw_command_rad;
  configuration.input_upper[yaw_command] = configuration.max_yaw_command_rad;
  configuration.state_lower[collective_specific_force] =
      configuration.min_collective_specific_force_m_s2;
  configuration.state_upper[collective_specific_force] =
      configuration.max_collective_specific_force_m_s2;
  configuration.input_lower[collective_specific_force_command] =
      configuration.min_collective_specific_force_m_s2;
  configuration.input_upper[collective_specific_force_command] =
      configuration.max_collective_specific_force_m_s2;
  return configuration;
}

Configuration makeFlightConfiguration(const ModelParameters &model) {
  Configuration configuration = makeConfiguration(model);
  configuration.stage_weights = {80.0, 80.0, 200.0, 20.0, 20.0, 40.0,
                                 30.0, 30.0, 10.0, 2.0, 10.0};
  configuration.terminal_weights = {100.0, 100.0, 250.0, 30.0, 30.0, 60.0,
                                    50.0, 50.0, 15.0, 3.0, 15.0};
  configuration.input_weights = {2.0, 2.0, 1.0, 1.0};
  return configuration;
}

ReferenceHorizon makeHoldHorizon(double position_z, double gravity) {
  ReferenceTrajectory hold;
  hold.header_time_seconds = 1.0;
  hold.hold_after_end = true;
  hold.points.push_back({0.0,
                         {0.0, 0.0, position_z},
                         {0.0, 0.0, 0.0},
                         {0.0, 0.0, 0.0},
                         0.0,
                         0.0});

  ReferenceHorizon horizon;
  require(buildReferenceHorizon(hold, 0.0, 0.05, 0.7853981633974483, gravity,
                                horizon),
          "failed to build hold horizon");
  return horizon;
}

SolveResult solveAndRequireSafeOutcome(AcadosTpmcSolver &solver,
                                       const State &state,
                                       const ReferenceHorizon &reference,
                                       const Input &previous_input) {
  SolveRequest request;
  request.initial_state = state;
  request.reference = reference;
  request.previous_input = previous_input;
  request.deadline = Clock::now() + std::chrono::seconds(1);

  SolveResult result = solver.solve(request);
  require(result.sqp_statistics_count > 0,
          "Acados solver statistics are missing");
  require(result.sqp_statistics_count <= kMaximumSqpStatisticsRows,
          "Acados SQP iteration diagnostics exceed their fixed capacity");
  if (result.status == SolverStatus::maximum_iterations) {
    require(!result.valid, "maximum-iteration output must not be marked valid");
    require(result.detail.find("solver_output_rejected: status_not_success") !=
                std::string::npos,
            "maximum-iteration rejection diagnostic is missing");
    require(result.detail.find("safe_iterate_retained_for_warm_start") ==
                std::string::npos,
            "maximum-iteration iterate must not be retained for warm start");
    return result;
  }
  if (!result.valid) {
    std::cerr << "TMPC stress solve failed: status="
              << static_cast<int>(result.status) << " detail=" << result.detail
              << " solve_time=" << result.solve_time_seconds
              << " constraint_violation=" << result.max_constraint_violation
              << " first_input=[" << result.first_input[roll_command] << ", "
              << result.first_input[pitch_command] << ", "
              << result.first_input[yaw_command] << ", "
              << result.first_input[collective_specific_force_command] << "]\n";
  }
  require(result.status == SolverStatus::success,
          "External Mode regression returned an unexpected solver status");
  require(result.valid, "External Mode regression result is invalid");
  require(finite(result.first_input),
          "External Mode regression returned non-finite input");
  require(result.max_constraint_violation <= 1.0e-4,
          "External Mode regression violated a constraint");
  // SQP_RTI intentionally skips KKT residual computation (returns NaN) to
  // avoid an extra nonlinear-function evaluation on the 50 Hz control path.
  // Only full SQP solvers are expected to report finite KKT residuals.
  const bool is_rti =
      std::string(solver.backendName()).find("RTI") != std::string::npos;
  if (!is_rti) {
    require(finite(result.kkt_residuals),
            "External Mode regression has non-finite KKT residuals");
  }
  return result;
}

} // namespace

int main() {
  using namespace mpc_controller::tpmc;

  {
    const auto straight_corner = mpc_controller::trajectory::blendedCornerVelocity(
      {0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, {20.0, 0.0, 0.0}, 8.0, 2.0);
    require(near(straight_corner[0], 8.0, 1.0e-12) &&
            near(straight_corner[1], 0.0, 1.0e-12),
            "straight waypoint must retain the configured horizontal speed");

    const auto right_angle_corner = mpc_controller::trajectory::blendedCornerVelocity(
      {0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, {10.0, 10.0, 0.0}, 8.0, 2.0);
    require(right_angle_corner[0] > 0.0 && right_angle_corner[1] > 0.0,
            "right-angle waypoint velocity must follow the direction bisector");
    require(std::hypot(right_angle_corner[0], right_angle_corner[1]) < 4.0,
            "right-angle waypoint velocity was not reduced sufficiently");

    const auto u_turn_corner = mpc_controller::trajectory::blendedCornerVelocity(
      {0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, 8.0, 2.0);
    require(near(std::hypot(u_turn_corner[0], u_turn_corner[1]), 0.0, 1.0e-12),
            "U-turn waypoint must stop before reversing direction");

    const auto duplicate_corner = mpc_controller::trajectory::blendedCornerVelocity(
      {10.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, {20.0, 0.0, 0.0}, 8.0, 2.0);
    require(near(std::hypot(duplicate_corner[0], duplicate_corner[1]), 0.0, 1.0e-12),
            "duplicate waypoint must not create a non-zero terminal velocity");

    mpc_controller::trajectory::Boundary start;
    start.sample.position = {0.0, 0.0, 10.0};
    start.sample.yaw = 0.0;
    mpc_controller::trajectory::Boundary finish;
    finish.sample.position = {30.0, 5.0, 10.0};
    finish.sample.velocity = {0.0, 4.0, 0.0};
    finish.sample.yaw = 0.5 * kPi;
    mpc_controller::trajectory::Limits limits;
    limits.maximum_horizontal_speed_m_s = 18.0;
    limits.maximum_vertical_speed_m_s = 1.5;
    limits.maximum_acceleration_m_s2 = 8.0;
    limits.maximum_jerk_m_s3 = 12.0;
    limits.maximum_heading_rate_rad_s = kPi / 3.0;
    const auto trajectory =
      mpc_controller::trajectory::QuinticSegment::create(start, finish, limits);
    require(trajectory.has_value(), "minimum-time trajectory could not be planned");
    const auto initial = trajectory->sample(0.0);
    const auto terminal = trajectory->sample(trajectory->durationSeconds());
    require(near(initial.position[0], start.sample.position[0], 1.0e-12),
            "trajectory does not preserve its initial position");
    require(near(terminal.position[0], finish.sample.position[0], 1.0e-9),
            "trajectory does not reach its target position");
    require(near(terminal.velocity[0], finish.sample.velocity[0], 1.0e-9) &&
              near(terminal.velocity[1], finish.sample.velocity[1], 1.0e-9),
            "trajectory terminal velocity is not preserved");
    require(near(terminal.acceleration[0], 0.0, 1.0e-8),
            "trajectory terminal acceleration is not zero");

    mpc_controller::trajectory::Boundary next_finish;
    next_finish.sample.position = {30.0, 35.0, 10.0};
    next_finish.sample.yaw = 0.5 * kPi;
    const auto next_trajectory =
      mpc_controller::trajectory::QuinticSegment::create({terminal}, next_finish, limits);
    require(next_trajectory.has_value(), "continuous next trajectory could not be planned");
    const auto next_initial = next_trajectory->sample(0.0);
    require(near(next_initial.velocity[0], terminal.velocity[0], 1.0e-9) &&
              near(next_initial.velocity[1], terminal.velocity[1], 1.0e-9),
            "velocity is discontinuous at the trajectory leg boundary");
    for (int index = 0; index <= 100; ++index) {
      const auto sample = trajectory->sample(
        trajectory->durationSeconds() * static_cast<double>(index) / 100.0);
      require(std::hypot(sample.velocity[0], sample.velocity[1]) <=
                limits.maximum_horizontal_speed_m_s + 1.0e-6,
              "trajectory exceeds horizontal speed limit");
      require(std::sqrt(sample.acceleration[0] * sample.acceleration[0] +
                        sample.acceleration[1] * sample.acceleration[1] +
                        sample.acceleration[2] * sample.acceleration[2]) <=
                limits.maximum_acceleration_m_s2 + 1.0e-6,
              "trajectory exceeds acceleration limit");
      require(std::abs(sample.yaw_rate) <=
                limits.maximum_heading_rate_rad_s + 1.0e-6,
              "trajectory exceeds maxHeadingRate");
    }
  }

  ModelParameters model;
  State hover{0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
              0.0, 0.0, 0.0, 0.0, model.gravity_m_s2};
  Input hover_command{0.0, 0.0, 0.0, model.gravity_m_s2};
  const State hover_derivative =
      continuousDynamics(hover, hover_command, model);
  assert(near(hover_derivative[velocity_x], 0.0, 1.0e-12));
  assert(near(hover_derivative[velocity_y], 0.0, 1.0e-12));
  assert(near(hover_derivative[velocity_z], 0.0, 1.0e-12));
  assert(near(hover_derivative[roll], 0.0, 1.0e-12));
  assert(near(hover_derivative[pitch], 0.0, 1.0e-12));
  assert(near(hover_derivative[yaw], 0.0, 1.0e-12));
  assert(near(hover_derivative[yaw_rate], 0.0, 1.0e-12));
  assert(near(hover_derivative[collective_specific_force], 0.0, 1.0e-12));

  State yaw_near_positive_pi = hover;
  yaw_near_positive_pi[yaw] = kPi - 0.01;
  Input equivalent_yaw_command = hover_command;
  equivalent_yaw_command[yaw_command] = -kPi + 0.01;
  const State wrapped_yaw_derivative =
      continuousDynamics(yaw_near_positive_pi, equivalent_yaw_command, model);
  require(
      near(wrapped_yaw_derivative[yaw_rate],
           model.yaw_natural_frequency_rad_s *
               model.yaw_natural_frequency_rad_s * 0.02,
           1.0e-12),
      "yaw dynamics must use the shortest periodic command error");
  require(near(wrapped_yaw_derivative[yaw], 0.0, 1.0e-12),
          "yaw angle derivative must equal measured yaw rate");

  const State integrated = integrateErk4(hover, hover_command, 0.05, model);
  for (std::size_t index = 0; index < kStateDimension; ++index) {
    assert(near(integrated[index], hover[index], 1.0e-12));
  }

  Input increased_collective_command = hover_command;
  increased_collective_command[collective_specific_force_command] += 1.0;
  const State collective_response = integrateErk4(
    hover, increased_collective_command, 0.05, model);
  require(
    collective_response[collective_specific_force] > hover[collective_specific_force] &&
    collective_response[collective_specific_force] <
      increased_collective_command[collective_specific_force_command],
    "identified collective dynamics must respond monotonically within one control period");

  mpc_controller::state_estimation::CollectiveForceFilter collective_filter;
  require(collective_filter.configure(0.15, 0.02),
          "collective filter rejected valid timing");
  require(near(*collective_filter.update(9.8), 9.8, 1.0e-12),
          "collective filter must initialize from the first measurement");
  const double filtered_force_spike = *collective_filter.update(12.1);
  require(filtered_force_spike > 9.8 && filtered_force_spike < 10.2,
          "collective filter did not attenuate a one-sample force spike");

  ReferenceTrajectory trajectory;
  trajectory.header_time_seconds = 1.0;
  trajectory.hold_after_end = true;
  trajectory.points.push_back(
      {0.0, {0.0, 0.0, 1.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, 0.0, 0.0});
  ReferenceHorizon horizon;
  require(buildReferenceHorizon(trajectory, 0.0, 0.05, 0.7853981633974483,
                                model.gravity_m_s2, horizon),
          "failed to build initial horizon");
  assert(near(horizon[0].state[roll], 0.0, 1.0e-12));
  assert(near(horizon[0].state[pitch], 0.0, 1.0e-12));
  assert(near(horizon[0].state[yaw], 0.0, 1.0e-12));
  assert(near(horizon[0].state[yaw_rate], 0.0, 1.0e-12));
  assert(near(horizon[0].state[collective_specific_force], model.gravity_m_s2,
              1.0e-12));
  assert(near(horizon[0].input[collective_specific_force_command],
              model.gravity_m_s2, 1.0e-12));

  ReferenceTrajectory accelerated;
  accelerated.header_time_seconds = 1.0;
  accelerated.points.push_back(
      {0.0, {0.0, 0.0, 1.0}, {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, 0.0, 0.0});
  accelerated.points.push_back(
      {1.0, {0.0, 0.0, 1.0}, {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, 0.0, 0.0});
  require(buildReferenceHorizon(accelerated, 0.0, 0.05, 0.7853981633974483,
                                model.gravity_m_s2, horizon),
          "failed to build accelerated horizon");
  assert(horizon[0].state[pitch] > 0.0);
  assert(near(horizon[0].state[roll], 0.0, 1.0e-12));

  Configuration configuration = makeConfiguration(model);
  assert(near(constraintViolation(hover, hover_command, configuration), 0.0,
              1.0e-12));
  Input yaw_rate_violation = hover_command;
  yaw_rate_violation[yaw_command] = configuration.max_yaw_command_rate_rad_s *
                                        configuration.sample_time_seconds +
                                    0.01;
  require(inputTransitionConstraintViolation(hover_command, yaw_rate_violation,
                                             configuration) > 0.0,
          "yaw transition violation was not detected");
  require(describeInputTransitionViolation(hover_command, yaw_rate_violation,
                                           configuration)
                  .find("input_transition[yaw_command]") != std::string::npos,
          "yaw transition diagnostic is missing");
  State tilted = hover;
  tilted[roll] = configuration.max_tilt_rad + 0.1;
  assert(constraintViolation(tilted, hover_command, configuration) > 0.0);

  AcadosTpmcSolver solver(configuration);
  assert(solver.configured());
  assert(solver.dependencyStatus() == "acados generated solver ready");

  SolveRequest request;
  request.initial_state = hover;
  request.reference = horizon;
  request.previous_input = hover_command;
  request.deadline = Clock::now() + std::chrono::seconds(1);
  const SolveResult result = solver.solve(request);
  assert(result.status == SolverStatus::success);
  assert(result.valid);
  assert(finite(result.first_input));
  assert(std::isfinite(result.solve_time_seconds));
  assert(result.solve_time_seconds >= 0.0);
  assert(std::isfinite(result.preparation_time_seconds));
  assert(std::isfinite(result.acados_wall_time_seconds));
  assert(std::isfinite(result.postprocessing_time_seconds));
  assert(std::isfinite(result.end_to_end_time_seconds));
  assert(result.preparation_time_seconds >= 0.0);
  assert(result.acados_wall_time_seconds >= 0.0);
  assert(result.postprocessing_time_seconds >= 0.0);
  assert(result.end_to_end_time_seconds >= 0.0);
  const auto postprocessing_timings = postprocessingTimings(result);
  for (const double timing_seconds : postprocessing_timings) {
    assert(std::isfinite(timing_seconds));
    assert(timing_seconds >= 0.0);
  }
  const double attributed_postprocessing_seconds =
      result.acados_metadata_time_seconds + result.diagnostics_time_seconds +
      result.sqp_statistics_time_seconds + result.prediction_read_time_seconds +
      result.constraint_validation_time_seconds +
      result.result_finalization_time_seconds;
  require(result.postprocessing_time_seconds + 1.0e-9 >=
              attributed_postprocessing_seconds,
          "postprocessing components exceed the measured total");
  require(near(result.postprocessing_time_seconds,
               attributed_postprocessing_seconds +
                   result.postprocessing_unattributed_time_seconds,
               1.0e-9),
          "postprocessing timing remainder is inconsistent");
  require(result.end_to_end_time_seconds + 1.0e-9 >=
              result.preparation_time_seconds +
                  result.acados_wall_time_seconds,
          "solver timing split must fit within the end-to-end wall time");
  assert(result.max_constraint_violation <= 1.0e-4);

  SolveRequest expired_request = request;
  expired_request.deadline = Clock::now() - std::chrono::milliseconds(1);
  const SolveResult expired_result = solver.solve(expired_request);
  require(expired_result.status == SolverStatus::deadline_exceeded,
          "expired request must report a solver deadline miss");
  require(expired_result.deadline_missed,
          "expired request must preserve deadline telemetry");
  require(!expired_result.valid,
          "expired request must not publish a solver result");
  require(expired_result.acados_wall_time_seconds == 0.0,
          "expired request must not invoke Acados");

  SolveRequest invalid_request = request;
  invalid_request.reference[0].state[position_x] =
      std::numeric_limits<double>::quiet_NaN();
  const SolveResult failed_result = solver.solve(invalid_request);
  require(!failed_result.valid, "non-finite reference must be rejected");
  require(failed_result.status == SolverStatus::invalid_input,
          "non-finite reference must report invalid input");
  require(failed_result.detail.find("reference[0]: state[position_x]") !=
              std::string::npos,
          "invalid reference diagnostic must identify the state field");

  SolveRequest invalid_collective_request = request;
  invalid_collective_request.initial_state[collective_specific_force] = 0.0;
  const SolveResult invalid_collective_result =
      solver.solve(invalid_collective_request);
  require(!invalid_collective_result.valid,
          "out-of-bound measured collective force must be rejected");
  require(invalid_collective_result.detail.find(
              "initial_state: state[collective_specific_force]") !=
              std::string::npos,
          "collective-force diagnostic must identify the handover field");
  require(!hasValidCollectiveSpecificForce(
              invalid_collective_request.initial_state, configuration),
          "invalid collective force must not be handover-ready");
  require(hasValidCollectiveSpecificForce(hover, configuration),
          "hover collective force must be handover-ready");

  const SolveResult recovered_result = solver.solve(request);
  assert(recovered_result.status == SolverStatus::success);
  assert(recovered_result.valid);

  // Representative External Mode entry: the measured vehicle is above the
  // fallback hold altitude and the controller must still produce a bounded,
  // numerically valid first command.
  Configuration flight_configuration = makeFlightConfiguration(model);
  AcadosTpmcSolver flight_solver(flight_configuration);
  assert(flight_solver.configured());
  State external_entry = hover;
  external_entry[position_z] = 3.0;
  const ReferenceHorizon fallback_hold =
      makeHoldHorizon(1.0, model.gravity_m_s2);
  solveAndRequireSafeOutcome(flight_solver, external_entry, fallback_hold,
                             hover_command);

  // Regression case from External Mode entry: PX4 reports the vehicle near
  // the captured 3 m hold point with a small yaw offset and measured thrust.
  // This must remain solvable with the nonlinear tilt constraint enabled.
  State captured_hold_entry = external_entry;
  captured_hold_entry[position_z] = 3.03;
  captured_hold_entry[yaw] = -0.138;
  captured_hold_entry[collective_specific_force] = 9.67;
  ReferenceHorizon captured_hold = makeHoldHorizon(3.03, model.gravity_m_s2);
  for (auto &reference : captured_hold) {
    reference.state[yaw] = -0.138;
    reference.input[yaw_command] = -0.138;
  }
  AcadosTpmcSolver entry_solver(flight_configuration);
  solveAndRequireSafeOutcome(entry_solver, captured_hold_entry, captured_hold,
                             hover_command);
  Input captured_hold_input = hover_command;
  captured_hold_input[yaw_command] = captured_hold_entry[yaw];
  captured_hold_input[collective_specific_force_command] =
      captured_hold_entry[collective_specific_force];
  AcadosTpmcSolver yaw_hold_solver(flight_configuration);
  const SolveResult yaw_hold_result = solveAndRequireSafeOutcome(
      yaw_hold_solver, captured_hold_entry, captured_hold, captured_hold_input);
  require(
      std::abs(shortestAngle(captured_hold_input[yaw_command],
                             yaw_hold_result.first_input[yaw_command])) < 0.005,
      "steady hold must not drive yaw away from its captured reference");
  SolveRequest yaw_rate_limited_request;
  yaw_rate_limited_request.initial_state = captured_hold_entry;
  yaw_rate_limited_request.reference = captured_hold;
  yaw_rate_limited_request.previous_input = hover_command;
  yaw_rate_limited_request.deadline = Clock::now() + std::chrono::seconds(1);
  const SolveResult yaw_rate_limited_result =
      entry_solver.solve(yaw_rate_limited_request);
  require(yaw_rate_limited_result.valid,
          "yaw-rate-limited Acados solve must remain valid");
  require(std::abs(yaw_rate_limited_result.first_input[yaw_command] -
                   yaw_rate_limited_request.previous_input[yaw_command]) <=
              flight_configuration.max_yaw_command_rate_rad_s *
                      flight_configuration.sample_time_seconds +
                  1.0e-9,
          "Acados yaw command exceeded its per-step rate bound");

  // The same log later contained a moderate attitude/velocity disturbance.
  // Keep this case covered so a solver change cannot hide the original
  // External Mode failure behind the fallback path.
  State disturbed_external_state = captured_hold_entry;
  disturbed_external_state[position_x] = 0.063;
  disturbed_external_state[position_y] = -0.009;
  disturbed_external_state[position_z] = 2.916;
  disturbed_external_state[velocity_x] = 0.262;
  disturbed_external_state[velocity_y] = 0.003;
  disturbed_external_state[velocity_z] = -0.208;
  disturbed_external_state[roll] = 0.120;
  disturbed_external_state[pitch] = -0.130;
  disturbed_external_state[collective_specific_force] = 8.50;
  AcadosTpmcSolver disturbed_solver(flight_configuration);
  const SolveResult disturbed_result = solveAndRequireSafeOutcome(
      disturbed_solver, disturbed_external_state, captured_hold, hover_command);
  require(disturbed_result.status == SolverStatus::success &&
              disturbed_result.valid,
          "disturbed External Mode cold start must converge");
  const auto &disturbed_feedback = disturbed_result.sqp_statistics[
      disturbed_result.sqp_statistics_count - 1];
  require(disturbed_feedback.qp_status == 0,
          "RTI feedback QP must converge at External Mode cold start");
  require(std::isfinite(disturbed_feedback.qp_residuals[1]),
          "RTI feedback must expose an equality-residual diagnostic");

  State measured_tilt_state = disturbed_external_state;
  measured_tilt_state[pitch] = 0.262;
  const SolveResult measured_tilt_result = disturbed_solver.solve(
      {measured_tilt_state, captured_hold, hover_command,
       Clock::now() + std::chrono::seconds(1)});
  require(measured_tilt_result.status != SolverStatus::invalid_input,
          "finite measured attitude must reach Acados for recovery planning");
  require(measured_tilt_result.valid,
          "recoverable measured tilt must produce a valid future trajectory");
  require(tiltAngle(measured_tilt_result.predicted_states[1]) <=
              flight_configuration.max_tilt_rad + 1.0e-4,
          "first predicted state must return to the tilt envelope");

  State recoverable_tilt_state = captured_hold_entry;
  recoverable_tilt_state[pitch] = flight_configuration.max_tilt_rad + 0.005;
  AcadosTpmcSolver recoverable_tilt_solver(flight_configuration);
  const SolveResult recoverable_tilt_result = recoverable_tilt_solver.solve(
      {recoverable_tilt_state, captured_hold, captured_hold_input,
       Clock::now() + std::chrono::seconds(1)});
  require(recoverable_tilt_result.status != SolverStatus::invalid_input,
          "stage-zero tilt must not be rejected as a future constraint");

  const mpc_controller::px4_thrust::Mapping thrust_mapping{model.gravity_m_s2,
                                                           0.60};
  const auto hover_thrust = mpc_controller::px4_thrust::specificForceToBodyFrdZ(
      model.gravity_m_s2, thrust_mapping);
  assert(hover_thrust.has_value());
  assert(near(*hover_thrust, -0.60, 1.0e-12));
  const auto over_limit_thrust =
      mpc_controller::px4_thrust::specificForceToBodyFrdZ(20.0, thrust_mapping);
  assert(!over_limit_thrust.has_value());

  constexpr double kCapturedYawEnuRad = -0.13;
  const Eigen::Quaterniond level_hover_enu(
      Eigen::AngleAxisd(kCapturedYawEnuRad, Eigen::Vector3d::UnitZ()));
  const auto level_hover_ned =
      mpc_controller::px4_control::fluEnuToFrdNed(level_hover_enu);
  require(level_hover_ned.has_value(),
          "level-hover frame conversion rejected a finite attitude");
  const double level_hover_yaw_ned =
      mpc_controller::command_safety::yawAngleRad(*level_hover_ned);
  require(near(level_hover_yaw_ned, 0.5 * kPi - kCapturedYawEnuRad, 1.0e-12),
          "level-hover ENU yaw was not converted to PX4 NED yaw");
  const auto level_hover_round_trip =
      mpc_controller::px4_control::frdNedToFluEnu(*level_hover_ned);
  require(level_hover_round_trip.has_value(),
          "level-hover frame conversion is not reversible");
  require(std::abs(level_hover_round_trip->angularDistance(level_hover_enu)) <
              1.0e-12,
          "level-hover frame conversion changed the physical attitude");

  mpc_controller::frame::Px4LocalPositionSample local_sample;
  local_sample.timestamp_sample = 1;
  local_sample.xy_valid = true;
  local_sample.z_valid = true;
  local_sample.v_xy_valid = true;
  local_sample.v_z_valid = true;
  local_sample.heading_good_for_control = true;
  mpc_controller::frame::Px4AttitudeSample attitude_sample;
  attitude_sample.timestamp_sample = 1;
  attitude_sample.body_frd_to_world_ned = {
      level_hover_ned->w(), level_hover_ned->x(), level_hover_ned->y(),
      level_hover_ned->z()};
  mpc_controller::frame::Px4AngularVelocitySample angular_velocity_sample;
  angular_velocity_sample.timestamp_sample = 1;
  angular_velocity_sample.body_rate_frd = {0.0, 0.0, -0.4};
  mpc_controller::frame::VehicleStateData converted_state;
  require(mpc_controller::frame::convert(local_sample, attitude_sample,
                                         angular_velocity_sample,
                                         converted_state),
          "state bridge rejected a finite level-yaw-rate sample");
  require(near(converted_state.yaw_rate_enu, 0.4, 1.0e-12),
          "state bridge did not convert FRD yaw rate to ENU/FLU");

  mpc_controller::command_safety::Limits safety_limits;
  safety_limits.maximum_tilt_rad = 0.2;
  safety_limits.minimum_collective_specific_force_m_s2 = 8.0;
  safety_limits.maximum_collective_specific_force_m_s2 = 12.0;
  safety_limits.maximum_collective_rate_m_s3 = 2.0;
  mpc_controller::command_safety::Limiter safety_limiter(safety_limits);
  const Eigen::Quaterniond unsafe_attitude =
      Eigen::AngleAxisd(0.6, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ());
  const auto first_safe_command =
      safety_limiter.limit(unsafe_attitude, 16.0, 0.05);
  require(first_safe_command.valid, "safety limiter rejected finite command");
  require(first_safe_command.tilt_limited,
          "safety limiter did not constrain unsafe tilt");
  require(first_safe_command.collective_limited,
          "safety limiter did not constrain unsafe collective force");
  require(mpc_controller::command_safety::tiltAngleRad(
              first_safe_command.attitude_body_flu_to_world_enu) <=
              safety_limits.maximum_tilt_rad + 1.0e-12,
          "safety limiter exceeded configured tilt");
  require(near(first_safe_command.collective_specific_force_m_s2,
               model.gravity_m_s2 + 0.1, 1.0e-12),
          "safety limiter exceeded configured collective slew rate");
  const auto descending_safe_command =
      safety_limiter.limit(Eigen::Quaterniond::Identity(), 1.0, 0.1);
  require(near(descending_safe_command.collective_specific_force_m_s2,
               model.gravity_m_s2 - 0.1, 1.0e-12),
          "safety limiter exceeded downward collective slew rate");
  safety_limiter.reset(0.0);
  const auto yaw_limited_command = safety_limiter.limit(
      Eigen::Quaterniond(Eigen::AngleAxisd(1.0, Eigen::Vector3d::UnitZ())),
      model.gravity_m_s2, 0.1);
  require(yaw_limited_command.yaw_limited,
          "safety limiter did not constrain yaw rate");
  require(near(mpc_controller::command_safety::yawAngleRad(
                   yaw_limited_command.attitude_body_flu_to_world_enu),
               safety_limits.maximum_yaw_rate_rad_s * 0.1, 1.0e-12),
          "safety limiter exceeded configured yaw rate");

  // A small lateral tracking error exercises the nonlinear tilt constraint
  // without depending on a particular mission trajectory.
  ReferenceTrajectory lateral_hold;
  lateral_hold.header_time_seconds = 1.0;
  lateral_hold.hold_after_end = true;
  lateral_hold.points.push_back(
      {0.0, {0.5, -0.5, 3.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, 0.0, 0.0});
  ReferenceHorizon lateral_reference;
  require(buildReferenceHorizon(lateral_hold, 0.0, 0.05,
                                flight_configuration.max_tilt_rad,
                                model.gravity_m_s2, lateral_reference),
          "failed to build lateral horizon");
  solveAndRequireSafeOutcome(flight_solver, external_entry, lateral_reference,
                             hover_command);

  double total_solve_time = 0.0;
  double maximum_solve_time = 0.0;
  std::array<double, 8> total_postprocessing_timings{};
  std::array<double, 8> maximum_postprocessing_timings{};
  constexpr int kBenchmarkIterations = 100;
  for (int iteration = 0; iteration < kBenchmarkIterations; ++iteration) {
    SolveRequest benchmark_request;
    benchmark_request.initial_state = hover;
    benchmark_request.reference = fallback_hold;
    benchmark_request.previous_input = hover_command;
    benchmark_request.deadline = Clock::now() + std::chrono::seconds(1);
    const SolveResult benchmark_result = flight_solver.solve(benchmark_request);
    assert(benchmark_result.valid);
    total_solve_time += benchmark_result.solve_time_seconds;
    maximum_solve_time =
        std::max(maximum_solve_time, benchmark_result.solve_time_seconds);
    const auto timings = postprocessingTimings(benchmark_result);
    for (std::size_t index = 0; index < timings.size(); ++index) {
      total_postprocessing_timings[index] += timings[index];
      maximum_postprocessing_timings[index] =
          std::max(maximum_postprocessing_timings[index], timings[index]);
    }
  }
  std::cout << "TMPC benchmark: iterations=" << kBenchmarkIterations
            << " mean_ms=" << (total_solve_time / kBenchmarkIterations) * 1.0e3
            << " max_ms=" << maximum_solve_time * 1.0e3 << '\n';
  constexpr std::array<const char *, 8> kPostprocessingTimingNames{
      "post", "meta", "diag", "sqp_stat", "prediction", "constraints",
      "final", "other"};
  std::cout << "TMPC postprocessing benchmark:";
  for (std::size_t index = 0; index < kPostprocessingTimingNames.size();
       ++index) {
    std::cout << ' ' << kPostprocessingTimingNames[index]
              << "_mean_ms="
              << total_postprocessing_timings[index] /
                     kBenchmarkIterations *
                     1.0e3
              << ' ' << kPostprocessingTimingNames[index]
              << "_max_ms=" << maximum_postprocessing_timings[index] * 1.0e3;
  }
  std::cout << '\n';

  return 0;
}
