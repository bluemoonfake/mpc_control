#include <gtest/gtest.h>

#include <mrs_mpc_solvers/mpc_controller.h>

#include <Eigen/Core>

#include <cmath>
#include <vector>

namespace {

// buoc du doan
constexpr int kHorizon = 26;

// x = [position (m), velocity (m/s), acceleration (m/s^2)].
constexpr int kStateDimension = 3;

// 100Hz
constexpr double kDtFirst = 0.01;

// Khoang thoi gian giua cac prediction con lai
// dt1 + (horizon - 1) * dt
constexpr double kDt = 0.20;

// a[k+1] = p1 * a[k] + p2 * u[k].
// Voi X/Y, p1=0 va p2=1 nen a[k+1]=u[k]: solver output la acceleration
// command/correction. MRS dung p1=0.5, p2=0.5 rieng cho truc Z.
constexpr double kModelP1 = 0.0;   //acc 
constexpr double kModelP2 = 1.0;   // control

// So vong lap primal-dual toi da cho moi lan giai QP.
constexpr int kMaxIterations = 45;

// Constraint |velocity[k]| <= 2 m/s tren toan prediction horizon.
constexpr double kMaxSpeed = 2.0;

// Constraint cua state acceleration
constexpr double kDisabledStateAccelerationLimit = 999.0;

// gioi han acceleration command cua MPC.
constexpr double kMaxControl = 2.0;

// Constraint toc do thay doi control, don vi m/s^3 (jerk):
// |u[0]-u_last| <= max_rate*dt1 va |u[k]-u[k-1]| <= max_rate*dt.
constexpr double kMaxControlRate = 5.0;

constexpr double kTolerance = 1.0e-5;

// Q = diag(500, 100, 100) phat sai so tai cac buoc x[1]..x[25]:
// 500*position_error^2 + 100*velocity_error^2 + 100*acceleration_error^2.
const std::vector<double> kStageWeights{500.0, 100.0, 100.0};

// S = diag(1000, 300, 300) phat state cuoi x[26]. Chi phi cuoi
const std::vector<double> kTerminalWeights{1000.0, 300.0, 300.0};

struct SolveResult {
  int iterations{};
  double first_control{};
  Eigen::MatrixXd predicted_states;
};

Eigen::MatrixXd makeConstantReference(
    const double position,
    const double velocity = 0.0,
    const double acceleration = 0.0)
{
  Eigen::MatrixXd reference =
      Eigen::MatrixXd::Zero(kHorizon * kStateDimension, 1);

  for (int step = 0; step < kHorizon; ++step) {
    reference(step * kStateDimension + 0, 0) = position;
    reference(step * kStateDimension + 1, 0) = velocity;
    reference(step * kStateDimension + 2, 0) = acceleration;
  }

  return reference;
}

SolveResult solveAxis(
    const Eigen::Vector3d& initial_state,
    Eigen::MatrixXd reference,
    const double last_control = 0.0,
    const double max_control = kMaxControl,
    const double max_control_rate = kMaxControlRate,
    const double model_p1 = kModelP1,
    const double model_p2 = kModelP2)
{
  mrs_mpc_solvers::mpc_controller::Solver solver(
      "solver_contract",
      false,
      kMaxIterations,
      kStageWeights,
      kTerminalWeights,
      kDtFirst,
      kDt,
      model_p1,
      model_p2);

  Eigen::MatrixXd state(kStateDimension, 1);
  state = initial_state;

  solver.setLastInput(last_control);
  solver.setLimits(
      kMaxSpeed,
      kDisabledStateAccelerationLimit,
      max_control,
      max_control_rate,
      kDtFirst,
      kDt);
  solver.setInitialState(state);
  solver.loadReference(reference);

  SolveResult result;
  result.iterations = solver.solveMPC();
  result.first_control = solver.getFirstControlInput();
  result.predicted_states =
      Eigen::MatrixXd::Zero(kHorizon * kStateDimension, 1);
  solver.getStates(result.predicted_states);

  return result;
}

void expectNominalSolve(const SolveResult& result)
{
  EXPECT_GT(result.iterations, 0);
  EXPECT_LT(result.iterations, kMaxIterations);
  EXPECT_TRUE(std::isfinite(result.first_control));
  EXPECT_TRUE(result.predicted_states.allFinite());
}

TEST(MrsSolverContract, ZeroStateAndReferenceProduceZeroCorrection)
{
  const auto result = solveAxis(
      Eigen::Vector3d::Zero(), makeConstantReference(0.0));

  expectNominalSolve(result);
  EXPECT_NEAR(result.first_control, 0.0, kTolerance);
  EXPECT_TRUE(result.predicted_states.allFinite());
  EXPECT_LT(result.predicted_states.cwiseAbs().maxCoeff(), kTolerance);
}

TEST(MrsSolverContract, PositivePositionStepProducesPositiveCorrection)
{
  const auto result = solveAxis(
      Eigen::Vector3d::Zero(), makeConstantReference(1.0));

  expectNominalSolve(result);
  EXPECT_GT(result.first_control, 0.0);
  EXPECT_LE(result.first_control, kMaxControl + kTolerance);

  const double first_step_rate_bound = kMaxControlRate * kDtFirst;
  EXPECT_LE(std::abs(result.first_control), first_step_rate_bound + kTolerance);
}

TEST(MrsSolverContract, PositionStepsAreSignSymmetric)
{
  const auto positive = solveAxis(
      Eigen::Vector3d::Zero(), makeConstantReference(1.0));
  const auto negative = solveAxis(
      Eigen::Vector3d::Zero(), makeConstantReference(-1.0));

  expectNominalSolve(positive);
  expectNominalSolve(negative);
  EXPECT_GT(positive.first_control, 0.0);
  EXPECT_LT(negative.first_control, 0.0);
  EXPECT_NEAR(
      negative.first_control, -positive.first_control, kTolerance);
}

TEST(MrsSolverContract, PositiveVelocityErrorProducesBrakingCorrection)
{
  const Eigen::Vector3d state(0.0, 1.0, 0.0);
  const auto result = solveAxis(state, makeConstantReference(0.0));

  expectNominalSolve(result);
  EXPECT_LT(result.first_control, 0.0);
}

TEST(MrsSolverContract, FirstControlHonorsMagnitudeAndRateLimits)
{
  constexpr double last_control = 0.70;
  constexpr double configured_max_control = 0.75;
  constexpr double configured_max_rate = 5.0;

  const auto result = solveAxis(
      Eigen::Vector3d::Zero(),
      makeConstantReference(-10.0),
      last_control,
      configured_max_control,
      configured_max_rate);

  expectNominalSolve(result);
  EXPECT_LE(std::abs(result.first_control), configured_max_control + kTolerance);
  EXPECT_LE(
      std::abs(result.first_control - last_control),
      configured_max_rate * kDtFirst + kTolerance);
}

TEST(MrsSolverContract, FirstPredictedStateMatchesConfiguredDynamics)
{
  const Eigen::Vector3d state(1.25, -0.40, 0.30);
  const auto result = solveAxis(state, makeConstantReference(2.0));

  expectNominalSolve(result);

  const double expected_position = state.x() + kDtFirst * state.y();
  const double expected_velocity = state.y() + kDtFirst * state.z();
  const double expected_acceleration =
      kModelP1 * state.z() + kModelP2 * result.first_control;

  EXPECT_NEAR(result.predicted_states(0, 0), expected_position, kTolerance);
  EXPECT_NEAR(result.predicted_states(1, 0), expected_velocity, kTolerance);
  EXPECT_NEAR(
      result.predicted_states(2, 0), expected_acceleration, kTolerance);
}

TEST(MrsSolverContract, VerticalFirstAccelerationMatchesFilteredModel)
{
  constexpr double vertical_model_p1 = 0.5;
  constexpr double vertical_model_p2 = 0.5;
  const Eigen::Vector3d state(1.25, -0.40, 0.30);

  const auto result = solveAxis(
      state,
      makeConstantReference(2.0),
      0.0,
      kMaxControl,
      kMaxControlRate,
      vertical_model_p1,
      vertical_model_p2);

  expectNominalSolve(result);

  const double expected_acceleration =
      vertical_model_p1 * state.z()
      + vertical_model_p2 * result.first_control;

  EXPECT_NEAR(
      result.predicted_states(2, 0), expected_acceleration, kTolerance);
}

}  // namespace
