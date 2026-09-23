#include "tinympc/tiny_api.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

int main()
{
  constexpr int kStateDimension = 9;
  constexpr int kInputDimension = 3;
  // TinyMPC uses N state knots and N - 1 input columns. The production
  // oracle currently has 26 control columns, hence the probe uses N = 27.
  constexpr int kInputColumns = 26;
  constexpr int kHorizon = kInputColumns + 1;
  constexpr int kInputLinearRows = 3;

  tinyMatrix a = tinyMatrix::Identity(kStateDimension, kStateDimension);
  tinyMatrix b = tinyMatrix::Zero(kStateDimension, kInputDimension);
  b.topLeftCorner(kInputDimension, kInputDimension) =
    tinyMatrix::Identity(kInputDimension, kInputDimension);
  tinyVector f = tinyVector::Zero(kStateDimension);
  tinyMatrix q = tinyMatrix::Identity(kStateDimension, kStateDimension);
  tinyMatrix r = tinyMatrix::Identity(kInputDimension, kInputDimension);
  TinySolver *solver = nullptr;
  if (tiny_setup(&solver, a, b, f, q, r, 0.05, kStateDimension,
                 kInputDimension, kHorizon, 0) != 0 || solver == nullptr) {
    std::cerr << "tiny_setup failed\n";
    return 1;
  }

  tinyMatrix tv_state_a = tinyMatrix::Zero(kHorizon, kStateDimension);
  tinyMatrix tv_state_b = tinyMatrix::Zero(1, kHorizon);
  tinyMatrix tv_input_a =
    tinyMatrix::Zero(kInputLinearRows * (kHorizon - 1), kInputDimension);
  tinyMatrix tv_input_b = tinyMatrix::Zero(kInputLinearRows, kHorizon - 1);
  if (tiny_set_tv_linear_constraints(
        solver, tv_state_a, tv_state_b, tv_input_a, tv_input_b) != 0) {
    std::cerr << "time-varying linear constraint API failed\n";
    return 2;
  }

  tinyMatrix input_a = tinyMatrix::Zero(kInputLinearRows, kInputDimension);
  // A polygon with a non-orthogonal face tests more than a setter return code:
  // ux <= 0.25, uy <= 0.25, and ux + uy <= 0.30.
  input_a << 1.0, 0.0, 0.0,
             0.0, 1.0, 0.0,
             1.0, 1.0, 0.0;
  tinyVector input_b(kInputLinearRows);
  input_b << 0.25, 0.25, 0.30;
  if (tiny_set_linear_constraints(
        solver, tinyMatrix::Zero(0, kStateDimension), tinyVector::Zero(0),
        input_a, input_b) != 0) {
    std::cerr << "linear constraint API failed\n";
    return 3;
  }

  if (tiny_update_settings(
        solver->settings, 1e-3, 1e-3, 500, 1,
        0, 0, 0, 0, 0, 1, 0, 0) != 0) {
    std::cerr << "solver settings update failed\n";
    return 4;
  }

  tinyVector x0 = tinyVector::Zero(kStateDimension);
  tinyMatrix x_ref = tinyMatrix::Zero(kStateDimension, kHorizon);
  tinyMatrix u_ref = tinyMatrix::Zero(kInputDimension, kInputColumns);
  u_ref.row(0).setConstant(2.0);
  u_ref.row(1).setConstant(2.0);
  if (tiny_set_x0(solver, x0) != 0 ||
      tiny_set_x_ref(solver, x_ref) != 0 ||
      tiny_set_u_ref(solver, u_ref) != 0) {
    std::cerr << "reference setup failed\n";
    return 5;
  }

  if (tiny_solve(solver) != 0 || solver->solution == nullptr ||
      solver->solution->solved == 0) {
    std::cerr << "constrained TinyMPC solve failed\n";
    return 6;
  }

  constexpr tinytype kConstraintTolerance = 5e-3;
  tinytype max_solution_violation = 0.0;
  tinytype max_linear_slack_violation = 0.0;
  tinytype max_solution_to_slack_gap = 0.0;
  tinytype max_positive_ux = 0.0;
  for (int column = 0; column < solver->solution->u.cols(); ++column) {
    const tinyVector input = solver->solution->u.col(column);
    const tinyVector linear_slack = solver->work->zlnew.col(column);
    max_positive_ux = std::max(max_positive_ux, input(0));
    for (int row = 0; row < input_a.rows(); ++row) {
      max_solution_violation = std::max(
        max_solution_violation, (input_a.row(row).dot(input) - input_b(row)));
      max_linear_slack_violation = std::max(
        max_linear_slack_violation,
        (input_a.row(row).dot(linear_slack) - input_b(row)));
    }
    max_solution_to_slack_gap = std::max(
      max_solution_to_slack_gap, (input - linear_slack).cwiseAbs().maxCoeff());
  }

  std::cout << "TinyMPC constrained solve: solution_max_violation="
            << max_solution_violation
            << ", linear_slack_max_violation=" << max_linear_slack_violation
            << ", solution_slack_gap=" << max_solution_to_slack_gap
            << ", max_positive_ux=" << max_positive_ux << '\n';

  if (max_solution_violation > kConstraintTolerance ||
      max_linear_slack_violation > kConstraintTolerance ||
      max_solution_to_slack_gap > kConstraintTolerance ||
      max_positive_ux < 0.05) {
    std::cerr << "PARITY_BLOCKED: TinyMPC returned trajectory is not feasible "
                 "for its enabled linear constraints\n";
    std::cerr << "TinyMPC constrained solve does not provide a feasible "
                 "returned solution with the pinned linear-constraint API\n";
    return 7;
  }

  std::cout << "TinyMPC capability probe: active constrained solve passed\n";
  std::cout << "TinyMPC parity gaps: fixed A/B/f, implicit Pinf terminal cost, "
               "and N state knots versus N-1 input columns\n";
  std::cout << "TinyMPC capability probe: runtime integration remains disabled\n";
  return 0;
}
