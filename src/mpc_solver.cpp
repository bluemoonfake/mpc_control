#include "mpc_controller/mpc_solver.hpp"

#include <osqp.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace mpc_controller::axis_mpc
{
namespace
{

inline constexpr std::size_t kConstraintCount = 4 * kHorizonLength;
using StateTrajectory = Eigen::Matrix<double, 3 * kHorizonLength, 1>;
using StateMapping = Eigen::Matrix<double, 3 * kHorizonLength, 3>;
using ControlMapping = Eigen::Matrix<double, 3 * kHorizonLength, kHorizonLength>;
using ControlMatrix = Eigen::Matrix<double, kHorizonLength, kHorizonLength>;
using ControlVector = Eigen::Matrix<double, kHorizonLength, 1>;
using ConstraintMatrix = Eigen::Matrix<double, kConstraintCount, kHorizonLength>;
using BoundsVector = Eigen::Matrix<double, kConstraintCount, 1>;
using StateLimitVector = Eigen::Matrix<double, kHorizonLength, 1>;

constexpr double kRecoveryMargin = 1.0e-6;
constexpr double kLimitToleranceMultiplier = 10.0;

bool recoveryBounds(
  double recovery, double limit,
  double &lower, double &upper) noexcept
{
  lower = -limit;
  upper = limit;
  if (recovery > limit) {
    upper = recovery + kRecoveryMargin;
    return true;
  }
  if (recovery < -limit) {
    lower = recovery - kRecoveryMargin;
    return true;
  }
  return false;
}

struct ProblemData
{
  ControlVector gradient = ControlVector::Zero();
  BoundsVector lower = BoundsVector::Zero();
  BoundsVector upper = BoundsVector::Zero();
  StateTrajectory free_prediction = StateTrajectory::Zero();
  StateLimitVector lower_speed = StateLimitVector::Zero();
  StateLimitVector upper_speed = StateLimitVector::Zero();
  StateLimitVector lower_acceleration = StateLimitVector::Zero();
  StateLimitVector upper_acceleration = StateLimitVector::Zero();
  bool recovery_constraint_active = false;
};

bool validConfiguration(const Configuration &config) noexcept
{
  const auto weights = [](const std::array<double, 3> &values) {
      return std::all_of(values.begin(), values.end(), [](double value) {
        return std::isfinite(value) && value >= 0.0;
      });
    };
  const std::array positive_values{
    config.dt_first, config.dt_later, config.max_speed, config.max_acceleration,
    config.max_control, config.max_control_rate, config.admm_rho,
    config.absolute_tolerance, config.relative_tolerance};
  return std::all_of(positive_values.begin(), positive_values.end(), [](double value) {
      return std::isfinite(value) && value > 0.0;
    })
    && std::isfinite(config.model_time_constant) && config.model_time_constant >= 0.0
    && std::isfinite(config.control_weight) && config.control_weight >= 0.0
    && std::isfinite(config.control_rate_weight) && config.control_rate_weight >= 0.0
    && weights(config.stage_weights) && weights(config.terminal_weights)
    && config.max_iterations > 0;
}

Eigen::Matrix3d transitionMatrix(double dt, double alpha) noexcept
{
  // acceleration-response model with b=1-alpha:
  // [p+;v+;a+] = [[1,dt,alpha*dt^2/2],[0,1,alpha*dt],[0,0,alpha]] x
  //                + [b*dt^2/2,b*dt,b]' u.
  // alpha+b=1 preserves constant acceleration when a=u.
  Eigen::Matrix3d matrix = Eigen::Matrix3d::Zero();
  matrix(0, 0) = 1.0;
  matrix(0, 1) = dt;
  matrix(0, 2) = 0.5 * alpha * dt * dt;
  matrix(1, 1) = 1.0;
  matrix(1, 2) = alpha * dt;
  matrix(2, 2) = alpha;
  return matrix;
}

State inputMatrix(double dt, double alpha) noexcept
{
  const double response = 1.0 - alpha;
  return State(0.5 * response * dt * dt, response * dt, response);
}

class Problem final
{
public:
  explicit Problem(const Configuration &configuration)
  : configuration_(configuration)
  {
    if (!validConfiguration(configuration_)) {
      return;
    }

    Eigen::Matrix3d state_transition = Eigen::Matrix3d::Identity();
    Eigen::Matrix<double, 3, kHorizonLength> control_transition =
      Eigen::Matrix<double, 3, kHorizonLength>::Zero();
    for (std::size_t step = 0; step < kHorizonLength; ++step) {
      const double dt = step == 0 ? configuration_.dt_first : configuration_.dt_later;
      const double alpha = configuration_.model_time_constant > 0.0 ?
        std::exp(-dt / configuration_.model_time_constant) : 0.0;
      const Eigen::Matrix3d a = transitionMatrix(dt, alpha);
      state_transition = a * state_transition;
      control_transition = a * control_transition;
      control_transition.col(static_cast<Eigen::Index>(step)) += inputMatrix(dt, alpha);
      state_mapping_.template block<3, 3>(3 * step, 0) = state_transition;
      control_mapping_.template block<3, kHorizonLength>(3 * step, 0) = control_transition;
      const bool terminal = step + 1 == kHorizonLength;
      const auto &weight = terminal ?
        configuration_.terminal_weights : configuration_.stage_weights;
      // Rectangle-rule discretization of integral(e'Qe): the 10 ms first
      // knot must not carry the same stage cost as a 200 ms knot. Keep the
      // tuned 200 ms weights as the reference scale; terminal cost is S.
      const double stage_scale = terminal ? 1.0 : dt / configuration_.dt_later;
      weights_.template segment<3>(3 * step) =
        stage_scale * State(weight[0], weight[1], weight[2]);
    }

    for (std::size_t step = 0; step < kHorizonLength; ++step) {
      const auto index = static_cast<Eigen::Index>(step);
      constraint_matrix_(index, index) = 1.0;
      constraint_matrix_(kHorizonLength + index, index) = 1.0;
      if (step > 0) {
        constraint_matrix_(kHorizonLength + index, index - 1) = -1.0;
      }
      constraint_matrix_.row(2 * kHorizonLength + index) =
        control_mapping_.row(3 * index + 1);
      constraint_matrix_.row(3 * kHorizonLength + index) =
        control_mapping_.row(3 * index + 2);
    }
    for (Eigen::Index row = 0; row < constraint_matrix_.rows(); ++row) {
      const double norm = constraint_matrix_.row(row).norm();
      constraint_scale_(row) = norm > 1.0e-12 ? 1.0 / norm : 1.0;
      constraint_matrix_.row(row) *= constraint_scale_(row);
    }

    // J = e'Qe + r_u U'U + r_du (DU-d)'(DU-d).
    // D forms successive input increments. Its first row represents
    // U[0]-u_previous; the fixed previous input is added to q at update time.
    ControlMatrix difference = ControlMatrix::Zero();
    difference(0, 0) = 1.0;
    for (std::size_t step = 1; step < kHorizonLength; ++step) {
      const auto index = static_cast<Eigen::Index>(step);
      difference(index, index) = 1.0;
      difference(index, index - 1) = -1.0;
    }
    hessian_ = 2.0 * control_mapping_.transpose()
      * weights_.asDiagonal() * control_mapping_
      + 2.0 * configuration_.control_weight * ControlMatrix::Identity()
      + 2.0 * configuration_.control_rate_weight
      * difference.transpose() * difference;
    objective_scale_ = std::max(1.0, hessian_.cwiseAbs().maxCoeff());
    hessian_ /= objective_scale_;
    valid_ = hessian_.allFinite() && constraint_matrix_.allFinite();
  }

  bool valid() const noexcept {return valid_;}
  const ControlMatrix &hessian() const noexcept {return hessian_;}
  const ConstraintMatrix &constraints() const noexcept {return constraint_matrix_;}

  bool update(
    const State &initial_state, const Reference &reference,
    double last_control, ProblemData &data) const noexcept
  {
    if (!valid_ || !initial_state.allFinite() || !std::isfinite(last_control)
      || !std::all_of(reference.begin(), reference.end(),
        [](const State &state) {return state.allFinite();})) {
      return false;
    }

    StateTrajectory reference_vector;
    for (std::size_t step = 0; step < kHorizonLength; ++step) {
      reference_vector.template segment<3>(3 * step) = reference[step];
    }
    data.free_prediction = state_mapping_ * initial_state;
    const StateTrajectory error = data.free_prediction - reference_vector;
    data.gradient = (2.0 * control_mapping_.transpose()
      * weights_.asDiagonal() * error) / objective_scale_;
    data.gradient(0) -= 2.0 * configuration_.control_rate_weight
      * last_control / objective_scale_;

    ControlVector recovery_control;
    double previous_control = std::clamp(
      last_control, -configuration_.max_control, configuration_.max_control);
    for (std::size_t step = 0; step < kHorizonLength; ++step) {
      const double dt = step == 0 ? configuration_.dt_first : configuration_.dt_later;
      const double change = configuration_.max_control_rate * dt;
      previous_control = previous_control > 0.0 ?
        std::max(0.0, previous_control - change) :
        std::min(0.0, previous_control + change);
      recovery_control(static_cast<Eigen::Index>(step)) = previous_control;
    }
    const StateTrajectory recovery = data.free_prediction
      + control_mapping_ * recovery_control;

    data.recovery_constraint_active = false;
    for (std::size_t step = 0; step < kHorizonLength; ++step) {
      const auto index = static_cast<Eigen::Index>(step);
      data.lower(index) = -configuration_.max_control;
      data.upper(index) = configuration_.max_control;

      const auto rate_row = static_cast<Eigen::Index>(kHorizonLength + step);
      const double dt = step == 0 ? configuration_.dt_first : configuration_.dt_later;
      const double change = configuration_.max_control_rate * dt;
      data.lower(rate_row) = step == 0 ? last_control - change : -change;
      data.upper(rate_row) = step == 0 ? last_control + change : change;

      const double recovery_speed = recovery(3 * index + 1);
      data.recovery_constraint_active |= recoveryBounds(
        recovery_speed, configuration_.max_speed,
        data.lower_speed(index), data.upper_speed(index));
      const auto speed_row = static_cast<Eigen::Index>(2 * kHorizonLength + step);
      data.lower(speed_row) = data.lower_speed(index) - data.free_prediction(3 * index + 1);
      data.upper(speed_row) = data.upper_speed(index) - data.free_prediction(3 * index + 1);

      const double recovery_acceleration = recovery(3 * index + 2);
      data.recovery_constraint_active |= recoveryBounds(
        recovery_acceleration, configuration_.max_acceleration,
        data.lower_acceleration(index), data.upper_acceleration(index));
      const auto acceleration_row = static_cast<Eigen::Index>(3 * kHorizonLength + step);
      data.lower(acceleration_row) = data.lower_acceleration(index)
        - data.free_prediction(3 * index + 2);
      data.upper(acceleration_row) = data.upper_acceleration(index)
        - data.free_prediction(3 * index + 2);
    }
    data.lower.array() *= constraint_scale_.array();
    data.upper.array() *= constraint_scale_.array();
    return data.gradient.allFinite() && data.lower.allFinite() && data.upper.allFinite()
      && (data.lower.array() <= data.upper.array()).all();
  }

  Prediction predict(const ProblemData &data, const ControlVector &control) const noexcept
  {
    Prediction prediction{};
    const StateTrajectory states = data.free_prediction + control_mapping_ * control;
    for (std::size_t step = 0; step < kHorizonLength; ++step) {
      prediction[step] = states.template segment<3>(3 * step);
    }
    return prediction;
  }

  bool outputValid(
    const Result &result, const ProblemData &data,
    double last_control) const noexcept
  {
    const double tolerance = kLimitToleranceMultiplier
      * (configuration_.absolute_tolerance + configuration_.relative_tolerance);
    if (!std::isfinite(result.first_control)
      || std::abs(result.first_control) > configuration_.max_control + tolerance
      || std::abs(result.first_control - last_control)
      > configuration_.max_control_rate * configuration_.dt_first + tolerance) {
      return false;
    }
    for (std::size_t step = 0; step < kHorizonLength; ++step) {
      const auto index = static_cast<Eigen::Index>(step);
      const State &state = result.prediction[step];
      if (!state.allFinite()
        || state(1) < data.lower_speed(index) - tolerance
        || state(1) > data.upper_speed(index) + tolerance
        || state(2) < data.lower_acceleration(index) - tolerance
        || state(2) > data.upper_acceleration(index) + tolerance) {
        return false;
      }
    }
    return true;
  }

private:
  Configuration configuration_;
  bool valid_ = false;
  StateMapping state_mapping_ = StateMapping::Zero();
  ControlMapping control_mapping_ = ControlMapping::Zero();
  ConstraintMatrix constraint_matrix_ = ConstraintMatrix::Zero();
  BoundsVector constraint_scale_ = BoundsVector::Ones();
  ControlMatrix hessian_ = ControlMatrix::Zero();
  StateTrajectory weights_ = StateTrajectory::Zero();
  double objective_scale_ = 1.0;
};

struct CscStorage
{
  using MatrixPtr = std::unique_ptr<OSQPCscMatrix, decltype(&OSQPCscMatrix_free)>;

  std::vector<OSQPFloat> values;
  std::vector<OSQPInt> rows;
  std::vector<OSQPInt> columns;
  MatrixPtr matrix{nullptr, &OSQPCscMatrix_free};
};

template<typename Matrix>
void makeCsc(const Matrix &input, bool upper_triangle, CscStorage &output)
{
  output.columns.push_back(0);
  for (Eigen::Index column = 0; column < input.cols(); ++column) {
    const Eigen::Index last_row = upper_triangle ? column : input.rows() - 1;
    for (Eigen::Index row = 0; row <= last_row; ++row) {
      const double value = input(row, column);
      if (std::abs(value) > 1.0e-14) {
        output.values.push_back(value);
        output.rows.push_back(static_cast<OSQPInt>(row));
      }
    }
    output.columns.push_back(static_cast<OSQPInt>(output.values.size()));
  }
  output.matrix.reset(OSQPCscMatrix_new(
    input.rows(), input.cols(), output.values.size(), output.values.data(),
    output.rows.data(), output.columns.data()));
}

Status statusFromOsqp(OSQPInt status) noexcept
{
  switch (status) {
    case OSQP_PRIMAL_INFEASIBLE:
    case OSQP_PRIMAL_INFEASIBLE_INACCURATE:
    case OSQP_DUAL_INFEASIBLE:
    case OSQP_DUAL_INFEASIBLE_INACCURATE:
      return Status::infeasible_bounds;
    case OSQP_TIME_LIMIT_REACHED:
      return Status::deadline_exceeded;
    case OSQP_MAX_ITER_REACHED:
      return Status::max_iterations;
    case OSQP_NON_CVX:
      return Status::factorization_failure;
    default:
      return Status::invalid_input;
  }
}

}  // namespace

class Solver::Impl
{
public:
  explicit Impl(const Configuration &configuration)
  : configuration_(configuration), problem_(configuration)
  {
    if (!problem_.valid()) {
      return;
    }
    makeCsc(problem_.hessian(), true, p_);
    makeCsc(problem_.constraints(), false, a_);
    gradient_.assign(kHorizonLength, 0.0);
    lower_.assign(kConstraintCount, -1.0);
    upper_.assign(kConstraintCount, 1.0);
    dual_.assign(kConstraintCount, 0.0);
    settings_.reset(OSQPSettings_new());
    if (!p_.matrix || !a_.matrix || !settings_) {
      return;
    }
    settings_->verbose = 0;
    settings_->warm_starting = 1;
    settings_->polishing = 0;
    settings_->max_iter = configuration_.max_iterations;
    settings_->rho = configuration_.admm_rho;
    settings_->eps_abs = configuration_.absolute_tolerance;
    settings_->eps_rel = configuration_.relative_tolerance;
    settings_->check_termination = 1;
    settings_->adaptive_rho = OSQP_ADAPTIVE_RHO_UPDATE_ITERATIONS;
    settings_->adaptive_rho_interval = 25;
    OSQPSolver *solver = nullptr;
    const OSQPInt error = osqp_setup(
      &solver, p_.matrix.get(), gradient_.data(), a_.matrix.get(),
      lower_.data(), upper_.data(), kConstraintCount,
      kHorizonLength, settings_.get());
    solver_.reset(solver);
    configured_ = error == 0 && solver_;
  }

  bool configured() const noexcept {return configured_;}

  void reset() noexcept
  {
    warm_start_.setZero();
    warm_start_valid_ = false;
    if (solver_) {
      osqp_cold_start(solver_.get());
    }
  }

  Result solve(
    const State &initial_state, const Reference &reference,
    double last_control, Clock::time_point deadline) noexcept
  {
    Result result;
    if (!configured_ || Clock::now() >= deadline) {
      result.status = configured_ ? Status::deadline_exceeded : Status::invalid_input;
      return result;
    }

    ProblemData data;
    if (!problem_.update(initial_state, reference, last_control, data)) {
      return result;
    }
    result.recovery_constraint_active = data.recovery_constraint_active;
    std::copy_n(data.gradient.data(), kHorizonLength, gradient_.begin());
    std::copy_n(data.lower.data(), kConstraintCount, lower_.begin());
    std::copy_n(data.upper.data(), kConstraintCount, upper_.begin());
    if (osqp_update_data_vec(
        solver_.get(), gradient_.data(), lower_.data(), upper_.data()) != 0) {
      return result;
    }

    const double remaining = std::chrono::duration<double>(deadline - Clock::now()).count();
    if (!std::isfinite(remaining) || remaining <= 0.0) {
      result.status = Status::deadline_exceeded;
      return result;
    }
    settings_->time_limit = std::max(remaining, 1.0e-6);
    if (osqp_update_settings(solver_.get(), settings_.get()) != 0) {
      return result;
    }

    if (warm_start_valid_) {
      // Receding horizon: move every planned input one knot forward. Updating
      // only U[0] leaves U[1..N-1] at stale time indices and can bias the
      // inexact ADMM solution until the first command has the wrong sign.
      warm_start_.head(kHorizonLength - 1) =
        warm_start_.tail(kHorizonLength - 1).eval();
    } else {
      warm_start_.setConstant(std::clamp(
        last_control, -configuration_.max_control, configuration_.max_control));
    }
    std::fill(dual_.begin(), dual_.end(), 0.0);
    if (osqp_warm_start(solver_.get(), warm_start_.data(), dual_.data()) != 0
      || osqp_solve(solver_.get()) != 0 || !solver_->info) {
      result.status = Status::factorization_failure;
      reset();
      return result;
    }

    result.iterations = solver_->info->iter;
    result.primal_residual = solver_->info->prim_res;
    result.dual_residual = solver_->info->dual_res;
    const OSQPInt status = solver_->info->status_val;
    if (status != OSQP_SOLVED && status != OSQP_SOLVED_INACCURATE) {
      result.status = statusFromOsqp(status);
      reset();
      return result;
    }
    if (!solver_->solution || !solver_->solution->x || !solver_->solution->y) {
      result.status = Status::non_finite_output;
      reset();
      return result;
    }

    for (std::size_t index = 0; index < kHorizonLength; ++index) {
      warm_start_(index) = solver_->solution->x[index];
    }
    if (!warm_start_.allFinite()) {
      result.status = Status::non_finite_output;
      reset();
      return result;
    }

    const BoundsVector constraint_value = problem_.constraints() * warm_start_;
    const BoundsVector projected = constraint_value.cwiseMax(data.lower).cwiseMin(data.upper);
    result.primal_tolerance = configuration_.absolute_tolerance
      + configuration_.relative_tolerance * std::max(
      constraint_value.template lpNorm<Eigen::Infinity>(),
      projected.template lpNorm<Eigen::Infinity>());
    Eigen::Map<const BoundsVector> dual(solver_->solution->y);
    result.dual_tolerance = configuration_.absolute_tolerance
      + configuration_.relative_tolerance
      * (problem_.constraints().transpose() * dual).template lpNorm<Eigen::Infinity>();

    result.prediction = problem_.predict(data, warm_start_);
    result.first_control = warm_start_(0);
    result.valid = problem_.outputValid(result, data, last_control);
    if (!result.valid) {
      result.status = Status::output_limit;
      reset();
      return result;
    }
    result.status = Status::success;
    warm_start_valid_ = true;
    return result;
  }

private:
  Configuration configuration_;
  Problem problem_;
  CscStorage p_;
  CscStorage a_;
  std::unique_ptr<OSQPSettings, decltype(&OSQPSettings_free)> settings_{
    nullptr, &OSQPSettings_free};
  std::unique_ptr<OSQPSolver, decltype(&osqp_cleanup)> solver_{
    nullptr, &osqp_cleanup};
  std::vector<OSQPFloat> gradient_;
  std::vector<OSQPFloat> lower_;
  std::vector<OSQPFloat> upper_;
  std::vector<OSQPFloat> dual_;
  ControlVector warm_start_ = ControlVector::Zero();
  bool configured_ = false;
  bool warm_start_valid_ = false;
};

Solver::Solver(const Configuration &configuration)
: impl_(std::make_unique<Impl>(configuration))
{
}

Solver::~Solver() = default;

bool Solver::configured() const noexcept
{
  return impl_ && impl_->configured();
}

void Solver::reset() noexcept
{
  if (impl_) {
    impl_->reset();
  }
}

Result Solver::solve(
  const State &initial_state, const Reference &reference,
  double last_control, Clock::time_point deadline) noexcept
{
  return impl_ ? impl_->solve(initial_state, reference, last_control, deadline) : Result{};
}

}  // namespace mpc_controller::axis_mpc

namespace mpc_controller::coupled_mpc
{
namespace
{

constexpr std::size_t kControlCount = kInputDimension * kHorizonLength;
constexpr std::size_t kVelocitySlackOffset = kControlCount;
constexpr std::size_t kAccelerationSlackOffset = kVelocitySlackOffset + kHorizonLength;
constexpr std::size_t kDecisionCount = kAccelerationSlackOffset + kHorizonLength;

// Row layout per horizon step, parameterized by kPolygonSides:
//   [0 .. P-1]         u_xy polygon      (P rows)
//   [P]                u_z               (1 row)
//   [P+1 .. 2P]        Δu_xy polygon     (P rows)
//   [2P+1]             Δu_z              (1 row)
//   [2P+2 .. 3P+1]     v_xy polygon      (P rows)
//   [3P+2]             v_z upper         (1 row)
//   [3P+3]             v_z lower         (1 row)
//   [3P+4 .. 4P+3]     a_xy polygon      (P rows)
//   [4P+4]             a_z upper         (1 row)
//   [4P+5]             a_z lower         (1 row)
//   [4P+6]             s_v slack         (1 row)
//   [4P+7]             s_a slack         (1 row)
//   [4P+8 .. 5P+7]     tilt polygon      (P rows)
//   [5P+8]             collective thrust (1 row)
//   Total: 5P + 9 rows per step
constexpr std::size_t kRowUz          = kPolygonSides;
constexpr std::size_t kRowRateXyBase  = kPolygonSides + 1;
constexpr std::size_t kRowRateUz      = 2 * kPolygonSides + 1;
constexpr std::size_t kRowVelXyBase   = 2 * kPolygonSides + 2;
constexpr std::size_t kRowVelZUpper   = 3 * kPolygonSides + 2;
constexpr std::size_t kRowVelZLower   = 3 * kPolygonSides + 3;
constexpr std::size_t kRowAccXyBase   = 3 * kPolygonSides + 4;
constexpr std::size_t kRowAccZUpper   = 4 * kPolygonSides + 4;
constexpr std::size_t kRowAccZLower   = 4 * kPolygonSides + 5;
constexpr std::size_t kRowSlackVel    = 4 * kPolygonSides + 6;
constexpr std::size_t kRowSlackAcc    = 4 * kPolygonSides + 7;
constexpr std::size_t kRowTiltBase    = 4 * kPolygonSides + 8;
constexpr std::size_t kRowThrust      = 5 * kPolygonSides + 8;
constexpr std::size_t kRowsPerStep    = 5 * kPolygonSides + 9;
constexpr std::size_t kConstraintCount = kRowsPerStep * kHorizonLength;
constexpr double kLimitToleranceMultiplier = 10.0;

using StateTrajectory = Eigen::VectorXd;
using DecisionVector = Eigen::VectorXd;
using ConstraintVector = Eigen::VectorXd;
using StateMapping = Eigen::MatrixXd;
using ControlMapping = Eigen::MatrixXd;
using ProblemMatrix = Eigen::MatrixXd;

std::size_t controlIndex(std::size_t step, std::size_t axis) noexcept
{
  return kInputDimension * step + axis;
}

std::size_t rowIndex(std::size_t step, std::size_t local) noexcept
{
  return kRowsPerStep * step + local;
}

bool finiteNonnegativeWeights(const std::array<double, 3> &values) noexcept
{
  return std::all_of(values.begin(), values.end(), [](double value) {
    return std::isfinite(value) && value >= 0.0;
  });
}

bool validConfiguration(const Configuration &config) noexcept
{
  const std::array positive_values{
    config.dt_first, config.dt_later, config.max_speed_xy, config.max_speed_z,
    config.max_acceleration_xy, config.max_acceleration_z, config.max_control_xy,
    config.max_control_z, config.max_control_rate_xy, config.max_control_rate_z,
    config.gravity_m_s2, config.max_tilt_rad,
    config.min_collective_specific_force_m_s2,config.max_collective_specific_force_m_s2, config.constraint_slack_weight,
    config.max_constraint_slack, config.admm_rho, config.absolute_tolerance,config.relative_tolerance};
  if (!std::all_of(positive_values.begin(), positive_values.end(), [](double value) {
      return std::isfinite(value) && value > 0.0;
    })) {
    return false;
  }
  if (!std::all_of(
      config.model_time_constant_xyz.begin(), config.model_time_constant_xyz.end(),
      [](double value) {return std::isfinite(value) && value >= 0.0;})) {
    return false;
  }
  if (!finiteNonnegativeWeights(config.control_weights)
    || !finiteNonnegativeWeights(config.control_rate_weights)) {
    return false;
  }
  if (!finiteNonnegativeWeights(config.stage_weights_xy)
    || !finiteNonnegativeWeights(config.terminal_weights_xy)
    || !finiteNonnegativeWeights(config.stage_weights_z)
    || !finiteNonnegativeWeights(config.terminal_weights_z)
    || config.max_iterations <= 0 || config.max_tilt_rad >= 0.5 * M_PI
    || config.min_collective_specific_force_m_s2
    >= config.max_collective_specific_force_m_s2
    || config.max_control_xy >= config.max_collective_specific_force_m_s2) {
    return false;
  }
  // The inscribed cylindrical force envelope must retain a non-empty vertical range.
  const double maximum_force_z = std::sqrt(config.max_collective_specific_force_m_s2 * config.max_collective_specific_force_m_s2- config.max_control_xy * config.max_control_xy);
  const double lower_control_z = std::max(-config.max_control_z,config.min_collective_specific_force_m_s2 - config.gravity_m_s2);
  const double upper_control_z = std::min(config.max_control_z, maximum_force_z - config.gravity_m_s2);
  return std::isfinite(maximum_force_z) && lower_control_z <= upper_control_z;
}

Eigen::Matrix<double, 9, 9> transitionMatrix(
  double dt, const Eigen::Vector3d &alpha) noexcept
{
  // Block-diagonal 3-D form of the same a+b=1 response model used above.
  Eigen::Matrix<double, 9, 9> matrix = Eigen::Matrix<double, 9, 9>::Zero();
  matrix.template block<3, 3>(0, 0).setIdentity();
  matrix.template block<3, 3>(0, 3) = dt * Eigen::Matrix3d::Identity();
  matrix.template block<3, 3>(0, 6) = (0.5 * dt * dt * alpha).asDiagonal();
  matrix.template block<3, 3>(3, 3).setIdentity();
  matrix.template block<3, 3>(3, 6) = (dt * alpha).asDiagonal();
  matrix.template block<3, 3>(6, 6) = alpha.asDiagonal();
  return matrix;
}

Eigen::Matrix<double, 9, 3> inputMatrix(
  double dt, const Eigen::Vector3d &alpha) noexcept
{
  const Eigen::Vector3d response = Eigen::Vector3d::Ones() - alpha;
  Eigen::Matrix<double, 9, 3> matrix = Eigen::Matrix<double, 9, 3>::Zero();
  matrix.template block<3, 3>(0, 0) =(0.5 * dt * dt * response).asDiagonal();
  matrix.template block<3, 3>(3, 0) = (dt * response).asDiagonal();
  matrix.template block<3, 3>(6, 0) = response.asDiagonal();
  return matrix;
}

struct ProblemData
{
  DecisionVector gradient = DecisionVector::Zero(kDecisionCount);
  ConstraintVector lower = ConstraintVector::Zero(kConstraintCount);
  ConstraintVector upper = ConstraintVector::Zero(kConstraintCount);
  StateTrajectory free_prediction = StateTrajectory::Zero(9 * kHorizonLength);
};

class Problem final
{
public:
  explicit Problem(const Configuration &config)
  : config_(config),
    state_mapping_(StateMapping::Zero(9 * kHorizonLength, 9)),
    control_mapping_(ControlMapping::Zero(9 * kHorizonLength, kControlCount)),
    weights_(StateTrajectory::Zero(9 * kHorizonLength)),
    hessian_(ProblemMatrix::Zero(kDecisionCount, kDecisionCount)),
    constraints_(ProblemMatrix::Zero(kConstraintCount, kDecisionCount)),
    constraint_scale_(ConstraintVector::Ones(kConstraintCount))
  {
    if (!validConfiguration(config_)) return;
    buildPrediction();
    buildObjective();
    buildConstraints();
    valid_ = state_mapping_.allFinite() && control_mapping_.allFinite()
      && hessian_.allFinite() && constraints_.allFinite();
  }

  bool valid() const noexcept {return valid_;}
  const ProblemMatrix &hessian() const noexcept {return hessian_;}
  const ProblemMatrix &constraints() const noexcept {return constraints_;}

  bool update(
    const State &initial_state, const Reference &reference,
    const Input &last_control, ProblemData &data) const noexcept
  {
    if (!valid_ || !initial_state.allFinite() || !last_control.allFinite()
      || !std::all_of(reference.begin(), reference.end(),
        [](const State &state) {return state.allFinite();})) {
      return false;
    }

    StateTrajectory reference_vector(9 * kHorizonLength);
    for (std::size_t step = 0; step < kHorizonLength; ++step) {
      reference_vector.segment<9>(9 * step) = reference[step];
    }
    data.free_prediction = state_mapping_ * initial_state;
    const StateTrajectory error = data.free_prediction - reference_vector;
    data.gradient.setZero();
    data.gradient.head(kControlCount) = (2.0 * control_mapping_.transpose() * weights_.asDiagonal() * error)/ objective_scale_;
    // The first rate term is r_du ||u_0-u_previous||^2. Its constant part is
    // omitted; the remaining linear term belongs in the OSQP gradient.
    for (std::size_t axis = 0; axis < kInputDimension; ++axis) {
      data.gradient(static_cast<Eigen::Index>(axis)) -= 2.0 * config_.control_rate_weights[axis] * last_control(static_cast<Eigen::Index>(axis)) / objective_scale_;
    }

    data.lower.setConstant(-OSQP_INFTY);
    data.upper.setConstant(OSQP_INFTY);
    const double polygon_scale = std::cos(M_PI / static_cast<double>(kPolygonSides));
    const double velocity_xy_limit = config_.max_speed_xy * polygon_scale;
    const double acceleration_xy_limit = config_.max_acceleration_xy * polygon_scale;
    const double control_xy_limit = config_.max_control_xy * polygon_scale;
    const double tilt_scale = std::tan(config_.max_tilt_rad) * polygon_scale;
    const double maximum_force_z = std::sqrt(config_.max_collective_specific_force_m_s2* config_.max_collective_specific_force_m_s2- config_.max_control_xy * config_.max_control_xy);

    for (std::size_t step = 0; step < kHorizonLength; ++step) {
      const double dt = step == 0 ? config_.dt_first : config_.dt_later;
      for (std::size_t side = 0; side < kPolygonSides; ++side) {
        const double angle = 2.0 * M_PI * static_cast<double>(side)
          / static_cast<double>(kPolygonSides);
        const double nx = std::cos(angle);
        const double ny = std::sin(angle);

        data.upper(rowIndex(step, side)) = control_xy_limit;
        const std::size_t rate_row = rowIndex(step, kRowRateXyBase + side);
        data.upper(rate_row) = config_.max_control_rate_xy * dt * polygon_scale;
        if (step == 0) {
          data.upper(rate_row) += nx * last_control.x() + ny * last_control.y();
        }

        const std::size_t velocity_row = rowIndex(step, kRowVelXyBase + side);
        const double free_velocity = nx * data.free_prediction(9 * step + 3)
          + ny * data.free_prediction(9 * step + 4);
        data.upper(velocity_row) = velocity_xy_limit - free_velocity;

        const std::size_t acceleration_row = rowIndex(step, kRowAccXyBase + side);
        const double free_acceleration = nx * data.free_prediction(9 * step + 6)
          + ny * data.free_prediction(9 * step + 7);
        data.upper(acceleration_row) = acceleration_xy_limit - free_acceleration;

        data.upper(rowIndex(step, kRowTiltBase + side)) = tilt_scale * config_.gravity_m_s2;
      }

      const std::size_t input_z_row = rowIndex(step, kRowUz);
      data.lower(input_z_row) = -config_.max_control_z;
      data.upper(input_z_row) = config_.max_control_z;

      const std::size_t rate_z_row = rowIndex(step, kRowRateUz);
      const double rate_z_change = config_.max_control_rate_z * dt;
      data.lower(rate_z_row) = step == 0 ? last_control.z() - rate_z_change : -rate_z_change;
      data.upper(rate_z_row) = step == 0 ? last_control.z() + rate_z_change : rate_z_change;

      const double free_velocity_z = data.free_prediction(9 * step + 5);
      data.upper(rowIndex(step, kRowVelZUpper)) = config_.max_speed_z - free_velocity_z;
      data.upper(rowIndex(step, kRowVelZLower)) = config_.max_speed_z + free_velocity_z;
      const double free_acceleration_z = data.free_prediction(9 * step + 8);
      data.upper(rowIndex(step, kRowAccZUpper)) = config_.max_acceleration_z - free_acceleration_z;
      data.upper(rowIndex(step, kRowAccZLower)) = config_.max_acceleration_z + free_acceleration_z;

      data.lower(rowIndex(step, kRowSlackVel)) = 0.0;
      data.upper(rowIndex(step, kRowSlackVel)) = config_.max_constraint_slack;
      data.lower(rowIndex(step, kRowSlackAcc)) = 0.0;
      data.upper(rowIndex(step, kRowSlackAcc)) = config_.max_constraint_slack;

      const std::size_t thrust_row = rowIndex(step, kRowThrust);
      data.lower(thrust_row) = std::max(
        -config_.max_control_z,
        config_.min_collective_specific_force_m_s2 - config_.gravity_m_s2);
      data.upper(thrust_row) = std::min(
        config_.max_control_z, maximum_force_z - config_.gravity_m_s2);
    }
    data.lower.array() *= constraint_scale_.array();
    data.upper.array() *= constraint_scale_.array();
    return data.gradient.allFinite() && data.lower.allFinite() && data.upper.allFinite()
      && (data.lower.array() <= data.upper.array()).all();
  }

  Result result(
    const ProblemData &data, const DecisionVector &decision,
    double absolute_tolerance, double relative_tolerance) const noexcept
  {
    Result output;
    if (decision.size() != static_cast<Eigen::Index>(kDecisionCount)
      || !decision.allFinite()) {
      output.status = Status::non_finite_output;
      return output;
    }
    const StateTrajectory states = data.free_prediction
      + control_mapping_ * decision.head(kControlCount);
    if (!states.allFinite()) {
      output.status = Status::non_finite_output;
      return output;
    }
    for (std::size_t step = 0; step < kHorizonLength; ++step) {
      output.prediction[step] = states.segment<9>(9 * step);
      output.max_predicted_speed_xy = std::max(output.max_predicted_speed_xy,output.prediction[step].segment<2>(3).norm());
      output.max_predicted_acceleration_xy = std::max(output.max_predicted_acceleration_xy,output.prediction[step].segment<2>(6).norm());
      const Input control = decision.segment<3>(3 * step);
      const Eigen::Vector3d force(control.x(), control.y(), control.z() + config_.gravity_m_s2);
      const double force_norm = force.norm();
      if (!std::isfinite(force_norm) || force.z() <= 0.0) {
        output.status = Status::non_finite_output;
        return output;
      }
      output.max_predicted_tilt_rad = std::max(output.max_predicted_tilt_rad,std::atan2(force.head<2>().norm(), force.z()));
      output.max_predicted_collective_specific_force_m_s2 = std::max(output.max_predicted_collective_specific_force_m_s2, force_norm);
      output.max_velocity_slack = std::max(output.max_velocity_slack, decision(kVelocitySlackOffset + step));
      output.max_acceleration_slack = std::max(output.max_acceleration_slack, decision(kAccelerationSlackOffset + step));
    }
    output.first_control = decision.head<3>();
    output.recovery_constraint_active = output.max_velocity_slack > 1.0e-6 || output.max_acceleration_slack > 1.0e-6;

    const ConstraintVector value = constraints_ * decision;
    output.max_constraint_violation = 0.0;
    for (Eigen::Index row = 0; row < value.size(); ++row) {
      output.max_constraint_violation = std::max(output.max_constraint_violation,std::max(data.lower(row) - value(row), value(row) - data.upper(row)));
    }
    const double tolerance = kLimitToleranceMultiplier * (absolute_tolerance + relative_tolerance);
    output.valid = output.first_control.allFinite()
      && output.max_constraint_violation <= tolerance
      && output.max_predicted_tilt_rad <= config_.max_tilt_rad + tolerance
      && output.max_predicted_collective_specific_force_m_s2
      <= config_.max_collective_specific_force_m_s2 + tolerance
      && output.max_velocity_slack <= config_.max_constraint_slack + tolerance
      && output.max_acceleration_slack <= config_.max_constraint_slack + tolerance;
    output.status = output.valid ? Status::success : Status::output_limit;
    return output;
  }

private:
  void buildPrediction()
  {
    Eigen::Matrix<double, 9, 9> state_transition =
      Eigen::Matrix<double, 9, 9>::Identity();
    Eigen::Matrix<double, 9, Eigen::Dynamic> control_transition =
      Eigen::Matrix<double, 9, Eigen::Dynamic>::Zero(9, kControlCount);
    for (std::size_t step = 0; step < kHorizonLength; ++step) {
      const double dt = step == 0 ? config_.dt_first : config_.dt_later;
      Eigen::Vector3d alpha;
      for (std::size_t axis = 0; axis < 3; ++axis) {
        alpha(static_cast<Eigen::Index>(axis)) = config_.model_time_constant_xyz[axis] > 0.0 ?std::exp(-dt / config_.model_time_constant_xyz[axis]) : 0.0;
      }
      const auto a = transitionMatrix(dt, alpha);
      state_transition = a * state_transition;
      control_transition = a * control_transition;
      control_transition.block<9, 3>(0, 3 * step) += inputMatrix(dt, alpha);
      state_mapping_.block<9, 9>(9 * step, 0) = state_transition;
      control_mapping_.block(9 * step, 0, 9, kControlCount) = control_transition;

      const auto &xy = step + 1 == kHorizonLength ? config_.terminal_weights_xy : config_.stage_weights_xy;
      const auto &z = step + 1 == kHorizonLength ? config_.terminal_weights_z : config_.stage_weights_z;
      // J = sum(dt_k * e_k'Qe_k) + e_N'Se_N. Parameters are tuned for the
      // regular 200 ms knot, so only the shorter first stage is normalized.
      const bool terminal = step + 1 == kHorizonLength;
      const double stage_scale = terminal ? 1.0 : dt / config_.dt_later;
      for (std::size_t derivative = 0; derivative < 3; ++derivative) {
        weights_(9 * step + 3 * derivative) = stage_scale * xy[derivative];
        weights_(9 * step + 3 * derivative + 1) = stage_scale * xy[derivative];
        weights_(9 * step + 3 * derivative + 2) = stage_scale * z[derivative];
      }
    }
  }

  void buildObjective()
  {
    hessian_.topLeftCorner(kControlCount, kControlCount) =
      2.0 * control_mapping_.transpose() * weights_.asDiagonal() * control_mapping_;
    // Add r_u ||U||^2 and r_du ||DU||^2 directly to the condensed Hessian.
    // The rate block couples adjacent 3-D inputs but keeps X/Y/Z weights
    // independently tunable.
    for (std::size_t step = 0; step < kHorizonLength; ++step) {
      for (std::size_t axis = 0; axis < kInputDimension; ++axis) {
        const auto current = static_cast<Eigen::Index>(controlIndex(step, axis));
        const double control_weight = config_.control_weights[axis];
        const double rate_weight = config_.control_rate_weights[axis];
        hessian_(current, current) += 2.0 * (control_weight + rate_weight);
        if (step > 0) {
          const auto previous = static_cast<Eigen::Index>(controlIndex(step - 1, axis));
          hessian_(previous, previous) += 2.0 * rate_weight;
          hessian_(current, previous) -= 2.0 * rate_weight;
          hessian_(previous, current) -= 2.0 * rate_weight;
        }
      }
    }
    for (std::size_t step = 0; step < kHorizonLength; ++step) {
      hessian_(kVelocitySlackOffset + step, kVelocitySlackOffset + step) = 2.0 * config_.constraint_slack_weight;
      hessian_(kAccelerationSlackOffset + step, kAccelerationSlackOffset + step) = 2.0 * config_.constraint_slack_weight;
    }
    objective_scale_ = std::max(1.0, hessian_.cwiseAbs().maxCoeff());
    hessian_ /= objective_scale_;
  }

  void buildConstraints()
  {
    const double polygon_scale = std::cos(M_PI / static_cast<double>(kPolygonSides));
    const double tilt_scale = std::tan(config_.max_tilt_rad) * polygon_scale;
    for (std::size_t step = 0; step < kHorizonLength; ++step) {
      const auto ux = static_cast<Eigen::Index>(controlIndex(step, 0));
      const auto uy = static_cast<Eigen::Index>(controlIndex(step, 1));
      const auto uz = static_cast<Eigen::Index>(controlIndex(step, 2));
      for (std::size_t side = 0; side < kPolygonSides; ++side) {
        const double angle = 2.0 * M_PI * static_cast<double>(side)
          / static_cast<double>(kPolygonSides);
        const double nx = std::cos(angle);
        const double ny = std::sin(angle);

        constraints_(rowIndex(step, side), ux) = nx;
        constraints_(rowIndex(step, side), uy) = ny;

        const auto rate_row = static_cast<Eigen::Index>(rowIndex(step, kRowRateXyBase + side));
        constraints_(rate_row, ux) = nx;
        constraints_(rate_row, uy) = ny;
        if (step > 0) {
          constraints_(rate_row, controlIndex(step - 1, 0)) = -nx;
          constraints_(rate_row, controlIndex(step - 1, 1)) = -ny;
        }

        const auto velocity_row = static_cast<Eigen::Index>(rowIndex(step, kRowVelXyBase + side));
        constraints_.row(velocity_row).head(kControlCount) =
          nx * control_mapping_.row(9 * step + 3)
          + ny * control_mapping_.row(9 * step + 4);
        constraints_(velocity_row, kVelocitySlackOffset + step) = -1.0;

        const auto acceleration_row = static_cast<Eigen::Index>(rowIndex(step, kRowAccXyBase + side));
        constraints_.row(acceleration_row).head(kControlCount) =
          nx * control_mapping_.row(9 * step + 6)
          + ny * control_mapping_.row(9 * step + 7);
        constraints_(acceleration_row, kAccelerationSlackOffset + step) = -1.0;

        const auto tilt_row = static_cast<Eigen::Index>(rowIndex(step, kRowTiltBase + side));
        constraints_(tilt_row, ux) = nx;
        constraints_(tilt_row, uy) = ny;
        constraints_(tilt_row, uz) = -tilt_scale;
      }

      constraints_(rowIndex(step, kRowUz), uz) = 1.0;
      constraints_(rowIndex(step, kRowRateUz), uz) = 1.0;
      if (step > 0) {
        constraints_(rowIndex(step, kRowRateUz), controlIndex(step - 1, 2)) = -1.0;
      }

      constraints_.row(rowIndex(step, kRowVelZUpper)).head(kControlCount) =
        control_mapping_.row(9 * step + 5);
      constraints_(rowIndex(step, kRowVelZUpper), kVelocitySlackOffset + step) = -1.0;
      constraints_.row(rowIndex(step, kRowVelZLower)).head(kControlCount) =
        -control_mapping_.row(9 * step + 5);
      constraints_(rowIndex(step, kRowVelZLower), kVelocitySlackOffset + step) = -1.0;

      constraints_.row(rowIndex(step, kRowAccZUpper)).head(kControlCount) =
        control_mapping_.row(9 * step + 8);
      constraints_(rowIndex(step, kRowAccZUpper), kAccelerationSlackOffset + step) = -1.0;
      constraints_.row(rowIndex(step, kRowAccZLower)).head(kControlCount) =
        -control_mapping_.row(9 * step + 8);
      constraints_(rowIndex(step, kRowAccZLower), kAccelerationSlackOffset + step) = -1.0;

      constraints_(rowIndex(step, kRowSlackVel), kVelocitySlackOffset + step) = 1.0;
      constraints_(rowIndex(step, kRowSlackAcc), kAccelerationSlackOffset + step) = 1.0;
      constraints_(rowIndex(step, kRowThrust), uz) = 1.0;
    }

    for (Eigen::Index row = 0; row < constraints_.rows(); ++row) {
      const double norm = constraints_.row(row).norm();
      constraint_scale_(row) = norm > 1.0e-12 ? 1.0 / norm : 1.0;
      constraints_.row(row) *= constraint_scale_(row);
    }
  }

  Configuration config_;
  bool valid_ = false;
  StateMapping state_mapping_;
  ControlMapping control_mapping_;
  StateTrajectory weights_;
  ProblemMatrix hessian_;
  ProblemMatrix constraints_;
  ConstraintVector constraint_scale_;
  double objective_scale_ = 1.0;
};

struct CscStorage
{
  using MatrixPtr = std::unique_ptr<OSQPCscMatrix, decltype(&OSQPCscMatrix_free)>;
  std::vector<OSQPFloat> values;
  std::vector<OSQPInt> rows;
  std::vector<OSQPInt> columns;
  MatrixPtr matrix{nullptr, &OSQPCscMatrix_free};
};

void makeCsc(const ProblemMatrix &input, bool upper_triangle, CscStorage &output)
{
  output.columns.push_back(0);
  for (Eigen::Index column = 0; column < input.cols(); ++column) {
    const Eigen::Index last_row = upper_triangle ? column : input.rows() - 1;
    for (Eigen::Index row = 0; row <= last_row; ++row) {
      const double value = input(row, column);
      if (std::abs(value) > 1.0e-14) {
        output.values.push_back(value);
        output.rows.push_back(static_cast<OSQPInt>(row));
      }
    }
    output.columns.push_back(static_cast<OSQPInt>(output.values.size()));
  }
  output.matrix.reset(OSQPCscMatrix_new(
    input.rows(), input.cols(), output.values.size(), output.values.data(),
    output.rows.data(), output.columns.data()));
}

Status statusFromOsqp(OSQPInt status) noexcept
{
  switch (status) {
    case OSQP_PRIMAL_INFEASIBLE:
    case OSQP_PRIMAL_INFEASIBLE_INACCURATE:
    case OSQP_DUAL_INFEASIBLE:
    case OSQP_DUAL_INFEASIBLE_INACCURATE:
      return Status::infeasible_bounds;
    case OSQP_TIME_LIMIT_REACHED:
      return Status::deadline_exceeded;
    case OSQP_MAX_ITER_REACHED:
      return Status::max_iterations;
    case OSQP_NON_CVX:
      return Status::factorization_failure;
    default:
      return Status::invalid_input;
  }
}

}  // namespace

class Solver::Impl
{
public:
  explicit Impl(const Configuration &config)
  : config_(config), problem_(config),
    gradient_(kDecisionCount, 0.0), lower_(kConstraintCount, -1.0),
    upper_(kConstraintCount, 1.0), dual_(kConstraintCount, 0.0),
    warm_start_(DecisionVector::Zero(kDecisionCount))
  {
    if (!problem_.valid()) return;
    makeCsc(problem_.hessian(), true, p_);
    makeCsc(problem_.constraints(), false, a_);
    settings_.reset(OSQPSettings_new());
    if (!p_.matrix || !a_.matrix || !settings_) return;
    settings_->verbose = 0;
    settings_->warm_starting = 1;
    settings_->polishing = 0;
    settings_->max_iter = config_.max_iterations;
    settings_->rho = config_.admm_rho;
    settings_->eps_abs = config_.absolute_tolerance;
    settings_->eps_rel = config_.relative_tolerance;
    // The coupled problem has many polygon rows. Checking residuals every
    // iteration costs more than several ADMM updates; five keeps the deadline
    // observable without paying that full reduction on every step.
    settings_->check_termination = 5;
    settings_->adaptive_rho = OSQP_ADAPTIVE_RHO_UPDATE_ITERATIONS;
    settings_->adaptive_rho_interval = 25;
    OSQPSolver *solver = nullptr;
    const OSQPInt error = osqp_setup(
      &solver, p_.matrix.get(), gradient_.data(), a_.matrix.get(), lower_.data(),
      upper_.data(), kConstraintCount, kDecisionCount, settings_.get());
    solver_.reset(solver);
    configured_ = error == 0 && solver_;
  }

  bool configured() const noexcept {return configured_;}

  void reset() noexcept
  {
    warm_start_.setZero();
    warm_start_valid_ = false;
    if (solver_) osqp_cold_start(solver_.get());
  }

  Result solve(
    const State &initial_state, const Reference &reference,
    const Input &last_control, Clock::time_point deadline) noexcept
  {
    Result output;
    if (!configured_ || Clock::now() >= deadline) {
      output.status = configured_ ? Status::deadline_exceeded : Status::invalid_input;
      return output;
    }
    ProblemData data;
    if (!problem_.update(initial_state, reference, last_control, data)) return output;
    std::copy_n(data.gradient.data(), kDecisionCount, gradient_.begin());
    std::copy_n(data.lower.data(), kConstraintCount, lower_.begin());
    std::copy_n(data.upper.data(), kConstraintCount, upper_.begin());
    if (osqp_update_data_vec(
        solver_.get(), gradient_.data(), lower_.data(), upper_.data()) != 0) {
      output.status = Status::factorization_failure;
      return output;
    }
    const double remaining = std::chrono::duration<double>(deadline - Clock::now()).count();
    if (!std::isfinite(remaining) || remaining <= 0.0) {
      output.status = Status::deadline_exceeded;
      return output;
    }
    settings_->time_limit = std::max(remaining, 1.0e-6);
    if (osqp_update_settings(solver_.get(), settings_.get()) != 0) {
      output.status = Status::factorization_failure;
      return output;
    }

    if (warm_start_valid_) {
      // Receding-horizon warm start: U[k] <- U[k+1]. The constraint rows are
      // step-major, so the ADMM multipliers use the same one-step shift.
      warm_start_.segment(0, kControlCount - 3) =
        warm_start_.segment(3, kControlCount - 3).eval();
      warm_start_.segment(kVelocitySlackOffset, kHorizonLength - 1) =
        warm_start_.segment(kVelocitySlackOffset + 1, kHorizonLength - 1).eval();
      warm_start_.segment(kAccelerationSlackOffset, kHorizonLength - 1) =
        warm_start_.segment(kAccelerationSlackOffset + 1, kHorizonLength - 1).eval();
      Eigen::Map<Eigen::Matrix<OSQPFloat, Eigen::Dynamic, 1>> dual_warm_start(
        dual_.data(), kConstraintCount);
      dual_warm_start.head(kConstraintCount - kRowsPerStep) =
        dual_warm_start.tail(kConstraintCount - kRowsPerStep).eval();
    } else {
      for (std::size_t step = 0; step < kHorizonLength; ++step) {
        warm_start_.segment<3>(3 * step) = last_control;
      }
      warm_start_.tail(2 * kHorizonLength).setZero();
      std::fill(dual_.begin(), dual_.end(), 0.0);
    }
    if (osqp_warm_start(solver_.get(), warm_start_.data(), dual_.data()) != 0
      || osqp_solve(solver_.get()) != 0 || !solver_->info) {
      output.status = Status::factorization_failure;
      reset();
      return output;
    }
    output.iterations = solver_->info->iter;
    output.primal_residual = solver_->info->prim_res;
    output.dual_residual = solver_->info->dual_res;
    const OSQPInt status = solver_->info->status_val;
    if (status != OSQP_SOLVED && status != OSQP_SOLVED_INACCURATE) {
      output.status = statusFromOsqp(status);
      reset();
      return output;
    }
    if (!solver_->solution || !solver_->solution->x || !solver_->solution->y) {
      output.status = Status::non_finite_output;
      reset();
      return output;
    }
    for (std::size_t index = 0; index < kDecisionCount; ++index) {
      warm_start_(index) = solver_->solution->x[index];
    }
    std::copy_n(solver_->solution->y, kConstraintCount, dual_.begin());
    if (!warm_start_.allFinite()) {
      output.status = Status::non_finite_output;
      reset();
      return output;
    }

    output = problem_.result(
      data, warm_start_, config_.absolute_tolerance, config_.relative_tolerance);
    output.iterations = solver_->info->iter;
    output.primal_residual = solver_->info->prim_res;
    output.dual_residual = solver_->info->dual_res;
    const ConstraintVector constraint_value = problem_.constraints() * warm_start_;
    const ConstraintVector projected = constraint_value.cwiseMax(data.lower).cwiseMin(data.upper);
    output.primal_tolerance = config_.absolute_tolerance
      + config_.relative_tolerance * std::max(
      constraint_value.template lpNorm<Eigen::Infinity>(),
      projected.template lpNorm<Eigen::Infinity>());
    Eigen::Map<const ConstraintVector> dual(solver_->solution->y, kConstraintCount);
    output.dual_tolerance = config_.absolute_tolerance
      + config_.relative_tolerance
      * (problem_.constraints().transpose() * dual).template lpNorm<Eigen::Infinity>();
    if (!output.valid) {
      reset();
      return output;
    }
    warm_start_valid_ = true;
    return output;
  }

private:
  Configuration config_;
  Problem problem_;
  CscStorage p_;
  CscStorage a_;
  std::unique_ptr<OSQPSettings, decltype(&OSQPSettings_free)> settings_{
    nullptr, &OSQPSettings_free};
  std::unique_ptr<OSQPSolver, decltype(&osqp_cleanup)> solver_{
    nullptr, &osqp_cleanup};
  std::vector<OSQPFloat> gradient_;
  std::vector<OSQPFloat> lower_;
  std::vector<OSQPFloat> upper_;
  std::vector<OSQPFloat> dual_;
  DecisionVector warm_start_;
  bool configured_ = false;
  bool warm_start_valid_ = false;
};

Solver::Solver(const Configuration &configuration)
: impl_(std::make_unique<Impl>(configuration))
{
}

Solver::~Solver() = default;

bool Solver::configured() const noexcept
{
  return impl_ && impl_->configured();
}

void Solver::reset() noexcept
{
  if (impl_) impl_->reset();
}

Result Solver::solve(
  const State &initial_state, const Reference &reference,
  const Input &last_control, Clock::time_point deadline) noexcept
{
  return impl_ ? impl_->solve(initial_state, reference, last_control, deadline) : Result{};
}

}  // namespace mpc_controller::coupled_mpc
