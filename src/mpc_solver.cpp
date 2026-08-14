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
    && weights(config.stage_weights) && weights(config.terminal_weights)
    && config.max_iterations > 0;
}

Eigen::Matrix3d transitionMatrix(double dt, double alpha) noexcept
{
  Eigen::Matrix3d matrix = Eigen::Matrix3d::Zero();
  matrix(0, 0) = 1.0;
  matrix(0, 1) = dt;
  matrix(0, 2) = 0.5 * dt * dt;
  matrix(1, 1) = 1.0;
  matrix(1, 2) = dt;
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
      const auto &weight = step + 1 == kHorizonLength ?
        configuration_.terminal_weights : configuration_.stage_weights;
      weights_.template segment<3>(3 * step) = State(weight[0], weight[1], weight[2]);
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

    hessian_ = 2.0 * control_mapping_.transpose()
      * weights_.asDiagonal() * control_mapping_;
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
      warm_start_(0) = warm_start_(1);
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
