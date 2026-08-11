#pragma once

#include <Eigen/Cholesky>
#include <Eigen/Core>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

namespace mpc_controller::axis_mpc
{

inline constexpr std::size_t kStateDimension = 3;
inline constexpr std::size_t kHorizonLength = 26;
inline constexpr std::size_t kConstraintCount = 4 * kHorizonLength;

using State = Eigen::Matrix<double, 3, 1>;
using Reference = std::array<State, kHorizonLength>;
using Prediction = std::array<State, kHorizonLength>;

struct Configuration
{
  double dt_first = 0.01;
  double dt_later = 0.20;
  double model_alpha = 0.0;
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
  int iterations = 0;
  double first_control = 0.0;
  Prediction prediction{};
};

inline bool validConfiguration(const Configuration &configuration) noexcept
{
  const auto finite_positive = [](double value) {
      return std::isfinite(value) && value > 0.0;
    };
  const auto valid_weights = [](const std::array<double, 3> &weights) {
      return std::all_of(weights.begin(), weights.end(), [](double value) {
        return std::isfinite(value) && value >= 0.0;
      });
    };
  return finite_positive(configuration.dt_first)
    && finite_positive(configuration.dt_later)
    && std::isfinite(configuration.model_alpha)
    && configuration.model_alpha >= 0.0 && configuration.model_alpha < 1.0
    && valid_weights(configuration.stage_weights)
    && valid_weights(configuration.terminal_weights)
    && finite_positive(configuration.max_speed)
    && finite_positive(configuration.max_acceleration)
    && finite_positive(configuration.max_control)
    && finite_positive(configuration.max_control_rate)
    && configuration.max_iterations > 0
    && finite_positive(configuration.admm_rho)
    && finite_positive(configuration.absolute_tolerance)
    && finite_positive(configuration.relative_tolerance);
}

inline Eigen::Matrix3d transitionMatrix(double dt, double alpha) noexcept
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

inline State inputMatrix(double dt, double alpha) noexcept
{
  const double beta = 1.0 - alpha;
  return State(0.5 * beta * dt * dt, beta * dt, beta);
}

class Solver final
{
public:
  explicit Solver(const Configuration &configuration)
  : configuration_(configuration)
  {
    configured_ = validConfiguration(configuration_);
    if (configured_) {
      configured_ = buildCondensedProblem();
    }
  }

  bool configured() const noexcept
  {
    return configured_;
  }

  void reset() noexcept
  {
    control_.setZero();
    projected_.setZero();
    scaled_dual_.setZero();
    warm_start_valid_ = false;
  }

  Result solve(const State &initial_state, const Reference &reference, double last_control) noexcept
  {
    Result result;
    if (!configured_ || !initial_state.allFinite() || !std::isfinite(last_control)) {
      return result;
    }
    for (const auto &state : reference) {
      if (!state.allFinite()) {
        return result;
      }
    }

    Eigen::Matrix<double, 3 * kHorizonLength, 1> reference_vector;
    for (std::size_t step = 0; step < kHorizonLength; ++step) {
      reference_vector.template segment<3>(static_cast<Eigen::Index>(3 * step)) = reference[step];
    }

    const auto free_prediction = state_mapping_ * initial_state;
    const auto error = free_prediction - reference_vector;
    const ControlVector gradient =
      (2.0 * control_mapping_.transpose() * weighted(error)) / objective_scale_;

    BoundsVector lower;
    BoundsVector upper;
    makeBounds(free_prediction, last_control, lower, upper);
    if (!(lower.array() <= upper.array()).all()) {
      return result;
    }

    if (!warm_start_valid_) {
      control_.setConstant(std::clamp(last_control,
        -configuration_.max_control, configuration_.max_control));
      projected_ = clamp(constraint_matrix_ * control_, lower, upper);
      scaled_dual_.setZero();
    } else {
      projected_ = clamp(projected_, lower, upper);
    }

    for (int iteration = 1; iteration <= configuration_.max_iterations; ++iteration) {
      const ControlVector right_hand_side = -gradient
        + configuration_.admm_rho * constraint_matrix_.transpose()
        * (projected_ - scaled_dual_);
      control_ = factorization_.solve(right_hand_side);
      if (factorization_.info() != Eigen::Success || !control_.allFinite()) {
        reset();
        return result;
      }

      const BoundsVector previous_projected = projected_;
      const BoundsVector constraint_value = constraint_matrix_ * control_;
      projected_ = clamp(constraint_value + scaled_dual_, lower, upper);
      scaled_dual_ += constraint_value - projected_;

      const double primal_residual =
        (constraint_value - projected_).template lpNorm<Eigen::Infinity>();
      const double dual_residual = (configuration_.admm_rho
        * constraint_matrix_.transpose() * (projected_ - previous_projected))
        .template lpNorm<Eigen::Infinity>();
      const double primal_tolerance = configuration_.absolute_tolerance
        + configuration_.relative_tolerance * std::max(
        constraint_value.template lpNorm<Eigen::Infinity>(),
        projected_.template lpNorm<Eigen::Infinity>());
      const double dual_tolerance = configuration_.absolute_tolerance
        + configuration_.relative_tolerance
        * (configuration_.admm_rho * constraint_matrix_.transpose() * scaled_dual_)
        .template lpNorm<Eigen::Infinity>();

      if (primal_residual <= primal_tolerance && dual_residual <= dual_tolerance) {
        result.iterations = iteration;
        break;
      }
    }

    if (result.iterations == 0 || !control_.allFinite()) {
      warm_start_valid_ = false;
      return result;
    }

    const auto predicted_vector = free_prediction + control_mapping_ * control_;
    if (!predicted_vector.allFinite()) {
      warm_start_valid_ = false;
      return result;
    }
    for (std::size_t step = 0; step < kHorizonLength; ++step) {
      result.prediction[step] = predicted_vector.template segment<3>(
        static_cast<Eigen::Index>(3 * step));
    }
    result.first_control = control_(0);
    result.valid = outputWithinLimits(result, last_control);
    warm_start_valid_ = result.valid;
    return result;
  }

private:
  using StateMapping = Eigen::Matrix<double, 3 * kHorizonLength, 3>;
  using ControlMapping = Eigen::Matrix<double, 3 * kHorizonLength, kHorizonLength>;
  using ControlMatrix = Eigen::Matrix<double, kHorizonLength, kHorizonLength>;
  using ControlVector = Eigen::Matrix<double, kHorizonLength, 1>;
  using ConstraintMatrix = Eigen::Matrix<double, kConstraintCount, kHorizonLength>;
  using BoundsVector = Eigen::Matrix<double, kConstraintCount, 1>;

  bool buildCondensedProblem() noexcept
  {
    state_mapping_.setZero();
    control_mapping_.setZero();
    Eigen::Matrix3d state_transition = Eigen::Matrix3d::Identity();
    Eigen::Matrix<double, 3, kHorizonLength> control_transition =
      Eigen::Matrix<double, 3, kHorizonLength>::Zero();

    for (std::size_t step = 0; step < kHorizonLength; ++step) {
      const double dt = step == 0 ? configuration_.dt_first : configuration_.dt_later;
      const Eigen::Matrix3d a = transitionMatrix(dt, configuration_.model_alpha);
      const State b = inputMatrix(dt, configuration_.model_alpha);
      state_transition = a * state_transition;
      control_transition = a * control_transition;
      control_transition.col(static_cast<Eigen::Index>(step)) += b;
      state_mapping_.template block<3, 3>(static_cast<Eigen::Index>(3 * step), 0) =
        state_transition;
      control_mapping_.template block<3, kHorizonLength>(
        static_cast<Eigen::Index>(3 * step), 0) = control_transition;
    }

    constraint_matrix_.setZero();
    for (std::size_t step = 0; step < kHorizonLength; ++step) {
      const auto index = static_cast<Eigen::Index>(step);
      constraint_matrix_(index, index) = 1.0;
      constraint_matrix_(static_cast<Eigen::Index>(kHorizonLength + step), index) = 1.0;
      if (step > 0) {
        constraint_matrix_(static_cast<Eigen::Index>(kHorizonLength + step), index - 1) = -1.0;
      }
      constraint_matrix_.row(static_cast<Eigen::Index>(2 * kHorizonLength + step)) =
        control_mapping_.row(static_cast<Eigen::Index>(3 * step + 1));
      constraint_matrix_.row(static_cast<Eigen::Index>(3 * kHorizonLength + step)) =
        control_mapping_.row(static_cast<Eigen::Index>(3 * step + 2));
    }

    for (Eigen::Index row = 0; row < constraint_matrix_.rows(); ++row) {
      const double norm = constraint_matrix_.row(row).norm();
      constraint_scale_(row) = norm > 1.0e-12 ? 1.0 / norm : 1.0;
      constraint_matrix_.row(row) *= constraint_scale_(row);
    }

    hessian_ = 2.0 * control_mapping_.transpose() * weighted(control_mapping_);
    objective_scale_ = std::max(1.0, hessian_.cwiseAbs().maxCoeff());
    hessian_ /= objective_scale_;
    const ControlMatrix system = hessian_
      + configuration_.admm_rho * constraint_matrix_.transpose() * constraint_matrix_;
    factorization_.compute(system);
    return factorization_.info() == Eigen::Success;
  }

  template<typename Derived>
  typename Derived::PlainObject weighted(const Eigen::MatrixBase<Derived> &value) const
  {
    typename Derived::PlainObject output = value.eval();
    for (std::size_t step = 0; step < kHorizonLength; ++step) {
      const auto &weights = step + 1 == kHorizonLength
        ? configuration_.terminal_weights : configuration_.stage_weights;
      for (std::size_t state = 0; state < kStateDimension; ++state) {
        output.row(static_cast<Eigen::Index>(3 * step + state)) *= weights[state];
      }
    }
    return output;
  }

  void makeBounds(
    const Eigen::Matrix<double, 3 * kHorizonLength, 1> &free_prediction,
    double last_control, BoundsVector &lower, BoundsVector &upper) const noexcept
  {
    for (std::size_t step = 0; step < kHorizonLength; ++step) {
      const auto control_row = static_cast<Eigen::Index>(step);
      lower(control_row) = -configuration_.max_control;
      upper(control_row) = configuration_.max_control;

      const auto rate_row = static_cast<Eigen::Index>(kHorizonLength + step);
      const double dt = step == 0 ? configuration_.dt_first : configuration_.dt_later;
      const double rate_delta = configuration_.max_control_rate * dt;
      lower(rate_row) = step == 0 ? last_control - rate_delta : -rate_delta;
      upper(rate_row) = step == 0 ? last_control + rate_delta : rate_delta;

      const double free_velocity = free_prediction(static_cast<Eigen::Index>(3 * step + 1));
      const auto velocity_row = static_cast<Eigen::Index>(2 * kHorizonLength + step);
      lower(velocity_row) = -configuration_.max_speed - free_velocity;
      upper(velocity_row) = configuration_.max_speed - free_velocity;

      const double free_acceleration = free_prediction(static_cast<Eigen::Index>(3 * step + 2));
      const auto acceleration_row = static_cast<Eigen::Index>(3 * kHorizonLength + step);
      lower(acceleration_row) = -configuration_.max_acceleration - free_acceleration;
      upper(acceleration_row) = configuration_.max_acceleration - free_acceleration;
    }
    lower.array() *= constraint_scale_.array();
    upper.array() *= constraint_scale_.array();
  }

  static BoundsVector clamp(
    const BoundsVector &value, const BoundsVector &lower, const BoundsVector &upper) noexcept
  {
    return value.cwiseMax(lower).cwiseMin(upper);
  }

  bool outputWithinLimits(const Result &result, double last_control) const noexcept
  {
    const double tolerance = 10.0 * (
      configuration_.absolute_tolerance + configuration_.relative_tolerance);
    if (!std::isfinite(result.first_control)
      || std::abs(result.first_control) > configuration_.max_control + tolerance
      || std::abs(result.first_control - last_control)
      > configuration_.max_control_rate * configuration_.dt_first + tolerance) {
      return false;
    }
    return std::all_of(result.prediction.begin(), result.prediction.end(), [&](const State &state) {
      return state.allFinite()
        && std::abs(state(1)) <= configuration_.max_speed + tolerance
        && std::abs(state(2)) <= configuration_.max_acceleration + tolerance;
    });
  }

  Configuration configuration_;
  bool configured_ = false;
  bool warm_start_valid_ = false;
  StateMapping state_mapping_ = StateMapping::Zero();
  ControlMapping control_mapping_ = ControlMapping::Zero();
  ConstraintMatrix constraint_matrix_ = ConstraintMatrix::Zero();
  BoundsVector constraint_scale_ = BoundsVector::Ones();
  ControlMatrix hessian_ = ControlMatrix::Zero();
  double objective_scale_ = 1.0;
  Eigen::LDLT<ControlMatrix> factorization_;
  ControlVector control_ = ControlVector::Zero();
  BoundsVector projected_ = BoundsVector::Zero();
  BoundsVector scaled_dual_ = BoundsVector::Zero();
};

}  // namespace mpc_controller::axis_mpc
