#include "mpc_controller/ports/solver.hpp"

#include <Eigen/Core>

#include <hpipm_common.h>
#include <hpipm_d_ocp_qp.h>
#include <hpipm_d_ocp_qp_dim.h>
#include <hpipm_d_ocp_qp_ipm.h>
#include <hpipm_d_ocp_qp_sol.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <ctime>
#include <limits>
#include <memory>
#include <numeric>
#include <vector>

namespace mpc_controller::coupled_mpc
{
namespace
{

constexpr std::size_t kPhysicalStateDimension = 9;
constexpr std::size_t kPreviousInputOffset = kPhysicalStateDimension;
constexpr std::size_t kAugmentedStateDimension = 12;
constexpr std::size_t kStageCount = kHorizonLength;
constexpr double kLimitToleranceMultiplier = 50.0;
constexpr double kInfinity = 1.0e19;
constexpr std::size_t kOppositePairCount = kPolygonSides / 2U;
constexpr std::size_t kRowControlXyBase = 0U;
constexpr std::size_t kRowRateXyBase = kRateRowLocalBegin;
constexpr std::size_t kRowRateUz = kPolygonSides;
constexpr std::size_t kRowVelXyBase = kVelocityRowLocalBegin;
constexpr std::size_t kRowVelZUpper = 2U * kPolygonSides + 1U;
constexpr std::size_t kRowVelZLower = 2U * kPolygonSides + 2U;
constexpr std::size_t kRowTiltBase = 2U * kPolygonSides + 3U;
constexpr std::size_t kRowThrust = 3U * kPolygonSides + 3U;

// OCP stages own their constraint rows; unlike the condensed backend there is
// no global row offset to apply here.
constexpr std::size_t rowIndex(
  std::size_t /*stage*/, std::size_t local_row) noexcept
{
  return local_row;
}

using PhysicalState = Eigen::Matrix<double, kPhysicalStateDimension, 1>;
using AugmentedState = Eigen::Matrix<double, kAugmentedStateDimension, 1>;
using AugmentedTransition = Eigen::Matrix<double, kAugmentedStateDimension, kAugmentedStateDimension>;
using AugmentedInput = Eigen::Matrix<double, kAugmentedStateDimension, kInputDimension>;

bool threadCpuTimeSeconds(double &seconds) noexcept
{
  timespec value{};
  if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &value) != 0) return false;
  seconds = static_cast<double>(value.tv_sec) + static_cast<double>(value.tv_nsec) * 1.0e-9;
  return true;
}

double elapsedThreadCpuSeconds(double start, bool valid) noexcept
{
  double end = 0.0;
  if (!valid || !threadCpuTimeSeconds(end)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return std::max(0.0, end - start);
}

bool finiteWeights(const std::array<double, 3> &values) noexcept
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
    config.min_collective_specific_force_m_s2,
    config.max_collective_specific_force_m_s2};
  if (!std::all_of(positive_values.begin(), positive_values.end(), [](double value) {
      return std::isfinite(value) && value > 0.0;
    })) {
    return false;
  }
  return std::all_of(
    config.model_time_constant_xyz.begin(), config.model_time_constant_xyz.end(),
    [](double value) {return std::isfinite(value) && value >= 0.0;})
    && finiteWeights(config.control_weights)
    && finiteWeights(config.control_rate_weights)
    && finiteWeights(config.stage_weights_xy)
    && finiteWeights(config.terminal_weights_xy)
    && finiteWeights(config.stage_weights_z)
    && finiteWeights(config.terminal_weights_z)
    && config.max_iterations > 0
    && config.max_tilt_rad < 0.5 * M_PI
    && config.min_collective_specific_force_m_s2 < config.max_collective_specific_force_m_s2
    && config.max_control_xy < config.max_collective_specific_force_m_s2;
}

Eigen::Matrix<double, kPhysicalStateDimension, kPhysicalStateDimension> transitionMatrix(
  double dt, const Eigen::Vector3d &alpha) noexcept
{
  Eigen::Matrix<double, kPhysicalStateDimension, kPhysicalStateDimension> matrix =
    Eigen::Matrix<double, kPhysicalStateDimension, kPhysicalStateDimension>::Zero();
  matrix.template block<3, 3>(0, 0).setIdentity();
  matrix.template block<3, 3>(0, 3) = dt * Eigen::Matrix3d::Identity();
  matrix.template block<3, 3>(0, 6) = (0.5 * dt * dt * alpha).asDiagonal();
  matrix.template block<3, 3>(3, 3).setIdentity();
  matrix.template block<3, 3>(3, 6) = (dt * alpha).asDiagonal();
  matrix.template block<3, 3>(6, 6) = alpha.asDiagonal();
  return matrix;
}

Eigen::Matrix<double, kPhysicalStateDimension, kInputDimension> inputMatrix(
  double dt, const Eigen::Vector3d &alpha) noexcept
{
  Eigen::Matrix<double, kPhysicalStateDimension, kInputDimension> matrix =
    Eigen::Matrix<double, kPhysicalStateDimension, kInputDimension>::Zero();
  const Eigen::Vector3d response = Eigen::Vector3d::Ones() - alpha;
  matrix.template block<3, 3>(0, 0) =
    (0.5 * dt * dt * response).asDiagonal();
  matrix.template block<3, 3>(3, 0) = (dt * response).asDiagonal();
  matrix.template block<3, 3>(6, 0) = response.asDiagonal();
  return matrix;
}

template<typename Matrix>
std::vector<double> vectorize(const Matrix &matrix)
{
  return std::vector<double>(matrix.data(), matrix.data() + matrix.size());
}

struct StageData
{
  std::vector<double> A;
  std::vector<double> B;
  std::vector<double> b;
  std::vector<double> Q;
  std::vector<double> S;
  std::vector<double> R;
  std::vector<double> q;
  std::vector<double> r;
  std::vector<double> C;
  std::vector<double> D;
  std::vector<double> lower;
  std::vector<double> upper;
  std::vector<double> lower_mask;
  std::vector<double> upper_mask;
  std::vector<double> state_lower;
  std::vector<double> state_upper;
  std::vector<double> state_lower_mask;
  std::vector<double> state_upper_mask;
  std::vector<int> state_indices;
  std::vector<int> state_equalities;
};

class Problem final
{
public:
  explicit Problem(const Configuration &config)
  : config_(config), stages_(kHorizonLength + 1U)
  {
    if (!validConfiguration(config_)) return;
    buildDynamics();
    buildCost();
    buildConstraints();
    valid_ = true;
  }

  bool valid() const noexcept {return valid_;}

  const std::vector<StageData> &stages() const noexcept {return stages_;}

  bool update(
    const State &initial, const Reference &reference, const Input &last_control,
    const Limits &requested_limits) noexcept
  {
    if (!valid_ || !initial.allFinite() || !last_control.allFinite()
      || !std::all_of(reference.begin(), reference.end(), [](const State &state) {
        return state.allFinite();
      })) {
      return false;
    }
    const Limits limits = effectiveLimits(requested_limits);
    const PhysicalState measured = initial;
    AugmentedState initial_augmented = AugmentedState::Zero();
    initial_augmented.head<9>() = measured;
    initial_augmented.segment<3>(kPreviousInputOffset) = last_control;
    stages_[0].state_lower = vectorize(initial_augmented);
    stages_[0].state_upper = stages_[0].state_lower;

    double cumulative_rate_xy = 0.0;
    double cumulative_rate_z = 0.0;
    for (std::size_t stage = 0; stage < kStageCount; ++stage) {
      const std::size_t step = stage;
      const double dt = step == 0U ? config_.dt_first : config_.dt_later;
      const double polygon_scale = std::cos(
        M_PI / static_cast<double>(kPolygonSides));
      const double velocity_xy_limit = limits.max_speed_xy * polygon_scale;
      const double configured_control_xy_limit =
        config_.max_control_xy * polygon_scale;
      const double control_xy_limit = std::min(
        config_.max_control_xy, limits.max_acceleration_xy) * polygon_scale;
      const double configured_control_z_limit = config_.max_control_z;
      const double control_z_limit = std::min(
        config_.max_control_z, limits.max_acceleration_z);
      const double tilt_scale =
        std::tan(config_.max_tilt_rad) * polygon_scale;
      const double maximum_force_z = std::sqrt(
        config_.max_collective_specific_force_m_s2
        * config_.max_collective_specific_force_m_s2
        - config_.max_control_xy * config_.max_control_xy);
      const double previous_xy_support = xyPolygonSupport(last_control);
      const double previous_z_magnitude = std::abs(last_control.z());
      cumulative_rate_xy += limits.max_control_rate_xy * dt * polygon_scale;
      cumulative_rate_z += limits.max_control_rate_z * dt;
      const double stage_control_xy_limit = std::min(
        configured_control_xy_limit,
        std::max(control_xy_limit, previous_xy_support - cumulative_rate_xy));
      const double stage_control_z_limit = std::min(
        configured_control_z_limit,
        std::max(control_z_limit, previous_z_magnitude - cumulative_rate_z));

      auto &stage_data = stages_[stage];
      stage_data.lower.assign(kFullConstraintRowsPerStage, -kInfinity);
      stage_data.upper.assign(kFullConstraintRowsPerStage, kInfinity);
      stage_data.lower_mask.assign(kFullConstraintRowsPerStage, 0.0);
      stage_data.upper_mask.assign(kFullConstraintRowsPerStage, 0.0);
      const auto setBounds = [&](std::size_t local, double lower, double upper) {
          stage_data.lower[local] = lower;
          stage_data.upper[local] = upper;
          stage_data.lower_mask[local] = lower > -kInfinity ? 1.0 : 0.0;
          stage_data.upper_mask[local] = upper < kInfinity ? 1.0 : 0.0;
        };

      for (std::size_t side = 0; side < kOppositePairCount; ++side) {
        const double rate_change = limits.max_control_rate_xy * dt * polygon_scale;
        setBounds(rowIndex(step, kRowControlXyBase + side),
          -stage_control_xy_limit, stage_control_xy_limit);
        setBounds(rowIndex(step, kRowRateXyBase + side), -rate_change, rate_change);
      }
      setBounds(rowIndex(step, kRowRateUz),
        -limits.max_control_rate_z * dt, limits.max_control_rate_z * dt);

      for (std::size_t side = 0; side < kPolygonSides; ++side) {
        setBounds(rowIndex(step, kRowVelXyBase + side),
          -kInfinity, velocity_xy_limit);
        setBounds(rowIndex(step, kRowTiltBase + side),
          -kInfinity, tilt_scale * config_.gravity_m_s2);
      }
      setBounds(rowIndex(step, kRowVelZUpper), -kInfinity, limits.max_speed_z);
      setBounds(rowIndex(step, kRowVelZLower), -kInfinity, limits.max_speed_z);
      setBounds(rowIndex(step, kRowThrust),
        std::max(-stage_control_z_limit,
          config_.min_collective_specific_force_m_s2 - config_.gravity_m_s2),
        std::min(stage_control_z_limit, maximum_force_z - config_.gravity_m_s2));

      const bool terminal = step + 1U == kStageCount;
      const std::size_t cost_stage = stage + 1U;
      const auto &weights_xy = terminal
        ? config_.terminal_weights_xy : config_.stage_weights_xy;
      const auto &weights_z = terminal
        ? config_.terminal_weights_z : config_.stage_weights_z;
      const double stage_scale = terminal ? 1.0 : dt / config_.dt_later;
      if (cost_stage <= kHorizonLength) {
        auto &cost = stages_[cost_stage];
        cost.q.assign(kAugmentedStateDimension, 0.0);
        for (std::size_t derivative = 0; derivative < 3U; ++derivative) {
          const double xy_weight = stage_scale * weights_xy[derivative];
          const double z_weight = stage_scale * weights_z[derivative];
          cost.q[3U * derivative] = -2.0 * xy_weight * reference[step](3U * derivative);
          cost.q[3U * derivative + 1U] =
            -2.0 * xy_weight * reference[step](3U * derivative + 1U);
          cost.q[3U * derivative + 2U] =
            -2.0 * z_weight * reference[step](3U * derivative + 2U);
        }
      }
    }
    return true;
  }

  Limits effectiveLimits(const Limits &requested) const noexcept
  {
    return {boundedLimit(requested.max_speed_xy, config_.max_speed_xy),
            boundedLimit(requested.max_speed_z, config_.max_speed_z),
            boundedLimit(requested.max_acceleration_xy,
                         config_.max_acceleration_xy),
            boundedLimit(requested.max_acceleration_z,
                         config_.max_acceleration_z),
            boundedLimit(requested.max_control_rate_xy,
                         config_.max_control_rate_xy),
            boundedLimit(requested.max_control_rate_z,
                         config_.max_control_rate_z)};
  }

  static double boundedLimit(double requested, double maximum) noexcept
  {
    return std::isfinite(requested) && requested > 0.0
      ? std::min(requested, maximum) : maximum;
  }

  double maxConstraintViolation(
    const std::array<Input, kHorizonLength> &controls,
    const std::array<PhysicalState, kHorizonLength> &predictions,
    const State &initial, const Input &last_control) const noexcept
  {
    double maximum = 0.0;
    AugmentedState state = AugmentedState::Zero();
    state.head<9>() = initial;
    state.segment<3>(kPreviousInputOffset) = last_control;
    for (std::size_t step = 0; step < kStageCount; ++step) {
      const auto &stage = stages_[step];
      const Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
        Eigen::ColMajor>> C(stage.C.data(), stageCount(stage), kAugmentedStateDimension);
      const Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
        Eigen::ColMajor>> D(stage.D.data(), stageCount(stage), kInputDimension);
      const Eigen::Map<const Eigen::VectorXd> lower(stage.lower.data(), stageCount(stage));
      const Eigen::Map<const Eigen::VectorXd> upper(stage.upper.data(), stageCount(stage));
      const Eigen::VectorXd value = C * state + D * controls[step];
      for (Eigen::Index row = 0; row < value.size(); ++row) {
        if (lower(row) > -kInfinity) maximum = std::max(maximum, lower(row) - value(row));
        if (upper(row) < kInfinity) maximum = std::max(maximum, value(row) - upper(row));
      }
      state.head<9>() = predictions[step];
      state.segment<3>(kPreviousInputOffset) = controls[step];
    }
    return std::max(0.0, maximum);
  }

  static std::size_t stageCount(const StageData &stage) noexcept
  {
    return stage.lower.size();
  }

private:
  static double xyPolygonSupport(const Input &input) noexcept
  {
    double support = 0.0;
    for (std::size_t side = 0; side < kOppositePairCount; ++side) {
      const double angle = 2.0 * M_PI * static_cast<double>(side)
        / static_cast<double>(kPolygonSides);
      support = std::max(support,
        std::abs(std::cos(angle) * input.x() + std::sin(angle) * input.y()));
    }
    return support;
  }

  void buildDynamics()
  {
    for (std::size_t stage = 0; stage < kStageCount; ++stage) {
      const double dt = stage == 0U ? config_.dt_first : config_.dt_later;
      Eigen::Vector3d alpha;
      for (std::size_t axis = 0; axis < 3U; ++axis) {
        alpha(static_cast<Eigen::Index>(axis)) = config_.model_time_constant_xyz[axis] == 0.0
          ? 0.0 : std::exp(-dt / config_.model_time_constant_xyz[axis]);
      }
      const auto physical_a = transitionMatrix(dt, alpha);
      const auto physical_b = inputMatrix(dt, alpha);
      AugmentedTransition augmented_a = AugmentedTransition::Zero();
      augmented_a.topLeftCorner<9, 9>() = physical_a;
      AugmentedInput augmented_b = AugmentedInput::Zero();
      augmented_b.topRows<9>() = physical_b;
      augmented_b.bottomRows<3>().setIdentity();
      stages_[stage].A = vectorize(augmented_a);
      stages_[stage].B = vectorize(augmented_b);
      stages_[stage].b.assign(kAugmentedStateDimension, 0.0);
    }
  }

  void buildCost()
  {
    for (std::size_t stage = 0; stage <= kHorizonLength; ++stage) {
      auto &cost = stages_[stage];
      Eigen::Matrix<double, kAugmentedStateDimension, kAugmentedStateDimension> Q =
        Eigen::Matrix<double, kAugmentedStateDimension, kAugmentedStateDimension>::Zero();
      Eigen::Matrix<double, kInputDimension, kAugmentedStateDimension> S =
        Eigen::Matrix<double, kInputDimension, kAugmentedStateDimension>::Zero();
      Eigen::Matrix<double, kInputDimension, kInputDimension> R =
        Eigen::Matrix<double, kInputDimension, kInputDimension>::Zero();
      const bool terminal = stage == kHorizonLength;
      for (std::size_t axis = 0; axis < 3U; ++axis) {
        const double rate = config_.control_rate_weights[axis];
        if (!terminal) {
          Q(kPreviousInputOffset + axis, kPreviousInputOffset + axis) = 2.0 * rate;
          S(axis, kPreviousInputOffset + axis) = -2.0 * rate;
          R(axis, axis) = 2.0 * (config_.control_weights[axis] + rate);
        }
      }
      if (stage > 0U) {
        const bool last_state = terminal;
        const auto &xy = last_state
          ? config_.terminal_weights_xy : config_.stage_weights_xy;
        const auto &z = last_state
          ? config_.terminal_weights_z : config_.stage_weights_z;
        const double dt = stage == 1U ? config_.dt_first : config_.dt_later;
        const double scale = last_state ? 1.0 : dt / config_.dt_later;
        for (std::size_t derivative = 0; derivative < 3U; ++derivative) {
          Q(3U * derivative, 3U * derivative) = 2.0 * scale * xy[derivative];
          Q(3U * derivative + 1U, 3U * derivative + 1U) =
            2.0 * scale * xy[derivative];
          Q(3U * derivative + 2U, 3U * derivative + 2U) =
            2.0 * scale * z[derivative];
        }
      }
      cost.Q = vectorize(Q);
      cost.S = vectorize(S);
      cost.R = vectorize(R);
      cost.q.assign(kAugmentedStateDimension, 0.0);
      cost.r.assign(kInputDimension, 0.0);
    }
  }

  void buildConstraints()
  {
    const double polygon_scale = std::cos(M_PI / static_cast<double>(kPolygonSides));
    const double tilt_scale = std::tan(config_.max_tilt_rad) * polygon_scale;
    for (std::size_t stage = 0; stage < kStageCount; ++stage) {
      auto &data = stages_[stage];
      data.C.assign(kFullConstraintRowsPerStage * kAugmentedStateDimension, 0.0);
      data.D.assign(kFullConstraintRowsPerStage * kInputDimension, 0.0);
      const auto setC = [&](std::size_t row, std::size_t column, double value) {
          data.C[column * kFullConstraintRowsPerStage + row] = value;
        };
      const auto setD = [&](std::size_t row, std::size_t column, double value) {
          data.D[column * kFullConstraintRowsPerStage + row] = value;
        };
      const auto &A = stages_[stage].A;
      const auto &B = stages_[stage].B;
      const auto a = [&](std::size_t row, std::size_t column) {
          return A[column * kAugmentedStateDimension + row];
        };
      const auto b = [&](std::size_t row, std::size_t column) {
          return B[column * kAugmentedStateDimension + row];
        };

      for (std::size_t side = 0; side < kOppositePairCount; ++side) {
        const double angle = 2.0 * M_PI * static_cast<double>(side)
          / static_cast<double>(kPolygonSides);
        const double nx = std::cos(angle);
        const double ny = std::sin(angle);
        setD(rowIndex(stage, kRowControlXyBase + side), 0, nx);
        setD(rowIndex(stage, kRowControlXyBase + side), 1, ny);
        setC(rowIndex(stage, kRowRateXyBase + side), 9, -nx);
        setC(rowIndex(stage, kRowRateXyBase + side), 10, -ny);
        setD(rowIndex(stage, kRowRateXyBase + side), 0, nx);
        setD(rowIndex(stage, kRowRateXyBase + side), 1, ny);
      }
      setC(rowIndex(stage, kRowRateUz), 11, -1.0);
      setD(rowIndex(stage, kRowRateUz), 2, 1.0);

      for (std::size_t side = 0; side < kPolygonSides; ++side) {
        const double angle = 2.0 * M_PI * static_cast<double>(side)
          / static_cast<double>(kPolygonSides);
        const double nx = std::cos(angle);
        const double ny = std::sin(angle);
        for (std::size_t column = 0; column < kAugmentedStateDimension; ++column) {
          setC(rowIndex(stage, kRowVelXyBase + side), column,
            nx * a(3, column) + ny * a(4, column));
        }
        for (std::size_t column = 0; column < kInputDimension; ++column) {
          setD(rowIndex(stage, kRowVelXyBase + side), column,
            nx * b(3, column) + ny * b(4, column));
        }
        for (std::size_t column = 0; column < kAugmentedStateDimension; ++column) {
          setC(rowIndex(stage, kRowTiltBase + side), column, 0.0);
        }
        setD(rowIndex(stage, kRowTiltBase + side), 0, nx);
        setD(rowIndex(stage, kRowTiltBase + side), 1, ny);
        setD(rowIndex(stage, kRowTiltBase + side), 2, -tilt_scale);
      }
      for (std::size_t column = 0; column < kAugmentedStateDimension; ++column) {
        setC(rowIndex(stage, kRowVelZUpper), column, a(5, column));
        setC(rowIndex(stage, kRowVelZLower), column, -a(5, column));
      }
      for (std::size_t column = 0; column < kInputDimension; ++column) {
        setD(rowIndex(stage, kRowVelZUpper), column, b(5, column));
        setD(rowIndex(stage, kRowVelZLower), column, -b(5, column));
      }
      setD(rowIndex(stage, kRowThrust), 2, 1.0);
    }
    stages_[kHorizonLength].C.clear();
    stages_[kHorizonLength].D.clear();
  }

  Configuration config_;
  std::vector<StageData> stages_;
  bool valid_ = false;
};

}  // namespace

class Solver::Impl
{
public:
  explicit Impl(const Configuration &config)
  : config_(config), problem_(config)
  {
    if (!problem_.valid()) return;
    createHpipm();
  }

  bool configured() const noexcept {return configured_;}

  void reset() noexcept
  {
    warm_start_valid_ = false;
  }

  Result solve(
    const State &initial, const Reference &reference, const Input &last_control,
    const Limits &limits, Clock::time_point deadline) noexcept
  {
    Result output;
    double cpu_start = 0.0;
    const bool cpu_valid = threadCpuTimeSeconds(cpu_start);
    const auto finish = [&]() noexcept {
        output.coupled_solve_cpu_seconds = elapsedThreadCpuSeconds(cpu_start, cpu_valid);
        return output;
      };
    if (!configured_ || Clock::now() >= deadline) {
      output.status = configured_ ? Status::deadline_exceeded : Status::invalid_input;
      return finish();
    }
    const auto problem_start = Clock::now();
    if (!problem_.update(initial, reference, last_control, limits)) {
      output.problem_update_seconds = std::chrono::duration<double>(
        Clock::now() - problem_start).count();
      return finish();
    }
    output.problem_update_seconds = std::chrono::duration<double>(
      Clock::now() - problem_start).count();
    updateHpipmProblem();
    output.remaining_budget_seconds =
      std::chrono::duration<double>(deadline - Clock::now()).count();

    const auto warm_start_begin = Clock::now();
    if (!warm_start_valid_) {
      initializeColdStart();
    } else {
      shiftWarmStart(initial, last_control);
    }
    output.warm_start_prepare_seconds = std::chrono::duration<double>(
      Clock::now() - warm_start_begin).count();
    output.warm_start_seconds = output.warm_start_prepare_seconds;

    const auto solve_start = Clock::now();
    double solve_cpu_start = 0.0;
    const bool solve_cpu_valid = threadCpuTimeSeconds(solve_cpu_start);
    int hpipm_status = MAX_ITER;
    d_ocp_qp_ipm_solve(&qp_, &solution_, &arg_, &workspace_);
    d_ocp_qp_ipm_get_status(&workspace_, &hpipm_status);
    d_ocp_qp_ipm_get_iter(&workspace_, &output.iterations);
    double max_stationarity = std::numeric_limits<double>::infinity();
    double max_equality = std::numeric_limits<double>::infinity();
    double max_inequality = std::numeric_limits<double>::infinity();
    double max_complementarity = std::numeric_limits<double>::infinity();
    d_ocp_qp_ipm_get_max_res_stat(&workspace_, &max_stationarity);
    d_ocp_qp_ipm_get_max_res_eq(&workspace_, &max_equality);
    d_ocp_qp_ipm_get_max_res_ineq(&workspace_, &max_inequality);
    d_ocp_qp_ipm_get_max_res_comp(&workspace_, &max_complementarity);
    (void)max_complementarity;
    output.dual_residual = max_stationarity;
    output.primal_residual = std::max(max_equality, max_inequality);
    // HPIPM uses absolute tolerances for stationarity/equality and relative
    // tolerances for inequality/complementarity.  The aggregate fields are
    // intentionally conservative because the ROS message predates HPIPM and
    // exposes only one primal and one dual tolerance.
    output.primal_tolerance = std::min(config_.absolute_tolerance,
      config_.relative_tolerance);
    output.dual_tolerance = config_.absolute_tolerance;
    output.solve_seconds = std::chrono::duration<double>(
      Clock::now() - solve_start).count();
    output.solve_cpu_seconds = elapsedThreadCpuSeconds(
      solve_cpu_start, solve_cpu_valid);
    output.coupled_solve_cpu_seconds = output.solve_cpu_seconds;
    output.result_postprocess_seconds = 0.0;

    if (Clock::now() > deadline) {
      output.status = Status::deadline_exceeded;
      warm_start_valid_ = false;
      return finish();
    }
    if (hpipm_status != SUCCESS) {
      output.status = hpipm_status == MAX_ITER
        ? Status::max_iterations : Status::factorization_failure;
      warm_start_valid_ = false;
      return finish();
    }

    std::array<Input, kHorizonLength> controls{};
    std::array<PhysicalState, kHorizonLength> predictions{};
    std::array<double, kAugmentedStateDimension> state_buffer{};
    for (std::size_t stage = 0; stage < kHorizonLength; ++stage) {
      std::array<double, kInputDimension> input_buffer{};
      d_ocp_qp_sol_get_u(static_cast<int>(stage), &solution_, input_buffer.data());
      controls[stage] = Input(input_buffer[0], input_buffer[1], input_buffer[2]);
      d_ocp_qp_sol_get_x(static_cast<int>(stage + 1U), &solution_, state_buffer.data());
      for (std::size_t index = 0; index < kPhysicalStateDimension; ++index) {
        predictions[stage](static_cast<Eigen::Index>(index)) = state_buffer[index];
      }
      output.prediction[stage] = predictions[stage];
      output.max_predicted_speed_xy = std::max(
        output.max_predicted_speed_xy, predictions[stage].segment<2>(3).norm());
      output.max_predicted_acceleration_xy = std::max(
        output.max_predicted_acceleration_xy, predictions[stage].segment<2>(6).norm());
      const Eigen::Vector3d force(
        controls[stage].x(), controls[stage].y(), controls[stage].z() + config_.gravity_m_s2);
      if (!force.allFinite() || force.z() <= 0.0) {
        output.status = Status::non_finite_output;
        warm_start_valid_ = false;
        return finish();
      }
      output.max_predicted_tilt_rad = std::max(
        output.max_predicted_tilt_rad,
        std::atan2(force.head<2>().norm(), force.z()));
      output.max_predicted_collective_specific_force_m_s2 = std::max(
        output.max_predicted_collective_specific_force_m_s2, force.norm());
    }
    output.first_control = controls[0];
    output.max_constraint_violation = problem_.maxConstraintViolation(
      controls, predictions, initial, last_control);
    const double tolerance = kLimitToleranceMultiplier * (
      config_.absolute_tolerance + config_.relative_tolerance);
    output.valid = output.first_control.allFinite()
      && output.max_constraint_violation <= tolerance
      && output.max_predicted_tilt_rad <= config_.max_tilt_rad + tolerance
      && output.max_predicted_collective_specific_force_m_s2
        <= config_.max_collective_specific_force_m_s2 + tolerance;
    output.status = output.valid ? Status::success : Status::output_limit;
    warm_start_valid_ = output.valid;
    return finish();
  }

private:
  void initializeColdStart() noexcept
  {
    int no_warm_start = 0;
    d_ocp_qp_ipm_arg_set_warm_start(
      &no_warm_start, &arg_);
    d_ocp_qp_init_var(&qp_, &solution_, &arg_, &workspace_);
    d_ocp_qp_ipm_arg_set_warm_start(&warm_start_mode_, &arg_);
  }

  void shiftWarmStart(const State &initial, const Input &last_control) noexcept
  {
    const auto &stages = problem_.stages();
    std::array<Input, kHorizonLength> previous_controls{};
    for (std::size_t stage = 0; stage < kHorizonLength; ++stage) {
      std::array<double, kInputDimension> input_buffer{};
      d_ocp_qp_sol_get_u(static_cast<int>(stage), &solution_, input_buffer.data());
      previous_controls[stage] = Input(
        input_buffer[0], input_buffer[1], input_buffer[2]);
    }

    AugmentedState state = AugmentedState::Zero();
    state.head<9>() = initial;
    state.segment<3>(kPreviousInputOffset) = last_control;
    d_ocp_qp_sol_set_x(0, state.data(), &solution_);
    for (std::size_t stage = 0; stage < kHorizonLength; ++stage) {
      const Input input = stage + 1U < kHorizonLength
        ? previous_controls[stage + 1U] : previous_controls[stage];
      auto &stage_data = stages[stage];
      const Eigen::Map<const AugmentedTransition> transition(stage_data.A.data());
      const Eigen::Map<const AugmentedInput> input_matrix(stage_data.B.data());
      state = transition * state + input_matrix * input;
      std::array<double, kInputDimension> input_buffer{
        input.x(), input.y(), input.z()};
      d_ocp_qp_sol_set_u(static_cast<int>(stage), input_buffer.data(), &solution_);
      d_ocp_qp_sol_set_x(static_cast<int>(stage + 1U), state.data(), &solution_);
    }
  }

  void createHpipm()
  {
    constexpr int horizon = static_cast<int>(kHorizonLength);
    const auto &stages = problem_.stages();
    dim_memory_.resize(d_ocp_qp_dim_memsize(horizon));
    d_ocp_qp_dim_create(horizon, &dim_, dim_memory_.data());
    for (int stage = 0; stage <= horizon; ++stage) {
      d_ocp_qp_dim_set_nx(stage, static_cast<int>(kAugmentedStateDimension), &dim_);
      d_ocp_qp_dim_set_nu(stage, stage < horizon ? static_cast<int>(kInputDimension) : 0, &dim_);
      d_ocp_qp_dim_set_nbx(stage, stage == 0 ? static_cast<int>(kAugmentedStateDimension) : 0, &dim_);
      d_ocp_qp_dim_set_nbu(stage, 0, &dim_);
      d_ocp_qp_dim_set_ng(stage, stage < horizon ? static_cast<int>(kFullConstraintRowsPerStage) : 0, &dim_);
      d_ocp_qp_dim_set_ns(stage, 0, &dim_);
      d_ocp_qp_dim_set_nbxe(stage, stage == 0 ? static_cast<int>(kAugmentedStateDimension) : 0, &dim_);
    }
    qp_memory_.resize(d_ocp_qp_memsize(&dim_));
    d_ocp_qp_create(&dim_, &qp_, qp_memory_.data());
    solution_memory_.resize(d_ocp_qp_sol_memsize(&dim_));
    d_ocp_qp_sol_create(&dim_, &solution_, solution_memory_.data());

    for (int stage = 0; stage < horizon; ++stage) {
      const auto &data = stages[static_cast<std::size_t>(stage)];
      d_ocp_qp_set_A(stage, const_cast<double *>(data.A.data()), &qp_);
      d_ocp_qp_set_B(stage, const_cast<double *>(data.B.data()), &qp_);
      d_ocp_qp_set_b(stage, const_cast<double *>(data.b.data()), &qp_);
      d_ocp_qp_set_Q(stage, const_cast<double *>(data.Q.data()), &qp_);
      d_ocp_qp_set_S(stage, const_cast<double *>(data.S.data()), &qp_);
      d_ocp_qp_set_R(stage, const_cast<double *>(data.R.data()), &qp_);
      d_ocp_qp_set_C(stage, const_cast<double *>(data.C.data()), &qp_);
      d_ocp_qp_set_D(stage, const_cast<double *>(data.D.data()), &qp_);
      std::vector<int> state_indices(kAugmentedStateDimension);
      std::iota(state_indices.begin(), state_indices.end(), 0);
      d_ocp_qp_set_idxbx(stage, state_indices.data(), &qp_);
      d_ocp_qp_set_idxbxe(stage, state_indices.data(), &qp_);
    }
    const auto &terminal = stages.back();
    d_ocp_qp_set_Q(horizon, const_cast<double *>(terminal.Q.data()), &qp_);

    dim_ipm_memory_.resize(d_ocp_qp_ipm_arg_memsize(&dim_));
    d_ocp_qp_ipm_arg_create(&dim_, &arg_, dim_ipm_memory_.data());
    d_ocp_qp_ipm_arg_set_default(SPEED, &arg_);
    d_ocp_qp_ipm_arg_set_iter_max(&config_.max_iterations, &arg_);
    d_ocp_qp_ipm_arg_set_warm_start(&warm_start_mode_, &arg_);
    d_ocp_qp_ipm_arg_set_pred_corr(&pred_corr_, &arg_);
    d_ocp_qp_ipm_arg_set_ric_alg(&ric_alg_, &arg_);
    d_ocp_qp_ipm_arg_set_split_step(&split_step_, &arg_);
    d_ocp_qp_ipm_arg_set_tol_stat(&config_.absolute_tolerance, &arg_);
    d_ocp_qp_ipm_arg_set_tol_eq(&config_.absolute_tolerance, &arg_);
    d_ocp_qp_ipm_arg_set_tol_ineq(&config_.relative_tolerance, &arg_);
    d_ocp_qp_ipm_arg_set_tol_comp(&config_.relative_tolerance, &arg_);
    d_ocp_qp_ipm_arg_set_comp_res_exit(&compute_residuals_, &arg_);
    workspace_memory_.resize(d_ocp_qp_ipm_ws_memsize(&dim_, &arg_));
    d_ocp_qp_ipm_ws_create(&dim_, &arg_, &workspace_, workspace_memory_.data());
    configured_ = true;
  }

  void updateHpipmProblem()
  {
    const auto &stages = problem_.stages();
    for (int stage = 0; stage < static_cast<int>(kHorizonLength); ++stage) {
      const auto &data = stages[static_cast<std::size_t>(stage)];
      d_ocp_qp_set_q(stage, const_cast<double *>(data.q.data()), &qp_);
      d_ocp_qp_set_r(stage, const_cast<double *>(data.r.data()), &qp_);
      d_ocp_qp_set_lg(stage, const_cast<double *>(data.lower.data()), &qp_);
      d_ocp_qp_set_ug(stage, const_cast<double *>(data.upper.data()), &qp_);
      d_ocp_qp_set_lg_mask(stage, const_cast<double *>(data.lower_mask.data()), &qp_);
      d_ocp_qp_set_ug_mask(stage, const_cast<double *>(data.upper_mask.data()), &qp_);
    }
    d_ocp_qp_set_q(
      static_cast<int>(kHorizonLength),
      const_cast<double *>(stages.back().q.data()), &qp_);
    const auto &initial = stages.front();
    d_ocp_qp_set_lbx(0, const_cast<double *>(initial.state_lower.data()), &qp_);
    d_ocp_qp_set_ubx(0, const_cast<double *>(initial.state_upper.data()), &qp_);
    std::vector<double> mask(kAugmentedStateDimension, 1.0);
    d_ocp_qp_set_lbx_mask(0, mask.data(), &qp_);
    d_ocp_qp_set_ubx_mask(0, mask.data(), &qp_);
  }

  Configuration config_;
  Problem problem_;
  d_ocp_qp_dim dim_{};
  d_ocp_qp qp_{};
  d_ocp_qp_sol solution_{};
  d_ocp_qp_ipm_arg arg_{};
  d_ocp_qp_ipm_ws workspace_{};
  std::vector<unsigned char> dim_memory_;
  std::vector<unsigned char> qp_memory_;
  std::vector<unsigned char> solution_memory_;
  std::vector<unsigned char> dim_ipm_memory_;
  std::vector<unsigned char> workspace_memory_;
  int warm_start_mode_ = 1;
  int pred_corr_ = 1;
  int ric_alg_ = 1;
  int split_step_ = 1;
  int compute_residuals_ = 1;
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
  const Input &last_control, const Limits &limits,
  Clock::time_point deadline) noexcept
{
  return impl_ ? impl_->solve(initial_state, reference, last_control, limits, deadline)
               : Result{};
}

}  // namespace mpc_controller::coupled_mpc
