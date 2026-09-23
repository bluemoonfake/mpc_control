#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

namespace mpc_controller::types
{

inline constexpr std::size_t kStateDimension = 9;
inline constexpr std::size_t kInputDimension = 3;
inline constexpr std::size_t kHorizonLength = 30;

}  // namespace mpc_controller::types

namespace mpc_controller::reference
{

inline constexpr std::size_t kAxisCount = 3;
inline constexpr std::size_t kHorizonLength = types::kHorizonLength;

using Vector3 = std::array<double, kAxisCount>;

struct TrackingLimits
{
  double max_speed_xy = 0.0;
  double max_speed_z = 0.0;
  double max_acceleration_xy = 0.0;
  double max_acceleration_z = 0.0;
  double max_control_rate_xy = 0.0;
  double max_control_rate_z = 0.0;
};

struct Point
{
  double time_from_start = 0.0;
  Vector3 position{};
  Vector3 velocity{};
  Vector3 acceleration{};
  double yaw = 0.0;
  double yaw_rate = 0.0;
};

struct Trajectory
{
  double header_time_seconds = 0.0;
  bool hold_after_end = false;
  std::vector<Point> points;
  TrackingLimits limits{};
};

struct Horizon
{
  TrackingLimits limits{};
  std::array<Point, kHorizonLength> points{};
};

inline bool finite(const Vector3 &value) noexcept
{
  return std::all_of(value.begin(), value.end(), [](double item) {
    return std::isfinite(item);
  });
}

inline bool finite(const Point &point) noexcept
{
  return std::isfinite(point.time_from_start) && point.time_from_start >= 0.0
    && finite(point.position) && finite(point.velocity) && finite(point.acceleration)
    && std::isfinite(point.yaw) && std::isfinite(point.yaw_rate);
}

inline bool finite(const TrackingLimits &limits) noexcept
{
  const std::array values{
    limits.max_speed_xy, limits.max_speed_z,
    limits.max_acceleration_xy, limits.max_acceleration_z,
    limits.max_control_rate_xy, limits.max_control_rate_z};
  return std::all_of(values.begin(), values.end(), [](double value) {
    return std::isfinite(value) && value >= 0.0;
  });
}

inline bool isValidTrajectory(const Trajectory &trajectory) noexcept
{
  if (!std::isfinite(trajectory.header_time_seconds) ||
    !finite(trajectory.limits) || trajectory.points.empty()) {
    return false;
  }
  for (std::size_t index = 0; index < trajectory.points.size(); ++index) {
    if (!finite(trajectory.points[index])) {
      return false;
    }
    if (index > 0 && trajectory.points[index].time_from_start <=
      trajectory.points[index - 1].time_from_start) {
      return false;
    }
  }
  return true;
}

class ValidatedTrajectory final
{
public:
  ValidatedTrajectory() = default;

  explicit ValidatedTrajectory(Trajectory trajectory)
  {
    assign(std::move(trajectory));
  }

  bool assign(Trajectory trajectory)
  {
    if (!isValidTrajectory(trajectory)) {
      trajectory_.reset();
      return false;
    }
    trajectory_ = std::move(trajectory);
    return true;
  }

  void reset() noexcept
  {
    trajectory_.reset();
  }

  bool valid() const noexcept
  {
    return trajectory_.has_value();
  }

  const Trajectory &get() const noexcept
  {
    return *trajectory_;
  }

private:
  std::optional<Trajectory> trajectory_;
};

class TrajectoryResampler final
{
public:
  static double shortestAngle(double from, double to) noexcept
  {
    return std::atan2(std::sin(to - from), std::cos(to - from));
  }

  static bool validTrajectory(const Trajectory &trajectory) noexcept
  {
    return isValidTrajectory(trajectory);
  }

  static bool sampleAt(
    const Trajectory &trajectory, double time_from_start, Point &output) noexcept
  {
    if (!validTrajectory(trajectory)) {
      return false;
    }
    return sampleValidated(trajectory, time_from_start, output);
  }

  static bool sampleAt(
    const ValidatedTrajectory &trajectory, double time_from_start, Point &output) noexcept
  {
    if (!trajectory.valid()) {
      return false;
    }
    return sampleValidated(trajectory.get(), time_from_start, output);
  }

  static bool sampleHorizon(
    const Trajectory &trajectory, double trajectory_age_seconds,
    double dt_first, double dt_later, Horizon &horizon) noexcept
  {
    if (!validTrajectory(trajectory)) {
      return false;
    }
    horizon.limits = trajectory.limits;
    return sampleValidatedHorizon(
      trajectory, trajectory_age_seconds, dt_first, dt_later, horizon);
  }

  static bool sampleHorizon(
    const ValidatedTrajectory &trajectory, double trajectory_age_seconds,
    double dt_first, double dt_later, Horizon &horizon) noexcept
  {
    if (!trajectory.valid()) {
      return false;
    }
    horizon.limits = trajectory.get().limits;
    return sampleValidatedHorizon(
      trajectory.get(), trajectory_age_seconds, dt_first, dt_later, horizon);
  }

  static bool buildHorizon(
    const Trajectory &trajectory, double trajectory_age_seconds,
    double dt_first, double dt_later, Horizon &horizon) noexcept
  {
    return sampleHorizon(trajectory, trajectory_age_seconds, dt_first, dt_later, horizon);
  }

  static bool buildHorizon(
    const ValidatedTrajectory &trajectory, double trajectory_age_seconds,
    double dt_first, double dt_later, Horizon &horizon) noexcept
  {
    return sampleHorizon(trajectory, trajectory_age_seconds, dt_first, dt_later, horizon);
  }

private:
  static bool sampleValidated(
    const Trajectory &trajectory, double time_from_start, Point &output) noexcept
  {
    if (!std::isfinite(time_from_start)) {
      return false;
    }
    if (time_from_start <= trajectory.points.front().time_from_start) {
      output = trajectory.points.front();
      return true;
    }
    if (time_from_start >= trajectory.points.back().time_from_start) {
      output = trajectory.points.back();
      if (trajectory.hold_after_end) {
        output.velocity = {0.0, 0.0, 0.0};
        output.acceleration = {0.0, 0.0, 0.0};
        output.yaw_rate = 0.0;
      }
      return true;
    }

    const auto it = std::lower_bound(
      trajectory.points.begin(), trajectory.points.end(), time_from_start,
      [](const Point &point, double target) {
        return point.time_from_start < target;
      });
    if (it == trajectory.points.begin()) {
      output = *it;
      return true;
    }
    const auto &previous = *(it - 1);
    const auto &next = *it;
    const double delta_t = next.time_from_start - previous.time_from_start;
    if (delta_t <= 1.0e-6) {
      output = next;
      return true;
    }

    const double fraction = std::clamp((time_from_start - previous.time_from_start) / delta_t, 0.0, 1.0);
    output.time_from_start = time_from_start;
    for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
      output.position[axis] = previous.position[axis] + fraction * (next.position[axis] - previous.position[axis]);
      output.velocity[axis] = previous.velocity[axis] + fraction * (next.velocity[axis] - previous.velocity[axis]);
      output.acceleration[axis] = previous.acceleration[axis] + fraction * (next.acceleration[axis] - previous.acceleration[axis]);
    }
    output.yaw = previous.yaw + fraction * shortestAngle(previous.yaw, next.yaw);
    output.yaw_rate = previous.yaw_rate + fraction * (next.yaw_rate - previous.yaw_rate);
    return true;
  }

  static bool sampleValidatedHorizon(
    const Trajectory &trajectory, double trajectory_age_seconds,
    double dt_first, double dt_later, Horizon &horizon) noexcept
  {
    if (!std::isfinite(trajectory_age_seconds) || trajectory_age_seconds < 0.0 ||
      !std::isfinite(dt_first) || dt_first <= 0.0 ||
      !std::isfinite(dt_later) || dt_later <= 0.0) {
      return false;
    }

    double stage_time = trajectory_age_seconds;
    for (std::size_t step = 0; step < kHorizonLength; ++step) {
      stage_time += (step == 0) ? dt_first : dt_later;
      if (!sampleValidated(trajectory, stage_time, horizon.points[step])) {
        return false;
      }
    }
    return true;
  }
};

}  // namespace mpc_controller::reference

namespace mpc_controller::state_admission
{

struct TimestampHistory
{
  std::uint64_t boot_time = 0;
  std::uint64_t synchronized_time = 0;
  bool last_was_synchronized = false;
  bool received = false;
};

struct TimestampDecision
{
  bool accepted = false;
  bool domain_switched = false;
};

inline bool synchronizedTimestamp(std::uint64_t timestamp) noexcept
{
  constexpr std::uint64_t synchronized_epoch_threshold_us = 1000000000000ULL;
  return timestamp >= synchronized_epoch_threshold_us;
}

inline TimestampDecision acceptTimestamp(
  TimestampHistory &history, std::uint64_t timestamp) noexcept
{
  if (timestamp == 0) {
    return {};
  }

  const bool synchronized = synchronizedTimestamp(timestamp);
  std::uint64_t &previous = synchronized ? history.synchronized_time : history.boot_time;
  if (previous != 0 && timestamp < previous) {
    return {};
  }

  const bool domain_switched = history.received && synchronized != history.last_was_synchronized;
  previous = timestamp;
  history.last_was_synchronized = synchronized;
  history.received = true;
  return {true, domain_switched};
}

struct Timing
{
  std::uint64_t sample_time = 0;
  double age = std::numeric_limits<double>::infinity();
  bool received = false;
};

enum class Reject : std::uint8_t
{
  none,
  position_stale,
  attitude_stale,
  rate_stale,
  sample_skew,
  timestamp,
  invalid_configuration
};

struct Decision
{
  Reject reason = Reject::timestamp;
  double skew = std::numeric_limits<double>::infinity();
  bool valid = false;
};

inline bool fresh(const Timing &source, double timeout) noexcept
{
  return source.received && std::isfinite(source.age) && source.age >= 0.0 &&
    source.age <= timeout;
}

inline Decision evaluate(
  const Timing &position, const Timing &attitude, const Timing &rate,
  double timeout, double max_skew) noexcept
{
  Decision result;
  if (!std::isfinite(timeout) || !std::isfinite(max_skew) ||
    timeout < 0.0 || max_skew < 0.0) {
    result.reason = Reject::invalid_configuration;
    return result;
  }
  if (!fresh(position, timeout)) {
    result.reason = Reject::position_stale;
    return result;
  }
  if (!fresh(attitude, timeout)) {
    result.reason = Reject::attitude_stale;
    return result;
  }
  if (!fresh(rate, timeout)) {
    result.reason = Reject::rate_stale;
    return result;
  }
  if (position.sample_time == 0 || attitude.sample_time == 0 || rate.sample_time == 0) {
    result.reason = Reject::timestamp;
    return result;
  }
  const auto oldest = std::min({position.sample_time, attitude.sample_time, rate.sample_time});
  const auto newest = std::max({position.sample_time, attitude.sample_time, rate.sample_time});
  result.skew = static_cast<double>(newest - oldest) * 1.0e-6;
  if (!std::isfinite(result.skew) || result.skew > max_skew) {
    result.reason = Reject::sample_skew;
    return result;
  }
  result.reason = Reject::none;
  result.valid = true;
  return result;
}

inline const char *reasonName(Reject reason) noexcept
{
  switch (reason) {
    case Reject::none: return "NONE";
    case Reject::position_stale: return "POSITION_STALE";
    case Reject::attitude_stale: return "ATTITUDE_STALE";
    case Reject::rate_stale: return "ANGULAR_VELOCITY_STALE";
    case Reject::sample_skew: return "SAMPLE_SKEW";
    case Reject::timestamp: return "TIMESTAMP";
    case Reject::invalid_configuration: return "INVALID_CONFIGURATION";
  }
  return "UNKNOWN";
}

}  // namespace mpc_controller::state_admission

namespace mpc_controller::mission
{

class TrajectoryCurve final
{
public:
  struct State
  {
    std::array<double, 3> position{0.0, 0.0, 0.0};
    std::array<double, 3> velocity{0.0, 0.0, 0.0};
    std::array<double, 3> acceleration{0.0, 0.0, 0.0};
  };

  struct Sample : State
  {
    std::array<double, 3> jerk{0.0, 0.0, 0.0};
  };

  struct Limits
  {
    double max_speed_xy_m_s = 18.0;
    double max_speed_z_m_s = 3.0;
    double max_acceleration_xy_m_s2 = 6.0;
    double max_acceleration_z_m_s2 = 5.0;
    double max_jerk_xy_m_s3 = 8.0;
    double max_jerk_z_m_s3 = 4.0;
  };

  static std::optional<TrajectoryCurve> create(
    const State &start, const State &end, double initial_duration_seconds,
    const Limits &limits) noexcept;

  Sample sample(double elapsed_seconds) const noexcept;
  double durationSeconds() const noexcept { return duration_seconds_; }
  bool withinLimits() const noexcept;

private:
  using Coefficients = std::array<std::array<double, 3>, 6>;

  TrajectoryCurve(
    State start, State end, double duration_seconds,
    Coefficients coefficients, Limits limits) noexcept
  : start_(start), end_(end), duration_seconds_(duration_seconds),
    coefficients_(coefficients), limits_(limits) {}

  static TrajectoryCurve make(const State &start, const State &end, double duration_seconds, const Limits &limits) noexcept;
  double derivativeBound(std::size_t axis, std::size_t derivative_order) const noexcept;

  State start_;
  State end_;
  double duration_seconds_ = 0.0;
  Coefficients coefficients_{};
  Limits limits_;
};

namespace trajectory_detail
{

inline bool finite(const TrajectoryCurve::State &state) noexcept
{
  for (const auto &values : {state.position, state.velocity, state.acceleration}) {
    for (const double value : values) {
      if (!std::isfinite(value)) {
        return false;
      }
    }
  }
  return true;
}

inline bool positiveFinite(double value) noexcept
{
  return std::isfinite(value) && value > 0.0;
}

inline bool validLimits(const TrajectoryCurve::Limits &limits) noexcept
{
  return positiveFinite(limits.max_speed_xy_m_s) &&
    positiveFinite(limits.max_speed_z_m_s) &&
    positiveFinite(limits.max_acceleration_xy_m_s2) &&
    positiveFinite(limits.max_acceleration_z_m_s2) &&
    positiveFinite(limits.max_jerk_xy_m_s3) &&
    positiveFinite(limits.max_jerk_z_m_s3);
}

using Polynomial = std::array<double, 6>;

inline double evaluate(const Polynomial &polynomial, std::size_t degree, double time) noexcept
{
  double value = polynomial[degree];
  for (std::size_t power = degree; power > 0U; --power) {
    value = value * time + polynomial[power - 1U];
  }
  return value;
}

inline void appendRoot(std::vector<double> &roots, double root, double lower, double upper)
{
  constexpr double tolerance = 1e-9;
  if (!std::isfinite(root) || root < lower - tolerance || root > upper + tolerance) {
    return;
  }
  root = std::clamp(root, lower, upper);
  for (const double existing : roots) {
    if (std::abs(existing - root) <= 1e-7) {
      return;
    }
  }
  roots.push_back(root);
}

inline void findRoots(
  const Polynomial &polynomial, std::size_t degree, double lower,
  double upper, std::vector<double> &roots)
{
  while (degree > 0U && std::abs(polynomial[degree]) <= 1e-12) {
    --degree;
  }
  if (degree == 0U) {
    return;
  }
  if (degree == 1U) {
    appendRoot(roots, -polynomial[0] / polynomial[1], lower, upper);
    return;
  }
  if (degree == 2U) {
    const double a = polynomial[2];
    const double b = polynomial[1];
    const double c = polynomial[0];
    const double discriminant = b * b - 4.0 * a * c;
    if (discriminant < 0.0) {
      return;
    }
    const double root = std::sqrt(std::max(0.0, discriminant));
    appendRoot(roots, (-b - root) / (2.0 * a), lower, upper);
    appendRoot(roots, (-b + root) / (2.0 * a), lower, upper);
    return;
  }

  Polynomial derivative{};
  for (std::size_t power = 1U; power <= degree; ++power) {
    derivative[power - 1U] = static_cast<double>(power) * polynomial[power];
  }
  std::vector<double> critical_points;
  findRoots(derivative, degree - 1U, lower, upper, critical_points);

  std::vector<double> points{lower};
  points.insert(points.end(), critical_points.begin(), critical_points.end());
  points.push_back(upper);
  std::sort(points.begin(), points.end());
  for (std::size_t index = 0U; index + 1U < points.size(); ++index) {
    double left = points[index];
    double right = points[index + 1U];
    double left_value = evaluate(polynomial, degree, left);
    double right_value = evaluate(polynomial, degree, right);
    if (std::abs(left_value) <= 1e-10) {
      appendRoot(roots, left, lower, upper);
    }
    if (std::abs(right_value) <= 1e-10) {
      appendRoot(roots, right, lower, upper);
    }
    if (left_value * right_value >= 0.0) {
      continue;
    }
    for (std::size_t iteration = 0U; iteration < 64U; ++iteration) {
      const double middle = 0.5 * (left + right);
      const double middle_value = evaluate(polynomial, degree, middle);
      if (std::abs(middle_value) <= 1e-12) {
        left_value = middle_value;
        right_value = middle_value;
        break;
      }
      if (left_value * middle_value <= 0.0) {
        right = middle;
        right_value = middle_value;
      } else {
        left = middle;
        left_value = middle_value;
      }
    }
    appendRoot(roots, 0.5 * (left + right), lower, upper);
  }
}

}  // namespace trajectory_detail

inline std::optional<TrajectoryCurve> TrajectoryCurve::create(
  const State &start, const State &end, double initial_duration_seconds,
  const Limits &limits) noexcept
{
  if (!trajectory_detail::finite(start) || !trajectory_detail::finite(end) ||
    !trajectory_detail::validLimits(limits) ||
    !trajectory_detail::positiveFinite(initial_duration_seconds)) {
    return std::nullopt;
  }

  double duration = std::max(initial_duration_seconds, 0.05);
  for (std::size_t attempt = 0; attempt < 160U; ++attempt) {
    auto curve = make(start, end, duration, limits);
    if (curve.withinLimits()) {
      return curve;
    }
    duration *= 1.15;
    if (!std::isfinite(duration) || duration > 3600.0) {
      break;
    }
  }
  return std::nullopt;
}

inline TrajectoryCurve TrajectoryCurve::make(const State &start, const State &end, double duration_seconds, const Limits &limits) noexcept
{
  Coefficients coefficients{};
  const double t = duration_seconds;
  const double t2 = t * t;
  const double t3 = t2 * t;
  const double t4 = t3 * t;
  const double t5 = t4 * t;

  for (std::size_t axis = 0; axis < 3U; ++axis) {
    const double delta_position = end.position[axis] - start.position[axis] - start.velocity[axis] * t - 0.5 * start.acceleration[axis] * t2;
    const double delta_velocity = end.velocity[axis] - start.velocity[axis] - start.acceleration[axis] * t;
    const double delta_acceleration = end.acceleration[axis] - start.acceleration[axis];

    coefficients[0][axis] = start.position[axis];
    coefficients[1][axis] = start.velocity[axis];
    coefficients[2][axis] = 0.5 * start.acceleration[axis];
    coefficients[3][axis] = (10.0 * delta_position - 4.0 * delta_velocity * t + 0.5 * delta_acceleration * t2) / t3;
    coefficients[4][axis] = (-15.0 * delta_position + 7.0 * delta_velocity * t - delta_acceleration * t2) / t4;
    coefficients[5][axis] = (6.0 * delta_position - 3.0 * delta_velocity * t + 0.5 * delta_acceleration * t2) / t5;
  }

  return TrajectoryCurve(start, end, duration_seconds, coefficients, limits);
}

inline TrajectoryCurve::Sample TrajectoryCurve::sample(double elapsed_seconds) const noexcept
{
  if (elapsed_seconds <= 0.0) {
    Sample output;
    static_cast<State &>(output) = start_;
    return output;
  }
  if (elapsed_seconds >= duration_seconds_) {
    Sample output;
    static_cast<State &>(output) = end_;
    return output;
  }

  Sample output;
  const double t = elapsed_seconds;
  for (std::size_t axis = 0; axis < 3U; ++axis) {
    const auto &c = coefficients_;
    output.position[axis] = c[0][axis] + t * (c[1][axis] + t * (c[2][axis] + t * (c[3][axis] + t * (c[4][axis] + t * c[5][axis]))));
    output.velocity[axis] = c[1][axis] + t * (2.0 * c[2][axis] + t * (3.0 * c[3][axis] + t * (4.0 * c[4][axis] + t * 5.0 * c[5][axis])));
    output.acceleration[axis] = 2.0 * c[2][axis] + t * (6.0 * c[3][axis] + t * (12.0 * c[4][axis] + t * 20.0 * c[5][axis]));
    output.jerk[axis] = 6.0 * c[3][axis] + t * (24.0 * c[4][axis] + t * 60.0 * c[5][axis]);
  }
  return output;
}

inline double TrajectoryCurve::derivativeBound(std::size_t axis, std::size_t derivative_order) const noexcept
{
  if (axis >= 3U || derivative_order == 0U || derivative_order > 3U) {
    return 0.0;
  }

  trajectory_detail::Polynomial derivative{};
  const auto &coefficients = coefficients_;
  const std::size_t degree = 5U - derivative_order;
  for (std::size_t power = derivative_order; power < 6U; ++power) {
    double factor = 1.0;
    for (std::size_t term = 0; term < derivative_order; ++term) {
      factor *= static_cast<double>(power - term);
    }
    derivative[power - derivative_order] = coefficients[power][axis] * factor;
  }

  std::vector<double> critical_points;
  if (degree > 0U) {
    trajectory_detail::Polynomial slope{};
    for (std::size_t power = 1U; power <= degree; ++power) {
      slope[power - 1U] = static_cast<double>(power) * derivative[power];
    }
    trajectory_detail::findRoots(slope, degree - 1U, 0.0, duration_seconds_, critical_points);
  }

  double bound = std::max(
    std::abs(trajectory_detail::evaluate(derivative, degree, 0.0)),
    std::abs(trajectory_detail::evaluate(derivative, degree, duration_seconds_)));
  for (const double time : critical_points) {
    bound = std::max(bound, std::abs(trajectory_detail::evaluate(derivative, degree, time)));
  }
  return bound;
}

inline bool TrajectoryCurve::withinLimits() const noexcept
{
  const double speed_xy = std::hypot(derivativeBound(0U, 1U), derivativeBound(1U, 1U));
  bool speed_ok = (speed_xy <= limits_.max_speed_xy_m_s + 1.0e-9);
  if (!speed_ok) {
    speed_ok = true;
    constexpr std::size_t kSamples = 32U;
    for (std::size_t i = 0U; i <= kSamples; ++i) {
      const double t = duration_seconds_ * static_cast<double>(i) / static_cast<double>(kSamples);
      const auto s = sample(t);
      if (std::hypot(s.velocity[0], s.velocity[1]) > limits_.max_speed_xy_m_s + 1.0e-6) {
        speed_ok = false;
        break;
      }
    }
  }

  const double acceleration_xy = std::hypot(derivativeBound(0U, 2U), derivativeBound(1U, 2U));
  bool acc_ok = (acceleration_xy <= limits_.max_acceleration_xy_m_s2 + 1.0e-9);
  if (!acc_ok) {
    acc_ok = true;
    constexpr std::size_t kSamples = 32U;
    for (std::size_t i = 0U; i <= kSamples; ++i) {
      const double t = duration_seconds_ * static_cast<double>(i) / static_cast<double>(kSamples);
      const auto s = sample(t);
      if (std::hypot(s.acceleration[0], s.acceleration[1]) > limits_.max_acceleration_xy_m_s2 + 1.0e-6) {
        acc_ok = false;
        break;
      }
    }
  }

  const double jerk_xy = std::hypot(derivativeBound(0U, 3U), derivativeBound(1U, 3U));
  bool jerk_ok = (jerk_xy <= limits_.max_jerk_xy_m_s3 + 1.0e-9);
  if (!jerk_ok) {
    jerk_ok = true;
    constexpr std::size_t kSamples = 32U;
    for (std::size_t i = 0U; i <= kSamples; ++i) {
      const double t = duration_seconds_ * static_cast<double>(i) / static_cast<double>(kSamples);
      const auto s = sample(t);
      if (std::hypot(s.jerk[0], s.jerk[1]) > limits_.max_jerk_xy_m_s3 + 1.0e-6) {
        jerk_ok = false;
        break;
      }
    }
  }

  return speed_ok &&
    derivativeBound(2U, 1U) <= limits_.max_speed_z_m_s + 1.0e-6 && acc_ok &&
    derivativeBound(2U, 2U) <= limits_.max_acceleration_z_m_s2 + 1.0e-6 && jerk_ok &&
    derivativeBound(2U, 3U) <= limits_.max_jerk_z_m_s3 + 1.0e-6;
}

}  // namespace mpc_controller::mission
