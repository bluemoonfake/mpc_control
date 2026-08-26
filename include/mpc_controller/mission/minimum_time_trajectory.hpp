#pragma once

#include "mpc_controller/controller/reference_model.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>

namespace mpc_controller::trajectory {

struct Limits
{
  double maximum_horizontal_speed_m_s = 4.0;
  double maximum_vertical_speed_m_s = 1.5;
  double maximum_acceleration_m_s2 = 2.5;
  double maximum_jerk_m_s3 = 5.0;
  double maximum_heading_rate_rad_s = 1.0471975511965976;
};

struct Boundary
{
  reference::Sample sample{};
};

inline std::array<double, 3> blendedCornerVelocity(
  const std::array<double, 3> &previous,
  const std::array<double, 3> &corner,
  const std::array<double, 3> &next,
  double maximum_horizontal_speed_m_s,
  double maximum_vertical_speed_m_s) noexcept
{
  constexpr double kMinimumDirectionNorm = 1.0e-6;
  const auto finitePosition = [](const std::array<double, 3> &position) {
    return std::all_of(position.begin(), position.end(), [](double value) {
      return std::isfinite(value);
    });
  };
  if (!finitePosition(previous) || !finitePosition(corner) || !finitePosition(next) ||
      !std::isfinite(maximum_horizontal_speed_m_s) || maximum_horizontal_speed_m_s <= 0.0 ||
      !std::isfinite(maximum_vertical_speed_m_s) || maximum_vertical_speed_m_s <= 0.0) {
    return {};
  }

  std::array<double, 3> incoming{};
  std::array<double, 3> outgoing{};
  for (std::size_t axis = 0; axis < incoming.size(); ++axis) {
    incoming[axis] = corner[axis] - previous[axis];
    outgoing[axis] = next[axis] - corner[axis];
  }
  const auto vectorNorm = [](const std::array<double, 3> &value) {
    return std::sqrt(value[0] * value[0] + value[1] * value[1] +
                     value[2] * value[2]);
  };
  const double incoming_norm = vectorNorm(incoming);
  const double outgoing_norm = vectorNorm(outgoing);
  if (incoming_norm <= kMinimumDirectionNorm || outgoing_norm <= kMinimumDirectionNorm) {
    return {};
  }

  std::array<double, 3> bisector{};
  double direction_alignment = 0.0;
  for (std::size_t axis = 0; axis < bisector.size(); ++axis) {
    const double incoming_unit = incoming[axis] / incoming_norm;
    const double outgoing_unit = outgoing[axis] / outgoing_norm;
    bisector[axis] = incoming_unit + outgoing_unit;
    direction_alignment += incoming_unit * outgoing_unit;
  }
  const double bisector_norm = vectorNorm(bisector);
  if (bisector_norm <= kMinimumDirectionNorm) {
    return {};
  }
  for (double &component : bisector) {
    component /= bisector_norm;
  }

  // cos(turn_angle / 2) is one for a straight path and zero for a U-turn.
  // Cubing it deliberately slows sharp corners more strongly than shallow
  // bends, preventing fixed endpoint derivatives from making a quintic leg
  // loop far outside the two adjacent path segments.
  const double half_angle_cosine = std::sqrt(std::max(
    0.0, 0.5 * (1.0 + std::clamp(direction_alignment, -1.0, 1.0))));
  const double corner_speed_scale = half_angle_cosine * half_angle_cosine *
    half_angle_cosine;
  if (corner_speed_scale <= kMinimumDirectionNorm) {
    return {};
  }

  double maximum_speed_along_bisector = std::numeric_limits<double>::infinity();
  const double horizontal_component = std::hypot(bisector[0], bisector[1]);
  if (horizontal_component > kMinimumDirectionNorm) {
    maximum_speed_along_bisector = std::min(
      maximum_speed_along_bisector,
      maximum_horizontal_speed_m_s / horizontal_component);
  }
  if (std::abs(bisector[2]) > kMinimumDirectionNorm) {
    maximum_speed_along_bisector = std::min(
      maximum_speed_along_bisector,
      maximum_vertical_speed_m_s / std::abs(bisector[2]));
  }
  if (!std::isfinite(maximum_speed_along_bisector)) {
    return {};
  }

  std::array<double, 3> velocity{};
  for (std::size_t axis = 0; axis < velocity.size(); ++axis) {
    velocity[axis] = bisector[axis] * maximum_speed_along_bisector * corner_speed_scale;
  }
  return velocity;
}

class QuinticSegment
{
public:
  static std::optional<QuinticSegment> create(
    const Boundary &start, const Boundary &finish, const Limits &limits) noexcept
  {
    if (!finite(start) || !finite(finish) || !valid(limits)) {
      return std::nullopt;
    }

    const double initial_duration = minimumDuration(start, finish, limits);
    for (double duration = initial_duration; duration <= kMaximumDurationSeconds;
         duration *= kDurationGrowth) {
      QuinticSegment segment(start, finish, duration);
      if (segment.respects(limits)) {
        return segment;
      }
    }
    return std::nullopt;
  }

  double durationSeconds() const noexcept { return duration_seconds_; }

  reference::Sample sample(double elapsed_seconds) const noexcept
  {
    const double time = std::clamp(elapsed_seconds, 0.0, duration_seconds_);
    reference::Sample output;
    for (std::size_t axis = 0; axis < output.position.size(); ++axis) {
      output.position[axis] = evaluate(position_[axis], time, 0);
      output.velocity[axis] = evaluate(position_[axis], time, 1);
      output.acceleration[axis] = evaluate(position_[axis], time, 2);
    }
    output.yaw = evaluate(yaw_, time, 0);
    output.yaw_rate = evaluate(yaw_, time, 1);
    return output;
  }

private:
  struct Polynomial
  {
    std::array<double, 6> coefficient{};
  };

  static constexpr double kMinimumDurationSeconds = 0.1;
  static constexpr double kMaximumDurationSeconds = 120.0;
  static constexpr double kDurationGrowth = 1.15;
  static constexpr std::size_t kConstraintSamples = 80;

  QuinticSegment(const Boundary &start, const Boundary &finish,
                 double duration_seconds) noexcept
  : duration_seconds_(duration_seconds)
  {
    for (std::size_t axis = 0; axis < position_.size(); ++axis) {
      position_[axis] = polynomial(
        start.sample.position[axis], start.sample.velocity[axis],
        start.sample.acceleration[axis], finish.sample.position[axis],
        finish.sample.velocity[axis], finish.sample.acceleration[axis],
        duration_seconds_);
    }
    const double target_yaw = start.sample.yaw + shortestAngle(
      start.sample.yaw, finish.sample.yaw);
    yaw_ = polynomial(start.sample.yaw, start.sample.yaw_rate, 0.0,
                      target_yaw, finish.sample.yaw_rate, 0.0,
                      duration_seconds_);
  }

  static bool finite(const Boundary &boundary) noexcept
  {
    return reference::finite(boundary.sample.position) &&
      reference::finite(boundary.sample.velocity) &&
      reference::finite(boundary.sample.acceleration) &&
      std::isfinite(boundary.sample.yaw) && std::isfinite(boundary.sample.yaw_rate);
  }

  static bool valid(const Limits &limits) noexcept
  {
    return std::isfinite(limits.maximum_horizontal_speed_m_s) &&
      limits.maximum_horizontal_speed_m_s > 0.0 &&
      std::isfinite(limits.maximum_vertical_speed_m_s) &&
      limits.maximum_vertical_speed_m_s > 0.0 &&
      std::isfinite(limits.maximum_acceleration_m_s2) &&
      limits.maximum_acceleration_m_s2 > 0.0 &&
      std::isfinite(limits.maximum_jerk_m_s3) &&
      limits.maximum_jerk_m_s3 > 0.0 &&
      std::isfinite(limits.maximum_heading_rate_rad_s) &&
      limits.maximum_heading_rate_rad_s > 0.0;
  }

  static double shortestAngle(double from, double to) noexcept
  {
    return std::atan2(std::sin(to - from), std::cos(to - from));
  }

  static double norm(const std::array<double, 3> &value) noexcept
  {
    return std::sqrt(value[0] * value[0] + value[1] * value[1] +
                     value[2] * value[2]);
  }

  static double horizontalNorm(const std::array<double, 3> &value) noexcept
  {
    return std::hypot(value[0], value[1]);
  }

  static double minimumDuration(const Boundary &start, const Boundary &finish,
                                const Limits &limits) noexcept
  {
    const std::array<double, 3> displacement{
      finish.sample.position[0] - start.sample.position[0],
      finish.sample.position[1] - start.sample.position[1],
      finish.sample.position[2] - start.sample.position[2]};
    const double horizontal_time = horizontalNorm(displacement) /
      limits.maximum_horizontal_speed_m_s;
    const double vertical_time = std::abs(displacement[2]) /
      limits.maximum_vertical_speed_m_s;
    const double yaw_time = std::abs(shortestAngle(start.sample.yaw,
                                                    finish.sample.yaw)) /
      limits.maximum_heading_rate_rad_s;
    return std::max({kMinimumDurationSeconds, horizontal_time, vertical_time,
                     yaw_time});
  }

  static Polynomial polynomial(double position_initial, double velocity_initial,
                               double acceleration_initial, double position_final,
                               double velocity_final, double acceleration_final,
                               double duration_seconds) noexcept
  {
    const double duration_squared = duration_seconds * duration_seconds;
    const double duration_cubed = duration_squared * duration_seconds;
    const double duration_fourth = duration_cubed * duration_seconds;
    const double duration_fifth = duration_fourth * duration_seconds;
    const double displacement = position_final - position_initial;
    Polynomial output;
    output.coefficient = {
      position_initial,
      velocity_initial,
      0.5 * acceleration_initial,
      (20.0 * displacement - (8.0 * velocity_final + 12.0 * velocity_initial) * duration_seconds -
       (3.0 * acceleration_initial - acceleration_final) * duration_squared) /
        (2.0 * duration_cubed),
      (30.0 * (position_initial - position_final) +
       (14.0 * velocity_final + 16.0 * velocity_initial) * duration_seconds +
       (3.0 * acceleration_initial - 2.0 * acceleration_final) * duration_squared) /
        (2.0 * duration_fourth),
      (12.0 * displacement - (6.0 * velocity_final + 6.0 * velocity_initial) * duration_seconds -
       (acceleration_initial - acceleration_final) * duration_squared) /
        (2.0 * duration_fifth)};
    return output;
  }

  static double evaluate(const Polynomial &polynomial, double time,
                         int derivative) noexcept
  {
    const auto &coefficient = polynomial.coefficient;
    switch (derivative) {
      case 0:
        return coefficient[0] + coefficient[1] * time + coefficient[2] * time * time +
          coefficient[3] * time * time * time + coefficient[4] * time * time * time * time +
          coefficient[5] * time * time * time * time * time;
      case 1:
        return coefficient[1] + 2.0 * coefficient[2] * time +
          3.0 * coefficient[3] * time * time + 4.0 * coefficient[4] * time * time * time +
          5.0 * coefficient[5] * time * time * time * time;
      case 2:
        return 2.0 * coefficient[2] + 6.0 * coefficient[3] * time +
          12.0 * coefficient[4] * time * time + 20.0 * coefficient[5] * time * time * time;
      default:
        return 6.0 * coefficient[3] + 24.0 * coefficient[4] * time +
          60.0 * coefficient[5] * time * time;
    }
  }

  bool respects(const Limits &limits) const noexcept
  {
    for (std::size_t index = 0; index <= kConstraintSamples; ++index) {
      const double time = duration_seconds_ * static_cast<double>(index) /
        static_cast<double>(kConstraintSamples);
      std::array<double, 3> velocity{};
      std::array<double, 3> acceleration{};
      std::array<double, 3> jerk{};
      for (std::size_t axis = 0; axis < velocity.size(); ++axis) {
        velocity[axis] = evaluate(position_[axis], time, 1);
        acceleration[axis] = evaluate(position_[axis], time, 2);
        jerk[axis] = evaluate(position_[axis], time, 3);
      }
      if (horizontalNorm(velocity) > limits.maximum_horizontal_speed_m_s + 1.0e-6 ||
          std::abs(velocity[2]) > limits.maximum_vertical_speed_m_s + 1.0e-6 ||
          norm(acceleration) > limits.maximum_acceleration_m_s2 + 1.0e-6 ||
          norm(jerk) > limits.maximum_jerk_m_s3 + 1.0e-6 ||
          std::abs(evaluate(yaw_, time, 1)) >
            limits.maximum_heading_rate_rad_s + 1.0e-6) {
        return false;
      }
    }
    return true;
  }

  std::array<Polynomial, 3> position_{};
  Polynomial yaw_{};
  double duration_seconds_ = 0.0;
};

}  // namespace mpc_controller::trajectory
