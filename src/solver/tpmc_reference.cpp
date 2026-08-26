#include "mpc_controller/solver/tpmc_reference.hpp"

#include <algorithm>
#include <cmath>

namespace mpc_controller::tpmc {
namespace {

struct Vector3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

Vector3 add(Vector3 left, Vector3 right) noexcept {
  return {left.x + right.x, left.y + right.y, left.z + right.z};
}

Vector3 scale(Vector3 value, double factor) noexcept {
  return {value.x * factor, value.y * factor, value.z * factor};
}

double norm(Vector3 value) noexcept {
  return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
}

Vector3 cross(Vector3 left, Vector3 right) noexcept {
  return {left.y * right.z - left.z * right.y,
          left.z * right.x - left.x * right.z,
          left.x * right.y - left.y * right.x};
}

Vector3 normalized(Vector3 value) noexcept {
  const double length = norm(value);
  if (!std::isfinite(length) || length < 1.0e-9) {
    return {};
  }
  return scale(value, 1.0 / length);
}

struct AttitudeReference {
  double roll = 0.0;
  double pitch = 0.0;
  double collective_force = 0.0;
};

bool accelerationToAttitude(const Vector3 &acceleration, double yaw,
                            double gravity_m_s2, double maximum_tilt_rad,
                            AttitudeReference &output) noexcept {
  const Vector3 force = add(acceleration, {0.0, 0.0, gravity_m_s2});
  const double force_norm = norm(force);
  if (!std::isfinite(force_norm) || force_norm < 1.0e-9 || force.z <= 0.0) {
    return false;
  }

  const Vector3 body_z = scale(force, 1.0 / force_norm);
  const Vector3 heading{std::cos(yaw), std::sin(yaw), 0.0};
  const Vector3 body_y = normalized(cross(body_z, heading));
  if (norm(body_y) < 1.0e-9) {
    return false;
  }
  const Vector3 body_x = normalized(cross(body_y, body_z));
  if (norm(body_x) < 1.0e-9) {
    return false;
  }

  const double tilt = std::acos(std::clamp(body_z.z, -1.0, 1.0));
  if (!std::isfinite(tilt) || tilt > maximum_tilt_rad + 1.0e-9) {
    return false;
  }

  // R=[body_x body_y body_z] is body-FLU to world-ENU.
  const double r20 = body_x.z;
  const double r21 = body_y.z;
  const double r22 = body_z.z;
  output.roll = std::atan2(r21, r22);
  output.pitch = std::asin(std::clamp(-r20, -1.0, 1.0));
  output.collective_force = force_norm;
  return std::isfinite(output.roll) && std::isfinite(output.pitch) &&
         std::isfinite(output.collective_force);
}

TpmcReference toTpmcReference(const ReferencePoint &point,
                              double maximum_tilt_rad, double gravity_m_s2,
                              bool &valid) noexcept {
  TpmcReference output;
  AttitudeReference attitude;
  valid = accelerationToAttitude(
      {point.acceleration[0], point.acceleration[1], point.acceleration[2]},
      point.yaw, gravity_m_s2, maximum_tilt_rad, attitude);
  if (!valid) {
    return output;
  }

  output.state = {point.position[0], point.position[1],
                  point.position[2], point.velocity[0],
                  point.velocity[1], point.velocity[2],
                  attitude.roll,     attitude.pitch,
                  point.yaw,         point.yaw_rate,
                  attitude.collective_force};
  output.input = {attitude.roll, attitude.pitch, point.yaw,
                  attitude.collective_force};
  valid = finite(output.state) && finite(output.input);
  return output;
}

} // namespace

bool finite(const ReferencePoint &point) noexcept {
  return std::isfinite(point.time_from_start) && point.time_from_start >= 0.0 &&
         finite(point.position) && finite(point.velocity) &&
         finite(point.acceleration) && std::isfinite(point.yaw) &&
         std::isfinite(point.yaw_rate);
}

bool validTrajectory(const ReferenceTrajectory &trajectory) noexcept {
  if (!std::isfinite(trajectory.header_time_seconds) ||
      trajectory.points.empty()) {
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

double shortestAngle(double from, double to) noexcept {
  return std::atan2(std::sin(to - from), std::cos(to - from));
}

double rebaseAngle(double continuous_reference, double wrapped_angle) noexcept {
  return continuous_reference + shortestAngle(continuous_reference, wrapped_angle);
}

bool sampleReference(const ReferenceTrajectory &trajectory,
                     double time_from_start, ReferencePoint &output) noexcept {
  if (!validTrajectory(trajectory) || !std::isfinite(time_from_start)) {
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

  const auto next = std::lower_bound(
      trajectory.points.begin(), trajectory.points.end(), time_from_start,
      [](const ReferencePoint &point, double target) {
        return point.time_from_start < target;
      });
  if (next == trajectory.points.begin()) {
    output = *next;
    return true;
  }

  const ReferencePoint &previous = *(next - 1);
  const double delta_t = next->time_from_start - previous.time_from_start;
  if (delta_t <= 1.0e-9) {
    output = *next;
    return true;
  }

  const double fraction = std::clamp(
      (time_from_start - previous.time_from_start) / delta_t, 0.0, 1.0);
  output.time_from_start = time_from_start;
  for (std::size_t axis = 0; axis < 3; ++axis) {
    output.position[axis] =
        previous.position[axis] +
        fraction * (next->position[axis] - previous.position[axis]);
    output.velocity[axis] =
        previous.velocity[axis] +
        fraction * (next->velocity[axis] - previous.velocity[axis]);
    output.acceleration[axis] =
        previous.acceleration[axis] +
        fraction * (next->acceleration[axis] - previous.acceleration[axis]);
  }
  output.yaw = previous.yaw + fraction * shortestAngle(previous.yaw, next->yaw);
  output.yaw_rate =
      previous.yaw_rate + fraction * (next->yaw_rate - previous.yaw_rate);
  return finite(output);
}

bool buildReferenceHorizon(const ReferenceTrajectory &trajectory,
                           double trajectory_age_seconds,
                           double sample_time_seconds, double maximum_tilt_rad,
                           double gravity_m_s2,
                           ReferenceHorizon &output) noexcept {
  if (!validTrajectory(trajectory) || !std::isfinite(trajectory_age_seconds) ||
      trajectory_age_seconds < 0.0 || !std::isfinite(sample_time_seconds) ||
      sample_time_seconds <= 0.0 || !std::isfinite(maximum_tilt_rad) ||
      maximum_tilt_rad <= 0.0 || !std::isfinite(gravity_m_s2) ||
      gravity_m_s2 <= 0.0) {
    return false;
  }

  for (std::size_t step = 0; step <= kHorizonLength; ++step) {
    ReferencePoint physical_reference;
    const double stage_time = trajectory_age_seconds +
                              static_cast<double>(step) * sample_time_seconds;
    if (!sampleReference(trajectory, stage_time, physical_reference)) {
      return false;
    }
    bool point_valid = false;
    output[step] = toTpmcReference(physical_reference, maximum_tilt_rad,
                                   gravity_m_s2, point_valid);
    if (!point_valid) {
      return false;
    }
  }
  return true;
}

} // namespace mpc_controller::tpmc
