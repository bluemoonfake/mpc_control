#include <algorithm>
#include <cmath>

namespace
{

constexpr double kGravity = 9.80665;
constexpr double kMaxThrustAcceleration = 19.6133;
constexpr double kEpsilon = 1.0e-9;

}  // namespace

extern "C" int acceleration_to_attitude(
    const double acceleration_enu[3],
    const double yaw_enu,
    double quaternion_body_to_ned[4],
    double thrust_body_frd[3]) noexcept
{
  if (acceleration_enu == nullptr || quaternion_body_to_ned == nullptr
      || thrust_body_frd == nullptr || !std::isfinite(yaw_enu)) {
    return 0;
  }

  const double force_enu[3] = {
    acceleration_enu[0], acceleration_enu[1], acceleration_enu[2] + kGravity};
  const double force_ned[3] = {force_enu[1], force_enu[0], -force_enu[2]};
  const double norm = std::sqrt(
      force_ned[0] * force_ned[0] + force_ned[1] * force_ned[1]
      + force_ned[2] * force_ned[2]);
  if (!std::isfinite(norm) || norm < kEpsilon) {
    return 0;
  }

  const double b3[3] = {
    -force_ned[0] / norm, -force_ned[1] / norm, -force_ned[2] / norm};
  const double yaw_ned = 1.5707963267948966 - yaw_enu;
  const double heading[3] = {std::cos(yaw_ned), std::sin(yaw_ned), 0.0};

  double b2[3] = {
    b3[1] * heading[2] - b3[2] * heading[1],
    b3[2] * heading[0] - b3[0] * heading[2],
    b3[0] * heading[1] - b3[1] * heading[0]};
  const double b2_norm = std::sqrt(
      b2[0] * b2[0] + b2[1] * b2[1] + b2[2] * b2[2]);
  if (!std::isfinite(b2_norm) || b2_norm < kEpsilon) {
    return 0;
  }
  b2[0] /= b2_norm;
  b2[1] /= b2_norm;
  b2[2] /= b2_norm;

  const double b1[3] = {
    b2[1] * b3[2] - b2[2] * b3[1],
    b2[2] * b3[0] - b2[0] * b3[2],
    b2[0] * b3[1] - b2[1] * b3[0]};

  const double trace = b1[0] + b2[1] + b3[2];
  double qw;
  double qx;
  double qy;
  double qz;
  if (trace > 0.0) {
    const double scale = 0.5 / std::sqrt(trace + 1.0);
    qw = 0.25 / scale;
    qx = (b2[2] - b3[1]) * scale;
    qy = (b3[0] - b1[2]) * scale;
    qz = (b1[1] - b2[0]) * scale;
  } else if (b1[0] > b2[1] && b1[0] > b3[2]) {
    const double scale = 2.0 * std::sqrt(1.0 + b1[0] - b2[1] - b3[2]);
    qw = (b2[2] - b3[1]) / scale;
    qx = 0.25 * scale;
    qy = (b2[0] + b1[1]) / scale;
    qz = (b3[0] + b1[2]) / scale;
  } else if (b2[1] > b3[2]) {
    const double scale = 2.0 * std::sqrt(1.0 + b2[1] - b1[0] - b3[2]);
    qw = (b3[0] - b1[2]) / scale;
    qx = (b2[0] + b1[1]) / scale;
    qy = 0.25 * scale;
    qz = (b3[1] + b2[2]) / scale;
  } else {
    const double scale = 2.0 * std::sqrt(1.0 + b3[2] - b1[0] - b2[1]);
    qw = (b1[1] - b2[0]) / scale;
    qx = (b3[0] + b1[2]) / scale;
    qy = (b3[1] + b2[2]) / scale;
    qz = 0.25 * scale;
  }

  const double q_norm = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
  if (!std::isfinite(q_norm) || q_norm < kEpsilon) {
    return 0;
  }
  quaternion_body_to_ned[0] = qw / q_norm;
  quaternion_body_to_ned[1] = qx / q_norm;
  quaternion_body_to_ned[2] = qy / q_norm;
  quaternion_body_to_ned[3] = qz / q_norm;
  thrust_body_frd[0] = 0.0;
  thrust_body_frd[1] = 0.0;
  thrust_body_frd[2] = -std::clamp(norm / kMaxThrustAcceleration, 0.0, 1.0);
  return 1;
}

