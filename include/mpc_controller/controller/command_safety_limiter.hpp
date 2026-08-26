#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>

namespace mpc_controller::command_safety {

struct Limits
{
  double gravity_m_s2 = 9.80665;
  double maximum_tilt_rad = 0.7853981633974483;
  double maximum_tilt_rate_rad_s = 2.0;
  double minimum_collective_specific_force_m_s2 = 7.0;
  double maximum_collective_specific_force_m_s2 = 14.0;
  double maximum_collective_rate_m_s3 = 25.0;
  double maximum_yaw_rate_rad_s = 2.0;
};

struct Result
{
  bool valid = false;
  bool tilt_limited = false;
  bool collective_limited = false;
  bool yaw_limited = false;
  Eigen::Quaterniond attitude_body_flu_to_world_enu{
    Eigen::Quaterniond::Identity()};
  double collective_specific_force_m_s2 = 0.0;
};

inline bool valid(const Limits & limits) noexcept
{
  return std::isfinite(limits.gravity_m_s2) && limits.gravity_m_s2 > 0.0 &&
    std::isfinite(limits.maximum_tilt_rad) && limits.maximum_tilt_rad > 0.0 &&
    limits.maximum_tilt_rad < 0.5 * M_PI &&
    std::isfinite(limits.maximum_tilt_rate_rad_s) && limits.maximum_tilt_rate_rad_s > 0.0 &&
    std::isfinite(limits.minimum_collective_specific_force_m_s2) &&
    std::isfinite(limits.maximum_collective_specific_force_m_s2) &&
    limits.minimum_collective_specific_force_m_s2 > 0.0 &&
    limits.minimum_collective_specific_force_m_s2 <=
    limits.maximum_collective_specific_force_m_s2 &&
    std::isfinite(limits.maximum_collective_rate_m_s3) &&
    limits.maximum_collective_rate_m_s3 > 0.0 &&
    std::isfinite(limits.maximum_yaw_rate_rad_s) &&
    limits.maximum_yaw_rate_rad_s > 0.0;
}

inline double tiltAngleRad(const Eigen::Quaterniond & attitude) noexcept
{
  const Eigen::Quaterniond normalized = attitude.normalized();
  const double body_z_z = normalized.toRotationMatrix()(2, 2);
  return std::acos(std::clamp(body_z_z, -1.0, 1.0));
}

inline double yawAngleRad(const Eigen::Quaterniond & attitude) noexcept
{
  const Eigen::Matrix3d rotation = attitude.normalized().toRotationMatrix();
  return std::atan2(rotation(1, 0), rotation(0, 0));
}

inline double wrapAngleRad(double angle_rad) noexcept
{
  return std::atan2(std::sin(angle_rad), std::cos(angle_rad));
}

inline double shortestAngleRad(double from_rad, double to_rad) noexcept
{
  return wrapAngleRad(to_rad - from_rad);
}

inline Eigen::Quaterniond attitudeFromBodyZAndYaw(
  const Eigen::Vector3d & body_z_world_enu, double yaw_enu_rad) noexcept
{
  const Eigen::Vector3d body_z = body_z_world_enu.normalized();
  const Eigen::Vector3d heading(
    std::cos(yaw_enu_rad), std::sin(yaw_enu_rad), 0.0);
  Eigen::Vector3d body_y = body_z.cross(heading);
  body_y.normalize();
  Eigen::Vector3d body_x = body_y.cross(body_z);
  body_x.normalize();

  Eigen::Matrix3d rotation;
  rotation.col(0) = body_x;
  rotation.col(1) = body_y;
  rotation.col(2) = body_z;
  return Eigen::Quaterniond(rotation).normalized();
}

class Limiter
{
public:
  explicit Limiter(const Limits & limits = Limits{}) noexcept
  : limits_(limits), previous_body_z_(Eigen::Vector3d::UnitZ())
  {
    reset();
  }

  void configure(const Limits & limits) noexcept
  {
    limits_ = limits;
    reset();
  }

  void reset(double yaw_rad = 0.0) noexcept
  {
    previous_collective_specific_force_m_s2_ = limits_.gravity_m_s2;
    previous_yaw_rad_ = wrapAngleRad(yaw_rad);
    previous_body_z_ = Eigen::Vector3d::UnitZ();
  }

  Eigen::Quaterniond levelAttitude() const noexcept
  {
    return attitudeFromBodyZAndYaw(Eigen::Vector3d::UnitZ(), previous_yaw_rad_);
  }

  void holdCollective() noexcept
  {
    previous_collective_specific_force_m_s2_ = limits_.gravity_m_s2;
  }

  Result limit(const Eigen::Quaterniond & requested_attitude,
               double requested_collective_specific_force_m_s2,
               double elapsed_seconds) noexcept
  {
    Result result;
    if (!valid(limits_) || !std::isfinite(requested_collective_specific_force_m_s2) ||
        !std::isfinite(elapsed_seconds) || elapsed_seconds < 0.0 ||
        !requested_attitude.coeffs().allFinite() ||
        requested_attitude.norm() < 1.0e-9) {
      return result;
    }

    const Eigen::Quaterniond normalized_req = requested_attitude.normalized();
    const Eigen::Matrix3d rot_req = normalized_req.toRotationMatrix();
    Eigen::Vector3d requested_body_z = rot_req.col(2);

    // 1. Tilt magnitude limit
    const double tilt_angle = tiltAngleRad(normalized_req);
    if (tilt_angle > limits_.maximum_tilt_rad) {
      const Eigen::Vector2d horizontal(requested_body_z.x(), requested_body_z.y());
      requested_body_z = horizontal.norm() < 1.0e-9
        ? Eigen::Vector3d::UnitZ()
        : Eigen::Vector3d(
            horizontal.x() / horizontal.norm() * std::sin(limits_.maximum_tilt_rad),
            horizontal.y() / horizontal.norm() * std::sin(limits_.maximum_tilt_rad),
            std::cos(limits_.maximum_tilt_rad));
      result.tilt_limited = true;
    }

    // 2. Tilt slew rate limit on body_z
    Eigen::Vector3d limited_body_z = requested_body_z;
    if (elapsed_seconds > 0.0 && limits_.maximum_tilt_rate_rad_s > 0.0) {
      const double dot_z = std::clamp(previous_body_z_.dot(requested_body_z), -1.0, 1.0);
      const double angle_z = std::acos(dot_z);
      const double max_tilt_change = limits_.maximum_tilt_rate_rad_s * elapsed_seconds;
      if (angle_z > max_tilt_change && angle_z > 1.0e-6) {
        const double t = max_tilt_change / angle_z;
        const double sin_angle = std::sin(angle_z);
        limited_body_z = (std::sin((1.0 - t) * angle_z) / sin_angle) * previous_body_z_ +
                         (std::sin(t * angle_z) / sin_angle) * requested_body_z;
        limited_body_z.normalize();
        result.tilt_limited = true;
      }
    }

    // 3. Yaw rate limit
    const double requested_yaw_rad = yawAngleRad(normalized_req);
    const double maximum_yaw_change =
      limits_.maximum_yaw_rate_rad_s * elapsed_seconds;
    const double yaw_change = std::clamp(
      shortestAngleRad(previous_yaw_rad_, requested_yaw_rad),
      -maximum_yaw_change, maximum_yaw_change);
    const double limited_yaw_rad = wrapAngleRad(previous_yaw_rad_ + yaw_change);
    result.yaw_limited =
      std::abs(shortestAngleRad(requested_yaw_rad, limited_yaw_rad)) > 1.0e-9;

    // 4. Combine into final attitude
    const Eigen::Quaterniond final_attitude = attitudeFromBodyZAndYaw(
      limited_body_z, limited_yaw_rad);
    result.attitude_body_flu_to_world_enu = final_attitude;

    // 5. Collective force rate limit
    const double bounded_collective = std::clamp(
      requested_collective_specific_force_m_s2,
      limits_.minimum_collective_specific_force_m_s2,
      limits_.maximum_collective_specific_force_m_s2);
    const double maximum_change =
      limits_.maximum_collective_rate_m_s3 * elapsed_seconds;
    result.collective_specific_force_m_s2 = std::clamp(
      bounded_collective,
      previous_collective_specific_force_m_s2_ - maximum_change,
      previous_collective_specific_force_m_s2_ + maximum_change);
    result.collective_limited =
      std::abs(result.collective_specific_force_m_s2 -
               requested_collective_specific_force_m_s2) > 1.0e-9;

    previous_collective_specific_force_m_s2_ =
      result.collective_specific_force_m_s2;
    previous_yaw_rad_ = limited_yaw_rad;
    previous_body_z_ = limited_body_z;
    result.valid = true;
    return result;
  }

private:
  Limits limits_{};
  double previous_collective_specific_force_m_s2_ = 9.80665;
  double previous_yaw_rad_ = 0.0;
  Eigen::Vector3d previous_body_z_{Eigen::Vector3d::UnitZ()};
};

}  // namespace mpc_controller::command_safety
