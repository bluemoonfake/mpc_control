#pragma once

#include "mpc_controller/domain/control/mpc.hpp"
#include <Eigen/Core>
#include <limits>
#include <optional>

namespace mpc_controller::pid_validation {
using Reference = translational::ReferenceTrajectoryData; 
using Point = translational::ReferencePoint;
using ValidatedReference = translational::ValidatedTrajectory;

struct Sample {
  Eigen::Vector3f position, velocity, acceleration;
  float yaw, yaw_rate;
};

inline std::optional<Sample> sampleNed(const Reference &ref, double now, double receipt_age, double timeout) {
  //Valid input time source
  if (!std::isfinite(now) || !std::isfinite(receipt_age) ||
      !std::isfinite(timeout) || timeout <= 0 || receipt_age < 0 ||
      receipt_age > timeout || ref.header_time_seconds <= 0 ||
      !translational::ReferenceSampler::validTrajectory(ref)) return {};
    
  const double t = now - ref.header_time_seconds;
  //Valid input waypoint
  if (!std::isfinite(t) || t < ref.points.front().time_from_start ||
      t > timeout || (t > ref.points.back().time_from_start && !ref.hold_after_end))
    return {};
  Point p;
  if (!translational::ReferenceSampler::sampleAt(ref, t, p)) return {};
  const auto ned = [](const auto &v) {
    return Eigen::Vector3f(v[1], v[0], -v[2]);
  };
  const double yaw = 0.5 * M_PI - p.yaw;
  Sample out{ned(p.position), ned(p.velocity), ned(p.acceleration),
             static_cast<float>(std::atan2(std::sin(yaw), std::cos(yaw))),
             static_cast<float>(-p.yaw_rate)};
  if (!out.position.allFinite() || !out.velocity.allFinite() ||
      !out.acceleration.allFinite() || !std::isfinite(out.yaw) ||
      !std::isfinite(out.yaw_rate)) return {};
  return out;
}

inline std::optional<Sample> sampleNed(
  const ValidatedReference &validated, double now, double receipt_age, double timeout)
{
  if (!validated.valid()) return {};
  const auto &ref = validated.get();
  if (!std::isfinite(now) || !std::isfinite(receipt_age) ||
      !std::isfinite(timeout) || timeout <= 0 || receipt_age < 0 ||
      receipt_age > timeout || ref.header_time_seconds <= 0)
    return {};

  const double t = now - ref.header_time_seconds;
  if (!std::isfinite(t) || t < ref.points.front().time_from_start ||
      t > timeout || (t > ref.points.back().time_from_start && !ref.hold_after_end))
    return {};
  Point p;
  if (!translational::ReferenceSampler::sampleAt(validated, t, p)) return {};
  const auto ned = [](const auto &v) {
    return Eigen::Vector3f(v[1], v[0], -v[2]);
  };
  const double yaw = 0.5 * M_PI - p.yaw;
  Sample out{ned(p.position), ned(p.velocity), ned(p.acceleration),
             static_cast<float>(std::atan2(std::sin(yaw), std::cos(yaw))),
             static_cast<float>(-p.yaw_rate)};
  if (!out.position.allFinite() || !out.velocity.allFinite() ||
      !out.acceleration.allFinite() || !std::isfinite(out.yaw) ||
      !std::isfinite(out.yaw_rate)) return {};
  return out;
}
} // namespace mpc_controller::pid_validation
