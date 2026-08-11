#pragma once

#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>

namespace mpc_controller::hold
{

enum class CaptureState : std::uint8_t
{
  waiting_for_readiness = 0,
  ready_not_stable = 1,
  stable_dwell = 2,
  captured = 3
};

struct Thresholds
{
  // ENTER limits are intentionally strict and are used by explicit hold
  // capture. EXIT limits are only for the stateful pre-handover stability
  // gate; they must be wider than ENTER limits and do not relax controller
  // health, freshness, finite-value, envelope, or saturation checks.
  double max_velocity_xy_m_s = 0.032;
  double max_velocity_z_m_s = 0.18;
  double max_acceleration_xy_m_s2 = 0.080;
  double max_acceleration_z_m_s2 = 0.70;
  double max_body_rate_xy_rad_s = 0.045;
  double max_body_rate_z_rad_s = 0.030;
  double max_roll_rad = 0.0065;
  double max_pitch_rad = 0.0070;
  double dwell_seconds = 2.0;

  // Derived from the M5.2D stock-hover window. v_xy EXIT is 0.0353 m/s,
  // above the observed 0.033547 m/s maximum. Other EXIT values are 1.1x
  // their strict ENTER limits. A 0.20 s violation dwell is ten samples at
  // the nominal 50 Hz status rate.
  double exit_max_velocity_xy_m_s = 0.0353;
  double exit_max_velocity_z_m_s = 0.198;
  double exit_max_acceleration_xy_m_s2 = 0.088;
  double exit_max_acceleration_z_m_s2 = 0.77;
  double exit_max_body_rate_xy_rad_s = 0.0495;
  double exit_max_body_rate_z_rad_s = 0.033;
  double exit_max_roll_rad = 0.00715;
  double exit_max_pitch_rad = 0.0077;
  double exit_dwell_seconds = 0.20;
};

struct StabilityMetrics
{
  double velocity_xy_m_s = 0.0;
  double velocity_z_m_s = 0.0;
  double acceleration_xy_m_s2 = 0.0;
  double acceleration_z_m_s2 = 0.0;
  double body_rate_xy_rad_s = 0.0;
  double body_rate_z_rad_s = 0.0;
  double roll_rad = 0.0;
  double pitch_rad = 0.0;
};

struct Input
{
  std::array<double, 3> position{};
  std::array<double, 3> velocity{};
  std::array<double, 3> acceleration{};
  // Quaternion order is [w, x, y, z], body FLU to world ENU.
  std::array<double, 4> attitude_wxyz{1.0, 0.0, 0.0, 0.0};
  std::array<double, 3> body_rate{};
  double yaw = 0.0;
  std::uint64_t timestamp = 0;
  bool valid = false;
  bool fresh = false;
  bool control_ready = false;
  bool heading_valid = false;
  bool yaw_valid = false;
  bool attitude_valid = false;
  bool body_rate_valid = false;
};

struct Snapshot
{
  std::array<double, 3> position{};
  std::array<double, 3> velocity{};
  std::array<double, 3> acceleration{};
  std::array<double, 4> attitude_wxyz{1.0, 0.0, 0.0, 0.0};
  std::array<double, 3> body_rate{};
  double yaw = 0.0;
  std::uint64_t timestamp = 0;
  StabilityMetrics stability{};
};

inline bool finiteArray(const std::array<double, 3> &value) noexcept
{
  for (const double element : value) {
    if (!std::isfinite(element)) {
      return false;
    }
  }
  return true;
}

inline bool finiteArray(const std::array<double, 4> &value) noexcept
{
  for (const double element : value) {
    if (!std::isfinite(element)) {
      return false;
    }
  }
  return true;
}

inline bool validThresholds(const Thresholds &thresholds) noexcept
{
  return std::isfinite(thresholds.max_velocity_xy_m_s) &&
         thresholds.max_velocity_xy_m_s > 0.0 &&
         std::isfinite(thresholds.max_velocity_z_m_s) && thresholds.max_velocity_z_m_s > 0.0 &&
         std::isfinite(thresholds.max_acceleration_xy_m_s2) &&
         thresholds.max_acceleration_xy_m_s2 > 0.0 &&
         std::isfinite(thresholds.max_acceleration_z_m_s2) &&
         thresholds.max_acceleration_z_m_s2 > 0.0 &&
         std::isfinite(thresholds.max_body_rate_xy_rad_s) &&
         thresholds.max_body_rate_xy_rad_s > 0.0 &&
         std::isfinite(thresholds.max_body_rate_z_rad_s) &&
         thresholds.max_body_rate_z_rad_s > 0.0 &&
         std::isfinite(thresholds.max_roll_rad) && thresholds.max_roll_rad > 0.0 &&
         std::isfinite(thresholds.max_pitch_rad) && thresholds.max_pitch_rad > 0.0 &&
         std::isfinite(thresholds.dwell_seconds) && thresholds.dwell_seconds > 0.0 &&
         std::isfinite(thresholds.exit_max_velocity_xy_m_s) &&
         thresholds.exit_max_velocity_xy_m_s > thresholds.max_velocity_xy_m_s &&
         std::isfinite(thresholds.exit_max_velocity_z_m_s) &&
         thresholds.exit_max_velocity_z_m_s > thresholds.max_velocity_z_m_s &&
         std::isfinite(thresholds.exit_max_acceleration_xy_m_s2) &&
         thresholds.exit_max_acceleration_xy_m_s2 > thresholds.max_acceleration_xy_m_s2 &&
         std::isfinite(thresholds.exit_max_acceleration_z_m_s2) &&
         thresholds.exit_max_acceleration_z_m_s2 > thresholds.max_acceleration_z_m_s2 &&
         std::isfinite(thresholds.exit_max_body_rate_xy_rad_s) &&
         thresholds.exit_max_body_rate_xy_rad_s > thresholds.max_body_rate_xy_rad_s &&
         std::isfinite(thresholds.exit_max_body_rate_z_rad_s) &&
         thresholds.exit_max_body_rate_z_rad_s > thresholds.max_body_rate_z_rad_s &&
         std::isfinite(thresholds.exit_max_roll_rad) &&
         thresholds.exit_max_roll_rad > thresholds.max_roll_rad &&
         std::isfinite(thresholds.exit_max_pitch_rad) &&
         thresholds.exit_max_pitch_rad > thresholds.max_pitch_rad &&
         std::isfinite(thresholds.exit_dwell_seconds) && thresholds.exit_dwell_seconds > 0.0;
}

inline bool attitudeRollPitch(
  const std::array<double, 4> &q, double &roll, double &pitch) noexcept
{
  if (!finiteArray(q)) {
    return false;
  }
  const double norm = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  if (!std::isfinite(norm) || norm < 1.0e-9) {
    return false;
  }
  const double w = q[0] / norm;
  const double x = q[1] / norm;
  const double y = q[2] / norm;
  const double z = q[3] / norm;
  roll = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
  const double sin_pitch = std::clamp(2.0 * (w * y - z * x), -1.0, 1.0);
  pitch = std::asin(sin_pitch);
  return std::isfinite(roll) && std::isfinite(pitch);
}

inline bool ready(const Input &input) noexcept
{
  return input.valid && input.fresh && input.control_ready && input.heading_valid &&
         input.yaw_valid && input.attitude_valid && input.body_rate_valid &&
         input.timestamp != 0U && std::isfinite(input.yaw) && finiteArray(input.position) &&
         finiteArray(input.velocity) && finiteArray(input.acceleration) &&
         finiteArray(input.attitude_wxyz) && finiteArray(input.body_rate);
}

inline bool measure(const Input &input, StabilityMetrics &metrics) noexcept
{
  if (!ready(input)) {
    return false;
  }
  double roll = 0.0;
  double pitch = 0.0;
  if (!attitudeRollPitch(input.attitude_wxyz, roll, pitch)) {
    return false;
  }
  metrics.velocity_xy_m_s = std::hypot(input.velocity[0], input.velocity[1]);
  metrics.velocity_z_m_s = std::abs(input.velocity[2]);
  metrics.acceleration_xy_m_s2 = std::hypot(input.acceleration[0], input.acceleration[1]);
  metrics.acceleration_z_m_s2 = std::abs(input.acceleration[2]);
  metrics.body_rate_xy_rad_s = std::hypot(input.body_rate[0], input.body_rate[1]);
  metrics.body_rate_z_rad_s = std::abs(input.body_rate[2]);
  metrics.roll_rad = std::abs(roll);
  metrics.pitch_rad = std::abs(pitch);
  return std::isfinite(metrics.velocity_xy_m_s) && std::isfinite(metrics.velocity_z_m_s) &&
         std::isfinite(metrics.acceleration_xy_m_s2) && std::isfinite(metrics.acceleration_z_m_s2) &&
         std::isfinite(metrics.body_rate_xy_rad_s) && std::isfinite(metrics.body_rate_z_rad_s) &&
         std::isfinite(metrics.roll_rad) && std::isfinite(metrics.pitch_rad);
}

inline bool stable(const StabilityMetrics &metrics, const Thresholds &thresholds) noexcept
{
  return validThresholds(thresholds) &&
         metrics.velocity_xy_m_s <= thresholds.max_velocity_xy_m_s &&
         metrics.velocity_z_m_s <= thresholds.max_velocity_z_m_s &&
         metrics.acceleration_xy_m_s2 <= thresholds.max_acceleration_xy_m_s2 &&
         metrics.acceleration_z_m_s2 <= thresholds.max_acceleration_z_m_s2 &&
         metrics.body_rate_xy_rad_s <= thresholds.max_body_rate_xy_rad_s &&
         metrics.body_rate_z_rad_s <= thresholds.max_body_rate_z_rad_s &&
         metrics.roll_rad <= thresholds.max_roll_rad &&
         metrics.pitch_rad <= thresholds.max_pitch_rad;
}

inline bool withinExitBand(
  const StabilityMetrics &metrics, const Thresholds &thresholds) noexcept
{
  return validThresholds(thresholds) &&
         metrics.velocity_xy_m_s <= thresholds.exit_max_velocity_xy_m_s &&
         metrics.velocity_z_m_s <= thresholds.exit_max_velocity_z_m_s &&
         metrics.acceleration_xy_m_s2 <= thresholds.exit_max_acceleration_xy_m_s2 &&
         metrics.acceleration_z_m_s2 <= thresholds.exit_max_acceleration_z_m_s2 &&
         metrics.body_rate_xy_rad_s <= thresholds.exit_max_body_rate_xy_rad_s &&
         metrics.body_rate_z_rad_s <= thresholds.exit_max_body_rate_z_rad_s &&
         metrics.roll_rad <= thresholds.exit_max_roll_rad &&
         metrics.pitch_rad <= thresholds.exit_max_pitch_rad;
}

// Stateful stability used only before ownership transfer. CaptureOnce below
// remains strict: a capture still requires ENTER limits and a continuous
// strict dwell. Once this gate is stable, a short ENTER-band excursion does
// not revoke handover stability; an EXIT-band violation must persist for the
// configured exit dwell before it does.
class HysteresisGate final
{
public:
  explicit HysteresisGate(const Thresholds &thresholds = {}) noexcept
  : thresholds_(thresholds) {}

  void update(double now_seconds, const Input &input) noexcept
  {
    StabilityMetrics metrics;
    const bool ready = measure(input, metrics) && validThresholds(thresholds_) &&
      std::isfinite(now_seconds);
    if (!ready) {
      stable_ = false;
      enter_since_seconds_.reset();
      exit_since_seconds_.reset();
      return;
    }

    if (!stable_) {
      exit_since_seconds_.reset();
      if (!mpc_controller::hold::stable(metrics, thresholds_)) {
        enter_since_seconds_.reset();
        return;
      }
      if (!enter_since_seconds_) {
        enter_since_seconds_ = now_seconds;
      }
      if (now_seconds - *enter_since_seconds_ >= thresholds_.dwell_seconds) {
        stable_ = true;
      }
      return;
    }

    enter_since_seconds_.reset();
    if (mpc_controller::hold::withinExitBand(metrics, thresholds_)) {
      exit_since_seconds_.reset();
      return;
    }
    if (!exit_since_seconds_) {
      exit_since_seconds_ = now_seconds;
    }
    if (now_seconds - *exit_since_seconds_ >= thresholds_.exit_dwell_seconds) {
      stable_ = false;
      exit_since_seconds_.reset();
    }
  }

  bool stable() const noexcept {return stable_;}
  const Thresholds &thresholds() const noexcept {return thresholds_;}

private:
  Thresholds thresholds_{};
  std::optional<double> enter_since_seconds_;
  std::optional<double> exit_since_seconds_;
  bool stable_ = false;
};

class CaptureOnce final
{
public:
  explicit CaptureOnce(const Thresholds &thresholds = {}) noexcept
  : thresholds_(thresholds) {}

  void update(double now_seconds, const Input &input) noexcept
  {
    latest_input_ = input;
    StabilityMetrics current_metrics;
    current_ready_ = measure(input, current_metrics);
    current_stable_ = current_ready_ && stable(current_metrics, thresholds_);
    if (current_ready_) {
      latest_metrics_ = current_metrics;
    } else {
      latest_metrics_.reset();
    }
    if (state_ == CaptureState::captured) {
      return;
    }
    if (!validThresholds(thresholds_) || !ready(input)) {
      state_ = CaptureState::waiting_for_readiness;
      stable_since_seconds_.reset();
      latest_metrics_.reset();
      return;
    }

    StabilityMetrics metrics = current_metrics;
    if (!current_ready_ || !current_stable_) {
      state_ = CaptureState::ready_not_stable;
      stable_since_seconds_.reset();
      latest_metrics_ = metrics;
      return;
    }

    latest_metrics_ = metrics;
    if (!std::isfinite(now_seconds)) {
      state_ = CaptureState::ready_not_stable;
      stable_since_seconds_.reset();
      return;
    }
    if (!stable_since_seconds_) {
      stable_since_seconds_ = now_seconds;
    }
    state_ = CaptureState::stable_dwell;
  }

  bool requestCapture(double now_seconds) noexcept
  {
    if (state_ != CaptureState::stable_dwell || !stable_since_seconds_ || !latest_input_) {
      return false;
    }
    const double dwell = now_seconds - *stable_since_seconds_;
    if (!std::isfinite(dwell) || dwell < thresholds_.dwell_seconds || !latest_metrics_) {
      return false;
    }
    snapshot_.position = latest_input_->position;
    snapshot_.velocity = latest_input_->velocity;
    snapshot_.acceleration = latest_input_->acceleration;
    snapshot_.attitude_wxyz = latest_input_->attitude_wxyz;
    snapshot_.body_rate = latest_input_->body_rate;
    snapshot_.yaw = latest_input_->yaw;
    snapshot_.timestamp = latest_input_->timestamp;
    snapshot_.stability = *latest_metrics_;
    state_ = CaptureState::captured;
    return true;
  }

  CaptureState state() const noexcept {return state_;}
  bool captured() const noexcept {return state_ == CaptureState::captured;}
  bool currentReady() const noexcept {return current_ready_;}
  bool currentStable() const noexcept {return current_stable_;}
  bool dwellComplete(double now_seconds) const noexcept
  {
    return state_ == CaptureState::stable_dwell && stable_since_seconds_ &&
           std::isfinite(now_seconds) &&
           now_seconds - *stable_since_seconds_ >= thresholds_.dwell_seconds;
  }
  double dwellSeconds(double now_seconds) const noexcept
  {
    if (!stable_since_seconds_ || !std::isfinite(now_seconds)) {
      return 0.0;
    }
    return std::max(0.0, now_seconds - *stable_since_seconds_);
  }
  const Snapshot &snapshot() const noexcept {return snapshot_;}
  const std::optional<StabilityMetrics> &latestMetrics() const noexcept {return latest_metrics_;}
  const Thresholds &thresholds() const noexcept {return thresholds_;}

private:
  Thresholds thresholds_{};
  CaptureState state_ = CaptureState::waiting_for_readiness;
  std::optional<double> stable_since_seconds_;
  std::optional<Input> latest_input_;
  std::optional<StabilityMetrics> latest_metrics_;
  bool current_ready_ = false;
  bool current_stable_ = false;
  Snapshot snapshot_{};
};

inline const char *stateName(CaptureState state) noexcept
{
  switch (state) {
    case CaptureState::waiting_for_readiness: return "WAITING_FOR_READINESS";
    case CaptureState::ready_not_stable: return "READY_NOT_STABLE";
    case CaptureState::stable_dwell: return "STABLE_DWELL";
    case CaptureState::captured: return "CAPTURED";
  }
  return "UNKNOWN";
}

}  // namespace mpc_controller::hold
