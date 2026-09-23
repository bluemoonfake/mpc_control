#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace mpc_controller::mission
{

enum class AcceptancePolicy
{
  StopAndDwell,
  FlyThrough
};

class MissionProgressTracker final
{
public:
  struct Target
  {
    std::string id;
    std::array<double, 3> position{};
    double acceptance_radius_m = 2.5;
    double hold_duration_s = 0.0;
    double horizontal_speed_m_s = 0.0;
    double vertical_speed_m_s = 0.0;
    double heading_rad = std::numeric_limits<double>::quiet_NaN();
    double max_heading_rate_rad_s = 0.0;
    double maximum_acceleration_m_s2 = std::numeric_limits<double>::quiet_NaN();
    double maximum_jerk_m_s3 = std::numeric_limits<double>::quiet_NaN();
    AcceptancePolicy acceptance_policy = AcceptancePolicy::StopAndDwell;
  };

  struct VehicleState
  {
    std::array<double, 3> position{};
    std::array<double, 3> velocity{};
    double yaw = 0.0;
  };

  struct Goal
  {
    Target target;
    double yaw = 0.0;
    double yaw_rate_rad_s = 0.0;
    std::size_t index = 0;
    std::size_t count = 0;
  };

  struct Update
  {
    bool active = false;
    bool target_accepted = false;
    bool completed_now = false;
    bool completed = false;
    std::size_t reached_index = 0;
    Target reached_target;
    Goal goal;
  };

  explicit MissionProgressTracker(double acceptance_speed_m_s)
  : acceptance_speed_m_s_(acceptance_speed_m_s)
  {
  }

  bool load(std::vector<Target> targets, const VehicleState &initial_state)
  {
    if (!validTargets(targets) || !finite(initial_state.position) ||
      !finite(initial_state.velocity) || !std::isfinite(initial_state.yaw)) {
      return false;
    }
    targets_ = std::move(targets);
    index_ = 0;
    completed_ = false;
    dwell_started_at_s_ = std::numeric_limits<double>::quiet_NaN();
    leg_start_position_ = initial_state.position;
    yaw_ = initial_state.yaw;
    last_update_at_s_ = std::numeric_limits<double>::quiet_NaN();
    return true;
  }

  void cancel() noexcept
  {
    targets_.clear();
    index_ = 0;
    completed_ = false;
    dwell_started_at_s_ = std::numeric_limits<double>::quiet_NaN();
  }

  bool active() const noexcept
  {
    return !targets_.empty();
  }

  std::size_t currentIndex() const noexcept
  {
    return index_;
  }

  Update update(const VehicleState &state, double now_seconds)
  {
    Update output;
    if (!active() || !finite(state.position) || !finite(state.velocity) ||
      !std::isfinite(state.yaw) || !std::isfinite(now_seconds)) {
      return output;
    }

    const auto &target = targets_[index_];
    if (!completed_ && accepted(state, target)) {
      if (!std::isfinite(dwell_started_at_s_)) {
        dwell_started_at_s_ = now_seconds;
      }
      if (now_seconds - dwell_started_at_s_ >= target.hold_duration_s) {
        output.target_accepted = true;
        output.reached_index = index_;
        output.reached_target = target;
        advance(target, now_seconds, output);
      }
    } else {
      dwell_started_at_s_ = std::numeric_limits<double>::quiet_NaN();
    }

    output.active = true;
    output.completed = completed_;
    output.goal = currentGoal(now_seconds);
    return output;
  }

private:
  static bool finite(const std::array<double, 3> &values) noexcept
  {
    for (const double value : values) {
      if (!std::isfinite(value)) {
        return false;
      }
    }
    return true;
  }

  static bool validTargets(const std::vector<Target> &targets) noexcept
  {
    if (targets.empty()) {
      return false;
    }
    for (const auto &target : targets) {
      if (!finite(target.position) ||
        !std::isfinite(target.acceptance_radius_m) ||
        target.acceptance_radius_m <= 0.0 ||
        !std::isfinite(target.hold_duration_s) ||
        target.hold_duration_s < 0.0 ||
        !std::isfinite(target.horizontal_speed_m_s) ||
        target.horizontal_speed_m_s < 0.0 ||
        !std::isfinite(target.vertical_speed_m_s) ||
        target.vertical_speed_m_s < 0.0 ||
        !std::isfinite(target.max_heading_rate_rad_s) ||
        target.max_heading_rate_rad_s <= 0.0 ||
        (!std::isnan(target.maximum_acceleration_m_s2) &&
        (!std::isfinite(target.maximum_acceleration_m_s2) ||
        target.maximum_acceleration_m_s2 <= 0.0)) ||
        (!std::isnan(target.maximum_jerk_m_s3) &&
        (!std::isfinite(target.maximum_jerk_m_s3) ||
        target.maximum_jerk_m_s3 <= 0.0)) ||
        (target.acceptance_policy == AcceptancePolicy::FlyThrough &&
         target.hold_duration_s > 0.0)) {
        return false;
      }
    }
    return true;
  }

  bool accepted(const VehicleState &state, const Target &target) const noexcept
  {
    const double dx = state.position[0] - target.position[0];
    const double dy = state.position[1] - target.position[1];
    const double dz = state.position[2] - target.position[2];
    const double speed = std::sqrt(state.velocity[0] * state.velocity[0] +
      state.velocity[1] * state.velocity[1] + state.velocity[2] * state.velocity[2]);
    const bool inside_radius =
      std::sqrt(dx * dx + dy * dy + dz * dz) <= target.acceptance_radius_m;
    if (target.acceptance_policy == AcceptancePolicy::FlyThrough) {
      return inside_radius;
    }
    return inside_radius && speed <= acceptance_speed_m_s_;
  }

  void advance(const Target &reached, double now_seconds, Update &output)
  {
    dwell_started_at_s_ = std::numeric_limits<double>::quiet_NaN();
    if (index_ + 1U == targets_.size()) {
      completed_ = true;
      output.completed_now = true;
      return;
    }
    leg_start_position_ = reached.position;
    ++index_;
    last_update_at_s_ = now_seconds;
  }

  Goal currentGoal(double now_seconds)
  {
    const auto &target = targets_[index_];
    const double desired_yaw = std::isfinite(target.heading_rad)
      ? target.heading_rad : courseYaw(target.position);
    const double dt = std::isfinite(last_update_at_s_)
      ? std::max(0.0, now_seconds - last_update_at_s_) : 0.0;
    const double yaw_error = shortestAngle(yaw_, desired_yaw);
    const double max_step = target.max_heading_rate_rad_s * dt;
    const double yaw_step = std::clamp(yaw_error, -max_step, max_step);
    const double yaw_rate = dt > 0.0 ? yaw_step / dt : 0.0;
    yaw_ = normalizeAngle(yaw_ + yaw_step);
    last_update_at_s_ = now_seconds;
    return {target, yaw_, yaw_rate, index_, targets_.size()};
  }

  double courseYaw(const std::array<double, 3> &target_position) const noexcept
  {
    const double dx = target_position[0] - leg_start_position_[0];
    const double dy = target_position[1] - leg_start_position_[1];
    return std::hypot(dx, dy) > 1.0e-3 ? std::atan2(dy, dx) : yaw_;
  }

  static double shortestAngle(double from, double to) noexcept
  {
    return std::atan2(std::sin(to - from), std::cos(to - from));
  }

  static double normalizeAngle(double yaw) noexcept
  {
    return std::atan2(std::sin(yaw), std::cos(yaw));
  }

  double acceptance_speed_m_s_ = 0.5;
  std::vector<Target> targets_;
  std::size_t index_ = 0;
  bool completed_ = false;
  double dwell_started_at_s_ = std::numeric_limits<double>::quiet_NaN();
  std::array<double, 3> leg_start_position_{};
  double yaw_ = 0.0;
  double last_update_at_s_ = std::numeric_limits<double>::quiet_NaN();
};

}  // namespace mpc_controller::mission
