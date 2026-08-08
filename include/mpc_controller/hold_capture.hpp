#pragma once

#include <array>
#include <cmath>
#include <cstdint>

namespace mpc_controller::hold
{

enum class CaptureState
{
  waiting_for_state,
  hold_captured_state
};

struct Input
{
  std::array<double, 3> position{};
  double yaw = 0.0;
  std::uint64_t timestamp = 0;
  bool valid = false;
  bool control_ready = false;
  bool heading_valid = false;
  bool yaw_valid = false;
};

struct Snapshot
{
  std::array<double, 3> position{};
  double yaw = 0.0;
  std::uint64_t timestamp = 0;
};

class CaptureOnce final
{
public:
  bool tryCapture(const Input &input) noexcept
  {
    if (state_ != CaptureState::waiting_for_state || !eligible(input)) {
      return false;
    }

    snapshot_.position = input.position;
    snapshot_.yaw = input.yaw;
    snapshot_.timestamp = input.timestamp;
    state_ = CaptureState::hold_captured_state;
    return true;
  }

  CaptureState state() const noexcept {return state_;}
  bool captured() const noexcept {return state_ == CaptureState::hold_captured_state;}
  const Snapshot &snapshot() const noexcept {return snapshot_;}

private:
  static bool eligible(const Input &input) noexcept
  {
    return input.valid && input.control_ready && input.heading_valid && input.yaw_valid
      && input.timestamp != 0U && std::isfinite(input.yaw)
      && std::isfinite(input.position[0]) && std::isfinite(input.position[1])
      && std::isfinite(input.position[2]);
  }

  CaptureState state_ = CaptureState::waiting_for_state;
  Snapshot snapshot_{};
};

}  // namespace mpc_controller::hold
