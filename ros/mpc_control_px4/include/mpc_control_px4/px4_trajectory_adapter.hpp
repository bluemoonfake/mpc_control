#pragma once

#include <mpc_control_msgs/msg/trajectory_command.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>

#include <cstdint>
#include <optional>
#include <string>

namespace mpc_control_px4
{

enum class Px4AdapterFailureReason : std::uint8_t
{
  None = 0,
  InvalidFrame,
  InvalidTimestamp,
  NonFiniteTranslationalCommand,
  NonFiniteYawCommand,
  TimestampNotMonotonic,
};

struct Px4TrajectoryAdapterConfig
{
  std::string input_frame_id = "map";
};

struct Px4TrajectoryAdapterResult
{
  px4_msgs::msg::TrajectorySetpoint setpoint{};
  bool valid = false;
  Px4AdapterFailureReason failure_reason = Px4AdapterFailureReason::None;
  const char* detail = "";
};

class Px4TrajectoryAdapter final
{
public:
  explicit Px4TrajectoryAdapter(
      Px4TrajectoryAdapterConfig configuration = {});

  Px4TrajectoryAdapterResult convert(
      const mpc_control_msgs::msg::TrajectoryCommand& command,
      std::uint64_t px4_timestamp_us) noexcept;

  void reset() noexcept;

  const Px4TrajectoryAdapterConfig& configuration() const noexcept
  {
    return configuration_;
  }

private:
  static bool finiteFloat(double value) noexcept;
  static double wrapPi(double angle) noexcept;
  static float toFloat(double value) noexcept;

  Px4TrajectoryAdapterConfig configuration_;
  std::optional<std::uint64_t> last_timestamp_us_;
};

}  // namespace mpc_control_px4
