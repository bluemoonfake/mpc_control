#include "mpc_controller/msg/reference_trajectory.hpp"
#include "mpc_controller/msg/trajectory_point.hpp"
#include "mpc_controller/msg/vehicle_state.hpp"
#include "mpc_controller/hold_capture.hpp"
#include "mpc_controller/reference_model.hpp"

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

class ReferenceGeneratorNode final : public rclcpp::Node
{
public:
  ReferenceGeneratorNode()
  : Node("reference_generator_node")
  {
    declare_parameter("trajectory_type", parameters_.type);
    declare_parameter("frame_id", frame_id_);
    declare_parameter("hold_position", std::vector<double>{0.0, 0.0, 1.0});
    declare_parameter("line_start", std::vector<double>{0.0, 0.0, 1.0});
    declare_parameter("line_end", std::vector<double>{1.0, 0.0, 1.0});
    declare_parameter("line_duration_seconds", parameters_.line_duration_seconds);
    declare_parameter("circle_center", std::vector<double>{0.0, 0.0, 1.0});
    declare_parameter("circle_radius", parameters_.circle_radius);
    declare_parameter("circle_period_seconds", parameters_.circle_period_seconds);
    declare_parameter("circle_phase_rad", parameters_.circle_phase_rad);
    declare_parameter("circle_direction", parameters_.circle_direction);
    declare_parameter("hold_yaw_rad", parameters_.hold_yaw_rad);
    declare_parameter("hold_current_state_on_enable", false);
    declare_parameter("horizon_seconds", horizon_seconds_);
    declare_parameter("sample_period_seconds", sample_period_seconds_);
    declare_parameter("publish_rate_hz", publish_rate_hz_);

    get_parameter("trajectory_type", parameters_.type);
    get_parameter("frame_id", frame_id_);
    valid_config_ = true;
    getVectorParameter("hold_position", parameters_.hold_position);
    getVectorParameter("line_start", parameters_.line_start);
    getVectorParameter("line_end", parameters_.line_end);
    get_parameter("line_duration_seconds", parameters_.line_duration_seconds);
    getVectorParameter("circle_center", parameters_.circle_center);
    get_parameter("circle_radius", parameters_.circle_radius);
    get_parameter("circle_period_seconds", parameters_.circle_period_seconds);
    get_parameter("circle_phase_rad", parameters_.circle_phase_rad);
    get_parameter("circle_direction", parameters_.circle_direction);
    get_parameter("hold_yaw_rad", parameters_.hold_yaw_rad);
    get_parameter("hold_current_state_on_enable", hold_current_state_on_enable_);
    get_parameter("horizon_seconds", horizon_seconds_);
    get_parameter("sample_period_seconds", sample_period_seconds_);
    get_parameter("publish_rate_hz", publish_rate_hz_);

    valid_config_ = valid_config_ && mpc_controller::reference::valid(parameters_)
      && std::isfinite(horizon_seconds_) && horizon_seconds_ >= 0.0
      && std::isfinite(sample_period_seconds_) && sample_period_seconds_ > 0.0
      && std::isfinite(publish_rate_hz_) && publish_rate_hz_ > 0.0
      && !frame_id_.empty();
    if (!valid_config_) {
      RCLCPP_ERROR(get_logger(), "Invalid reference generator parameters; publishing disabled");
    } else {
      publisher_ = create_publisher<Reference>("reference_trajectory", 10);
      if (hold_current_state_on_enable_) {
        state_subscription_ = create_subscription<State>(
          "vehicle_state", rclcpp::QoS(10),
          std::bind(&ReferenceGeneratorNode::stateCallback, this, std::placeholders::_1));
        RCLCPP_INFO(
          get_logger(),
          "Current-state hold enabled; waiting for a valid control-ready VehicleState");
      }
      timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(1.0 / publish_rate_hz_)),
        std::bind(&ReferenceGeneratorNode::publish, this));
    }
  }

private:
  using Reference = mpc_controller::msg::ReferenceTrajectory;
  using Point = mpc_controller::msg::TrajectoryPoint;
  using State = mpc_controller::msg::VehicleState;

  void stateCallback(const State::SharedPtr message)
  {
    if (!message) {
      return;
    }

    const auto timestamp = rclcpp::Time(message->header.stamp).nanoseconds();
    if (timestamp <= 0) {
      return;
    }

    mpc_controller::hold::Input input;
    input.position = {message->position[0], message->position[1], message->position[2]};
    input.yaw = message->yaw;
    input.timestamp = static_cast<std::uint64_t>(timestamp);
    input.valid = message->valid;
    input.control_ready = message->control_ready;
    input.heading_valid = message->heading_valid;
    input.yaw_valid = message->yaw_valid;
    if (!hold_capture_.tryCapture(input)) {
      return;
    }

    const auto &snapshot = hold_capture_.snapshot();
    parameters_.type = "hold";
    parameters_.hold_position = snapshot.position;
    parameters_.hold_yaw_rad = snapshot.yaw;
    hold_reference_captured_ = true;
    RCLCPP_INFO(
      get_logger(),
      "Captured current-state hold reference exactly once at position=[%.3f %.3f %.3f], "
      "yaw=%.3f rad, state_timestamp=%lu",
      parameters_.hold_position[0], parameters_.hold_position[1],
      parameters_.hold_position[2], parameters_.hold_yaw_rad,
      static_cast<unsigned long>(snapshot.timestamp));
  }

  void getVectorParameter(const std::string &name, std::array<double, 3> &output)
  {
    const auto value = get_parameter(name).as_double_array();
    if (value.size() == 3) {
      std::copy(value.begin(), value.end(), output.begin());
    } else {
      valid_config_ = false;
      RCLCPP_ERROR(get_logger(), "Parameter '%s' must contain exactly 3 values", name.c_str());
    }
  }

  static builtin_interfaces::msg::Duration durationMessage(uint64_t nanoseconds)
  {
    builtin_interfaces::msg::Duration duration;
    duration.sec = static_cast<int32_t>(nanoseconds / 1000000000ULL);
    duration.nanosec = static_cast<uint32_t>(nanoseconds % 1000000000ULL);
    return duration;
  }

  void publish()
  {
    if (hold_current_state_on_enable_ && !hold_reference_captured_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Current-state hold waiting for VehicleState.control_ready=true");
      return;
    }
    const uint64_t sample_ns = static_cast<uint64_t>(std::llround(sample_period_seconds_ * 1.0e9));
    const uint64_t horizon_ns = static_cast<uint64_t>(std::llround(horizon_seconds_ * 1.0e9));
    if (sample_ns == 0) {
      return;
    }

    Reference message;
    message.header.stamp = get_clock()->now();
    message.header.frame_id = frame_id_;
    message.trajectory_id = trajectory_id_++;
    message.hold_after_end = true;
    const uint64_t count = horizon_ns / sample_ns + 1;
    message.points.reserve(static_cast<std::size_t>(count));
    for (uint64_t index = 0; index < count; ++index) {
      const double time = static_cast<double>(index * sample_ns) * 1.0e-9;
      mpc_controller::reference::Sample sample;
      if (!mpc_controller::reference::sample(parameters_, time, sample)) {
        RCLCPP_ERROR(get_logger(), "Reference sample failed at t=%.6f", time);
        return;
      }
      Point point;
      point.time_from_start = durationMessage(index * sample_ns);
      point.position = sample.position;
      point.velocity = sample.velocity;
      point.acceleration = sample.acceleration;
      point.yaw = sample.yaw;
      point.yaw_rate = sample.yaw_rate;
      point.yaw_valid = true;
      message.points.push_back(point);
    }
    publisher_->publish(message);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "ReferenceTrajectory published id=%lu points=%lu hold_capture=%s position=[%.3f %.3f %.3f] yaw=%.3f",
      static_cast<unsigned long>(message.trajectory_id),
      static_cast<unsigned long>(message.points.size()),
      hold_reference_captured_ ? "yes" : "no",
      parameters_.hold_position[0], parameters_.hold_position[1],
      parameters_.hold_position[2], parameters_.hold_yaw_rad);
  }

  mpc_controller::reference::Parameters parameters_{};
  std::string frame_id_ = "map";
  double horizon_seconds_ = 30.0;
  double sample_period_seconds_ = 0.1;
  double publish_rate_hz_ = 1.0;
  bool hold_current_state_on_enable_ = false;
  bool hold_reference_captured_ = false;
  mpc_controller::hold::CaptureOnce hold_capture_;
  bool valid_config_ = false;
  uint64_t trajectory_id_ = 1;
  rclcpp::Publisher<Reference>::SharedPtr publisher_;
  rclcpp::Subscription<State>::SharedPtr state_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ReferenceGeneratorNode>());
  rclcpp::shutdown();
  return 0;
}
