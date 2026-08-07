#include "mpc_controller/msg/reference_trajectory.hpp"

#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <chrono>
#include <functional>

class ReferenceGeneratorNode final : public rclcpp::Node
{
public:
  ReferenceGeneratorNode()
  : Node("reference_generator_node")
  {
    declare_parameter("frame_id", frame_id_);
    declare_parameter("radius", radius_);
    declare_parameter("center_z", center_z_);
    declare_parameter("period_seconds", period_seconds_);
    declare_parameter("horizon_seconds", horizon_seconds_);
    declare_parameter("sample_period_seconds", sample_period_seconds_);
    declare_parameter("publish_rate_hz", publish_rate_hz_);
    get_parameter("frame_id", frame_id_);
    get_parameter("radius", radius_);
    get_parameter("center_z", center_z_);
    get_parameter("period_seconds", period_seconds_);
    get_parameter("horizon_seconds", horizon_seconds_);
    get_parameter("sample_period_seconds", sample_period_seconds_);
    get_parameter("publish_rate_hz", publish_rate_hz_);

    publisher_ = create_publisher<Msg>("reference_trajectory", 10);
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / publish_rate_hz_)),
        std::bind(&ReferenceGeneratorNode::publish, this));
  }

private:
  using Msg = mpc_controller::msg::ReferenceTrajectory;

  void publish()
  {
    constexpr double pi = 3.14159265358979323846;
    const double omega = 2.0 * pi / period_seconds_;
    const int count = static_cast<int>(horizon_seconds_ / sample_period_seconds_) + 1;
    Msg message;
    message.header.stamp = get_clock()->now();
    message.header.frame_id = frame_id_;
    message.trajectory_id = trajectory_id_++;
    message.hold_after_end = true;
    message.points.reserve(static_cast<std::size_t>(count));
    for (int i = 0; i < count; ++i) {
      const double t = static_cast<double>(i) * sample_period_seconds_;
      const double phase = omega * t;
      const double c = std::cos(phase);
      const double s = std::sin(phase);
      Msg::_points_type::value_type point;
      point.time_from_start.sec = static_cast<int32_t>(t);
      point.time_from_start.nanosec = static_cast<uint32_t>((t - point.time_from_start.sec) * 1.0e9);
      point.position = {radius_ * c, radius_ * s, center_z_};
      point.velocity = {-radius_ * omega * s, radius_ * omega * c, 0.0};
      point.acceleration = {
        -radius_ * omega * omega * c,
        -radius_ * omega * omega * s,
        0.0};
      point.yaw = std::atan2(point.velocity[1], point.velocity[0]);
      point.yaw_rate = omega;
      point.yaw_valid = true;
      message.points.push_back(point);
    }
    publisher_->publish(message);
  }

  std::string frame_id_ = "map";
  double radius_ = 2.0;
  double center_z_ = 1.0;
  double period_seconds_ = 60.0;
  double horizon_seconds_ = 30.0;
  double sample_period_seconds_ = 0.1;
  double publish_rate_hz_ = 1.0;
  uint64_t trajectory_id_ = 1;
  rclcpp::Publisher<Msg>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ReferenceGeneratorNode>());
  rclcpp::shutdown();
  return 0;
}
