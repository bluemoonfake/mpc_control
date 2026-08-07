#include "mpc_control_px4/px4_trajectory_setpoint_publisher.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<mpc_control_px4::Px4TrajectorySetpointPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
