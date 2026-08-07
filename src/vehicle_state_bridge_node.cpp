#include "mpc_controller/msg/vehicle_state.hpp"

#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <functional>

class VehicleStateBridgeNode final : public rclcpp::Node
{
public:
  VehicleStateBridgeNode()
  : Node("vehicle_state_bridge_node")
  {
    state_publisher_ = create_publisher<State>("vehicle_state", 10);
    subscription_ = create_subscription<Px4State>(
        "fmu/out/vehicle_local_position_v1",
        rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
        std::bind(&VehicleStateBridgeNode::callback, this, std::placeholders::_1));
  }

private:
  using State = mpc_controller::msg::VehicleState;
  using Px4State = px4_msgs::msg::VehicleLocalPosition;

  void callback(const Px4State::SharedPtr message)
  {
    if (!message || !std::isfinite(message->x) || !std::isfinite(message->y)
        || !std::isfinite(message->z) || !std::isfinite(message->vx)
        || !std::isfinite(message->vy) || !std::isfinite(message->vz)
        || !std::isfinite(message->ax) || !std::isfinite(message->ay)
        || !std::isfinite(message->az) || !std::isfinite(message->heading)) {
      return;
    }
    State state;
    state.header.stamp = get_clock()->now();
    state.header.frame_id = "map";
    state.position = {message->y, message->x, -message->z};
    state.velocity = {message->vy, message->vx, -message->vz};
    state.acceleration = {message->ay, message->ax, -message->az};
    state.yaw = 1.5707963267948966 - message->heading;
    state.yaw_rate = 0.0;
    state.position_valid = message->xy_valid && message->z_valid;
    state.velocity_valid = message->v_xy_valid && message->v_z_valid;
    state.acceleration_valid = true;
    state.yaw_valid = message->heading_good_for_control;
    state.valid = state.position_valid && state.velocity_valid && state.yaw_valid;
    state.reset_counter = message->xy_reset_counter;
    state.reset_counter_valid = true;
    state_publisher_->publish(state);
  }

  rclcpp::Subscription<Px4State>::SharedPtr subscription_;
  rclcpp::Publisher<State>::SharedPtr state_publisher_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VehicleStateBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
