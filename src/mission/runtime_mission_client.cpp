#include "mpc_controller/srv/load_and_start_mission.hpp"

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <iostream>
#include <string>

namespace {

constexpr auto kServiceName =
    "/reference_generator_node/load_and_start_mission";

} // namespace

int main(int argc, char **argv) {
  if (argc != 3) {
    std::cerr << "Usage: runtime_mission_client <mpc|px4_pid> <mission_path>\n";
    return 2;
  }

  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("runtime_mission_client");
  auto client =
      node->create_client<mpc_controller::srv::LoadAndStartMission>(kServiceName);

  int result = 5;
  if (!client->wait_for_service(std::chrono::seconds(5))) {
    std::cerr << "Mission service unavailable: " << kServiceName << '\n';
  } else {
    auto request = std::make_shared<mpc_controller::srv::LoadAndStartMission::Request>();
    request->controller = argv[1];
    request->mission_path = argv[2];
    auto future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, future,
                                           std::chrono::seconds(10)) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      std::cerr << "Mission service call timed out or failed\n";
    } else {
      const auto response = future.get();
      std::cout << response->message << '\n';
      result = response->success ? 0 : 4;
    }
  }

  rclcpp::shutdown();
  return result;
}
