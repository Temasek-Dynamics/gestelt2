#include <gestelt_interfaces/srv/uav_command.hpp>

#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("uav_command_client");
  rclcpp::Client<gestelt_interfaces::srv::UAVCommand>::SharedPtr client =
    node->create_client<gestelt_interfaces::srv::UAVCommand>("/uav_command");

  auto request = std::make_shared<gestelt_interfaces::srv::UAVCommand::Request>();
  request->header.stamp = node->get_clock()->now();
  request->command = gestelt_interfaces::srv::UAVCommand::COMMAND_TAKEOFF;
  request->value = 3.14;
  request->mode = gestelt_interfaces::srv::UAVCommand::MODE_TRAJECTORY;

  while (!client->wait_for_service(5s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Got response: State[%d] State_name[%s] Success [%d]", 
		response->state, response->state_name, response->success);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
  }

  rclcpp::shutdown();
  return 0;
}