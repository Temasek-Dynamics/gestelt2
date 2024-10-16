/****************************************************************************
 * MIT License
 *  
 *	Copyright (c) 2024 John Tan. All rights reserved.
 *
 *	Permission is hereby granted, free of charge, to any person obtaining a copy
 *	of this software and associated documentation files (the "Software"), to deal
 *	in the Software without restriction, including without limitation the rights
 *	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *	copies of the Software, and to permit persons to whom the Software is
 *	furnished to do so, subject to the following conditions:
 *
 *	The above copyright notice and this permission notice shall be included in all
 *	copies or substantial portions of the Software.
 *
 *	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *	SOFTWARE.
 *
 ****************************************************************************/

#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

#include <gestelt_interfaces/srv/uav_command.hpp>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("uav_command_client");
  rclcpp::Client<gestelt_interfaces::srv::UAVCommand>::SharedPtr client =
    node->create_client<gestelt_interfaces::srv::UAVCommand>("/uav_command");

  auto request = std::make_shared<gestelt_interfaces::srv::UAVCommand::Request>();
  request->header.stamp = node->get_clock()->now();

  if (argc <= 1){
    std::cout << "Please input a valid argument" << std::endl;
    return 1;
  }
  request->command = atoll(argv[1]);

  switch(request->command)
  {
    case gestelt_interfaces::srv::UAVCommand::Request::COMMAND_TAKEOFF:
      request->value = atof(argv[2]); // Used for takeoff
      break;
    case gestelt_interfaces::srv::UAVCommand::Request::COMMAND_LAND:
      break;
    case gestelt_interfaces::srv::UAVCommand::Request::COMMAND_START_MISSION:
      request->mode = atoll(argv[2]); // Used for mission 
      break;
    case gestelt_interfaces::srv::UAVCommand::Request::COMMAND_STOP_MISSION:
      break;
    case gestelt_interfaces::srv::UAVCommand::Request::COMMAND_EMERGENCY_STOP:
      break;
    default:
      break;
  }
  
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
    auto response = result.get();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Got response: State[%d] State_name[%s] Success[%d]", 
		response->state, response->state_name.c_str(), response->success);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
  }

  rclcpp::shutdown();
  return 0;
}