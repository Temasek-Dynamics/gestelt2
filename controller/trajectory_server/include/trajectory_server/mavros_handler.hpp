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

#ifndef MAVROS_HANDLER_HPP_
#define MAVROS_HANDLER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>

#include <rclcpp/wait_for_message.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <Eigen/Eigen>

using namespace std::chrono_literals;

class MavrosHandler 
{
public:
  MavrosHandler(rclcpp::Node::SharedPtr node, std::string map_frame)
  : node_(node), map_frame_(map_frame)
  {
    /* Service clients */
    mavros_arm_client_ = 
      node_->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
    mavros_set_mode_client_ = 
      node_->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");

    /* Create publishers */
    mavros_cmd_pub_ = node_->create_publisher<mavros_msgs::msg::PositionTarget>(
      "mavros/setpoint_raw/local", rclcpp::SensorDataQoS());
  }

	void setState(const mavros_msgs::msg::State::UniquePtr& state){
		state_ = *state;

    connected_to_fcu_ = state_.connected;
	}

	void setOdom(const nav_msgs::msg::Odometry::UniquePtr& odom){
		odom_ = *odom;

    cur_pos_corr_ = Eigen::Vector3d(
      odom_.pose.pose.position.x, 
      odom_.pose.pose.position.y, 
      odom_.pose.pose.position.z - ground_height_) ;
	}

  void setGroundheight(const double& h){
    ground_height_ = h;
  }

  /* Changing of states */

	// Toggle enable/disable offboard
	bool toggleOffboardMode(bool toggle);

  /* Execution of controls */

	// Execute landing
	bool execLand();

	// Execute taking off of drone
	void execTakeOff(const Eigen::Vector3d& p, const Eigen::Vector2d& yaw_yawrate);

	// Execute hovering
	void execHover(const Eigen::Vector3d& p, const Eigen::Vector2d& yaw_yawrate);

	// Execute mission
	void execMission(
    const Eigen::Vector3d& p, 
    const Eigen::Vector3d& v, 
    const Eigen::Vector3d& a, 
    const Eigen::Vector2d& yaw_yawrate);

  void publishCmd(const Eigen::Vector3d& p, const Eigen::Vector3d& v, const Eigen::Vector3d& a, 
    const Eigen::Vector2d& yaw_yawrate, const uint16_t& type_mask);

	/* Checks */

	// Check if UAV state is in OFFBOARD mode and ARMED
	bool isUAVReady();

	// Check if UAV state is in "AUTO.LOITER" mode and DISARMED
	bool isUAVIdle();

	// Check if landing execution is complete
	bool isLanded();

	// Check if take off execution is complete
	bool isTakenOff(const double& takeoff_h);

  // returns true if FCU is connected via mavlink
  bool isConnected() const{
    return connected_to_fcu_;
  }

private:
  rclcpp::Node::SharedPtr node_;

  /* Params */
  double take_off_landing_tol_{0.2};
  std::string map_frame_;

  std::chrono::duration<double, std::milli> dur_7s = std::chrono::milliseconds(10000);

  /* Data */
  bool connected_to_fcu_{false};
  double ground_height_{0.0}; 
  Eigen::Vector3d cur_pos_corr_;// Correct for ground height

	mavros_msgs::msg::State state_; // (Mavros) UAV state
  nav_msgs::msg::Odometry odom_; // Vehicle odometry

	/* Publishers */
	rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr mavros_cmd_pub_;

	/* Service Clients */
	rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr mavros_arm_client_;
	rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mavros_set_mode_client_; 

	// Values set from mavros_msgs/PositionTarget message constants
	uint16_t IGNORE_VEL{mavros_msgs::msg::PositionTarget::IGNORE_VX 
                      | mavros_msgs::msg::PositionTarget::IGNORE_VY 
                      | mavros_msgs::msg::PositionTarget::IGNORE_VZ}; // Ignore velocity in typemask
	uint16_t IGNORE_ACC{mavros_msgs::msg::PositionTarget::IGNORE_AFX 
                      | mavros_msgs::msg::PositionTarget::IGNORE_AFY 
                      | mavros_msgs::msg::PositionTarget::IGNORE_AFZ}; // Ignore acceleration in typemask
	uint16_t IGNORE_YAW{mavros_msgs::msg::PositionTarget::IGNORE_YAW}; // Ignore yaw in typemask
	uint16_t IGNORE_YAW_RATE{mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE}; // Ignore yaw rate in typemask
};

/* Checks */

inline bool MavrosHandler::isUAVReady() {
	return (state_.mode == "OFFBOARD") 
		&& state_.armed;
}

inline bool MavrosHandler::isUAVIdle() {
	return (state_.mode == "AUTO.LOITER") 
		&& !state_.armed;
}

inline bool MavrosHandler::isLanded()
{
	// Check that difference between desired landing height and current UAV position
	// is within tolerance 
	return abs(cur_pos_corr_(2) - 0.0) < take_off_landing_tol_;
}

inline bool MavrosHandler::isTakenOff(const double& takeoff_h)
{
	return abs(cur_pos_corr_(2) - takeoff_h) < take_off_landing_tol_;
}

/* Changing of states */

inline bool MavrosHandler::toggleOffboardMode(bool toggle)
{
  auto arm_cmd_req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
  arm_cmd_req->value = toggle ?  true : false;

  auto set_mode_req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
  set_mode_req->custom_mode = toggle ? "OFFBOARD" : "AUTO.LOITER"; 

  rclcpp::Rate srv_loop_rate(2.0);
  rclcpp::Rate cmd_loop_rate(15.0);

  // send a few setpoints before starting
  for (int i = 0; i < 10 && rclcpp::ok(); i++)
  {
    execTakeOff(Eigen::Vector3d::Zero(), Eigen::Vector2d::Zero());
    cmd_loop_rate.sleep();
  }
  // auto last_request_t = node_->get_clock()->now();

  auto conditions_fulfilled = [&] () {
    return (toggle ? isUAVReady() : isUAVIdle());
  };

  while (!conditions_fulfilled()){
    // Set mode
    if (state_.mode != set_mode_req->custom_mode)
    {
      auto result = mavros_set_mode_client_->async_send_request(set_mode_req);

      // Refer to https://github.com/ros2/rclcpp/issues/206 to find out why 
      // we don't use spin_until_future_complete()
      std::future_status status;

      switch (status = result.wait_for(7s); status)
      {
        case std::future_status::deferred:
          RCLCPP_ERROR(node_->get_logger(), "Service call to PX4 set_mode_client deferred");
          break;
        case std::future_status::timeout:
          RCLCPP_ERROR(node_->get_logger(), "Service call to PX4 set_mode_client timeout");
          break;
        case std::future_status::ready:
          auto response = result.get();

          if (response->mode_sent){
            RCLCPP_INFO(node_->get_logger(), "Setting %s mode successful", 
              set_mode_req->custom_mode.c_str());
          }
          else {
            RCLCPP_ERROR(node_->get_logger(), "Setting %s mode failed", 
              set_mode_req->custom_mode.c_str());
          }
          break;
      }
    }

    // Arm/disarm
    if (state_.armed != arm_cmd_req->value) 
    {
      auto result = mavros_arm_client_->async_send_request(arm_cmd_req);

      std::future_status status;

      switch (status = result.wait_for(7s); status)
      {
        case std::future_status::deferred:
          RCLCPP_ERROR(node_->get_logger(), "Service call to PX4 arming_client deferred");
          break;
        case std::future_status::timeout:
          RCLCPP_ERROR(node_->get_logger(), "Service call to PX4 arming_client timeout");
          break;
        case std::future_status::ready:
          auto response = result.get();

          if (response->success){
            RCLCPP_INFO(node_->get_logger(), "Setting arm to %d successful", 
              arm_cmd_req->value );
          }
          else {
            RCLCPP_ERROR(node_->get_logger(), "Setting arm to %d failed", 
              arm_cmd_req->value );
          }
          break;
      }
    }

    srv_loop_rate.sleep();   
  }

  return true;
}

/* Execution of controls */

inline void MavrosHandler::execTakeOff(const Eigen::Vector3d& p, const Eigen::Vector2d& yaw_yawrate)
{ 
  uint16_t type_mask = IGNORE_VEL | IGNORE_ACC | IGNORE_YAW_RATE ; // Ignore Velocity, Acceleration and yaw rate

  publishCmd( p, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
              yaw_yawrate, type_mask);
}

inline void MavrosHandler::execHover(const Eigen::Vector3d& p, const Eigen::Vector2d& yaw_yawrate)
{
  uint16_t type_mask = IGNORE_VEL | IGNORE_ACC | IGNORE_YAW_RATE ; // Ignore Velocity, Acceleration and yaw rate

  publishCmd( p, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
              yaw_yawrate, type_mask);
}

inline void MavrosHandler::execMission(
  const Eigen::Vector3d& p, 
  const Eigen::Vector3d& v, 
  const Eigen::Vector3d& a, 
  const Eigen::Vector2d& yaw_yawrate)
{
  uint16_t type_mask = mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE; // Ignore Velocity, Acceleration and yaw rate
	publishCmd(p, v, a, yaw_yawrate, type_mask);
}

inline bool MavrosHandler::execLand()
{
  rclcpp::Rate srv_loop_rate(2.0);

  auto set_mode_req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
  set_mode_req->custom_mode = "AUTO.LAND"; 

  while (!isLanded()){
    auto result = mavros_set_mode_client_->async_send_request(set_mode_req);

    std::future_status status;

    switch (status = result.wait_for(7s); status)
    {
      case std::future_status::deferred:
        RCLCPP_ERROR(node_->get_logger(), "Service call to PX4 set_mode_client deferred");
        break;
      case std::future_status::timeout:
        RCLCPP_ERROR(node_->get_logger(), "Service call to PX4 set_mode_client timeout");
        break;
      case std::future_status::ready:
        auto response = result.get();

        if (response->mode_sent){
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Setting %s mode successful", 
            set_mode_req->custom_mode.c_str());
        }
        else {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Setting %s mode failed", 
            set_mode_req->custom_mode.c_str());
        }
        break;
    }

    srv_loop_rate.sleep();   
  }

  return true;
}

inline void MavrosHandler::publishCmd(
  const Eigen::Vector3d& p, 
  const Eigen::Vector3d& v, 
  const Eigen::Vector3d& a, 
  const Eigen::Vector2d& yaw_yawrate, 
  const uint16_t& type_mask)
{
  mavros_msgs::msg::PositionTarget cmd_msg;

  cmd_msg.header.stamp = node_->get_clock()->now();
  cmd_msg.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
  cmd_msg.type_mask = type_mask;

  cmd_msg.position.x = p(0);
  cmd_msg.position.y = p(1);
  cmd_msg.position.z = p(2);

  cmd_msg.velocity.x = v(0);
  cmd_msg.velocity.y = v(1);
  cmd_msg.velocity.z = v(2);
  
  cmd_msg.acceleration_or_force.x = a(0);
  cmd_msg.acceleration_or_force.y = a(1);
  cmd_msg.acceleration_or_force.z = a(2);
  cmd_msg.yaw = yaw_yawrate(0);
  // cmd_msg.yaw = M_PI;
  cmd_msg.yaw_rate = yaw_yawrate(1);
  // ROS_INFO("Velocity for final command: %f, %f, %f", v(0), v(1), v(2));
  // ROS_INFO("Acceleration for final command: %f, %f, %f", a(0), a(1), a(2));
  mavros_cmd_pub_->publish(cmd_msg);
}

#endif // MAVROS_HANDLER_HPP_