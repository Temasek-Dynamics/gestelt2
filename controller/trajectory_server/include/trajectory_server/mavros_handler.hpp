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
#include <mavros_msgs/msg/command_bool.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/set_mode.hpp>

class MavrosHandler 
{
public:
    MavrosHandler(rclcpp::Node::SharedPtr node)
    : node_(node)
    {
		/* Service clients */
		mavros_arm_client_ = 
			node_->create_client<mavros_msgs::msg::CommandBool>("mavros/cmd/arming");
		mavros_set_mode_client_ = 
			node_->create_client<mavros_msgs::msg::SetMode>("mavros/set_mode");

		/* Create publishers */
		mavros_cmd_pub_ = node_->create_publisher<mavros_msgs::msg::PositionTarget>(
			"mavros/setpoint_raw/local", 10);

    }

	void setState(const mavros_msgs::msg::State& state)
	{
		state_ = state;
	}

	// Toggle enable/disable offboard
	bool toggleOffboardMode(bool toggle);

	// Set last commanded value, used for:
	//		1. Hovering
	//		2. Takeoff
	//		3. landing
	void setLastCmd();
	
	// Execute taking off of drone
	void execTakeOff();

	// Execute hovering
	void execHover();

	// Execute landing
	void execLand();

	/* Checks */

	// Check if UAV state is in OFFBOARD mode and ARMED
	bool isUAVReady();

	// Check if UAV state is in "AUTO.LOITER" mode and DISARMED
	bool isUAVIdle();

	// Check if landing execution is complete
	bool isLanded();

	// Check if take off execution is complete
	bool isTakenOff();

private:
    rclcpp::Node::SharedPtr node_;

	/* (Mavros) UAV state*/
	mavros_msgs::msg::State state_;

	/* Publishers */
	rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr mavros_cmd_pub_;

	/* Service Clients */
	rclcpp::Client<mavros_msgs::msg::CommandBool>::SharedPtr mavros_arm_client_;
	rclcpp::Client<mavros_msgs::msg::SetMode>::SharedPtr mavros_set_mode_client_; 

	// Values set from mavros_msgs/PositionTarget message constants
	uint16_t IGNORE_VEL{mavros_msgs::msg::PositionTarget::IGNORE_VX | mavros_msgs::msg::PositionTarget::IGNORE_VY | mavros_msgs::msg::PositionTarget::IGNORE_VZ}; // Ignore velocity in typemask
	uint16_t IGNORE_ACC{mavros_msgs::msg::PositionTarget::IGNORE_AFX | mavros_msgs::msg::PositionTarget::IGNORE_AFY | mavros_msgs::msg::PositionTarget::IGNORE_AFZ}; // Ignore acceleration in typemask
	uint16_t IGNORE_YAW{mavros_msgs::msg::PositionTarget::IGNORE_YAW}; // Ignore yaw in typemask
	uint16_t IGNORE_YAW_RATE{mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE}; // Ignore yaw rate in typemask
}

inline bool MavrosHandler::isUAVReady() {
	return (mavros_state_.mode == "OFFBOARD") 
		&& mavros_state_.armed;
}

inline bool MavrosHandler::isUAVIdle() {
	return (mavros_state_.mode == "AUTO.LOITER") 
		&& !mavros_state_.armed;
}

inline bool MavrosHandler::isLanded()
{
	// Check that difference between desired landing height and current UAV position
	// is within tolerance 
	return abs(uav_pose_.pose.position.z - landed_height_) < take_off_landing_tol_;
}

inline bool MavrosHandler::isTakenOff()
{
	return abs(uav_pose_.pose.position.z - takeoff_height_) < take_off_landing_tol_;
}

inline bool MavrosHandler::toggleOffboardMode(bool toggle)
{
    bool arm_val = toggle ?  true : false;
    std::string set_mode_val = toggle ? "OFFBOARD" : "AUTO.LOITER"; 

    auto conditions_fulfilled = [&] () {
      return (toggle ? isUAVReady() : isUAVIdle());
    };

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = arm_val;

    mavros_msgs::SetMode set_mode_srv;
    set_mode_srv.request.custom_mode = set_mode_val;

    // Make sure takeoff is not immediately sent, 
    // this will help to stream the correct data to the program first.
    // Will give a 1sec buffer
    // ros::Duration(1.0).sleep();

    ros::Rate rate(pub_cmd_freq_);

    // send a few setpoints before starting
    for (int i = 0; ros::ok() && i < 10; i++)
    {
      execTakeOff();
      ros::spinOnce();
      rate.sleep();
    }
    ros::Time last_request_t = ros::Time::now();

    while (!conditions_fulfilled()){

      bool request_timeout = ((ros::Time::now() - last_request_t) > ros::Duration(2.0));

      if (mavros_state_.mode != set_mode_val && request_timeout)
      {
        if (set_mode_client.call(set_mode_srv))
        {
          if (set_mode_srv.response.mode_sent){
            logInfo(str_fmt("Setting %s mode successful", set_mode_val.c_str()));
          }
          else {
            logInfo(str_fmt("Setting %s mode failed", set_mode_val.c_str()));
          }
        }
        else {
          logInfo("Service call to PX4 set_mode_client failed");
        }

        last_request_t = ros::Time::now();
      }
      else if (mavros_state_.armed != arm_val && request_timeout) 
      {
        if (arming_client.call(arm_cmd)){
          if (arm_cmd.response.success){
            logInfo(str_fmt("Setting arm to %d successful", arm_val));
          }
          else {
            logInfo(str_fmt("Setting arm to %d failed", arm_val));
          }
        }
        else {
          logInfo("Service call to PX4 arming_client failed");
        }

        last_request_t = ros::Time::now();
      }
      ros::spinOnce();
      rate.sleep();        
    }

    return true;
}

inline void MavrosHandler::execTakeOff(const Eigen::Vector3d& p, const Eigen::Vector2d& yaw_yawrate)
{ 
  int type_mask = IGNORE_VEL | IGNORE_ACC | IGNORE_YAW_RATE ; // Ignore Velocity, Acceleration and yaw rate

  publishCmd( p, Vector3d::Zero(), Vector3d::Zero(),
              yaw_yawrate 
              type_mask);
}

inline void MavrosHandler::execHover(const Eigen::Vector3d& p, const Eigen::Vector2d& yaw_yawrate)
{
  int type_mask = IGNORE_VEL | IGNORE_ACC | IGNORE_YAW_RATE ; // Ignore Velocity, Acceleration and yaw rate

  publishCmd( p, Vector3d::Zero(), Vector3d::Zero(),
              yaw_yawrate 
              type_mask);
}

inline void MavrosHandler::execMission(const Eigen::Vector3d& p, const Eigen::Vector3d& v, const Eigen::Vector3d& a, 
  const Eigen::Vector2d& yaw_yawrate)
{
	publishCmd(p, v, a, yaw_yawrate, );
}

inline void MavrosHandler::publishCmd(const Eigen::Vector3d& p, const Eigen::Vector3d& v, const Eigen::Vector3d& a, 
  const Eigen::Vector2d& yaw_yawrate, uint16_t type_mask)
{
  mavros_msgs::msg::PositionTarget cmd_msg;

  cmd_msg.header.stamp = ros::Time::now();
  cmd_msg.header.frame_id = origin_frame_;
  cmd_msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
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
  cmd_msg.yaw_rate = yaw_yawrate(1);
  // ROS_INFO("Velocity for final command: %f, %f, %f", v(0), v(1), v(2));
  // ROS_INFO("Acceleration for final command: %f, %f, %f", a(0), a(1), a(2));
  mavros_cmd_pub_->publish(pos_cmd);
}

#endif // MAVROS_HANDLER_HPP_