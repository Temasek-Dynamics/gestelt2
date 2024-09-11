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

#ifndef TRAJECTORY_SERVER_HPP_
#define TRAJECTORY_SERVER_HPP_

#include <chrono>
#include <iostream>
#include <stdint.h>

#include <Eigen/Eigen>

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/actuator_motors.hpp>
#include <px4_msgs/msg/vehicle_torque_setpoint.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
// #include <px4_msgs/msg/vehicle_control_mode.hpp>

#include <px4_msgs/msg/vehicle_odometry.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <gestelt_interfaces/srv/uav_command.hpp>

#include <frame_transforms.hpp>

#include <uavsm.hpp>


using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;


class TrajectoryServer : public rclcpp::Node
{
public:
	TrajectoryServer() : Node("trajectory_server")
	{
		// start UAV state machine
		fsm_list::start();

		initParams();

		initPubSubTimers();

		initSrv();

		offboard_setpoint_counter_ = 0;
	}

private:
	void initParams();
	void initPubSubTimers();
	void initSrv();

	/* Time callbacks */
	void setOffboardTimerCB();
	void pubOdomTimerCB();
	void SMTickTimerCB();

	/* Publisher methods */
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
	void publishOffboardCtrlMode(const int& offb_ctrl_mode);
	void publishTrajectorySetpoint(const Eigen::Vector3d& pos_enu, const double& yaw);
	void publishActuatorCmds(const Eigen::Vector4d& motor_in );
	void publishTorqueSetpoint(const Eigen::Vector3d& torques);
	void publishThrustSetpoint(const Eigen::Vector3d& thrust_vec);

	/* Service callbacks */
	void uavCmdSrvCB(const std::shared_ptr<gestelt_interfaces::srv::UAVCommand::Request> request,
						std::shared_ptr<gestelt_interfaces::srv::UAVCommand::Response>  response);

	/* Subscriber callbacks */
	void odometrySubCB(const VehicleOdometry::UniquePtr msg);

	/* PX4-related methods */
	void arm();
	void disarm();

	/* Helper methods */


private:
	rclcpp::TimerBase::SharedPtr set_offb_timer_;	
	rclcpp::TimerBase::SharedPtr pub_cmd_timer_;
	rclcpp::TimerBase::SharedPtr pub_odom_timer_;
	rclcpp::TimerBase::SharedPtr sm_tick_timer_;

	/* Publishers */
	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_pub_;

	// Controller inputs
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
	rclcpp::Publisher<ActuatorMotors>::SharedPtr actuator_cmd_pub_;
	rclcpp::Publisher<VehicleTorqueSetpoint>::SharedPtr torque_setpoint_pub_;
	rclcpp::Publisher<VehicleThrustSetpoint>::SharedPtr thrust_setpoint_pub_;

	// State publishers
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

	/* Subscribers */
	rclcpp::Subscription<VehicleOdometry>::SharedPtr odometry_sub_;

	// std::atomic<uint64_t> timestamp_;   //!< common synced timestamped
	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

	/* Services */
	rclcpp::Service<gestelt_interfaces::srv::UAVCommand>::SharedPtr uav_cmd_srv_;

	/* Stored Data */
	Eigen::Vector3d cur_pos_;	// current position
	Eigen::Quaterniond cur_ori_;	// current orientation
	Eigen::Vector3d cur_vel_;	// current velocity
	Eigen::Vector3d cur_ang_vel_;	// current angular velocity

	/* Param */
	std::string origin_frame_; // Origin frame of uav i.e. "world" or "map"
	std::string base_link_frame_; // base link frame of uav

	int offboard_ctrl_mode_{-1}; // offboard control mode
	double takeoff_height_; // [m] Take off height
	
	double pub_cmd_freq_; // [Hz] Frequency to publish PVA commands
	double pub_odom_freq_; // [Hz] Frequency of state machine ticks
	double sm_tick_freq_; // [Hz] State machine tick frequency
	double set_offb_ctrl_freq_; // [Hz] Frequency of state machine ticks
};

#endif //TRAJECTORY_SERVER_HPP_