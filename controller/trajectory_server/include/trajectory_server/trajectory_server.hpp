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

#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>

#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/state.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <gestelt_interfaces/srv/uav_command.hpp>
#include <gestelt_interfaces/msg/all_uav_command.hpp>
#include <gestelt_interfaces/msg/uav_state.hpp>
#include <gestelt_interfaces/msg/nav_state.hpp>

#include <logger_wrapper/logger_wrapper.hpp>

#include <frame_transforms.hpp>
#include <uavsm.hpp>
#include <trajectory_server/poly_traj_cmd.hpp>

#include <trajectory_server/mavros_handler.hpp>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace std::placeholders;

/* Geofence is used for enforcing limits on positional commands */
enum FCUInterface {
	MAVROS,
	MICRO_XRCE_DDS
};

/* TrajectoryType is used for setting the expected trajectory command type */
enum TrajectoryType {
	MINCO, /* MINCO trajectory */
	MPC, /* Position, velocity, acceleration commands */
	UNDEFINED_TRAJ_TYPE,
};

/* Geofence is used for enforcing limits on positional commands */
struct Geofence{
	Geofence(const std::shared_ptr<logger_wrapper::LoggerWrapper>& logger)
	: logger_(logger)
	{
	}

	bool withinLimits(const Eigen::Vector3d& pos){

		if (pos(0) < min_x || pos(0) > max_x){
			logger_->logError(strFmt("Commanded x position (%f) exceeded limits (%f,%f)", 
				pos(0), min_x, max_x));

			return false;
		}
		else if (pos(1) < min_y || pos(1) > max_y) {

			logger_->logError(strFmt("Commanded y position (%f) exceeded limits (%f,%f)", 
				pos(1), min_y, max_y));

			return false;
		}
		else if (pos(2) < min_z || pos(2) > max_z) {

			logger_->logError(strFmt("Commanded z position (%f) exceeded limits (%f,%f)", 
				pos(2), min_z, max_z));

			return false;
		}

		return true;
	}
	std::shared_ptr<logger_wrapper::LoggerWrapper> logger_;

	double min_x, max_x;
	double min_y, max_y;
	double min_z, max_z;
};

class TrajectoryServer : public rclcpp::Node
{
public:
	TrajectoryServer();
	virtual ~TrajectoryServer();
	void init();

private:
	void initParams();
	void initPubSubTimers();
	void initSrv();

	/* Time callbacks */

	/* Timer for publishign controls to FCU */
	void pubCtrlTimerCB();
	/* Timer sending offboard mode to FCU, required for maintaining offboard mode */
	void setOffboardTimerCB();
	/* Timer for publishing state information */
	void pubStateTimerCB();
	/* Timer for state machine ticking, to query current state and execute actions relevant to the state  */
	void SMTickTimerCB();

	/* Publisher methods */

	/* Service callbacks */
	void uavCmdSrvCB(const std::shared_ptr<gestelt_interfaces::srv::UAVCommand::Request> request,
						std::shared_ptr<gestelt_interfaces::srv::UAVCommand::Response>  response);

	/* Subscriber callbacks */
	void allUAVCmdSubCB(const gestelt_interfaces::msg::AllUAVCommand::UniquePtr msg);
	void linMPCCmdSubCB(const mavros_msgs::msg::PositionTarget::UniquePtr msg);
	void navStateSubCB(const gestelt_interfaces::msg::NavState::UniquePtr msg);

	void mavrosStateSubCB(const mavros_msgs::msg::State::UniquePtr msg);
	void mavrosOdomSubCB(const nav_msgs::msg::Odometry::UniquePtr msg);

	/* PX4-related methods */

	/* Helper methods */

	Eigen::Vector3d quaternionToRPY(const Eigen::Quaterniond& q){
		return q.toRotationMatrix().eulerAngles(0, 1, 2); // In roll, pitch and yaw
	}

	/* Checking methods */
	

private:
	/* Callback groups */
	rclcpp::CallbackGroup::SharedPtr control_cb_group_;
	rclcpp::CallbackGroup::SharedPtr fcu_cb_group_;
	rclcpp::CallbackGroup::SharedPtr others_cb_group_;

	/* Timers */
	rclcpp::TimerBase::SharedPtr set_offb_timer_;	
	rclcpp::TimerBase::SharedPtr pub_ctrl_timer_;
	rclcpp::TimerBase::SharedPtr pub_state_timer_;
	rclcpp::TimerBase::SharedPtr sm_tick_timer_;

	/* Publishers */
		rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr vel_magnitude_pub_;

		// State publishers
		rclcpp::Publisher<gestelt_interfaces::msg::UAVState>::SharedPtr uav_state_pub_;

	/* Subscribers */
		// Subscribers to navigator command
		rclcpp::Subscription<mavros_msgs::msg::PositionTarget>::SharedPtr lin_mpc_cmd_sub_;

		// Subscribers to state
		rclcpp::Subscription<gestelt_interfaces::msg::NavState>::SharedPtr navigator_state_sub_;
		rclcpp::Subscription<gestelt_interfaces::msg::AllUAVCommand>::SharedPtr all_uav_cmd_sub_;

		// MAVROS
		rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr mavros_status_sub_;
		rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr mavros_odom_sub_;

	/* Service Server */
	rclcpp::Service<gestelt_interfaces::srv::UAVCommand>::SharedPtr uav_cmd_srv_;

	/* Params */
	int drone_id_{0};

	std::string map_frame_; // Origin frame of uav i.e. "world" or "map"
	std::string base_link_frame_; // base link frame of uav

	double take_off_landing_tol_{0.15}; // [m] Take off and landing tolerance for execution to be completed

	double pub_ctrl_freq_; // [Hz] Frequency of publishing controls
	double pub_state_freq_; // [Hz] Frequency to publish odometry
	double sm_tick_freq_; // [Hz] State machine tick frequency
	double set_offb_ctrl_freq_; // [Hz] Frequency of state machine ticks
	double nav_state_timeout_; // [Hz] Frequency of state machine ticks

	TrajectoryType traj_type_;	// Trajectory plugin type

	FCUInterface fcu_interface_; // Type of interface with FCU, currently available are Mavros and Micro XRCE D-S

	/* Safety */
	std::unique_ptr<Geofence> geofence_; // Geofence to enforce positional limits

	/* Trajectory command data */
	// std::unique_ptr<PolyTrajCmd> poly_traj_cmd_; // MINCO trajectory command reader

	/* Stored Data */
	Eigen::Vector3d cur_pos_{0.0, 0.0, 0.0};		// Current position
	Eigen::Vector3d cur_pos_corr_{0.0, 0.0, 0.0};		// Current position with corrected ground height
	
	Eigen::Vector3d cur_vel_{0.0, 0.0, 0.0};		// Current velocity
	Eigen::Quaterniond cur_ori_{1.0, 0.0, 0.0, 0.0};	// Current orientation
	Eigen::Vector3d cur_ang_vel_{0.0, 0.0, 0.0};	// Current angular velocity
	double ground_height_{0.0}; // Starting ground height

	Eigen::Vector3d cmd_pos_enu_{0.0, 0.0, 0.0};		// Last commanded position [ENU frame]
	Eigen::Vector3d cmd_vel_enu_{0.0, 0.0, 0.0};		
	Eigen::Vector3d cmd_acc_enu_{0.0, 0.0, 0.0};				
	Eigen::Vector2d cmd_yaw_yawrate_{0.0, 0.0};
	Eigen::Vector3d cmd_jerk_enu_{0.0, 0.0, 0.0};	    // For logging purposes, not used in command

	/* Logger */
	std::shared_ptr<logger_wrapper::LoggerWrapper> logger_;

	/* FCU interface */
	std::unique_ptr<MavrosHandler> mavros_handler_; // Class for handling mavros stuff
	
	double last_cmd_pub_t_{0.0}; // Time since last flight controller command is published

	double last_nav_heartbeat_{0.0}; // Time since last navigator heartbeat 
}; // class TrajectoryServer

#endif //TRAJECTORY_SERVER_HPP_