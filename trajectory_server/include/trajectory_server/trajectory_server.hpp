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

#include <tf2_ros/transform_broadcaster.h>

#include <nav_msgs/msg/odometry.hpp>

#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/actuator_motors.hpp"
#include "px4_msgs/msg/vehicle_torque_setpoint.hpp"
#include "px4_msgs/msg/vehicle_thrust_setpoint.hpp"
#include "px4_msgs/msg/vehicle_attitude_setpoint.hpp"
#include "px4_msgs/msg/vehicle_rates_setpoint.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"

#include <gestelt_interfaces/srv/uav_command.hpp>
#include <gestelt_interfaces/msg/all_uav_command.hpp>
#include <gestelt_interfaces/msg/uav_state.hpp>

#include <logger_wrapper/logger_wrapper.hpp>

#include <uavsm.hpp>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace std::placeholders;

enum PX4_CUSTOM_MAIN_MODE {
	PX4_CUSTOM_MAIN_MODE_MANUAL = 1,
	PX4_CUSTOM_MAIN_MODE_ALTCTL, // 2
	PX4_CUSTOM_MAIN_MODE_POSCTL,  // 3
	PX4_CUSTOM_MAIN_MODE_AUTO, // 4
	PX4_CUSTOM_MAIN_MODE_ACRO, // 5
	PX4_CUSTOM_MAIN_MODE_OFFBOARD,  // 6
	PX4_CUSTOM_MAIN_MODE_STABILIZED,
	PX4_CUSTOM_MAIN_MODE_RATTITUDE_LEGACY,
	PX4_CUSTOM_MAIN_MODE_SIMPLE, /* unused, but reserved for future use */
	PX4_CUSTOM_MAIN_MODE_TERMINATION
};

enum PX4_CUSTOM_SUB_MODE_AUTO {
	PX4_CUSTOM_SUB_MODE_AUTO_READY = 1,
	PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF,
	PX4_CUSTOM_SUB_MODE_AUTO_LOITER,
	PX4_CUSTOM_SUB_MODE_AUTO_MISSION,
	PX4_CUSTOM_SUB_MODE_AUTO_RTL,
	PX4_CUSTOM_SUB_MODE_AUTO_LAND,
	PX4_CUSTOM_SUB_MODE_AUTO_RESERVED_DO_NOT_USE, // was PX4_CUSTOM_SUB_MODE_AUTO_RTGS, deleted 2020-03-05
	PX4_CUSTOM_SUB_MODE_AUTO_FOLLOW_TARGET,
	PX4_CUSTOM_SUB_MODE_AUTO_PRECLAND,
	PX4_CUSTOM_SUB_MODE_AUTO_VTOL_TAKEOFF,
	PX4_CUSTOM_SUB_MODE_EXTERNAL1,
	PX4_CUSTOM_SUB_MODE_EXTERNAL2,
	PX4_CUSTOM_SUB_MODE_EXTERNAL3,
	PX4_CUSTOM_SUB_MODE_EXTERNAL4,
	PX4_CUSTOM_SUB_MODE_EXTERNAL5,
	PX4_CUSTOM_SUB_MODE_EXTERNAL6,
	PX4_CUSTOM_SUB_MODE_EXTERNAL7,
	PX4_CUSTOM_SUB_MODE_EXTERNAL8
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

private:
	void getParameters();

	/* Timer for publishign controls to FCU */
	void pubCtrlTimerCB();
	/* Timer sending offboard mode to FCU, required for maintaining offboard mode */
	void setOffboardTimerCB();
	/* Timer for publishing state information */
	void pubStateTimerCB();
	/* Timer for state machine ticking, to query current state and execute actions relevant to the state  */
	void SMTickTimerCB();

	/* Publisher methods */
	/**
	 * @brief Publish vehicle commands
	 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
	 * @param param1    Command parameter 1
	 * @param param2    Command parameter 2
	 */
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0, float param3=0.0);
	/**
	 * @brief Publish the offboard control mode.
	 *        For this example, only position and altitude controls are active.
	 * @param offb_ctrl_mode    Offboard control mode
	 */
	void publishOffboardCtrlMode(const int& offb_ctrl_mode);
	/**
	 * @brief Publish a trajectory setpoint
	 * 
	 * @param pos Position (ENU frame) [m]
	 * @param yaw Yaw [degrees]
	 * @param vel Velociyy (ENU frame) [m/s] 
	 * @param acc Acceleration (ENU frame) [m/s^2]
	 */
	void publishTrajectorySetpoint(
		const Eigen::Vector3d& pos, 
		const Eigen::Vector2d& yaw_yawrate = Eigen::Vector2d(NAN, NAN),
		const Eigen::Vector3d& vel = Eigen::Vector3d(NAN, NAN, NAN), 
		const Eigen::Vector3d& acc = Eigen::Vector3d(NAN, NAN, NAN));

	void publishAttitudeSetpoint(const double& thrust, const Eigen::Vector4d& q_d);
	void publishRatesSetpoint(const double& thrust, const Eigen::Vector3d& rates);
	void publishTorqueThrustSetpoint(const double& thrust, const Eigen::Vector3d& torques);
	void publishActuatorCmds(const Eigen::Vector4d& motor_in );

	/* Subscriber callbacks */
	void globalUAVCmdSubCB(const gestelt_interfaces::msg::AllUAVCommand::UniquePtr msg);
	void UAVCmdSubCB(const gestelt_interfaces::msg::AllUAVCommand::UniquePtr msg);
	void odometrySubCB(const px4_msgs::msg::VehicleOdometry::UniquePtr msg);
	void vehicleStatusSubCB(const px4_msgs::msg::VehicleStatus::UniquePtr msg);
	void intmdCmdSubCB(const px4_msgs::msg::TrajectorySetpoint::UniquePtr msg);

	/* Service callbacks */
	// void uavCmdSrvCB(const std::shared_ptr<gestelt_interfaces::srv::UAVCommand::Request> request,
	// 	std::shared_ptr<gestelt_interfaces::srv::UAVCommand::Response>  response);

	/* Helper methods */

	inline Eigen::Vector3d quaternionToRPY(const Eigen::Quaterniond& q){
		return q.toRotationMatrix().eulerAngles(0, 1, 2); // In roll, pitch and yaw
	}

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
	rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
	rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
	// Controller inputs
	rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
	rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr attitude_setpoint_pub_;
	rclcpp::Publisher<px4_msgs::msg::VehicleRatesSetpoint>::SharedPtr rates_setpoint_pub_;
	rclcpp::Publisher<px4_msgs::msg::VehicleTorqueSetpoint>::SharedPtr torque_setpoint_pub_;
	rclcpp::Publisher<px4_msgs::msg::VehicleThrustSetpoint>::SharedPtr thrust_setpoint_pub_;
	rclcpp::Publisher<px4_msgs::msg::ActuatorMotors>::SharedPtr actuator_cmd_pub_;
	// State publishers
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
	rclcpp::Publisher<gestelt_interfaces::msg::UAVState>::SharedPtr uav_state_pub_;

	/* Subscribers */
	rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr fcu_odom_sub_;
	rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
	rclcpp::Subscription<gestelt_interfaces::msg::AllUAVCommand>::SharedPtr global_uav_cmd_sub_;
	rclcpp::Subscription<gestelt_interfaces::msg::AllUAVCommand>::SharedPtr uav_cmd_sub_;
	rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr lin_mpc_cmd_sub_;

	/* Service Server */
	// rclcpp::Service<gestelt_interfaces::srv::UAVCommand>::SharedPtr uav_cmd_srv_;

	/* Params */
	int drone_id_{0};

	bool pub_map_to_baselink_tf_{false}; // If enabled, publish map to base link transformation

	std::string map_frame_; // Origin frame of uav i.e. "world" or "map"
	std::string base_link_frame_; // Origin frame of uav i.e. "world" or "map"

	double take_off_landing_tol_{0.15}; // [m] Take off and landing tolerance for execution to be completed

	double pub_ctrl_freq_; // [Hz] Frequency of publishing controls
	double pub_state_freq_; // [Hz] Frequency to publish odometry
	double sm_tick_freq_; // [Hz] State machine tick frequency
	double set_offb_ctrl_freq_; // [Hz] Frequency of state machine ticks

	/* Stored Data */
	Eigen::Vector3d cur_pos_enu_{0.0, 0.0, 0.0};		// Current position
	Eigen::Vector3d cur_pos_enu_corr_{0.0, 0.0, 0.0};	// Current position with corrected ground height
	Eigen::Vector3d cur_vel_enu_{0.0, 0.0, 0.0};		// Current velocity
	Eigen::Quaterniond cur_ori_enu_{1.0, 0.0, 0.0, 0.0};	// Current orientation (w,x,y,z)
	Eigen::Vector3d cur_ang_vel_enu_{0.0, 0.0, 0.0};	// Current angular velocity

	Eigen::Vector3d cmd_pos_enu_{0.0, 0.0, 0.0};		// Last commanded position [ENU frame]
	Eigen::Vector3d cmd_pos_enu_corr_{0.0, 0.0, 0.0}; // Last commanded position with corrected height
	Eigen::Vector3d cmd_vel_enu_{0.0, 0.0, 0.0};
	Eigen::Vector3d cmd_acc_enu_{0.0, 0.0, 0.0};
	Eigen::Vector2d cmd_yaw_yawrate_{0.0, 0.0};

	/* Safety */
	std::unique_ptr<Geofence> geofence_; // Geofence to enforce positional limits

	std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_; 

	/* Logger */
	std::shared_ptr<logger_wrapper::LoggerWrapper> logger_;

	/* FCU interface */
	int arming_state_; // Arming state
	int nav_state_; // Navigation state 
	bool pre_flight_checks_pass_; // Pre flight checks pass
	bool connected_to_fcu_; // Indicates connection to FCU

	Eigen::Vector3d ground_height_{0.0, 0.0, 0.0};		// Current position
	
	double last_cmd_pub_t_{0.0}; // Time since last flight controller command is published

}; // class TrajectoryServer

#endif //TRAJECTORY_SERVER_HPP_