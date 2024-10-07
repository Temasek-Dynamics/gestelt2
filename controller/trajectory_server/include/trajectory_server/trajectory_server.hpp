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
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
// #include <px4_msgs/msg/vehicle_control_mode.hpp>

#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <gestelt_interfaces/srv/uav_command.hpp>

#include <logger_wrapper/logger_wrapper.hpp>

#include <frame_transforms.hpp>
#include <uavsm.hpp>
#include <trajectory_server/poly_traj_cmd.hpp>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
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

enum TrajectoryType {
	MINCO,
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
	void setOffboardTimerCB();
	void pubOdomTimerCB();
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
	 * @param pos Position (ENU frame)
	 * @param yaw Yaw 
	 * @param vel Velociyy (ENU frame)
	 * @param acc Acceleration (ENU frame)
	 */
	void publishTrajectorySetpoint(
		const Eigen::Vector3d& pos, 
		const Eigen::Vector2d& yaw_yawrate,
		const Eigen::Vector3d& vel = Eigen::Vector3d(0.0, 0.0, 0.0), 
		const Eigen::Vector3d& acc = Eigen::Vector3d(0.0, 0.0, 0.0));
	void publishAttitudeSetpoint(const double& thrust, const Eigen::Vector4d& q_d);
	void publishRatesSetpoint(const double& thrust, const Eigen::Vector3d& rates);
	void publishTorqueThrustSetpoint(const double& thrust, const Eigen::Vector3d& torques);
	void publishActuatorCmds(const Eigen::Vector4d& motor_in );

	/* Service callbacks */
	void uavCmdSrvCB(const std::shared_ptr<gestelt_interfaces::srv::UAVCommand::Request> request,
						std::shared_ptr<gestelt_interfaces::srv::UAVCommand::Response>  response);

	/* Subscriber callbacks */
	void odometrySubCB(const VehicleOdometry::UniquePtr msg);
	void vehicleStatusSubCB(const VehicleStatus::UniquePtr msg);

	/* PX4-related methods */

	/* Helper methods */

	/* Load Back-end Trajectory plugin*/
	TrajectoryType getTrajAdaptorType(const std::string& name)
	{
		// Check all states
		if (name == "MINCO"){
			return TrajectoryType::MINCO;
		}
		else {
			return TrajectoryType::MINCO;
		}
	}

	/* Checking methods */
	

private:
	/* Callback groups */
	rclcpp::CallbackGroup::SharedPtr control_cb_group_;
	rclcpp::CallbackGroup::SharedPtr fcu_cb_group_;
	rclcpp::CallbackGroup::SharedPtr others_cb_group_;

	/* Timers */
	rclcpp::TimerBase::SharedPtr set_offb_timer_;	
	rclcpp::TimerBase::SharedPtr pub_cmd_timer_;
	rclcpp::TimerBase::SharedPtr pub_odom_timer_;
	rclcpp::TimerBase::SharedPtr sm_tick_timer_;

	/* Publishers */
	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_pub_;

	// Controller inputs
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
	rclcpp::Publisher<VehicleAttitudeSetpoint>::SharedPtr attitude_setpoint_pub_;
	rclcpp::Publisher<VehicleRatesSetpoint>::SharedPtr rates_setpoint_pub_;
	rclcpp::Publisher<VehicleTorqueSetpoint>::SharedPtr torque_setpoint_pub_;
	rclcpp::Publisher<VehicleThrustSetpoint>::SharedPtr thrust_setpoint_pub_;
	rclcpp::Publisher<ActuatorMotors>::SharedPtr actuator_cmd_pub_;

	// State publishers
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

	/* Subscribers */
	rclcpp::Subscription<VehicleOdometry>::SharedPtr odometry_sub_;
	rclcpp::Subscription<VehicleStatus>::SharedPtr vehicle_status_sub_;

	/* Services */
	rclcpp::Service<gestelt_interfaces::srv::UAVCommand>::SharedPtr uav_cmd_srv_;

	/* Params */
	int drone_id_{1};

	std::string origin_frame_; // Origin frame of uav i.e. "world" or "map"
	std::string base_link_frame_; // base link frame of uav

	double default_takeoff_height_; // [m] Take off height
	double take_off_landing_tol_{0.15}; // [m] Take off and landing tolerance

	double pub_cmd_freq_; // [Hz] Frequency to publish PVA commands
	double pub_odom_freq_; // [Hz] Frequency of state machine ticks
	double sm_tick_freq_; // [Hz] State machine tick frequency
	double set_offb_ctrl_freq_; // [Hz] Frequency of state machine ticks

	TrajectoryType traj_type_;	// Trajectory plugin type
	PolyTrajCmd poly_traj_cmd_; // MINCO trajectory command reader

	/* Stored Data */
	Eigen::Vector3d cur_pos_;	// current position
	Eigen::Quaterniond cur_ori_;	// current orientation
	Eigen::Vector3d cur_vel_;	// current velocity
	Eigen::Vector3d cur_ang_vel_;	// current angular velocity

	/* Logger */
	std::shared_ptr<logger_wrapper::LoggerWrapper> logger_;

	/* UAV state*/
	int arming_state_; // Arming state
	int nav_state_; // Navigation state 
	bool pre_flight_checks_pass_; // Pre flight checks pass
	bool connected_to_fcu_; // Indicates connection to FCU
	
}; // class TrajectoryServer

#endif //TRAJECTORY_SERVER_HPP_