/****************************************************************************
 *
 * Copyright 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Offboard control example
 * @file offboard_control.cpp
 * @addtogroup examples
 * @author Mickey Cowden <info@cowden.tech>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 */
#include <chrono>
#include <iostream>


#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/actuator_motors.hpp>
#include <px4_msgs/msg/vehicle_torque_setpoint.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
// #include <px4_msgs/msg/vehicle_control_mode.hpp>

#include <px4_msgs/msg/vehicle_odometry.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <Eigen/Eigen>

#include <trajectory_server/frame_transforms.h>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class TrajectoryServer : public rclcpp::Node
{
public:
	TrajectoryServer() : Node("trajectory_server")
	{
		initParams();

		/* Publishers */
		vehicle_command_pub_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

		offboard_control_mode_pub_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		
		trajectory_setpoint_pub_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		actuator_cmd_pub_ = this->create_publisher<ActuatorMotors>("/fmu/in/actuator_motors", 10);
		torque_setpoint_pub_ = this->create_publisher<VehicleTorqueSetpoint>("/fmu/in/vehicle_torque_setpoint", 10);
		thrust_setpoint_pub_ = this->create_publisher<VehicleThrustSetpoint>("/fmu/in/vehicle_thrust_setpoint", 10);

		odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

		/* Subscribers */
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos_sensor = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

		odometry_sub_ = this->create_subscription<VehicleOdometry>(
			"/fmu/out/vehicle_odometry", qos_sensor, std::bind(&TrajectoryServer::odometrySubCB, this, std::placeholders::_1));

		/* Timers */

		// set_offb_timer_ = this->create_wall_timer((1/set_offb_ctrl_freq_) *1ms, std::bind(&TrajectoryServer::setOffboardTimerCB, this));
		// pub_cmd_timer_ = this->create_wall_timer((1/pub_cmd_freq_) *1ms, std::bind(&TrajectoryServer::pubCmdTimerCB, this));
		pub_odom_timer_ = this->create_wall_timer((1/pub_odom_freq_) *1ms, std::bind(&TrajectoryServer::pubOdomTimerCB, this));

		offboard_setpoint_counter_ = 0;
	}

	void arm();
	void disarm();

private:
	rclcpp::TimerBase::SharedPtr set_offb_timer_;
	rclcpp::TimerBase::SharedPtr pub_cmd_timer_;
	rclcpp::TimerBase::SharedPtr pub_odom_timer_;

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

	int offboard_ctrl_mode_{0}; // offboard control mode

	/* Stored Data */
	Eigen::Vector3d cur_pos_;	// current position
	Eigen::Quaterniond cur_ori_;	// current orientation
	Eigen::Vector3d cur_vel_;	// current velocity
	Eigen::Vector3d cur_ang_vel_;	// current angular velocity

	/* Param */
	std::string origin_frame_; // Origin frame of uav i.e. "world" or "map"
	std::string base_link_frame_; // base link frame of uav

	double pub_cmd_freq_; // [Hz] Frequency to publish PVA commands
	double pub_odom_freq_; // [Hz] Frequency of state machine ticks
	double set_offb_ctrl_freq_; // [Hz] Frequency of state machine ticks
	double takeoff_height_; // [m] Take off height

private:
	void initParams();

	/* Time callbacks */
	void setOffboardTimerCB();
	void pubCmdTimerCB();
	void pubOdomTimerCB();

	/* Publisher methods */
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
	void publishOffboardCtrlMode(const int& offb_ctrl_mode);
	void publishTrajectorySetpoint(const Eigen::Vector3d& pos_enu, const double& yaw);
	void publishActuatorCmds(const Eigen::Vector4d& motor_in );
	void publishTorqueSetpoint(const Eigen::Vector3d& torques);
	void publishThrustSetpoint(const Eigen::Vector3d& thrust_vec);

	/* Subscriber methods */
	void odometrySubCB(const VehicleOdometry::UniquePtr msg);
};

void TrajectoryServer::initParams()
{
	this->declare_parameter("origin_frame", "world");
	this->declare_parameter("base_link_frame", "base_link");

	this->declare_parameter("takeoff_height", 1.0);
	this->declare_parameter("set_offb_ctrl_freq", 4.0);
	this->declare_parameter("pub_odom_freq", 30.0);
	this->declare_parameter("pub_cmd_freq", 30.0);

	origin_frame_ = this->get_parameter("origin_frame").as_string();
	base_link_frame_ = this->get_parameter("base_link_frame").as_string();

	takeoff_height_ = this->get_parameter("takeoff_height").as_double();
	set_offb_ctrl_freq_ = this->get_parameter("set_offb_ctrl_freq").as_double();
	pub_odom_freq_ = this->get_parameter("pub_odom_freq").as_double();
	pub_cmd_freq_ = this->get_parameter("pub_cmd_freq").as_double();

	std::cout << "pub_odom_freq: " << pub_odom_freq_ << std::endl;
	std::cout << "pub_cmd_freq: " << pub_cmd_freq_ << std::endl;
}

void TrajectoryServer::odometrySubCB(const VehicleOdometry::UniquePtr msg)
{
	// float32[3] position         # Position in meters. Frame of reference defined by local_frame. NaN if invalid/unknown
	// float32[4] q                # Quaternion rotation from FRD body frame to reference frame. First value NaN if invalid/unknown
	// float32[3] velocity         # Velocity in meters/sec. Frame of reference defined by velocity_frame variable. NaN if invalid/unknown
	// float32[3] angular_velocity # Angular velocity in body-fixed frame (rad/s). NaN if invalid/unknown

	// For frame transforms, refer to https://docs.px4.io/main/en/ros2/user_guide.html

	std::vector<double> position_d = std::vector<double>(msg->position.begin(), msg->position.end());
	std::vector<double> q_d = {msg->q[1], msg->q[2], msg->q[3], msg->q[0] };
	std::vector<double> velocity_d = std::vector<double>(msg->velocity.begin(), msg->velocity.end());
	std::vector<double> angular_velocity_d = std::vector<double>(msg->angular_velocity.begin(), msg->angular_velocity.end());

	std::cout << "Current pose (before): " << Eigen::Vector3f(msg->position.data()).transpose() << std::endl;
	std::cout << "Yaw (before): " << frame_transforms::utils::quaternion::quaternion_get_yaw(Eigen::Quaterniond(q_d.data())) << std::endl;

	auto pos_frame_tf = frame_transforms::StaticTF::NED_TO_ENU;
	auto vel_frame_tf = frame_transforms::StaticTF::NED_TO_ENU;

	if (msg->pose_frame == VehicleOdometry::POSE_FRAME_NED){	// NED Earth-fixed frame
		pos_frame_tf = frame_transforms::StaticTF::NED_TO_ENU;
	}
	else if (msg->pose_frame == VehicleOdometry::POSE_FRAME_FRD) { // FRD world-fixed frame
		pos_frame_tf = frame_transforms::StaticTF::NED_TO_ENU;
	}

	cur_pos_ = transform_static_frame(Eigen::Vector3d(position_d.data()), pos_frame_tf);
	cur_ori_ = transform_orientation(Eigen::Quaterniond(q_d.data()), pos_frame_tf);

	if (msg->pose_frame == VehicleOdometry::POSE_FRAME_NED){	// NED Earth-fixed frame
		vel_frame_tf = frame_transforms::StaticTF::NED_TO_ENU;
	}
	if (msg->pose_frame == VehicleOdometry::POSE_FRAME_FRD) { // FRD world-fixed frame
		vel_frame_tf = frame_transforms::StaticTF::NED_TO_ENU;
	}

	cur_vel_ = transform_static_frame(Eigen::Vector3d(velocity_d.data()), vel_frame_tf);
	cur_ang_vel_ = transform_static_frame(Eigen::Vector3d(angular_velocity_d.data()), vel_frame_tf);

	std::cout << "Current pose (after): " << cur_pos_.transpose() << std::endl;
	std::cout << "Yaw (After): " << frame_transforms::utils::quaternion::quaternion_get_yaw(cur_ori_) << std::endl;
}

void TrajectoryServer::setOffboardTimerCB()
{
	// offboard_control_mode needs to be paired with trajectory_setpoint
	publishOffboardCtrlMode(offboard_ctrl_mode_);

	if (offboard_setpoint_counter_ == 10) {
		// Change to Offboard mode after 10 setpoints

		// VEHICLE_CMD_DO_SET_MODE: |Mode, as defined by ENUM MAV_MODE| Custom mode |
		this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

		// Arm the vehicle
		this->arm();
	}

	offboard_setpoint_counter_++;
}

void TrajectoryServer::pubOdomTimerCB()
{
	nav_msgs::msg::Odometry odom_msg;

	odom_msg.header.frame_id = origin_frame_;
	odom_msg.header.stamp = this->get_clock()->now();

	odom_msg.child_frame_id = base_link_frame_;

	odom_msg.pose.pose.position.x = cur_pos_(0);
	odom_msg.pose.pose.position.y = cur_pos_(1);
	odom_msg.pose.pose.position.z = cur_pos_(2);

	odom_msg.pose.pose.orientation.w = cur_ori_.w();
	odom_msg.pose.pose.orientation.x = cur_ori_.x();
	odom_msg.pose.pose.orientation.y = cur_ori_.y();
	odom_msg.pose.pose.orientation.z = cur_ori_.z();

	odom_msg.twist.twist.linear.x = cur_vel_(0);
	odom_msg.twist.twist.linear.y = cur_vel_(1);
	odom_msg.twist.twist.linear.z = cur_vel_(2);

	odom_msg.twist.twist.angular.x = cur_ang_vel_(0);
	odom_msg.twist.twist.angular.y = cur_ang_vel_(1);
	odom_msg.twist.twist.angular.z = cur_ang_vel_(2);

	odom_pub_->publish(odom_msg);
}

// Publish actuator commands
void TrajectoryServer::pubCmdTimerCB()
{
	switch (offboard_ctrl_mode_){
	case 0:	//Position/Velocity/Acceleration mode
		publishTrajectorySetpoint(Eigen::Vector3d(0.0, 1.0, 2.0), 1.57);
		break;
	case 1: //Thrust/Torque mode
		// publishTorqueSetpoint();
		// publishThrustSetpoint();
		break;
	case 2: //Actuator mode
		// publishActuatorCmds();
		break;
	default: 
		break;
	}
}

/**
 * @brief Send a command to Arm the vehicle
 */
void TrajectoryServer::arm()
{
	// VEHICLE_CMD_COMPONENT_ARM_DISARM: Arms / Disarms a component |1 to arm, 0 to disarm
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, VehicleCommand::ARMING_ACTION_ARM);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void TrajectoryServer::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, VehicleCommand::ARMING_ACTION_DISARM);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void TrajectoryServer::publishOffboardCtrlMode(const int& offb_ctrl_mode)
{
	OffboardControlMode msg{};

	if (offb_ctrl_mode == 0){ // Position mode
		msg.position = true;
		msg.velocity = false;
		msg.acceleration = false;
		msg.attitude = false;
		msg.body_rate = false;
		msg.thrust_and_torque = false;
		msg.direct_actuator = false;
	}
	if (offb_ctrl_mode == 1){ // thrust_and_torque mode
		msg.position = false;
		msg.velocity = false;
		msg.acceleration = false;
		msg.attitude = false;
		msg.body_rate = false;
		msg.thrust_and_torque = true;
		msg.direct_actuator = false;
	}
	if (offb_ctrl_mode == 2){ // actuator mode
		msg.position = false;
		msg.velocity = false;
		msg.acceleration = false;
		msg.attitude = false;
		msg.body_rate = false;
		msg.thrust_and_torque = false;
		msg.direct_actuator = true;
	}

	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_pub_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void TrajectoryServer::publishTrajectorySetpoint(const Eigen::Vector3d& pos_enu, const double& yaw)
{
	TrajectorySetpoint msg{};
	// Convert from ENU to NED
	Eigen::Vector3d pos_ned = transform_static_frame(pos_enu, frame_transforms::StaticTF::ENU_TO_NED);

	msg.position = {(float) pos_ned(0), (float) pos_ned(1), (float) pos_ned(2)};
	msg.yaw = (float) yaw; // [-PI:PI]
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_pub_->publish(msg);
}

void TrajectoryServer::publishActuatorCmds(const Eigen::Vector4d& motor_in )
{
	ActuatorMotors msg{};

	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;	// In microseconds

	// msg.reversible_flags     # bitset which motors are configured to be reversible
	// float32[12] control # range: [-1, 1], where 1 means maximum positive thrust,
	// 					# -1 maximum negative (if not supported by the output, <0 maps to NaN),
	// 					# and NaN maps to disarmed (stop the motors)

	msg.control[0] = (float) motor_in(0);
	msg.control[1] = (float) motor_in(1);
	msg.control[2] = (float) motor_in(2);
	msg.control[3] = (float) motor_in(3);

	actuator_cmd_pub_->publish(msg);
}

void TrajectoryServer::publishTorqueSetpoint(const Eigen::Vector3d& torques)
{
	VehicleTorqueSetpoint msg{};

	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;	// In microseconds

	// xyz: torque setpoint about X, Y, Z body axis (normalized)
	msg.xyz = {(float) torques(0), (float) torques(1), (float) torques(2)};

	torque_setpoint_pub_->publish(msg);
}

void TrajectoryServer::publishThrustSetpoint(const Eigen::Vector3d& thrust_vec)
{
	VehicleThrustSetpoint msg{};

	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;	// In microseconds
	
	// xyz: thrust setpoint along X, Y, Z body axis [-1, 1]
	msg.xyz = {(float) thrust_vec(0), (float) thrust_vec(1), (float) thrust_vec(2)};

	thrust_setpoint_pub_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void TrajectoryServer::publish_vehicle_command(uint16_t command, float param1, float param2)
{
	VehicleCommand msg{};
	msg.param1 = param1;	// param1 is used to set the base_mode variable. 1 is MAV_MODE_FLAG_CUSTOM_MODE_ENABLED. See mavlink's MAV_MODE_FLAG 
	// For param2 and param3, refer to https://github.com/PX4/PX4-Autopilot/blob/0186d687b2ac3d62789806d341bd868b388c2504/src/modules/commander/px4_custom_mode.h
	msg.param2 = param2;	// custom_main_mode. 6 is PX4_CUSTOM_MAIN_MODE::PX4_CUSTOM_MAIN_MODE_OFFBOARD
	msg.command = command;	// command id
	msg.target_system = 1;		// System which should execute the command
	msg.target_component = 1;	// Component which should execute the command, 0 for all components
	msg.source_system = 1;		// System sending the command
	msg.source_component = 1;	//  Component / mode executor sending the command
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

	vehicle_command_pub_->publish(msg);
}

int main(int argc, char *argv[])
{
	std::cout << "Starting trajectory server node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::executors::MultiThreadedExecutor executor;
	auto node = std::make_shared<TrajectoryServer>();
	executor.add_node(node);
	executor.spin();

	rclcpp::shutdown();
	return 0;
}
