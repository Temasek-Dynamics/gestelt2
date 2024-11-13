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

#include <fake_drone/fake_drone.hpp>

FakeDrone::FakeDrone()
: Node("fake_drone")
{
	sim_update_cb_group_ = this->create_callback_group(
		rclcpp::CallbackGroupType::Reentrant);

    fcu_cb_group_ = this->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);

    traj_sp_cb_group_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

	std::string param_ns = "fake_drone";

	vehicle_status_msg_ = std::make_unique<VehicleStatus>();
	vehicle_odom_msg_ = std::make_unique<VehicleOdometry>();
	
	this->declare_parameter(param_ns+".drone_id", -1);

	this->declare_parameter(param_ns+".state_update_frequency", -1.0);

	this->declare_parameter(param_ns+".init_x", 0.0);
	this->declare_parameter(param_ns+".init_y", 0.0);
	this->declare_parameter(param_ns+".init_yaw", 0.0);

	drone_id_ = this->get_parameter(param_ns+".drone_id").as_int();

	double state_update_freq = this->get_parameter(param_ns+".state_update_frequency").as_double();

	Eigen::Vector3d init_pos;
	init_pos(0) = this->get_parameter(param_ns+".init_x").as_double();
	init_pos(1) = this->get_parameter(param_ns+".init_y").as_double();
	init_pos(2) = 0.0; 
	double init_yaw = this->get_parameter(param_ns+".init_yaw").as_double();

	Eigen::Vector3d init_pos_ned = vecENUToNED(init_pos);
	vehicle_odom_msg_->position = {(float)init_pos_ned(0), (float)init_pos_ned(1), (float)init_pos_ned(2)}; 

	Eigen::Quaterniond quat = RPYToQuaternion(0.0, 0.0, init_yaw);
	vehicle_odom_msg_->q = {(float)quat.w(),(float)quat.x(),(float)quat.y(),(float)quat.z() }; 

	vehicle_status_msg_->arming_state = false;
	vehicle_status_msg_->nav_state = VehicleStatus::NAVIGATION_STATE_POSCTL;

	/* Subscribers */
    auto fcu_topics_opt = rclcpp::SubscriptionOptions();
	fcu_topics_opt.callback_group = fcu_cb_group_;

	auto traj_sp_opt = rclcpp::SubscriptionOptions();
	traj_sp_opt.callback_group = traj_sp_cb_group_;

	trajectory_setpoint_sub_ = this->create_subscription<TrajectorySetpoint>(
		"fmu/in/trajectory_setpoint", rclcpp::SystemDefaultsQoS(), 
		std::bind(&FakeDrone::trajectorySetpointSubCB, this, _1), traj_sp_opt);

	offboard_control_mode_sub_ = this->create_subscription<OffboardControlMode>(
		"fmu/in/offboard_control_mode", rclcpp::SystemDefaultsQoS(), 
		std::bind(&FakeDrone::offboardCtrlModeSubCB, this, _1), fcu_topics_opt);

	vehicle_command_sub_ = this->create_subscription<VehicleCommand>(
		"fmu/in/vehicle_command", rclcpp::SystemDefaultsQoS(), 
		std::bind(&FakeDrone::VehicleCommandSubCB, this, _1), fcu_topics_opt);

	/* Publishers */
	fcu_odom_pub_ = this->create_publisher<VehicleOdometry>(
		"fmu/out/vehicle_odometry", rclcpp::SensorDataQoS());

	vehicle_status_pub_ = this->create_publisher<VehicleStatus>(
		"fmu/out/vehicle_status", rclcpp::SensorDataQoS());

	/**
	 * Timer for drone state update  
	*/
	state_update_timer_ = this->create_wall_timer((1.0/state_update_freq) *1000ms, 
                                            std::bind(&FakeDrone::stateUpdateTimerCB, this), 
                                            sim_update_cb_group_);

	printf("[fake_drone] drone%d created with start_pose [%.2lf %.2lf %.2lf]! \n", 
		drone_id_, init_pos(0), init_pos(1), init_pos(2));
	
}

FakeDrone::~FakeDrone()
{
}

/* Subscriber Callbacks*/

void FakeDrone::trajectorySetpointSubCB(const TrajectorySetpoint::UniquePtr &msg)
{
	if (vehicle_status_msg_->nav_state == VehicleStatus::NAVIGATION_STATE_AUTO_LAND){
		// TODO: land vehicle
	}
	else if (vehicle_status_msg_->nav_state == VehicleStatus::NAVIGATION_STATE_OFFBOARD)
	{
		{
			std::lock_guard<std::mutex> state_mutex_guard(state_mutex_);
			if (!std::isnan(msg->position[0]) && !std::isnan(msg->position[1]) && !std::isnan(msg->position[2]))
			{
				vehicle_odom_msg_->position = {msg->position[0], msg->position[1], msg->position[2]};
			}
			if (!std::isnan(msg->velocity[0]) && !std::isnan(msg->velocity[1]) && !std::isnan(msg->velocity[2]))
			{
				vehicle_odom_msg_->velocity = {msg->velocity[0], msg->velocity[1], msg->velocity[2]};
			}
			Eigen::Quaterniond quat = RPYToQuaternion(0.0, 0.0, (double) msg->yaw);
			// quaternion in order of (w,x,y,z)
			vehicle_odom_msg_->q = {(float)quat.w(),(float)quat.x(),(float)quat.y(),(float)quat.z() }; 
		}
	}

}

void FakeDrone::offboardCtrlModeSubCB(const OffboardControlMode::UniquePtr &msg)
{
	// TODO: add timeout if frequency < 2 Hz
}

void FakeDrone::VehicleCommandSubCB(const VehicleCommand::UniquePtr &msg)
{
	switch (msg->command){
		case VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM:
			if (msg->param1 == VehicleCommand::ARMING_ACTION_DISARM){
				vehicle_status_msg_->arming_state = false;
			}	
			else if (msg->param1 == VehicleCommand::ARMING_ACTION_ARM){
					vehicle_status_msg_->arming_state = true;
			}
			else {
				printf("[fake_drone] drone%d received invalid vehicle_command \n", drone_id_);
			}
			break;
		case VehicleCommand::VEHICLE_CMD_DO_SET_MODE:
			if (msg->param1 != 1){
				printf("[fake_drone] drone%d received invalid vehicle_command \n", drone_id_);
				return;
			}
			else{
				if (msg->param2 == PX4_CUSTOM_MAIN_MODE::PX4_CUSTOM_MAIN_MODE_OFFBOARD){
					vehicle_status_msg_->nav_state = VehicleStatus::NAVIGATION_STATE_OFFBOARD;
				}
				else if (msg->param2 == PX4_CUSTOM_MAIN_MODE::PX4_CUSTOM_MAIN_MODE_AUTO){
					if (msg->param3 == PX4_CUSTOM_SUB_MODE_AUTO::PX4_CUSTOM_SUB_MODE_AUTO_LAND)
					{
						vehicle_status_msg_->nav_state = VehicleStatus::NAVIGATION_STATE_AUTO_LAND;
					}
				}
				else {
					printf("[fake_drone] drone%d received invalid vehicle_command \n", drone_id_);
					return;
				}
			}
			break;
		default:
			printf("[fake_drone] drone%d received invalid vehicle_command \n", drone_id_);
			break;
	} 
}

/* Timer Callbacks*/

void FakeDrone::stateUpdateTimerCB()
{
	{
		std::lock_guard<std::mutex> state_mutex_guard(state_mutex_);
		fcu_odom_pub_->publish(*vehicle_odom_msg_);
	}
	vehicle_status_pub_->publish(*vehicle_status_msg_);
}
