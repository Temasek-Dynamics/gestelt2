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

/**
 * @brief Trajectory Server
 * @file trajectory_server.cpp
 * @author John Tan <johntgz@nus.edu.sg>
 */

#include <trajectory_server/trajectory_server.hpp>

#include <frame_transforms.hpp>

TrajectoryServer::TrajectoryServer()
	: Node("trajectory_server")
{
	logger_ = std::make_shared<logger_wrapper::LoggerWrapper>(this->get_logger(), this->get_clock());

	logger_->logInfo("Initializing");

	fsm_list::start(); // start UAV state machine
 
	tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
	tf_static_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);

	geofence_ = std::make_unique<Geofence>(logger_);

	// Declare params
	this->declare_parameter("namespace", "");
	this->declare_parameter("map_frame", "map");
	this->declare_parameter("base_link_frame", "base_link");
	this->declare_parameter("camera_frame", "camera_link");
	/* Frequencies for timers and periodic publishers*/
	this->declare_parameter("set_offb_ctrl_freq", 4.0);
	this->declare_parameter("pub_state_freq", 30.0);
	this->declare_parameter("pub_ctrl_freq", 30.0);
	this->declare_parameter("state_machine_tick_freq", 30.0);
	this->declare_parameter("publish_map_to_baselink_tf", true);
	this->declare_parameter("publish_base_link_to_camera_tf", true);
	this->declare_parameter("transform_cmd_from_nwu_to_enu", true);

	this->declare_parameter("cmd_rot_x", 0.0);
	this->declare_parameter("cmd_rot_y", 0.0);
	this->declare_parameter("cmd_rot_z", 0.0);

	/* Safety */
	geofence_->min_x = this->declare_parameter("safety.geofence.min_x", 0.0);
	geofence_->min_y = this->declare_parameter("safety.geofence.min_y", 0.0);
	geofence_->min_z = this->declare_parameter("safety.geofence.min_z", 0.0);
	geofence_->max_x = this->declare_parameter("safety.geofence.max_x", 0.0);
	geofence_->max_y = this->declare_parameter("safety.geofence.max_y", 0.0);
	geofence_->max_z = this->declare_parameter("safety.geofence.max_z", 0.0);

	// Get Params
	std::string ns = this->get_parameter("namespace").as_string();
	if (ns.size() < 2){
		throw std::runtime_error("Invalid namespace provided to trajectory_server, "
			"should be in the form of 'dX' where X is an integer");
	}
	drone_id_ = std::stoi(ns.substr(1, ns.size()-1));

	map_frame_ = this->get_parameter("map_frame").as_string();
	base_link_frame_ = this->get_parameter("base_link_frame").as_string();
	camera_frame_ = this->get_parameter("camera_frame").as_string();
	set_offb_ctrl_freq_ = this->get_parameter("set_offb_ctrl_freq").as_double();
	pub_state_freq_ = this->get_parameter("pub_state_freq").as_double();
	pub_ctrl_freq_ = this->get_parameter("pub_ctrl_freq").as_double();
	sm_tick_freq_ = this->get_parameter("state_machine_tick_freq").as_double();

	pub_map_to_baselink_tf_ = this->get_parameter("publish_map_to_baselink_tf").as_bool();
	pub_baselink_to_camera_tf_ = this->get_parameter("publish_base_link_to_camera_tf").as_bool();
	transform_cmd_from_nwu_to_enu_ = this->get_parameter("transform_cmd_from_nwu_to_enu").as_bool();
	cmd_rot_z_ = this->get_parameter("cmd_rot_z").as_double();
	cmd_rot_y_ = this->get_parameter("cmd_rot_y").as_double();
	cmd_rot_x_ = this->get_parameter("cmd_rot_x").as_double();

	// Create callback groups
	control_cb_group_ = this->create_callback_group(
		rclcpp::CallbackGroupType::MutuallyExclusive);
	fcu_cb_group_ = this->create_callback_group(
		rclcpp::CallbackGroupType::Reentrant);
	others_cb_group_ = this->create_callback_group(
		rclcpp::CallbackGroupType::Reentrant);

	auto fcu_sub_opt = rclcpp::SubscriptionOptions();
	fcu_sub_opt.callback_group = fcu_cb_group_;

	/* Publishers */
	vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
		"fmu/in/vehicle_command", 10);
	offboard_control_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
		"fmu/in/offboard_control_mode", 10);
	trajectory_setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
		"fmu/in/trajectory_setpoint", 10);
	actuator_cmd_pub_ = this->create_publisher<px4_msgs::msg::ActuatorMotors>(
		"fmu/in/actuator_motors", 10);
	torque_setpoint_pub_ = this->create_publisher<px4_msgs::msg::VehicleTorqueSetpoint>(
		"fmu/in/vehicle_torque_setpoint", 10);
	thrust_setpoint_pub_ = this->create_publisher<px4_msgs::msg::VehicleThrustSetpoint>(
		"fmu/in/vehicle_thrust_setpoint", 10);

	odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
		"odom", rclcpp::SensorDataQoS());

	uav_state_pub_ = this->create_publisher<gestelt_interfaces::msg::UAVState>(
		"uav_state", 5);

	/* Subscribers */
	fcu_odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
		"fmu/out/vehicle_odometry", rclcpp::SensorDataQoS(), 
		std::bind(&TrajectoryServer::odometrySubCB, this, _1), fcu_sub_opt);

	vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
		"fmu/out/vehicle_status", rclcpp::SensorDataQoS(), 
		std::bind(&TrajectoryServer::vehicleStatusSubCB, this, _1), fcu_sub_opt);

	lin_mpc_cmd_sub_ = this->create_subscription<px4_msgs::msg::TrajectorySetpoint>(
		"intermediate_cmd", rclcpp::SensorDataQoS(),
		std::bind(&TrajectoryServer::intmdCmdSubCB, this, _1), fcu_sub_opt);

	global_uav_cmd_sub_ = this->create_subscription<gestelt_interfaces::msg::AllUAVCommand>(
		"/global_uav_command", rclcpp::ServicesQoS(),
		std::bind(&TrajectoryServer::globalUAVCmdSubCB, this, _1), fcu_sub_opt);

	uav_cmd_sub_ = this->create_subscription<gestelt_interfaces::msg::AllUAVCommand>(
		"uav_command", rclcpp::ServicesQoS(),
		std::bind(&TrajectoryServer::UAVCmdSubCB, this, _1), fcu_sub_opt);

	/* Timers */
	pub_ctrl_timer_ = this->create_wall_timer((1.0 / pub_ctrl_freq_) * 1000ms,
		std::bind(&TrajectoryServer::pubCtrlTimerCB, this), control_cb_group_);
	sm_tick_timer_ = this->create_wall_timer((1.0 / sm_tick_freq_) * 1000ms,
		std::bind(&TrajectoryServer::SMTickTimerCB, this), others_cb_group_);
	set_offb_timer_ = this->create_wall_timer((1.0 / set_offb_ctrl_freq_) * 1000ms,
		std::bind(&TrajectoryServer::setOffboardTimerCB, this), others_cb_group_);
	pub_state_timer_ = this->create_wall_timer((1.0 / pub_state_freq_) * 1000ms,
		std::bind(&TrajectoryServer::pubStateTimerCB, this), others_cb_group_);
												
	logger_->logInfo("Initialized");
}

TrajectoryServer::~TrajectoryServer()
{}

/****************** */
/* SUBSCRIBER CALLBACKS */
/****************** */

void TrajectoryServer::odometrySubCB(const px4_msgs::msg::VehicleOdometry::UniquePtr msg)
{
	// float32[3] position         # Position in meters. Frame of reference defined by local_frame. NaN if invalid/unknown
	// float32[4] q                # Quaternion rotation from FRD body frame to reference frame. First value NaN if invalid/unknown
	// float32[3] velocity         # Velocity in meters/sec. Frame of reference defined by velocity_frame variable. NaN if invalid/unknown
	// float32[3] angular_velocity # Angular velocity in body-fixed frame (rad/s). NaN if invalid/unknown

	// For frame transforms, refer to https://docs.px4.io/main/en/ros2/user_guide.html

	std::vector<double> pos = std::vector<double>(msg->position.begin(), msg->position.end());
	// std::vector<double> q_d = {msg->q[0], msg->q[1], msg->q[2], msg->q[3] }; // in order of (w,x,y,z)
	std::vector<double> vel = std::vector<double>(msg->velocity.begin(), msg->velocity.end());
	std::vector<double> ang_vel = std::vector<double>(msg->angular_velocity.begin(), msg->angular_velocity.end());

	auto pos_frame_tf = frame_transforms::StaticTF::NED_TO_ENU;
	if (msg->pose_frame == px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED 
		|| msg->pose_frame == px4_msgs::msg::VehicleOdometry::POSE_FRAME_FRD){	// NED Earth-fixed frame or FRD world-fixed frame
		pos_frame_tf = frame_transforms::StaticTF::NED_TO_ENU;
	}
	auto vel_frame_tf = frame_transforms::StaticTF::NED_TO_ENU;
	if (msg->velocity_frame == px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED
		|| msg->velocity_frame == px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_FRD){	// NED Earth-fixed frame or FRD world-fixed frame
		vel_frame_tf = frame_transforms::StaticTF::NED_TO_ENU;
	}

	cur_pos_enu_ = transform_static_frame(
		Eigen::Vector3d(pos.data()), pos_frame_tf);

	if (UAV::is_in_state<Idle>())
	{

		logger_->logInfoThrottle(strFmt("Using current height %f as ground height", 
										cur_pos_enu_(2)), 5.0);
		// Set ground height as the height at which the UAV is resting on the ground
		ground_height_ = Eigen::Vector3d(0.0, 0.0, cur_pos_enu_(2));
	}

	// Correct for ground height
	cur_pos_enu_corr_ = cur_pos_enu_ - ground_height_;
	cur_ori_enu_ = transform_orientation(
		frame_transforms::utils::quaternion::array_to_eigen_quat(msg->q), pos_frame_tf);
	cur_vel_enu_ = transform_static_frame(
		Eigen::Vector3d(vel.data()), vel_frame_tf);
	cur_ang_vel_enu_ = transform_static_frame(
		Eigen::Vector3d(ang_vel.data()), vel_frame_tf);


	nav_msgs::msg::Odometry odom_msg;

	odom_msg.header.frame_id = map_frame_;
	odom_msg.header.stamp = this->get_clock()->now();

	odom_msg.child_frame_id = base_link_frame_;

	odom_msg.pose.pose.position.x = cur_pos_enu_corr_(0);
	odom_msg.pose.pose.position.y = cur_pos_enu_corr_(1);
	odom_msg.pose.pose.position.z = cur_pos_enu_corr_(2);

	odom_msg.pose.pose.orientation.w = cur_ori_enu_.w();
	odom_msg.pose.pose.orientation.x = cur_ori_enu_.x();
	odom_msg.pose.pose.orientation.y = cur_ori_enu_.y();
	odom_msg.pose.pose.orientation.z = cur_ori_enu_.z();

	odom_msg.twist.twist.linear.x = cur_vel_enu_(0);
	odom_msg.twist.twist.linear.y = cur_vel_enu_(1);
	odom_msg.twist.twist.linear.z = cur_vel_enu_(2);

	odom_msg.twist.twist.angular.x = cur_ang_vel_enu_(0);
	odom_msg.twist.twist.angular.y = cur_ang_vel_enu_(1);
	odom_msg.twist.twist.angular.z = cur_ang_vel_enu_(2);

	odom_pub_->publish(odom_msg);
}

void TrajectoryServer::vehicleStatusSubCB(const px4_msgs::msg::VehicleStatus::UniquePtr msg)
{
	arming_state_ = msg->arming_state;
	nav_state_ = msg->nav_state;
	pre_flight_checks_pass_ = msg->pre_flight_checks_pass;

	// if (!pre_flight_checks_pass_){
	// 	logger_->logErrorThrottle("Failed preflight checks!", 1.0);
	// }

	// logger_->logInfoThrottle(strFmt("arming_state[%d], nav_state[%d], pre_flight_checks_pass_[%d]", 
	// 							arming_state_, nav_state_, pre_flight_checks_pass_), 1.0);

	connected_to_fcu_ = true;
}


void TrajectoryServer::intmdCmdSubCB(const px4_msgs::msg::TrajectorySetpoint::UniquePtr msg)
{
	// Message received in ENU frame
	if (UAV::is_in_state<Mission>())
	{
		cmd_pos_enu_ = Eigen::Vector3d(
			msg->position[0], msg->position[1], msg->position[2]);
		cmd_pos_enu_corr_ = cmd_pos_enu_ + ground_height_;

		cmd_vel_enu_ = Eigen::Vector3d(
			msg->velocity[0], msg->velocity[1], msg->velocity[2]);
		cmd_acc_enu_ = Eigen::Vector3d(
			msg->acceleration[0], msg->acceleration[1], msg->acceleration[2]);
		// cmd_jerk_enu_ = Eigen::Vector3d(msg->jerk[0], msg->jerk[1], msg->jerk[2]);

		cmd_yaw_yawrate_(0) = msg->yaw;
		cmd_yaw_yawrate_(1) = msg->yawspeed;
	}
}

void TrajectoryServer::UAVCmdSubCB(const gestelt_interfaces::msg::AllUAVCommand::UniquePtr msg)
{
	logger_->logInfo(strFmt("Incoming request\n Command: %d"
							" Mode: %d"
							" Value: %f",
							msg->command, msg->mode, msg->value));

	// Check if value and mode is within bounds for specific commands
	switch (msg->command)
	{
	case gestelt_interfaces::msg::AllUAVCommand::COMMAND_TAKEOFF:
		if (msg->value < 0.5 || msg->value > 3.0)
		{
			logger_->logError("Value for COMMAND_TAKEOFF should be between 0.5 and 3.0, inclusive");

			return;
		}

		break;
	case gestelt_interfaces::msg::AllUAVCommand::COMMAND_START_MISSION:

		if (msg->mode > 4)
		{
			logger_->logError("Value for COMMAND_START_MISSION should be between 0 and 4 inclusive");

			return;
		}
		break;
	default:
		break;
	}

	// Send event to state machine
	sendUAVCommandEvent(msg->command, msg->value, msg->mode);
}

void TrajectoryServer::globalUAVCmdSubCB(const gestelt_interfaces::msg::AllUAVCommand::UniquePtr msg)
{
	logger_->logInfo(strFmt("Incoming request\n Command: %d"
							" Mode: %d"
							" Value: %f",
							msg->command, msg->mode, msg->value));

	// Check if value and mode is within bounds for specific commands
	switch (msg->command)
	{
	case gestelt_interfaces::msg::AllUAVCommand::COMMAND_TAKEOFF:
		if (msg->value < 0.5 || msg->value > 3.0)
		{
			logger_->logError("Value for COMMAND_TAKEOFF should be between 0.5 and 3.0, inclusive");

			return;
		}

		break;
	case gestelt_interfaces::msg::AllUAVCommand::COMMAND_START_MISSION:

		if (msg->mode > 4)
		{
			logger_->logError("Value for COMMAND_START_MISSION should be between 0 and 4 inclusive");

			return;
		}
		break;
	default:
		break;
	}

	// Send event to state machine
	sendUAVCommandEvent(msg->command, msg->value, msg->mode);
}


/****************** */
/* TIMER CALLBACKS */
/****************** */

void TrajectoryServer::setOffboardTimerCB()
{
	gestelt_interfaces::msg::UAVState uav_state;

	// Check all states
	if (UAV::is_in_state<Unconnected>())
	{
		uav_state.state = gestelt_interfaces::msg::UAVState::UNCONNECTED;
	}
	else if (UAV::is_in_state<Idle>())
	{
		uav_state.state = gestelt_interfaces::msg::UAVState::IDLE;
	}
	else if (UAV::is_in_state<Landing>())
	{
		uav_state.state = gestelt_interfaces::msg::UAVState::LANDING;

		if (nav_state_ != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LAND)
		{
			// Set to land mode
			this->publish_vehicle_command(
				px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 
				PX4_CUSTOM_MAIN_MODE::PX4_CUSTOM_MAIN_MODE_AUTO,
				PX4_CUSTOM_SUB_MODE_AUTO::PX4_CUSTOM_SUB_MODE_AUTO_LAND);
		}
	}
	else if (UAV::is_in_state<TakingOff>())
	{
		uav_state.state = gestelt_interfaces::msg::UAVState::TAKINGOFF;

		if (arming_state_ != px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED)
		{ 
			// Arm vehicle
			this->publish_vehicle_command(
				px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 
				px4_msgs::msg::VehicleCommand::ARMING_ACTION_ARM);
		}

		if (nav_state_ != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD)
		{ 
			// Set to custom (offboard) mode
			this->publish_vehicle_command(
				px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 
				PX4_CUSTOM_MAIN_MODE::PX4_CUSTOM_MAIN_MODE_OFFBOARD);
		}

		// PERIODICALLY: Publish offboard control mode message 
		publishOffboardCtrlMode(0);	// Position control
	}
	else if (UAV::is_in_state<Hovering>())
	{
		uav_state.state = gestelt_interfaces::msg::UAVState::HOVERING;

		// PERIODICALLY: Publish offboard control mode message 
		publishOffboardCtrlMode(0);	// Position control
	}
	else if (UAV::is_in_state<Mission>())
	{
		uav_state.state = gestelt_interfaces::msg::UAVState::MISSION;
		
		// PERIODICALLY: Publish offboard control mode message
		publishOffboardCtrlMode(fsm_list::fsmtype::current_state_ptr->getControlMode());
	}
	else if (UAV::is_in_state<EmergencyStop>())
	{
		uav_state.state = gestelt_interfaces::msg::UAVState::EMERGENCYSTOP;
	}
	else
	{
		uav_state.state = gestelt_interfaces::msg::UAVState::UNDEFINED;
	}

	uav_state_pub_->publish(uav_state);
}

void TrajectoryServer::pubCtrlTimerCB()
{
	static double take_off_hover_T = 1.0/5.0;// Take off and landing period

	// Check all states
	if (UAV::is_in_state<Unconnected>())
	{
		// Do nothing
	}
	else if (UAV::is_in_state<Idle>())
	{
		// Do nothing
	}
	else if (UAV::is_in_state<Landing>())
	{
		// Do nothing
	}
	else if (UAV::is_in_state<TakingOff>())
	{
		publishTrajectorySetpoint(cmd_pos_enu_corr_, cmd_yaw_yawrate_);

		if (this->get_clock()->now().seconds() - last_cmd_pub_t_ > take_off_hover_T){

			last_cmd_pub_t_ = this->get_clock()->now().seconds();
		}
	}
	else if (UAV::is_in_state<Hovering>())
	{
		publishTrajectorySetpoint(cmd_pos_enu_corr_, cmd_yaw_yawrate_);

		if (this->get_clock()->now().seconds() - last_cmd_pub_t_ > take_off_hover_T){

			last_cmd_pub_t_ = this->get_clock()->now().seconds();
		}
	}
	else if (UAV::is_in_state<Mission>())
	{
		switch (fsm_list::fsmtype::current_state_ptr->getControlMode()){
			case gestelt_interfaces::msg::AllUAVCommand::MODE_TRAJECTORY:
				publishTrajectorySetpoint(
					cmd_pos_enu_corr_, cmd_yaw_yawrate_, cmd_vel_enu_, cmd_acc_enu_);
				break;
			case gestelt_interfaces::msg::AllUAVCommand::MODE_ATTITUDE: 
				logger_->logErrorThrottle("ATTITUDE CONTROL UNIMPLEMENTED", 1.0);
				// publishAttitudeSetpoint(1.0, Eigen::Vector4d(0.0, 0.0, 0.0, 1.0));
				break;
			case gestelt_interfaces::msg::AllUAVCommand::MODE_RATES: 
				logger_->logErrorThrottle("RATE CONTROL UNIMPLEMENTED", 1.0);
				// publishRatesSetpoint(1.0, Eigen::Vector3d(0.0, 0.0, 0.0));
				break;
			case gestelt_interfaces::msg::AllUAVCommand::MODE_THRUST_TORQUE: 
				logger_->logErrorThrottle("THRUST_TORQUE CONTROL UNIMPLEMENTED", 1.0);
				// publishTorqueThrustSetpoint(1.0, Eigen::Vector3d(0.0, 0.0, 0.0));
				break;
			case gestelt_interfaces::msg::AllUAVCommand::MODE_MOTORS:
				logger_->logErrorThrottle("MOTORS CONTROL UNIMPLEMENTED", 1.0);
				// publishActuatorCmds(Eigen::Vector4d(1.0, 1.0, 1.0, 1.0));
				break;
			default:
				logger_->logErrorThrottle("UNDEFINED TRAJECTORY MODE", 1.0);
				// Undefined. Do nothing
				break;
		}

	}
	else if (UAV::is_in_state<EmergencyStop>())
	{
		// Disarm vehicle forcefully
		if (arming_state_ != px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED)
		{ 
			this->publish_vehicle_command(
				px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 
				px4_msgs::msg::VehicleCommand::ARMING_ACTION_DISARM,
				21196);
		}

		last_cmd_pub_t_ = this->get_clock()->now().seconds();
	}
	else
	{
		logger_->logErrorThrottle("[UNDEFINED STATE] IN pubCtrlTimerCB", 1.0);
	}
}

void TrajectoryServer::SMTickTimerCB()
{
	// Check all states
	if (UAV::is_in_state<Unconnected>())
	{
		logger_->logWarnThrottle("[Unconnected]", 2.5);
		if (connected_to_fcu_){
			sendEvent(Idle_E());
		}
	}
	else if (UAV::is_in_state<Idle>())
	{
		logger_->logInfoThrottle("[Idle]", 2.5);
		if (arming_state_ != px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED)
		{ 
			// Disarm vehicle
			this->publish_vehicle_command(
				px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 
				px4_msgs::msg::VehicleCommand::ARMING_ACTION_DISARM);
		}

		cmd_pos_enu_ = cur_pos_enu_corr_;

		cmd_yaw_yawrate_(0) = frame_transforms::utils::quaternion::quaternion_get_yaw(cur_ori_enu_); 
		cmd_yaw_yawrate_(1) = NAN; 
	}
	else if (UAV::is_in_state<Landing>())
	{
		logger_->logInfoThrottle("[Landing]", 1.0);

		if (arming_state_ == px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED)
		{
			sendEvent(Idle_E());
		}
	}
	else if (UAV::is_in_state<TakingOff>())
	{
		logger_->logInfoThrottle("[TakingOff]", 1.0);

		cmd_pos_enu_ = cur_pos_enu_corr_;
		cmd_pos_enu_(2) = fsm_list::fsmtype::current_state_ptr->getTakeoffHeight();

		cmd_pos_enu_corr_ = cmd_pos_enu_ + ground_height_;

		if (abs(cur_pos_enu_corr_(2) - fsm_list::fsmtype::current_state_ptr->getTakeoffHeight()) 
			< take_off_landing_tol_)
		{
			sendEvent(Hover_E());
		}
	}
	else if (UAV::is_in_state<Hovering>())
	{
		logger_->logInfoThrottle("[Hovering]", 2.0);
	}
	else if (UAV::is_in_state<Mission>())
	{
		logger_->logInfoThrottle("[Mission]", 5.0);
	}
	else if (UAV::is_in_state<EmergencyStop>())
	{
		logger_->logInfoThrottle("[EmergencyStop]", 1.0);
	}
	else
	{
		logger_->logInfoThrottle("Undefined UAV state", 2.0);
	}
}

void TrajectoryServer::pubStateTimerCB()
{
	if (pub_map_to_baselink_tf_){
		// broadcast tf from map to base link 
		geometry_msgs::msg::TransformStamped map_to_base_link_tf;

		map_to_base_link_tf.header.stamp = this->get_clock()->now();
		map_to_base_link_tf.header.frame_id = map_frame_; 
		map_to_base_link_tf.child_frame_id = base_link_frame_; 

		map_to_base_link_tf.transform.translation.x = cur_pos_enu_corr_(0);
		map_to_base_link_tf.transform.translation.y = cur_pos_enu_corr_(1);
		map_to_base_link_tf.transform.translation.z = cur_pos_enu_corr_(2);

		// Flip y axis
		map_to_base_link_tf.transform.rotation.x = cur_ori_enu_.x();
		map_to_base_link_tf.transform.rotation.y = cur_ori_enu_.y();
		map_to_base_link_tf.transform.rotation.z = cur_ori_enu_.z();
		map_to_base_link_tf.transform.rotation.w = cur_ori_enu_.w();
		
		tf_broadcaster_->sendTransform(map_to_base_link_tf);
	}

	// if (pub_baselink_to_camera_tf_){

	// 	// broadcast tf link from global map frame to local map origin 
	// 	geometry_msgs::msg::TransformStamped base_link_to_camera_tf;

	// 	base_link_to_camera_tf.header.stamp = this->get_clock()->now();
	// 	base_link_to_camera_tf.header.frame_id = "camera_link"; 
	// 	base_link_to_camera_tf.child_frame_id = camera_frame_; 

	// 	base_link_to_camera_tf.transform.translation.x = 0.0;
	// 	base_link_to_camera_tf.transform.translation.y = 0.0;
	// 	base_link_to_camera_tf.transform.translation.z = 0.0;

	// 	base_link_to_camera_tf.transform.rotation.x = 0.0;
	// 	base_link_to_camera_tf.transform.rotation.y = 0.0; 
	// 	base_link_to_camera_tf.transform.rotation.z = 0.0;
	// 	base_link_to_camera_tf.transform.rotation.w = 1.0; 

	// 	tf_static_broadcaster_->sendTransform(base_link_to_camera_tf);
	// }
}

/****************** */
/* PUBLISHER METHODS */
/****************** */

void TrajectoryServer::publishOffboardCtrlMode(const int& offb_ctrl_mode)
{
	px4_msgs::msg::OffboardControlMode msg{};

	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	msg.position = false;
	msg.velocity = false;
	msg.acceleration = false;

	msg.attitude = false;
	msg.body_rate = false;
	msg.thrust_and_torque = false;
	msg.direct_actuator = false;

	switch (offb_ctrl_mode){
		case gestelt_interfaces::msg::AllUAVCommand::MODE_TRAJECTORY: //0
			msg.position = true;
			msg.velocity = false;
			msg.acceleration = false;
			break;
		case gestelt_interfaces::msg::AllUAVCommand::MODE_ATTITUDE: //1
			msg.attitude = true;
			break;
		case gestelt_interfaces::msg::AllUAVCommand::MODE_RATES: //2
			msg.body_rate = true;
			break;
		case gestelt_interfaces::msg::AllUAVCommand::MODE_THRUST_TORQUE: //3
			msg.thrust_and_torque = true;
			break;
		case gestelt_interfaces::msg::AllUAVCommand::MODE_MOTORS: //4
			msg.direct_actuator = true;
			break;
		default:
			// Undefined
			break;
	}
	
	offboard_control_mode_pub_->publish(msg);
}

void TrajectoryServer::publishTrajectorySetpoint(
	const Eigen::Vector3d& pos, 
	const Eigen::Vector2d& yaw_yawrate,
	const Eigen::Vector3d& vel,
	const Eigen::Vector3d& acc)
{
	px4_msgs::msg::TrajectorySetpoint msg{};
	
	// # NED local world frame
	// float32[3] position # in meters
	// float32[3] velocity # in meters/second
	// float32[3] acceleration # in meters/second^2
	// float32[3] jerk # in meters/second^3 (for logging only)

	// float32 yaw # euler angle of desired attitude in radians -PI..+PI
	// float32 yawspeed # angular velocity around NED frame z-axis in radians/second

	Eigen::Vector3d pos_ned, vel_ned, acc_ned;

	if (transform_cmd_from_nwu_to_enu_){
		Eigen::Matrix3d m;
		m = Eigen::AngleAxisd(cmd_rot_z_, Eigen::Vector3d::UnitZ())
			* Eigen::AngleAxisd(cmd_rot_y_,  Eigen::Vector3d::UnitY())
			* Eigen::AngleAxisd(cmd_rot_x_, Eigen::Vector3d::UnitX());

		pos_ned = m * pos;
		vel_ned = m * vel;
		acc_ned = m * acc;
	}

	// Convert from ENU to NED
	pos_ned = transform_static_frame(pos, frame_transforms::StaticTF::ENU_TO_NED);
	vel_ned = transform_static_frame(vel, frame_transforms::StaticTF::ENU_TO_NED);
	acc_ned = transform_static_frame(acc, frame_transforms::StaticTF::ENU_TO_NED);

	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000; // In microseconds
	msg.position = {(float) pos_ned(0), (float) pos_ned(1), (float) pos_ned(2)};
	msg.velocity = {(float) vel_ned(0), (float) vel_ned(1), (float) vel_ned(2)};
	msg.acceleration = {(float) acc_ned(0), (float) acc_ned(1), (float) acc_ned(2)};

	// Correct YAW to transfrom from ENU to NED frame
	float yaw_corr = (float) -yaw_yawrate(0) + M_PI/2;

	// yaw_corr = yaw_corr >= M_PI ? yaw_corr - 2*M_PI : yaw_corr;
	// yaw_corr = yaw_corr <= -M_PI ? yaw_corr + 2*M_PI : yaw_corr;

	msg.yaw = yaw_corr; // [-PI:PI]
	msg.yawspeed = NAN; // angular velocity around NED frame z-axis in radians/second
	// msg.yawspeed = (float) yaw_yawrate(1); // angular velocity around NED frame z-axis in radians/second

	trajectory_setpoint_pub_->publish(msg);
}

void TrajectoryServer::publishAttitudeSetpoint(const double& thrust, const Eigen::Vector4d& q_d)
{
	px4_msgs::msg::VehicleAttitudeSetpoint msg{};
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;	// In microseconds

	// body angular rates in FRD frame
	msg.q_d = {(float)q_d(0), (float)q_d(1), (float)q_d(2), (float)q_d(3)};

	// Normalized thrust command in body NED frame [-1,1]
	msg.thrust_body = {0.0, 0.0, (float)thrust}; // NED Frame

	attitude_setpoint_pub_->publish(msg);
}

void TrajectoryServer::publishRatesSetpoint(const double& thrust, const Eigen::Vector3d& rates)
{
	px4_msgs::msg::VehicleRatesSetpoint msg{};
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;	// In microseconds

	// body angular rates in FRD frame
	msg.roll = rates(0);	// [rad/s] roll rate setpoint
	msg.pitch = rates(1);	// [rad/s] pitch rate setpoint
	msg.yaw = rates(2);		// [rad/s] yaw rate setpoint

	// Normalized thrust command in body NED frame [-1,1]
	msg.thrust_body = {0.0, 0.0, (float)thrust}; // NED Frame

	rates_setpoint_pub_->publish(msg);
}

void TrajectoryServer::publishTorqueThrustSetpoint(const double& thrust, const Eigen::Vector3d& torques)
{
	px4_msgs::msg::VehicleTorqueSetpoint torque_msg{};
	px4_msgs::msg::VehicleThrustSetpoint thrust_msg{};

	torque_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;	// In microseconds
	thrust_msg.timestamp = torque_msg.timestamp;	

	// xyz: torque setpoint about X, Y, Z body axis (normalized)
	torque_msg.xyz = {(float) torques(0), (float) torques(1), (float) torques(2)};
	// Normalized thrust command in body NED frame [-1,1]
	thrust_msg.xyz = {0.0, 0.0, (float)thrust}; // NED Frame

	torque_setpoint_pub_->publish(torque_msg);
	thrust_setpoint_pub_->publish(thrust_msg);
}

void TrajectoryServer::publishActuatorCmds(const Eigen::Vector4d& motor_in )
{
	px4_msgs::msg::ActuatorMotors msg{};

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

void TrajectoryServer::publish_vehicle_command(uint16_t command, float param1, float param2, float param3)
{
	// Refer to https://docs.px4.io/main/en/msg_docs/VehicleCommand.html for message format
	px4_msgs::msg::VehicleCommand msg{};
	msg.param1 = param1;	// param1 is used to set the base_mode variable. 1 is MAV_MODE_FLAG_CUSTOM_MODE_ENABLED. See mavlink's MAV_MODE_FLAG 
	// For param2 and param3, refer to https://github.com/PX4/PX4-Autopilot/blob/0186d687b2ac3d62789806d341bd868b388c2504/src/modules/commander/px4_custom_mode.h
	msg.param2 = param2;	// custom_main_mode. 6 is PX4_CUSTOM_MAIN_MODE::PX4_CUSTOM_MAIN_MODE_OFFBOARD
	msg.param3 = param3;	// sub mode
	msg.command = command;	// command id
	msg.target_system = drone_id_+1;		// System which should execute the command
	msg.target_component = 1;	// Component which should execute the command, 0 for all components
	msg.source_system = drone_id_+1;		// System sending the command
	msg.source_component = 1;	//  Component / mode executor sending the command
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

	vehicle_command_pub_->publish(msg);
}
