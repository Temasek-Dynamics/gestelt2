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

TrajectoryServer::TrajectoryServer() 
: Node("trajectory_server")
{
	logger_ = std::make_shared<logger_wrapper::LoggerWrapper>(this->get_logger(), this->get_clock());
	logger_->logInfo("Initializing");

	initParams();

	// start UAV state machine
	fsm_list::start();

	// Create callback groups
    control_cb_group_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    fcu_cb_group_ = this->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
    others_cb_group_ = this->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);

	initPubSubTimers();

	initSrv();
}

TrajectoryServer::~TrajectoryServer()
{
}

void TrajectoryServer::init()
{
	// Initialize Trajectory command reader
	poly_traj_cmd_ = std::make_unique<PolyTrajCmd>(this->shared_from_this());

	logger_->logInfo("Initialized");
}



void TrajectoryServer::initParams()
{
	/**
	 * Declare params
	 */

	this->declare_parameter("origin_frame", "world");
	this->declare_parameter("base_link_frame", "base_link");

	/* Offboard params */
	this->declare_parameter("offboard_control_mode", 1);

	/* Frequencies for timers and periodic publishers*/
	this->declare_parameter("set_offb_ctrl_freq", 4.0);
	this->declare_parameter("pub_state_freq", 30.0);
	this->declare_parameter("pub_ctrl_freq", 30.0);
	this->declare_parameter("state_machine_tick_freq", 30.0);

	this->declare_parameter("trajectory_type", "");

	/**
	 * Get params
	 */
	origin_frame_ = this->get_parameter("origin_frame").as_string();
	base_link_frame_ = this->get_parameter("base_link_frame").as_string();

	/* Frequencies for timers and periodic publishers*/
	set_offb_ctrl_freq_ = this->get_parameter("set_offb_ctrl_freq").as_double();
	pub_state_freq_ = this->get_parameter("pub_state_freq").as_double();
	pub_ctrl_freq_ = this->get_parameter("pub_ctrl_freq").as_double();
	sm_tick_freq_ = this->get_parameter("state_machine_tick_freq").as_double();

	auto getTrajAdaptorType = [=](const std::string& name) -> TrajectoryType
	{
		// Check all states
		if (name == "MINCO"){
			return TrajectoryType::MINCO;
		}
		else {
			return TrajectoryType::MINCO;
		}
	};

	traj_type_ = getTrajAdaptorType(this->get_parameter("trajectory_type").as_string());
}


void TrajectoryServer::initPubSubTimers()
{
    auto fcu_sub_opt = rclcpp::SubscriptionOptions();
		fcu_sub_opt.callback_group = fcu_cb_group_;

	/* Publishers */
	vehicle_command_pub_ = this->create_publisher<VehicleCommand>("fmu/in/vehicle_command", 10);

	offboard_control_mode_pub_ = this->create_publisher<OffboardControlMode>("fmu/in/offboard_control_mode", 10);
	
	trajectory_setpoint_pub_ = this->create_publisher<TrajectorySetpoint>("fmu/in/trajectory_setpoint", 10);
	actuator_cmd_pub_ = this->create_publisher<ActuatorMotors>("fmu/in/actuator_motors", 10);
	torque_setpoint_pub_ = this->create_publisher<VehicleTorqueSetpoint>("fmu/in/vehicle_torque_setpoint", 10);
	thrust_setpoint_pub_ = this->create_publisher<VehicleThrustSetpoint>("fmu/in/vehicle_thrust_setpoint", 10);

	odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

	uav_state_pub_ = this->create_publisher<gestelt_interfaces::msg::UAVState>("uav_state", 10);

	/* Subscribers */
	odometry_sub_ = this->create_subscription<VehicleOdometry>(
		"fmu/out/vehicle_odometry", rclcpp::SensorDataQoS(), 
		std::bind(&TrajectoryServer::odometrySubCB, this, _1), fcu_sub_opt);

	vehicle_status_sub_ = this->create_subscription<VehicleStatus>(
		"fmu/out/vehicle_status", rclcpp::SensorDataQoS(), 
		std::bind(&TrajectoryServer::vehicleStatusSubCB, this, _1), fcu_sub_opt);

	/* Timers */
	pub_ctrl_timer_ = this->create_wall_timer((1.0/pub_ctrl_freq_)*1000ms, 
												std::bind(&TrajectoryServer::pubCtrlTimerCB, this), control_cb_group_);
	sm_tick_timer_ = this->create_wall_timer((1.0/sm_tick_freq_)*1000ms, 
												std::bind(&TrajectoryServer::SMTickTimerCB, this), others_cb_group_);
	set_offb_timer_ = this->create_wall_timer((1.0/set_offb_ctrl_freq_) *1000ms, 
												std::bind(&TrajectoryServer::setOffboardTimerCB, this), others_cb_group_);
	pub_state_timer_ = this->create_wall_timer((1.0/pub_state_freq_) *1000ms, 
												std::bind(&TrajectoryServer::pubStateTimerCB, this), others_cb_group_);

}

void TrajectoryServer::initSrv()
{
	uav_cmd_srv_ = this->create_service<gestelt_interfaces::srv::UAVCommand>("uav_command", 
																			std::bind(&TrajectoryServer::uavCmdSrvCB, this, 
																						_1, _2),
																			rclcpp::ServicesQoS(),
																			others_cb_group_);
}

/****************** */
/* SUBSCRIBER CALLBACKS */
/****************** */

void TrajectoryServer::vehicleStatusSubCB(const VehicleStatus::UniquePtr msg)
{
	arming_state_ = msg->arming_state;
	nav_state_ = msg->nav_state;
	pre_flight_checks_pass_ = msg->pre_flight_checks_pass;

	logger_->logInfoThrottle(strFmt("arming_state[%d], nav_state[%d], pre_flight_checks_pass_[%d]", 
								arming_state_, nav_state_, pre_flight_checks_pass_), 1.0);

	connected_to_fcu_ = true;
	{
		// # Encodes the system state of the vehicle published by commander

		// uint64 timestamp # time since system start (microseconds)

		// uint64 armed_time # Arming timestamp (microseconds)
		// uint64 takeoff_time # Takeoff timestamp (microseconds)

		// uint8 arming_state
		// uint8 ARMING_STATE_DISARMED = 1
		// uint8 ARMING_STATE_ARMED    = 2

		// uint8 latest_arming_reason
		// uint8 latest_disarming_reason
		// uint8 ARM_DISARM_REASON_TRANSITION_TO_STANDBY = 0
		// uint8 ARM_DISARM_REASON_STICK_GESTURE = 1
		// uint8 ARM_DISARM_REASON_RC_SWITCH = 2
		// uint8 ARM_DISARM_REASON_COMMAND_INTERNAL = 3
		// uint8 ARM_DISARM_REASON_COMMAND_EXTERNAL = 4
		// uint8 ARM_DISARM_REASON_MISSION_START = 5
		// uint8 ARM_DISARM_REASON_SAFETY_BUTTON = 6
		// uint8 ARM_DISARM_REASON_AUTO_DISARM_LAND = 7
		// uint8 ARM_DISARM_REASON_AUTO_DISARM_PREFLIGHT = 8
		// uint8 ARM_DISARM_REASON_KILL_SWITCH = 9
		// uint8 ARM_DISARM_REASON_LOCKDOWN = 10
		// uint8 ARM_DISARM_REASON_FAILURE_DETECTOR = 11
		// uint8 ARM_DISARM_REASON_SHUTDOWN = 12
		// uint8 ARM_DISARM_REASON_UNIT_TEST = 13

		// uint64 nav_state_timestamp # time when current nav_state activated

		// uint8 nav_state_user_intention                  # Mode that the user selected (might be different from nav_state in a failsafe situation)

		// uint8 nav_state                                 # Currently active mode
		// uint8 NAVIGATION_STATE_MANUAL = 0               # Manual mode
		// uint8 NAVIGATION_STATE_ALTCTL = 1               # Altitude control mode
		// uint8 NAVIGATION_STATE_POSCTL = 2               # Position control mode
		// uint8 NAVIGATION_STATE_AUTO_MISSION = 3         # Auto mission mode
		// uint8 NAVIGATION_STATE_AUTO_LOITER = 4          # Auto loiter mode
		// uint8 NAVIGATION_STATE_AUTO_RTL = 5             # Auto return to launch mode
		// uint8 NAVIGATION_STATE_POSITION_SLOW = 6
		// uint8 NAVIGATION_STATE_FREE5 = 7
		// uint8 NAVIGATION_STATE_FREE4 = 8
		// uint8 NAVIGATION_STATE_FREE3 = 9
		// uint8 NAVIGATION_STATE_ACRO = 10                # Acro mode
		// uint8 NAVIGATION_STATE_FREE2 = 11
		// uint8 NAVIGATION_STATE_DESCEND = 12             # Descend mode (no position control)
		// uint8 NAVIGATION_STATE_TERMINATION = 13         # Termination mode
		// uint8 NAVIGATION_STATE_OFFBOARD = 14
		// uint8 NAVIGATION_STATE_STAB = 15                # Stabilized mode
		// uint8 NAVIGATION_STATE_FREE1 = 16
		// uint8 NAVIGATION_STATE_AUTO_TAKEOFF = 17        # Takeoff
		// uint8 NAVIGATION_STATE_AUTO_LAND = 18           # Land
		// uint8 NAVIGATION_STATE_AUTO_FOLLOW_TARGET = 19  # Auto Follow
		// uint8 NAVIGATION_STATE_AUTO_PRECLAND = 20       # Precision land with landing target
		// uint8 NAVIGATION_STATE_ORBIT = 21               # Orbit in a circle
		// uint8 NAVIGATION_STATE_AUTO_VTOL_TAKEOFF = 22   # Takeoff, transition, establish loiter
		// uint8 NAVIGATION_STATE_EXTERNAL1 = 23
		// uint8 NAVIGATION_STATE_EXTERNAL2 = 24
		// uint8 NAVIGATION_STATE_EXTERNAL3 = 25
		// uint8 NAVIGATION_STATE_EXTERNAL4 = 26
		// uint8 NAVIGATION_STATE_EXTERNAL5 = 27
		// uint8 NAVIGATION_STATE_EXTERNAL6 = 28
		// uint8 NAVIGATION_STATE_EXTERNAL7 = 29
		// uint8 NAVIGATION_STATE_EXTERNAL8 = 30
		// uint8 NAVIGATION_STATE_MAX = 31

		// uint8 executor_in_charge                        # Current mode executor in charge (0=Autopilot)

		// uint32 valid_nav_states_mask                    # Bitmask for all valid nav_state values
		// uint32 can_set_nav_states_mask                  # Bitmask for all modes that a user can select

		// # Bitmask of detected failures
		// uint16 failure_detector_status
		// uint16 FAILURE_NONE = 0
		// uint16 FAILURE_ROLL = 1              # (1 << 0)
		// uint16 FAILURE_PITCH = 2             # (1 << 1)
		// uint16 FAILURE_ALT = 4               # (1 << 2)
		// uint16 FAILURE_EXT = 8               # (1 << 3)
		// uint16 FAILURE_ARM_ESC = 16          # (1 << 4)
		// uint16 FAILURE_BATTERY = 32          # (1 << 5)
		// uint16 FAILURE_IMBALANCED_PROP = 64  # (1 << 6)
		// uint16 FAILURE_MOTOR = 128           # (1 << 7)

		// uint8 hil_state
		// uint8 HIL_STATE_OFF = 0
		// uint8 HIL_STATE_ON = 1

		// uint8 FAILSAFE_DEFER_STATE_DISABLED = 0
		// uint8 FAILSAFE_DEFER_STATE_ENABLED = 1
		// uint8 FAILSAFE_DEFER_STATE_WOULD_FAILSAFE = 2 # Failsafes deferred, but would trigger a failsafe

		// bool failsafe # true if system is in failsafe state (e.g.:RTL, Hover, Terminate, ...)
		// bool failsafe_and_user_took_over # true if system is in failsafe state but the user took over control
		// uint8 failsafe_defer_state # one of FAILSAFE_DEFER_STATE_*

		// # Link loss
		// bool gcs_connection_lost              # datalink to GCS lost
		// uint8 gcs_connection_lost_counter     # counts unique GCS connection lost events
		// bool high_latency_data_link_lost # Set to true if the high latency data link (eg. RockBlock Iridium 9603 telemetry module) is lost

		// # MAVLink identification
		// uint8 system_type  # system type, contains mavlink MAV_TYPE
		// uint8 system_id	   # system id, contains MAVLink's system ID field
		// uint8 component_id # subsystem / component id, contains MAVLink's component ID field

		// bool safety_button_available # Set to true if a safety button is connected
		// bool safety_off # Set to true if safety is off

		// bool power_input_valid                            # set if input power is valid
		// bool usb_connected                                # set to true (never cleared) once telemetry received from usb link

		// bool open_drone_id_system_present
		// bool open_drone_id_system_healthy

		// bool avoidance_system_required                    # Set to true if avoidance system is enabled via COM_OBS_AVOID parameter
		// bool avoidance_system_valid                       # Status of the obstacle avoidance system

		// bool rc_calibration_in_progress
		// bool calibration_enabled

		// bool pre_flight_checks_pass		# true if all checks necessary to arm pass
	}

}

void TrajectoryServer::odometrySubCB(const VehicleOdometry::UniquePtr msg)
{
	// float32[3] position         # Position in meters. Frame of reference defined by local_frame. NaN if invalid/unknown
	// float32[4] q                # Quaternion rotation from FRD body frame to reference frame. First value NaN if invalid/unknown
	// float32[3] velocity         # Velocity in meters/sec. Frame of reference defined by velocity_frame variable. NaN if invalid/unknown
	// float32[3] angular_velocity # Angular velocity in body-fixed frame (rad/s). NaN if invalid/unknown

	// For frame transforms, refer to https://docs.px4.io/main/en/ros2/user_guide.html

	std::vector<double> position_d = std::vector<double>(msg->position.begin(), msg->position.end());
	std::vector<double> q_d = {msg->q[1], msg->q[2], msg->q[3], msg->q[0] }; // PX4 uses (w,x,y,z), We reorder to (x,y,z,w) 
	std::vector<double> velocity_d = std::vector<double>(msg->velocity.begin(), msg->velocity.end());
	std::vector<double> angular_velocity_d = std::vector<double>(msg->angular_velocity.begin(), msg->angular_velocity.end());

	// std::cout << "Current pose (before): " << Eigen::Vector3f(msg->position.data()).transpose() << std::endl;
	// std::cout << "Yaw (before): " << frame_transforms::utils::quaternion::quaternion_get_yaw(Eigen::Quaterniond(q_d.data())) << std::endl;

	auto pos_frame_tf = frame_transforms::StaticTF::NED_TO_ENU;
	auto vel_frame_tf = frame_transforms::StaticTF::NED_TO_ENU;

	if (msg->pose_frame == VehicleOdometry::POSE_FRAME_NED){	// NED Earth-fixed frame
		pos_frame_tf = frame_transforms::StaticTF::NED_TO_ENU;
	}
	else if (msg->pose_frame == VehicleOdometry::POSE_FRAME_FRD) { // FRD world-fixed frame
		pos_frame_tf = frame_transforms::StaticTF::NED_TO_ENU;
	}

	if (msg->velocity_frame == VehicleOdometry::VELOCITY_FRAME_NED){	// NED Earth-fixed frame
		vel_frame_tf = frame_transforms::StaticTF::NED_TO_ENU;
	}
	if (msg->velocity_frame == VehicleOdometry::VELOCITY_FRAME_FRD) { // FRD world-fixed frame
		vel_frame_tf = frame_transforms::StaticTF::NED_TO_ENU;
	}

	cur_pos_ = transform_static_frame(Eigen::Vector3d(position_d.data()), pos_frame_tf);
	
	cur_pos_corr_ = cur_pos_;
	cur_pos_corr_(2) -=  ground_height_; // Adjust for ground height

	cur_ori_ = transform_orientation(Eigen::Quaterniond(q_d.data()), pos_frame_tf);

	cur_vel_ = transform_static_frame(Eigen::Vector3d(velocity_d.data()), vel_frame_tf);
	cur_ang_vel_ = transform_static_frame(Eigen::Vector3d(angular_velocity_d.data()), vel_frame_tf);

	// std::cout << "Current pose (after): " << cur_pos_.transpose() << std::endl;
	// std::cout << "Yaw (After): " << frame_transforms::utils::quaternion::quaternion_get_yaw(cur_ori_) << std::endl;
}

/****************** */
/* TIMER CALLBACKS */
/****************** */

void TrajectoryServer::setOffboardTimerCB()
{
	gestelt_interfaces::msg::UAVState uav_state;

	// Check all states
	if (UAV::is_in_state<Unconnected>()){
		uav_state.state = gestelt_interfaces::msg::UAVState::UNCONNECTED;
	}
	else if (UAV::is_in_state<Idle>()){
		uav_state.state = gestelt_interfaces::msg::UAVState::IDLE;
	}
	else if (UAV::is_in_state<Landing>()){
		uav_state.state = gestelt_interfaces::msg::UAVState::LANDING;

		if (nav_state_ != VehicleStatus::NAVIGATION_STATE_AUTO_LAND)
		{
			// Set to land mode
			this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 
											PX4_CUSTOM_MAIN_MODE::PX4_CUSTOM_MAIN_MODE_AUTO,
											PX4_CUSTOM_SUB_MODE_AUTO::PX4_CUSTOM_SUB_MODE_AUTO_LAND);
		}
	}
	else if (UAV::is_in_state<TakingOff>()){
		uav_state.state = gestelt_interfaces::msg::UAVState::TAKINGOFF;

		if (arming_state_ != VehicleStatus::ARMING_STATE_ARMED)
		{ 
			// Arm vehicle
			this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 
											VehicleCommand::ARMING_ACTION_ARM);
		}

		if (nav_state_ != VehicleStatus::NAVIGATION_STATE_OFFBOARD)
		{ 
			// Set to custom (offboard) mode
			this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 
											PX4_CUSTOM_MAIN_MODE::PX4_CUSTOM_MAIN_MODE_OFFBOARD);
		}

		// PERIODICALLY: Publish offboard control mode message 
		publishOffboardCtrlMode(0);	// Position control
	}
	else if (UAV::is_in_state<Hovering>()){
		uav_state.state = gestelt_interfaces::msg::UAVState::HOVERING;
		
		// PERIODICALLY: Publish offboard control mode message 
		publishOffboardCtrlMode(0);	// Position control
	}
	else if (UAV::is_in_state<Mission>()){
		uav_state.state = gestelt_interfaces::msg::UAVState::MISSION;
		
		// PERIODICALLY: Publish offboard control mode message
		publishOffboardCtrlMode(fsm_list::fsmtype::current_state_ptr->getControlMode());
	}
	else if (UAV::is_in_state<EmergencyStop>()){
		uav_state.state = gestelt_interfaces::msg::UAVState::EMERGENCYSTOP;
	}
	else {
		uav_state.state = gestelt_interfaces::msg::UAVState::UNDEFINED;
	}

	uav_state_pub_->publish(uav_state);
}

void TrajectoryServer::pubCtrlTimerCB()
{
	static double take_off_hover_T = 1.0/5.0;// Take off and landing period
	static double estop_T = 1.0/50.0;// EStop period

	// Check all states
	if (UAV::is_in_state<Unconnected>()){
		// Do nothing
	}
	else if (UAV::is_in_state<Idle>()){
		// Do nothing
	}
	else if (UAV::is_in_state<Landing>()){
		// Do nothing
	}
	else if (UAV::is_in_state<TakingOff>()){
		if (this->get_clock()->now().seconds() - last_cmd_pub_t_ > take_off_hover_T){
			publishTrajectorySetpoint(
				Eigen::Vector3d(0.0, 0.0, 
					fsm_list::fsmtype::current_state_ptr->getTakeoffHeight() + ground_height_), 
				Eigen::Vector2d(0.0, 0.0));

			last_cmd_pub_t_ = this->get_clock()->now().seconds();
		}

	}
	else if (UAV::is_in_state<Hovering>()){
		if (this->get_clock()->now().seconds() - last_cmd_pub_t_ > take_off_hover_T){
			publishTrajectorySetpoint(
				Eigen::Vector3d(0.0, 0.0, 
					fsm_list::fsmtype::current_state_ptr->getTakeoffHeight() + ground_height_), 
				Eigen::Vector2d(0.0, 0.0));

			last_cmd_pub_t_ = this->get_clock()->now().seconds();
		}

	}
	else if (UAV::is_in_state<Mission>()){
		switch (fsm_list::fsmtype::current_state_ptr->getControlMode()){
			case gestelt_interfaces::srv::UAVCommand::Request::MODE_TRAJECTORY:
				if (poly_traj_cmd_->getCmd(pos_enu_, yaw_yawrate_, vel_enu_, acc_enu_))
				{
					pos_enu_(2) +=  ground_height_; // Adjust for ground height
					publishTrajectorySetpoint(pos_enu_, yaw_yawrate_, vel_enu_);
				}
				break;
			case gestelt_interfaces::srv::UAVCommand::Request::MODE_ATTITUDE: 
				// publishAttitudeSetpoint(1.0, Eigen::Vector4d(0.0, 0.0, 0.0, 1.0));
				break;
			case gestelt_interfaces::srv::UAVCommand::Request::MODE_RATES: 
				// publishRatesSetpoint(1.0, Eigen::Vector3d(0.0, 0.0, 0.0));
				break;
			case gestelt_interfaces::srv::UAVCommand::Request::MODE_THRUST_TORQUE: 
				// publishTorqueThrustSetpoint(1.0, Eigen::Vector3d(0.0, 0.0, 0.0));
				break;
			case gestelt_interfaces::srv::UAVCommand::Request::MODE_MOTORS:
				// publishActuatorCmds(Eigen::Vector4d(1.0, 1.0, 1.0, 1.0));
				break;
			default:
				// Undefined. Do nothing
				break;
		}
	}
	else if (UAV::is_in_state<EmergencyStop>()){
		if (this->get_clock()->now().seconds() - last_cmd_pub_t_ > estop_T){
			// Disarm vehicle forcefully
			if (arming_state_ != VehicleStatus::ARMING_STATE_DISARMED)
			{ 
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 
										VehicleCommand::ARMING_ACTION_DISARM,
										21196);
			}

			last_cmd_pub_t_ = this->get_clock()->now().seconds();
		}
	}
	else {
	}

}

void TrajectoryServer::SMTickTimerCB()
{
	// Check all states
	if (UAV::is_in_state<Unconnected>()){
		logger_->logInfoThrottle("[Unconnected]", 1.0);
		if (connected_to_fcu_){
			sendEvent(Idle_E());
		}
	}
	else if (UAV::is_in_state<Idle>()){
		// logger_->logInfoThrottle("[Idle]", 1.0);
		if (arming_state_ != VehicleStatus::ARMING_STATE_DISARMED)
		{ 
			// Disarm vehicle
			this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 
									VehicleCommand::ARMING_ACTION_DISARM);
		}

		ground_height_ = cur_pos_(2); // Set ground height to last z value
	}
	else if (UAV::is_in_state<Landing>()){
		logger_->logInfoThrottle("[Landing]", 1.0);
		if (arming_state_ == VehicleStatus::ARMING_STATE_DISARMED)
		{
			sendEvent(Idle_E());
		}
	}
	else if (UAV::is_in_state<TakingOff>()){
		logger_->logInfoThrottle("[TakingOff]", 1.0);
		if (abs(cur_pos_corr_(2) - fsm_list::fsmtype::current_state_ptr->getTakeoffHeight()) < take_off_landing_tol_)
		{
			sendEvent(Hover_E());
		}
	}
	else if (UAV::is_in_state<Hovering>()){
		logger_->logInfoThrottle("[Hovering]", 1.0);
	}
	else if (UAV::is_in_state<Mission>()){
		// logger_->logInfoThrottle("[Mission]", 1.0);
	}
	else if (UAV::is_in_state<EmergencyStop>()){
		// logger_->logInfoThrottle("[EmergencyStop]", 1.0);
	}
	else {
		// logger_->logInfoThrottle("Undefined UAV state", 1.0);
	}
}

void TrajectoryServer::pubStateTimerCB()
{
	nav_msgs::msg::Odometry odom_msg;

	odom_msg.header.frame_id = origin_frame_;
	odom_msg.header.stamp = this->get_clock()->now();

	odom_msg.child_frame_id = base_link_frame_;

	odom_msg.pose.pose.position.x = cur_pos_corr_(0);
	odom_msg.pose.pose.position.y = cur_pos_corr_(1);
	odom_msg.pose.pose.position.z = cur_pos_corr_(2);

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

/****************** */
/* PUBLISHER METHODS */
/****************** */

void TrajectoryServer::publishOffboardCtrlMode(const int& offb_ctrl_mode)
{
	OffboardControlMode msg{};

	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	msg.position = false;
	msg.velocity = false;
	msg.acceleration = false;

	msg.attitude = false;
	msg.body_rate = false;
	msg.thrust_and_torque = false;
	msg.direct_actuator = false;

	switch (offb_ctrl_mode){
		case gestelt_interfaces::srv::UAVCommand::Request::MODE_TRAJECTORY: //0
			msg.position = true;
			msg.velocity = true;
			msg.acceleration = true;
			break;
		case gestelt_interfaces::srv::UAVCommand::Request::MODE_ATTITUDE: //1
			msg.attitude = true;
			break;
		case gestelt_interfaces::srv::UAVCommand::Request::MODE_RATES: //2
			msg.body_rate = true;
			break;
		case gestelt_interfaces::srv::UAVCommand::Request::MODE_THRUST_TORQUE: //3
			msg.thrust_and_torque = true;
			break;
		case gestelt_interfaces::srv::UAVCommand::Request::MODE_MOTORS: //4
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
	TrajectorySetpoint msg{};

	// # NED local world frame
	// float32[3] position # in meters
	// float32[3] velocity # in meters/second
	// float32[3] acceleration # in meters/second^2
	// float32[3] jerk # in meters/second^3 (for logging only)

	// float32 yaw # euler angle of desired attitude in radians -PI..+PI
	// float32 yawspeed # angular velocity around NED frame z-axis in radians/second

	// Convert from ENU to NED
	Eigen::Vector3d pos_ned = transform_static_frame(pos, frame_transforms::StaticTF::ENU_TO_NED);
	Eigen::Vector3d vel_ned = transform_static_frame(vel, frame_transforms::StaticTF::ENU_TO_NED);
	Eigen::Vector3d acc_ned = transform_static_frame(acc, frame_transforms::StaticTF::ENU_TO_NED);

	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	msg.position = {(float) pos_ned(0), (float) pos_ned(1), (float) pos_ned(2)};
	msg.velocity = {(float) vel_ned(0), (float) vel_ned(1), (float) vel_ned(2)};
	msg.acceleration = {(float) acc_ned(0), (float) acc_ned(1), (float) acc_ned(2)};

	msg.yaw = (float) yaw_yawrate(0); // [-PI:PI]
	msg.yawspeed = (float) yaw_yawrate(1); // angular velocity around NED frame z-axis in radians/second

	trajectory_setpoint_pub_->publish(msg);
}

void TrajectoryServer::publishAttitudeSetpoint(const double& thrust, const Eigen::Vector4d& q_d)
{
	VehicleAttitudeSetpoint msg{};
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;	// In microseconds

	// body angular rates in FRD frame
	msg.q_d = {(float)q_d(0), (float)q_d(1), (float)q_d(2), (float)q_d(3)};

	// Normalized thrust command in body NED frame [-1,1]
	msg.thrust_body = {0.0, 0.0, (float)thrust}; // NED Frame

	attitude_setpoint_pub_->publish(msg);
}

void TrajectoryServer::publishRatesSetpoint(const double& thrust, const Eigen::Vector3d& rates)
{
	VehicleRatesSetpoint msg{};
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
	VehicleTorqueSetpoint torque_msg{};
	VehicleThrustSetpoint thrust_msg{};

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

void TrajectoryServer::publish_vehicle_command(uint16_t command, float param1, float param2, float param3)
{
	// Refer to https://docs.px4.io/main/en/msg_docs/VehicleCommand.html for message format
	VehicleCommand msg{};
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

/****************** */
/* SERVICE CALLBACKS*/
/****************** */

void TrajectoryServer::uavCmdSrvCB(const std::shared_ptr<gestelt_interfaces::srv::UAVCommand::Request> request,
          								std::shared_ptr<gestelt_interfaces::srv::UAVCommand::Response>  response)
{
	// request->header
	logger_->logInfo(strFmt("Incoming request\n Command: %d" " Mode: %d" " Value: %f",
					request->command, request->mode, request->value));


	// Checkn if value and mode is within bounds for specific commands
	switch (request->command)
	{
		case gestelt_interfaces::srv::UAVCommand::Request::COMMAND_TAKEOFF:  
			if (request->value < 0.5 || request->value > 3.0){
				response->success = false;
				response->state = (int) getUAVState();
				response->state_name = getUAVStateString();
				logger_->logError("Value for COMMAND_TAKEOFF should be between 0.5 and 3.0, inclusive");

				return;
			}

			break;
		case gestelt_interfaces::srv::UAVCommand::Request::COMMAND_START_MISSION: 

			if (request->mode < 0 || request->mode > 4){
				response->success = false;
				response->state = (int) getUAVState();
				response->state_name = getUAVStateString();
				logger_->logError("Value for COMMAND_START_MISSION should be between 0 and 4, inclusive");

				return;
			}

			break;
		default:                    
			break;
	}

	static double takeoff_timeout = 10.0;
	static double start_mission_timeout = 1.0;
	static double land_timeout = 10.0;
	static double stop_mission_timeout = 1.0;
	static double estop_timeout = 0.1;

	// Send event to state machine
	sendUAVCommandEvent(request->command, request->value, request->mode);

	double srv_rcv_t = this->get_clock()->now().seconds();
	bool timeout = false;

	// Wait for state change to be completed
	switch (request->command)
	{
		case gestelt_interfaces::srv::UAVCommand::Request::COMMAND_TAKEOFF:  
			logger_->logInfo("COMMAND_TAKEOFF");

			// Wait for state change to complete
			while (!UAV::is_in_state<Hovering>() && !timeout)
			{
				if ((srv_rcv_t - this->get_clock()->now().seconds()) >= takeoff_timeout){
					timeout = true;
					break;
				}
				rclcpp::sleep_for(std::chrono::milliseconds(25));
			}
			break;
		case gestelt_interfaces::srv::UAVCommand::Request::COMMAND_LAND:  
			logger_->logInfo("COMMAND_LAND");

			while (!UAV::is_in_state<Idle>() && !timeout)
			{
				if ((srv_rcv_t - this->get_clock()->now().seconds()) >= land_timeout){
					timeout = true;
					break;
				}
				rclcpp::sleep_for(std::chrono::milliseconds(25));
			}
			break;
		case gestelt_interfaces::srv::UAVCommand::Request::COMMAND_START_MISSION: 
			logger_->logInfo("COMMAND_START_MISSION");

			while (!UAV::is_in_state<Mission>() && !timeout)
			{
				if ((srv_rcv_t - this->get_clock()->now().seconds()) >= start_mission_timeout){
					timeout = true;
					break;
				}
				rclcpp::sleep_for(std::chrono::milliseconds(25));
			}
			break;
		case gestelt_interfaces::srv::UAVCommand::Request::COMMAND_STOP_MISSION: 
			logger_->logInfo("COMMAND_STOP_MISSION");

			while (!UAV::is_in_state<Hovering>() && !timeout)
			{
				if ((srv_rcv_t - this->get_clock()->now().seconds()) >= stop_mission_timeout){
					timeout = true;
					break;
				}
				rclcpp::sleep_for(std::chrono::milliseconds(25));
			}

			break;
		case gestelt_interfaces::srv::UAVCommand::Request::COMMAND_EMERGENCY_STOP: 
			logger_->logInfo("COMMAND_EMERGENCY_STOP");

			while (!UAV::is_in_state<EmergencyStop>() && !timeout)
			{
				if ((srv_rcv_t - this->get_clock()->now().seconds()) >= estop_timeout){
					timeout = true;
					break;
				}
				rclcpp::sleep_for(std::chrono::milliseconds(25));
			}

			break;
		default:                    
			break;
	}

	response->state = (int) getUAVState();
	response->state_name = getUAVStateString();

	if (timeout){
		response->success = false;

		logger_->logInfo(strFmt("Timeout!!: State[%d] State_name[%s] Success [%d]", 
			response->state, response->state_name.c_str(), response->success));
	}
	else {
		response->success = true;

		logger_->logInfo(strFmt("Success! Sending back response: State[%d] State_name[%s] Success [%d]", 
			response->state, response->state_name.c_str(), response->success));
	}

}


/****************** */
/* HELPER METHODS */
/****************** */

int main(int argc, char *argv[])
{
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);

	rclcpp::init(argc, argv);
	rclcpp::executors::MultiThreadedExecutor executor;

	auto node = std::make_shared<TrajectoryServer>();

	node->init();

	executor.add_node(node);
	executor.spin();

	rclcpp::shutdown();
	return 0;
}



