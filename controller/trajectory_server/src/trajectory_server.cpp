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

	geofence_ = std::make_unique<Geofence>(logger_);

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
	// poly_traj_cmd_ = std::make_unique<PolyTrajCmd>(this->shared_from_this());

	// Initialize mavros handler
	mavros_handler_ = std::make_unique<MavrosHandler>(this->shared_from_this(), map_frame_);

	logger_->logInfo("Initialized");
}

void TrajectoryServer::initParams()
{
	/**
	 * Declare params
	 */

	this->declare_parameter("drone_id", 0);

	this->declare_parameter("map_frame", "map");
	this->declare_parameter("base_link_frame", "base_link");

	this->declare_parameter("trajectory_type", "");

	this->declare_parameter("fcu_interface", "");

	/* Frequencies for timers and periodic publishers*/
	this->declare_parameter("set_offb_ctrl_freq", 4.0);
	this->declare_parameter("pub_state_freq", 30.0);
	this->declare_parameter("pub_ctrl_freq", 30.0);
	this->declare_parameter("state_machine_tick_freq", 30.0);

	/* Safety */
	nav_state_timeout_ = this->declare_parameter("safety.navigator_state_timeout", 0.5);
	geofence_->min_x = this->declare_parameter("safety.geofence.min_x", 0.0);
	geofence_->min_y = this->declare_parameter("safety.geofence.min_y", 0.0);
	geofence_->min_z = this->declare_parameter("safety.geofence.min_z", 0.0);
	geofence_->max_x = this->declare_parameter("safety.geofence.max_x", 0.0);
	geofence_->max_y = this->declare_parameter("safety.geofence.max_y", 0.0);
	geofence_->max_z = this->declare_parameter("safety.geofence.max_z", 0.0);

	/**
	 * Get params
	 */
	drone_id_ = this->get_parameter("drone_id").as_int();

	map_frame_ = this->get_parameter("map_frame").as_string();
	base_link_frame_ = this->get_parameter("base_link_frame").as_string();

	/* Frequencies for timers and periodic publishers*/
	set_offb_ctrl_freq_ = this->get_parameter("set_offb_ctrl_freq").as_double();
	pub_state_freq_ = this->get_parameter("pub_state_freq").as_double();
	pub_ctrl_freq_ = this->get_parameter("pub_ctrl_freq").as_double();
	sm_tick_freq_ = this->get_parameter("state_machine_tick_freq").as_double();

	auto getTrajAdaptorType = [=](const std::string &name) -> TrajectoryType
	{
		// Check all states
		if (name == "MINCO")
		{
			return TrajectoryType::MINCO;
		}
		else if (name == "MPC")
		{
			return TrajectoryType::MPC;
		}
		else
		{
			return TrajectoryType::UNDEFINED_TRAJ_TYPE;
		}
	};

	traj_type_ = getTrajAdaptorType(this->get_parameter("trajectory_type").as_string());

	auto getFCUInterface = [=](const std::string &name) -> FCUInterface
	{
		if (name == "MAVROS")
		{
			return FCUInterface::MAVROS;
		}
		else if (name == "MICRO_XRCE_DDS")
		{
			return FCUInterface::MICRO_XRCE_DDS;
		}
		else
		{
			return FCUInterface::MAVROS;
		}
	};

	fcu_interface_ = getFCUInterface(this->get_parameter("fcu_interface").as_string());
}

void TrajectoryServer::initPubSubTimers()
{
	auto fcu_sub_opt = rclcpp::SubscriptionOptions();
	fcu_sub_opt.callback_group = fcu_cb_group_;

	/* Publishers */
	uav_state_pub_ = this->create_publisher<gestelt_interfaces::msg::UAVState>(
		"uav_state", 10);

	/* Subscribers */
	navigator_state_sub_ = this->create_subscription<gestelt_interfaces::msg::NavState>(
		"navigator/state", rclcpp::SensorDataQoS(),
		std::bind(&TrajectoryServer::navStateSubCB, this, _1), fcu_sub_opt);

	lin_mpc_cmd_sub_ = this->create_subscription<mavros_msgs::msg::PositionTarget>(
		"navigator/intmd_cmd", rclcpp::SensorDataQoS(),
		std::bind(&TrajectoryServer::linMPCCmdSubCB, this, _1), fcu_sub_opt);

	// Mavros topics
	if (fcu_interface_ == FCUInterface::MAVROS)
	{
		auto state_qos = rclcpp::QoS(10).transient_local();

		/* Publishers */
		vel_magnitude_pub_ = this->create_publisher<std_msgs::msg::Float32>(
			"vel_magnitude", 10);

		/* Subscribers */
		mavros_status_sub_ = this->create_subscription<mavros_msgs::msg::State>(
			"mavros/state", state_qos,
			std::bind(&TrajectoryServer::mavrosStateSubCB, this, _1), fcu_sub_opt);

		mavros_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
			"odom", rclcpp::SensorDataQoS(),
			std::bind(&TrajectoryServer::mavrosOdomSubCB, this, _1), fcu_sub_opt);

		all_uav_cmd_sub_ = this->create_subscription<gestelt_interfaces::msg::AllUAVCommand>(
			"/all_uav_command", rclcpp::ServicesQoS(),
			std::bind(&TrajectoryServer::allUAVCmdSubCB, this, _1), fcu_sub_opt);
	}
	else if (fcu_interface_ == FCUInterface::MICRO_XRCE_DDS)
	{
		logger_->logError("UNIMPLEMENTED FLIGHT CONTROLLER INTERFACE SPECIFIED. Set fcu_interface param to something else. Shutting down...");
		rclcpp::shutdown();
	}
	else
	{
		logger_->logError("INVALID FLIGHT CONTROLLER INTERFACE SPECIFIED. Set fcu_interface param properly. Shutting down...");
		rclcpp::shutdown();
	}

	/* Timers */
	pub_ctrl_timer_ = this->create_wall_timer((1.0 / pub_ctrl_freq_) * 1000ms,
											  std::bind(&TrajectoryServer::pubCtrlTimerCB, this), control_cb_group_);
	sm_tick_timer_ = this->create_wall_timer((1.0 / sm_tick_freq_) * 1000ms,
											 std::bind(&TrajectoryServer::SMTickTimerCB, this), others_cb_group_);
	set_offb_timer_ = this->create_wall_timer((1.0 / set_offb_ctrl_freq_) * 1000ms,
											  std::bind(&TrajectoryServer::setOffboardTimerCB, this), others_cb_group_);
	pub_state_timer_ = this->create_wall_timer((1.0 / pub_state_freq_) * 1000ms,
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
// Mavros

void TrajectoryServer::mavrosStateSubCB(const mavros_msgs::msg::State::UniquePtr msg)
{
	mavros_handler_->setState(msg);
}

void TrajectoryServer::mavrosOdomSubCB(const nav_msgs::msg::Odometry::UniquePtr msg)
{
	mavros_handler_->setOdom(msg);

	cur_pos_ = Eigen::Vector3d{msg->pose.pose.position.x,
							   msg->pose.pose.position.y,
							   msg->pose.pose.position.z};

	cur_vel_ = Eigen::Vector3d{msg->twist.twist.linear.x,
							   msg->twist.twist.linear.y,
							   msg->twist.twist.linear.z};

	// Quaternionf q << quat.x, quat.y, quat.z, quat.w;
	cur_ori_ = Eigen::Quaterniond(msg->pose.pose.orientation.w,
								  msg->pose.pose.orientation.x,
								  msg->pose.pose.orientation.y,
								  msg->pose.pose.orientation.z);

	// Correct for ground height
	cur_pos_corr_ = Eigen::Vector3d(cur_pos_(0), cur_pos_(1), cur_pos_(2) - ground_height_);
}

// Mavigator and state machine command topics

void TrajectoryServer::navStateSubCB(const gestelt_interfaces::msg::NavState::UniquePtr msg)
{
	// process msg
	// msg->state
	// gestelt_interfaces::msg::NavState::IDLE
	// gestelt_interfaces::msg::NavState::PLANNING
	// gestelt_interfaces::msg::NavState::PLANNING_TIMEOUT

	// if (msg->state == gestelt_interfaces::msg::NavState::PLANNING_TIMEOUT){
	// 	// Set to landing
	// 	sendUAVCommandEvent(gestelt_interfaces::msg::AllUAVCommand::COMMAND_LAND, 0, 0);
	// }

	last_nav_heartbeat_ = this->get_clock()->now().seconds();
}

void TrajectoryServer::linMPCCmdSubCB(const mavros_msgs::msg::PositionTarget::UniquePtr msg)
{
	// Message received in ENU frame
	if (UAV::is_in_state<Mission>())
	{
		cmd_pos_enu_ = Eigen::Vector3d(
			msg->position.x, msg->position.y, msg->position.z);
		cmd_vel_enu_ = Eigen::Vector3d(
			msg->velocity.x, msg->velocity.y, msg->velocity.z);
		cmd_acc_enu_ = Eigen::Vector3d(
			msg->acceleration_or_force.x, msg->acceleration_or_force.y, msg->acceleration_or_force.z);

		cmd_yaw_yawrate_(0) = msg->yaw;
		cmd_yaw_yawrate_(1) = msg->yaw_rate;
	}
}

void TrajectoryServer::allUAVCmdSubCB(const gestelt_interfaces::msg::AllUAVCommand::UniquePtr msg)
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
	if (last_nav_heartbeat_ - this->get_clock()->now().seconds() > nav_state_timeout_)
	{
		// Set to landing
		logger_->logErrorThrottle("Navigator heartbeat timeout!", 1.0);
		sendUAVCommandEvent(gestelt_interfaces::msg::AllUAVCommand::COMMAND_LAND, 0, 0);
	}

	// Check all states
	if (UAV::is_in_state<Unconnected>())
	{
	}
	else if (UAV::is_in_state<Idle>())
	{
	}
	else if (UAV::is_in_state<Landing>())
	{
		mavros_handler_->execLand();
	}
	else if (UAV::is_in_state<TakingOff>())
	{
		if (!mavros_handler_->isUAVReady())
		{
			mavros_handler_->toggleOffboardMode(true);
		}
	}
	else if (UAV::is_in_state<Hovering>())
	{
	}
	else if (UAV::is_in_state<Mission>())
	{
	}
	else if (UAV::is_in_state<EmergencyStop>())
	{
	}
	else
	{
	}
}

void TrajectoryServer::pubCtrlTimerCB()
{
	static double take_off_hover_T = 1.0 / 5.0; // Take off and landing period
	static double estop_T = 1.0 / 50.0;			// EStop period
	cmd_yaw_yawrate_(1) = NAN;					// set yaw rate to 0

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
	}
	else if (UAV::is_in_state<TakingOff>())
	{
		cmd_pos_enu_(2) = fsm_list::fsmtype::current_state_ptr->getTakeoffHeight();
		// Adjust for ground height
		Eigen::Vector3d pos_enu_corr = Eigen::Vector3d(cmd_pos_enu_(0), cmd_pos_enu_(1), cmd_pos_enu_(2) + ground_height_); // Adjust for ground height

		if (this->get_clock()->now().seconds() - last_cmd_pub_t_ > take_off_hover_T)
		{
			mavros_handler_->execTakeOff(pos_enu_corr, cmd_yaw_yawrate_);

			last_cmd_pub_t_ = this->get_clock()->now().seconds();
		}
	}
	else if (UAV::is_in_state<Hovering>())
	{
		// Adjust for ground height
		Eigen::Vector3d pos_enu_corr = Eigen::Vector3d(cmd_pos_enu_(0), cmd_pos_enu_(1), cmd_pos_enu_(2) + ground_height_); // Adjust for ground height

		if (this->get_clock()->now().seconds() - last_cmd_pub_t_ > take_off_hover_T)
		{
			mavros_handler_->execHover(pos_enu_corr, cmd_yaw_yawrate_);

			last_cmd_pub_t_ = this->get_clock()->now().seconds();
		}
	}
	else if (UAV::is_in_state<Mission>())
	{
		Eigen::Vector3d pos_enu_corr = Eigen::Vector3d(cmd_pos_enu_(0), cmd_pos_enu_(1), cmd_pos_enu_(2) + ground_height_); // Adjust for ground height

		if (!geofence_->withinLimits(pos_enu_corr))
		{
			// skip if command is not within limits
			logger_->logError("OUT OF GEOFENCE: SKIP COMMAND EXECUTION!");
			return;
		}

		mavros_handler_->execMission(pos_enu_corr, cmd_vel_enu_, cmd_acc_enu_, cmd_yaw_yawrate_);

	}
	else if (UAV::is_in_state<EmergencyStop>())
	{
		if (this->get_clock()->now().seconds() - last_cmd_pub_t_ > estop_T)
		{


			last_cmd_pub_t_ = this->get_clock()->now().seconds();
		}
	}
	else
	{
	}
}

void TrajectoryServer::SMTickTimerCB()
{
	// Check all states
	if (UAV::is_in_state<Unconnected>())
	{
		logger_->logWarnThrottle("[Unconnected]", 1.0);

		if (mavros_handler_->isConnected())
		{
			sendEvent(Idle_E());
		}
	}
	else if (UAV::is_in_state<Idle>())
	{
		logger_->logInfoThrottle("[Idle]", 1.0);

		ground_height_ = cur_pos_(2); // Set ground height to last z value
		cmd_pos_enu_ = cur_pos_corr_; // Set last commanded position as current position
		cmd_yaw_yawrate_(0) = frame_transforms::utils::quaternion::quaternion_get_yaw(cur_ori_);

		mavros_handler_->setGroundheight(ground_height_);
	}
	else if (UAV::is_in_state<Landing>())
	{
		logger_->logInfoThrottle("[Landing]", 1.0);

		if (mavros_handler_->isLanded())
		{
			sendEvent(Idle_E());
		}
	}
	else if (UAV::is_in_state<TakingOff>())
	{
		logger_->logInfoThrottle("[TakingOff]", 1.0);
		double take_off_h = fsm_list::fsmtype::current_state_ptr->getTakeoffHeight();

		if (mavros_handler_->isTakenOff(take_off_h))
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
		logger_->logInfoThrottle("[Mission]", 2.0);
	}
	else if (UAV::is_in_state<EmergencyStop>())
	{
		// logger_->logInfoThrottle("[EmergencyStop]", 1.0);
	}
	else
	{
		// logger_->logInfoThrottle("Undefined UAV state", 1.0);
	}
}

void TrajectoryServer::pubStateTimerCB()
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
	}
	else if (UAV::is_in_state<TakingOff>())
	{
		uav_state.state = gestelt_interfaces::msg::UAVState::TAKINGOFF;
	}
	else if (UAV::is_in_state<Hovering>())
	{
		uav_state.state = gestelt_interfaces::msg::UAVState::HOVERING;
	}
	else if (UAV::is_in_state<Mission>())
	{
		uav_state.state = gestelt_interfaces::msg::UAVState::MISSION;
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

/****************** */
/* PUBLISHER METHODS */
/****************** */


/****************** */
/* SERVICE CALLBACKS*/
/****************** */

void TrajectoryServer::uavCmdSrvCB(const std::shared_ptr<gestelt_interfaces::srv::UAVCommand::Request> request,
								   std::shared_ptr<gestelt_interfaces::srv::UAVCommand::Response> response)
{
	// request->header
	logger_->logInfo(strFmt("Incoming request\n Command: %d"
							" Mode: %d"
							" Value: %f",
							request->command, request->mode, request->value));

	// Checkn if value and mode is within bounds for specific commands
	switch (request->command)
	{
	case gestelt_interfaces::srv::UAVCommand::Request::COMMAND_TAKEOFF:
		if (request->value < 0.5 || request->value > 3.0)
		{
			response->success = false;
			response->state = (int)getUAVState();
			response->state_name = getUAVStateString();
			logger_->logError("Value for COMMAND_TAKEOFF should be between 0.5 and 3.0, inclusive");

			return;
		}

		break;
	case gestelt_interfaces::srv::UAVCommand::Request::COMMAND_START_MISSION:

		if (request->mode > 4)
		{
			response->success = false;
			response->state = (int)getUAVState();
			response->state_name = getUAVStateString();
			logger_->logError("Value for COMMAND_START_MISSION should be between 0 and 4 inclusive");

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
			if ((srv_rcv_t - this->get_clock()->now().seconds()) >= takeoff_timeout)
			{
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
			if ((srv_rcv_t - this->get_clock()->now().seconds()) >= land_timeout)
			{
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
			if ((srv_rcv_t - this->get_clock()->now().seconds()) >= start_mission_timeout)
			{
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
			if ((srv_rcv_t - this->get_clock()->now().seconds()) >= stop_mission_timeout)
			{
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
			if ((srv_rcv_t - this->get_clock()->now().seconds()) >= estop_timeout)
			{
				timeout = true;
				break;
			}
			rclcpp::sleep_for(std::chrono::milliseconds(25));
		}

		break;
	default:
		break;
	}

	response->state = (int)getUAVState();
	response->state_name = getUAVStateString();

	if (timeout)
	{
		response->success = false;

		logger_->logInfo(strFmt("Timeout!!: State[%d] State_name[%s] Success [%d]",
								response->state, response->state_name.c_str(), response->success));
	}
	else
	{
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
