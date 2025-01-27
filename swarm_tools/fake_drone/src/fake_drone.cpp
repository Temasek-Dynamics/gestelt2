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
	tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

	sim_update_cb_group_ = this->create_callback_group(
		rclcpp::CallbackGroupType::Reentrant);

    traj_sp_cb_group_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
	
	/* Parameters */
	std::string param_ns = "fake_drone";
	this->declare_parameter(param_ns+".drone_id", -1);
	this->declare_parameter(param_ns+".map_frame", "map");
	this->declare_parameter(param_ns+".base_link_frame", "base_link");

	this->declare_parameter(param_ns+".state_update_frequency", -1.0);
	this->declare_parameter(param_ns+".odom_update_frequency", -1.0);

	this->declare_parameter(param_ns+".init_x", 0.0);
	this->declare_parameter(param_ns+".init_y", 0.0);
	this->declare_parameter(param_ns+".init_yaw", 0.0);

	drone_id_ = this->get_parameter(param_ns+".drone_id").as_int();
	std::string map_frame = this->get_parameter(param_ns+".map_frame").as_string();
	std::string base_link_frame = this->get_parameter(param_ns+".base_link_frame").as_string();

	double state_update_frequency = this->get_parameter(param_ns+".state_update_frequency").as_double();
	double odom_update_frequency = this->get_parameter(param_ns+".odom_update_frequency").as_double();

	Eigen::Vector3d init_pos;
	init_pos(0) = this->get_parameter(param_ns+".init_x").as_double();
	init_pos(1) = this->get_parameter(param_ns+".init_y").as_double();
	init_pos(2) = 0.5; 
	double init_yaw = this->get_parameter(param_ns+".init_yaw").as_double();
	Eigen::Quaterniond quat = RPYToQuaternion(0.0, 0.0, init_yaw);

	/* Assign initial values */
	odom_msg_ = std::make_unique<nav_msgs::msg::Odometry>();
	odom_msg_->header.frame_id = map_frame;
	odom_msg_->pose.pose.position.x = init_pos(0);
	odom_msg_->pose.pose.position.y = init_pos(1);
	odom_msg_->pose.pose.position.z = init_pos(2);
	odom_msg_->pose.pose.orientation.x = quat.x();
	odom_msg_->pose.pose.orientation.y = quat.y();
	odom_msg_->pose.pose.orientation.z = quat.z();
	odom_msg_->pose.pose.orientation.w = quat.w();

	vehicle_state_msg_ = std::make_unique<mavros_msgs::msg::State>();
	vehicle_state_msg_->armed = false;
	vehicle_state_msg_->connected = true;
	vehicle_state_msg_->mode = "AUTO.LOITER";

	map_to_bl_tf_.header.frame_id = map_frame; 
	map_to_bl_tf_.child_frame_id = base_link_frame; 

	/* Services */
	cmd_arming_srv_ =
		this->create_service<mavros_msgs::srv::CommandBool>(
			"mavros/cmd/arming", std::bind(&FakeDrone::cmdArmingSrvCB, this, _1, _2)) ;
	set_mode_srv_ =
		this->create_service<mavros_msgs::srv::SetMode>(
			"mavros/set_mode", std::bind(&FakeDrone::setModeSrvCB, this, _1, _2));

	/* Subscribers */
	auto traj_sp_opt = rclcpp::SubscriptionOptions();
	traj_sp_opt.callback_group = traj_sp_cb_group_;

	auto state_qos = rclcpp::QoS(10).transient_local();

	trajectory_setpoint_sub_ = this->create_subscription<mavros_msgs::msg::PositionTarget>(
		"mavros/setpoint_raw/local", rclcpp::SystemDefaultsQoS(), 
		std::bind(&FakeDrone::trajectorySetpointSubCB, this, _1), traj_sp_opt);

	/* Publishers */
	odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
		"mavros/local_position/odom", rclcpp::SensorDataQoS());

	vehicle_state_pub_ = this->create_publisher<mavros_msgs::msg::State>(
		"mavros/state", state_qos);

	/**
	 * Timer for drone state update  
	*/
	odom_update_timer_ = this->create_wall_timer((1.0/odom_update_frequency) *1000ms, 
                                            std::bind(&FakeDrone::odomUpdateTimerCB, this), 
                                            sim_update_cb_group_);

	state_update_timer_ = this->create_wall_timer((1.0/state_update_frequency) *1000ms, 
                                            std::bind(&FakeDrone::stateUpdateTimerCB, this), 
                                            sim_update_cb_group_);

	printf("[fake_drone] drone%d created with start_pose [%.2lf %.2lf %.2lf]! \n", 
		drone_id_, init_pos(0), init_pos(1), init_pos(2));
}

FakeDrone::~FakeDrone()
{
}

/* Subscriber Callbacks*/

void FakeDrone::trajectorySetpointSubCB(const mavros_msgs::msg::PositionTarget::UniquePtr &msg)
{
	if (vehicle_state_msg_->mode == "AUTO.LOITER"){
		// TODO: land vehicle
	}
	else if (vehicle_state_msg_->mode == "OFFBOARD" && vehicle_state_msg_->armed)
	{
		std::lock_guard<std::mutex> state_mutex_guard(state_mutex_);

		map_to_bl_tf_.header.stamp = this->get_clock()->now();
		odom_msg_->header.stamp = this->get_clock()->now();

		if (!std::isnan(msg->position.x) && !std::isnan(msg->position.y) && !std::isnan(msg->position.z))
		{
			odom_msg_->pose.pose.position.x = msg->position.x;
			odom_msg_->pose.pose.position.y = msg->position.y;
			odom_msg_->pose.pose.position.z = msg->position.z;

			map_to_bl_tf_.transform.translation.x = msg->position.x;
			map_to_bl_tf_.transform.translation.y = msg->position.y;
			map_to_bl_tf_.transform.translation.z = msg->position.z;

		}
		if (!std::isnan(msg->velocity.x) && !std::isnan(msg->velocity.y) && !std::isnan(msg->velocity.z))
		{
			odom_msg_->twist.twist.linear.x = msg->velocity.x;
			odom_msg_->twist.twist.linear.y = msg->velocity.y;
			odom_msg_->twist.twist.linear.z = msg->velocity.z;
		}
		Eigen::Quaterniond quat = RPYToQuaternion(0.0, 0.0, (double) msg->yaw);
		odom_msg_->pose.pose.orientation.x = quat.x();
		odom_msg_->pose.pose.orientation.y = quat.y();
		odom_msg_->pose.pose.orientation.z = quat.z();
		odom_msg_->pose.pose.orientation.w = quat.w();

		map_to_bl_tf_.transform.rotation.x = quat.x();
		map_to_bl_tf_.transform.rotation.y = quat.y();
		map_to_bl_tf_.transform.rotation.z = quat.z();
		map_to_bl_tf_.transform.rotation.w = quat.w();

	}

}


void FakeDrone::cmdArmingSrvCB(const std::shared_ptr<mavros_msgs::srv::CommandBool::Request> req,
					std::shared_ptr<mavros_msgs::srv::CommandBool::Response> resp)
{
	if (req->value){
		printf("[fake_drone] drone%d armed! \n", drone_id_);
	}
	else {
		printf("[fake_drone] drone%d disarmed! \n", drone_id_);
	}
	vehicle_state_msg_->armed = req->value;

	resp->success = true;
}

void FakeDrone::setModeSrvCB(const std::shared_ptr<mavros_msgs::srv::SetMode::Request> req,
					std::shared_ptr<mavros_msgs::srv::SetMode::Response> resp)
{
	vehicle_state_msg_->mode = req->custom_mode;

	printf("[fake_drone] drone%d switched to mode %s! \n", 
			drone_id_, req->custom_mode.c_str());

	resp->mode_sent = true;
}

/* Timer Callbacks*/

void FakeDrone::odomUpdateTimerCB()
{
	std::lock_guard<std::mutex> state_mutex_guard(state_mutex_);
	odom_pub_->publish(*odom_msg_);
	tf_broadcaster_->sendTransform(map_to_bl_tf_);
}

void FakeDrone::stateUpdateTimerCB()
{
	vehicle_state_pub_->publish(*vehicle_state_msg_);
}
