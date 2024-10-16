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
	bl_broadcaster_tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

	sim_update_cb_group_ = this->create_callback_group(
		rclcpp::CallbackGroupType::Reentrant);

	std::string param_ns = "fake_drone";
	
	this->declare_parameter(param_ns+".drone_id", -1);

	this->declare_parameter(param_ns+".state_update_frequency", -1.0);
	this->declare_parameter(param_ns+".tf_broadcast_frequency", -1.0);

	this->declare_parameter(param_ns+".map_frame", "world");
	this->declare_parameter(param_ns+".local_map_frame",  "local_map_origin");
	this->declare_parameter(param_ns+".uav_frame","base_link");

	this->declare_parameter(param_ns+".t_unit", 0.1);

	this->declare_parameter(param_ns+".init.x", 0.0);
	this->declare_parameter(param_ns+".init.y", 0.0);
	this->declare_parameter(param_ns+".init.z", 0.0);

	this->declare_parameter(param_ns+".t_step", -1.0);

	drone_id_ = this->get_parameter(param_ns+".drone_id").as_int();

	double state_update_freq = this->get_parameter(param_ns+".state_update_frequency").as_double();
	double tf_broadcast_freq = this->get_parameter(param_ns+".tf_broadcast_frequency").as_double();

	map_frame_ = this->get_parameter(param_ns+".map_frame").as_string();
	local_map_frame_ = this->get_parameter(param_ns+".local_map_frame").as_string();
	uav_frame_ = this->get_parameter(param_ns+".uav_frame").as_string();

	t_unit_ = this->get_parameter(param_ns+".t_unit").as_double();

	Eigen::Vector3d init_pos;
	init_pos(0) = this->get_parameter(param_ns+".init.x").as_double();
	init_pos(1) = this->get_parameter(param_ns+".init.y").as_double();
	init_pos(2) = this->get_parameter(param_ns+".init.z").as_double();

	t_step_ = this->get_parameter(param_ns+".t_step").as_double();

	// Set initial position
	odom_msg_.pose.pose.position.x = init_pos(0);
	odom_msg_.pose.pose.position.y = init_pos(1);
	odom_msg_.pose.pose.position.z = init_pos(2);
	odom_msg_.pose.pose.orientation.x = odom_msg_.pose.pose.orientation.y = odom_msg_.pose.pose.orientation.z = 0.0;
	odom_msg_.pose.pose.orientation.w = 1.0;

	pose_msg_.pose = odom_msg_.pose.pose;
	odom_msg_.header.frame_id = pose_msg_.header.frame_id = map_frame_;
		
	/* Subscribers and publishers*/
	fe_plan_sub_ = this->create_subscription<gestelt_interfaces::msg::SpaceTimePath>(
		"fe_plan", rclcpp::SystemDefaultsQoS(), std::bind(&FakeDrone::frontEndPlanCB, this, _1));

	odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
	pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose", 10);

	/**
	 * Timer for drone state update  
	*/

	tf_update_timer_ = this->create_wall_timer((1.0/tf_broadcast_freq) *1000ms, 
                                            std::bind(&FakeDrone::tfUpdateTimerCB, this), 
                                            sim_update_cb_group_);

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

void FakeDrone::frontEndPlanCB(const gestelt_interfaces::msg::SpaceTimePath::UniquePtr &msg)
{	
	// Generate spline from plan 
	// std::vector<tinyspline::real> points;

	fe_space_time_path_.clear();

	// for (size_t i = 0; i < msg->plan.size(); i += 1)
	// {	
	// 	// // Add control point
	// 	// points.push_back(fe_plan_msg_.plan[i].position.x);
	// 	// points.push_back(fe_plan_msg_.plan[i].position.y);
	// 	// points.push_back(fe_plan_msg_.plan[i].position.z);

	// 	fe_space_time_path_.push_back(Eigen::Vector4d{
	// 		msg->plan[i].position.x,
	// 		msg->plan[i].position.y, 
	// 		msg->plan[i].position.z,
	// 		(double) msg->plan_time[i],
	// 	});
	// }

	// spline_ = std::make_shared<tinyspline::BSpline>(tinyspline::BSpline::interpolateCubicNatural(points, 3));
}

/* Timer Callbacks*/

void FakeDrone::tfUpdateTimerCB()
{
  geometry_msgs::msg::TransformStamped bl_to_map_tf;

  bl_to_map_tf.header.stamp = this->get_clock()->now();
  bl_to_map_tf.header.frame_id = local_map_frame_; 
  bl_to_map_tf.child_frame_id = uav_frame_; 

  bl_to_map_tf.transform.translation.x = pose_msg_.pose.position.x;
  bl_to_map_tf.transform.translation.y = pose_msg_.pose.position.y;
  bl_to_map_tf.transform.translation.z = pose_msg_.pose.position.z;

  bl_to_map_tf.transform.rotation = pose_msg_.pose.orientation;
  
  bl_broadcaster_tf_->sendTransform(bl_to_map_tf);
}

void FakeDrone::stateUpdateTimerCB()
{
  odom_msg_.header.stamp = pose_msg_.header.stamp = this->get_clock()->now();

	
	{
		std::lock_guard<std::mutex> state_mutex_guard(state_mutex_);
		odom_pub_->publish(odom_msg_);
		pose_pub_->publish(pose_msg_);
	}

}

// void FakeDrone::setStateFromTraj(	const std::vector<Eigen::Vector4d>& space_time_path, 
// 									const double& plan_start_t)
// {

// 	odom_msg_.header.stamp = pose_msg_.header.stamp = t_now;

// 	// odom_msg_.pose.pose = fe_plan_msg.plan[cur_plan_idx];
// 	odom_msg_.pose.pose.position.x = x;
// 	odom_msg_.pose.pose.position.y = y;
// 	odom_msg_.pose.pose.position.z = z;

// 	odom_msg_.pose.pose.orientation.x = 0.0;
// 	odom_msg_.pose.pose.orientation.y = 0.0;
// 	odom_msg_.pose.pose.orientation.z = 0.0;
// 	odom_msg_.pose.pose.orientation.w = 1.0;

// 	// pose_msg_.header.stamp = t_now;

// 	// pose_msg_.pose = fe_plan_msg.plan[cur_plan_idx];
// 	pose_msg_.pose = odom_msg_.pose.pose;
// }

