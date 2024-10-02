#include <fake_drone/fake_drone.hpp>

FakeDrone::FakeDrone()
: Node("fake_drone")
{
	bl_broadcaster_tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
	min_jerk_opt_ = std::make_unique<minco::MinJerkOpt>();

	sim_update_cb_group_ = this->create_callback_group(
		rclcpp::CallbackGroupType::Reentrant);

	std::string param_ns = "fake_drone";
	
	this->declare_parameter(param_ns+".drone_id", -1);

	this->declare_parameter(param_ns+".state_update_frequency", -1.0);
	this->declare_parameter(param_ns+".tf_broadcast_frequency", -1.0);

	this->declare_parameter(param_ns+".global_frame", "world");
	this->declare_parameter(param_ns+".local_map_frame",  "local_map_origin");
	this->declare_parameter(param_ns+".uav_frame","base_link");

	this->declare_parameter(param_ns+".t_unit", 0.1);

	this->declare_parameter(param_ns+".init.x", 0.0);
	this->declare_parameter(param_ns+".init.y", 0.0);
	this->declare_parameter(param_ns+".init.z", 0.0);

	this->declare_parameter(param_ns+".fe_stride", -1);

	this->declare_parameter(param_ns+".t_step", -1.0);

	drone_id_ = this->get_parameter(param_ns+".drone_id").as_int();

	double state_update_freq = this->get_parameter(param_ns+".state_update_frequency").as_double();
	double tf_broadcast_freq = this->get_parameter(param_ns+".tf_broadcast_frequency").as_double();

	global_frame_ = this->get_parameter(param_ns+".global_frame").as_string();
	local_map_frame_ = this->get_parameter(param_ns+".local_map_frame").as_string();
	uav_frame_ = this->get_parameter(param_ns+".uav_frame").as_string();

	t_unit_ = this->get_parameter(param_ns+".t_unit").as_double();

	Eigen::Vector3d init_pos;
	init_pos(0) = this->get_parameter(param_ns+".init.x").as_double();
	init_pos(1) = this->get_parameter(param_ns+".init.y").as_double();
	init_pos(2) = this->get_parameter(param_ns+".init.z").as_double();

	fe_stride_ = this->get_parameter(param_ns+".fe_stride").as_int();

	t_step_ = this->get_parameter(param_ns+".t_step").as_double();

	// Set initial position
	odom_msg_.pose.pose.position.x = init_pos(0);
	odom_msg_.pose.pose.position.y = init_pos(1);
	odom_msg_.pose.pose.position.z = init_pos(2);
	odom_msg_.pose.pose.orientation.x = odom_msg_.pose.pose.orientation.y = odom_msg_.pose.pose.orientation.z = 0.0;
	odom_msg_.pose.pose.orientation.w = 1.0;

	pose_msg_.pose = odom_msg_.pose.pose;
	odom_msg_.header.frame_id = pose_msg_.header.frame_id = global_frame_;
		
		/* Subscribers and publishers*/
	fe_plan_sub_ = this->create_subscription<gestelt_interfaces::msg::SpaceTimePath>(
		"fe_plan", rclcpp::SystemDefaultsQoS(), std::bind(&FakeDrone::frontEndPlanCB, this, _1));

	odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
	pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose", 10);
	
	minco_traj_viz_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("minco_traj_viz", 10);

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

	for (size_t i = 0; i < msg->plan.size(); i += fe_stride_)
	{	
		// // Add control point
		// points.push_back(fe_plan_msg_.plan[i].position.x);
		// points.push_back(fe_plan_msg_.plan[i].position.y);
		// points.push_back(fe_plan_msg_.plan[i].position.z);

		fe_space_time_path_.push_back(Eigen::Vector4d{
			msg->plan[i].position.x,
			msg->plan[i].position.y, 
			msg->plan[i].position.z,
			(double) msg->plan_time[i],
		});
	}

	// spline_ = std::make_shared<tinyspline::BSpline>(tinyspline::BSpline::interpolateCubicNatural(points, 3));

	// Generate minimum jerk from space time path
	genMinJerkTraj(fe_space_time_path_);
	fe_minco_traj_ = min_jerk_opt_->getTraj(msg->t_plan_start);
	Eigen::MatrixXd cstr_pts = min_jerk_opt_->getConstraintPts(5);
	viz_helper::VizHelper::pubExecTraj(cstr_pts, minco_traj_viz_pub_, global_frame_);

	new_plan_rcv_ = true;
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

	if (new_plan_rcv_){
    	setStateFromTraj(fe_minco_traj_);
	}
	
	{
		std::lock_guard<std::mutex> state_mutex_guard(state_mutex_);
		odom_pub_->publish(odom_msg_);
		pose_pub_->publish(pose_msg_);
	}

}

// void FakeDrone::setStateFromTraj(	const std::vector<Eigen::Vector4d>& space_time_path, 
// 									const double& plan_start_t)
// {

// 	auto t_now = this->get_clock()->now();
// 	double e_t_start = t_now.seconds() - plan_start_t;			 	// [s] Elapsed time since plan start
// 	int e_t_start_st = (int) tToSpaceTimeUnits(e_t_start); 	// [space-time units] Elapsed time since plan start

// 	if (e_t_start_st < 0){
// 		// trajectory starts in the future
// 		std::cout << "trajectory starts in the future" << std::endl;
// 		return;
// 	}

// 	if (e_t_start_st >= space_time_path.back()(3)){
// 		// Time exceeded end of trajectory. Trajectory has finished executing in the past
// 		std::cout << "Trajectory has finished executing in the past" << std::endl;
// 		new_plan_rcv_ = false;
// 		return;
// 	}

// 	// Iterate through plan and choose state at current time
// 	int cur_idx = 0;
// 	bool exceed_prev = false;

// 	for (size_t i = 0; i < space_time_path.size(); i++){	// for each point in the path
// 		cur_idx = i;
// 		if (exceed_prev){
// 			// if elapsed time exceeded previous timestamp and <= current timestamp, end loop
// 			if (e_t_start_st < space_time_path[i](3))
// 			{
// 				// std::cout << "e_t_start_st(" << e_t_start_st << ") <= " << space_time_path[i](3) << std::endl;
// 				break;
// 			}
// 		}
// 		else {
// 			exceed_prev = e_t_start_st >= space_time_path[i](3) ? true : false;
// 		}
// 	}

// 	double x = space_time_path[cur_idx](0);
// 	double y = space_time_path[cur_idx](1);
// 	double z = space_time_path[cur_idx](2);

// 	// alpha: Arc length parameterization of spline. Formed by time ratio
// 	// double alpha = ((double)e_t_start_st) / (fe_plan_msg.plan_time.back());

// 	// std::vector<tinyspline::real> result = spline_->eval(alpha).result();
// 	// double x = result[0];
// 	// double y = result[1];
// 	// double z = result[2];

//   	std::lock_guard<std::mutex> state_mutex_guard(state_mutex_);

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


void FakeDrone::setStateFromTraj(const std::shared_ptr<minco::Trajectory>& traj)
{
	if (traj == nullptr )
	{
		return;
	}

	auto t_now = this->get_clock()->now();
	double e_t_start = t_now.seconds() - traj->getGlobalStartTime();			 	// [s] Elapsed time since plan start

	if (e_t_start < 0.0){
		// trajectory starts in the future
		std::cout << "trajectory starts in the future" << std::endl;
		return;
	}
	else if (e_t_start >= traj->getTotalDuration()){
		// Time exceeded end of trajectory. Trajectory has finished executing in the past
		std::cout << "Trajectory has finished executing in the past" << std::endl;
		new_plan_rcv_ = false;
		return;
	}

	Eigen::Vector3d pos = traj->getPos(e_t_start);
	Eigen::Vector3d vel = traj->getVel(e_t_start);
	Eigen::Vector3d acc = traj->getAcc(e_t_start);
	Eigen::Vector3d jer = traj->getJer(e_t_start);

	std::pair<double, double> yaw_yawdot(0, 0);
	Eigen::Quaterniond quat{0.0, 0.0, 0.0, 1};

	/*** calculate yaw ***/
	yaw_yawdot = calculate_yaw(traj, e_t_start, t_now.seconds() - t_last_traj_samp_);
	quat = RPYToQuaternion(0.0, 0.0, yaw_yawdot.first);

	t_last_traj_samp_ = t_now.seconds();
	{
		std::lock_guard<std::mutex> state_mutex_guard(state_mutex_);

		odom_msg_.header.stamp = pose_msg_.header.stamp = t_now;

		odom_msg_.pose.pose.position.x = pos(0);
		odom_msg_.pose.pose.position.y = pos(1);
		odom_msg_.pose.pose.position.z = pos(2);

		odom_msg_.pose.pose.orientation.x = quat.x();
		odom_msg_.pose.pose.orientation.y = quat.y();
		odom_msg_.pose.pose.orientation.z = quat.z();
		odom_msg_.pose.pose.orientation.w = quat.w();

		pose_msg_.pose = odom_msg_.pose.pose;
	}
}

void FakeDrone::genMinJerkTraj(const std::vector<Eigen::Vector4d>& space_time_path)
{
	Eigen::Matrix3d start_PVA, goal_PVA;

	start_PVA.block<3,1>(0, 0) =  space_time_path[0].head<3>();
	start_PVA.block<3,1>(0, 1) = Eigen::Vector3d{0.0, 0.0, 0.0};
	start_PVA.block<3,1>(0, 2) = Eigen::Vector3d{0.0, 0.0, 0.0};

	goal_PVA.block<3,1>(0, 0) =  space_time_path.back().head<3>();
	goal_PVA.block<3,1>(0, 1) = Eigen::Vector3d{0.0, 0.0, 0.0};
	goal_PVA.block<3,1>(0, 2) = Eigen::Vector3d{0.0, 0.0, 0.0};

	Eigen::MatrixXd inner_pts(3, space_time_path.size()-2);
	Eigen::VectorXd seg_durations(space_time_path.size()-1);

	for (size_t i = 1, j = 0; i < space_time_path.size()-1; i++, j++){
		inner_pts.col(j) = space_time_path[i].head<3>();
	}

	for (size_t i = 1, j = 0; i < space_time_path.size(); i++, j++){
		seg_durations(j) = double(space_time_path[i](3) - space_time_path[j](3)) * t_unit_;
	}

	min_jerk_opt_->generate(start_PVA, goal_PVA, inner_pts, seg_durations);

	return;
}



std::pair<double, double> FakeDrone::calculate_yaw(const std::shared_ptr<minco::Trajectory>& traj, 
													const double& t_cur, const double& dt)
{
	std::pair<double, double> yaw_yawdot(0, 0);

	// get direction vector
	Eigen::Vector3d dir = t_cur + t_step_ <= traj->getTotalDuration()
								? traj->getPos(t_cur + t_step_) - traj->getPos(t_cur)
								: traj->getPos(traj->getTotalDuration()) - traj->getPos(t_cur);

	double yaw_temp = dir.norm() > 0.1
							? atan2(dir(1), dir(0))
							: prev_yaw_;

	double yawdot = 0;
	double d_yaw = yaw_temp - prev_yaw_;
	if (d_yaw >= M_PI)
	{
		d_yaw -= 2 * M_PI;
	}
	if (d_yaw <= -M_PI)
	{
		d_yaw += 2 * M_PI;
	}
	
	// Set maximum values for yaw_dot and yaw_ddot
	const double YDM = d_yaw >= 0 ? YAW_DOT_MAX_PER_SEC : -YAW_DOT_MAX_PER_SEC;
	const double YDDM = d_yaw >= 0 ? YAW_DOT_DOT_MAX_PER_SEC : -YAW_DOT_DOT_MAX_PER_SEC;
	double d_yaw_max;

	if (fabs(prev_yaw_dot_ + dt * YDDM) <= fabs(YDM)) // Within yaw_dot limits
	{
		// yawdot = prev_yaw_dot_ + dt * YDDM;
		d_yaw_max = (prev_yaw_dot_ * dt) + (0.5 * YDDM * dt * dt);
	}
	else // exceed yaw_dot limits
	{
		// yawdot = YDM;
		double t1 = (YDM - prev_yaw_dot_) / YDDM;
		d_yaw_max = ((dt - t1) + dt) * (YDM - prev_yaw_dot_) / 2.0;
	}

	if (fabs(d_yaw) > fabs(d_yaw_max))
	{
		d_yaw = d_yaw_max;
	}
	yawdot = d_yaw / dt;

	double yaw = prev_yaw_ + d_yaw;
	if (yaw > M_PI)
		yaw -= 2 * M_PI;
	if (yaw < -M_PI)
		yaw += 2 * M_PI;
	yaw_yawdot.first = yaw;
	yaw_yawdot.second = yawdot;

	prev_yaw_ = yaw_yawdot.first;
	prev_yaw_dot_ = yaw_yawdot.second;

	yaw_yawdot.second = yaw_temp;

	return yaw_yawdot;
}


