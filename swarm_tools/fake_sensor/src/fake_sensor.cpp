#include <fake_sensor/fake_sensor.hpp>

FakeSensor::FakeSensor()
: Node("fake_sensor")
{
	reentrant_cb_grp_ = this->create_callback_group(
		rclcpp::CallbackGroupType::Reentrant);

	tf_buffer_ =
		std::make_unique<tf2_ros::Buffer>(this->get_clock());
	tf_listener_ =
		std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
	fake_map_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
	sensor_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

	this->declare_parameter("drone_id", -1);
	this->declare_parameter("tf.listen_to_tf", false);
	this->declare_parameter("tf.listen_freq", -1.0);

	// Frame parameters
	this->declare_parameter("global_frame", "world");
	this->declare_parameter("map_frame", "map");
	this->declare_parameter("sensor_frame", "camera_link");

	// Pcd map file parameters
	this->declare_parameter("pcd_map.filepath", "");

	// Fake sensor parameters
	this->declare_parameter("fake_laser.sensor_range", -1.0);
	this->declare_parameter("fake_laser.sensor_refresh_frequency", -1.0);
	this->declare_parameter("fake_laser.resolution", -1.0);
	this->declare_parameter("fake_laser.horizontal.laser_line_num", -1);
	this->declare_parameter("fake_laser.horizontal.laser_range_dgr", -1.0);
	this->declare_parameter("fake_laser.vertical.laser_line_num", -1);
	this->declare_parameter("fake_laser.vertical.laser_range_dgr", -1.0);

	// Downsampler parameters
	this->declare_parameter("pcd_voxel_filter.enable", true);
	this->declare_parameter("pcd_voxel_filter.voxel_size", -1.0);

	/* Get Parameters */
	drone_id_ = this->get_parameter("drone_id").as_int();
	listen_to_tf_ = this->get_parameter("tf.listen_to_tf").as_bool();

	// Frame parameters
	global_frame_ = this->get_parameter("global_frame").as_string();
	map_frame_ = this->get_parameter("map_frame").as_string();
	sensor_frame_ = this->get_parameter("sensor_frame").as_string();

	// Pcd map file parameters
	std::string map_filepath = this->get_parameter("pcd_map.filepath").as_string();

	// Fake sensor parameters
	double sensor_max_range = this->get_parameter("fake_laser.sensor_range").as_double();
	double sensor_refresh_freq = this->get_parameter("fake_laser.sensor_refresh_frequency").as_double();
	double resolution = this->get_parameter("fake_laser.resolution").as_double();
	int hrz_laser_line_num = this->get_parameter("fake_laser.horizontal.laser_line_num").as_int();
	int vtc_laser_line_num = this->get_parameter("fake_laser.vertical.laser_line_num").as_int();
	double hrz_laser_range_dgr = this->get_parameter("fake_laser.horizontal.laser_range_dgr").as_double();
	double vtc_laser_range_dgr = this->get_parameter("fake_laser.vertical.laser_range_dgr").as_double();

	voxel_filter_enable_ = this->get_parameter("pcd_voxel_filter.enable").as_bool();
	voxel_size_ = this->get_parameter("pcd_voxel_filter.voxel_size").as_double();

	/* Subscribers */
	odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
		"odom", rclcpp::SensorDataQoS(), std::bind(&FakeSensor::odomSubCB, this, _1));

	auto reentrant_sub_opt = rclcpp::SubscriptionOptions();
	reentrant_sub_opt.callback_group = reentrant_cb_grp_;

	/* Publishers */
    sensor_pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud", 10);

	if (voxel_filter_enable_){
		vox_grid_ = std::make_shared<pcl::VoxelGrid<pcl::PointXYZ>>();
	}

	// Passthrough filter to filter out noise in close proximity of the agent
	pass_fil_x_ = std::make_shared<pcl::PassThrough<pcl::PointXYZ>>();
	pass_fil_y_ = std::make_shared<pcl::PassThrough<pcl::PointXYZ>>();
	pass_fil_z_ = std::make_shared<pcl::PassThrough<pcl::PointXYZ>>();

	pass_fil_x_->setFilterFieldName ("x");
	pass_fil_y_->setFilterFieldName ("y");
	pass_fil_z_->setFilterFieldName ("z");

	pass_fil_x_->setFilterLimits (-0.025, 0.025);
	pass_fil_y_->setFilterLimits (-0.025, 0.025);
	pass_fil_z_->setFilterLimits (-0.025, 0.025);

	pass_fil_x_->setNegative (true);
	pass_fil_y_->setNegative (true);
	pass_fil_z_->setNegative (true);

	// Load point cloud map from file
	if (pcl::io::loadPCDFile(map_filepath, *fake_map_cloud_) == -1) 
	{
		RCLCPP_ERROR(this->get_logger(), "Invalid PCD filepath input: %s\n", map_filepath.c_str());
		rclcpp::shutdown();
	}
	RCLCPP_INFO(this->get_logger(), "Loaded PCD input file from %s\n", map_filepath.c_str());

	// // Get 'map' to map_frame fixed TF, used for transforming PCD map to map_frame
	// try {
	// 	RCLCPP_INFO(this->get_logger(), "Getting transform from map to global frame");
	// 	auto tf_res = tf_buffer_->lookupTransform(
	// 		map_frame_, global_frame_, tf2::TimePointZero,
	// 		tf2_ros::fromRclcpp(rclcpp::Duration::from_seconds(5.0)));

	// 	// Create transformation matrix from global to sensor frame
	// 	gbl_to_map_tf_mat_.block<3, 3>(0, 0) = Eigen::Quaterniond(
	// 		tf_res.transform.rotation.w,
	// 		tf_res.transform.rotation.x,
	// 		tf_res.transform.rotation.y,
	// 		tf_res.transform.rotation.z).toRotationMatrix();

	// 	gbl_to_map_tf_mat_.block<3,1>(0,3) = Eigen::Vector3d(
	// 		tf_res.transform.translation.x,
	// 		tf_res.transform.translation.y,
	// 		tf_res.transform.translation.z);
	// } 
	// catch (const tf2::TransformException & ex) {
	// 	RCLCPP_ERROR(
	// 		this->get_logger(), "Could not get transform from global frame(%s) to map_frame_(%s): %s. SHUTTING DOWN.",
	// 		global_frame_.c_str(), map_frame_.c_str(), ex.what());
	// 	rclcpp::shutdown();
	// 	return;
	// }

	// // Transform point cloud map from global_frame_ frame to map_frame_
	// pcl::transformPointCloud (*fake_map_cloud_, *fake_map_cloud_, gbl_to_map_tf_mat_);
  	fake_map_cloud_->header.frame_id = global_frame_;

	// Set up sensor renderer
	sensor_renderer_.set_parameters(
		resolution,
		sensor_max_range,
		*fake_map_cloud_,
		vtc_laser_range_dgr,
		hrz_laser_range_dgr,
		vtc_laser_line_num,
		hrz_laser_line_num);

	sensor_update_timer_ = this->create_wall_timer((1.0/sensor_refresh_freq) *1000ms, 
							std::bind(&FakeSensor::sensorUpdateTimerCB, this),
							reentrant_cb_grp_);

	RCLCPP_INFO(this->get_logger(), "Initialized");
}

FakeSensor::~FakeSensor()
{}

/* Subscriber Callbacks*/

void FakeSensor::odomSubCB(const nav_msgs::msg::Odometry::UniquePtr msg)
{
	{
		std::lock_guard<std::mutex> odom_mtx_guard(odom_mutex_);
		cur_odom_ = *msg;
	}
	pose_rcv_ = true;
}

/* Timer Callbacks*/

void FakeSensor::sensorUpdateTimerCB()
{
	if (listen_to_tf_){

		// Get 'map' to map_frame fixed TF, used for transforming PCD map to map_frame
		try {
			auto tf_res = tf_buffer_->lookupTransform(
				sensor_frame_, global_frame_, tf2::TimePointZero,
				tf2_ros::fromRclcpp(rclcpp::Duration::from_seconds(5.0)));

			// Create transformation matrix from global to sensor frame
			sensor_to_gbl_tf_mat_.block<3, 3>(0, 0) = Eigen::Quaterniond(
				tf_res.transform.rotation.w,
				tf_res.transform.rotation.x,
				tf_res.transform.rotation.y,
				tf_res.transform.rotation.z).toRotationMatrix();

			sensor_to_gbl_tf_mat_.block<3,1>(0,3) = Eigen::Vector3d(
				tf_res.transform.translation.x,
				tf_res.transform.translation.y,
				tf_res.transform.translation.z);
		} 
		catch (const tf2::TransformException & ex) {
			RCLCPP_ERROR(this->get_logger(), 
				"Could not get transform from sensor_frame (%s) to global_frame(%s): %s. ",
				sensor_frame_.c_str(), global_frame_.c_str(), ex.what());
			return;
		}

		Eigen::Matrix3d sensor_rot = sensor_to_gbl_tf_mat_.block<3, 3>(0, 0);
		Eigen::Vector3d sensor_pos = sensor_to_gbl_tf_mat_.block<3,1>(0,3);

		if (sensor_rot.array().isNaN().any() || sensor_pos.array().isNaN().any()){
			RCLCPP_ERROR(this->get_logger(), "NaN value in sensor orientation and position");
			return;
		}

		// Generate point cloud in global frame using position and orientation of sensor
		sensor_renderer_.render_sensed_points(
			sensor_pos,  // sensor position in global frame
			sensor_rot,  // sensor orientation in global frame
			*sensor_cloud_);

		gbl_to_sensor_tf_mat_ = sensor_to_gbl_tf_mat_.inverse();

		// sensor_cloud_ is in [map_frame], we transform it to [sensor_frame]
		pcl::transformPointCloud (*sensor_cloud_, *sensor_cloud_, gbl_to_sensor_tf_mat_);
	}
	else {
		if (!pose_rcv_)
		{
			return;
		}

		{
			std::lock_guard<std::mutex> odom_mtx_guard(odom_mutex_);

			// Create map to sensor TF matrix
			map_to_sensor_tf_mat_.block<3, 3>(0, 0) = Eigen::Quaterniond(
				cur_odom_.pose.pose.orientation.w,
				cur_odom_.pose.pose.orientation.x,
				cur_odom_.pose.pose.orientation.y,
				cur_odom_.pose.pose.orientation.z).toRotationMatrix();
			map_to_sensor_tf_mat_.block<3,1>(0, 3) = Eigen::Vector3d(
				cur_odom_.pose.pose.position.x,
				cur_odom_.pose.pose.position.y,
				cur_odom_.pose.pose.position.z);

		}

		if (map_to_sensor_tf_mat_.block<3, 3>(0, 0).array().isNaN().any() 
			|| map_to_sensor_tf_mat_.block<3,1>(0,3).array().isNaN().any()){
			RCLCPP_ERROR(this->get_logger(), "NaN value in sensor orientation and position");
			return;
		}

		// Generate point cloud from position and orientation of sensor
		sensor_renderer_.render_sensed_points(
			map_to_sensor_tf_mat_.block<3, 1>(0, 3), // sensor position in map frame
			map_to_sensor_tf_mat_.block<3, 3>(0, 0), // sensor orientation in map frame
			*sensor_cloud_);

		sensor_to_map_tf_mat_ = map_to_sensor_tf_mat_.inverse();

		// sensor_cloud_ is in [map_frame], we transform it to [sensor_frame]
		pcl::transformPointCloud (*sensor_cloud_, *sensor_cloud_, sensor_to_map_tf_mat_);
	}

	// Downsample cloud
	if (voxel_filter_enable_){
		vox_grid_->setInputCloud(sensor_cloud_);
		vox_grid_->setLeafSize(voxel_size_, voxel_size_, voxel_size_);
		vox_grid_->filter(*sensor_cloud_);
	}

	// Filter away points very close to the agent
	pass_fil_x_-> setInputCloud (sensor_cloud_);
	pass_fil_x_-> filter (*sensor_cloud_);
	pass_fil_y_-> setInputCloud (sensor_cloud_);
	pass_fil_y_-> filter (*sensor_cloud_);
	pass_fil_z_-> setInputCloud (sensor_cloud_);
	pass_fil_z_-> filter (*sensor_cloud_);

	// Publish cloud 
	sensor_msgs::msg::PointCloud2 sensor_cloud_msg;
	// printf("sensor_cloud_ size(%ld), width(%ld), height(%ld)",
	// 		sensor_cloud_->size(), sensor_cloud_->width, sensor_cloud_->height); 
	if (sensor_cloud_->points.empty()){
		RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Publish empty /cloud");
		
		sensor_cloud_msg.header.frame_id = sensor_frame_;
		sensor_cloud_msg.header.stamp = this->get_clock()->now();

		sensor_pc_pub_->publish(sensor_cloud_msg);
		return;
	}

	sensor_cloud_->width = sensor_cloud_->points.size();
	sensor_cloud_->height = 1;

	pcl::toROSMsg(*sensor_cloud_, sensor_cloud_msg);

	sensor_cloud_msg.header.frame_id = sensor_frame_;
	sensor_cloud_msg.header.stamp = this->get_clock()->now();

	sensor_pc_pub_->publish(sensor_cloud_msg);

}
