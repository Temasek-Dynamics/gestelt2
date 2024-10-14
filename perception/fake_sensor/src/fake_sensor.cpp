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
	this->declare_parameter("map_frame", "map");
	this->declare_parameter("local_map_frame", "local_map_frame");
	this->declare_parameter("sensor_frame", "camera_frame");

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
	double tf_listen_freq = this->get_parameter("tf.listen_freq").as_double();

	// Frame parameters
	map_frame_ = this->get_parameter("map_frame").as_string();
	local_map_frame_ = this->get_parameter("local_map_frame").as_string();
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

	/* Publishers */
    sensor_pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud", 10);

	if (voxel_filter_enable_){
		vox_grid_ = std::make_shared<pcl::VoxelGrid<pcl::PointXYZ>>();
	}

	// Load point cloud map from file
	if (pcl::io::loadPCDFile(map_filepath, *fake_map_cloud_) == -1) 
	{
		RCLCPP_ERROR(this->get_logger(), "drone%d has invalid PCD filepath input: %s\n", 
			drone_id_, map_filepath.c_str());
		rclcpp::shutdown();
	}
	RCLCPP_INFO(this->get_logger(), "drone%d loaded PCD input file from %s\n", 
		drone_id_, map_filepath.c_str());
	
	// Get 'world' to map_frame fixed TF, used for transforming PCD map to map_frame
	try {
		auto tf_world_to_map = tf_buffer_->lookupTransform(
			map_frame_, "world",
			tf2::TimePointZero,
			tf2_ros::fromRclcpp(rclcpp::Duration::from_seconds(5.0)));

		// Create transformation matrix from global to sensor frame
		world_to_map_tf_mat_.block<3, 3>(0, 0) = Eigen::Quaterniond(
			tf_world_to_map.transform.rotation.w,
			tf_world_to_map.transform.rotation.x,
			tf_world_to_map.transform.rotation.y,
			tf_world_to_map.transform.rotation.z).toRotationMatrix();
		world_to_map_tf_mat_(0, 3) = tf_world_to_map.transform.translation.x;
		world_to_map_tf_mat_(1, 3) = tf_world_to_map.transform.translation.y;
		world_to_map_tf_mat_(2, 3) = tf_world_to_map.transform.translation.z;
		world_to_map_tf_mat_(3, 3) = 1.0;

	} catch (const tf2::TransformException & ex) {
			RCLCPP_ERROR(
				this->get_logger(), "Could not get transform from world_frame('world') to map_frame_(%s): %s",
				map_frame_.c_str(), ex.what());
		rclcpp::shutdown();
		return;
	}

	// Transform point cloud map from 'world' frame to map_frame_
	pcl::transformPointCloud (*fake_map_cloud_, *fake_map_cloud_, world_to_map_tf_mat_);
  	fake_map_cloud_->header.frame_id = map_frame_;

	// Set up sensor renderer
	sensor_renderer_.set_parameters(
		resolution,
		sensor_max_range,
		*fake_map_cloud_,
		vtc_laser_range_dgr,
		hrz_laser_range_dgr,
		vtc_laser_line_num,
		hrz_laser_line_num);

	/**
	 * Timers that handles drone state at each time frame 
	*/
	if (listen_to_tf_){
		tf_listen_timer_ = this->create_wall_timer((1.0/tf_listen_freq) *1000ms, 
								std::bind(&FakeSensor::TFListenCB, this),
								reentrant_cb_grp_);
	}

	sensor_update_timer_ = this->create_wall_timer((1.0/sensor_refresh_freq) *1000ms, 
							std::bind(&FakeSensor::sensorUpdateTimerCB, this),
							reentrant_cb_grp_);
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

void FakeSensor::TFListenCB()
{
	try
    {
		{
			std::lock_guard<std::mutex> sensor_tf_mtx_guard(sensor_tf_mutex_);
			// Get transform from sensor_frame to global frame
			sensor_to_map_tf_ = tf_buffer_->lookupTransform(
				map_frame_, sensor_frame_, 
				tf2::TimePointZero,
				tf2_ros::fromRclcpp(rclcpp::Duration::from_seconds(0.2)));
		}
		cam_tf_valid_ = true;
    }
    catch (const tf2::TransformException &ex)
    {
		RCLCPP_ERROR(
			this->get_logger(), "Could not get transform from map_frame(%s) to map_frame_(%s): %s",
			sensor_frame_.c_str(), map_frame_.c_str(), ex.what());

		cam_tf_valid_ = false;
		return;
    }
}

void FakeSensor::sensorUpdateTimerCB()
{
	if (!cam_tf_valid_ && !pose_rcv_){
		return;
	}

	if (listen_to_tf_){
		geometry_msgs::msg::TransformStamped sensor_to_map_tf;

		{
			std::lock_guard<std::mutex> sensor_tf_mtx_guard(sensor_tf_mutex_);
			sensor_to_map_tf = sensor_to_map_tf_;
		}

		Eigen::Matrix3d sensor_ori_in_map_frame(Eigen::Quaterniond(sensor_to_map_tf.transform.rotation.w,
													sensor_to_map_tf.transform.rotation.x,
													sensor_to_map_tf.transform.rotation.y,
													sensor_to_map_tf.transform.rotation.z));
		Eigen::Vector3d sensor_pos_in_map_frame(
			sensor_to_map_tf.transform.translation.x, 
			sensor_to_map_tf.transform.translation.y, 
			sensor_to_map_tf.transform.translation.z);

		if (sensor_ori_in_map_frame.array().isNaN().any() 
			|| sensor_pos_in_map_frame.array().isNaN().any()){
			RCLCPP_ERROR(this->get_logger(), "NaN value in sensor orientation and position");
			return;
		}

		// Generate point cloud from position and orientation of sensor
		sensor_renderer_.render_sensed_points(sensor_pos_in_map_frame, sensor_ori_in_map_frame, *sensor_cloud_);
		
		// Create transformation matrix from global to sensor frame
		sensor_to_map_tf_mat_.block<3, 3>(0, 0) = Eigen::Quaterniond(
			sensor_to_map_tf.transform.rotation.w,
			sensor_to_map_tf.transform.rotation.x,
			sensor_to_map_tf.transform.rotation.y,
			sensor_to_map_tf.transform.rotation.z).toRotationMatrix();
		sensor_to_map_tf_mat_(0, 3) = sensor_to_map_tf.transform.translation.x;
		sensor_to_map_tf_mat_(1, 3) = sensor_to_map_tf.transform.translation.y;
		sensor_to_map_tf_mat_(2, 3) = sensor_to_map_tf.transform.translation.z;
		sensor_to_map_tf_mat_(3, 3) = 1.0;

		map_to_sensor_tf_mat_ = sensor_to_map_tf_mat_.inverse();
	}
	else {
		{
			std::lock_guard<std::mutex> odom_mtx_guard(odom_mutex_);

			// Create map to sensor TF matrix
			map_to_sensor_tf_mat_.block<3, 3>(0, 0) = Eigen::Quaterniond(
				cur_odom_.pose.pose.orientation.w,
				cur_odom_.pose.pose.orientation.x,
				cur_odom_.pose.pose.orientation.y,
				cur_odom_.pose.pose.orientation.z).toRotationMatrix();
			map_to_sensor_tf_mat_(0, 3) = cur_odom_.pose.pose.position.x;
			map_to_sensor_tf_mat_(1, 3) = cur_odom_.pose.pose.position.y;
			map_to_sensor_tf_mat_(2, 3) = cur_odom_.pose.pose.position.z;
			map_to_sensor_tf_mat_(3, 3) = 1.0;
		}

		if (map_to_sensor_tf_mat_.block<3, 3>(0, 0).array().isNaN().any() 
			|| map_to_sensor_tf_mat_.block<3,1>(0,3).array().isNaN().any()){
			RCLCPP_ERROR(this->get_logger(), "NaN value in sensor orientation and position");
			return;
		}

		sensor_to_map_tf_mat_ = map_to_sensor_tf_mat_.inverse();

		// Generate point cloud from position and orientation of sensor
		sensor_renderer_.render_sensed_points(
			sensor_to_map_tf_mat_.block<3,1>(0,3), 	// sensor position in map frame
			sensor_to_map_tf_mat_.block<3, 3>(0, 0), // sensor orientation in map frame
			*sensor_cloud_);
		
		// map_to_sensor_tf_mat_ = map_to_sensor_tf_mat_.inverse().eval();
	}

	// sensor_cloud_ is in [map_frame], we transform it to [sensor_frame]
	pcl::transformPointCloud (*sensor_cloud_, *sensor_cloud_, map_to_sensor_tf_mat_);

	// Downsample cloud
	if (voxel_filter_enable_){
		vox_grid_->setInputCloud(sensor_cloud_);
		vox_grid_->setLeafSize(voxel_size_, voxel_size_, voxel_size_);
		vox_grid_->filter(*sensor_cloud_);
	}

	// Publish cloud 
	sensor_msgs::msg::PointCloud2 sensor_cloud_msg;
	pcl::toROSMsg(*sensor_cloud_, sensor_cloud_msg);

	sensor_cloud_msg.header.frame_id = sensor_frame_;
	sensor_cloud_msg.header.stamp = this->get_clock()->now();

	sensor_pc_pub_->publish(sensor_cloud_msg);

}
