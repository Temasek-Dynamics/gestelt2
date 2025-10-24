#include <fake_sensor/fake_sensor.hpp>

#include <tf2_eigen/tf2_eigen.hpp>

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
	cloud_sensor_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
	cloud_global_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

	this->declare_parameter("drone_id", -1);

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
	
	// RCLCPP_INFO(this->get_logger(), "Obtaining drone id \n");
	drone_id_ = this->get_parameter("drone_id").as_int();
	// RCLCPP_INFO(this->get_logger(), "Obtained drone id \n");
	
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
	// odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
	// 	"odom", rclcpp::SensorDataQoS(), std::bind(&FakeSensor::odomSubCB, this, _1));

	auto reentrant_sub_opt = rclcpp::SubscriptionOptions();
	reentrant_sub_opt.callback_group = reentrant_cb_grp_;

	// RCLCPP_INFO(this->get_logger(), "Drone id in int %d\n", drone_id_);
	/* Publishers */
    cloud_sensor_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
		"d" + std::to_string(drone_id_) + "/cloud", rclcpp::SensorDataQoS());
	// RCLCPP_INFO(this->get_logger(), "Drone id in str %s\n", std::to_string(drone_id_).c_str());

	cloud_global_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
		"d" + std::to_string(drone_id_) + "/cloud_global", rclcpp::SensorDataQoS());

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

/* Timer Callbacks*/

void FakeSensor::sensorUpdateTimerCB()
{
	// We receive sensor data in world frame, and we transform it to sensor frame

	Eigen::Matrix3d sensor_ori; // Sensor orientation in global frame
	Eigen::Vector3d sensor_pos; // Sensor position in global frame

	// Get global frame to sensor_frame TF
	try {
		// Get transform from global_frame to sensor_frame
		auto tf_res = tf_buffer_->lookupTransform(
			global_frame_, sensor_frame_,
			tf2::TimePointZero,
			tf2_ros::fromRclcpp(rclcpp::Duration::from_seconds(1.0)));

		global_to_sensor_mat_ = tf2::transformToEigen(
			tf_res.transform).matrix().cast<double>();

		// // Create transformation matrix from global to sensor frame
		// global_to_sensor_mat_.block<3, 3>(0, 0) = Eigen::Quaterniond(
		// 	tf_res.transform.rotation.w,
		// 	tf_res.transform.rotation.x,
		// 	tf_res.transform.rotation.y,
		// 	tf_res.transform.rotation.z).toRotationMatrix();

		// global_to_sensor_mat_.block<3,1>(0, 3) = Eigen::Vector3d(
		// 	tf_res.transform.translation.x,
		// 	tf_res.transform.translation.y,
		// 	tf_res.transform.translation.z);

		sensor_pos = global_to_sensor_mat_.block<3, 1>(0, 3);
		sensor_ori = global_to_sensor_mat_.block<3, 3>(0, 0);
	} 
	catch (const tf2::TransformException & ex) {
		RCLCPP_ERROR(this->get_logger(), 
			"Could not get transform from global_frame (%s) to sensor_frame(%s): %s. ",
			global_frame_.c_str(), sensor_frame_.c_str(), ex.what());
		return;
	}

	if (sensor_ori.array().isNaN().any() || sensor_pos.array().isNaN().any()){
		RCLCPP_ERROR(this->get_logger(), "NaN value in sensor orientation and position");
		return;
	}

	// Output point clouds are given in global frame 
	sensor_renderer_.render_sensed_points(
		sensor_pos,  // sensor position in global frame
		sensor_ori,  // sensor orientation in global frame
		*cloud_global_);


	// RCLCPP_INFO(this->get_logger(),
	// 	"sensor_pos (%0.2f, %0.2f, %0.2f), size(%ld), width(%ld), height(%ld)",
	// 	sensor_pos(0), sensor_pos(1), sensor_pos(2), 
	// 	cloud_global_->points.size(), cloud_global_->width, cloud_global_->height);

	// Downsample cloud
	if (voxel_filter_enable_){
		vox_grid_->setInputCloud(cloud_global_);
		vox_grid_->setLeafSize(voxel_size_, voxel_size_, voxel_size_);
		vox_grid_->filter(*cloud_global_);
	}

	// publish sensor pcd in global frame
	sensor_msgs::msg::PointCloud2 cloud_global_msg;
	if (!cloud_global_->points.empty()){
		pcl::toROSMsg(*cloud_global_, cloud_global_msg);
	}
	cloud_global_msg.header.frame_id = global_frame_;
	cloud_global_msg.header.stamp = this->get_clock()->now();

	cloud_global_pub_->publish(cloud_global_msg);

	sensor_to_global_mat_ = global_to_sensor_mat_.inverse();

	// RCLCPP_INFO(this->get_logger(),
	// 	"sensor_to_global transformation (%0.2f, %0.2f, %0.2f)",
	// 	sensor_to_global_mat_.block<3, 1>(0, 3)(0), 
	// 	sensor_to_global_mat_.block<3, 1>(0, 3)(1), 
	// 	sensor_to_global_mat_.block<3, 1>(0, 3)(2));

	pcl::transformPointCloud (*cloud_global_, *cloud_sensor_, sensor_to_global_mat_);

	// Filter away points very close to the agent
	pass_fil_x_-> setInputCloud (cloud_sensor_);
	pass_fil_x_-> filter (*cloud_sensor_);
	pass_fil_y_-> setInputCloud (cloud_sensor_);
	pass_fil_y_-> filter (*cloud_sensor_);
	pass_fil_z_-> setInputCloud (cloud_sensor_);
	pass_fil_z_-> filter (*cloud_sensor_);

	sensor_msgs::msg::PointCloud2 cloud_sensor_msg;
	if (!cloud_sensor_->points.empty()){
		pcl::toROSMsg(*cloud_sensor_, cloud_sensor_msg);
	}
	else {
		RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
								"Publishing empty sensor cloud");
	}
	cloud_sensor_msg.header.frame_id = sensor_frame_;
	cloud_sensor_msg.header.stamp = this->get_clock()->now();

	cloud_sensor_pub_->publish(cloud_sensor_msg);

}
