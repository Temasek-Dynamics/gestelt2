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

#include "voxel_map/voxel_map.hpp"

namespace voxel_map
{

/** Initialization methods */

VoxelMap::VoxelMap(rclcpp::Node::SharedPtr node, const Eigen::Vector3d& map_origin) 
: node_(node)
{
  md_.world_to_map.block<3,1>(0,3) = map_origin;

	// Create callback groups
  reentrant_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::Reentrant);

	logger_ = std::make_shared<logger_wrapper::LoggerWrapper>(node_->get_logger(), node_->get_clock());

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);

  initParams();

  reset(mp_.resolution_);

  initPubSubTimer();

  tm_update_local_map_.updateID(drone_id_);
  tm_bonxai_insert_.updateID(drone_id_);
  tm_slice_map_.updateID(drone_id_);

  if (dbg_input_entire_map_){
    md_.cam_to_map = md_.world_to_map;

    logger_->logInfo(strFmt("DEBUG: INPUT ENTIRE MAP. Waiting for point cloud on topic %s",  entire_pcd_map_topic_.c_str()));

		sensor_msgs::msg::PointCloud2 pcd_map_msg;

    //https://github.com/ros2/rclcpp/issues/1953
    auto got_pcd = rclcpp::wait_for_message(pcd_map_msg, node_, entire_pcd_map_topic_, 10s);

    if (!got_pcd){
			logger_->logError(strFmt("No point cloud topic %s received. Shutting down.",  entire_pcd_map_topic_.c_str()));
			rclcpp::shutdown();
    }
    pcd2MsgToMap(pcd_map_msg);
  }

  // logger_->logInfo(strFmt("Map origin (%f, %f, %f)",  
  //   mp_.global_map_origin_(0), mp_.global_map_origin_(1), mp_.global_map_origin_(2)));
  // logger_->logInfo(strFmt("Global map size (%f, %f, %f)",  
  //   mp_.global_map_size_(0), mp_.global_map_size_(1), mp_.global_map_size_(2)));
  // // logger_->logInfo(strFmt("Local map size (%f, %f, %f)",  
  // //   mp_.local_map_size_(0), mp_.local_map_size_(1), mp_.local_map_size_(2)));
  // logger_->logInfo(strFmt("Resolution %f m, Inflation %f m",  
    // mp_.resolution_, mp_.inflation_));
	logger_->logInfo("Initialized voxel_map");
}

VoxelMap::~VoxelMap(){
}

void VoxelMap::initPubSubTimer()
{
  /* Initialize Subscribers */
  if (!dbg_input_entire_map_){
    cloud_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(
      node_, "cloud", rmw_qos_profile_sensor_data);
    odom_sub_ = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>(
      node_, "odom", rmw_qos_profile_sensor_data);

    sync_cloud_odom_ = std::make_shared<message_filters::Synchronizer<SyncPolicyCloudOdom>>(
        SyncPolicyCloudOdom(5), *cloud_sub_, *odom_sub_);
    sync_cloud_odom_->registerCallback(&VoxelMap::cloudOdomCB, this);
    sync_cloud_odom_->setAgePenalty(0.50);
  }

  /* Initialize Publishers */
	occ_map_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("occ_map", 10);
	// slice_map_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("slice", rclcpp::SensorDataQoS());
	local_map_bounds_pub_ = node_->create_publisher<geometry_msgs::msg::PolygonStamped>("local_map/bounds", 10);

  /* Initialize ROS Timers */
	viz_map_timer_ = node_->create_wall_timer((1.0/viz_occ_map_freq_) *1000ms, 
    std::bind(&VoxelMap::vizMapTimerCB, this), reentrant_group_);
	update_local_map_timer_ = node_->create_wall_timer((1.0/update_local_map_freq_) *1000ms, 
    std::bind(&VoxelMap::updateLocalMapTimerCB, this), reentrant_group_);
	pub_lcl_map_tf_timer_ = node_->create_wall_timer((1.0/100.0) *1000ms, 
    std::bind(&VoxelMap::pubLocalMapTFTimerCB, this), reentrant_group_);

  if (check_collisions_){ // True if we want to publish collision visualizations between the agent and static obstacles
    // Publisher for collision visualizations
	  // collision_viz_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("collision_viz");
	  check_collisions_timer_ = node_->create_wall_timer((1.0/check_col_freq_) *1000ms, std::bind(&VoxelMap::checkCollisionsTimerCB, this));
  }

}

void VoxelMap::initParams()
{
  std::string param_ns = "voxel_map";

  /* Map parameters */
  node_->declare_parameter(param_ns+".verbose_print", false);

  node_->declare_parameter(param_ns+".debug_input_entire_map", false);
  node_->declare_parameter(param_ns+".entire_pcd_map_topic", "/fake_map");

  node_->declare_parameter(param_ns+".map_slicing.min_height_cm", -1);
  node_->declare_parameter(param_ns+".map_slicing.max_height_cm",  -1);
  node_->declare_parameter(param_ns+".map_slicing.z_separation_cm",  -1);

  node_->declare_parameter(param_ns+".global_map.size_x", -1.0);
  node_->declare_parameter(param_ns+".global_map.size_y", -1.0);
  node_->declare_parameter(param_ns+".global_map.size_z", -1.0);

  node_->declare_parameter(param_ns+".local_map.size_x", -1.0);
  node_->declare_parameter(param_ns+".local_map.size_y", -1.0);
  node_->declare_parameter(param_ns+".local_map.size_z", -1.0);
  node_->declare_parameter(param_ns+".local_map.update_local_map_frequency",  -1.0);
  node_->declare_parameter(param_ns+".local_map.viz_map_frequency",  -1.0);

  node_->declare_parameter(param_ns+".occ_map.resolution", -1.0);
  node_->declare_parameter(param_ns+".occ_map.inflation", -1.0);
  node_->declare_parameter(param_ns+".occ_map.max_range", -1.0);
  node_->declare_parameter(param_ns+".occ_map.ground_height", 0.0);

  /* Camera extrinsic parameters  */
  node_->declare_parameter(param_ns+".camera_to_body.roll",  0.0);
  node_->declare_parameter(param_ns+".camera_to_body.pitch", 0.0);
  node_->declare_parameter(param_ns+".camera_to_body.yaw",   0.0);
  node_->declare_parameter(param_ns+".camera_to_body.t_x", 0.0);
  node_->declare_parameter(param_ns+".camera_to_body.t_y", 0.0);
  node_->declare_parameter(param_ns+".camera_to_body.t_z", 0.0);

  /* Collision checking*/
  node_->declare_parameter(param_ns+".collision_check.enable",  false);
  node_->declare_parameter(param_ns+".collision_check.warn_radius",  -1.0); 
  node_->declare_parameter(param_ns+".collision_check.fatal_radius",  -1.0); 
  node_->declare_parameter(param_ns+".collision_check.check_collision_frequency",  -1.0); 

	/**
	 * Get params
	 */

  drone_id_ = node_->get_parameter("drone_id").as_int();

  /* Map parameters */
  verbose_print_ = node_->get_parameter(param_ns+".verbose_print").as_bool();

  dbg_input_entire_map_ = node_->get_parameter(param_ns+".debug_input_entire_map").as_bool();
  entire_pcd_map_topic_ = node_->get_parameter(param_ns+".entire_pcd_map_topic").as_string();

  bool_map_3d_.min_height_cm = node_->get_parameter(param_ns+".map_slicing.min_height_cm").as_int();
  bool_map_3d_.max_height_cm = node_->get_parameter(param_ns+".map_slicing.max_height_cm").as_int();
  bool_map_3d_.z_separation_cm = node_->get_parameter(param_ns+".map_slicing.z_separation_cm").as_int();

  auto cmToM = [&](const int& val_cm){ 
    /* Convert from units of centimeters to meters*/
    return ((double) val_cm)/100.0;  
  };

  bool_map_3d_.z_separation_m = cmToM(bool_map_3d_.z_separation_cm); 
  bool_map_3d_.min_height_m = cmToM(bool_map_3d_.min_height_cm);  
  bool_map_3d_.max_height_m = cmToM(bool_map_3d_.max_height_cm);   

  mp_.global_map_size_(0) = node_->get_parameter(param_ns+".global_map.size_x").as_double();
  mp_.global_map_size_(1) = node_->get_parameter(param_ns+".global_map.size_y").as_double();
  mp_.global_map_size_(2) = node_->get_parameter(param_ns+".global_map.size_z").as_double();

  mp_.local_map_size_(0) = node_->get_parameter(param_ns+".local_map.size_x").as_double();
  mp_.local_map_size_(1) = node_->get_parameter(param_ns+".local_map.size_y").as_double();
  mp_.local_map_size_(2) = node_->get_parameter(param_ns+".local_map.size_z").as_double();
  update_local_map_freq_ = node_->get_parameter(param_ns+".local_map.update_local_map_frequency").as_double();
  viz_occ_map_freq_ = node_->get_parameter(param_ns+".local_map.viz_map_frequency").as_double();

  mp_.resolution_ = node_->get_parameter(param_ns+".occ_map.resolution").as_double();
  mp_.inflation_ = node_->get_parameter(param_ns+".occ_map.inflation").as_double();
  mp_.max_range = node_->get_parameter(param_ns+".occ_map.max_range").as_double();
  mp_.ground_height_ = node_->get_parameter(param_ns+".occ_map.ground_height").as_double();
  mp_.inf_num_voxels_ = std::ceil(mp_.inflation_/mp_.resolution_);

  /* Frame IDs */
  mp_.map_frame = node_->get_parameter("map_frame").as_string();
  mp_.local_map_frame = node_->get_parameter("local_map_frame").as_string();

  /* Camera extrinsic parameters  */
  md_.cam_to_body_rpy_deg(0) = node_->get_parameter(param_ns+".camera_to_body.roll").as_double();
  md_.cam_to_body_rpy_deg(1) = node_->get_parameter(param_ns+".camera_to_body.pitch").as_double();
  md_.cam_to_body_rpy_deg(2) = node_->get_parameter(param_ns+".camera_to_body.yaw").as_double();
  md_.cam_to_body.col(3)(0) = node_->get_parameter(param_ns+".camera_to_body.t_x").as_double();
  md_.cam_to_body.col(3)(1) = node_->get_parameter(param_ns+".camera_to_body.t_y").as_double();
  md_.cam_to_body.col(3)(2) = node_->get_parameter(param_ns+".camera_to_body.t_z").as_double();

  /* Collision checking*/
  check_collisions_ = node_->get_parameter(param_ns+".collision_check.enable").as_bool();
  col_warn_radius_ = node_->get_parameter(param_ns+".collision_check.warn_radius").as_double();
  col_fatal_radius_ = node_->get_parameter(param_ns+".collision_check.fatal_radius").as_double();
  check_col_freq_ = node_->get_parameter(param_ns+".collision_check.check_collision_frequency").as_double();

}

void VoxelMap::reset(const double& resolution){

  // Set up Bonxai data structure
  bonxai_map_ = std::make_unique<BonxaiT>(resolution);

  local_occ_map_pts_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  local_global_occ_map_pts_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcd_in_map_frame_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  md_.last_sensor_msg_time = node_->get_clock()->now().seconds();

  // Camera to body transform
  md_.cam_to_body.block<3, 3>(0, 0) =  (Eigen::AngleAxisd((M_PI/180.0) * md_.cam_to_body_rpy_deg(2), Eigen::Vector3d::UnitZ())
                                      * Eigen::AngleAxisd((M_PI/180.0) * md_.cam_to_body_rpy_deg(1), Eigen::Vector3d::UnitY())
                                      * Eigen::AngleAxisd((M_PI/180.0) * md_.cam_to_body_rpy_deg(0), Eigen::Vector3d::UnitX())).toRotationMatrix();

  // Global map origin is FIXED at a corner of the global map i.e. (-W/2, -L/2, 0)
  mp_.global_map_origin_ = Eigen::Vector3d(
    -mp_.global_map_size_(0) / 2.0, 
    -mp_.global_map_size_(1) / 2.0, 
    mp_.ground_height_);

  /* LOCAL MAP COORDINATES ARE NOT FIXED! */
  // Local map min is at a corner of the local map i.e. (uav_pos_x - local_sz_x /2, uav_pos_y - local_sz_y /2, 0), relative to the current position of the robot
  mp_.local_map_origin_ = Eigen::Vector3d(
    -mp_.local_map_size_(0) / 2.0, 
    -mp_.local_map_size_(1) / 2.0, 
    mp_.ground_height_);

  // Local map max is at the other corner of the local map i.e. (uav_pos_x + local_sz_x /2, uav_pos_y + local_sz_y/2, 0), relative to the current position of the robot
  mp_.local_map_max_ = Eigen::Vector3d(
    mp_.local_map_size_(0) / 2.0, 
    mp_.local_map_size_(1) / 2.0, 
    mp_.ground_height_ + mp_.local_map_size_(2));

  mp_.global_map_num_voxels_ = (mp_.global_map_size_.cwiseProduct(Eigen::Vector3d::Constant(1/mp_.resolution_))).cast<int>();
  mp_.local_map_num_voxels_ = (mp_.local_map_size_.cwiseProduct(Eigen::Vector3d::Constant(1/mp_.resolution_))).cast<int>();

}

/* Core methods */

void VoxelMap::setCamToMapPose(const geometry_msgs::msg::Pose &pose)
{
  Eigen::Quaterniond map_to_body_q = Eigen::Quaterniond(pose.orientation.w,
                                                pose.orientation.x,
                                                pose.orientation.y,
                                                pose.orientation.z);
  // UAV body to map frame
  md_.body_to_map.block<3, 3>(0, 0) = map_to_body_q.toRotationMatrix();
  md_.body_to_map.block<3,1>(0,3) << pose.position.x, pose.position.y, pose.position.z;

  // Converts camera to UAV origin frame
  md_.cam_to_map = md_.cam_to_body * md_.body_to_map;

  md_.has_pose_ = true;
}

void VoxelMap::pcd2MsgToMap(const sensor_msgs::msg::PointCloud2 &msg)
{
  /* Bonxai*/

  // Input Point cloud is assumed to be in frame id of the sensor
  if (!isPoseValid()){
    return;
  }

  if (msg.data.empty()){
    // logger_->logWarnThrottle("Empty point cloud received!", 1.0);
    return;
  }

  // Get point cloud from ROS message
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcd = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::fromROSMsg(msg, *pcd);

  // Add point cloud to bonxai_maps_
  pcdToVoxelMap(pcd);
}

void VoxelMap::pcdToVoxelMap(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pcd_in_cam_frame)
{
  // Transform point cloud from camera frame to uav origin frame (the global reference frame)
  pcl::transformPointCloud(*pcd_in_cam_frame, *pcd_in_map_frame_, md_.cam_to_map);
  pcd_in_map_frame_->header.frame_id = mp_.map_frame;

  // Getting the Translation from the sensor to the Global Reference Frame
  const pcl::PointXYZ sensor_origin(
    md_.cam_to_map(0, 3), md_.cam_to_map(1, 3), md_.cam_to_map(2, 3));

  tm_bonxai_insert_.start();
  bonxai_map_->insertPointCloud(pcd_in_map_frame_->points, sensor_origin, mp_.max_range);
  tm_bonxai_insert_.stop(false);
}

void VoxelMap::updateLocalMap(){

  if (!isPoseValid()){
    return;
  }

  // OCCUPIED VALUE: 100
  // FREE VALUE: 0
  // UNKNOWN VALUE: -1

  // Update local map origin 
  mp_.local_map_origin_ = Eigen::Vector3d(
    md_.body_to_map.block<3,1>(0,3)(0) - (mp_.local_map_size_(0) / 2.0), 
    md_.body_to_map.block<3,1>(0,3)(1) - (mp_.local_map_size_(1) / 2.0), 
    mp_.ground_height_);
  // Update local map max position based on current UAV position
  mp_.local_map_max_ = Eigen::Vector3d(
    md_.body_to_map.block<3,1>(0,3)(0) + (mp_.local_map_size_(0) / 2.0), 
    md_.body_to_map.block<3,1>(0,3)(1) + (mp_.local_map_size_(1) / 2.0), 
    mp_.ground_height_ + mp_.local_map_size_(2) );
  
  // Get all occupied coordinates 
  std::vector<Bonxai::CoordT> occ_coords;
  bonxai_map_->getOccupiedVoxels(occ_coords);
  if (occ_coords.size() <= 1){ // Empty map
    return;
  }

  {
    std::lock_guard<std::mutex> lcl_occ_map_guard(lcl_occ_map_mtx_);

    // Clear existing local map
    local_occ_map_pts_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    local_global_occ_map_pts_.reset(new pcl::PointCloud<pcl::PointXYZ>());

    for (auto& coord : occ_coords) // For each occupied coordinate
    {
      // obs_gbl_pos: global obstacle pos
      // Check if the obstacle within local map bounds
      Bonxai::Point3D obs_gbl_pos_pt3d = bonxai_map_->grid().coordToPos(coord);

      // obs_gbl_pos: With respect to (0,0,0) of world frame
      Eigen::Vector3d obs_gbl_pos = Bonxai::ConvertPoint<Eigen::Vector3d>(obs_gbl_pos_pt3d);
      if (!isInLocalMap(obs_gbl_pos)){ 
        // Point is outside the local map
        continue;
      }

      // obs_lcl_pos: With respect to local origin
      Eigen::Vector3d obs_lcl_pos = obs_gbl_pos - mp_.local_map_origin_;
      
      local_occ_map_pts_->push_back(pcl::PointXYZ(obs_lcl_pos(0), obs_lcl_pos(1), obs_lcl_pos(2)));

      local_global_occ_map_pts_->push_back(pcl::PointXYZ(obs_gbl_pos_pt3d.x, obs_gbl_pos_pt3d.y, obs_gbl_pos_pt3d.z));
    }
  }

  local_occ_map_pts_->width = local_occ_map_pts_->points.size();
  local_occ_map_pts_->height = 1;
  local_occ_map_pts_->is_dense = true; 

  local_global_occ_map_pts_->width = local_global_occ_map_pts_->points.size();
  local_global_occ_map_pts_->height = 1;
  local_global_occ_map_pts_->is_dense = true; 

}

void VoxelMap::sliceMap(const double& slice_z_cm, const double& thickness, std::vector<bool>& bool_map) 
{
  if (local_occ_map_pts_->points.empty()){
    logger_->logWarnThrottle("Local map is empty!", 1.0);
    return;
  }


  double slice_z = ((double) slice_z_cm)/100.0;

  pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_layer(new pcl::PointCloud<pcl::PointXYZ>);
  // Apply passthrough filter
  pcl::PassThrough<pcl::PointXYZ> z_filter;
  z_filter.setInputCloud(local_occ_map_pts_);
  z_filter.setFilterFieldName("z");
  z_filter.setFilterLimits(slice_z - (thickness/2) , slice_z + (thickness/2));
  z_filter.filter(*pcd_layer);

  // Iterate through each occupied point
  for (const auto& pt : *pcd_layer){
    double map_x = (pt.x)/getRes();
    double map_y = (pt.y)/getRes();

    // inflate map
    for(int x = map_x - mp_.inf_num_voxels_; x <= map_x + mp_.inf_num_voxels_; x++)
    {
      for(int y = map_y - mp_.inf_num_voxels_; y <= map_y + mp_.inf_num_voxels_; y++)
      {
        // Convert from local map coordinates to 1-D index
        int idx = x + y * mp_.local_map_num_voxels_(0);

        if (idx < 0 || idx >= (int) bool_map.size()){
          continue;
        }

        bool_map[idx] = true;
      }
    }

    // set map boundaries as occupied

    // Left and right side of map (iterate along y)
    for(int y = 0; y < mp_.local_map_num_voxels_(1); y++)
    {
      int idx_l = 0 + y * mp_.local_map_num_voxels_(0);
      int idx_r = (mp_.local_map_num_voxels_(0)-1) + y * mp_.local_map_num_voxels_(0);

      bool_map[idx_l] = bool_map[idx_r] = true;
    }

    // Top and bottom side of map (iterate along x)
    for(int x = 0; x < mp_.local_map_num_voxels_(0); x++)
    {
      int idx_top = x + 0 * mp_.local_map_num_voxels_(0);
      int idx_btm = x + (mp_.local_map_num_voxels_(1)-1) * mp_.local_map_num_voxels_(0);

      bool_map[idx_top] = bool_map[idx_btm] = true;
    }
  }

  // publishSliceMap(pcd_layer);
}

/** Timer callbacks */

void VoxelMap::vizMapTimerCB()
{
  publishOccMap(local_occ_map_pts_); // publish point cloud within local map
  // publishLocalMapBounds(); // publish boundaries of local map volume
}

void VoxelMap::updateLocalMapTimerCB()
{
  updateLocalMap(); // Update local map voxels

  // Create horizontal map slices
  tm_slice_map_.start();

  {
    std::lock_guard<std::mutex> bool_map_3d_guard(bool_map_3d_mtx_);

    bool_map_3d_.origin = mp_.local_map_origin_;

    bool_map_3d_.width = mp_.local_map_num_voxels_(0);
    bool_map_3d_.height = mp_.local_map_num_voxels_(1);
    bool_map_3d_.resolution = mp_.resolution_;

    for (int z_cm = bool_map_3d_.min_height_cm; 
          z_cm <= bool_map_3d_.max_height_cm; 
          z_cm += bool_map_3d_.z_separation_cm)
    {
      bool_map_3d_.bool_maps[z_cm] = std::vector<bool>(mp_.local_map_num_voxels_(0) * mp_.local_map_num_voxels_(1), false);

      sliceMap(z_cm, getRes(), bool_map_3d_.bool_maps[z_cm]);
    }
  }
  
  tm_slice_map_.stop(verbose_print_);

  local_map_updated_ = true;
}

void VoxelMap::pubLocalMapTFTimerCB()
{
  // broadcast tf link from global map frame to local map origin 
  geometry_msgs::msg::TransformStamped gbl_to_lcl_origin_tf;

  gbl_to_lcl_origin_tf.header.stamp = node_->get_clock()->now();
  gbl_to_lcl_origin_tf.header.frame_id = mp_.map_frame; 
  gbl_to_lcl_origin_tf.child_frame_id = mp_.local_map_frame; 

  gbl_to_lcl_origin_tf.transform.translation.x = mp_.local_map_origin_(0);
  gbl_to_lcl_origin_tf.transform.translation.y = mp_.local_map_origin_(1);
  gbl_to_lcl_origin_tf.transform.translation.z = mp_.local_map_origin_(2);

  gbl_to_lcl_origin_tf.transform.rotation.x = 0.0;
  gbl_to_lcl_origin_tf.transform.rotation.y = 0.0;
  gbl_to_lcl_origin_tf.transform.rotation.z = 0.0;
  gbl_to_lcl_origin_tf.transform.rotation.w = 1.0;
  
  tf_broadcaster_->sendTransform(gbl_to_lcl_origin_tf);
}

void VoxelMap::checkCollisionsTimerCB()
{
  if (!isPoseValid()){
    return;
  }

  // Eigen::Vector3d query_pos( 	body_to_map.block<3,1>(0,3)(0),
  //                             body_to_map.block<3,1>(0,3)(1),
  //                             body_to_map.block<3,1>(0,3)(2);

  // // Get nearest obstacle position
  // Eigen::Vector3d occ_nearest;
  // double dist_to_obs;
  // if (!getNearestOccupiedCell(query_pos, occ_nearest, dist_to_obs)){
  //   return;
  // }

  // // Publish collision sphere visualizations.
  // if (dist_to_obs <= col_warn_radius_){
  //   publishCollisionSphere(occ_nearest, dist_to_obs, col_fatal_radius_, col_warn_radius_);
  // }
}

/** Subscriber callbacks */

void VoxelMap::cloudOdomCB( const sensor_msgs::msg::PointCloud2::SharedPtr msg_pc, 
                            const nav_msgs::msg::Odometry::SharedPtr msg_odom)
{
  setCamToMapPose(msg_odom->pose.pose);
  pcd2MsgToMap(*msg_pc);
  md_.last_sensor_msg_time = node_->get_clock()->now().seconds();
}

/** Publishers */

void VoxelMap::publishOccMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& occ_map_pts)
{
  sensor_msgs::msg::PointCloud2 cloud_msg;
  {
    std::lock_guard<std::mutex> lcl_occ_map_guard(lcl_occ_map_mtx_);
    pcl::toROSMsg(*occ_map_pts, cloud_msg);
  }
  
  cloud_msg.header.frame_id = mp_.local_map_frame;
  cloud_msg.header.stamp = node_->get_clock()->now();

  occ_map_pub_->publish(cloud_msg);
}

// void VoxelMap::publishSliceMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& slice_map)
// {
//   if (slice_map_pub_.get_subscription_count() > 0){
//     sensor_msgs::msg::PointCloud2 cloud_msg;
//     pcl::toROSMsg(*slice_map, cloud_msg);

//     cloud_msg.header.frame_id = "local_map_frame";
//     cloud_msg.header.stamp = node_->get_clock()->now();
//     slice_map_pub_->publish(cloud_msg);
//     // logger_->logInfo(strFmt("Published occupancy grid with %ld voxels", local_occ_map_pts_.points.size()));
//   }
  
// }

// void VoxelMap::publishCollisionSphere(
//   const Eigen::Vector3d &pos, const double& dist_to_obs, 
//   const double& fatal_radius, const double& warn_radius)
// {
//   static int col_viz_id = 0;

//   visualization_msgs::Marker sphere;
//   sphere.header.frame_id = mp_.local_map_frame_;
//   sphere.header.stamp = node_->get_clock()->now();
//   sphere.type = visualization_msgs::Marker::SPHERE;
//   sphere.action = visualization_msgs::Marker::ADD;
//   sphere.ns = "collision_viz";
//   sphere.pose.orientation.w = 1.0;
//   sphere.id = col_viz_id++;

//   // Make the alpha and red color value scale from 0.0 to 1.0 depending on the distance to the obstacle. 
//   // With the upper limit being the warn_radius, and the lower limit being the fatal_radius
//   double fatal_ratio = std::clamp((warn_radius - dist_to_obs)/(warn_radius - fatal_radius), 0.0, 1.001);
  
//   if (fatal_ratio >= 1.0){
//     sphere.color.r = 1.0;
//     sphere.color.g = 0.0;
//     sphere.color.b = 0.0; 
//     sphere.color.a = 0.8;
//   }
//   else {
//     // Goes from blue to purple
//     sphere.color.r = fatal_ratio*(1.0);
//     sphere.color.g = 0.0;
//     sphere.color.b = 1.0; // If fatal, make blue value 0.0, so sphere is entire red.
//     sphere.color.a = 0.3 + (fatal_ratio*(0.8-0.3));
//   }

//   double scale = fatal_ratio < 1.0 ? 0.35 : 0.6;

//   sphere.scale.x = scale;
//   sphere.scale.y = scale;
//   sphere.scale.z = scale;
//   sphere.pose.position.x = pos(0);
//   sphere.pose.position.y = pos(1);
//   sphere.pose.position.z = pos(2);

//   collision_viz_pub_->publish(sphere);
// }

// void VoxelMap::publishLocalMapBounds()
// {
//   geometry_msgs::PolygonStamped local_map_poly;
//   local_map_poly.header.frame_id = mp_.local_map_frame_;
//   local_map_poly.header.stamp = node_->get_clock()->now();

//   geometry_msgs::Point32 min_corner, corner_0, corner_1, max_corner;
//   min_corner.x = mp_.local_map_origin_(0);
//   min_corner.y = mp_.local_map_origin_(1);

//   corner_0.x = mp_.local_map_max_(0);
//   corner_0.y = mp_.local_map_origin_(1);

//   corner_1.x = mp_.local_map_origin_(0);
//   corner_1.y = mp_.local_map_max_(1);

//   max_corner.x = mp_.local_map_max_(0);
//   max_corner.y = mp_.local_map_max_(1);

//   min_corner.z = corner_0.z = corner_1.z = max_corner.z = mp_.local_map_origin_(2);;

//   local_map_poly.polygon.points.push_back(min_corner);
//   local_map_poly.polygon.points.push_back(corner_0);
//   local_map_poly.polygon.points.push_back(max_corner);
//   local_map_poly.polygon.points.push_back(corner_1);

//   local_map_bounds_pub_->publish(local_map_poly);
// }

}  // namespace voxel_map
