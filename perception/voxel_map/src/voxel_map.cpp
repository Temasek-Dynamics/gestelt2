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

VoxelMap::VoxelMap(rclcpp::Node::SharedPtr node, 
                    const Eigen::Vector3d& map_origin, 
                    const int& num_drones) 
: node_(node)
{
	// Create callback groups

  mapping_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  tf_broadcast_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

	logger_ = std::make_shared<logger_wrapper::LoggerWrapper>(node_->get_logger(), node_->get_clock());

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
  tf_buffer_ =
    std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ =
    std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  initParams();

  reset(mp_.resolution_);

  initPubSubTimer();

  for (int i = 0; i < num_drones; i++){
    swarm_poses_.push_back(Eigen::Vector3d::Constant(999.9));
    swarm_vels_.push_back(Eigen::Vector3d::Constant(0.0));
  }

  if (dbg_input_entire_map_){
    md_.cam_to_map.block<3,1>(0,3) = map_origin;

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

  // initialize timer with drone_id
  // tm_update_local_map_.updateID(drone_id_);
  // tm_bonxai_insert_.updateID(drone_id_);
  // tm_slice_map_.updateID(drone_id_);

}

VoxelMap::~VoxelMap(){
}

void VoxelMap::initPubSubTimer()
{
  /* Initialize Subscribers */
  if (!dbg_input_entire_map_){
    // cloud_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(
    //   node_, "cloud", rmw_qos_profile_sensor_data);
    // odom_sub_ = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>(
    //   node_, "odom", rmw_qos_profile_sensor_data);

    // sync_cloud_odom_ = std::make_shared<message_filters::Synchronizer<SyncPolicyCloudOdom>>(
    //     SyncPolicyCloudOdom(5), *cloud_sub_, *odom_sub_);
    // sync_cloud_odom_->registerCallback(&VoxelMap::cloudOdomCB, this);
    // sync_cloud_odom_->setAgePenalty(0.50);
  }
  cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    "cloud", rclcpp::SensorDataQoS(), std::bind(&VoxelMap::cloudCB, this, _1) );

  /* Initialize Publishers */
	occ_map_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
    "occ_map", 10);
	// slice_map_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("slice", rclcpp::SensorDataQoS());
	local_map_bounds_pub_ = node_->create_publisher<geometry_msgs::msg::PolygonStamped>(
    "local_map/bounds", 10);
  
  /* Initialize Subscribers */
	reset_map_sub_ = node_->create_subscription<std_msgs::msg::Empty>(
    "reset_map", 10, std::bind(&VoxelMap::resetMapCB, this, _1) );

  /* Initialize ROS Timers */
	// viz_map_timer_ = node_->create_wall_timer((1.0/viz_occ_map_freq_) *1000ms, 
  //   std::bind(&VoxelMap::vizMapTimerCB, this), reentrant_group_);
	update_local_map_timer_ = node_->create_wall_timer((1.0/update_local_map_freq_) *1000ms, 
    std::bind(&VoxelMap::updateLocalMapTimerCB, this), mapping_group_);
	pub_lcl_map_tf_timer_ = node_->create_wall_timer((1.0/100.0) *1000ms, 
    std::bind(&VoxelMap::pubLocalMapTFTimerCB, this), tf_broadcast_group_);

  if (check_collisions_){ // True if we want to publish collision visualizations between the agent and static obstacles
    // Publisher for collision visualizations
	  collision_viz_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("static_collisions", 10);
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

  node_->declare_parameter(param_ns+".dyn_obs.mark_in_occ_map", false);
  node_->declare_parameter(param_ns+".dyn_obs.time_vel", 0.5);

  node_->declare_parameter(param_ns+".map_slicing.min_height_cm", -1);
  node_->declare_parameter(param_ns+".map_slicing.max_height_cm",  -1);
  node_->declare_parameter(param_ns+".map_slicing.z_sep_cm",  -1);

  node_->declare_parameter(param_ns+".global_map.size_x", -1.0);
  node_->declare_parameter(param_ns+".global_map.size_y", -1.0);
  node_->declare_parameter(param_ns+".global_map.size_z", -1.0);

  node_->declare_parameter(param_ns+".local_map.size_x", -1.0);
  node_->declare_parameter(param_ns+".local_map.size_y", -1.0);
  node_->declare_parameter(param_ns+".local_map.size_z", -1.0);
  node_->declare_parameter(param_ns+".local_map.update_local_map_frequency",  -1.0);
  node_->declare_parameter(param_ns+".local_map.viz_map_frequency",  -1.0);

  node_->declare_parameter(param_ns+".occ_map.resolution", -1.0);
  node_->declare_parameter(param_ns+".occ_map.dynamic_inflation", -1.0);
  node_->declare_parameter(param_ns+".occ_map.static_inflation", -1.0);
  node_->declare_parameter(param_ns+".occ_map.max_range", -1.0);
  node_->declare_parameter(param_ns+".occ_map.ground_height", 0.0);

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

  dyn_obs_mark_in_occ_map_ = node_->get_parameter(param_ns+".dyn_obs.mark_in_occ_map").as_bool();
  time_vel_ = node_->get_parameter(param_ns+".dyn_obs.time_vel").as_double();

  bool_map_3d_.min_height_cm = node_->get_parameter(param_ns+".map_slicing.min_height_cm").as_int();
  bool_map_3d_.max_height_cm = node_->get_parameter(param_ns+".map_slicing.max_height_cm").as_int();
  bool_map_3d_.z_sep_cm = node_->get_parameter(param_ns+".map_slicing.z_sep_cm").as_int();

  auto cmToM = [&](const int& val_cm){ 
    /* Convert from units of centimeters to meters*/
    return ((double) val_cm)/100.0;  
  };

  bool_map_3d_.z_sep_m = cmToM(bool_map_3d_.z_sep_cm); 
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
  mp_.agent_inflation_ = node_->get_parameter(param_ns+".occ_map.dynamic_inflation").as_double();
  mp_.static_inflation_ = node_->get_parameter(param_ns+".occ_map.static_inflation").as_double();
  mp_.max_range = node_->get_parameter(param_ns+".occ_map.max_range").as_double();
  mp_.ground_height_ = node_->get_parameter(param_ns+".occ_map.ground_height").as_double();
  mp_.inf_static_vox_ = std::ceil(mp_.static_inflation_/mp_.resolution_);
  mp_.inf_dyn_vox_ = std::ceil(mp_.agent_inflation_/mp_.resolution_);

  /* Frame IDs */
  mp_.global_map_frame = node_->get_parameter("map_frame").as_string();
  mp_.local_map_frame = node_->get_parameter("local_map_frame").as_string();
  mp_.camera_frame = node_->get_parameter("camera_frame").as_string();
  mp_.base_link_frame = node_->get_parameter("base_link_frame").as_string();

  /* Collision checking*/
  check_collisions_ = node_->get_parameter(param_ns+".collision_check.enable").as_bool();
  col_warn_radius_ = node_->get_parameter(param_ns+".collision_check.warn_radius").as_double();
  col_fatal_radius_ = node_->get_parameter(param_ns+".collision_check.fatal_radius").as_double();
  check_col_freq_ = node_->get_parameter(param_ns+".collision_check.check_collision_frequency").as_double();

}

void VoxelMap::reset(const double& resolution){

  // Set up Bonxai data structure
  bonxai_map_ = std::make_unique<BonxaiT>(resolution);
  
  // Set up KDTree for collision checks
  // KD_TREE(float delete_param = 0.5, float balance_param = 0.6 , float box_length = 0.2);
  // kdtree_ = std::make_unique<KD_TREE<pcl::PointXYZ>>(0.5, 0.6, 0.1);

  lcl_pcd_lclmapframe_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  lcl_pcd_lclmapframe_->header.frame_id = mp_.local_map_frame;
  lcl_pcd_fixedmapframe_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  lcl_pcd_fixedmapframe_->header.frame_id = mp_.global_map_frame;
  pcd_in_map_frame_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcd_in_map_frame_->header.frame_id = mp_.global_map_frame;

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
    -mp_.local_map_size_(2) / 2.0);

  // Local map max is at the other corner of the local map i.e. (uav_pos_x + local_sz_x /2, uav_pos_y + local_sz_y/2, 0), relative to the current position of the robot
  mp_.local_map_max_ = Eigen::Vector3d(
    mp_.local_map_size_(0) / 2.0, 
    mp_.local_map_size_(1) / 2.0, 
    mp_.local_map_size_(2) / 2.0);

  mp_.local_map_num_voxels_ = (mp_.local_map_size_.cwiseProduct(Eigen::Vector3d::Constant(1/mp_.resolution_))).cast<int>();

  md_.last_cloud_cb_time = node_->get_clock()->now().seconds();
}

/* Core methods */

void VoxelMap::updateLocalMap(){

  try {
    // map to base_link
    auto tf_map_to_bl = tf_buffer_->lookupTransform(
      mp_.base_link_frame, mp_.global_map_frame,
      tf2::TimePointZero,
      tf2_ros::fromRclcpp(rclcpp::Duration::from_seconds(1.0)));

    md_.map_to_bl = tf2::transformToEigen(
      tf_map_to_bl.transform).matrix().cast<double>();

  } 
  catch (const tf2::TransformException & ex) {
		RCLCPP_ERROR(
			node_->get_logger(), "Could not get transform from camera frame '%s' to map frame '%s': %s",
			mp_.camera_frame.c_str(), mp_.global_map_frame.c_str(), ex.what());
    return;
  }

  // Update local map origin, which is the location of the local map origin IN the global map frame
  mp_.local_map_origin_ = Eigen::Vector3d(
    md_.map_to_bl.block<3,1>(0,3)(0) - (mp_.local_map_size_(0) / 2.0), 
    md_.map_to_bl.block<3,1>(0,3)(1) - (mp_.local_map_size_(1) / 2.0), 
    md_.map_to_bl.block<3,1>(0,3)(2) - (mp_.local_map_size_(2) / 2.0));

  // Update local map max position based on current UAV position
  mp_.local_map_max_ = Eigen::Vector3d(
    md_.map_to_bl.block<3,1>(0,3)(0) + (mp_.local_map_size_(0) / 2.0), 
    md_.map_to_bl.block<3,1>(0,3)(1) + (mp_.local_map_size_(1) / 2.0), 
    md_.map_to_bl.block<3,1>(0,3)(2) + (mp_.local_map_size_(2) / 2.0));
  
  // Get all occupied coordinates 
  std::vector<Bonxai::CoordT> occ_coords;
  bonxai_map_->getOccupiedVoxels(occ_coords);
  if (occ_coords.size() <= 1){ // Empty map
    logger_->logWarnThrottle("Skipping update of local map. Bonxai map is empty!", 1.0);
    return;
  }

  {
    std::lock_guard<std::mutex> lcl_occ_map_guard(lcl_occ_map_mtx_);

    // Clear existing local map
    lcl_pcd_lclmapframe_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    lcl_pcd_fixedmapframe_.reset(new pcl::PointCloud<pcl::PointXYZ>());

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

      // Add inflated obstacles
      // lcl_pts_fixedmapframe_.push_back(obs_gbl_pos); 
      lcl_pts_fixedmapframe_.push_back(obs_gbl_pos + Eigen::Vector3d{mp_.static_inflation_, 0.0, 0.0}); 
      lcl_pts_fixedmapframe_.push_back(obs_gbl_pos + Eigen::Vector3d{-mp_.static_inflation_, 0.0, 0.0}); 
      lcl_pts_fixedmapframe_.push_back(obs_gbl_pos + Eigen::Vector3d{0.0, mp_.static_inflation_, 0.0}); 
      lcl_pts_fixedmapframe_.push_back(obs_gbl_pos + Eigen::Vector3d{0.0, -mp_.static_inflation_, 0.0}); 
      lcl_pts_fixedmapframe_.push_back(obs_gbl_pos + Eigen::Vector3d{0.0, 0.0, mp_.static_inflation_}); 
      lcl_pts_fixedmapframe_.push_back(obs_gbl_pos + Eigen::Vector3d{0.0, 0.0, -mp_.static_inflation_}); 

      // obs_lcl_pos: With respect to local origin
      Eigen::Vector3d obs_lcl_pos = obs_gbl_pos - mp_.local_map_origin_;
      
      // Add pts in local map to local_map and fixed_map frame
      lcl_pcd_lclmapframe_->push_back(
        pcl::PointXYZ(obs_lcl_pos(0), obs_lcl_pos(1), obs_lcl_pos(2)));
      lcl_pcd_fixedmapframe_->push_back(
        pcl::PointXYZ(obs_gbl_pos_pt3d.x, obs_gbl_pos_pt3d.y, obs_gbl_pos_pt3d.z));
    }

    lcl_pcd_lclmapframe_->width = lcl_pcd_lclmapframe_->points.size();
    lcl_pcd_lclmapframe_->height = 1;
    lcl_pcd_lclmapframe_->is_dense = true; 

    lcl_pcd_fixedmapframe_->width = lcl_pcd_fixedmapframe_->points.size();
    lcl_pcd_fixedmapframe_->height = 1;
    lcl_pcd_fixedmapframe_->is_dense = true; 

  }

  // Add local occ points in fixed map frame to KDTree
  // kdtree_->Build(lcl_pcd_fixedmapframe_->points);
}

void VoxelMap::getMapSlice(const double& slice_z_cm, 
  const double& thickness, std::vector<bool>& bool_map) 
{
  // if (lcl_pcd_lclmapframe_->points.empty()){
  //   logger_->logWarnThrottle("Local map is empty!", 1.0);
  //   return;
  // }

  double slice_z = ((double) slice_z_cm)/100.0;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  // Apply passthrough filter
  pcl::PassThrough<pcl::PointXYZ> z_filter;
  z_filter.setInputCloud(lcl_pcd_lclmapframe_);
  z_filter.setFilterFieldName("z");
  z_filter.setFilterLimits(slice_z - (thickness/2) , slice_z + (thickness/2));
  z_filter.filter(*cloud);
  
  for (const auto& pt : *cloud){ // Iterate through each occupied point
    // Position in local map frame 
    double lcl_grid_x = (pt.x)/getRes();
    double lcl_grid_y = (pt.y)/getRes();

    // inflate map 
    for(int x = lcl_grid_x - mp_.inf_static_vox_; x <= lcl_grid_x + mp_.inf_static_vox_; x++)
    {
      for(int y = lcl_grid_y - mp_.inf_static_vox_; y <= lcl_grid_y + mp_.inf_static_vox_; y++)
      {
        // Convert from 2D coordinates to 1D index
        int idx = x + y * mp_.local_map_num_voxels_(0);

        if (idx < 0 || idx >= (int) bool_map.size()){ 
          // if out of map, skip
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

  // publishSliceMap(cloud);
}

/** Timer callbacks */

void VoxelMap::vizMapTimerCB()
{
  // publishOccMap(lcl_pcd_lclmapframe_); // publish point cloud within local map
}

void VoxelMap::updateLocalMapTimerCB()
{
  if (isTimeout(md_.last_cloud_cb_time, 0.5)){
    logger_->logWarnThrottle(strFmt("cloud message timeout exceeded 0.5s: (%f, %f)",  
                                      md_.last_cloud_cb_time, node_->get_clock()->now().seconds()), 1.0);
  }

  updateLocalMap(); // Update local map voxels

  publishOccMap(lcl_pcd_fixedmapframe_);
  publishLocalMapBounds(); // publish boundaries of local map volume

  // Create horizontal map slices
  // tm_slice_map_.start();

  std::lock_guard<std::mutex> bool_map_3d_guard(bool_map_3d_mtx_);

  // Generate boolean map from static obstacles
  {
    bool_map_3d_.origin = mp_.local_map_origin_;

    bool_map_3d_.width = mp_.local_map_num_voxels_(0);
    bool_map_3d_.height = mp_.local_map_num_voxels_(1);
    bool_map_3d_.resolution = mp_.resolution_;

    // Iterate through all heights of the map
    for (int z_cm = bool_map_3d_.min_height_cm; 
          z_cm <= bool_map_3d_.max_height_cm; 
          z_cm += bool_map_3d_.z_sep_cm)
    {
      bool_map_3d_.bool_maps[z_cm] = std::vector<bool>(
        mp_.local_map_num_voxels_(0) * mp_.local_map_num_voxels_(1), false);
      // Get a slice of the map occupancy grid as a boolean array
      getMapSlice(z_cm, getRes(), bool_map_3d_.bool_maps[z_cm]);
    }
  }

  // [Communication-less] Add obstacles to boolean map based on position of other drones
  if (dyn_obs_mark_in_occ_map_){
    for (int id = 0; id < (int)swarm_poses_.size(); id++){ // for each drone id
      if (id == drone_id_){
        continue;
      }

      if (!isInLocalMap(swarm_poses_[id])){ 
        // Point is outside the local map
        continue;
      }

      // Convert from global map to local map 
      Eigen::Vector3d lcl_map_start_pos = swarm_poses_[id] - mp_.local_map_origin_; // in [m] meters
      double z_m = swarm_poses_[id](2);

      auto roundToMultInt = [&](const int& num, const int& mult, 
                                const int& min, const int& max)
      {
        if (mult == 0){
          return num;
        }

        if (num > max){
          return max;
        }

        if (num < min){
          return min;
        }

        int rem = (int)num % mult;
        if (rem == 0){
          return num;
        }

        return rem < (mult/2) ? (num-rem) : (num-rem) + mult;
      };

      int z_cm = roundToMultInt((int) (z_m * 100.0), 
                                    bool_map_3d_.z_sep_cm,
                                    bool_map_3d_.min_height_cm,
                                    bool_map_3d_.max_height_cm);

      int top_height = z_cm + bool_map_3d_.z_sep_cm;
      int btm_height = z_cm - bool_map_3d_.z_sep_cm;

      // Add the other layer sandwiching the current drone position as occupied too
      int z2_cm = z_cm;  
      if (top_height <= bool_map_3d_.max_height_cm 
          && (double)z_cm <= z_m * 100.0 
          && z_m * 100.0 < (double)top_height)
      {
        z2_cm = top_height;
      }
      else if (btm_height >= bool_map_3d_.min_height_cm
        && (double)btm_height < z_m * 100.0
        && z_m * 100.0 < (double)z_cm ) 
      {
        z2_cm = btm_height;
      }

      // Iterate for a short duration along velocity vector
      for (double t = 0; t <= time_vel_; t += 0.05 )
      {
        Eigen::Vector3d lcl_map_pos = lcl_map_start_pos + t * swarm_vels_[id];

        // Convert from fixed map origin to local map origin
        Eigen::Vector3d lcl_map_grid = lcl_map_pos/getRes(); // In grid coordinares

        // inflate map 
        for(int x = lcl_map_grid(0) - mp_.inf_dyn_vox_; x <= lcl_map_grid(0) + mp_.inf_dyn_vox_; x++)
        {
          for(int y = lcl_map_grid(1) - mp_.inf_dyn_vox_; y <= lcl_map_grid(1) + mp_.inf_dyn_vox_; y++)
          {
            // Convert from 2D coordinates to 1D index
            int idx = x + y * mp_.local_map_num_voxels_(0);

            if (idx < 0 || idx >= (int) bool_map_3d_.bool_maps[z_cm].size()){ 
              // if out of map, skip
              continue;
            }

            bool_map_3d_.bool_maps[z_cm][idx] = true;
            bool_map_3d_.bool_maps[z2_cm][idx] = true;
          }
        }

      }

    }

  }

  // tm_slice_map_.stop(verbose_print_);

  local_map_updated_ = true;
}

void VoxelMap::pubLocalMapTFTimerCB()
{
  // broadcast tf link from global map frame to local map origin 
  geometry_msgs::msg::TransformStamped gbl_to_lcl_origin_tf;

  gbl_to_lcl_origin_tf.header.stamp = node_->get_clock()->now();
  gbl_to_lcl_origin_tf.header.frame_id = mp_.global_map_frame; 
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
  // // Get nearest obstacle position
  // Eigen::Vector3d occ_nearest;
  // double dist_to_obs;
  // // if (!getNearestOccupiedCell(md_.map_to_cam.block<3,1>(0,3), occ_nearest, dist_to_obs)){
  // //   return;
  // // }

  // // Publish collision sphere visualizations.
  // if (dist_to_obs <= col_warn_radius_){
  //   publishCollisionSphere(occ_nearest, dist_to_obs, col_fatal_radius_, col_warn_radius_);
  // }
  
}

/** Subscriber callbacks */

void VoxelMap::resetMapCB(const std_msgs::msg::Empty::SharedPtr )
{
  bonxai_map_ = std::make_unique<BonxaiT>(mp_.resolution_);
  logger_->logWarn("Reset map as requested by user!");
}

void VoxelMap::cloudCB(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  md_.last_cloud_cb_time = node_->get_clock()->now().seconds();

  // Input Point cloud is assumed to be in frame id of the sensor
  if (msg->data.empty()){
    logger_->logWarnThrottle("Empty point cloud received!", 1.0);
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr pcd = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  // Get point cloud from ROS message
  pcl::fromROSMsg(*msg, *pcd);

  // Some code here taken from bonxai_ros/bonxai_server.cpp

  // remove NaN and Inf values
  size_t filtered_index = 0;
  for (const auto& point : (*pcd).points) {
    if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
      (*pcd).points[filtered_index++] = point;
    }
  }
  (*pcd).resize(filtered_index);

  // Get camera to map frame TF
  try {
    auto tf_cam_to_map = tf_buffer_->lookupTransform(
      mp_.global_map_frame, mp_.camera_frame,
      msg->header.stamp,
      rclcpp::Duration::from_seconds(1.0));

      md_.cam_to_map = tf2::transformToEigen(
        tf_cam_to_map.transform).matrix().cast<double>();

      md_.map_to_cam = md_.cam_to_map.inverse();
  } 
  catch (const tf2::TransformException & ex) {
		RCLCPP_ERROR(
			node_->get_logger(), "Could not get transform from camera frame '%s' to map frame '%s': %s",
			mp_.camera_frame.c_str(), mp_.global_map_frame.c_str(), ex.what());
    return;
  }

  if (!isInGlobalMap(md_.map_to_cam.block<3,1>(0,3)))
  {
    logger_->logErrorThrottle(strFmt("Camera pose (%.2f, %.2f, %.2f) is not \
        within global map boundary. Skip PCD insertion.", 
       md_.map_to_cam.col(3)(0), md_.map_to_cam.col(3)(1), md_.map_to_cam.col(3)(2)), 1.0);
    return;
  }

  // Transform point cloud from camera frame to global_map_frame 
  pcl::transformPointCloud(*pcd, *pcd_in_map_frame_, md_.cam_to_map);

  // Getting the Translation from the sensor to the Global Reference Frame
  const pcl::PointXYZ sensor_origin(
    md_.cam_to_map(0, 3), md_.cam_to_map(1, 3), md_.cam_to_map(2, 3));

  // tm_bonxai_insert_.start();
  // Add point cloud to bonxai_maps_
  bonxai_map_->insertPointCloud(pcd_in_map_frame_->points, sensor_origin, mp_.max_range);
  // tm_bonxai_insert_.stop(false);
}

/** Publishers */

void VoxelMap::publishOccMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& occ_map_pts)
{
  sensor_msgs::msg::PointCloud2 cloud_msg;
  {
    std::lock_guard<std::mutex> lcl_occ_map_guard(lcl_occ_map_mtx_);
    pcl::toROSMsg(*occ_map_pts, cloud_msg);
  }
  
  cloud_msg.header.stamp = node_->get_clock()->now();
  cloud_msg.header.frame_id = mp_.global_map_frame;

  occ_map_pub_->publish(cloud_msg);
}

void VoxelMap::publishCollisionSphere(
  const Eigen::Vector3d &pos, const double& dist_to_obs, 
  const double& fatal_radius, const double& warn_radius)
{
  static int col_viz_id = 0;

  visualization_msgs::msg::Marker sphere;
  sphere.header.frame_id = mp_.global_map_frame;
  sphere.header.stamp = node_->get_clock()->now();
  sphere.type = visualization_msgs::msg::Marker::SPHERE;
  sphere.action = visualization_msgs::msg::Marker::ADD;
  sphere.ns = "static_collision";
  sphere.pose.orientation.w = 1.0;
  sphere.id = col_viz_id++;

  // Make the alpha and red color value scale from 0.0 to 1.0 depending on the distance to the obstacle. 
  // With the upper limit being the warn_radius, and the lower limit being the fatal_radius
  double fatal_ratio = std::clamp((warn_radius - dist_to_obs)/(warn_radius - fatal_radius), 0.0, 1.001);
  
  if (fatal_ratio >= 1.0){
    sphere.color.r = 1.0;
    sphere.color.g = 0.0;
    sphere.color.b = 0.0; 
    sphere.color.a = 0.8;
  }
  else {
    // Goes from blue to purple
    sphere.color.r = fatal_ratio*(1.0);
    sphere.color.g = 0.0;
    sphere.color.b = 1.0; // If fatal, make blue value 0.0, so sphere is entire red.
    sphere.color.a = 0.3 + (fatal_ratio*(0.8-0.3));
  }

  double scale = fatal_ratio < 1.0 ? 0.35 : 0.6;

  sphere.scale.x = scale;
  sphere.scale.y = scale;
  sphere.scale.z = scale;
  sphere.pose.position.x = pos(0);
  sphere.pose.position.y = pos(1);
  sphere.pose.position.z = pos(2);

  collision_viz_pub_->publish(sphere);
}


// void VoxelMap::publishSliceMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& slice_map)
// {
//   if (slice_map_pub_.get_subscription_count() > 0){
//     sensor_msgs::msg::PointCloud2 cloud_msg;
//     pcl::toROSMsg(*slice_map, cloud_msg);

//     cloud_msg.header.frame_id = "local_map_frame";
//     cloud_msg.header.stamp = node_->get_clock()->now();
//     slice_map_pub_->publish(cloud_msg);
//     // logger_->logInfo(strFmt("Published occupancy grid with %ld voxels", lcl_pcd_lclmapframe_.points.size()));
//   }
  
// }

void VoxelMap::publishLocalMapBounds()
{
  geometry_msgs::msg::PolygonStamped local_map_poly;
  local_map_poly.header.frame_id = mp_.global_map_frame;
  local_map_poly.header.stamp = node_->get_clock()->now();

  geometry_msgs::msg::Point32 min_corner, corner_0, corner_1, max_corner;
  min_corner.x = mp_.local_map_origin_(0);
  min_corner.y = mp_.local_map_origin_(1);

  corner_0.x = mp_.local_map_max_(0);
  corner_0.y = mp_.local_map_origin_(1);

  corner_1.x = mp_.local_map_origin_(0);
  corner_1.y = mp_.local_map_max_(1);

  max_corner.x = mp_.local_map_max_(0);
  max_corner.y = mp_.local_map_max_(1);

  min_corner.z = corner_0.z = corner_1.z = max_corner.z = mp_.local_map_origin_(2);

  local_map_poly.polygon.points.push_back(min_corner);
  local_map_poly.polygon.points.push_back(corner_0);
  local_map_poly.polygon.points.push_back(max_corner);
  local_map_poly.polygon.points.push_back(corner_1);

  local_map_bounds_pub_->publish(local_map_poly);
}

void VoxelMap::pcd2MsgToMap(const sensor_msgs::msg::PointCloud2 &msg)
{
  // // Input Point cloud is assumed to be in frame id of the sensor
  // if (msg.data.empty()){
  //   logger_->logWarnThrottle("Empty point cloud received!", 1.0);
  //   return;
  // }

  // // Get point cloud from ROS message
  // pcl::PointCloud<pcl::PointXYZ>::Ptr pcd = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  // pcl::fromROSMsg(msg, *pcd);

  // // Transform point cloud from camera frame to global_map_frame (the global reference frame)
  // pcl::transformPointCloud(*pcd, *pcd_in_map_frame_, md_.cam_to_map);
  // pcd_in_map_frame_->header.frame_id = mp_.global_map_frame;

  // // Getting the Translation from the sensor to the Global Reference Frame
  // const pcl::PointXYZ sensor_origin(
  //   md_.cam_to_map(0, 3), md_.cam_to_map(1, 3), md_.cam_to_map(2, 3));

  // tm_bonxai_insert_.start();
  // // Add point cloud to bonxai_maps_
  // bonxai_map_->insertPointCloud(pcd_in_map_frame_->points, sensor_origin, mp_.max_range);
  // tm_bonxai_insert_.stop(false);
}

}  // namespace voxel_map
