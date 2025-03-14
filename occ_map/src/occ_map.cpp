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

#include "occ_map/occ_map.hpp"

#include "occ_map/cost_values.hpp"

#include "tf2_ros/create_timer_ros.h"

namespace occ_map
{

OccMap::OccMap(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("occ_map", "", options),
  name_("occ_map")
{
  is_lifecycle_follower_ = false;
  init();
}

OccMap::OccMap(
  const std::string & name,
  const std::string & parent_namespace,
  const bool & use_sim_time)
: nav2_util::LifecycleNode(name, "",
    // NodeOption arguments take precedence over the ones provided on the command line
    // use this to make sure the node is placed on the provided namespace
    rclcpp::NodeOptions().arguments({
    "--ros-args", "-r", std::string("__ns:=") +
    nav2_util::add_namespaces(parent_namespace, name),
    "--ros-args", "-r", name + ":" + std::string("__node:=") + name,
    "--ros-args", "-p", "use_sim_time:=" + std::string(use_sim_time ? "true" : "false"),
  })),
  name_(name)
{
  init();
}

OccMap::~OccMap(){
}

int OccMap::getCostIdx(const Eigen::Vector3i &idx)
{
  Bonxai::CoordT coord = {idx(0), idx(1), idx(2)};

  if (bonxai_map_->isOccupied(coord)){
    return LETHAL_OBSTACLE;
  }

  if (bonxai_map_->isUnknown(coord)){
    return NO_INFORMATION;
  }

  if (bonxai_map_->isFree(coord)){
    return FREE_SPACE;
  }

  return NO_INFORMATION; 
}

int OccMap::getCost(const Eigen::Vector3d &pos)
{
  const auto coord = bonxai_map_->grid().posToCoord(pos);

  if (bonxai_map_->isOccupied(coord)){
    return LETHAL_OBSTACLE;
  }

  if (bonxai_map_->isUnknown(coord)){
    return NO_INFORMATION;
  }

  if (bonxai_map_->isFree(coord)){
    return FREE_SPACE;
  }
  
  return NO_INFORMATION; 
}


void OccMap::init() 
{
  logger_->logInfo("Creating OccMap");

	logger_ = std::make_shared<logger_wrapper::LoggerWrapper>(get_logger(), get_clock());
  
  /* Map parameters */
  declare_parameter("print_timer", false);

  declare_parameter("update_local_map_frequency",  -1.0);
  declare_parameter("viz_map_frequency",  -1.0);

  declare_parameter("world_frame", "world");
  declare_parameter("map_frame", "local_map");
  declare_parameter("camera_frame", "camera_link");
  declare_parameter("base_link_frame", "base_link");

  declare_parameter("global_map.size_x", -1.0);
  declare_parameter("global_map.size_y", -1.0);
  declare_parameter("global_map.size_z", -1.0);

  declare_parameter("local_map.size_x", -1.0);
  declare_parameter("local_map.size_y", -1.0);
  declare_parameter("local_map.size_z", -1.0);

  /* Probabilistic map*/
  declare_parameter("occ_map.resolution", -1.0);
  declare_parameter("occ_map.static_inflation", -1.0);
  declare_parameter("occ_map.max_range", -1.0);

  declare_parameter("occ_map.prob_miss_log", 0.4);
  declare_parameter("occ_map.prob_hit_log", 0.7);
  declare_parameter("occ_map.clamp_min_log", 0.12);
  declare_parameter("occ_map.clamp_max_log", 0.97);
  declare_parameter("occ_map.occupancy_threshold_log", 0.5);

  /* Passthrough filter for input point cloud */
  declare_parameter("cloud_in_passthrough.min_z", 0.0);
  declare_parameter("cloud_in_passthrough.max_z", 5.0);

  /* Noise filter */
  declare_parameter("noise_search_radius", 0.2);
  declare_parameter("noise_min_neighbors", 3);

  declare_parameter("initial_transform_timeout", rclcpp::ParameterValue(60.0));
  declare_parameter("transform_tolerance", rclcpp::ParameterValue(0.3));

}

nav2_util::CallbackReturn
OccMap::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  logger_->logInfo("Configuring");

  try {
    getParameters();
  } catch (const std::exception & e) {
    logger_->logError(strFmt("Failed to configure occupancy map! %s.", e.what()));
      
    return nav2_util::CallbackReturn::FAILURE;
  }

  callback_group_ = create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);

  // Create the transform-related objects
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(),
    get_node_timers_interface(),
    callback_group_);
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  /* Initialize Subscribers */
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "cloud", rclcpp::SensorDataQoS(), std::bind(&OccMap::cloudCB, this, _1) );

  /* Initialize Publishers */
  occ_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "occ_map", rclcpp::SensorDataQoS());
  local_map_bounds_pub_ = create_publisher<geometry_msgs::msg::PolygonStamped>(
    "local_map/bounds", rclcpp::SensorDataQoS());
  
  /* Initialize Subscribers */
  reset_map_sub_ = create_subscription<std_msgs::msg::Empty>(
    "reset_map", 10, std::bind(&OccMap::resetMapCB, this, _1) );

  /* Initialize ROS Timers */
  viz_map_timer_ = create_wall_timer((1.0/viz_occ_map_freq_) *1000ms, 
    std::bind(&OccMap::vizMapTimerCB, this), callback_group_);
  update_local_map_timer_ = create_wall_timer((1.0/update_local_map_freq_) *1000ms, 
    std::bind(&OccMap::updateLocalMapTimerCB, this), callback_group_);

  // Set up Bonxai data structure
  bonxai_map_ = std::make_unique<Bonxai::ProbabilisticMap>(mp_.resolution_);
  bonxai_map_->setOptions(bonxai_options_);
  
  // KD_TREE(float delete_param = 0.5, float balance_param = 0.6 , float box_length = 0.2);
  kdtree_ = std::make_unique<KD_TREE<pcl::PointXYZ>>(0.5, 0.6, 0.1);

  occ_pcd_in_lcl_frame_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  occ_pcd_in_lcl_frame_->header.frame_id = mp_.map_frame;
  occ_pcd_in_gbl_frame_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  occ_pcd_in_gbl_frame_->header.frame_id = mp_.global_frame;

  // Global map origin is FIXED at a corner of the global map i.e. (-W/2, -L/2, 0)
  mp_.global_map_origin_ = Eigen::Vector3d(
    -mp_.global_map_size_(0) / 2.0, 
    -mp_.global_map_size_(1) / 2.0, 
    0.0);

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

  md_.last_cloud_cb_time = get_clock()->now().seconds();

  // Initialize point cloud filters
  z_filter_cloud_in_.setFilterFieldName("z");
  z_filter_cloud_in_.setFilterLimits(cloud_in_min_z_, cloud_in_max_z_);
  // z_filter_cloud_in_.setNegative (true);

  // initialize timer with drone_id
  // tm_update_local_map_.updateID(drone_id_);
  // tm_bonxai_insert_.updateID(drone_id_);


  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_callback_group(callback_group_, get_node_base_interface());
  executor_thread_ = std::make_unique<nav2_util::NodeThread>(executor_);

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
OccMap::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  logger_->logInfo("Activating");

  logger_->logInfo("Checking transform");

  // Check transform from global map to base link frame
  std::string tf_error;
  rclcpp::Rate r(2);
  const auto initial_transform_timeout = rclcpp::Duration::from_seconds(
    initial_transform_timeout_);
  const auto initial_transform_timeout_point = now() + initial_transform_timeout;
  while (rclcpp::ok() &&
    !tf_buffer_->canTransform(
      mp_.global_frame, mp_.base_link_frame, tf2::TimePointZero, &tf_error))
  {
    RCLCPP_INFO(
      get_logger(), "Timed out waiting for transform from %s to %s"
      " to become available, tf error: %s",
      mp_.base_link_frame.c_str(), mp_.global_frame.c_str(), tf_error.c_str());

    // Check timeout
    if (now() > initial_transform_timeout_point) {
      RCLCPP_ERROR(
        get_logger(),
        "Failed to activate %s because "
        "transform from %s to %s did not become available before timeout",
        get_name(), mp_.base_link_frame.c_str(), mp_.global_frame.c_str());

      return nav2_util::CallbackReturn::FAILURE;
    }

    // The error string will accumulate and errors will typically be the same, so the last
    // will do for the warning above. Reset the string here to avoid accumulation
    tf_error.clear();
    r.sleep();
  }

  // Get other agent's frame to map_frame transform
  try {
    auto tf_res = tf_buffer_->lookupTransform(
      mp_.map_frame, mp_.global_frame, tf2::TimePointZero,
      tf2_ros::fromRclcpp(rclcpp::Duration::from_seconds(initial_transform_timeout_)));

    mp_.world_to_map.block<3,3>(0,0) = Eigen::Quaterniond(
      tf_res.transform.rotation.w,
      tf_res.transform.rotation.x,
      tf_res.transform.rotation.y,
      tf_res.transform.rotation.z).toRotationMatrix();
    mp_.world_to_map.block<3,1>(0,3) = Eigen::Vector3d(
      tf_res.transform.translation.x,
      tf_res.transform.translation.y,
      tf_res.transform.translation.z);

    mp_.map_to_world = mp_.world_to_map.inverse();

  }
  catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(
      this->get_logger(), "Could not get transform from global frame '%s' to map frame '%s': %s",
      mp_.global_frame.c_str(), mp_.map_frame.c_str(), ex.what());
    return nav2_util::CallbackReturn::FAILURE;
  }


  occ_map_pub_->on_activate();
  local_map_bounds_pub_->on_activate();

  // Create callback
  dyn_params_handler_ = add_on_set_parameters_callback(
    std::bind(&OccMap::dynamicParametersCB, this, _1));

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
OccMap::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  logger_->logInfo("Deactivating");

  remove_on_set_parameters_callback(dyn_params_handler_.get());
  dyn_params_handler_.reset();

  occ_map_pub_->on_deactivate();
  local_map_bounds_pub_->on_deactivate();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
OccMap::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  logger_->logInfo("Cleaning up");

  executor_thread_.reset();

  bonxai_map_.reset();
  kdtree_.reset();

  tf_listener_.reset();
  tf_buffer_.reset();

  occ_pcd_in_lcl_frame_.reset();
  occ_pcd_in_gbl_frame_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
OccMap::on_shutdown(const rclcpp_lifecycle::State &)
{
  logger_->logInfo("Shutting down");
  
  return nav2_util::CallbackReturn::SUCCESS;
}

void OccMap::getParameters()
{
  get_parameter("drone_id", drone_id_);

  /* Map parameters */
  get_parameter("print_timer", print_timer_);

  /* Frame IDs */
  get_parameter("world_frame", mp_.global_frame);
  get_parameter("map_frame", mp_.map_frame);
  get_parameter("camera_frame", mp_.camera_frame);
  get_parameter("base_link_frame", mp_.base_link_frame);

  get_parameter("global_map.size_x", mp_.global_map_size_(0));
  get_parameter("global_map.size_y", mp_.global_map_size_(1));
  get_parameter("global_map.size_z", mp_.global_map_size_(2));

  get_parameter("local_map.size_x", mp_.local_map_size_(0));
  get_parameter("local_map.size_y", mp_.local_map_size_(1));
  get_parameter("local_map.size_z", mp_.local_map_size_(2));

  get_parameter("local_map.update_local_map_frequency",  update_local_map_freq_);
  get_parameter("local_map.viz_map_frequency", viz_occ_map_freq_);

  get_parameter("occ_map.resolution", mp_.resolution_);
  get_parameter("occ_map.static_inflation", mp_.static_inflation_);
  get_parameter("occ_map.max_range", mp_.max_range);

  mp_.inf_static_vox_ = std::ceil(mp_.static_inflation_/mp_.resolution_);

  /* Probabilistic map*/
  get_parameter("occ_map.prob_miss_log", bonxai_options_.prob_miss_log);
  get_parameter("occ_map.prob_hit_log", bonxai_options_.prob_hit_log);
  get_parameter("occ_map.clamp_min_log", bonxai_options_.clamp_min_log);
  get_parameter("occ_map.clamp_max_log", bonxai_options_.clamp_max_log);
  get_parameter("occ_map.occupancy_threshold_log", bonxai_options_.occupancy_threshold_log);

  bonxai_options_.prob_miss_log = Bonxai::ProbabilisticMap::logods(bonxai_options_.prob_miss_log);
  bonxai_options_.prob_hit_log = Bonxai::ProbabilisticMap::logods(bonxai_options_.prob_hit_log);
  bonxai_options_.clamp_min_log = Bonxai::ProbabilisticMap::logods(bonxai_options_.clamp_min_log);
  bonxai_options_.clamp_max_log = Bonxai::ProbabilisticMap::logods(bonxai_options_.clamp_max_log);
  bonxai_options_.occupancy_threshold_log = Bonxai::ProbabilisticMap::logods(bonxai_options_.occupancy_threshold_log);

  /* Noise filter */
  get_parameter("noise_search_radius", noise_search_radius_);
  get_parameter("noise_min_neighbors", noise_min_neighbors_);

  /* Passthrough filter for input point cloud */
  get_parameter("cloud_in_min_z", cloud_in_min_z_);
  get_parameter("cloud_in_max_z", cloud_in_max_z_);

  get_parameter("initial_transform_timeout", initial_transform_timeout_);
  get_parameter("transform_tolerance", transform_tolerance_);

}

/* Core methods */

void OccMap::updateLocalMap(){

  try {
    // map to base_link
    auto tf_bl_to_map = tf_buffer_->lookupTransform(
      mp_.global_frame, mp_.base_link_frame,
      tf2::TimePointZero,
      tf2_ros::fromRclcpp(rclcpp::Duration::from_seconds(1.0)));

    md_.bl_to_map = tf2::transformToEigen(
      tf_bl_to_map.transform).matrix().cast<double>();
  } 
  catch (const tf2::TransformException & ex) {
    logger_->logError(strFmt("Could not get transform from base link frame '%s' to map frame '%s': %s",
			mp_.base_link_frame.c_str(), mp_.global_frame.c_str(), ex.what()));
    return;
  }

  // Update local map origin, which is the location of the local map origin IN the global map frame
  mp_.local_map_origin_ = Eigen::Vector3d(
    md_.bl_to_map.block<3,1>(0,3)(0) - (mp_.local_map_size_(0) / 2.0), 
    md_.bl_to_map.block<3,1>(0,3)(1) - (mp_.local_map_size_(1) / 2.0), 
    0.0);

  // Update local map max position based on current UAV position
  mp_.local_map_max_ = Eigen::Vector3d(
    md_.bl_to_map.block<3,1>(0,3)(0) + (mp_.local_map_size_(0) / 2.0), 
    md_.bl_to_map.block<3,1>(0,3)(1) + (mp_.local_map_size_(1) / 2.0), 
    mp_.local_map_size_(2));

  // Get all occupied coordinates 
  std::vector<Bonxai::CoordT> occ_coords;
  {
    std::lock_guard<std::mutex> mtx_grd(bonxai_map_mtx_);
    bonxai_map_->getOccupiedVoxels(occ_coords);
  }
  if (occ_coords.size() <= 1){ // Empty map
    logger_->logWarnThrottle("Skipping update of local map. Bonxai map is empty!", 1.0);
    return;
  }

  {
    std::lock_guard<std::mutex> lcl_occ_map_guard(lcl_occ_map_mtx_);

    // Clear existing local map
    raw_lcl_pcd_in_global_frame_.reset(new pcl::PointCloud<pcl::PointXYZ>());

    occ_pcd_in_lcl_frame_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    occ_pcd_in_gbl_frame_.reset(new pcl::PointCloud<pcl::PointXYZ>());

    lcl_pts_in_global_frame_.clear();

    /**
     * Add points within a local bound of the agent
     */
    for (auto& coord : occ_coords) // For each occupied coordinate
    {
      // obs_gbl_pos: global obstacle pos
      Bonxai::Point3D obs_gbl_pos_pt3d = bonxai_map_->grid().coordToPos(coord);

      // obs_gbl_pos: With respect to (0,0,0) of world frame
      Eigen::Vector3d obs_gbl_pos = Bonxai::ConvertPoint<Eigen::Vector3d>(obs_gbl_pos_pt3d);

      if (!inLocalMap(obs_gbl_pos)){ 
        // Point is outside the local map
        continue;
      }

      raw_lcl_pcd_in_global_frame_->push_back(
        pcl::PointXYZ(obs_gbl_pos(0), obs_gbl_pos(1), obs_gbl_pos(2)));
    }

    kdtree_ = std::make_unique<KD_TREE<pcl::PointXYZ>>(0.5, 0.6, 0.1);
    // Add local occ points in fixed map frame to KDTree
    kdtree_->Build(raw_lcl_pcd_in_global_frame_->points);

    for (auto& pt_in_gbl_frame : raw_lcl_pcd_in_global_frame_->points) // For each point in local bounds
    {
      Eigen::Vector3d pt_in_gbl_frame_eig = Eigen::Vector3d(pt_in_gbl_frame.x, pt_in_gbl_frame.y, pt_in_gbl_frame.z);

      // pt_in_lcl_frame_eig: With respect to local origins
      Eigen::Vector3d pt_in_lcl_frame_eig = pt_in_gbl_frame_eig - mp_.local_map_origin_;

      std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> nb_points;
      // Perform radius search for each point and get nunber of points
      kdtree_->Radius_Search(pt_in_gbl_frame, noise_search_radius_, nb_points);
      if ((int) nb_points.size() < noise_min_neighbors_){
        continue;
      }

      // occ_pcd_in_lcl_frame_: Used by map slice and hence dynamic voronoi
      occ_pcd_in_lcl_frame_->push_back(
        pcl::PointXYZ(pt_in_lcl_frame_eig(0), pt_in_lcl_frame_eig(1), pt_in_lcl_frame_eig(2)));

      // occ_pcd_in_gbl_frame_: Used for visualization
      occ_pcd_in_gbl_frame_->push_back(pt_in_gbl_frame);

      // lcl_pts_in_global_frame_: Used for SFC
      lcl_pts_in_global_frame_.push_back(pt_in_gbl_frame_eig);
    }

    occ_pcd_in_lcl_frame_->width = occ_pcd_in_lcl_frame_->points.size();
    occ_pcd_in_lcl_frame_->height = 1;
    occ_pcd_in_lcl_frame_->is_dense = true; 

    occ_pcd_in_gbl_frame_->width = occ_pcd_in_gbl_frame_->points.size();
    occ_pcd_in_gbl_frame_->height = 1;
    occ_pcd_in_gbl_frame_->is_dense = true; 
  }
}

/** Timer callbacks */

void OccMap::vizMapTimerCB()
{
  {
    std::lock_guard<std::mutex> lcl_occ_map_guard(lcl_occ_map_mtx_);
    publishOccMap(occ_pcd_in_gbl_frame_);
  }

  publishLocalMapBounds(); // publish boundaries of local map volume
}

void OccMap::updateLocalMapTimerCB()
{
  updateLocalMap(); // Update local map voxels

  local_map_updated_ = true;
}

/** Subscriber callbacks */

void OccMap::resetMapCB(const std_msgs::msg::Empty::SharedPtr )
{
  std::lock_guard<std::mutex> mtx_grd(bonxai_map_mtx_);

  bonxai_map_ = std::make_unique<Bonxai::ProbabilisticMap>(mp_.resolution_);
  logger_->logWarn("Reset map as requested by user!");
}

void OccMap::cloudCB(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (isTimeout(md_.last_cloud_cb_time, 0.5)){
    logger_->logWarnThrottle(strFmt("cloud message timeout exceeded 0.5s: (%f, %f)",  
                                      md_.last_cloud_cb_time, get_clock()->now().seconds()), 1.0);
  }

  md_.last_cloud_cb_time = get_clock()->now().seconds();

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
      mp_.global_frame, mp_.camera_frame,
      msg->header.stamp,
      rclcpp::Duration::from_seconds(1.0));

      md_.cam_to_map = tf2::transformToEigen(
        tf_cam_to_map.transform).matrix().cast<double>();

      // md_.map_to_cam = md_.cam_to_map.inverse();
  } 
  catch (const tf2::TransformException & ex) {
    logger_->logError(strFmt("Could not get transform from camera frame '%s' to map frame '%s': %s",
			mp_.camera_frame.c_str(), mp_.global_frame.c_str(), ex.what()));

    return;
  }

  if (!inGlobalMap(md_.cam_to_map.block<3,1>(0,3)))
  {
    logger_->logErrorThrottle(strFmt("Camera pose (%.2f, %.2f, %.2f) is not within global map boundary. Skip PCD insertion.", 
       md_.cam_to_map.col(3)(0), md_.cam_to_map.col(3)(1), md_.cam_to_map.col(3)(2)), 1.0);
    return;
  }

  auto pcd_in_map_frame = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcd_in_map_frame->header.frame_id = mp_.global_frame;

  // Transform point cloud from camera frame to global_frame 
  pcl::transformPointCloud(*pcd, *pcd_in_map_frame, md_.cam_to_map);

  // Getting the Translation from the sensor to the Global Reference Frame
  const pcl::PointXYZ sensor_origin(
    md_.cam_to_map(0, 3), md_.cam_to_map(1, 3), md_.cam_to_map(2, 3));

  z_filter_cloud_in_.setInputCloud(pcd_in_map_frame);
  z_filter_cloud_in_.filter(*pcd_in_map_frame);

  // tm_bonxai_insert_.start();
  // Add point cloud to bonxai_maps_
  std::lock_guard<std::mutex> mtx_grd(bonxai_map_mtx_);
  bonxai_map_->insertPointCloud(pcd_in_map_frame->points, sensor_origin, mp_.max_range);
  // tm_bonxai_insert_.stop(false);
}

void OccMap::publishLocalMapBounds()
{
  geometry_msgs::msg::PolygonStamped local_map_poly;
  local_map_poly.header.frame_id = mp_.global_frame;
  local_map_poly.header.stamp = get_clock()->now();

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

rcl_interfaces::msg::SetParametersResult OccMap::dynamicParametersCB(const std::vector<rclcpp::Parameter> & parameters)
{
  std::string param_ns = "voxel_map";

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  logger_->logInfo("Updating parameters");

  bool update_bonxai{false};

  for (const rclcpp::Parameter & param : parameters)
  {
    /* Noise filter*/

    if (param.get_name() == "noise_search_radius")
    {
      if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
      {
        result.successful = false;
        result.reason = "noise_search_radius must be a double";
        break;
      }
      logger_->logInfo("Updated 'noise_search_radius' parameters");
      noise_search_radius_ = param.as_double();
    }

    if (param.get_name() == "noise_min_neighbors")
    {
      if (param.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER)
      {
        result.successful = false;
        result.reason = "noise_min_neighbors must be an integer";
        break;
      }
      logger_->logInfo("Updated 'noise_min_neighbors' parameters");
      noise_min_neighbors_ = param.as_int();
    }
    
    /* Occupancy mapping */

    if (param.get_name() == "occ_map.prob_miss_log")
    {
      if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
      {
        result.successful = false;
        result.reason = "occ_map.prob_miss_log must be a double";
        break;
      }
      logger_->logInfo("Updated 'occ_map.prob_miss_log' parameters");
      update_bonxai = true;
      bonxai_options_.prob_miss_log = Bonxai::ProbabilisticMap::logods(param.as_double());
    }
    
    if (param.get_name() == "occ_map.prob_hit_log")
    {
      if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
      {
        result.successful = false;
        result.reason = "occ_map.prob_hit_log must be a double";
        break;
      }
      logger_->logInfo("Updated 'occ_map.prob_hit_log' parameters");
      update_bonxai = true;
      bonxai_options_.prob_hit_log = Bonxai::ProbabilisticMap::logods(param.as_double());
    }
    
    if (param.get_name() == "occ_map.clamp_min_log")
    {
      if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
      {
        result.successful = false;
        result.reason = "occ_map.clamp_min_log must be a double";
        break;
      }
      logger_->logInfo("Updated 'occ_map.clamp_min_log' parameters");
      update_bonxai = true;
      bonxai_options_.clamp_min_log = Bonxai::ProbabilisticMap::logods(param.as_double());
    }
    
    if (param.get_name() == "occ_map.clamp_max_log")
    {
      if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
      {
        result.successful = false;
        result.reason = "occ_map.clamp_max_log must be a double";
        break;
      }
      logger_->logInfo("Updated 'occ_map.clamp_max_log' parameters");
      update_bonxai = true;
      bonxai_options_.clamp_max_log = Bonxai::ProbabilisticMap::logods(param.as_double());
    }
    
    if (param.get_name() == "occ_map.occupancy_threshold_log")
    {
      if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
      {
        result.successful = false;
        result.reason = "occ_map.occupancy_threshold_log must be a double";
        break;
      }
      logger_->logInfo("Updated 'occ_map.occupancy_threshold_log' parameters");
      update_bonxai = true;
      bonxai_options_.occupancy_threshold_log = Bonxai::ProbabilisticMap::logods(param.as_double());
    }

    /* Inflation */
    if (param.get_name() == "occ_map.static_inflation")
    {
      if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
      {
        result.successful = false;
        result.reason = "occ_map.static_inflation must be a double";
        break;
      }
      logger_->logInfo("Updated 'occ_map.static_inflation' parameters");
      mp_.static_inflation_ = param.as_double();
      mp_.inf_static_vox_ = std::ceil(mp_.static_inflation_/mp_.resolution_);
    }
  }

  if (update_bonxai){
    bonxai_map_->setOptions(bonxai_options_);
  }

  return result;
}

}  // namespace occ_map
