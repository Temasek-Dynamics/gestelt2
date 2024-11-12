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

#include <voronoi_planner/voronoi_planner.hpp>

namespace navigator
{

VoronoiPlanner::VoronoiPlanner()
: Node("voronoi_planner")
{
	logger_ = std::make_shared<logger_wrapper::LoggerWrapper>(this->get_logger(), this->get_clock());

  initParams();

	// Create callback groups
  planning_cb_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  mapping_cb_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  swarm_plan_cb_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::Reentrant);
  others_cb_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::Reentrant);

  initPubSubTimer();

  tm_front_end_plan_.updateID(drone_id_);
  tm_voro_map_init_.updateID(drone_id_);

  astar_params_.drone_id = drone_id_;
  astar_params_.max_iterations = 99999;
  astar_params_.debug_viz = true;
  astar_params_.tie_breaker = 1.001;
  astar_params_.cost_function_type  = 1; // 0: getOctileDist, 1: getL1Norm, 2: getL2Norm, 3: getChebyshevDist
  astar_params_.t_unit = t_unit_;
}

void VoronoiPlanner::init()
{
  tf_buffer_ =
    std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ =
    std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Get world to map fixed TF
  try {
    auto tf_world_to_map = tf_buffer_->lookupTransform(
      map_frame_, "world",
      tf2::TimePointZero,
      tf2_ros::fromRclcpp(rclcpp::Duration::from_seconds(5.0)));
    // Set fixed map origin
    map_origin_(0) = tf_world_to_map.transform.translation.x;
    map_origin_(1) = tf_world_to_map.transform.translation.y;
    
  } catch (const tf2::TransformException & ex) {
		RCLCPP_ERROR(
			this->get_logger(), "Could not get transform from world_frame('world') to map_frame_(%s): %s",
			map_frame_.c_str(), ex.what());
    rclcpp::shutdown();
    return;
  }

  // Initialize planner
  fe_planner_ = std::make_unique<global_planner::SpaceTimeAStar>(astar_params_, this->get_clock());

  // Initialize map
  voxel_map_ = std::make_unique<voxel_map::VoxelMap>(
    this->shared_from_this(), map_origin_, num_drones_);


  // Initialize minimum jerk optimizer
	min_jerk_opt_ = std::make_unique<minco::MinJerkOpt>();

  // Initialize visualization helper
  viz_helper_ = std::make_unique<viz_helper::VizHelper>(this->get_clock());

	logger_->logInfo("Initialized");
} 

VoronoiPlanner::~VoronoiPlanner()
{}

void VoronoiPlanner::initPubSubTimer()
{
  auto others_sub_opt = rclcpp::SubscriptionOptions();
  others_sub_opt.callback_group = others_cb_group_;

  auto swarm_plan_sub_opt = rclcpp::SubscriptionOptions();
  swarm_plan_sub_opt.callback_group = swarm_plan_cb_group_;

  /* Publishers */

  // Map publishers

  // TODO: Change to dynamic instead of hard-coded values
  for (int z_cm = 50; 
        z_cm <= 400; 
        z_cm += 50)
  {
    occ_map_pubs_[z_cm] = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "occ_map_" + std::to_string(z_cm), 10);
    voro_occ_grid_pubs_[z_cm] = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "voro_map_" + std::to_string(z_cm), 10);
  }

  voro_planning_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "voro_planning", 10);

  // voronoi_graph_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("voronoi_graph", 10);

  // Planner publishers
  fe_plan_broadcast_pub_ = this->create_publisher<gestelt_interfaces::msg::SpaceTimePath>(
    "fe_plan/broadcast", rclcpp::SensorDataQoS());

  poly_traj_pub_ = this->create_publisher<minco_interfaces::msg::PolynomialTrajectory>(
    "poly_traj", rclcpp::SensorDataQoS());
  minco_traj_broadcast_pub_ = this->create_publisher<minco_interfaces::msg::MincoTrajectory>(
    "minco_traj/broadcast", rclcpp::SensorDataQoS());

  // Visualization
  agent_id_text_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("agent_id_text", 10);
  plan_req_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("fe_plan_req", 10);
  fe_closed_list_viz_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
    "fe_plan/closed_list", 10);
  fe_plan_viz_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("fe_plan/viz", 10);

	minco_traj_viz_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("minco_traj_viz", 10);

  /* Subscribers */
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", rclcpp::SensorDataQoS(), std::bind(&VoronoiPlanner::odomSubCB, this, _1), others_sub_opt);

  // Subscribe to front end plan individually from each agent
  if (commless_){
    for (int i = 0; i < num_drones_; i++){

      std::function<void(const nav_msgs::msg::Odometry::UniquePtr msg)> bound_callback_func =
        std::bind(&VoronoiPlanner::swarmOdomCB, this, _1, i);

      swarm_odom_subs_.push_back(
        this->create_subscription<nav_msgs::msg::Odometry>(
          "/d"+ std::to_string(i) + "/" + "odom", 
          rclcpp::SensorDataQoS(), 
          bound_callback_func, 
          swarm_plan_sub_opt)
      );

    }
  }
  else {
    for (int i = 0; i < drone_id_; i++){
      fe_plan_broadcast_subs_.push_back(
        this->create_subscription<gestelt_interfaces::msg::SpaceTimePath>(
          "/d"+std::to_string(i)+"/fe_plan/broadcast", 
          rclcpp::SensorDataQoS(), 
          std::bind(&VoronoiPlanner::FEPlanSubCB, this, _1), 
          swarm_plan_sub_opt)
      );
    }
  }


  plan_req_dbg_sub_ = this->create_subscription<gestelt_interfaces::msg::PlanRequest>(
    "plan_request_dbg", rclcpp::SystemDefaultsQoS(), std::bind(&VoronoiPlanner::planReqDbgSubCB, this, _1));
  goals_sub_ = this->create_subscription<gestelt_interfaces::msg::Goals>(
    "goals", rclcpp::SystemDefaultsQoS(), std::bind(&VoronoiPlanner::goalsSubCB, this, _1));
  point_goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "point_goal", rclcpp::SystemDefaultsQoS(), std::bind(&VoronoiPlanner::pointGoalSubCB, this, _1));

  /* Timers */
	plan_fe_timer_ = this->create_wall_timer((1.0/fe_planner_freq_) *1000ms, 
                                            std::bind(&VoronoiPlanner::planFETimerCB, this), 
                                            planning_cb_group_);

	gen_voro_map_timer_ = this->create_wall_timer((1.0/gen_voro_map_freq_) *1000ms, 
                                                  std::bind(&VoronoiPlanner::genVoroMapTimerCB, this),
                                                  planning_cb_group_);
}

void VoronoiPlanner::initParams()
{
  std::string param_ns = "navigator";
  
  this->declare_parameter("drone_id", -1);

  this->declare_parameter(param_ns+".num_drones", 4);
  this->declare_parameter(param_ns+".communication_less", false);

  this->declare_parameter(param_ns+".planner_frequency", 10.0);
  this->declare_parameter(param_ns+".generate_voronoi_frequency", 10.0);

  this->declare_parameter(param_ns+".planner.goal_tolerance", 0.1);
  this->declare_parameter(param_ns+".planner.verbose_print", false);
  this->declare_parameter(param_ns+".planner.plan_once", false);
  this->declare_parameter(param_ns+".planner.t_unit", 0.1);
  this->declare_parameter(param_ns+".planner.output_json_filepath", "");

  this->declare_parameter(param_ns+".reservation_table.inflation", 0.3);
  this->declare_parameter(param_ns+".reservation_table.time_buffer", 0.5);
  this->declare_parameter(param_ns+".reservation_table.window_size", -1);

  this->declare_parameter(param_ns+".min_jerk_trajectory.front_end_stride", 3);

  this->declare_parameter("map_frame", "map");
  this->declare_parameter("local_map_frame", "local_map_frame");

	/**
	 * Get params
	 */

  drone_id_ = this->get_parameter("drone_id").as_int();

  num_drones_ = this->get_parameter(param_ns+".num_drones").as_int();
  commless_ = this->get_parameter(param_ns+".communication_less").as_bool();

  fe_planner_freq_ = this->get_parameter(param_ns+".planner_frequency").as_double();
  gen_voro_map_freq_ = this->get_parameter(param_ns+".generate_voronoi_frequency").as_double();

  double goal_tol = this->get_parameter(param_ns+".planner.goal_tolerance").as_double();
  sqr_goal_tol_ = goal_tol * goal_tol;
  verbose_print_ = this->get_parameter(param_ns+".planner.verbose_print").as_bool();
  plan_once_ = this->get_parameter(param_ns+".planner.plan_once").as_bool();
  t_unit_ = this->get_parameter(param_ns+".planner.t_unit").as_double();
  output_json_filepath_ = this->get_parameter(param_ns+".planner.output_json_filepath").as_string();

  fe_stride_ = this->get_parameter(param_ns+".min_jerk_trajectory.front_end_stride").as_int();

  rsvn_tbl_inflation_ = this->get_parameter(param_ns+".reservation_table.inflation").as_double();
  rsvn_tbl_window_size_ = this->get_parameter(param_ns+".reservation_table.window_size").as_int();
  double rsvn_tbl_t_buffer = this->get_parameter(param_ns+".reservation_table.time_buffer").as_double();
  t_buffer_ = (int) std::lround(rsvn_tbl_t_buffer/t_unit_);  // [space-time units] for time buffer

  map_frame_ = this->get_parameter("map_frame").as_string();
  local_map_frame_ = this->get_parameter("local_map_frame").as_string();
}

/* Core methods */
bool VoronoiPlanner::plan(const Eigen::Vector3d& goal_pos){
  if (commless_){
    return planWithoutComms(goal_pos);
  }
  else {
    return planWithComms(goal_pos);
  }
}

bool VoronoiPlanner::planWithoutComms(const Eigen::Vector3d& goal_pos){
  if (!init_voro_maps_){
    logger_->logInfo("Voronoi maps not initialized! Request a plan after initialization!");
    return false;
  }

  {
    std::lock_guard<std::mutex> rsvn_tbl_guard(rsvn_tbl_mtx_);
    fe_planner_->setReservationTable(rsvn_tbl_);
  }

  std::lock_guard<std::mutex> voro_map_guard(voro_map_mtx_);

  // Assign voronoi map
  voro_params_.z_sep_cm = bool_map_3d_.z_sep_cm;
  voro_params_.local_origin_x = bool_map_3d_.origin(0);
  voro_params_.local_origin_y = bool_map_3d_.origin(1);
  voro_params_.max_height_cm = bool_map_3d_.max_height_cm;
  voro_params_.min_height_cm = bool_map_3d_.min_height_cm;
  voro_params_.res = bool_map_3d_.resolution;

  fe_planner_->setVoroMap(dyn_voro_arr_, voro_params_);

  Eigen::Vector3d start_pos, start_vel, start_acc;

  auto plan_start_clock = this->get_clock()->now();

  if (!sampleTrajectory(poly_traj_, plan_start_clock.seconds(), start_pos, start_vel, start_acc))
  {
    start_pos = cur_pos_;
    start_vel = cur_vel_;
    start_acc.setZero();
  }

  // Get RHP goal
  // Eigen::Vector3d rhp_goal_pos = goal_pos;
  Eigen::Vector3d rhp_goal_pos = getRHPGoal(
    start_pos, goal_pos, 
    voxel_map_->getLocalMapOrigin(0.15), voxel_map_->getLocalMapMax(0.15));

  viz_helper_->pubPlanRequestViz(start_pos, rhp_goal_pos, goal_pos, plan_req_pub_, map_frame_);

  tm_front_end_plan_.start();

  if (!fe_planner_->generatePlan(mapToLclMap(start_pos), mapToLclMap(rhp_goal_pos)))
  {
    logger_->logError(strFmt("Drone %d: Failed to generate FE plan from (%f, %f, %f) to (%f, %f, %f) with closed_list of size %ld", 
                              drone_id_, start_pos(0), start_pos(1), start_pos(2), 
                              rhp_goal_pos(0), rhp_goal_pos(1), rhp_goal_pos(2),
                              fe_planner_->getClosedList().size()));

    // viz_helper_->pubFrontEndClosedList(fe_planner_->getClosedList(), 
    //                   fe_closed_list_viz_pub_, map_frame_);

    tm_front_end_plan_.stop(verbose_print_);
    return false;
  }

  tm_front_end_plan_.stop(verbose_print_);

  // Visualize modified voronoi diagram

  nav_msgs::msg::OccupancyGrid voro_pln_map;

  voronoimapToOccGrid(*fe_planner_->getDynVoro(), 
                  bool_map_3d_.origin(0), bool_map_3d_.origin(1), 
                  voro_pln_map); // Occupancy map

  voro_planning_pub_->publish(voro_pln_map);


  // Retrieve space time path 
  std::vector<Eigen::Vector4d> lcl_map_path;
  // Current pose must converted from fixed map frame to local map frame before passing to planner
  // lcl_map_path = fe_planner_->getPathWithTime(mapToLclMap(cur_pos_));
  lcl_map_path = fe_planner_->getPathWithTime();

  auto lclMapToMapPath = [&](const std::vector<Eigen::Vector4d>& lcl_map_path, 
                             std::vector<Eigen::Vector4d>& map_path_w_t,
                             std::vector<Eigen::Vector3d>& map_path) {
    map_path.clear();
    map_path_w_t.clear();
    
    for (size_t i = 0; i < lcl_map_path.size(); i++){
      Eigen::Vector4d map_pt;
      map_pt.head<3>() = lclMapToMap(lcl_map_path[i].head<3>());
      map_pt(3) = lcl_map_path[i](3);

      map_path.push_back(map_pt.head<3>());
      map_path_w_t.push_back(map_pt);
    }
  };

  // Convert front-end path from local map frame to fixed map frame
  lclMapToMapPath(lcl_map_path, fe_path_with_t_, fe_path_);
  // Visualize front-end path
  viz_helper_->pubFrontEndPath(fe_path_, fe_plan_viz_pub_, map_frame_);

  // Convert from space time path to gestelt_interfaces::msg::SpaceTimePath
  gestelt_interfaces::msg::SpaceTimePath fe_plan_msg;

  fe_plan_msg.agent_id = drone_id_;
  fe_plan_msg.header.stamp = this->get_clock()->now();
  fe_plan_msg.t_plan_start = plan_start_clock.seconds();

  for (size_t i = 0; i < fe_path_with_t_.size(); i++){
    geometry_msgs::msg::Pose pose;
    pose.position.x = fe_path_with_t_[i](0); 
    pose.position.y = fe_path_with_t_[i](1);
    pose.position.z = fe_path_with_t_[i](2);
    pose.orientation.w = 1.0; 

    fe_plan_msg.plan.push_back(pose);
    fe_plan_msg.plan_time.push_back(int(fe_path_with_t_[i](3)));
  }

  fe_plan_broadcast_pub_->publish(fe_plan_msg);

  std::vector<Eigen::Vector4d> mjo_fe_path;
  // Sample at regular intervals from front-end path
  for (size_t i = 0; i < fe_path_with_t_.size(); i += fe_stride_)
	{	
		mjo_fe_path.push_back(Eigen::Vector4d{
			fe_path_with_t_[i](0),
			fe_path_with_t_[i](1),
			fe_path_with_t_[i](2),
			fe_path_with_t_[i](3)
		});
	}

  if (mjo_fe_path.size() >= 2){
    /* Generate minimum jerk trajectory */

    auto genMinJerkTraj = [&](std::unique_ptr<minco::MinJerkOpt>& mjo,
                              const std::vector<Eigen::Vector4d>& space_time_path,
                              const Eigen::Matrix3d& start_PVA,
                              const Eigen::Matrix3d& goal_PVA,
                              const double& t_plan_start){
      
      Eigen::MatrixXd inner_pts(3, space_time_path.size()-2);
      Eigen::VectorXd seg_durations(space_time_path.size()-1);

      for (size_t i = 1, j = 0; i < space_time_path.size()-1; i++, j++){
        inner_pts.col(j) = space_time_path[i].head<3>();
      }

      for (size_t i = 1, j = 0; i < space_time_path.size(); i++, j++){
        seg_durations(j) = double(space_time_path[i](3) - space_time_path[j](3)) * t_unit_;
      }

      mjo->generate(start_PVA, goal_PVA, inner_pts, seg_durations);

      return mjo->getTraj(t_plan_start);
    };

    Eigen::Matrix3d start_PVA, goal_PVA;
    start_PVA << start_pos, start_vel, start_acc;
    goal_PVA << mjo_fe_path.back().head<3>(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();

    poly_traj_ = genMinJerkTraj(min_jerk_opt_, mjo_fe_path, start_PVA, goal_PVA, plan_start_clock.seconds());

    Eigen::MatrixXd mjo_cstr_pts = min_jerk_opt_->getConstraintPts(5);

    minco_interfaces::msg::PolynomialTrajectory poly_msg; 
    minco_interfaces::msg::MincoTrajectory MINCO_msg; 

    polyTrajToMincoMsg(poly_traj_, plan_start_clock.seconds(), poly_msg, MINCO_msg);

    poly_traj_pub_->publish(poly_msg); // [map frame] Publish to corresponding drone for execution
    // TODO: convert MINCO_msg to world frame
    // minco_traj_broadcast_pub_->publish(MINCO_msg); // [map frame] Broadcast to all other drones
	  
    // Publish visualization
    viz_helper::VizHelper::pubExecTraj(mjo_cstr_pts, minco_traj_viz_pub_, map_frame_);
  }

  plan_complete_ = true;

  return true;

}

bool VoronoiPlanner::planWithComms(const Eigen::Vector3d& goal_pos){
  // if (!init_voro_maps_){
  //   logger_->logInfo("Voronoi maps not initialized! Request a plan after initialization!");
  //   return false;
  // }

  // if (plan_once_ && plan_complete_){
  //   return true;
  // }

  // // /* Lambda for checking if all higher priority plans are received*/
  // // auto isAllPrioPlansRcv = [&] () {
  // //   // Only check from 0 to current drone_id
  // //   for (int i = 0; i < drone_id_; i++){ 
  // //     if (rsvn_tbl_.find(i) == rsvn_tbl_.end()){
  // //       return false;
  // //     }
  // //   }
  // //   return true;
  // // };

  // // rclcpp::Rate loop_rate(10);
  // // while (!isAllPrioPlansRcv()){
  // //   loop_rate.sleep();
  // // }

  // // Update reservation table on planner
  // {
  //   std::lock_guard<std::mutex> rsvn_tbl_guard(rsvn_tbl_mtx_);
  //   fe_planner_->setReservationTable(rsvn_tbl_);
  //   // rsvn_tbl_.clear();
  // }

  // std::lock_guard<std::mutex> voro_map_guard(voro_map_mtx_);

  // // Assign voronoi map
  // voro_params_.z_sep_cm = bool_map_3d_.z_sep_cm;
  // voro_params_.local_origin_x = bool_map_3d_.origin(0);
  // voro_params_.local_origin_y = bool_map_3d_.origin(1);
  // voro_params_.max_height_cm = bool_map_3d_.max_height_cm;
  // voro_params_.min_height_cm = bool_map_3d_.min_height_cm;
  // voro_params_.res = bool_map_3d_.resolution;

  // fe_planner_->setVoroMap(dyn_voro_arr_, 
  //                         voro_params_);

  // Eigen::Vector3d start_pos, start_vel, start_acc;

  // // Eigen::Vector3d rhp_goal_pos = goal_pos;

  // auto plan_start_clock = this->get_clock()->now();

  // if (!sampleTrajectory(poly_traj_, plan_start_clock.seconds(), start_pos, start_vel, start_acc))
  // {
  //   start_pos = cur_pos_;
  //   start_vel = cur_vel_;
  //   start_acc.setZero();
  // }

  // // Get RHP goal
  // Eigen::Vector3d rhp_goal_pos = getRHPGoal(
  //   start_pos, goal_pos, 
  //   voxel_map_->getLocalMapOrigin(0.2), voxel_map_->getLocalMapMax(0.2));

  // // logger_->logError(strFmt("Drone %d: start_pos(%f, %f, %f): des_goal(%f, %f, %f) to rhp_goal(%f, %f, %f)", 
  // //                           drone_id_, 
  // //                           start_pos(0), start_pos(1), start_pos(2), 
  // //                           goal_pos(0), goal_pos(1), goal_pos(2), 
  // //                           rhp_goal_pos(0), rhp_goal_pos(1), rhp_goal_pos(2)));

  // viz_helper_->pubPlanRequestViz(start_pos, rhp_goal_pos, goal_pos, plan_req_pub_, map_frame_);

  // tm_front_end_plan_.start();

  // if (!fe_planner_->generatePlan(mapToLclMap(start_pos), mapToLclMap(rhp_goal_pos)))
  // {
  //   logger_->logError(strFmt("Drone %d: Failed to generate FE plan from (%f, %f, %f) to (%f, %f, %f) with closed_list of size %ld", 
  //                             drone_id_, start_pos(0), start_pos(1), start_pos(2), 
  //                             rhp_goal_pos(0), rhp_goal_pos(1), rhp_goal_pos(2),
  //                             fe_planner_->getClosedList().size()));

  //   tm_front_end_plan_.stop(verbose_print_);

  //   // viz_helper_->pubFrontEndClosedList(fe_planner_->getClosedList(), 
  //   //                   fe_closed_list_viz_pub_, map_frame_);

  //   return false;
  // }

  // tm_front_end_plan_.stop(verbose_print_);

  // // Retrieve space time path 
  // std::vector<Eigen::Vector4d> lcl_map_path;
  // // Current pose must converted from fixed map frame to local map frame before passing to planner
  // // lcl_map_path = fe_planner_->getPathWithTime(mapToLclMap(cur_pos_));
  // lcl_map_path = fe_planner_->getPathWithTime();

  // auto lclMapToMapPath = [&](const std::vector<Eigen::Vector4d>& lcl_map_path, 
  //                            std::vector<Eigen::Vector4d>& map_path_w_t,
  //                            std::vector<Eigen::Vector3d>& map_path) {
  //   map_path.clear();
  //   map_path_w_t.clear();
    
  //   for (size_t i = 0; i < lcl_map_path.size(); i++){
  //     Eigen::Vector4d map_pt;
  //     map_pt.head<3>() = lclMapToMap(lcl_map_path[i].head<3>());
  //     map_pt(3) = lcl_map_path[i](3);

  //     map_path.push_back(map_pt.head<3>());
  //     map_path_w_t.push_back(map_pt);
  //   }
  // };

  // // Convert front-end path from local map frame to fixed map frame
  // lclMapToMapPath(lcl_map_path, fe_path_with_t_, fe_path_);

  // // Visualize front-end path
  // // viz_helper_->pubSpaceTimePath(fe_path_with_t_, fe_plan_viz_pub_, map_frame_) ;
  // viz_helper_->pubFrontEndPath(fe_path_, fe_plan_viz_pub_, map_frame_);

  // // Convert from space time path to gestelt_interfaces::msg::SpaceTimePath
  // gestelt_interfaces::msg::SpaceTimePath fe_plan_msg;

  // fe_plan_msg.agent_id = drone_id_;
  // fe_plan_msg.header.stamp = this->get_clock()->now();
  // fe_plan_msg.t_plan_start = plan_start_clock.seconds();

  // for (size_t i = 0; i < fe_path_with_t_.size(); i++){
  //   geometry_msgs::msg::Pose pose;
  //   pose.position.x = fe_path_with_t_[i](0); 
  //   pose.position.y = fe_path_with_t_[i](1);
  //   pose.position.z = fe_path_with_t_[i](2);
  //   pose.orientation.w = 1.0; 

  //   fe_plan_msg.plan.push_back(pose);
  //   fe_plan_msg.plan_time.push_back(int(fe_path_with_t_[i](3)));
  // }

  // fe_plan_broadcast_pub_->publish(fe_plan_msg);

  // std::vector<Eigen::Vector4d> mjo_fe_path;
  // // Sample at regular intervals from front-end path
  // for (size_t i = 0; i < fe_path_with_t_.size(); i += fe_stride_)
	// {	
	// 	mjo_fe_path.push_back(Eigen::Vector4d{
	// 		fe_path_with_t_[i](0),
	// 		fe_path_with_t_[i](1),
	// 		fe_path_with_t_[i](2),
	// 		fe_path_with_t_[i](3)
	// 	});
	// }

  // if (mjo_fe_path.size() >= 2){
  //   /* Generate minimum jerk trajectory */


  //   auto genMinJerkTraj = [&](std::unique_ptr<minco::MinJerkOpt>& mjo,
  //                             const std::vector<Eigen::Vector4d>& space_time_path,
  //                             const Eigen::Matrix3d& start_PVA,
  //                             const Eigen::Matrix3d& goal_PVA,
  //                             const double& t_plan_start){
      
  //     Eigen::MatrixXd inner_pts(3, space_time_path.size()-2);
  //     Eigen::VectorXd seg_durations(space_time_path.size()-1);

  //     for (size_t i = 1, j = 0; i < space_time_path.size()-1; i++, j++){
  //       inner_pts.col(j) = space_time_path[i].head<3>();
  //     }

  //     for (size_t i = 1, j = 0; i < space_time_path.size(); i++, j++){
  //       seg_durations(j) = double(space_time_path[i](3) - space_time_path[j](3)) * t_unit_;
  //     }

  //     mjo->generate(start_PVA, goal_PVA, inner_pts, seg_durations);

  //     return mjo->getTraj(t_plan_start);
  //   };

  //   Eigen::Matrix3d start_PVA, goal_PVA;
  //   start_PVA << start_pos, start_vel, start_acc;
  //   goal_PVA << mjo_fe_path.back().head<3>(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();

  //   poly_traj_ = genMinJerkTraj(min_jerk_opt_, mjo_fe_path, start_PVA, goal_PVA, plan_start_clock.seconds());

  //   Eigen::MatrixXd mjo_cstr_pts = min_jerk_opt_->getConstraintPts(5);

  //   minco_interfaces::msg::PolynomialTrajectory poly_msg; 
  //   minco_interfaces::msg::MincoTrajectory MINCO_msg; 

  //   polyTrajToMincoMsg(poly_traj_, plan_start_clock.seconds(), poly_msg, MINCO_msg);

  //   poly_traj_pub_->publish(poly_msg); // [map frame] Publish to corresponding drone for execution
  //   // minco_traj_broadcast_pub_->publish(MINCO_msg); // [map frame] Broadcast to all other drones
	  
  //   // Publish visualization
  //   viz_helper::VizHelper::pubExecTraj(mjo_cstr_pts, minco_traj_viz_pub_, map_frame_);
  // }



  // // if (json_output_)
  // // {
  // //   json path_json;
  // //   path_json["map"] = "map0";

  // //   for (auto& pt : fe_path_with_t_)
  // //   {
	// // 			path_json["a_star_path"].push_back({
	// // 				{"point", {pt(0), pt(1), pt(2)}},
	// // 				{"time", {pt(3)}},
	// // 			});
  // //   }

  // //   std::ofstream o(output_json_filepath_.c_str());
  // //   o << std::setw(4) << path_json << std::endl;

  // //   printf("Saved path json to path: %s\n",  output_json_filepath_.c_str());
  // // }

  // // if (verbose_print_)
  // // {
  // //   logger_->logError(strFmt("Generated FE plan from (%f, %f, %f) to (%f, %f, %f)", 
  // //                             drone_id_, fe_path_[0](0), fe_path_[0](1), fe_path_[0](2), 
  // //                             fe_path_.back()(0), fe_path_.back()(1), fe_path_.back()(2)));
  // // }

  // plan_complete_ = true;

  return true;
}

/* Timer callbacks*/
void VoronoiPlanner::planFETimerCB()
{
  if (!init_voro_maps_){
    return;
  }

  // Check if waypoint queue is empty
  if (waypoints_.empty()){
    auto plan_start_clock = this->get_clock()->now();

    // Publish trajectory representing the drone staying in
    //  it's current position
    gestelt_interfaces::msg::SpaceTimePath fe_plan_msg;

    fe_plan_msg.agent_id = drone_id_;
    fe_plan_msg.header.stamp = plan_start_clock;
    fe_plan_msg.t_plan_start = plan_start_clock.seconds();

    for (size_t i = 0; i < (size_t) rsvn_tbl_window_size_; i++){
      geometry_msgs::msg::Pose pose;
      pose.position.x = cur_pos_(0); 
      pose.position.y = cur_pos_(1);
      pose.position.z = cur_pos_(2);
      pose.orientation.w = 1.0; 

      fe_plan_msg.plan.push_back(pose);
      fe_plan_msg.plan_time.push_back(i);
    }

    fe_plan_broadcast_pub_->publish(fe_plan_msg);

    return;
  }

  if (isGoalReached(cur_pos_, waypoints_.nextWP())){
    logger_->logInfo("Reached goal!");
    
    // If goals is within a given tolerance, then pop this goal and plan next goal (if available)
    waypoints_.popWP();

    return;
  }

  // Plan from current position to next waypoint
  if (!plan(waypoints_.nextWP())){

    // auto plan_start_clock = this->get_clock()->now();

    // // If previous trajectory is still valid, follow previous path
    // if (poly_traj_ != nullptr && poly_traj_->getGlobalStartTime() > 0.0){

    //   double e_t_start = plan_start_clock.seconds() - poly_traj_->getGlobalStartTime(); // Get time t relative to start of trajectory

    //   if (e_t_start > 0.0 && e_t_start < poly_traj_->getTotalDuration())
    //   {
    //     // Trajectory is still executing
    //     logger_->logWarn("Following previous trajectory!");
    //     return;
    //   }
    // }

    // // Else, generate a trajectory representing the drone staying in
    // //  it's current position

    // gestelt_interfaces::msg::SpaceTimePath fe_plan_msg;

    // fe_plan_msg.agent_id = drone_id_;
    // fe_plan_msg.header.stamp = plan_start_clock;
    // fe_plan_msg.t_plan_start = plan_start_clock.seconds();

    // for (size_t i = 0; i < (size_t) rsvn_tbl_window_size_; i++){
    //   geometry_msgs::msg::Pose pose;
    //   pose.position.x = cur_pos_(0); 
    //   pose.position.y = cur_pos_(1);
    //   pose.position.z = cur_pos_(2);
    //   pose.orientation.w = 1.0; 

    //   fe_plan_msg.plan.push_back(pose);
    //   fe_plan_msg.plan_time.push_back(i);
    // }

    // logger_->logWarn("Staying at current position!");
    // fe_plan_broadcast_pub_->publish(fe_plan_msg);
  }
}

void VoronoiPlanner::genVoroMapTimerCB()
{
  if (!voxel_map_->getBoolMap3D(bool_map_3d_)){
    return;
  }

  tm_voro_map_init_.start();

  std::lock_guard<std::mutex> voro_map_guard(voro_map_mtx_);

  // std::vector<Eigen::Vector3d> voro_verts;

  for (auto const& bool_map : bool_map_3d_.bool_maps) // For each boolean map
  {
    int z_cm = bool_map.first;
    double z_m = cmToM(bool_map.first);

    // Create DynamicVoronoi object if it does not exist
    dynamic_voronoi::DynamicVoronoiParams dyn_voro_params;
    dyn_voro_params.res = bool_map_3d_.resolution;
    // dyn_voro_params.origin_x = 0.0;
    // dyn_voro_params.origin_y = 0.0;
    dyn_voro_params.origin_z = z_m;
    dyn_voro_params.origin_z_cm = z_cm;

    // Initialize dynamic voronoi 
    dyn_voro_arr_[z_cm].reset();
    dyn_voro_arr_[z_cm] = std::make_shared<dynamic_voronoi::DynamicVoronoi>();
    dyn_voro_arr_[z_cm]->setParams(dyn_voro_params), 
    // Map is received in local_map_frame_
    dyn_voro_arr_[z_cm]->initializeMap(bool_map_3d_.width, 
                                      bool_map_3d_.height, 
                                      bool_map.second);

    dyn_voro_arr_[z_cm]->update(); // update distance map and Voronoi diagram
    dyn_voro_arr_[z_cm]->prune();  // prune the Voronoi

    nav_msgs::msg::OccupancyGrid occ_grid, voro_occ_grid;

    occmapToOccGrid(*dyn_voro_arr_[z_cm], 
                    bool_map_3d_.origin(0), bool_map_3d_.origin(1), 
                    occ_grid); // Occupancy map

    voronoimapToOccGrid(*dyn_voro_arr_[z_cm], 
                        bool_map_3d_.origin(0), bool_map_3d_.origin(1), 
                        voro_occ_grid); // Voronoi map

    voro_occ_grid_pubs_[z_cm]->publish(voro_occ_grid);
    occ_map_pubs_[z_cm]->publish(occ_grid);

    // // Get all voronoi vertices (voronoi cells that have at least 3 voronoi neighbours)
    // auto getVoronoiVertices = [&](std::shared_ptr<dynamic_voronoi::DynamicVoronoi> dyn_voro){
    //   std::vector<Eigen::Vector3d> voronoi_vertices;
    //   if (dyn_voro->getData() != nullptr){  
    //     for (int x=0; x < dyn_voro->getSizeX(); x++) {
    //       for (int y=0; y < dyn_voro->getSizeY(); y++) {
    //         if (dyn_voro->isVoronoiVertex(x, y)){
    //           voronoi_vertices.push_back(Eigen::Vector3d{ x * dyn_voro->getRes(), 
    //                                                       y * dyn_voro->getRes(), 
    //                                                       dyn_voro->getOriginZ()});
    //         }
    //       }
    //     }
    //   }

    //   return voronoi_vertices;
    // };
    // voro_verts_cur_layer: voronoi vertices at current layer
    // std::vector<Eigen::Vector3d> voro_verts_cur_layer = getVoronoiVertices(dyn_voro_arr_[z_cm]);
    // voro_verts.insert(voro_verts.end(), voro_verts_cur_layer.begin(), voro_verts_cur_layer.end());
  }

  tm_voro_map_init_.stop(false);

  // viz_helper_->pubVoroVertices(voro_verts, voronoi_graph_pub_, local_map_frame_);
  init_voro_maps_ = true; // Flag to indicate that all voronoi maps have been initialized
}

/* Subscriber callbacks*/

void VoronoiPlanner::swarmOdomCB(const nav_msgs::msg::Odometry::UniquePtr& msg, int id)
{
  //TODO: Odom is received in the drone's map frame but we have yet to handle if the drones' map frame is not in 'world' frame
  if (msg->header.frame_id != "world"){
    logger_->logError("Does not yet handle odom messages with frame_id != 'world'");
  }

  /**
   * Update swarm poses and velocities
   */

  Eigen::Vector3d pose = Eigen::Vector3d{msg->pose.pose.position.x,  
                          msg->pose.pose.position.y,  
                          msg->pose.pose.position.z};

  Eigen::Vector3d vel = Eigen::Vector3d{msg->twist.twist.linear.x, 
                        msg->twist.twist.linear.y, 
                        msg->twist.twist.linear.z};

  voxel_map_->updateSwarmState(id, pose, vel);

  if (id == drone_id_){
    return;
  }

  if (!init_voro_maps_){
    return;
  }

  /**
   * Clear reservation table and add to it
   */
  std::lock_guard<std::mutex> rsvn_tbl_guard(rsvn_tbl_mtx_);

  double t_plan_start = ((double) msg->header.stamp.nanosec) * 1e-9;

  rsvn_tbl_[id] = RsvnTbl(t_plan_start);
  cells_inf_ = (int) std::lround(rsvn_tbl_inflation_/bool_map_3d_.resolution); // Number of cells used for inflation

  Eigen::Vector3d lcl_map_start_pos = mapToLclMap(pose);
  int z_cm =roundToMultInt(mToCm(lcl_map_start_pos(2)), 
                                bool_map_3d_.z_sep_cm,
                                bool_map_3d_.min_height_cm,
                                bool_map_3d_.max_height_cm);

  // Iterate for a short duration along velocity vector
  for (double t = 0; t <= 1.0; t += 0.05 )
  {
    Eigen::Vector3d lcl_map_pos = lcl_map_start_pos + t * vel;

    IntPoint grid_pos;
    // get map position relative to local origin and set the obstacle 
    {
      std::lock_guard<std::mutex> voro_map_guard(voro_map_mtx_);

      bool within_lcl_map = dyn_voro_arr_[z_cm]->posToIdx(
        DblPoint(lcl_map_pos(0), lcl_map_pos(1)), grid_pos);
      if (!within_lcl_map){ // skip current point if not in map
        continue;
      }
    }

    // Inflate the cells by the given inflation radius
    for(int x = grid_pos.x - cells_inf_; x <= grid_pos.x + cells_inf_; x++)
    {
      for(int y = grid_pos.y - cells_inf_; y <= grid_pos.y + cells_inf_; y++)
      {
        // Reserve for time interval from previous t to current t, including time buffer
        for (int j = t - t_buffer_; j <= t + t_buffer_; j++) { 
          rsvn_tbl_[id].table.insert(Eigen::Vector4i{x, y, z_cm, j});
        }
      }
    }
  }

}

void VoronoiPlanner::odomSubCB(const nav_msgs::msg::Odometry::UniquePtr& msg)
{
  cur_pos_= Eigen::Vector3d{msg->pose.pose.position.x, 
                            msg->pose.pose.position.y, 
                            msg->pose.pose.position.z};
  cur_vel_= Eigen::Vector3d{msg->twist.twist.linear.x, 
                            msg->twist.twist.linear.y, 
                            msg->twist.twist.linear.z};

  viz_helper_->pubText(cur_pos_, agent_id_text_pub_, drone_id_, map_frame_);
}

void VoronoiPlanner::FEPlanSubCB(const gestelt_interfaces::msg::SpaceTimePath::UniquePtr msg)
{
  // // PRIORITY-BASED PLANNING: Only consider trajectories of drones with lower id
  // // if (msg->agent_id >= drone_id_ ){
  // //   return;
  // // }

  // if (!init_voro_maps_){
  //   return;
  // }

  // // logger_->logInfo(strFmt("Received plan from agent %d", msg->agent_id));

  // std::lock_guard<std::mutex> rsvn_tbl_guard(rsvn_tbl_mtx_);

  // // Clear reservation table
  // rsvn_tbl_[msg->agent_id] = RsvnTbl(msg->t_plan_start);
  
  // // Add all points on path (with inflation) to reservation table
  // cells_inf_ = (int) std::lround(rsvn_tbl_inflation_/bool_map_3d_.resolution); // Number of cells used for inflation

  // int window_size = (rsvn_tbl_window_size_ > 0) ? std::min(rsvn_tbl_window_size_, (int) msg->plan.size()) : msg->plan.size() ; 

  // int prev_t = 0; // prev_t: Relative time of last points
  // for (size_t i = 0; i < (size_t) window_size; i++){ // For points on plan up to window size
  //   IntPoint grid_pos;
  //   // get map position relative to local origin
  //   DblPoint map_2d_pos(msg->plan[i].position.x - bool_map_3d_.origin(0), 
  //                       msg->plan[i].position.y - bool_map_3d_.origin(1));
  //   int map_z_cm =roundToMultInt(mToCm(msg->plan[i].position.z), 
  //                                 bool_map_3d_.z_sep_cm,
  //                                 bool_map_3d_.min_height_cm,
  //                                 bool_map_3d_.max_height_cm);
  //   {
  //     // std::lock_guard<std::mutex> voro_map_guard(voro_map_mtx_);
  //     if (!dyn_voro_arr_[map_z_cm]->posToIdx(map_2d_pos, grid_pos)){ // skip current point if not in map
  //       continue;
  //     }
  //   }

  //   // Inflate the cells by the given inflation radius
  //   for(int x = grid_pos.x - cells_inf_; x <= grid_pos.x + cells_inf_; x++)
  //   {
  //     for(int y = grid_pos.y - cells_inf_; y <= grid_pos.y + cells_inf_; y++)
  //     {
  //       // Reserve for time interval from previous t to current t, including time buffer
  //       for (int j = prev_t - t_buffer_; j < msg->plan_time[i] + t_buffer_; j++) { 
  //         rsvn_tbl_[msg->agent_id].table.insert(Eigen::Vector4i{ x, y, map_z_cm, j});
  //       }
  //     }
  //   }

  //   prev_t = msg->plan_time[i]; 
  // } 
}

void VoronoiPlanner::pointGoalSubCB(const geometry_msgs::msg::PoseStamped::UniquePtr msg)
{
    std::vector<Eigen::Vector3d> wp_vec;

    if (msg->header.frame_id == "world")
    {
      // Transform from world to fixed map frame
      wp_vec.push_back(worldToMap(Eigen::Vector3d(
        msg->pose.position.x, msg->pose.position.y, 1.5)));
    }
    else if (msg->header.frame_id == map_frame_)
    {
      // Keep in map frame
      wp_vec.push_back(Eigen::Vector3d(
        msg->pose.position.x, msg->pose.position.y, 1.5));
    }
    else {
      logger_->logError(strFmt("Only accepting goals in 'world' or '%s' frame, ignoring goals.", map_frame_));
      return;
    }

    waypoints_.addMultipleWP(wp_vec);
}

void VoronoiPlanner::goalsSubCB(const gestelt_interfaces::msg::Goals::UniquePtr msg)
{
    if (msg->waypoints.empty())
    {
      logger_->logError(strFmt("Received empty waypoints. Ignoring waypoints."));

      return;
    }

    std::vector<Eigen::Vector3d> wp_vec;

    if (msg->header.frame_id == "world")
    {
      // Transform from world to fixed map frame
      for (auto& wp : msg->waypoints) {
        wp_vec.push_back(worldToMap(Eigen::Vector3d(
          wp.position.x, wp.position.y, wp.position.z)));
      }
    }
    else if (msg->header.frame_id == map_frame_)
    {
      // Keep in map frame
      for (auto& wp : msg->waypoints) {
        wp_vec.push_back(Eigen::Vector3d(
          wp.position.x, wp.position.y, wp.position.z));
      }
    }
    else {
      logger_->logError(strFmt("Only accepting goals in 'world' or '%s' frame, ignoring goals.", map_frame_));
      return;
    }


    waypoints_.addMultipleWP(wp_vec);
}

void VoronoiPlanner::planReqDbgSubCB(const gestelt_interfaces::msg::PlanRequest::UniquePtr msg)
{
  // Transform from map to local map
  // Eigen::Vector3d plan_start = mapToLclMap(
  //   Eigen::Vector3d(msg->start.position.x, msg->start.position.y, msg->start.position.z));
  // Eigen::Vector3d plan_end = mapToLclMap(
  //   Eigen::Vector3d(msg->goal.position.x, msg->goal.position.y, msg->goal.position.z));

  Eigen::Vector3d plan_end;

  if (msg->header.frame_id == "world")
  {
    cur_pos_ = worldToMap(Eigen::Vector3d(
      msg->start.position.x, msg->start.position.y, msg->start.position.z));
    plan_end = worldToMap(Eigen::Vector3d(
      msg->goal.position.x, msg->goal.position.y, msg->goal.position.z));
  }
  else if (msg->header.frame_id == map_frame_)
  {
    // Keep in map frame
    cur_pos_ = Eigen::Vector3d(
      msg->start.position.x, msg->start.position.y, msg->start.position.z);
    plan_end = Eigen::Vector3d(
      msg->goal.position.x, msg->goal.position.y, msg->goal.position.z);
  }
  else {
    logger_->logError(strFmt("Only accepting goals in 'world' or '%s' frame, ignoring goals.", map_frame_));
    return;
  }

  std::cout << "Agent " << msg->agent_id << ": plan request from ("<< 
                cur_pos_.transpose() << ") to (" << plan_end.transpose() << ")" << std::endl;

  plan(plan_end);
}

} // namespace navigator

