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
  others_cb_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::Reentrant);

  initPubSubTimer();

  tm_front_end_plan_.updateID(drone_id_);
  tm_voro_map_init_.updateID(drone_id_);

  astar_params_.drone_id = drone_id_;
  astar_params_.max_iterations = 9999;
  astar_params_.debug_viz = true;
  astar_params_.tie_breaker = 1.001;
  astar_params_.cost_function_type  = 1; // 0: getOctileDist, 1: getL1Norm, 2: getL2Norm, 3: getChebyshevDist
  astar_params_.t_unit = t_unit_;

  // Initialize visualization helper
  viz_helper_ = std::make_shared<VizHelper>(this->get_clock());
}

void VoronoiPlanner::init()
{
  // Initialize planner
  fe_planner_ = std::make_unique<global_planner::SpaceTimeAStar>(astar_params_, this->get_clock());

  // Initialize map
  voxel_map_ = std::make_shared<voxel_map::VoxelMap>(this->shared_from_this());

	logger_->logInfo("Initialized");
} 

VoronoiPlanner::~VoronoiPlanner()
{}

void VoronoiPlanner::initPubSubTimer(){
  auto others_sub_opt = rclcpp::SubscriptionOptions();
  others_sub_opt.callback_group = others_cb_group_;

  /* Publishers */

  // Map publishers
  occ_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("voro/occ_map", 10);
  voro_occ_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("voro/voro_map", 10);
  voronoi_graph_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("voronoi_graph", 10);

  // Planner publishers
  fe_plan_pub_ = this->create_publisher<gestelt_interfaces::msg::SpaceTimePath>("fe_plan", 10);
  fe_plan_broadcast_pub_ = this->create_publisher<gestelt_interfaces::msg::SpaceTimePath>("/fe_plan/broadcast", rclcpp::SensorDataQoS());

  // Visualization
  start_pt_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("fe_start", 10);
  goal_pt_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("fe_goal", 10);
  fe_closed_list_viz_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("fe_plan/closed_list", 10);
  fe_plan_viz_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("fe_plan/viz", 10);

  /* Subscribers */
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", rclcpp::SensorDataQoS(), std::bind(&VoronoiPlanner::odomSubCB, this, _1), others_sub_opt);

  fe_plan_broadcast_sub_ = this->create_subscription<gestelt_interfaces::msg::SpaceTimePath>(
    "/fe_plan/broadcast", rclcpp::SensorDataQoS(), std::bind(&VoronoiPlanner::FEPlanSubCB, this, _1), others_sub_opt);

  plan_req_dbg_sub_ = this->create_subscription<gestelt_interfaces::msg::PlanRequest>(
    "plan_request_dbg", rclcpp::SystemDefaultsQoS(), std::bind(&VoronoiPlanner::planReqDbgSubCB, this, _1));
  goals_sub_ = this->create_subscription<gestelt_interfaces::msg::Goals>(
    "goals", rclcpp::SystemDefaultsQoS(), std::bind(&VoronoiPlanner::goalsSubCB, this, _1));

  /* Timers */
	plan_fe_timer_ = this->create_wall_timer((1.0/fe_planner_freq_) *1000ms, 
                                            std::bind(&VoronoiPlanner::planFETimerCB, this), 
                                            planning_cb_group_);

	gen_voro_map_timer_ = this->create_wall_timer((1.0/gen_voro_map_freq_) *1000ms, 
                                                  std::bind(&VoronoiPlanner::genVoroMapTimerCB, this),
                                                  mapping_cb_group_);
}

void VoronoiPlanner::initParams()
{
  std::string param_ns = "navigator";
  
  this->declare_parameter(param_ns+".drone_id", -1);

  this->declare_parameter(param_ns+".planner_frequency", 10.0);
  this->declare_parameter(param_ns+".generate_voronoi_frequency", 10.0);

  this->declare_parameter(param_ns+".planner.goal_tolerance", 0.1);
  this->declare_parameter(param_ns+".planner.verbose_print", false);
  this->declare_parameter(param_ns+".planner.plan_once", false);
  this->declare_parameter(param_ns+".planner.t_unit", 0.1);

  this->declare_parameter(param_ns+".reservation_table.inflation", 0.3);
  this->declare_parameter(param_ns+".reservation_table.time_buffer", 0.5);
  this->declare_parameter(param_ns+".reservation_table.window_size", -1);

  this->declare_parameter(param_ns+".local_map_origin", "local_map_origin");
  this->declare_parameter(param_ns+".global_frame", "world");

	/**
	 * Get params
	 */

  drone_id_ = this->get_parameter(param_ns+".drone_id").as_int();

  fe_planner_freq_ = this->get_parameter(param_ns+".planner_frequency").as_double();
  gen_voro_map_freq_ = this->get_parameter(param_ns+".generate_voronoi_frequency").as_double();

  double goal_tol = this->get_parameter(param_ns+".planner.goal_tolerance").as_double();
  sqr_goal_tol_ = goal_tol * goal_tol;
  verbose_print_ = this->get_parameter(param_ns+".planner.verbose_print").as_bool();
  plan_once_ = this->get_parameter(param_ns+".planner.plan_once").as_bool();
  t_unit_ = this->get_parameter(param_ns+".planner.t_unit").as_double();

  rsvn_tbl_inflation_ = this->get_parameter(param_ns+".reservation_table.inflation").as_double();
  rsvn_tbl_window_size_ = this->get_parameter(param_ns+".reservation_table.window_size").as_int();
  double rsvn_tbl_t_buffer = this->get_parameter(param_ns+".reservation_table.time_buffer").as_double();
  t_buffer_ = (int) std::lround(rsvn_tbl_t_buffer/t_unit_);  // [space-time units] for time buffer

  local_map_origin_ = this->get_parameter(param_ns+".local_map_origin").as_string();
  global_frame_ = this->get_parameter(param_ns+".global_frame").as_string();
}

/* Core methods */
bool VoronoiPlanner::plan(const Eigen::Vector3d& start, const Eigen::Vector3d& goal){
  if (!init_voro_maps_){
    logger_->logInfo("Voronoi maps not initialized! Request a plan after initialization!");
    return false;
  }

  if (plan_once_ && plan_complete_){
    return true;
  }

  /* Lambda for checking if all higher priority plans are received*/
  auto isAllPrioPlansRcv = [&] () {
    for (int i = 0; i < drone_id_; i++){
      if (rsvn_tbl_.find(i) == rsvn_tbl_.end()){
        return false;
      }
    }
    return true;
  };

  rclcpp::Rate loop_rate(1000);
  while (!isAllPrioPlansRcv()){
    loop_rate.sleep();
  }

  // Assign voronoi map
  {
    std::lock_guard<std::mutex> voro_map_guard(voro_map_mtx_);

    voro_params_.z_separation_cm = bool_map_3d_.z_separation_cm;
    voro_params_.local_origin_x = bool_map_3d_.origin(0);
    voro_params_.local_origin_y = bool_map_3d_.origin(1);
    voro_params_.max_height_cm = bool_map_3d_.max_height_cm;
    voro_params_.min_height_cm = bool_map_3d_.min_height_cm;
    voro_params_.res = bool_map_3d_.resolution;

    fe_planner_->setVoroMap(dyn_voro_arr_, 
                            voro_params_);
  }

  // Update reservation table on planner
  {
    std::lock_guard<std::mutex> rsvn_tbl_guard(rsvn_tbl_mtx_);
    fe_planner_->setReservationTable(rsvn_tbl_);
    rsvn_tbl_.clear();
  }

  viz_helper_->pubStartGoalPts(start, goal, local_map_origin_, start_pt_pub_, goal_pt_pub_);

  tm_front_end_plan_.start();

  // Generate plan 
  if (!fe_planner_->generatePlan(start, goal))
  {
    logger_->logError(strFmt("Drone %d: Failed to generate plan from (%f, %f, %f) to (%f, %f, %f)", 
                              drone_id_, start(0), start(1), start(2), goal(0), goal(1), goal(2)));

    tm_front_end_plan_.stop(verbose_print_);

    logger_->logError(strFmt("Closed list size: %ld", fe_planner_->getClosedList().size()));
    // viz_helper_->pubFrontEndClosedList(fe_planner_->getClosedList(), 
    //                   fe_closed_list_viz_pub_, local_map_origin_);


    return false;
  }


  tm_front_end_plan_.stop(verbose_print_);

  // Retrieve space time path and publish it
  front_end_path_ = fe_planner_->getPath(cur_pos_);
  space_time_path_ = fe_planner_->getPathWithTime(cur_pos_);
  // smoothed_path_ = fe_planner_->getSmoothedPath();
  // smoothed_path_t_ = fe_planner_->getSmoothedPathWithTime();
  // viz_helper_->pubFrontEndClosedList(fe_planner_->getClosedList(), fe_closed_list_viz_pub_, local_map_origin_);

  // Convert from space time path to gestelt_interfaces::msg::SpaceTimePath
  gestelt_interfaces::msg::SpaceTimePath fe_plan_msg;

  fe_plan_msg.agent_id = drone_id_;
  fe_plan_msg.header.stamp = this->get_clock()->now();

  for (size_t i = 0; i < space_time_path_.size(); i++){
    geometry_msgs::msg::Pose pose;
    pose.position.x = space_time_path_[i](0);
    pose.position.y = space_time_path_[i](1);
    pose.position.z = space_time_path_[i](2);
    pose.orientation.w = 1.0; 

    fe_plan_msg.plan.push_back(pose);
    fe_plan_msg.plan_time.push_back(int(space_time_path_[i](3)));
  }

  fe_plan_msg.t_plan_start = this->get_clock()->now().seconds();

  fe_plan_pub_->publish(fe_plan_msg);
  fe_plan_broadcast_pub_->publish(fe_plan_msg);

  // viz_helper_->pubSpaceTimePath(space_time_path_, global_frame_, fe_plan_viz_pub_) ;

  viz_helper_->pubFrontEndPath(front_end_path_, global_frame_, fe_plan_viz_pub_);

  // double min_clr = DBL_MAX; // minimum path clearance 
  // double max_clr = 0.0;     // maximum path clearance 

  // for (const Eigen::Vector4d& pos_4d : space_time_path_)
  // {
  //   Eigen::Vector3d pos{pos_4d(0), pos_4d(1), pos_4d(2)}; 
  //   Eigen::Vector3d occ_nearest; 
  //   double dist_to_nearest_nb;
  //   if (voxel_map_->getNearestOccupiedCellLocal(pos, occ_nearest, dist_to_nearest_nb)){
  //     min_clr = (min_clr > dist_to_nearest_nb) ? dist_to_nearest_nb : min_clr;
  //     max_clr = (max_clr < dist_to_nearest_nb) ? dist_to_nearest_nb : max_clr;
  //   }
  // }
  // std::cout << "Maximum clearance: " << max_clr << ", Minimum clearance: " << min_clr << std::endl;

  plan_complete_ = true;

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
    return;
  }

  if (isGoalReached(cur_pos_, waypoints_.nextWP())){
    // If goals is within a given tolerance, then pop this goal and plan next goal (if available)
    waypoints_.popWP();

    return;
  }

  // Plan from current position to next waypoint
  plan(cur_pos_, waypoints_.nextWP());
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
    dynamic_voronoi::DynamicVoronoi::DynamicVoronoiParams dyn_voro_params;
    dyn_voro_params.res = bool_map_3d_.resolution;
    dyn_voro_params.origin_x = 0.0;
    dyn_voro_params.origin_y = 0.0;
    dyn_voro_params.origin_z = z_m;
    dyn_voro_params.origin_z_cm = z_cm;

    // Initialize dynamic voronoi 
    dyn_voro_arr_[z_cm] = std::make_shared<dynamic_voronoi::DynamicVoronoi>(dyn_voro_params);
    dyn_voro_arr_[z_cm]->initializeMap(bool_map_3d_.width, 
                                      bool_map_3d_.height, 
                                      bool_map.second);
    
    dyn_voro_arr_[z_cm]->update(); // update distance map and Voronoi diagram
    dyn_voro_arr_[z_cm]->prune();  // prune the Voronoi
    dyn_voro_arr_[z_cm]->updateAlternativePrunedDiagram();  

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

    nav_msgs::msg::OccupancyGrid occ_grid, voro_occ_grid;

    occmapToOccGrid(*dyn_voro_arr_[z_cm], 
                    bool_map_3d_.origin(0), bool_map_3d_.origin(1), 
                    occ_grid); // Occupancy map

    voronoimapToOccGrid(*dyn_voro_arr_[z_cm], 
                        bool_map_3d_.origin(0), bool_map_3d_.origin(1), 
                        voro_occ_grid); // Voronoi map

    voro_occ_grid_pub_->publish(voro_occ_grid);
    occ_map_pub_->publish(occ_grid);
  }

  // Link to the layer on top for bottommost layer
  dyn_voro_arr_[bool_map_3d_.min_height_cm]->top_voro_ = dyn_voro_arr_[bool_map_3d_.min_height_cm + bool_map_3d_.z_separation_cm];
  // Link to layer below for topmost layer
  dyn_voro_arr_[bool_map_3d_.max_height_cm]->bottom_voro_ = dyn_voro_arr_[bool_map_3d_.max_height_cm - bool_map_3d_.z_separation_cm];


  // Link all voronoi layers together
  for (int z_cm = bool_map_3d_.min_height_cm + bool_map_3d_.z_separation_cm ; 
          z_cm < bool_map_3d_.max_height_cm - bool_map_3d_.z_separation_cm; 
          z_cm += bool_map_3d_.z_separation_cm){
    // link to both layers on top and below
    dyn_voro_arr_[z_cm]->bottom_voro_ = dyn_voro_arr_[z_cm - bool_map_3d_.z_separation_cm];
    dyn_voro_arr_[z_cm]->top_voro_ = dyn_voro_arr_[z_cm + bool_map_3d_.z_separation_cm];
  }

  tm_voro_map_init_.stop(false);

  // viz_helper_->pubVoroVertices(voro_verts, local_map_origin_, voronoi_graph_pub_);

  init_voro_maps_ = true; // Flag to indicate that all voronoi maps have been initialized
}

/* Subscriber callbacks*/

void VoronoiPlanner::odomSubCB(const nav_msgs::msg::Odometry::UniquePtr msg)
{
  cur_pos_= Eigen::Vector3d{msg->pose.pose.position.x - bool_map_3d_.origin(0), 
                            msg->pose.pose.position.y - bool_map_3d_.origin(1), 
                            msg->pose.pose.position.z};
  // cur_vel_= Eigen::Vector3d{msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z};
}

void VoronoiPlanner::FEPlanSubCB(const gestelt_interfaces::msg::SpaceTimePath::UniquePtr msg)
{
  if (!init_voro_maps_){
    return;
  }

  // PRIORITY-BASED PLANNING: Only consider trajectories of drones with lower id
  if (msg->agent_id >= drone_id_ ){
    return;
  }

  std::lock_guard<std::mutex> rsvn_tbl_guard(rsvn_tbl_mtx_);

  // Clear reservation table
  rsvn_tbl_[msg->agent_id] = RsvnTbl(msg->t_plan_start);
  
  // Add all points on path (with inflation) to reservation table
  cells_inf_ = (int) std::lround(rsvn_tbl_inflation_/bool_map_3d_.resolution); // Number of cells used for inflation

  int window_size = (rsvn_tbl_window_size_ > 0) ? std::min(rsvn_tbl_window_size_, (int) msg->plan.size()) : msg->plan.size() ; 

  int prev_t = 0; // prev_t: Relative time of last points
  for (size_t i = 0; i < (size_t) window_size; i++){ // For points on plan up to window size
    IntPoint grid_pos;
    // get map position relative to local origin
    DblPoint map_2d_pos(msg->plan[i].position.x - bool_map_3d_.origin(0), 
                        msg->plan[i].position.y - bool_map_3d_.origin(1));
    int map_z_cm =roundToMultInt(mToCm(msg->plan[i].position.z), 
                                  bool_map_3d_.z_separation_cm,
                                  bool_map_3d_.min_height_cm,
                                  bool_map_3d_.max_height_cm);
    {
      std::lock_guard<std::mutex> voro_map_guard(voro_map_mtx_);
      dyn_voro_arr_[map_z_cm]->posToIdx(map_2d_pos, grid_pos);
    }

    // Inflate the cells by the given inflation radius
    for(int x = grid_pos.x - cells_inf_; x <= grid_pos.x + cells_inf_; x++)
    {
      for(int y = grid_pos.y - cells_inf_; y <= grid_pos.y + cells_inf_; y++)
      {
        // Reserve for time interval from previous t to current t, including time buffer
        for (int j = - t_buffer_; j < msg->plan_time[i] - prev_t + t_buffer_; j++) { 
          rsvn_tbl_[msg->agent_id].table.insert(Eigen::Vector4i{ x, y, 
                                                                map_z_cm, 
                                                                prev_t + j});
        }
      }
    }

    prev_t = msg->plan_time[i]; 
  } 

}

void VoronoiPlanner::goalsSubCB(const gestelt_interfaces::msg::Goals::UniquePtr msg)
{
    if (msg->waypoints.size() <= 0)
    {
      logger_->logError(strFmt("Received empty waypoints. Ignoring waypoints."));


      return;
    }
    if (msg->header.frame_id != "world" && msg->header.frame_id != "map" )
    {
      logger_->logError(strFmt("Only accepting goals in 'world' or 'map' frame, ignoring goals."));
      return;
    }

    std::vector<Eigen::Vector3d> wp_vec;

    for (auto& wp : msg->waypoints) {
      // Transform received waypoints from world to UAV origin frame
      wp_vec.push_back(Eigen::Vector3d(
                        wp.position.x - bool_map_3d_.origin(0), 
                        wp.position.y - bool_map_3d_.origin(1), 
                        wp.position.z));
    }

    waypoints_.addMultipleWP(wp_vec);
}

void VoronoiPlanner::planReqDbgSubCB(const gestelt_interfaces::msg::PlanRequest::UniquePtr msg)
{
  Eigen::Vector3d plan_start( 
                  msg->start.position.x - bool_map_3d_.origin(0),
                  msg->start.position.y - bool_map_3d_.origin(1),
                  msg->start.position.z);

  Eigen::Vector3d plan_end( 
                  msg->goal.position.x - bool_map_3d_.origin(0),
                  msg->goal.position.y - bool_map_3d_.origin(1),
                  msg->goal.position.z);

  std::cout << "Agent " << msg->agent_id << ": plan request from ("<< 
                plan_start.transpose() << ") to (" << plan_end.transpose() << ")" << std::endl;

  plan(plan_start, plan_end);
}

} // namespace navigator

