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

#include <navigator/navigator.hpp>

namespace navigator
{

Navigator::Navigator()
: Node("navigator")
{
	logger_ = std::make_shared<logger_wrapper::LoggerWrapper>(this->get_logger(), this->get_clock());

  initParams();

	// Create callback groups
  planning_cb_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive); // for planning
  mapping_cb_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive); // for generating voronoi maps
  swarm_plan_cb_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::Reentrant); 
  others_cb_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::Reentrant);

  initPubSubTimer();

  tm_front_end_plan_.updateID(drone_id_);
  tm_sfc_.updateID(drone_id_);
  tm_mpc_.updateID(drone_id_);
  tm_voro_gen_.updateID(drone_id_);
}

void Navigator::init()
{
  tf_buffer_ =
    std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ =
    std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Get global to map frame fixed TF
  try {
    auto tf_world_to_map = tf_buffer_->lookupTransform(
      map_frame_, global_frame_,
      tf2::TimePointZero,
      tf2_ros::fromRclcpp(rclcpp::Duration::from_seconds(5.0)));
    // Set fixed map origin
    map_origin_(0) = tf_world_to_map.transform.translation.x;
    map_origin_(1) = tf_world_to_map.transform.translation.y;
    
  } 
  catch (const tf2::TransformException & ex) {
		RCLCPP_ERROR(
			this->get_logger(), "Could not get transform from global frame '%s' to map frame '%s': %s",
			global_frame_.c_str(), map_frame_.c_str(), ex.what());
    rclcpp::shutdown();
    return;
  }

  // Initialize planner
  fe_planner_ = std::make_unique<global_planner::SpaceTimeAStar>(astar_params_, this->get_clock());

  // Initialize safe flight corridor
  poly_sfc_gen_ = std::make_unique<sfc::PolytopeSFC>(sfc_params_);

  // Initialize MPC controller
  mpc_controller_ = std::make_unique<pvaj_mpc::MPCController>(mpc_params_);

  // Initialize map
  voxel_map_ = std::make_unique<voxel_map::VoxelMap>(
    this->shared_from_this(), map_origin_, num_drones_);

  // Initialize minimum jerk optimizer
	// min_jerk_opt_ = std::make_unique<minco::MinJerkOpt>();

  // Initialize visualization helper
  viz_helper_ = std::make_unique<viz_helper::VizHelper>(this->get_clock());

	logger_->logInfo("Initialized");
} 

Navigator::~Navigator()
{}

void Navigator::initPubSubTimer()
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

  /* Planner publishers */
  fe_plan_broadcast_pub_ = this->create_publisher<gestelt_interfaces::msg::SpaceTimePath>(
    "fe_plan/broadcast", rclcpp::SensorDataQoS());

  lin_mpc_cmd_pub_ = this->create_publisher<mavros_msgs::msg::PositionTarget>(
    "navigator/intmd_cmd", rclcpp::SensorDataQoS());

  nav_state_pub_ = this->create_publisher<gestelt_interfaces::msg::NavState>(
    "navigator/state", rclcpp::SensorDataQoS());

  // poly_traj_pub_ = this->create_publisher<minco_interfaces::msg::PolynomialTrajectory>(
  //   "poly_traj", rclcpp::SensorDataQoS());
  // minco_traj_broadcast_pub_ = this->create_publisher<minco_interfaces::msg::MincoTrajectory>(
  //   "minco_traj/broadcast", rclcpp::SensorDataQoS());

  /* Visualization Publishers */
  voro_planning_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "voro_planning", 10);
  // voronoi_graph_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("voronoi_graph", 10);

  agent_id_text_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
    "agent_id_text", 10);
  plan_req_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
    "fe_plan_req", 10);
  fe_closed_list_viz_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
    "fe_plan/closed_list", 10);
  fe_plan_viz_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
    "fe_plan/viz", 10);

	minco_traj_viz_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
    "minco_traj_viz", 10);

  mpc_pred_pos_pub_ = this->create_publisher<nav_msgs::msg::Path>(
    "mpc/traj", 10);

  poly_sfc_pub_ = this->create_publisher<decomp_ros_msgs::msg::PolyhedronArray>(
    "sfc", 10);

  /* Subscribers */
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", rclcpp::SensorDataQoS(), std::bind(&Navigator::odomSubCB, this, _1), others_sub_opt);

  plan_req_dbg_sub_ = this->create_subscription<gestelt_interfaces::msg::PlanRequest>(
    "plan_request_dbg", rclcpp::SystemDefaultsQoS(), std::bind(&Navigator::planReqDbgSubCB, this, _1));
  goals_sub_ = this->create_subscription<gestelt_interfaces::msg::Goals>(
    "goals", rclcpp::SystemDefaultsQoS(), std::bind(&Navigator::goalsSubCB, this, _1));
  point_goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "point_goal", rclcpp::SystemDefaultsQoS(), std::bind(&Navigator::pointGoalSubCB, this, _1));

  // Subscribe to odometry individually from each agent
  for (int i = 0; i < num_drones_; i++){

    if (i == drone_id_){
      continue;
    }

    std::function<void(const nav_msgs::msg::Odometry::UniquePtr msg)> bound_callback_func =
      std::bind(&Navigator::swarmOdomCB, this, _1, i);

    swarm_odom_subs_.push_back(
      this->create_subscription<nav_msgs::msg::Odometry>(
        "/d"+ std::to_string(i) + "/" + "odom", 
        rclcpp::SensorDataQoS(), 
        bound_callback_func, 
        swarm_plan_sub_opt)
    );
  }

  /* Timers */
	pub_state_timer_ = this->create_wall_timer((1.0/pub_state_freq_) *1000ms, 
                                            std::bind(&Navigator::pubStateTimerCB, this), 
                                            others_cb_group_);

	plan_fe_timer_ = this->create_wall_timer((1.0/fe_planner_freq_) *1000ms, 
                                            std::bind(&Navigator::planFETimerCB, this), 
                                            planning_cb_group_);

	gen_voro_map_timer_ = this->create_wall_timer((1.0/gen_voro_map_freq_) *1000ms, 
                                                  std::bind(&Navigator::genVoroMapTimerCB, this),
                                                  mapping_cb_group_);

	send_mpc_cmd_timer_ = this->create_wall_timer((1.0/ctrl_samp_freq_) *1000ms, 
                                                  std::bind(&Navigator::sendMPCCmdTimerCB, this),
                                                  others_cb_group_);
}

void Navigator::initParams()
{
  std::string param_ns = "navigator";
  
  this->declare_parameter("drone_id", -1);
  this->declare_parameter(param_ns+".num_drones", 4);
  this->declare_parameter(param_ns+".plan_method", -1);
  this->declare_parameter(param_ns+".generate_voronoi_frequency", 10.0);
  this->declare_parameter(param_ns+".pub_state_frequency", 10.0);
  this->declare_parameter(param_ns+".planner_frequency", 10.0);

  this->declare_parameter(param_ns+".planner_recovery_timeout", 5.0);

  this->declare_parameter(param_ns+".point_goal_height", 1.5);

  /* Frame ids */
  this->declare_parameter("global_frame", "world");
  this->declare_parameter("map_frame", "map");
  this->declare_parameter("local_map_frame", "local_map_frame");
  this->declare_parameter("camera_frame", "camera_frame");
  this->declare_parameter("base_link_frame", "base_link_frame");

  /* Space time A* planner */
  this->declare_parameter(param_ns+".planner.goal_tolerance", 0.1);
  this->declare_parameter(param_ns+".planner.verbose_print", false);
  this->declare_parameter(param_ns+".planner.plan_once", false);
  this->declare_parameter(param_ns+".planner.t_unit", 0.1);
  this->declare_parameter(param_ns+".planner.output_json_filepath", "");



  this->declare_parameter(param_ns+".reservation_table.inflation", 0.3);
  this->declare_parameter(param_ns+".reservation_table.time_buffer", 0.5);
  this->declare_parameter(param_ns+".reservation_table.window_size", -1);

  /* SFC */
  this->declare_parameter(param_ns+".sfc.poly.bbox_x", 1.0);
  this->declare_parameter(param_ns+".sfc.poly.bbox_y", 1.0);
  this->declare_parameter(param_ns+".sfc.poly.bbox_z", 1.0);

  /* MPC */
  this->declare_parameter(param_ns+".mpc.ctrl_samp_freq", 30.0);
  this->declare_parameter(param_ns+".mpc.ref_path_samp_interval", 1);

  this->declare_parameter(param_ns+".mpc.horizon", 15);
  this->declare_parameter(param_ns+".mpc.time_step", 0.1);

  this->declare_parameter(param_ns+".mpc.R_p", 1000.0);
  this->declare_parameter(param_ns+".mpc.R_v", 0.0);
  this->declare_parameter(param_ns+".mpc.R_a", 0.0);
  this->declare_parameter(param_ns+".mpc.R_u", 0.0);
  this->declare_parameter(param_ns+".mpc.R_u_con", 0.2);
  this->declare_parameter(param_ns+".mpc.R_pN", 2000.0);
  this->declare_parameter(param_ns+".mpc.R_vN", 1000.0);
  this->declare_parameter(param_ns+".mpc.R_aN", 1000.0);

  this->declare_parameter(param_ns+".mpc.v_min", -10.0);
  this->declare_parameter(param_ns+".mpc.v_max", 10.0);
  this->declare_parameter(param_ns+".mpc.a_min", -20.0);
  this->declare_parameter(param_ns+".mpc.a_max", 20.0);
  this->declare_parameter(param_ns+".mpc.u_min", -50.0);
  this->declare_parameter(param_ns+".mpc.u_max", 50.0);


  /* MINCO */
  this->declare_parameter(param_ns+".min_jerk_trajectory.front_end_stride", 3);

	/**
	 * Get params
	 */

  drone_id_ = this->get_parameter("drone_id").as_int();
  num_drones_ = this->get_parameter(param_ns+".num_drones").as_int();
  pub_state_freq_ = this->get_parameter(param_ns+".pub_state_frequency").as_double();
  fe_planner_freq_ = this->get_parameter(param_ns+".planner_frequency").as_double();
  gen_voro_map_freq_ = this->get_parameter(param_ns+".generate_voronoi_frequency").as_double();

  planner_recovery_timeout_ = this->get_parameter(param_ns+".planner_recovery_timeout").as_double();

  point_goal_height_ = this->get_parameter(param_ns+".point_goal_height").as_double();

	auto getPlanMethod = [=](const int& method) -> PlanMethod
	{
    switch (method) {
      case 0: 
        return PlanMethod::COMMLESS_MINCO;
        break;
      case 1: 
        return PlanMethod::COMMLESS_MPC;
        break;
      case 2: 
        return PlanMethod::COMM_MINCO;
        break;
      default: 
        return PlanMethod::UNDEFINED;
        break;
    }
	};
  plan_method_ = getPlanMethod(this->get_parameter(param_ns+".plan_method").as_int());




  /* Frame ids */
  global_frame_ = this->get_parameter("global_frame").as_string();
  map_frame_ = this->get_parameter("map_frame").as_string();
  local_map_frame_ = this->get_parameter("local_map_frame").as_string();

  /* A* Planner */
  double goal_tol = this->get_parameter(param_ns+".planner.goal_tolerance").as_double();
  sqr_goal_tol_ = goal_tol * goal_tol;
  verbose_print_ = this->get_parameter(param_ns+".planner.verbose_print").as_bool();
  plan_once_ = this->get_parameter(param_ns+".planner.plan_once").as_bool();
  t_unit_ = this->get_parameter(param_ns+".planner.t_unit").as_double();
  output_json_filepath_ = this->get_parameter(param_ns+".planner.output_json_filepath").as_string();


  rsvn_tbl_inflation_ = this->get_parameter(param_ns+".reservation_table.inflation").as_double();
  rsvn_tbl_window_size_ = this->get_parameter(param_ns+".reservation_table.window_size").as_int();
  double rsvn_tbl_t_buffer = this->get_parameter(param_ns+".reservation_table.time_buffer").as_double();
  t_buffer_ = (int) std::lround(rsvn_tbl_t_buffer/t_unit_);  // [space-time units] for time buffer

  /* A* parameters */
  astar_params_.drone_id = drone_id_;
  astar_params_.max_iterations = 99999;
  astar_params_.debug_viz = true;
  astar_params_.tie_breaker = 1.001;
  astar_params_.cost_function_type  = 1; // 0: getOctileDist, 1: getL1Norm, 2: getL2Norm, 3: getChebyshevDist
  astar_params_.t_unit = t_unit_;

  // Liu SFC Params
  sfc_params_.map_frame = map_frame_;
  sfc_params_.bbox_x = this->get_parameter(param_ns+".sfc.poly.bbox_x").as_double();
  sfc_params_.bbox_y = this->get_parameter(param_ns+".sfc.poly.bbox_y").as_double();
  sfc_params_.bbox_z = this->get_parameter(param_ns+".sfc.poly.bbox_z").as_double();

  /* MINCO */
  fe_stride_ = this->get_parameter(param_ns+".min_jerk_trajectory.front_end_stride").as_int();

  /* MPC */
  ctrl_samp_freq_ = this->get_parameter(param_ns+".mpc.ctrl_samp_freq").as_double();
  ref_samp_intv_  = this->get_parameter(param_ns+".mpc.ref_path_samp_interval").as_int();

  mpc_params_.MPC_HORIZON  = this->get_parameter(param_ns+".mpc.horizon").as_int();
  mpc_params_.MPC_STEP = this->get_parameter(param_ns+".mpc.time_step").as_double();

  mpc_params_.R_p = this->get_parameter(param_ns+".mpc.R_p").as_double();
  mpc_params_.R_v = this->get_parameter(param_ns+".mpc.R_v").as_double();
  mpc_params_.R_a = this->get_parameter(param_ns+".mpc.R_a").as_double();
  mpc_params_.R_u = this->get_parameter(param_ns+".mpc.R_u").as_double();
  mpc_params_.R_u_con = this->get_parameter(param_ns+".mpc.R_u_con").as_double();
  mpc_params_.R_pN = this->get_parameter(param_ns+".mpc.R_pN").as_double();
  mpc_params_.R_vN = this->get_parameter(param_ns+".mpc.R_vN").as_double();
  mpc_params_.R_aN = this->get_parameter(param_ns+".mpc.R_aN").as_double();

  double v_min = this->get_parameter(param_ns+".mpc.v_min").as_double();
  double v_max = this->get_parameter(param_ns+".mpc.v_max").as_double();
  double a_min = this->get_parameter(param_ns+".mpc.a_min").as_double();
  double a_max = this->get_parameter(param_ns+".mpc.a_max").as_double();
  double u_min = this->get_parameter(param_ns+".mpc.u_min").as_double();
  double u_max = this->get_parameter(param_ns+".mpc.u_max").as_double();
  // State bounds
  mpc_params_.v_min = Eigen::Vector3d(v_min, v_min, v_min);
  mpc_params_.v_max = Eigen::Vector3d(v_max, v_max, v_max);
  mpc_params_.a_min = Eigen::Vector3d(a_min, a_min, a_min);
  mpc_params_.a_max = Eigen::Vector3d(a_max, a_max, a_max);
  // Control bounds
  mpc_params_.u_min = Eigen::Vector3d(u_min, u_min, u_min);
  mpc_params_.u_max = Eigen::Vector3d(u_max, u_max, u_max);

  mpc_params_.drone_id = drone_id_;
  // Dynamical Parameters
  mpc_params_.Drag.setZero();
  mpc_params_.Drag(0,0) = 0.0;
  mpc_params_.Drag(1,1) = 0.0;
  mpc_params_.Drag(2,2) = 0.0;
}

/* Timer callbacks*/

void Navigator::pubStateTimerCB()
{
  gestelt_interfaces::msg::NavState state_msg;
  state_msg.state = gestelt_interfaces::msg::NavState::IDLE;

  if (currently_planning_){
    state_msg.state = gestelt_interfaces::msg::NavState::PLANNING;

    // check if there is timeout for planning success
    if (last_plan_success_ - this->get_clock()->now().nanoseconds()/1e9 > planner_recovery_timeout_)
    {
      logger_->logError("Planner recovery timeout activated!");
      state_msg.state = gestelt_interfaces::msg::NavState::PLANNING_TIMEOUT;
    }
  }

  // Publish state
  nav_state_pub_->publish(state_msg);
}

void Navigator::sendMPCCmdTimerCB()
{
  if (mpc_pred_pos_.empty() 
      || mpc_pred_vel_.empty() 
      || mpc_pred_acc_.empty())
  {
    return;
  }

  std::lock_guard<std::mutex> mpc_pred_lk(mpc_pred_mtx_);

  double t_start = this->get_clock()->now().nanoseconds()/1e9;
  // // Get time t relative to start of MPC trajectory
  double e_t_start = t_start - last_mpc_solve_; 
  double total_traj_duration = (int)mpc_pred_acc_.size() * mpc_controller_->MPC_STEP;
  
  if (e_t_start < 0.0 || e_t_start >= total_traj_duration)
  {
    // Exceeded duration of trajectory or trajectory timestamp invalid
    return;
  }

  // Get index of point closest to current time
  //  elapsed time from start divided by time step of MPC
  int idx =  std::floor(e_t_start / mpc_controller_->MPC_STEP); 

  if (idx >= (int) mpc_pred_pos_.size()){
    logger_->logError("DEV ERROR!! sampleMPCTrajectory idx exceeded!");
    return;
  }

  // Publish PVA command
  pubPVAJCmd(mpc_pred_pos_[idx], 
            Eigen::Vector2d(0.0, NAN), 
            mpc_pred_vel_[idx], 
            mpc_pred_acc_[idx], 
            mpc_pred_u_[idx]);
}

void Navigator::planFETimerCB()
{
  // Check if waypoint queue is empty
  if (waypoints_.empty()){
    currently_planning_ = false;

    return;
  }

  Eigen::Vector3d goal = waypoints_.nextWP();

  if (isGoalReached(cur_pos_, goal)){
    logger_->logInfo("Reached goal!");
    
    // If goals is within a given tolerance, then pop this goal and plan next goal (if available)
    waypoints_.popWP();

    return;
  }

  currently_planning_ = true;

  // Plan from current position to next waypoint
  if (!planCommlessMPC(goal)){
    return;
  }
  
  last_plan_success_ = this->get_clock()->now().nanoseconds()/1e9;
}

void Navigator::genVoroMapTimerCB()
{
  if (!voxel_map_->getBoolMap3D(bool_map_3d_)){
    return;
  }

  std::unique_lock<std::mutex> map_lk(roadmap_mtx_);
  prod_map_cv_.wait(map_lk, [&]{return !map_ready_for_cons_;}); // wait for voronoi map to be consumed

  // std::vector<Eigen::Vector3d> voro_verts;

  tm_voro_gen_.start();

  // Initialize dynamic voronoi 

  for (auto const& bool_map : bool_map_3d_.bool_maps) // For each boolean map
  {
    int z_cm = bool_map.first;
    double z_m = cmToM(bool_map.first);

    // Create DynamicVoronoi object if it does not exist
    dynamic_voronoi::DynamicVoronoiParams dyn_voro_params;
    dyn_voro_params.res = bool_map_3d_.resolution;
    dyn_voro_params.origin_z = z_m;
    dyn_voro_params.origin_z_cm = z_cm;
    // dyn_voro_params.origin_x = 0.0;
    // dyn_voro_params.origin_y = 0.0;

    dyn_voro_arr_[z_cm].reset();
    dyn_voro_arr_[z_cm] = std::make_shared<dynamic_voronoi::DynamicVoronoi>();
    dyn_voro_arr_[z_cm]->setParams(dyn_voro_params);
    // Map is received in local_map_frame_
    dyn_voro_arr_[z_cm]->initializeMap( bool_map_3d_.width, 
                                        bool_map_3d_.height, 
                                        bool_map.second);

    dyn_voro_arr_[z_cm]->update(); // update distance map and Voronoi diagram
    dyn_voro_arr_[z_cm]->updateAlternativePrunedDiagram();
    // dyn_voro_arr_[z_cm]->prune();  // prune the Voronoi

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

  tm_voro_gen_.stop(false);
  // viz_helper_->pubVoroVertices(voro_verts, voronoi_graph_pub_, local_map_frame_);

  // Notify the planning thread that the voronoi map is ready for consumption
  map_ready_for_cons_ = true;
  cons_map_cv_.notify_one();
}

/* Core methods */

bool Navigator::planCommlessMPC(const Eigen::Vector3d& goal_pos){
  if (plan_once_ && plan_complete_){
    return true;
  }

  /*****/
  /* 1) Assign voronoi map and reservation table to planner */
  /*****/

  // Assign reservation table
  {
    // std::lock_guard<std::mutex> rsvn_tbl_guard(rsvn_tbl_mtx_);
    // fe_planner_->setReservationTable(rsvn_tbl_);
  }

  // Wait for condition variable notification
  std::unique_lock<std::mutex> map_lk(roadmap_mtx_);
  cons_map_cv_.wait(map_lk, [&]{return map_ready_for_cons_;});

  // Assign voronoi map
  voro_params_.z_sep_cm = bool_map_3d_.z_sep_cm;
  voro_params_.local_origin_x = bool_map_3d_.origin(0);
  voro_params_.local_origin_y = bool_map_3d_.origin(1);
  voro_params_.max_height_cm = bool_map_3d_.max_height_cm;
  voro_params_.min_height_cm = bool_map_3d_.min_height_cm;
  voro_params_.res = bool_map_3d_.resolution;

  fe_planner_->setVoroMap(dyn_voro_arr_, voro_params_);

  /*****/
  /* 2) Get current position, velocity and acceleration */
  /*****/

  auto plan_start_clock = this->get_clock()->now();

  Eigen::Vector3d start_pos, start_vel, start_acc;

  if (!sampleMPCTrajectory(mpc_controller_, plan_start_clock.nanoseconds()/1e9, 
                          start_pos, start_vel, start_acc))
  {
    start_pos = cur_pos_;
    start_vel = cur_vel_;
    start_acc.setZero();
  }

  /*****/
  /* 3) Get Receding Horizon Planning goal */
  /*****/

  // Get RHP goal
  Eigen::Vector3d rhp_goal_pos = getRHPGoal(
    start_pos, goal_pos, 
    voxel_map_->getLocalMapOrigin(0.15), voxel_map_->getLocalMapMax(0.15));

  // Publish start and goal visualization
  viz_helper_->pubPlanRequestViz(start_pos, rhp_goal_pos, goal_pos, plan_req_pub_, map_frame_);

  /*****/
  /* 4) Plan HCA* path */
  /*****/

  tm_front_end_plan_.start();

  bool fe_plan_success = fe_planner_->generatePlan(mapToLclMap(start_pos), mapToLclMap(rhp_goal_pos));

  // Notify the voronoi map generation thread that plan is complete
  map_ready_for_cons_ = false;
  prod_map_cv_.notify_one();

  tm_front_end_plan_.stop(false);
  tm_front_end_plan_.getWallAvg(verbose_print_);

  if (!fe_plan_success)
  {
    logger_->logError(strFmt("Drone %d: Failed to generate FE plan from (%f, %f, %f) to (%f, %f, %f) with closed_list of size %ld", 
                              drone_id_, 
                              start_pos(0), start_pos(1), start_pos(2), 
                              rhp_goal_pos(0), rhp_goal_pos(1), rhp_goal_pos(2),
                              fe_planner_->getClosedList().size()));
    // logger_->logError(strFmt("  Drone %d: local_map origin (%f, %f)", 
    //                           drone_id_, 
    //                           bool_map_3d_.origin(0), 
    //                           bool_map_3d_.origin(1)));

    // viz_helper_->pubFrontEndClosedList(fe_planner_->getClosedList(), 
    //                   fe_closed_list_viz_pub_, map_frame_);

    return false;
  }

  /*****/
  /* 5) Retrieve and transform HCA* path to map frame, publish visualization of path */
  /*****/

  // Current pose must converted from fixed map frame to local map frame 
  // before passing to planner
  // std::vector<Eigen::Vector4d> fe_path_with_t_lclmapframe = fe_planner_->getPathWithTime(mapToLclMap(cur_pos_));
  std::vector<Eigen::Vector4d> fe_path_with_t_lclmapframe = fe_planner_->getPathWithTime();
  std::vector<Eigen::Vector4d> fe_path_with_t_lclmapframe_smoothed = fe_planner_->getPathWithTimeSampled(10);
  // fe_sfc_segment_idx: Index of sampled original path. Indexed by polygon number. 
  //    The value is the front-end index at that polygon's starting segment
  std::vector<int> fe_sfc_segment_idx = fe_planner_->getPathIdxSampled(10);

  // std::vector<Eigen::Vector4d> fe_path_with_t_lclmapframe_smoothed = fe_planner_->getSmoothedPathWithTime();
  // std::vector<int> fe_sfc_segment_idx = fe_planner_->getSmoothedPathIdx(); // Index of smoothed path in original path

  auto transformPathFromLclMapToMap = [&](const std::vector<Eigen::Vector4d>& map_path_w_t_lclmapframe, 
                             std::vector<Eigen::Vector4d>& map_path_w_t,
                             std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& map_path) 
  {
    map_path.clear();
    map_path_w_t.clear();
    
    for (size_t i = 0; i < map_path_w_t_lclmapframe.size(); i++){
      Eigen::Vector4d map_pt;
      // Transform from local map frame to map
      map_pt.head<3>() = lclMapToMap(map_path_w_t_lclmapframe[i].head<3>());
      map_pt(3) = map_path_w_t_lclmapframe[i](3);

      map_path.push_back(map_pt.head<3>());
      map_path_w_t.push_back(map_pt);
    }
  };

  // Convert front-end path from local map frame to fixed map frame
  transformPathFromLclMapToMap(fe_path_with_t_lclmapframe, 
                              fe_path_with_t_, fe_path_);
  transformPathFromLclMapToMap(fe_path_with_t_lclmapframe_smoothed, 
                              fe_path_with_t_smoothed_, fe_path_smoothed_);
  // Visualize front-end path
  viz_helper_->pubFrontEndPath(fe_path_, fe_plan_viz_pub_, map_frame_);

  /*****/
  /* 6) Generate Polyhedron safe flight corridor */
  /*****/

  tm_sfc_.start();
  bool gen_sfc_success = poly_sfc_gen_->generateSFC(voxel_map_->getLclObsPts(), fe_path_smoothed_);
  tm_sfc_.stop(false);
  tm_sfc_.getWallAvg(verbose_print_);

  // Generate SFC based on smoothed front-end path
  if (!gen_sfc_success)
  {
    logger_->logError("Failed to generate safe flight corridor!");
    return false;
  }

  // Get polyhedrons
  std::vector<Polyhedron3D, Eigen::aligned_allocator<Polyhedron3D>> polyhedrons 
    = poly_sfc_gen_->getPolyVec();

  poly_sfc_pub_->publish(poly_sfc_gen_->getSFCMsg());

  // logger_->logInfo(strFmt(" Size of SFC: %d, Size of smoothed path: %d", 
  //                   polyhedrons.size(), fe_path_smoothed_.size()));

  /*****/
  /* 7) Generate commands for executing trajectory */
  /*****/

  // Find the nearest AStar path point to initial condition
  int fe_start_idx = 0;
  double min_dis = std::numeric_limits<double>::max();

  // TODO: improve this routine using KDTree
  for (int i = 0; i < (int)fe_path_.size() ; ++i) { // find the nearest point
    double dis = (cur_pos_ - fe_path_[i]).norm();
    if (dis < min_dis) {
        min_dis = dis;
        fe_start_idx = i;
    }

    if (i >= 10){
      break;
    }
  }

  auto polyhedronToPlanes = [&](const Polyhedron3D& poly, 
                                Eigen::MatrixX4d& planes) {
    int num_planes = (int) poly.vs_.size();
    planes.resize(num_planes, 4);

    for (int i = 0; i < num_planes; ++i) // for each plane
    {
      Eigen::Vector3d normal = poly.vs_[i].n_; // normal points outward (a,b,c) as in ax+by+cz = d
      Eigen::Vector3d pt = poly.vs_[i].p_; // Point on plane

      double d = normal.dot(pt);   // Scalar d obtained from point.dot(normal)

      // Final plane needs to have normal pointing outwards
      // ax + by + cy + d = 0
      planes.row(i) << normal(0), normal(1), normal(2), -d;
    }

  };


  // 7a) Set Safe Flight corridor for each MPC reference point

  // auto getPolygonIdx = [&](const int& fe_idx, std::vector<int>& fe_sampled_idx) -> int {
  //   for (int i = 1; i < (int)fe_sampled_idx.size(); i++){
  //     if (fe_idx <= fe_sampled_idx[i]){
  //       return i-1;
  //     }
  //   }

  //   return -1; 
  // };

  // int final_fe_idx = fe_start_idx; // last index of front end path that lies in SFC
  // // For each front_end point, check that it is within polygonal bounds
  // for (int i = 0; i < mpc_controller_->MPC_HORIZON; ++i){ // for each MPC point
  //   int fe_idx = fe_start_idx + i;

  //   final_fe_idx = fe_idx;

  //   // Check if Front end point exceeded safe flight corridor 
  //   if (fe_idx > fe_sfc_segment_idx.back()){
  //     break;
  //   }
  //   // Check if front end point exceeded front end path
  //   if (fe_idx >= (int) fe_path_.size()){
  //     break;
  //   }

  //   int poly_idx = getPolygonIdx(fe_idx, fe_sfc_segment_idx);
  //   if (poly_idx == -1){ 
  //     // exceeded safe flight corridor
  //     break;
  //   }

  //   Eigen::MatrixX4d planes;
  //   polyhedronToPlanes(polyhedrons[poly_idx], planes);

  //   if (!mpc_controller_->isInFSC(fe_path_[fe_idx], planes)) { 
  //     // if fe_idx not in sfc. Then all MPC reference points should end at previous fe_idx
  //     logger_->logError(strFmt("  CODE ERROR! FE Path idx %d is not in polygon %d", fe_idx, poly_idx ));
  //     final_fe_idx = fe_idx - 1 >= 0 ? fe_idx-1 : 0;
  //     break;
  //   }

  //   // Set safe flight corridor for i-th control iteration
  //   mpc_controller_->setFSC(planes, i); 
  // }

  int final_fe_idx = (int) fe_path_.size(); // last index of front end path that lies in SFC
  for (int i = 0, poly_idx = 0; i < mpc_controller_->MPC_HORIZON; i++) // for each MPC point
  {
    int fe_idx = fe_start_idx + i * ref_samp_intv_; // Current FE path index
    if (fe_idx >= (int)fe_path_.size()){ // exceed size of fe path
      fe_idx = fe_path_.size() - 1;
    }

    // int poly_start_idx = fe_sfc_segment_idx[poly_idx]; //idx of fe_path at start of segment
    int poly_end_idx = fe_sfc_segment_idx[poly_idx+1]; //idx of fe_path at end of segment

    if (fe_idx >= poly_end_idx) { // FE Path index exceed idx of fe_path at end of segment
      // logger_->logInfo(strFmt("FE Path idx(%d) >= segment %d", fe_idx, poly_start_idx ));
      poly_idx++; // Iterate to next polygon

      // if already last polygon, then set as last polygon
      poly_idx = poly_idx > (int)polyhedrons.size() - 1 ? (int)polyhedrons.size() -1 : poly_idx;
      // logger_->logError(" CODE ERROR! poly_idx exceeds size of polyehdrons");
    }

    Eigen::MatrixX4d planes;
    polyhedronToPlanes(polyhedrons[poly_idx], planes);

    if (!mpc_controller_->isInFSC(fe_path_[fe_idx], planes)) { 
      // if front end point is not in sfc
      // Then all reference points should end at previous fe_idx
      logger_->logError(strFmt("  CODE ERROR! FE Path idx %d is not in polygon %d", fe_idx, poly_idx ));
      final_fe_idx = fe_idx - 1 >= 0 ? fe_idx-1 : 0;
      break;
    }

    // Set safe flight corridor for i-th control iteration
    mpc_controller_->setFSC(planes, i); 
  }

  // 7b) Set reference path for MPC 
  Eigen::Vector3d last_p_ref; // last position reference
  for (int i = 0; i < mpc_controller_->MPC_HORIZON; i++) 
  {
    int fe_idx = fe_start_idx + i;
    if (fe_idx > final_fe_idx){ // exceed final SFC-bounded point on FE path
      fe_idx = final_fe_idx;
    }
    else if (fe_idx >= (int)fe_path_.size()){ // exceed size of fe path
      fe_idx = fe_path_.size() - 1;
    }

    Eigen::Vector3d p_ref = fe_path_[fe_idx]; // position reference
    Eigen::Vector3d v_ref(0, 0, 0); // vel reference
    Eigen::Vector3d a_ref(0, 0, 0); // acc reference

    if (i == 0) // First iteration
    {
      v_ref = (p_ref - cur_pos_) / mpc_controller_->MPC_STEP;
    }
    else // rest of the iteration
    {
      v_ref = (p_ref - last_p_ref) / mpc_controller_->MPC_STEP;
    }

    // Set PVA reference at given step i
    mpc_controller_->setReference(p_ref, v_ref, a_ref, i);

    last_p_ref = p_ref;
  }

  // 7c) Solve MPC and visualize path

  // Set initial condition
  mpc_controller_->setInitialCondition(start_pos, start_vel, start_acc);

  tm_mpc_.start();
  bool mpc_success = mpc_controller_->run();
  tm_mpc_.stop(false);
  tm_mpc_.getWallAvg(verbose_print_);

  last_mpc_solve_ = this->get_clock()->now().nanoseconds() / 1e9;

  if (!mpc_success){ // Successful MPC solve
    logger_->logError("MPC Solve failure!");
    return false;
  }

  // Set yaw
  // TODO: figure out a better way
  // int lk_ahd = 5; // Look ahead distance
  // int lk_ahd_idx = (fe_start_idx + lk_ahd) < fe_path_.size() ? fe_start_idx + lk_ahd : fe_start_idx;
  // yaw_yawrate(0) = std::atan2(fe_path_[lk_ahd_idx](1) - cur_pos_(1), 
  //                             fe_path_[lk_ahd_idx](0) - cur_pos_(0));

  // yaw_yawrate = calculateYaw(fe_path_, plan_start_clock.nanoseconds()/1e9, ...);

  // Check if MPC command values are valid
  auto checkValidCmd = [&](const Eigen::Vector3d& vec, const int& min_val, const int& max_val){
      for (const auto& val : vec){
        if (val < min_val || val > max_val){
          return false;
        }
      }
    return true;
  };

  {
    Eigen::Vector2d yaw_yawrate{0.0, NAN};
    Eigen::MatrixXd A1, B1; // system transition matrix

    mpc_controller_->getSystemModel(A1, B1, mpc_controller_->MPC_STEP);
    Eigen::VectorXd x_optimal = mpc_controller_->X_0_;

    std::lock_guard<std::mutex> mpc_pred_lk(mpc_pred_mtx_);

    mpc_pred_u_.clear();
    mpc_pred_pos_.clear();
    mpc_pred_vel_.clear();
    mpc_pred_acc_.clear();

    // Get predicted MPC path based on controls and check if valid
    Eigen::Vector3d u_optimal; 
    for (int i = 0; i < mpc_controller_->MPC_HORIZON; i++) {
        mpc_controller_->getOptimalControl(u_optimal, i);
        x_optimal = A1 * x_optimal + B1 * u_optimal;

        // Check if position, velocity and acceleration at valid values
        if (!checkValidCmd(x_optimal.segment<3>(0), -50.0, 50.0) ){
          logger_->logInfo(strFmt("Invalid MPC commanded position: (%f, %f, %f)"
            ,x_optimal.segment<3>(0)(0), x_optimal.segment<3>(0)(1), x_optimal.segment<3>(0)(2))) ;
          return false;
        }
        if (!checkValidCmd(x_optimal.segment<3>(3), -10.0, 10.0) ){
          logger_->logInfo(strFmt("Invalid MPC commanded velocity: (%f, %f, %f)"
            ,x_optimal.segment<3>(3)(0), x_optimal.segment<3>(3)(1), x_optimal.segment<3>(3)(2))) ;
          return false;
        }
        if (!checkValidCmd(x_optimal.segment<3>(6), -60.0, 60.0) ){
          logger_->logInfo(strFmt("Invalid MPC commanded acceleration: (%f, %f, %f)"
            ,x_optimal.segment<3>(6)(0), x_optimal.segment<3>(6)(1), x_optimal.segment<3>(6)(2))) ;
          return false;
        }

        mpc_pred_u_.push_back(u_optimal);
        mpc_pred_pos_.push_back(x_optimal.segment<3>(0));
        mpc_pred_vel_.push_back(x_optimal.segment<3>(3));
        mpc_pred_acc_.push_back(x_optimal.segment<3>(6));
    }
  }

  pubMPCPath(mpc_pred_pos_); // Publish MPC path for visualization

  plan_complete_ = true;

  return true;
}

/* Subscriber callbacks*/

void Navigator::swarmOdomCB(const nav_msgs::msg::Odometry::UniquePtr& msg, int id)
{
  //TODO: Odom is received in the drone's map frame but need to handle 
  //  if the drones' map frame is not in 'world' frame
  // if (msg->header.frame_id != "world"){
  //   logger_->logError("Does not yet handle odom messages with frame_id != 'world'");
  // }

  /**
   * Update swarm poses and velocities
   */

  Eigen::Vector3d pose = Eigen::Vector3d{msg->pose.pose.position.x,  
                          msg->pose.pose.position.y,  
                          msg->pose.pose.position.z};

  Eigen::Vector3d vel = Eigen::Vector3d{msg->twist.twist.linear.x, 
                        msg->twist.twist.linear.y, 
                        msg->twist.twist.linear.z};

  // Get other agent's frame to map_frame transform
  try {
    auto tf = tf_buffer_->lookupTransform(
      map_frame_, msg->header.frame_id, 
      tf2::TimePointZero,
      tf2_ros::fromRclcpp(rclcpp::Duration::from_seconds(0.5)));
    // Set fixed map origin
    pose(0) += tf.transform.translation.x;
    pose(1) += tf.transform.translation.y;
  } 
  catch (const tf2::TransformException & ex) {
		RCLCPP_ERROR(
			this->get_logger(), "Could not get transform from map frame '%s' to agent frame '%s': %s",
			msg->header.frame_id.c_str(), map_frame_.c_str(), ex.what());
    return;
  }

  voxel_map_->updateSwarmState(id, pose, vel);

  if (id == drone_id_){
    return;
  }


  // /**
  //  * Clear reservation table and add to it
  //  */

  // if (!init_voro_maps_){
  //   return;
  // }

  // std::lock_guard<std::mutex> rsvn_tbl_guard(rsvn_tbl_mtx_);

  // double t_plan_start = ((double) msg->header.stamp.nanosec) * 1e-9;

  // rsvn_tbl_[id] = RsvnTbl(t_plan_start);
  // cells_inf_ = (int) std::lround(rsvn_tbl_inflation_/bool_map_3d_.resolution); // Number of cells used for inflation

  // Eigen::Vector3d lcl_map_start_pos = mapToLclMap(pose);
  // int z_cm =roundToMultInt(mToCm(lcl_map_start_pos(2)), 
  //                               bool_map_3d_.z_sep_cm,
  //                               bool_map_3d_.min_height_cm,
  //                               bool_map_3d_.max_height_cm);

  // // Iterate for a short duration along velocity vector
  // for (double t = 0; t <= 1.0; t += 0.05 )
  // {
  //   Eigen::Vector3d lcl_map_pos = lcl_map_start_pos + t * vel;

  //   IntPoint grid_pos;
  //   // get map position relative to local origin and set the obstacle 
  //   {
  //     std::lock_guard<std::mutex> voro_map_guard(roadmap_mtx_);

  //     bool within_lcl_map = dyn_voro_arr_[z_cm]->posToIdx(
  //       DblPoint(lcl_map_pos(0), lcl_map_pos(1)), grid_pos);
  //     if (!within_lcl_map){ // skip current point if not in map
  //       continue;
  //     }
  //   }

  //   // Inflate the cells by the given inflation radius
  //   for(int x = grid_pos.x - cells_inf_; x <= grid_pos.x + cells_inf_; x++)
  //   {
  //     for(int y = grid_pos.y - cells_inf_; y <= grid_pos.y + cells_inf_; y++)
  //     {
  //       // Reserve for time interval from previous t to current t, including time buffer
  //       for (int j = t - t_buffer_; j <= t + t_buffer_; j++) { 
  //         rsvn_tbl_[id].table.insert(Eigen::Vector4i{x, y, z_cm, j});
  //       }
  //     }
  //   }
  // }

}

void Navigator::odomSubCB(const nav_msgs::msg::Odometry::UniquePtr& msg)
{
  cur_pos_= Eigen::Vector3d{msg->pose.pose.position.x, 
                            msg->pose.pose.position.y, 
                            msg->pose.pose.position.z};
  cur_vel_= Eigen::Vector3d{msg->twist.twist.linear.x, 
                            msg->twist.twist.linear.y, 
                            msg->twist.twist.linear.z};

  viz_helper_->pubText(cur_pos_, agent_id_text_pub_, drone_id_, map_frame_);
}

void Navigator::pointGoalSubCB(const geometry_msgs::msg::PoseStamped::UniquePtr msg)
{
    std::vector<Eigen::Vector3d> wp_vec;

    if (msg->header.frame_id == global_frame_)
    {
      // Transform from global to map frame
      wp_vec.push_back(worldToMap(Eigen::Vector3d(
        msg->pose.position.x, msg->pose.position.y, point_goal_height_)));
    }
    else if (msg->header.frame_id == map_frame_)
    {
      // Keep in map frame
      wp_vec.push_back(Eigen::Vector3d(
        msg->pose.position.x, msg->pose.position.y, point_goal_height_));
    }
    else {
      logger_->logError(strFmt("Only accepting goals in 'world' or '%s' frame, ignoring goals.", map_frame_));
      return;
    }

    waypoints_.addMultipleWP(wp_vec);
}

void Navigator::goalsSubCB(const gestelt_interfaces::msg::Goals::UniquePtr msg)
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
    last_plan_success_ = this->get_clock()->now().nanoseconds()/1e9;
}

void Navigator::planReqDbgSubCB(const gestelt_interfaces::msg::PlanRequest::UniquePtr msg)
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

}

} // namespace navigator

