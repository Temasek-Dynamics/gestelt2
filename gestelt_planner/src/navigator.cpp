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

  getParams();

	// Create callback groups
  planning_cb_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive); // for planning
  mapping_cb_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive); // for generating voronoi maps
  // swarm_plan_cb_group_ = this->create_callback_group(
  //   rclcpp::CallbackGroupType::Reentrant); 
  others_cb_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  initPubSubTimer();

  tm_front_end_plan_.updateID(drone_id_);
  tm_sfc_.updateID(drone_id_);
  tm_mpc_.updateID(drone_id_);
  tm_voro_gen_.updateID(drone_id_);
  tm_plan_pipeline_.updateID(drone_id_);
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

  // auto swarm_plan_sub_opt = rclcpp::SubscriptionOptions();
  // swarm_plan_sub_opt.callback_group = swarm_plan_cb_group_;

  /* Publishers */

  // Map publishers

  // TODO: Change to dynamic instead of hard-coded values
  for (int z_cm = min_height_cm_; 
        z_cm <= max_height_cm_; 
        z_cm += z_sep_cm_)
  {
    occ_map_pubs_[z_cm] = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "occ_map_" + std::to_string(z_cm), rclcpp::SensorDataQoS());
    voro_occ_grid_pubs_[z_cm] = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "voro_map_" + std::to_string(z_cm), rclcpp::SensorDataQoS());
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
  // voronoi_graph_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("voronoi_graph", 10);

  agent_id_text_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
    "agent_id_text", rclcpp::SensorDataQoS());
  plan_req_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
    "fe_plan_req", rclcpp::SensorDataQoS());
  fe_closed_list_viz_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
    "fe_plan/closed_list", rclcpp::SensorDataQoS());
  fe_plan_viz_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
    "fe_plan/viz", rclcpp::SensorDataQoS());

  mpc_pred_pos_pub_ = this->create_publisher<nav_msgs::msg::Path>(
    "mpc/traj", rclcpp::SensorDataQoS());

  poly_sfc_pub_ = this->create_publisher<decomp_ros_msgs::msg::PolyhedronArray>(
    "sfc", rclcpp::SensorDataQoS());

  /* Subscribers */
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", rclcpp::SensorDataQoS(), std::bind(&Navigator::odomSubCB, this, _1), others_sub_opt);

  plan_req_dbg_sub_ = this->create_subscription<gestelt_interfaces::msg::PlanRequest>(
    "plan_request_dbg", rclcpp::SystemDefaultsQoS(), std::bind(&Navigator::planReqDbgSubCB, this, _1));
  goals_sub_ = this->create_subscription<gestelt_interfaces::msg::Goals>(
    "goals", rclcpp::ServicesQoS(), std::bind(&Navigator::goalsSubCB, this, _1));
  point_goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "point_goal", rclcpp::SystemDefaultsQoS(), std::bind(&Navigator::pointGoalSubCB, this, _1));

  // Subscribe to odometry individually from each agent
  // for (int i = 0; i < num_drones_; i++){

  //   if (i == drone_id_){
  //     continue;
  //   }

  //   std::function<void(const nav_msgs::msg::Odometry::UniquePtr msg)> bound_callback_func =
  //     std::bind(&Navigator::swarmOdomCB, this, _1, i);

  //   swarm_odom_subs_.push_back(
  //     this->create_subscription<nav_msgs::msg::Odometry>(
  //       "/d"+ std::to_string(i) + "/" + "odom", 
  //       rclcpp::SensorDataQoS(), 
  //       bound_callback_func, 
  //       swarm_plan_sub_opt)
  //   );
  // }

  /* Timers */
	pub_state_timer_ = this->create_wall_timer((1.0/pub_state_freq_) *1000ms, 
                                            std::bind(&Navigator::pubStateTimerCB, this), 
                                            others_cb_group_);

	plan_fe_timer_ = this->create_wall_timer((1.0/fe_planner_freq_) *1000ms, 
                                            std::bind(&Navigator::planTimerCB, this), 
                                            planning_cb_group_);

	// gen_voro_map_timer_ = this->create_wall_timer((1.0/gen_voro_map_freq_) *1000ms, 
  //                                                 std::bind(&Navigator::genVoroMapTimerCB, this),
  //                                                 mapping_cb_group_);

	send_mpc_cmd_timer_ = this->create_wall_timer((1.0/ctrl_samp_freq_) *1000ms, 
                                                  std::bind(&Navigator::sendMPCCmdTimerCB, this),
                                                  others_cb_group_);
}

void Navigator::getParams()
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
  this->declare_parameter("local_map_frame", "lcl_map");
  this->declare_parameter("camera_frame", "camera_frame");
  this->declare_parameter("base_link_frame", "base_link");

  /* Voronoi map slicing*/
  this->declare_parameter(param_ns+".map_slicing.min_height_cm", -1);
  this->declare_parameter(param_ns+".map_slicing.max_height_cm",  -1);
  this->declare_parameter(param_ns+".map_slicing.z_sep_cm",  -1);
  this->declare_parameter(param_ns+".map_slicing.sample_thickness",  -1.0);

  /* Space time A* planner */
  this->declare_parameter(param_ns+".planner.goal_tolerance", 0.1);
  this->declare_parameter(param_ns+".planner.print_timer", false);
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
  this->declare_parameter(param_ns+".sfc.poly.sfc_sampling_interval", 10);

  /* MPC */
  this->declare_parameter(param_ns+".mpc.ctrl_samp_freq", 30.0);
  this->declare_parameter(param_ns+".mpc.ref_path_samp_interval", 1);

  this->declare_parameter(param_ns+".mpc.yaw_ctrl_flag", false);
  this->declare_parameter(param_ns+".mpc.yaw_lookahead_dist", 5);

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

  mpc_params_.v_min(0) = this->declare_parameter(param_ns+".mpc.vx_min", -10.0);
  mpc_params_.v_min(1) = this->declare_parameter(param_ns+".mpc.vy_min", -10.0);
  mpc_params_.v_min(2) = this->declare_parameter(param_ns+".mpc.vz_min", -10.0);
  mpc_params_.v_max(0) = this->declare_parameter(param_ns+".mpc.vx_max", 10.0);
  mpc_params_.v_max(1) = this->declare_parameter(param_ns+".mpc.vy_max", 10.0);
  mpc_params_.v_max(2) = this->declare_parameter(param_ns+".mpc.vz_max", 10.0);
  mpc_params_.a_min(0) = this->declare_parameter(param_ns+".mpc.ax_min", -20.0);
  mpc_params_.a_min(1) = this->declare_parameter(param_ns+".mpc.ay_min", -20.0);
  mpc_params_.a_min(2) = this->declare_parameter(param_ns+".mpc.az_min", -10.0);
  mpc_params_.a_max(0) = this->declare_parameter(param_ns+".mpc.ax_max", 20.0);
  mpc_params_.a_max(1) = this->declare_parameter(param_ns+".mpc.ay_max", 20.0);
  mpc_params_.a_max(2) = this->declare_parameter(param_ns+".mpc.az_max", 20.0);
  mpc_params_.u_min(0) = this->declare_parameter(param_ns+".mpc.ux_min", -50.0);
  mpc_params_.u_min(1) = this->declare_parameter(param_ns+".mpc.uy_min", -50.0);
  mpc_params_.u_min(2) = this->declare_parameter(param_ns+".mpc.uz_min", -50.0);
  mpc_params_.u_max(0) = this->declare_parameter(param_ns+".mpc.ux_max", 50.0);
  mpc_params_.u_max(1) = this->declare_parameter(param_ns+".mpc.uy_max", 50.0);
  mpc_params_.u_max(2) = this->declare_parameter(param_ns+".mpc.uz_max", 50.0);

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

  /* Voronoi */
  min_height_cm_ = this->get_parameter(param_ns+".map_slicing.min_height_cm").as_int();
  max_height_cm_ = this->get_parameter(param_ns+".map_slicing.max_height_cm").as_int();
  z_sep_cm_ = this->get_parameter(param_ns+".map_slicing.z_sep_cm").as_int();

  /* A* Planner */
  double goal_tol = this->get_parameter(param_ns+".planner.goal_tolerance").as_double();
  sqr_goal_tol_ = goal_tol * goal_tol;
  print_timer_ = this->get_parameter(param_ns+".planner.print_timer").as_bool();
  plan_once_ = this->get_parameter(param_ns+".planner.plan_once").as_bool();
  t_unit_ = this->get_parameter(param_ns+".planner.t_unit").as_double();
  output_json_filepath_ = this->get_parameter(param_ns+".planner.output_json_filepath").as_string();

  rsvn_tbl_inflation_ = this->get_parameter(param_ns+".reservation_table.inflation").as_double();
  rsvn_tbl_window_size_ = this->get_parameter(param_ns+".reservation_table.window_size").as_int();
  double rsvn_tbl_t_buffer = this->get_parameter(param_ns+".reservation_table.time_buffer").as_double();
  t_buffer_ = (int) std::lround(rsvn_tbl_t_buffer/t_unit_);  // [space-time units] for time buffer

  /* A* parameters */
  astar_params_.min_sqr_dist_obs = this->declare_parameter(param_ns+".planner.min_sqr_dist_obs", 2);
  astar_params_.max_iterations = this->declare_parameter(param_ns+".planner.max_iterations", 50000);
  astar_params_.tie_breaker = this->declare_parameter(param_ns+".planner.tie_breaker", 1.001);
  astar_params_.cost_function_type = this->declare_parameter(param_ns+".planner.cost_function_type", 1); 
  
  astar_params_.drone_id = drone_id_;
  astar_params_.debug_viz = true;
  astar_params_.t_unit = t_unit_;

  // Liu SFC Params
  sfc_params_.map_frame = map_frame_;
  sfc_params_.bbox_x = this->get_parameter(param_ns+".sfc.poly.bbox_x").as_double();
  sfc_params_.bbox_y = this->get_parameter(param_ns+".sfc.poly.bbox_y").as_double();
  sfc_params_.bbox_z = this->get_parameter(param_ns+".sfc.poly.bbox_z").as_double();
  sfc_params_.sfc_samp_intv = 
    this->get_parameter(param_ns+".sfc.poly.sfc_sampling_interval").as_int();

  /* Debug commands */
  dbg_fixed_yaw_ = this->declare_parameter(param_ns+".dbg_fixed_yaw", 0.0);
  dbg_fixed_yaw_rate_ = this->declare_parameter(param_ns+".dbg_fixed_yaw_rate", 0.0);

  /* MPC */
  samp_mpc_tol_ = this->declare_parameter(param_ns+".samp_mpc_tolerance", 0.5);

  ctrl_samp_freq_ = this->get_parameter(param_ns+".mpc.ctrl_samp_freq").as_double();
  ref_samp_intv_  = this->get_parameter(param_ns+".mpc.ref_path_samp_interval").as_int();

  mpc_params_.drone_id = drone_id_;

  mpc_params_.yaw_ctrl_flag  = this->get_parameter(param_ns+".mpc.yaw_ctrl_flag").as_bool();
  yaw_lookahead_dist_  = 
    this->get_parameter(param_ns+".mpc.yaw_lookahead_dist").as_int();

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
  std::lock_guard<std::mutex> mpc_pred_lk(mpc_pred_mtx_);

  if (mpc_pred_pos_prev_.empty() 
      || mpc_pred_vel_prev_.empty() 
      || mpc_pred_acc_prev_.empty())
  {
    return;
  }

  double t_start = this->get_clock()->now().nanoseconds() / 1e9;
  // Get time t relative to start of MPC trajectory
  double e_t_start = t_start - last_mpc_solve_; 
  double total_traj_duration = (int)mpc_pred_pos_prev_.size() * mpc_controller_->MPC_STEP;
  
  if (e_t_start < 0.0 || e_t_start >= total_traj_duration)
  {
    // Exceeded duration of trajectory or trajectory timestamp invalid
    logger_->logWarn("MPC Trajectory is supposed to start in the future or has finished execution! Clearing previously predicted MPC path");
    mpc_pred_pos_prev_.clear();
    mpc_pred_vel_prev_.clear();
    mpc_pred_acc_prev_.clear();

    return;
  }

  // Get index of point closest to current time
  //  elapsed time from start divided by time step of MPC
  int idx =  std::floor(e_t_start / mpc_controller_->MPC_STEP); 

  if (idx >= (int) mpc_pred_pos_prev_.size()){
    logger_->logWarn("Sampling of MPC Trajectory has exceeded trajectory duration! Clearing previously predicted MPC path");
    mpc_pred_pos_prev_.clear();
    mpc_pred_vel_prev_.clear();
    mpc_pred_acc_prev_.clear();

    return;
  }

  // Publish PVA command
  // Send msg in ENU frame
  mavros_msgs::msg::PositionTarget cmd_msg;

  cmd_msg.header.stamp = this->get_clock()->now();
  cmd_msg.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
  cmd_msg.type_mask = mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;

  cmd_msg.position.x = mpc_pred_pos_prev_[idx](0);
  cmd_msg.position.y = mpc_pred_pos_prev_[idx](1);
  cmd_msg.position.z = mpc_pred_pos_prev_[idx](2);
  cmd_msg.velocity.x = mpc_pred_vel_prev_[idx](0);
  cmd_msg.velocity.y = mpc_pred_vel_prev_[idx](1);
  cmd_msg.velocity.z = mpc_pred_vel_prev_[idx](2);
  cmd_msg.acceleration_or_force.x = mpc_pred_acc_prev_[idx](0);
  cmd_msg.acceleration_or_force.y = mpc_pred_acc_prev_[idx](1);
  cmd_msg.acceleration_or_force.z = mpc_pred_acc_prev_[idx](2);

  cmd_msg.yaw = mpc_yaw_yawrate_(0);
  cmd_msg.yaw_rate = mpc_yaw_yawrate_(1);

	lin_mpc_cmd_pub_->publish(cmd_msg);
}

void Navigator::planTimerCB()
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
  if (!plan(goal)){
    tm_voro_gen_.stop(false);
    tm_front_end_plan_.stop(false);
    tm_sfc_.stop(false);
    tm_mpc_.stop(false);
    tm_plan_pipeline_.stop(false);
    return;
  }
  
  last_plan_success_ = this->get_clock()->now().nanoseconds()/1e9;
}

void Navigator::genVoroMapTimerCB()
{
  // if (!voxel_map_->getBoolMap3D(bool_map_3d_)){
  //   return;
  // }

  // std::unique_lock<std::mutex> map_lk(roadmap_mtx_);
  // prod_map_cv_.wait(map_lk, [&]{return !map_ready_for_cons_;}); // wait for voronoi map to be consumed

  // tm_voro_gen_.start();

  // // Initialize dynamic voronoi 

  // for (auto const& bool_map : bool_map_3d_.bool_maps) // For each boolean map
  // {
  //   int z_cm = bool_map.first;
  //   double z_m = cmToM(bool_map.first);

  //   // Create DynamicVoronoi object if it does not exist
  //   dynamic_voronoi::DynamicVoronoiParams dyn_voro_params;
  //   dyn_voro_params.res = bool_map_3d_.resolution;
  //   dyn_voro_params.origin_z = z_m;
  //   dyn_voro_params.origin_z_cm = z_cm;
  //   // dyn_voro_params.origin_x = 0.0;
  //   // dyn_voro_params.origin_y = 0.0;

  //   dyn_voro_arr_[z_cm] = std::make_shared<dynamic_voronoi::DynamicVoronoi>();
  //   dyn_voro_arr_[z_cm]->setParams(dyn_voro_params);
  //   // Map is received in local_map_frame_
  //   dyn_voro_arr_[z_cm]->initializeMap( bool_map_3d_.width, 
  //                                       bool_map_3d_.height, 
  //                                       bool_map.second);

  //   dyn_voro_arr_[z_cm]->update(); // update distance map and Voronoi diagram
  //   dyn_voro_arr_[z_cm]->updateAlternativePrunedDiagram();
  //   // dyn_voro_arr_[z_cm]->prune();  // prune the Voronoi

  //   nav_msgs::msg::OccupancyGrid occ_grid, voro_occ_grid;

  //   occmapToOccGrid(*dyn_voro_arr_[z_cm], 
  //                   bool_map_3d_.origin(0), bool_map_3d_.origin(1), 
  //                   occ_grid); // Occupancy map

  //   voronoimapToOccGrid(*dyn_voro_arr_[z_cm], 
  //                       bool_map_3d_.origin(0), bool_map_3d_.origin(1), 
  //                       voro_occ_grid); // Voronoi map

  //   voro_occ_grid_pubs_[z_cm]->publish(voro_occ_grid);
  //   occ_map_pubs_[z_cm]->publish(occ_grid);

  // }

  // tm_voro_gen_.stop(false);
  // viz_helper_->pubVoroVertices(voro_verts, voronoi_graph_pub_, local_map_frame_);

  // Notify the planning thread that the voronoi map is ready for consumption
  // map_ready_for_cons_ = true;
  // cons_map_cv_.notify_one();
}

/* Core methods */

bool Navigator::plan(const Eigen::Vector3d& goal_pos){
  // logger_->logInfo(strFmt(" Drone %d: Before planner", drone_id_));

  if (plan_once_ && plan_complete_){
    return true;
  }

  tm_voro_gen_.start();

  if (!voxel_map_->getBoolMap3D(bool_map_3d_)){
    return false;
  }

  tm_plan_pipeline_.start();

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

  }

  tm_voro_gen_.stop(false);

  /*****/
  /* 1) Assign voronoi map and reservation table to planner */
  /*****/

  // logger_->logInfo(strFmt("   Drone %d: setVoroMap", drone_id_));

  // Assign reservation table
  // {
    // std::lock_guard<std::mutex> rsvn_tbl_guard(rsvn_tbl_mtx_);
    // fe_planner_->setReservationTable(rsvn_tbl_);
  // }

  // Wait for condition variable notification
  // std::unique_lock<std::mutex> map_lk(roadmap_mtx_);
  // cons_map_cv_.wait(map_lk, [&]{return map_ready_for_cons_;});

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

  if (prev_plan_fail_ || !sampleMPCTrajectory(mpc_controller_, plan_start_clock.nanoseconds()/1e9, 
                          start_pos, start_vel, start_acc))
  {
    start_pos = cur_pos_;
    start_vel.setZero();
    start_acc.setZero();
  }

  /*****/
  /* 3) Get Receding Horizon Planning goal */
  /*****/

  // logger_->logInfo(strFmt("   Drone %d: RHP", drone_id_));

  // Get RHP goal
  Eigen::Vector3d rhp_goal_pos = getRHPGoal(
    start_pos, goal_pos, 
    voxel_map_->getLocalMapOrigin(0.15), voxel_map_->getLocalMapMax(0.15));

  // Publish start and goal visualization
  viz_helper_->pubPlanRequestViz(
    start_pos, rhp_goal_pos, goal_pos, plan_req_pub_, map_frame_);

  /*****/
  /* 4) Plan HCA* path */
  /* Planning is performed in local map frame*/
  /*****/

  // logger_->logInfo(strFmt("   Drone %d: Front-end", drone_id_));

  tm_front_end_plan_.start();

  bool fe_plan_success = 
    fe_planner_->generatePlan(mapToLclMap(start_pos), mapToLclMap(rhp_goal_pos));

  // Notify the voronoi map generation thread that plan is complete
  // map_ready_for_cons_ = false;
  // prod_map_cv_.notify_one();

  tm_front_end_plan_.stop(print_timer_);
  tm_front_end_plan_.getWallAvg(print_timer_);

  if (!fe_plan_success)
  {
    logger_->logError(strFmt("Drone %d: Failed to generate FE plan from (%f, %f, %f) \
                              to (%f, %f, %f)", 
                              drone_id_, 
                              start_pos(0), start_pos(1), start_pos(2), 
                              rhp_goal_pos(0), rhp_goal_pos(1), rhp_goal_pos(2)));
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

  // logger_->logInfo(strFmt("   Drone %d: Transform path", drone_id_));

  // Current pose must converted from fixed map frame to local map frame 
  // before passing to planner
  // std::vector<Eigen::Vector4d> fe_path_with_t_lclmapframe = fe_planner_->getPathWithTime(mapToLclMap(cur_pos_));
  std::vector<Eigen::Vector4d> fe_path_with_t_lclmapframe = 
    fe_planner_->getPathWithTime();
  std::vector<Eigen::Vector4d> fe_path_with_t_lclmapframe_smoothed = 
    fe_planner_->getPathWithTimeSampled(sfc_params_.sfc_samp_intv);
  // fe_sfc_segment_idx: Index of sampled original path. Indexed by polygon number. 
  //    The value is the front-end index at that polygon's starting segment
  std::vector<int> fe_sfc_segment_idx = fe_planner_->getPathIdxSampled(sfc_params_.sfc_samp_intv);

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

  // logger_->logInfo(strFmt("   Drone %d: before generateSFC", drone_id_));

  tm_sfc_.start();
  bool gen_sfc_success = 
    poly_sfc_gen_->generateSFC(voxel_map_->getLclObsPts(), fe_path_smoothed_);
  tm_sfc_.stop(print_timer_);
  tm_sfc_.getWallAvg(print_timer_);


  // logger_->logInfo(strFmt("   Drone %d: After generateSFC", drone_id_));

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
  int fe_start_idx = 0; // A* index nearest to initial condition (current odom of drone)
  double min_dis = std::numeric_limits<double>::max();

  // TODO: improve this routine using KDTree
  for (int i = 0; i < (int)fe_path_.size() ; ++i) { // find the nearest point
    double dis = (cur_pos_ - fe_path_[i]).norm();
    if (dis < min_dis) {
        min_dis = dis;
        fe_start_idx = i;
    }
  }

  auto constrainPi = [](const double& x) -> double
  {
    double x_out = x;
    if (x_out > M_PI)
      x_out -= 2 * M_PI;

    if (x_out < -M_PI)
      x_out += 2 * M_PI;

    return x_out;
  };


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

  tm_mpc_.start();

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
  Eigen::Vector3d last_p_ref = cur_pos_; // last position reference
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

    if (i == mpc_controller_->MPC_HORIZON-1){
      // do nothing, set as zero
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

  bool mpc_success = mpc_controller_->run();
  tm_mpc_.stop(print_timer_);
  tm_mpc_.getWallAvg(print_timer_);

  if (!mpc_success){ // Successful MPC solve
    logger_->logError("MPC Solve failure!");
    return false;
  }

  // Check if MPC command values are valid
  auto checkValidCmd = [&](const Eigen::Vector3d& vec, const int& min_val, const int& max_val){
      for (const auto& val : vec){
        if (val < min_val || val > max_val){
          return false;
        }
      }
    return true;
  };

  Eigen::MatrixXd A1, B1; // system transition matrix

  mpc_controller_->getSystemModel(A1, B1, mpc_controller_->MPC_STEP);
  Eigen::VectorXd x_optimal = mpc_controller_->X_0_;

  {
    mpc_pred_u_.clear();
    mpc_pred_pos_.clear();
    mpc_pred_vel_.clear();
    mpc_pred_acc_.clear();

    // Get predicted MPC path based on controls and check if valid
    bool valid_cmd = true;
    Eigen::Vector3d u_optimal; 
    for (int i = 0; i < mpc_controller_->MPC_HORIZON; i++) {
        mpc_controller_->getOptimalControl(u_optimal, i);
        x_optimal = A1 * x_optimal + B1 * u_optimal;

        // Check if position, velocity and acceleration at valid values
        if (!checkValidCmd(x_optimal.segment<3>(0), -100.0, 100.0) ){
          logger_->logError(strFmt("Invalid MPC pos: (%f, %f, %f) at idx %d"
            ,x_optimal.segment<3>(0)(0), x_optimal.segment<3>(0)(1), x_optimal.segment<3>(0)(2), i)) ;
          valid_cmd = false;
          break;
        }
        if (!checkValidCmd(x_optimal.segment<3>(3), -5.0, 5.0) ){
          logger_->logError(strFmt("Invalid MPC vel: (%f, %f, %f) at idx %d"
            ,x_optimal.segment<3>(3)(0), x_optimal.segment<3>(3)(1), x_optimal.segment<3>(3)(2), i)) ;
          valid_cmd = false;
          break;
        }
        if (!checkValidCmd(x_optimal.segment<3>(6), -60.0, 60.0) ){
          logger_->logError(strFmt("Invalid MPC acc: (%f, %f, %f) at idx %d"
            ,x_optimal.segment<3>(6)(0), x_optimal.segment<3>(6)(1), x_optimal.segment<3>(6)(2), i)) ;
          valid_cmd = false;
          break;
        }

        mpc_pred_u_.push_back(u_optimal);
        mpc_pred_pos_.push_back(x_optimal.segment<3>(0));
        mpc_pred_vel_.push_back(x_optimal.segment<3>(3));
        mpc_pred_acc_.push_back(x_optimal.segment<3>(6));
    }

    if (valid_cmd){
      std::lock_guard<std::mutex> mpc_pred_lk(mpc_pred_mtx_);
      mpc_pred_u_prev_ = mpc_pred_u_;
      mpc_pred_pos_prev_ = mpc_pred_pos_;
      mpc_pred_vel_prev_ = mpc_pred_vel_;
      mpc_pred_acc_prev_ = mpc_pred_acc_;

      prev_plan_fail_ = false;

      last_mpc_solve_ = this->get_clock()->now().nanoseconds() / 1e9;
    }
    else {
      mpc_pred_u_prev_.clear();
      mpc_pred_pos_prev_.clear();
      mpc_pred_vel_prev_.clear();
      mpc_pred_acc_prev_.clear();

      prev_plan_fail_ = true;

      return false;
    }

  }

  // if (mpc_controller_->yaw_ctrl_flag_){
  //   double cmd_yaw = 0.0;
  //   double dt = 0.1;

  //   // Calculate commanded yaw
  //   int lookahead_idx = yaw_lookahead_dist_;
  //   std::cout << "yaw_lookahead_dist_: " << yaw_lookahead_dist_ << std::endl;

  //   if (0 < fe_path_.size() - lookahead_idx - 1) 
  //   {

  //     Eigen::Vector2d dir_vec(
  //       fe_path_[lookahead_idx](1) - cur_pos_(1),
  //       fe_path_[lookahead_idx](0) - cur_pos_(0)
  //     );
  //     std::cout << "within lookahead, dir_vec: " << dir_vec.transpose() << std::endl;

  //     cmd_yaw = std::atan2(dir_vec(0), dir_vec(1));
  //   }
  //   else {
  //     Eigen::Vector2d dir_vec(
  //       fe_path_.back()(1) - cur_pos_(1),
  //       fe_path_.back()(0) - cur_pos_(0)
  //     );
  //     std::cout << "dir_vec: " << dir_vec.transpose() << std::endl;

  //     cmd_yaw = std::atan2(dir_vec(0), dir_vec(1));
  //   }

  //   std::cout << "cmd_yaw: " << cmd_yaw << std::endl;

  //   double yawdot = 0;
  //   double d_yaw = cmd_yaw - mpc_yaw_yawrate_(0);

  //   d_yaw = constrainPi(d_yaw);

  //   const double YDM = d_yaw >= 0 ? YAW_DOT_MAX_PER_SEC : -YAW_DOT_MAX_PER_SEC;
  //   const double YDDM = d_yaw >= 0 ? YAW_DOT_DOT_MAX_PER_SEC : -YAW_DOT_DOT_MAX_PER_SEC;
  //   double d_yaw_max;

  //   if (fabs(mpc_yaw_yawrate_(1) + dt * YDDM) <= fabs(YDM)) // Within yaw_dot limits
  //   {
  //     d_yaw_max = (mpc_yaw_yawrate_(1) * dt) + (0.5 * YDDM * dt * dt);
  //   }
  //   else { // exceed yaw_dot limits
  //     double t1 = (YDM - mpc_yaw_yawrate_(1)) / YDDM;
  //     d_yaw_max = ((dt - t1) + dt) * (YDM - mpc_yaw_yawrate_(1)) / 2.0;
  //   }

  //   if (fabs(d_yaw) > fabs(d_yaw_max))
  //   {
  //     d_yaw = d_yaw_max;
  //   }
  //   yawdot = d_yaw / dt;

  //   double yaw = mpc_yaw_yawrate_(0) + d_yaw;
  //   yaw = constrainPi(yaw);

  //   mpc_yaw_yawrate_(0) = yaw;
  //   mpc_yaw_yawrate_(1) = yawdot;

  // }
  // else {
  //   mpc_yaw_yawrate_(0) = dbg_fixed_yaw_;
  //   mpc_yaw_yawrate_(1) = dbg_fixed_yaw_rate_;
  // }


  if (mpc_controller_->yaw_ctrl_flag_){
    double cmd_yaw = 0.0;
    double dt = 0.1;

    // Calculate commanded yaw
    int lookahead_idx = yaw_lookahead_dist_;
    // std::cout << "yaw_lookahead_dist_: " << yaw_lookahead_dist_ << std::endl;

    if (0 < fe_path_.size() - lookahead_idx - 1) 
    {

      Eigen::Vector2d dir_vec(
        fe_path_[lookahead_idx](1) - cur_pos_(1),
        fe_path_[lookahead_idx](0) - cur_pos_(0)
      );
      // std::cout << "within lookahead, dir_vec: " << dir_vec.transpose() << std::endl;

      cmd_yaw = std::atan2(dir_vec(0), dir_vec(1));
    }
    else {
      Eigen::Vector2d dir_vec(
        fe_path_.back()(1) - cur_pos_(1),
        fe_path_.back()(0) - cur_pos_(0)
      );
      // std::cout << "dir_vec: " << dir_vec.transpose() << std::endl;

      cmd_yaw = std::atan2(dir_vec(0), dir_vec(1));
    }

    // std::cout << "cmd_yaw: " << cmd_yaw << std::endl;

    mpc_yaw_yawrate_(0) = cmd_yaw;
    mpc_yaw_yawrate_(1) = 0.0;

  }
  else {
    mpc_yaw_yawrate_(0) = dbg_fixed_yaw_;
    mpc_yaw_yawrate_(1) = dbg_fixed_yaw_rate_;
  }


  pubMPCPath(mpc_pred_pos_); // Publish MPC path for visualization

  tm_plan_pipeline_.stop(print_timer_);

  // logger_->logInfo(strFmt(" Drone %d: After planner", drone_id_));

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

  Eigen::Quaterniond q(msg->pose.pose.orientation.w,
                        msg->pose.pose.orientation.x,
                        msg->pose.pose.orientation.y,
                        msg->pose.pose.orientation.z);

  Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0); // In yaw, pitch, roll

  cur_yaw_ = euler(0);

  viz_helper_->pubText(cur_pos_, agent_id_text_pub_, drone_id_, map_frame_);
}

void Navigator::pointGoalSubCB(const geometry_msgs::msg::PoseStamped::UniquePtr msg)
{
  logger_->logInfo(strFmt("Received point goal in %s frame", msg->header.frame_id.c_str()));

  std::vector<Eigen::Vector3d> wp_vec;

  if (msg->header.frame_id == global_frame_)
  {
    // Transform from global to map frame
    Eigen::Vector3d src_pt(msg->pose.position.x, msg->pose.position.y, point_goal_height_);
    Eigen::Vector3d tgt_pt;
    if (!worldToMap(src_pt, tgt_pt)){
      logger_->logError(strFmt("Failed to transform goals from '%s' frame to %s frame, ignoring goal request.", 
        global_frame_.c_str(), map_frame_.c_str()));
      return;
    }

    wp_vec.push_back(tgt_pt);
    logger_->logInfo(strFmt("  Added global goal (%f, %f, %f)", tgt_pt(0), tgt_pt(1), tgt_pt(2)));
  }
  else if (msg->header.frame_id == map_frame_)
  {
    // Keep in map frame
    wp_vec.push_back(Eigen::Vector3d(
      msg->pose.position.x, msg->pose.position.y, point_goal_height_));
  }
  else {
    logger_->logError(strFmt("Only accepting goals in '%s' or '%s' frame, ignoring goals.", 
      global_frame_.c_str(), map_frame_.c_str()));
    return;
  }
  waypoints_.reset();

  waypoints_.addMultipleWP(wp_vec);
}

void Navigator::goalsSubCB(const gestelt_interfaces::msg::Goals::UniquePtr msg)
{
  logger_->logInfo(strFmt("Received goals in %s frame", msg->header.frame_id.c_str()));

    if (msg->waypoints.empty())
    {
      logger_->logError(strFmt("Received empty waypoints. Ignoring waypoints."));

      return;
    }

    std::vector<Eigen::Vector3d> wp_vec;

    if (msg->header.frame_id == global_frame_)
    {
      // Transform from world to fixed map frame
      for (auto& wp : msg->waypoints) {
        Eigen::Vector3d src_pt(wp.position.x, wp.position.y, wp.position.z);
        Eigen::Vector3d tgt_pt;
        if (!worldToMap(src_pt, tgt_pt)){
          logger_->logError(strFmt("Failed to transform goals from '%s' frame to %s frame, ignoring goal request.", 
            global_frame_.c_str(), map_frame_.c_str()));
          return;
        }
        wp_vec.push_back(tgt_pt);
        logger_->logInfo(strFmt("  Added global goal (%f, %f, %f)", tgt_pt(0), tgt_pt(1), tgt_pt(2)));
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
      logger_->logError(strFmt("Only accepting goals in '%s' or '%s' frame, ignoring goal request.", 
        global_frame_.c_str(), map_frame_.c_str()));
      return;
    }

    waypoints_.addMultipleWP(wp_vec);
    last_plan_success_ = this->get_clock()->now().nanoseconds()/1e9;
}

void Navigator::planReqDbgSubCB(const gestelt_interfaces::msg::PlanRequest::UniquePtr msg)
{
  Eigen::Vector3d plan_end;

  if (msg->header.frame_id == map_frame_)
  {
    // Keep in map frame
    cur_pos_ = Eigen::Vector3d(
      msg->start.position.x, msg->start.position.y, msg->start.position.z);
    plan_end = Eigen::Vector3d(
      msg->goal.position.x, msg->goal.position.y, msg->goal.position.z);
  }
  else {
    logger_->logError(strFmt("Only accepting goals in '%s' frame, ignoring goals.", 
      map_frame_.c_str()));
    return;
  }

  std::cout << "Agent " << msg->agent_id << ": plan request from ("<< 
                cur_pos_.transpose() << ") to (" << plan_end.transpose() << ")" << std::endl;

}

} // namespace navigator

