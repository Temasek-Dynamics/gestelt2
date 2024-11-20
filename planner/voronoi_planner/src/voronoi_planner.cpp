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
    
  } 
  catch (const tf2::TransformException & ex) {
		RCLCPP_ERROR(
			this->get_logger(), "Could not get transform from world_frame('world') to map_frame_(%s): %s",
			map_frame_.c_str(), ex.what());
    rclcpp::shutdown();
    return;
  }

  // Initialize planner
  fe_planner_ = std::make_unique<global_planner::SpaceTimeAStar>(astar_params_, this->get_clock());

  // Initialize safe flight corridor
  poly_sfc_gen_ = std::make_unique<sfc::PolytopeSFC>(sfc_params_);

  // Initialize MPC controller
  mpc_ = std::make_unique<pvaj_mpc::MPCController>(mpc_params_);

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

  /* Planner publishers */
  fe_plan_broadcast_pub_ = this->create_publisher<gestelt_interfaces::msg::SpaceTimePath>(
    "fe_plan/broadcast", rclcpp::SensorDataQoS());

  lin_mpc_cmd_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
    "lin_mpc_cmd", rclcpp::SensorDataQoS());

  poly_traj_pub_ = this->create_publisher<minco_interfaces::msg::PolynomialTrajectory>(
    "poly_traj", rclcpp::SensorDataQoS());
  minco_traj_broadcast_pub_ = this->create_publisher<minco_interfaces::msg::MincoTrajectory>(
    "minco_traj/broadcast", rclcpp::SensorDataQoS());

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
    "odom", rclcpp::SensorDataQoS(), std::bind(&VoronoiPlanner::odomSubCB, this, _1), others_sub_opt);

  plan_req_dbg_sub_ = this->create_subscription<gestelt_interfaces::msg::PlanRequest>(
    "plan_request_dbg", rclcpp::SystemDefaultsQoS(), std::bind(&VoronoiPlanner::planReqDbgSubCB, this, _1));
  goals_sub_ = this->create_subscription<gestelt_interfaces::msg::Goals>(
    "goals", rclcpp::SystemDefaultsQoS(), std::bind(&VoronoiPlanner::goalsSubCB, this, _1));
  point_goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "point_goal", rclcpp::SystemDefaultsQoS(), std::bind(&VoronoiPlanner::pointGoalSubCB, this, _1));

  // Subscribe to front end plan individually from each agent
  if (plan_method_ == COMMLESS_MINCO || plan_method_ == COMMLESS_MPC){
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

  /* Timers */
	plan_fe_timer_ = this->create_wall_timer((1.0/fe_planner_freq_) *1000ms, 
                                            std::bind(&VoronoiPlanner::planFETimerCB, this), 
                                            planning_cb_group_);

	gen_voro_map_timer_ = this->create_wall_timer((1.0/gen_voro_map_freq_) *1000ms, 
                                                  std::bind(&VoronoiPlanner::genVoroMapTimerCB, this),
                                                  planning_cb_group_);

	send_mpc_cmd_timer_ = this->create_wall_timer((1.0/ctrl_samp_freq_) *1000ms, 
                                                  std::bind(&VoronoiPlanner::sendMPCCmdTimerCB, this),
                                                  others_cb_group_);
}

void VoronoiPlanner::initParams()
{
  std::string param_ns = "navigator";
  
  this->declare_parameter("drone_id", -1);
  this->declare_parameter(param_ns+".num_drones", 4);
  this->declare_parameter(param_ns+".plan_method", -1);
  this->declare_parameter(param_ns+".generate_voronoi_frequency", 10.0);
  this->declare_parameter(param_ns+".planner_frequency", 10.0);

  /* Frame ids */
  this->declare_parameter("map_frame", "map");
  this->declare_parameter("local_map_frame", "local_map_frame");

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
  fe_planner_freq_ = this->get_parameter(param_ns+".planner_frequency").as_double();
  gen_voro_map_freq_ = this->get_parameter(param_ns+".generate_voronoi_frequency").as_double();

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

/* Core methods */
void VoronoiPlanner::plan(const Eigen::Vector3d& goal_pos){
  switch (plan_method_) {
    case PlanMethod::COMMLESS_MINCO:

      if (!planCommlessMINCO(goal_pos)){
        auto plan_start_clock = this->get_clock()->now();

        // If previous trajectory is still valid, follow previous path
        if (poly_traj_ != nullptr && poly_traj_->getGlobalStartTime() > 0.0)
        {
          double e_t_start = plan_start_clock.nanoseconds()/1e9 - poly_traj_->getGlobalStartTime(); // Get time t relative to start of trajectory

          if (e_t_start > 0.0 && e_t_start < poly_traj_->getTotalDuration())
          {
            // Trajectory is still executing
            logger_->logWarn("Following previous trajectory!");
            return;
          }
        }

      }

      break;
    case PlanMethod::COMMLESS_MPC:
      if (!planCommlessMPC(goal_pos)){

      }

      break;
    case PlanMethod::COMM_MINCO:
      if (!planCommMINCO(goal_pos)){
        auto plan_start_clock = this->get_clock()->now();

        // If previous trajectory is still valid, follow previous path
        if (poly_traj_ != nullptr && poly_traj_->getGlobalStartTime() > 0.0)
        {
          double e_t_start = plan_start_clock.nanoseconds()/1e9 - poly_traj_->getGlobalStartTime(); // Get time t relative to start of trajectory

          if (e_t_start > 0.0 && e_t_start < poly_traj_->getTotalDuration())
          {
            // Trajectory is still executing
            logger_->logWarn("Following previous trajectory!");
            return;
          }
        }
        else // Else generate trajectory staying in it's current position
        {

          gestelt_interfaces::msg::SpaceTimePath fe_plan_msg;

          fe_plan_msg.agent_id = drone_id_;
          fe_plan_msg.header.stamp = plan_start_clock;
          fe_plan_msg.t_plan_start = plan_start_clock.nanoseconds()/1e9;

          for (size_t i = 0; i < (size_t) rsvn_tbl_window_size_; i++){
            geometry_msgs::msg::Pose pose;
            pose.position.x = cur_pos_(0); 
            pose.position.y = cur_pos_(1);
            pose.position.z = cur_pos_(2);
            pose.orientation.w = 1.0; 

            fe_plan_msg.plan.push_back(pose);
            fe_plan_msg.plan_time.push_back(i);
          }

          logger_->logWarn("Staying at current position!");
          fe_plan_broadcast_pub_->publish(fe_plan_msg);
        }
      }

      break;
    default:
      logger_->logError("Undefined planning method! Please set valid parameters for 'plan_method'");
      return;
      break;
  }
}

bool VoronoiPlanner::planCommlessMPC(const Eigen::Vector3d& goal_pos){
  if (!init_voro_maps_){
    logger_->logInfo("Voronoi maps not initialized! Request a plan after initialization!");
    return false;
  }

  /*****/
  /* 1) Assign voronoi map and reservation table to planner */
  /*****/

  // Assign reservation table
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

  /*****/
  /* 2) Get current position, velocity and acceleration */
  /*****/

  auto plan_start_clock = this->get_clock()->now();

  Eigen::Vector3d start_pos, start_vel, start_acc;

  if (!sampleMPCTrajectory(mpc_, plan_start_clock.nanoseconds()/1e9, 
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


  /*****/
  /* 5) Retrieve and transform HCA* path to map frame, publish visualization of path */
  /*****/

  // Current pose must converted from fixed map frame to local map frame 
  // before passing to planner
  // std::vector<Eigen::Vector4d> fe_path_with_t_lclmapframe = fe_planner_->getPathWithTime(mapToLclMap(cur_pos_));
  std::vector<Eigen::Vector4d> fe_path_with_t_lclmapframe = fe_planner_->getPathWithTime();
  std::vector<Eigen::Vector4d> fe_path_with_t_lclmapframe_smoothed = fe_planner_->getSmoothedPathWithTime();
  std::vector<int> fe_path_smoothed_idx = fe_planner_->getSmoothedPathIdx(); // Index of smoothed path in original path

  // std::cout << "fe_path_smoothed_idx with size " << fe_path_smoothed_idx.size() << std::endl;
  // for (int i = 0; i < fe_path_smoothed_idx.size(); ++i)
  // {
  //   std::cout << "  (" << i <<"): " << fe_path_smoothed_idx[i] << std::endl;
  // }

  auto transformPathFromLclMapToMap = [&](const std::vector<Eigen::Vector4d>& map_path_w_t_lclmapframe, 
                             std::vector<Eigen::Vector4d>& map_path_w_t,
                             std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& map_path) 
  {
    map_path.clear();
    map_path_w_t.clear();
    
    for (size_t i = 0; i < map_path_w_t_lclmapframe.size(); i++){
      Eigen::Vector4d map_pt;
      map_pt.head<3>() = lclMapToMap(map_path_w_t_lclmapframe[i].head<3>());
      map_pt(3) = map_path_w_t_lclmapframe[i](3);

      map_path.push_back(map_pt.head<3>());
      map_path_w_t.push_back(map_pt);
    }
  };

  // Convert front-end path from local map frame to fixed map frame
  transformPathFromLclMapToMap(fe_path_with_t_lclmapframe, fe_path_with_t_, fe_path_);
  transformPathFromLclMapToMap(fe_path_with_t_lclmapframe_smoothed, fe_path_with_t_smoothed_, fe_path_smoothed_);
  // Visualize front-end path
  viz_helper_->pubFrontEndPath(fe_path_, fe_plan_viz_pub_, map_frame_);

  /*****/
  /* 6) Generate Polyhedron safe flight corridor */
  /*****/

  if (!poly_sfc_gen_->generateSFC(voxel_map_->getLclObsPts(), fe_path_smoothed_))
  {
    logger_->logError("Failed to generate safe flight corridor!");
    return false;
  }

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
  for (int i = 0; i < (int) fe_path_.size(); ++i) { // find the nearest point
    double dis = (cur_pos_ - fe_path_[i]).norm();
    if (dis < min_dis) {
        min_dis = dis;
        fe_start_idx = i;
    }
  }

  // 7a) Set Safe Flight corridor

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
      planes.row(i) << -normal(0), -normal(1), -normal(2), -d;
    }

    // D = - [A, B, C].T * [x, y, z]

    // planes.resize(6, 4); // Ax + By + Cz + D = 0, D = -(Ax + By + Cz)

    // planes.row(0) <<  1,  0,  0, -100;
    // planes.row(1) <<  0,  1,  0, -100;
    // planes.row(2) <<  0,  0,  1, -100;
    
    // planes.row(3) << -1,  0,  0,  -100;
    // planes.row(4) <<  0, -1,  0,  -100;
    // planes.row(5) <<  0,  0, -1,  -100;

  };

  // int poly_idx = 0;
  // for (int i = 0; i < mpc_->MPC_HORIZON; i++) // for each MPC point
  // {
  //   int idx = fe_start_idx + i * ref_samp_intv_; // Current FE path index

  //   int cur_poly_start_idx = fe_path_smoothed_idx[poly_idx];
  //   int cur_poly_end_idx = fe_path_smoothed_idx[poly_idx+1];

  //   if (idx >= cur_poly_start_idx && idx >= cur_poly_end_idx)
  //   { // if idx 
  //     poly_idx++; // Iterate to next polygon
 
  //     if (poly_idx > polyhedrons.size() -1){
  //       logger_->logError("CODE ERROR! j exceeds size of polyehdrons");
  //     }
  //   }

  //   Eigen::MatrixX4d planes;
  //   polyhedronToPlanes(polyhedrons[poly_idx], planes);

  //   mpc_->setFSC(planes, i); 
  // }

  for (int i = 0, poly_idx = 0; i < mpc_->MPC_HORIZON; i++) // for each MPC point
  {
    int idx = fe_start_idx + i * ref_samp_intv_; // Current FE path index

    int poly_start_idx = fe_path_smoothed_idx[poly_idx];
    int poly_end_idx = fe_path_smoothed_idx[poly_idx+1];

    if (idx >= poly_end_idx) {
      logger_->logInfo(strFmt("idx(%d) >= %d", idx, poly_start_idx ));
      poly_idx++; // Iterate to next polygon

      if (poly_idx > polyhedrons.size() -1){
        logger_->logError(" CODE ERROR! poly_idx exceeds size of polyehdrons");
        poly_idx = polyhedrons.size() -1;
      }
    }

    Eigen::MatrixX4d planes;
    polyhedronToPlanes(polyhedrons[poly_idx], planes);

    if (!mpc_->isInFSC(fe_path_[idx], planes)) { // if not in sfc
      logger_->logError(strFmt("  CODE ERROR! idx %d is not in polygon %d", idx, poly_idx ));
    }

    mpc_->setFSC(planes, i); 
  }

  // 7b) Set reference path for MPC 
  Eigen::Vector3d last_p_ref; // last position reference
  for (int i = 0; i < mpc_->MPC_HORIZON; i++) {
    int idx = fe_start_idx + i * ref_samp_intv_;
    idx = (idx >= (int)fe_path_.size()) ? fe_path_.size() - 1 : idx; 

    Eigen::Vector3d p_ref = fe_path_[idx]; // position reference
    Eigen::Vector3d v_ref(0, 0, 0); // vel reference
    Eigen::Vector3d a_ref(0, 0, 0); // acc reference

    if (i == 0) // First iteration
      v_ref = (p_ref - cur_pos_) / mpc_->MPC_STEP;
    else 
      v_ref = (p_ref - last_p_ref) / mpc_->MPC_STEP;

    // Set PVA reference at given step i
    mpc_->setReference(p_ref, v_ref, a_ref, i);

    last_p_ref = p_ref;
  }

  // 7c) Solve MPC and visualize path

  // Set initial condition
  mpc_->setInitialCondition(start_pos, start_vel, start_acc);

  Eigen::Vector3d u_optimal, p_optimal, v_optimal, a_optimal, u_predict;
  Eigen::Vector2d yaw_yawrate{0.0, NAN};
  Eigen::MatrixXd A1, B1; // system transition matrix
  Eigen::VectorXd x_optimal = mpc_->X_0_;

  bool mpc_success;

  {
    mpc_success = mpc_->run();
    last_mpc_solve_ = this->get_clock()->now().nanoseconds() / 1e9;
  }

  if (mpc_success){ // Successful MPC solve

    mpc_->getOptimalControl(u_optimal, 0);
    mpc_->getSystemModel(A1, B1, mpc_->MPC_STEP);

    x_optimal = A1 * x_optimal + B1 * u_optimal;

    p_optimal = x_optimal.segment<3>(0); 
    v_optimal = x_optimal.segment<3>(3);
    a_optimal = x_optimal.segment<3>(6); 

    // Set yaw
    // TODO: figure out a better way
    // int lk_ahd = 5; // Look ahead distance
    // int lk_ahd_idx = (fe_start_idx + lk_ahd) < fe_path_.size() ? fe_start_idx + lk_ahd : fe_start_idx;
    // yaw_yawrate(0) = std::atan2(fe_path_[lk_ahd_idx](1) - cur_pos_(1), 
    //                             fe_path_[lk_ahd_idx](0) - cur_pos_(0));

    // yaw_yawrate = calculateYaw(fe_path_, plan_start_clock.nanoseconds()/1e9, ...);

    {
      std::lock_guard<std::mutex> mpc_pred_mtx_grd(mpc_pred_mtx_);

      // Visualize path
      mpc_pred_u_.clear();
      mpc_pred_pos_.clear();
      mpc_pred_vel_.clear();
      mpc_pred_acc_.clear();

      x_optimal = mpc_->X_0_;
      for (int i = 0; i < mpc_->MPC_HORIZON; i++) {
          mpc_->getOptimalControl(u_predict, i);
          mpc_->getSystemModel(A1, B1, mpc_->MPC_STEP);
          x_optimal = A1 * x_optimal + B1 * u_predict;
          mpc_pred_u_.push_back(u_predict);
          mpc_pred_pos_.push_back(x_optimal.segment<3>(0));
          mpc_pred_vel_.push_back(x_optimal.segment<3>(3));
          mpc_pred_acc_.push_back(x_optimal.segment<3>(6));
      }
    }
    
    pubMPCPath(mpc_pred_pos_);

  }
  else {
    logger_->logError("MPC Solve failure!");
    return false;
  }

  plan_complete_ = true;

  return true;

}

bool VoronoiPlanner::planCommlessMINCO(const Eigen::Vector3d& goal_pos){
  if (!init_voro_maps_){
    logger_->logInfo("Voronoi maps not initialized! Request a plan after initialization!");
    return false;
  }

  /* 1) Assign voronoi map and reservation table to planner */

  // {
  //   std::lock_guard<std::mutex> rsvn_tbl_guard(rsvn_tbl_mtx_);
  //   fe_planner_->setReservationTable(rsvn_tbl_);
  // }

  std::lock_guard<std::mutex> voro_map_guard(voro_map_mtx_);

  // Assign voronoi map
  voro_params_.z_sep_cm = bool_map_3d_.z_sep_cm;
  voro_params_.local_origin_x = bool_map_3d_.origin(0);
  voro_params_.local_origin_y = bool_map_3d_.origin(1);
  voro_params_.max_height_cm = bool_map_3d_.max_height_cm;
  voro_params_.min_height_cm = bool_map_3d_.min_height_cm;
  voro_params_.res = bool_map_3d_.resolution;

  fe_planner_->setVoroMap(dyn_voro_arr_, voro_params_);

  /* 2) Get current position, velocity and acceleration */

  Eigen::Vector3d start_pos, start_vel, start_acc;

  auto plan_start_clock = this->get_clock()->now();

  if (!sampleMINCOTrajectory(poly_traj_, plan_start_clock.nanoseconds()/1e9, start_pos, start_vel, start_acc))
  {
    start_pos = cur_pos_;
    start_vel = cur_vel_;
    start_acc.setZero();
  }

  /* 3) Get Receding Horizon Planning goal */

  // Get RHP goal
  // Eigen::Vector3d rhp_goal_pos = goal_pos;
  Eigen::Vector3d rhp_goal_pos = getRHPGoal(
    start_pos, goal_pos, 
    voxel_map_->getLocalMapOrigin(0.15), voxel_map_->getLocalMapMax(0.15));

  viz_helper_->pubPlanRequestViz(start_pos, rhp_goal_pos, goal_pos, plan_req_pub_, map_frame_);

  /* 4) Plan HCA* path */

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

  /* 4) Retrieve and transform HCA* path to map frame, publish visualization of path */

  // Current pose must converted from fixed map frame to local map frame 
  // before passing to planner
  std::vector<Eigen::Vector4d> fe_path_with_t_lclmapframe = fe_planner_->getPathWithTime();

  auto transformPathFromLclMapToMap = [&](const std::vector<Eigen::Vector4d>& map_path_w_t_lclmapframe, 
                             std::vector<Eigen::Vector4d>& map_path_w_t,
                             std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& map_path) {
    map_path.clear();
    map_path_w_t.clear();
    
    for (size_t i = 0; i < map_path_w_t_lclmapframe.size(); i++){
      Eigen::Vector4d map_pt;
      map_pt.head<3>() = lclMapToMap(map_path_w_t_lclmapframe[i].head<3>());
      map_pt(3) = map_path_w_t_lclmapframe[i](3);

      map_path.push_back(map_pt.head<3>());
      map_path_w_t.push_back(map_pt);
    }
  };

  // Convert front-end path from local map frame to fixed map frame
  transformPathFromLclMapToMap(fe_path_with_t_lclmapframe, fe_path_with_t_, fe_path_);
  
  // Visualize front-end path
  viz_helper_->pubFrontEndPath(fe_path_, fe_plan_viz_pub_, map_frame_);

  // Convert from space time path to gestelt_interfaces::msg::SpaceTimePath
  gestelt_interfaces::msg::SpaceTimePath fe_plan_msg;

  fe_plan_msg.agent_id = drone_id_;
  fe_plan_msg.header.stamp = plan_start_clock;
  fe_plan_msg.t_plan_start = plan_start_clock.nanoseconds()/1e9;

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

  /* 5) Generate commands for executing trajectory */

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

    poly_traj_ = genMinJerkTraj(min_jerk_opt_, mjo_fe_path, start_PVA, goal_PVA, plan_start_clock.nanoseconds()/1e9);

    Eigen::MatrixXd mjo_cstr_pts = min_jerk_opt_->getConstraintPts(5);

    minco_interfaces::msg::PolynomialTrajectory poly_msg; 
    minco_interfaces::msg::MincoTrajectory MINCO_msg; 

    polyTrajToMincoMsg(poly_traj_, plan_start_clock.nanoseconds()/1e9, poly_msg, MINCO_msg);

    poly_traj_pub_->publish(poly_msg); // [map frame] Publish to corresponding drone for execution
    // TODO: convert MINCO_msg to world frame
    // minco_traj_broadcast_pub_->publish(MINCO_msg); // [map frame] Broadcast to all other drones
    
    // Publish visualization
    viz_helper::VizHelper::pubExecTraj(mjo_cstr_pts, minco_traj_viz_pub_, map_frame_);
  }

  plan_complete_ = true;

  return true;

}

bool VoronoiPlanner::planCommMINCO(const Eigen::Vector3d& goal_pos){
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

  // if (!sampleMINCOTrajectory(poly_traj_, plan_start_clock.nanoseconds()/1e9;, start_pos, start_vel, start_acc))
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
  // fe_plan_msg.t_plan_start = plan_start_clock.nanoseconds()/1e9;;

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

  //   poly_traj_ = genMinJerkTraj(min_jerk_opt_, mjo_fe_path, start_PVA, goal_PVA, plan_start_clock.nanoseconds()/1e9);

  //   Eigen::MatrixXd mjo_cstr_pts = min_jerk_opt_->getConstraintPts(5);

  //   minco_interfaces::msg::PolynomialTrajectory poly_msg; 
  //   minco_interfaces::msg::MincoTrajectory MINCO_msg; 

  //   polyTrajToMincoMsg(poly_traj_, plan_start_clock.nanoseconds()/1e9, poly_msg, MINCO_msg);

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

    // // Publish trajectory representing the drone staying in
    // //  it's current position
    // gestelt_interfaces::msg::SpaceTimePath fe_plan_msg;

    // fe_plan_msg.agent_id = drone_id_;
    // fe_plan_msg.header.stamp = plan_start_clock;
    // fe_plan_msg.t_plan_start = plan_start_clock.nanoseconds()/1e9;

    // for (size_t i = 0; i < (size_t) rsvn_tbl_window_size_; i++){
    //   geometry_msgs::msg::Pose pose;
    //   pose.position.x = cur_pos_(0); 
    //   pose.position.y = cur_pos_(1);
    //   pose.position.z = cur_pos_(2);
    //   pose.orientation.w = 1.0; 

    //   fe_plan_msg.plan.push_back(pose);
    //   fe_plan_msg.plan_time.push_back(i);
    // }

    // fe_plan_broadcast_pub_->publish(fe_plan_msg);

    return;
  }

  if (isGoalReached(cur_pos_, waypoints_.nextWP())){
    logger_->logInfo("Reached goal!");
    
    // If goals is within a given tolerance, then pop this goal and plan next goal (if available)
    waypoints_.popWP();

    return;
  }

  // Plan from current position to next waypoint
  plan(waypoints_.nextWP());
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

  tm_voro_map_init_.stop(false);

  // viz_helper_->pubVoroVertices(voro_verts, voronoi_graph_pub_, local_map_frame_);
  init_voro_maps_ = true; // Flag to indicate that all voronoi maps have been initialized
}

void VoronoiPlanner::sendMPCCmdTimerCB()
{

  if (mpc_pred_pos_.empty() 
      || mpc_pred_vel_.empty() 
      || mpc_pred_acc_.empty())
  {
    return;
  }

  double t_start = this->get_clock()->now().nanoseconds()/1e9;
  // // Get time t relative to start of MPC trajectory
  double e_t_start = t_start - last_mpc_solve_; 
  double total_traj_duration = (int)mpc_pred_acc_.size() * mpc_->MPC_STEP;
  
  if (e_t_start < 0.0 || e_t_start >= total_traj_duration)
  {
    // Exceeded duration of trajectory or trajectory timestamp invalid
    return;
  }

  // Get index of point closest to current time
  int idx =  std::floor(e_t_start / mpc_->MPC_STEP); 

  std::lock_guard<std::mutex> mpc_pred_mtx_grd(mpc_pred_mtx_);

  if (idx >= (int) mpc_pred_acc_.size()){
    logger_->logError("DEV ERROR!! sampleMPCTrajectory idx exceeded!");
  }

  // Publish PVA command
  pubPVAJCmd(mpc_pred_pos_[idx], 
            Eigen::Vector2d(0.0, NAN), 
            mpc_pred_vel_[idx], 
            mpc_pred_acc_[idx], 
            mpc_pred_u_[idx]);
}

/* Subscriber callbacks*/

void VoronoiPlanner::swarmOdomCB(const nav_msgs::msg::Odometry::UniquePtr& msg, int id)
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
			this->get_logger(), "Could not get transform from agent(%s) to map_frame_(%s): %s",
			msg->header.frame_id.c_str(), map_frame_.c_str(), ex.what());
    return;
  }

  voxel_map_->updateSwarmState(id, pose, vel);

  if (id == drone_id_){
    return;
  }

  if (!init_voro_maps_){
    return;
  }

  // /**
  //  * Clear reservation table and add to it
  //  */
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
  //     std::lock_guard<std::mutex> voro_map_guard(voro_map_mtx_);

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

