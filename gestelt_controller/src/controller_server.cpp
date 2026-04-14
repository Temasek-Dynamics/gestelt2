// Copyright (c) 2019 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <vector>
#include <memory>
#include <string>
#include <utility>
#include <limits>

#include "lifecycle_msgs/msg/state.hpp"
#include "gestelt_core/controller_exceptions.hpp"
#include "nav_2d_utils/conversions.hpp"
#include "nav_2d_utils/tf_help.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"

#include "gestelt_controller/controller_server.hpp"

using namespace std::chrono_literals;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace gestelt_controller
{

ControllerServer::ControllerServer(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("controller_server", "", options),
  progress_checker_loader_("gestelt_core", "gestelt_core::ProgressChecker"),
  default_progress_checker_id_{"progress_checker"},
  default_progress_checker_type_{"gestelt_controller::SimpleProgressChecker"},
  goal_checker_loader_("gestelt_core", "gestelt_core::GoalChecker"),
  default_goal_checker_ids_{"goal_checker"},
  default_goal_checker_types_{"gestelt_controller::SimpleGoalChecker"},
  lp_loader_("gestelt_core", "gestelt_core::Controller"),
  default_ids_{"FollowPath"},
  default_types_{"linear_mpc_controller::LinearMPCController"},
  occ_map_update_timeout_(300ms)
{
  RCLCPP_INFO(get_logger(), "Creating controller server");

  declare_parameter("controller_frequency", 20.0);
  declare_parameter("inflation_escape_offset", 0.3);

  declare_parameter("action_server_result_timeout", 10.0);

  declare_parameter("progress_checker_plugin", default_progress_checker_id_);
  declare_parameter("goal_checker_plugins", default_goal_checker_ids_);
  declare_parameter("controller_plugins", default_ids_);

  declare_parameter("speed_limit_topic", rclcpp::ParameterValue("speed_limit"));

  declare_parameter("failure_tolerance", rclcpp::ParameterValue(0.0));
  declare_parameter("publish_zero_velocity", rclcpp::ParameterValue(true));
  declare_parameter("occ_map_update_timeout", 0.30);  // 300ms

  declare_parameter("print_runtime", rclcpp::ParameterValue(false));

  // Setup the local occupancy map
  occ_map_ = std::make_shared<occ_map::OccMap>(
    "local_occ_map", std::string{get_namespace()}, 
    get_parameter("use_sim_time").as_bool());
}

ControllerServer::~ControllerServer()
{
  progress_checker_.reset();
  goal_checkers_.clear();
  controllers_.clear();
  occ_map_thread_.reset();
}

nav2_util::CallbackReturn
ControllerServer::on_configure(const rclcpp_lifecycle::State & state)
{
  auto node = shared_from_this();

  RCLCPP_INFO(get_logger(), "Configuring controller interface");

  get_parameter("print_runtime", print_runtime_);
  // get_parameter("publish_zero_velocity", publish_zero_velocity_); (WIP)

  get_parameter("progress_checker_plugin", progress_checker_id_);
  if (progress_checker_id_ == default_progress_checker_id_) {
    nav2_util::declare_parameter_if_not_declared(
      node, default_progress_checker_id_ + ".plugin",
      rclcpp::ParameterValue(default_progress_checker_type_));
  }

  RCLCPP_INFO(get_logger(), "getting goal checker plugins..");
  get_parameter("goal_checker_plugins", goal_checker_ids_);
  if (goal_checker_ids_ == default_goal_checker_ids_) {
    for (size_t i = 0; i < default_goal_checker_ids_.size(); ++i) {
      nav2_util::declare_parameter_if_not_declared(
        node, default_goal_checker_ids_[i] + ".plugin",
        rclcpp::ParameterValue(default_goal_checker_types_[i]));
    }
  }

  get_parameter("controller_plugins", controller_ids_);
  if (controller_ids_ == default_ids_) {
    for (size_t i = 0; i < default_ids_.size(); ++i) {
      nav2_util::declare_parameter_if_not_declared(
        node, default_ids_[i] + ".plugin",
        rclcpp::ParameterValue(default_types_[i]));
    }
  }

  controller_types_.resize(controller_ids_.size());
  goal_checker_types_.resize(goal_checker_ids_.size());

  get_parameter("controller_frequency", controller_frequency_);
  RCLCPP_INFO(get_logger(), "Controller frequency set to %.4fHz", controller_frequency_);
  get_parameter("inflation_escape_offset", inflation_escape_offset_);
  RCLCPP_INFO(get_logger(), "Inflation escape offset set to %.2fm", inflation_escape_offset_);

  occ_map_->configure();
  // Launch a thread to run the occupancy map node
  occ_map_thread_ = std::make_unique<nav2_util::NodeThread>(occ_map_);

  try {
    progress_checker_type_ = nav2_util::get_plugin_type_param(node, progress_checker_id_);
    progress_checker_ = progress_checker_loader_.createUniqueInstance(progress_checker_type_);
    RCLCPP_INFO(
      get_logger(), "Created progress_checker : %s of type %s",
      progress_checker_id_.c_str(), progress_checker_type_.c_str());
    progress_checker_->initialize(node, progress_checker_id_);
  } catch (const pluginlib::PluginlibException & ex) {
    RCLCPP_FATAL(
      get_logger(),
      "Failed to create progress_checker. Exception: %s", ex.what());
    return nav2_util::CallbackReturn::FAILURE;
  }

  for (size_t i = 0; i != goal_checker_ids_.size(); i++) {
    try {
      goal_checker_types_[i] = nav2_util::get_plugin_type_param(node, goal_checker_ids_[i]);
      gestelt_core::GoalChecker::Ptr goal_checker =
        goal_checker_loader_.createUniqueInstance(goal_checker_types_[i]);
      RCLCPP_INFO(
        get_logger(), "Created goal checker : %s of type %s",
        goal_checker_ids_[i].c_str(), goal_checker_types_[i].c_str());
      goal_checker->initialize(node, goal_checker_ids_[i]);
      goal_checkers_.insert({goal_checker_ids_[i], goal_checker});
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(
        get_logger(),
        "Failed to create goal checker. Exception: %s", ex.what());
      on_cleanup(state);
      return nav2_util::CallbackReturn::FAILURE;
    }
  }

  for (size_t i = 0; i != goal_checker_ids_.size(); i++) {
    goal_checker_ids_concat_ += goal_checker_ids_[i] + std::string(" ");
  }
  
  RCLCPP_INFO(
    get_logger(),
    "Controller Server has %s goal checkers available.", goal_checker_ids_concat_.c_str());

  for (size_t i = 0; i != controller_ids_.size(); i++) {
    try {
      controller_types_[i] = nav2_util::get_plugin_type_param(node, controller_ids_[i]);
      gestelt_core::Controller::Ptr controller =
        lp_loader_.createUniqueInstance(controller_types_[i]);
      RCLCPP_INFO(
        get_logger(), "Created controller : %s of type %s",
        controller_ids_[i].c_str(), controller_types_[i].c_str());
      controller->configure(
        node, controller_ids_[i],
        occ_map_->getTfBuffer(), occ_map_);
      controllers_.insert({controller_ids_[i], controller});
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(
        get_logger(),
        "Failed to create controller. Exception: %s", ex.what());
      on_cleanup(state);
      return nav2_util::CallbackReturn::FAILURE;
    }
  }
  
  for (size_t i = 0; i != controller_ids_.size(); i++) {
    controller_ids_concat_ += controller_ids_[i] + std::string(" ");
  }

  RCLCPP_INFO(
    get_logger(),
    "Controller Server has %s controllers available.", controller_ids_concat_.c_str());

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", rclcpp::SensorDataQoS(), 
    std::bind(&ControllerServer::odometrySubCB, this, _1));

  received_global_plan_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "received_global_plan", rclcpp::SensorDataQoS(), 
    std::bind(&ControllerServer::globalPlanSubCB, this, _1));

  cmd_pub_ = node->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
    "intmd_cmd", rclcpp::SensorDataQoS());

  // Just for visualization only
  current_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
    "path_in_controller", rclcpp::SensorDataQoS());

  double action_server_result_timeout;
  get_parameter("action_server_result_timeout", action_server_result_timeout);
  rcl_action_server_options_t server_options = rcl_action_server_get_default_options();
  server_options.result_timeout.nanoseconds = RCL_S_TO_NS(action_server_result_timeout);
  
  // Create the action server that we implement with our followPath method
  try {
    action_server_ = std::make_unique<ActionServer>(
      shared_from_this(),
      "follow_path",
      std::bind(&ControllerServer::computeControl, this),
      nullptr,
      std::chrono::milliseconds(500),
      false);

  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR(get_logger(), "Error creating action server! %s", e.what());
    on_cleanup(state);
    return nav2_util::CallbackReturn::FAILURE;
  }

  // Create timer for publishing commands and commands to goal
	pub_cmd_timer_ = this->create_wall_timer((1.0/controller_frequency_) *1000ms, 
                                            std::bind(&ControllerServer::publishCmdTimerCB, this));
  goal_cmd_timer_ = this->create_wall_timer((1.0/controller_frequency_) *1000ms, 
                                             std::bind(&ControllerServer::computeGoalCmdTimerCB, this));
  inflation_cmd_timer_ = this->create_wall_timer((1.0/controller_frequency_) *1000ms, 
                                             std::bind(&ControllerServer::inflationAvoidanceTimerCB, this));
  pub_cmd_timer_->cancel(); // Stop timer immediately
  goal_cmd_timer_->cancel(); // Stop timer immediately
  inflation_cmd_timer_->cancel();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
ControllerServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  const auto occ_map_state = occ_map_->activate();
  if (occ_map_state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    return nav2_util::CallbackReturn::FAILURE;
  }
  ControllerMap::iterator it;
  for (it = controllers_.begin(); it != controllers_.end(); ++it) {
    it->second->activate();
  }
  cmd_pub_->on_activate();
  current_path_pub_->on_activate();
  action_server_->activate();

  pub_cmd_timer_->reset(); // Starts timer
  goal_cmd_timer_->reset();
  inflation_cmd_timer_->reset();

  auto node = shared_from_this();
  // Add callback for dynamic parameters
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&ControllerServer::dynamicParametersCallback, this, _1));

  // create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
ControllerServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  action_server_->deactivate();
  ControllerMap::iterator it;
  for (it = controllers_.begin(); it != controllers_.end(); ++it) {
    it->second->deactivate();
  }

  occ_map_->deactivate();

  cmd_pub_->on_deactivate();
  current_path_pub_->on_deactivate();

  remove_on_set_parameters_callback(dyn_params_handler_.get());
  dyn_params_handler_.reset();

  // destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
ControllerServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  // Cleanup the helper classes
  ControllerMap::iterator it;
  for (it = controllers_.begin(); it != controllers_.end(); ++it) {
    it->second->cleanup();
  }
  controllers_.clear();

  goal_checkers_.clear();
  progress_checker_->reset();

  occ_map_->cleanup();

  // Release any allocated resources
  action_server_.reset();
  odom_sub_.reset();
  received_global_plan_sub_.reset();
  occ_map_.reset();
  occ_map_thread_.reset();
  cmd_pub_.reset();
  current_path_pub_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
ControllerServer::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

bool ControllerServer::findControllerId(
  const std::string & c_name,
  std::string & current_controller)
{
  if (controllers_.find(c_name) == controllers_.end()) {
    if (controllers_.size() == 1 && c_name.empty()) {
      RCLCPP_WARN_ONCE(
        get_logger(), "No controller was specified in action call."
        " Server will use only plugin loaded %s. "
        "This warning will appear once.", controller_ids_concat_.c_str());
      current_controller = controllers_.begin()->first;
    } else {
      RCLCPP_ERROR(
        get_logger(), "FollowPath called with controller name %s, "
        "which does not exist. Available controllers are: %s.",
        c_name.c_str(), controller_ids_concat_.c_str());
      return false;
    }
  } else {
    RCLCPP_DEBUG(get_logger(), "Selected controller: %s.", c_name.c_str());
    current_controller = c_name;
  }

  return true;
}

bool ControllerServer::findGoalCheckerId(
  const std::string & c_name,
  std::string & current_goal_checker)
{
  if (goal_checkers_.find(c_name) == goal_checkers_.end()) {
    if (goal_checkers_.size() == 1 && c_name.empty()) {
      RCLCPP_WARN_ONCE(
        get_logger(), "No goal checker was specified in parameter 'current_goal_checker'."
        " Server will use only plugin loaded %s. "
        "This warning will appear once.", goal_checker_ids_concat_.c_str());
      current_goal_checker = goal_checkers_.begin()->first;
    } else {
      RCLCPP_ERROR(
        get_logger(), "FollowPath called with goal_checker name %s in parameter"
        " 'current_goal_checker', which does not exist. Available goal checkers are: %s.",
        c_name.c_str(), goal_checker_ids_concat_.c_str());
      return false;
    }
  } else {
    RCLCPP_DEBUG(get_logger(), "Selected goal checker: %s.", c_name.c_str());
    current_goal_checker = c_name;
  }

  return true;
}

void ControllerServer::computeControl()
{
  RCLCPP_INFO(get_logger(), "[computeControl] Acquiring lock(dynamic_params_lock_)");
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);
  RCLCPP_INFO(get_logger(), "[computeControl] Acquired lock(dynamic_params_lock_)");

  RCLCPP_INFO(get_logger(), "Received a goal, begin computing control effort.");

  yaw_first = true; 
  RCLCPP_INFO(get_logger(), "Set yaw_first flag");
  start_computing_goal_ = true;
  RCLCPP_INFO(get_logger(), "Set computeGoalCmdTimerCB() flag");
  inflation_avoidance_ = true;
  RCLCPP_INFO(get_logger(), "Set inflationAvoidanceTimerCB() flag");


  try {
    auto goal = action_server_->get_current_goal();
    if (!goal) {
      RCLCPP_INFO(get_logger(), "No goal received. Returning...");
      return;  //  goal would be nullptr if action_server_ is inactivate.
    }

    std::string c_name = goal->controller_id;
    std::string current_controller;
    if (findControllerId(c_name, current_controller)) {
      current_controller_ = current_controller;
    } else {
      throw gestelt_core::InvalidController("Failed to find controller name: " + c_name);
    }

    std::string gc_name = goal->goal_checker_id;
    std::string current_goal_checker;
    if (findGoalCheckerId(gc_name, current_goal_checker)) {
      current_goal_checker_ = current_goal_checker;
    } else {
      throw gestelt_core::ControllerException("Failed to find goal checker name: " + gc_name);
    }

    setPlannerPath(goal->path);
    progress_checker_->reset();

    last_valid_cmd_time_ = now();
    rclcpp::WallRate loop_rate(controller_frequency_);
    while (rclcpp::ok()) {
      auto start_time = this->now();

      if (action_server_ == nullptr || !action_server_->is_server_active()) {
        RCLCPP_DEBUG(get_logger(), "Action server unavailable or inactive. Stopping.");
        return;
      }

      if (action_server_->is_cancel_requested()) {
        if (controllers_[current_controller_]->cancel()) {
          RCLCPP_INFO(get_logger(), "Cancellation was successful. Stopping the robot.");
          action_server_->terminate_all();
          onGoalExit();
          return;
        } else {
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000, "Waiting for the controller to finish cancellation");
        }
      }

      updateGlobalPath();

      if (isGoalReached()) {
        RCLCPP_INFO(get_logger(), "Reached the goal!");
        break;
      }

      if (odomWithinObstacleInflation()){
        RCLCPP_WARN(get_logger(), "Odom is in obstacle");
        continue;
      }

      getControllerCommand();

      auto cycle_duration = this->now() - start_time;
      if (!loop_rate.sleep()) {
        RCLCPP_WARN(
          get_logger(),
          "Control loop missed its desired rate of %.4f Hz. Current loop rate is %.4f Hz.",
          controller_frequency_, 1 / cycle_duration.seconds());
      }
    }
    RCLCPP_INFO(get_logger(),"End of controller's action server (Success)");
  } catch (gestelt_core::InvalidController & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    start_publish_cmd_ = false;

    onGoalExit();
    std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
    // result->error_code = Action::Result::INVALID_CONTROLLER;
    action_server_->terminate_current(result);
    return;
  } catch (gestelt_core::ControllerTFError & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    start_publish_cmd_ = false;

    onGoalExit();
    std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
    // result->error_code = Action::Result::TF_ERROR;
    action_server_->terminate_current(result);
    return;
  } catch (gestelt_core::NoValidControl & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    start_publish_cmd_ = false;

    // onGoalExit();
    // std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
    // // result->error_code = Action::Result::NO_VALID_CONTROL;
    // action_server_->terminate_current(result);

    // return;
  } catch (gestelt_core::FailedToMakeProgress & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    start_publish_cmd_ = false;
    onGoalExit();
    std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
    // result->error_code = Action::Result::FAILED_TO_MAKE_PROGRESS;
    action_server_->terminate_current(result);
    return;
  } catch (gestelt_core::PatienceExceeded & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    start_publish_cmd_ = false;
    onGoalExit();
    std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
    // result->error_code = Action::Result::PATIENCE_EXCEEDED;
    action_server_->terminate_current(result);
    return;
  } catch (gestelt_core::InvalidPath & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    start_publish_cmd_ = false;
    onGoalExit();
    std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
    // result->error_code = Action::Result::INVALID_PATH;
    action_server_->terminate_current(result);
    return;
  } catch (gestelt_core::ControllerTimedOut & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    start_publish_cmd_ = false;
    onGoalExit();
    std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
    // result->error_code = Action::Result::CONTROLLER_TIMED_OUT;
    action_server_->terminate_current(result);
    return;
  } catch (gestelt_core::ControllerException & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    start_publish_cmd_ = false;
    onGoalExit();
    std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
    // result->error_code = Action::Result::UNKNOWN;
    action_server_->terminate_current(result);
    return;
  } catch (std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    start_publish_cmd_ = false;
    onGoalExit();
    std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
    // result->error_code = Action::Result::UNKNOWN;
    action_server_->terminate_current(result);
    return;
  }

  RCLCPP_INFO(get_logger(), "Controller succeeded, setting result");

  // start_publish_cmd_ = false;
  onGoalExit();

  // TODO(orduno) #861 Handle a pending preemption and set controller name
  action_server_->succeeded_current();
}

void ControllerServer::setPlannerPath(const nav_msgs::msg::Path & path)
{
  RCLCPP_INFO(get_logger(), "[setPlannerPath] Setting planner path");
  RCLCPP_DEBUG(
    get_logger(),
    "Providing path to the controller %s", current_controller_.c_str());
  if (path.poses.empty()) {
    throw gestelt_core::InvalidPath("Path is empty.");
  }
  controllers_[current_controller_]->setPlan(path);

  end_pose_ = path.poses.back();
  end_pose_.header.frame_id = path.header.frame_id;
  goal_checkers_[current_goal_checker_]->reset();

  RCLCPP_DEBUG(
    get_logger(), "Path end point is (%.2f, %.2f)",
    end_pose_.pose.position.x, end_pose_.pose.position.y);

  current_path_ = path;
  RCLCPP_INFO(get_logger(), "[setPlannerPath] Set planner path");
}

void ControllerServer::publishCmdTimerCB()
{
  if (!start_publish_cmd_){
    return;
  }
  RCLCPP_INFO(get_logger(), "[publishCmdTimerCB] Start publishing command");

  RCLCPP_INFO(get_logger(), "[publishCmdTimerCB] Acquiring mpc_pred_lk(mpc_pred_mtx_) lock");
  std::lock_guard<std::mutex> mpc_pred_lk(mpc_pred_mtx_);
  RCLCPP_INFO(get_logger(), "[publishCmdTimerCB] Acquired mpc_pred_lk(mpc_pred_mtx_) lock");

  if (cmd_pos_prev_.empty() 
      || cmd_vel_prev_.empty() 
      || cmd_acc_prev_.empty())
  {
    RCLCPP_ERROR(this->get_logger(), "  cmd_pos_prev_ is empty!");
    return;
  }

  // Get time t relative to start of MPC trajectory
  double e_t_start = now().nanoseconds()/1e9 - last_valid_cmd_time_.nanoseconds()/1e9; 
  double total_traj_duration = (int)cmd_pos_prev_.size() * controllers_[current_controller_]->getControllerTimestep();

  if (e_t_start < 0.0 || e_t_start >= total_traj_duration)
  {
    // Exceeded duration of trajectory or trajectory timestamp invalid
    RCLCPP_ERROR(this->get_logger(), " e_t_start(%f) exceeded total_traj_duration(%f)!", e_t_start, total_traj_duration);
    return;
  }

  // Get index of point closest to current time
  //  elapsed time from start divided by time step of MPC
  int idx =  std::floor(e_t_start / controllers_[current_controller_]->getControllerTimestep()); 

  if (idx >= (int) cmd_pos_prev_.size()){
    RCLCPP_ERROR(this->get_logger(), "FATAL DEVELOPMENT ERROR: idx of sampled command trajectory exceeded size!");
    return;
  }

  // Failsafe when stuck in obstacle
  if (cmd_pos_prev_.size() == 1){
    idx = 0;
  }

  // For checking idx position
  // RCLCPP_INFO(get_logger(), "Idx is at: %d", idx);

  // Trajectory setpoint is in ENU frame 
  // (Transforming to NED frame is done by trajectory server)
  px4_msgs::msg::TrajectorySetpoint traj_sp;

  // Use first index of optimal controls
  traj_sp.position = 
    {(float) cmd_pos_prev_[idx].x() , (float) cmd_pos_prev_[idx].y(), (float) cmd_pos_prev_[idx].z()};
  traj_sp.velocity = 
    {(float) cmd_vel_prev_[idx].x() , (float) cmd_vel_prev_[idx].y(), (float) cmd_vel_prev_[idx].z()};
  traj_sp.acceleration = 
    {(float) cmd_acc_prev_[idx].x() , (float) cmd_acc_prev_[idx].y(), (float) cmd_acc_prev_[idx].z()};
  traj_sp.jerk = 
    {(float) cmd_jerk_prev_[idx].x() , (float) cmd_jerk_prev_[idx].y(), (float) cmd_jerk_prev_[idx].z()};
    
  traj_sp.yaw = cmd_yaw_prev_(0);
  traj_sp.yawspeed = cmd_yaw_prev_(1);
  traj_sp.timestamp = now().nanoseconds() / 1000; // In microseconds
  cmd_pub_->publish(traj_sp);
  RCLCPP_DEBUG(get_logger(), "Publishing command at time %.2f", now().seconds());
}

void ControllerServer::computeGoalCmdTimerCB(){
  if (!start_computing_goal_){
    return;
  }
  if (isGoalReached()) {
    RCLCPP_INFO(get_logger(), "Begin computing command to goal");
    // Logic to get out of goal (Untested, WIP) //
    geometry_msgs::msg::PoseStamped pose;
    if (!getRobotPose(pose)) {
      throw gestelt_core::ControllerTFError("Failed to obtain robot pose");
    }
    bool break_flag = false;
    for (int dx = -1; dx <= 1; dx++)
    {
      for (int dy = -1; dy <= 1; dy++)
      {
        // for (int dz = -1; dz <= 1; dz++){
        const Eigen::Vector3d cur_pos_temp = Eigen::Vector3d(
          pose.pose.position.x + ((double)dx * 0.3), 
          pose.pose.position.y + ((double)dy * 0.3),
          pose.pose.position.z
        );
        const Eigen::Vector3d cur_pos_temp_opp = Eigen::Vector3d(
          pose.pose.position.x - ((double)dx * 0.3), 
          pose.pose.position.y - ((double)dy * 0.3),
          pose.pose.position.z
        );

        if (occ_map_->withinObstacleInflation(cur_pos_temp) && !(occ_map_->withinObstacleInflation(cur_pos_temp_opp))){
          {
            RCLCPP_INFO(get_logger(), "[computeGoalCmdTimerCB] A point 0.3m around odom pose is in inflation"
                                          "(dy,dx): (%d, %d), Point(%.2f, %.2f, %.2f), Opp point(%.2f, %.2f, %.2f)",
                                          dy, dx,
                                          cur_pos_temp.x(), cur_pos_temp.y(), cur_pos_temp.z(),
                                          cur_pos_temp_opp.x(), cur_pos_temp_opp.y(), cur_pos_temp_opp.z());
            RCLCPP_INFO(get_logger(), "[computeGoalCmdTimerCB] Acquiring mpc_pred_lk(mpc_pred_mtx_) lock");
            std::lock_guard<std::mutex> mpc_pred_lk(mpc_pred_mtx_);
            RCLCPP_INFO(get_logger(), "[computeGoalCmdTimerCB] Acquired mpc_pred_lk(mpc_pred_mtx_) lock");
            const auto deltaVector = cur_pos_ - cur_pos_temp; //vector from cur_pos_temp to cur_pos
            Eigen::Vector3d offset_cmd_pos = cur_pos_ + (deltaVector); // direct offset away
          
            RCLCPP_INFO(this->get_logger(), "[computeGoalCmdTimerCB] Setting waypoint to offset from pose to offset_cmd_pos (%f, %f, %f), (%f, %f, %f)", 
              pose.pose.position.x,
              pose.pose.position.y,
              pose.pose.position.z,
              offset_cmd_pos.x(),
              offset_cmd_pos.y(),
              offset_cmd_pos.z()
            );
            cmd_pos_prev_ = {offset_cmd_pos};
            cmd_vel_prev_ = {deltaVector};
            cmd_acc_prev_ = { Eigen::Vector3d(std::nanf(""),std::nanf(""),std::nanf("")) };
            cmd_jerk_prev_ = {Eigen::Vector3d(std::nanf(""),std::nanf(""),std::nanf(""))};
            last_valid_cmd_time_ = now();
            start_publish_cmd_ = true;
            break_flag = true;
            RCLCPP_INFO(get_logger(), "[computeGoalCmdTimerCB] Unlock mpc_pred_lk(mpc_pred_mtx_) lock");
          }
          break;
        }
        // }
        // if (break_flag){
        //   break;
        // }
      }
      if (break_flag){
        break;
      }
    }
    if (break_flag){
      return;
    }
    {
      RCLCPP_INFO(get_logger(), "[computeGoalCmdTimerCB] Area around odom is either free of obstacle, or within. Check if planner throws any error");
      RCLCPP_INFO(get_logger(), "[computeGoalCmdTimerCB] Acquiring mpc_pred_lk(mpc_pred_mtx_) lock");
      std::lock_guard<std::mutex> mpc_pred_lk(mpc_pred_mtx_);
      RCLCPP_INFO(get_logger(), "[computeGoalCmdTimerCB] Acquired mpc_pred_lk(mpc_pred_mtx_) lock");
      // Stuff to test (Transform goal wrt global to map) //
      geometry_msgs::msg::PoseStamped transformed_goal_pose;
      rclcpp::Duration tolerance(rclcpp::Duration::from_seconds(occ_map_->getTransformTolerance()));
      nav_2d_utils::transformPose(
        occ_map_->getTfBuffer(), occ_map_->getMapFrameID(),
        end_pose_, transformed_goal_pose, tolerance);
      Eigen::Vector3d goal_cmd_pos = Eigen::Vector3d(
                                      transformed_goal_pose.pose.position.x,
                                      transformed_goal_pose.pose.position.y,
                                      transformed_goal_pose.pose.position.z
                                      );
      
      RCLCPP_INFO(this->get_logger(), "[computeGoalCmdTimerCB] Setting waypoint to goal_cmd_pos(Global/Map) (%f, %f, %f), (%f, %f, %f)", 
        end_pose_.pose.position.x,
        end_pose_.pose.position.y,
        end_pose_.pose.position.z,
        goal_cmd_pos.x(),
        goal_cmd_pos.y(),
        goal_cmd_pos.z()
      );
      cmd_pos_prev_ = {goal_cmd_pos};
      cmd_vel_prev_ = { goal_cmd_pos - cur_pos_ }; // Vector from change in dist to use as vector
      cmd_acc_prev_ = { Eigen::Vector3d(std::nanf(""),std::nanf(""),std::nanf("")) };
      cmd_jerk_prev_ = {Eigen::Vector3d(std::nanf(""),std::nanf(""),std::nanf(""))};
      last_valid_cmd_time_ = now();
      start_publish_cmd_ = true;
      RCLCPP_INFO(get_logger(), "[computeGoalCmdTimerCB] Unlock mpc_pred_lk(mpc_pred_mtx_) lock");
    }
  }
  return;  
}

void ControllerServer::inflationAvoidanceTimerCB()
{
  if (!(inflation_avoidance_ && odomWithinObstacleInflation() && !isGoalReached())){
    // RCLCPP_INFO(get_logger(), "[inflationAvoidanceTimerCB]: No computing of inflation avoidance");
    return;
  }

  RCLCPP_INFO(get_logger(), "[inflationAvoidanceTimerCB]: Acquiring mpc_pred_lk(mpc_pred_mtx_) lock");
  std::lock_guard<std::mutex> mpc_pred_lk(mpc_pred_mtx_);
  RCLCPP_INFO(get_logger(), "[inflationAvoidanceTimerCB]: Acquired mpc_pred_lk(mpc_pred_mtx_) lock");
  geometry_msgs::msg::PoseStamped pose;
  if (!getRobotPose(pose)) {
    throw gestelt_core::ControllerTFError("Failed to obtain robot pose");
  }
  bool break_flag = false;
  bool euclidean_search = false;
  for (int dz = -2; dz <= 2; dz++){
    if (!euclidean_search){
      dz = 0;
    }

    for (int dx = -1; dx <= 1; dx++)
    {
      for (int dy = -1; dy <= 1; dy++)
      {
        // for (int dz = -1; dz <= 1; dz++){
        const Eigen::Vector3d cur_pos_temp = Eigen::Vector3d(
          pose.pose.position.x + ((double)dx * inflation_escape_offset_), 
          pose.pose.position.y + ((double)dy * inflation_escape_offset_),
          // pose.pose.position.z
          pose.pose.position.z + ((double)dz * 0.2)
        );
        const Eigen::Vector3d cur_pos_temp_opp = Eigen::Vector3d(
          pose.pose.position.x - ((double)dx * inflation_escape_offset_), 
          pose.pose.position.y - ((double)dy * inflation_escape_offset_),
          // pose.pose.position.z
          pose.pose.position.z + ((double)dz * 0.2)
        );
        // if (!occ_map_->inGlobalMap(cur_pos_)){ // Have a catch statement
        //   RCLCPP_ERROR(get_logger(), "[inflationAvoidanceTimerCB]: Odom Coordinates of (%.2f, %.2f, %.2f) was outside bounds",
        //                 cur_pos_.x(), cur_pos_.y(), cur_pos_.z());
        //   break_flag = true;
        //   break;
        // }
        if (occ_map_->withinObstacleInflation(cur_pos_temp) && !(occ_map_->withinObstacleInflation(cur_pos_temp_opp))){
          RCLCPP_INFO(get_logger(), "[inflationAvoidanceTimerCB]: Finding a free point %.2fm around odom pose out of inflation"
                                        "(dy,dx,dz): (%d, %d, %d), Point(%.2f, %.2f, %.2f), Opp point(%.2f, %.2f, %.2f)",
                                        inflation_escape_offset_,
                                        dy, dx, dz,
                                        cur_pos_temp.x(), cur_pos_temp.y(), cur_pos_temp.z(),
                                        cur_pos_temp_opp.x(), cur_pos_temp_opp.y(), cur_pos_temp_opp.z());
          const auto deltaVector = cur_pos_ - cur_pos_temp; //vector from cur_pos_temp to cur_pos
          // Eigen::Vector3d offset_cmd_pos = cur_pos_ + (deltaVector); // direct offset away
          Eigen::Vector3d offset_cmd_pos = Eigen::Vector3d((cur_pos_ + (deltaVector)).x(),
                                                           (cur_pos_ + (deltaVector)).y(),
                                                            cur_pos_temp.z()); // direct offset 

          if (!occ_map_->inGlobalMap(offset_cmd_pos)){
            RCLCPP_ERROR(get_logger(), "[inflationAvoidanceTimerCB]: offset_cmd_pos Coordinates of (%.2f, %.2f, %.2f) was outside bounds",
                        offset_cmd_pos.x(), offset_cmd_pos.y(), offset_cmd_pos.z());
            continue;
          }
        
          RCLCPP_INFO(this->get_logger(), "[inflationAvoidanceTimerCB]: Setting waypoint to offset from pose to offset_cmd_pos (%f, %f, %f), (%f, %f, %f)", 
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
            offset_cmd_pos.x(),
            offset_cmd_pos.y(),
            offset_cmd_pos.z()
          );
          cmd_pos_prev_ = {offset_cmd_pos};
          cmd_vel_prev_ = {deltaVector};
          cmd_acc_prev_ = { Eigen::Vector3d(std::nanf(""),std::nanf(""),std::nanf("")) };
          cmd_jerk_prev_ = {Eigen::Vector3d(std::nanf(""),std::nanf(""),std::nanf(""))};
          last_valid_cmd_time_ = now();
          start_publish_cmd_ = true;
          break_flag = true;
          break;
        }
        // }
        // if (break_flag){
        //   break;
        // }
      }
      if (break_flag){
        break;
      }
    }
    euclidean_search = true;

    if (break_flag){
      break;
    }
  }
  if (break_flag){
    RCLCPP_INFO(get_logger(), "[inflationAvoidanceTimerCB]: Unlock mpc_pred_lk(mpc_pred_mtx_) lock");
    return;
  }

  RCLCPP_INFO(get_logger(), "[inflationAvoidanceTimerCB]: Area around odom is in inflation. Stay at its own odom");
  cmd_pos_prev_ = {cur_pos_};
  cmd_vel_prev_ = {Eigen::Vector3d(std::nanf(""),std::nanf(""),std::nanf(""))};
  cmd_acc_prev_ = { Eigen::Vector3d(std::nanf(""),std::nanf(""),std::nanf("")) };
  cmd_jerk_prev_ = {Eigen::Vector3d(std::nanf(""),std::nanf(""),std::nanf(""))};
  last_valid_cmd_time_ = now();
  start_publish_cmd_ = true;
  RCLCPP_INFO(get_logger(), "[inflationAvoidanceTimerCB]: Unlock mpc_pred_lk(mpc_pred_mtx_) lock");
}

void ControllerServer::getControllerCommand()
{
  geometry_msgs::msg::PoseStamped pose;

  if (!getRobotPose(pose)) {
    throw gestelt_core::ControllerTFError("Failed to obtain robot pose");
  }

  if (!progress_checker_->check(pose)) {
    throw gestelt_core::ControllerException("Failed to make progress");
  }

  Eigen::Vector3d cur_pos = Eigen::Vector3d(
    pose.pose.position.x, 
    pose.pose.position.y, 
    pose.pose.position.z
  );
  Eigen::Quaterniond cur_ori = Eigen::Quaterniond(
    pose.pose.orientation.w,
    pose.pose.orientation.x,
    pose.pose.orientation.y,
    pose.pose.orientation.z
  );

  try {
    std::vector<Eigen::Vector3d> cmd_pos, cmd_vel, cmd_acc, cmd_jerk;
    Eigen::Vector2d cmd_yaw( NAN, NAN);

    tm_compute_controls_.start();
    RCLCPP_INFO(get_logger(), "[getControllerCommand] Start computing command from mpc");
    controllers_[current_controller_]->computeCommands(
      cur_pos, cur_ori, cur_vel_,
      goal_checkers_[current_goal_checker_].get(),
      cmd_pos, cmd_vel, cmd_acc, cmd_jerk, cmd_yaw);
    RCLCPP_INFO(get_logger(), "[getControllerCommand] Successful computed command from mpc");

    tm_compute_controls_.stop(print_runtime_);
    // tm_compute_controls_.getWallAvg(print_runtime_);

    last_valid_cmd_time_ = now();

    {
    // Save previous valid trajector
      RCLCPP_INFO(get_logger(), "[getControllerCommand] Acquiring mpc_pred_lk(mpc_pred_mtx_) lock");
      std::lock_guard<std::mutex> mpc_pred_lk(mpc_pred_mtx_);
      RCLCPP_INFO(get_logger(), "[getControllerCommand] Acquired mpc_pred_lk(mpc_pred_mtx_) lock");
      cmd_pos_prev_ = cmd_pos;
      cmd_vel_prev_ = cmd_vel;
      cmd_acc_prev_ = cmd_acc;
      cmd_jerk_prev_ = cmd_jerk;
      cmd_yaw_prev_ = cmd_yaw;

      float yaw_pose = atan2(2.0 * (cur_ori.w()*cur_ori.z() + cur_ori.x()*cur_ori.y()), 
                          1.0 - 2.0 * (cur_ori.y()*cur_ori.y() + cur_ori.z()*cur_ori.z()));

      if (yaw_first && !(yaw_pose > cmd_yaw(0) - yaw_resolution_ && yaw_pose < cmd_yaw(0) + yaw_resolution_)){ // < 10, > -10 
        RCLCPP_INFO(get_logger(), "[getControllerCommand] Send yaw only first");
        cmd_yaw_prev_ = cmd_yaw;
        cmd_pos_prev_ = cmd_pos;
        cmd_vel_prev_ = {Eigen::Vector3d(std::nanf(""),std::nanf(""),std::nanf(""))};
        cmd_acc_prev_ = {Eigen::Vector3d(std::nanf(""),std::nanf(""),std::nanf(""))};
        cmd_jerk_prev_ = {Eigen::Vector3d(std::nanf(""),std::nanf(""),std::nanf(""))};
      }

      else{
        RCLCPP_INFO(get_logger(), "[getControllerCommand] Send mpc traj");
        yaw_first = false;
        cmd_pos_prev_ = cmd_pos;
        cmd_vel_prev_ = cmd_vel;
        cmd_acc_prev_ = cmd_acc;
        cmd_jerk_prev_ = cmd_jerk;
        cmd_yaw_prev_ = cmd_yaw;
      }


      RCLCPP_INFO(get_logger(), "[getControllerCommand] Unlock mpc_pred_lk(mpc_pred_mtx_) lock");
    }

    start_publish_cmd_ = true;

    // Only no valid control exception types are valid to attempt to have control patience, as
    // other types will not be resolved with more attempts
  } 
  catch (gestelt_core::NoValidControl & e) {
    RCLCPP_ERROR(this->get_logger(), "[ControllerServer::getControllerCommand] %s", e.what());
    RCLCPP_INFO(get_logger(), "[getControllerCommand]NoValidControl: Acquiring mpc_pred_lk(mpc_pred_mtx_) lock");
    std::lock_guard<std::mutex> mpc_pred_lk(mpc_pred_mtx_);
    RCLCPP_INFO(get_logger(), "[getControllerCommand]NoValidControl: Acquired mpc_pred_lk(mpc_pred_mtx_) lock");

    // If the path is stale (no new plan for kPathStaleSecs), hover in place.
    const double path_age = (now() - last_path_time_).seconds();
    if (curr_path_.empty() || path_age > kPathStaleSecs) {
      RCLCPP_WARN(get_logger(), "[getControllerCommand]NoValidControl: Path stale (%.1fs) or empty. Hovering at current odom.", path_age);
      cmd_pos_prev_ = { cur_pos };
      cmd_vel_prev_ = { Eigen::Vector3d(std::nanf(""), std::nanf(""), std::nanf("")) };
      cmd_acc_prev_ = { Eigen::Vector3d(std::nanf(""), std::nanf(""), std::nanf("")) };
      cmd_jerk_prev_ = {Eigen::Vector3d(std::nanf(""),std::nanf(""),std::nanf(""))};
      last_valid_cmd_time_ = now();
      start_publish_cmd_ = true;
      return;
    }

    Eigen::Vector3d nearest_cmd_pos = Eigen::Vector3d::Zero();
    auto nearest_cmd_dist = std::numeric_limits<double>::max();

    // Use global path to get out of no valid control //
    for (const auto check_cmd_pos : curr_path_) {
      const auto deltaVector = check_cmd_pos - cur_pos; //vector from check_cmd_pos to cur_pos
      const auto delta = deltaVector.squaredNorm();

      if ((delta < nearest_cmd_dist) && !(occ_map_->withinObstacleInflation(check_cmd_pos))) {
        nearest_cmd_dist = delta;
        nearest_cmd_pos = check_cmd_pos; //go back to nearest point
        // RCLCPP_INFO(get_logger(), "[getControllerCommand]NoValidControl: Register nearest_cmd_pos at (%f, %f, %f)",
        //             nearest_cmd_pos.x(),
        //             nearest_cmd_pos.y(),
        //             nearest_cmd_pos.z());
      } //if
    } //for

    if (curr_path_.empty()){
      RCLCPP_INFO(get_logger(), "[getControllerCommand]NoValidControl: Empty path");
      nearest_cmd_pos = cur_pos;
    }

    if (nearest_cmd_pos.isZero()){
      RCLCPP_WARN(get_logger(), "[getControllerCommand]NoValidControl: No valid points on global path. Setting to current odom position. "
                    "nearest_cmd_pos_ is (%f, %f, %f)",
                      nearest_cmd_pos.x(),
                      nearest_cmd_pos.y(),
                      nearest_cmd_pos.z());
      nearest_cmd_pos = cur_pos;
    }
    else{
      RCLCPP_INFO(get_logger(), "[getControllerCommand]NoValidControl nearest cmd_pos_ is set to waypoint of path closest to odom at (%f, %f, %f)",
        nearest_cmd_pos.x(),
        nearest_cmd_pos.y(),
        nearest_cmd_pos.z());
    }
    // Use global path to get out of no valid control //

    cmd_pos_prev_ = { nearest_cmd_pos };

    //set the desired velocity, according to closest waypoint
    static const double MAX_VEL_COMPONENT = 0.25;
    static const double MAX_VEL_COMPONENT_SQ = std::pow(MAX_VEL_COMPONENT, 2);
    Eigen::Vector3d nearest_cmd_vel = nearest_cmd_pos - cur_pos;
    // Eigen::Vector3d nearest_cmd_vel = -cmd_vel_prev_[0];

    if (nearest_cmd_vel.x() > MAX_VEL_COMPONENT)
      nearest_cmd_vel[0] = MAX_VEL_COMPONENT;
    else if (nearest_cmd_vel.x() < -MAX_VEL_COMPONENT)
      nearest_cmd_vel[0] = -MAX_VEL_COMPONENT;
    //else it is 0 or MAX_VEL_COMPONENT

    if (nearest_cmd_vel.y() > MAX_VEL_COMPONENT)
      nearest_cmd_vel[1] = MAX_VEL_COMPONENT;
    else if (nearest_cmd_vel.y() < -MAX_VEL_COMPONENT)
      nearest_cmd_vel[1] = -MAX_VEL_COMPONENT;
    //else it is 0 or MAX_VEL_COMPONENT

    if (nearest_cmd_vel.z() > MAX_VEL_COMPONENT)
      nearest_cmd_vel[2] = MAX_VEL_COMPONENT;
    else if (nearest_cmd_vel.z() < -MAX_VEL_COMPONENT)
      nearest_cmd_vel[2] = -MAX_VEL_COMPONENT;
    //else it is 0 or MAX_VEL_COMPONENT

    cmd_vel_prev_ = { nearest_cmd_vel };
    // cmd_vel_prev_ = { Eigen::Vector3d::Zero() };
    // cmd_vel_prev_ = { Eigen::Vector3d(std::nanf(""),std::nanf(""),std::nanf("")) };

    //set the desired acceleration
    // cmd_acc_prev_ = { Eigen::Vector3d(0.25, 0.25, 0.25) };
    // cmd_acc_prev_ = { Eigen::Vector3d::Zero() };
    cmd_acc_prev_ = { Eigen::Vector3d(std::nanf(""),std::nanf(""),std::nanf("")) };
    cmd_jerk_prev_ = {Eigen::Vector3d(std::nanf(""),std::nanf(""),std::nanf(""))};

    RCLCPP_INFO(this->get_logger(), "[getControllerCommand]NoValidControl: Setting waypoint to nearest cmd_pos_ (%f, %f, %f)", 
      nearest_cmd_pos.x(),
      nearest_cmd_pos.y(),
      nearest_cmd_pos.z());
    
    last_valid_cmd_time_ = now();
    start_publish_cmd_ = true;
    RCLCPP_INFO(get_logger(), "[getControllerCommand]NoValidControl: Unlock mpc_pred_lk(mpc_pred_mtx_) lock");
  }

  // TODO: Speed feedback here is not implemented
  std::shared_ptr<Action::Feedback> feedback = std::make_shared<Action::Feedback>();
  feedback->speed = Eigen::Vector3d(0.0, 0.0, 0.0).norm();
    
  nav_msgs::msg::Path & current_path = current_path_;
  auto find_closest_pose_idx =
    [&pose, &current_path]() {
      size_t closest_pose_idx = 0;
      double curr_min_dist = std::numeric_limits<double>::max();
      for (size_t curr_idx = 0; curr_idx < current_path.poses.size(); ++curr_idx) {
        double curr_dist = nav2_util::geometry_utils::euclidean_distance(
          pose, current_path.poses[curr_idx]);
        if (curr_dist < curr_min_dist) {
          curr_min_dist = curr_dist;
          closest_pose_idx = curr_idx;
        }
      }
      return closest_pose_idx;
    };

  feedback->distance_to_goal =
    nav2_util::geometry_utils::calculate_path_length(current_path_, find_closest_pose_idx());

  action_server_->publish_feedback(feedback);
}

void ControllerServer::updateGlobalPath()
{
  RCLCPP_INFO(get_logger(), "[updateGlobalPath] Updating global path");
  if (action_server_->is_preempt_requested()) {
    RCLCPP_INFO(get_logger(), "Passing new path to controller.");
    auto goal = action_server_->accept_pending_goal();
    std::string current_controller;
    if (findControllerId(goal->controller_id, current_controller)) {
      current_controller_ = current_controller;
    } else {
      RCLCPP_INFO(
        get_logger(), "Terminating action, invalid controller %s requested.",
        goal->controller_id.c_str());
      action_server_->terminate_current();
      return;
    }
    std::string current_goal_checker;
    if (findGoalCheckerId(goal->goal_checker_id, current_goal_checker)) {
      current_goal_checker_ = current_goal_checker;
    } else {
      RCLCPP_INFO(
        get_logger(), "Terminating action, invalid goal checker %s requested.",
        goal->goal_checker_id.c_str());
      action_server_->terminate_current();
      return;
    }

    setPlannerPath(goal->path);
    RCLCPP_INFO(get_logger(), "[updateGlobalPath] Updated global path");
  }
}

void ControllerServer::onGoalExit()
{
  if (publish_zero_velocity_) {
    RCLCPP_INFO(get_logger(), "[onGoaExit] Publish zero vel");
    px4_msgs::msg::TrajectorySetpoint traj_sp;
    traj_sp.position = {(float) cur_pos_.x() , (float) cur_pos_.y(), (float) cur_pos_.z()};
    traj_sp.velocity = {0.0, 0.0, 0.0};
    traj_sp.acceleration = {0.0, 0.0, 0.0};
    traj_sp.yaw = NAN;
    traj_sp.yawspeed = NAN;
    traj_sp.timestamp = now().nanoseconds() / 1000; // In microseconds
    cmd_pub_->publish(traj_sp);
  }

  // Reset the state of the controllers after the task has ended
  ControllerMap::iterator it;
  for (it = controllers_.begin(); it != controllers_.end(); ++it) {
    it->second->reset();
  }
}

bool ControllerServer::isGoalReached()
{
  geometry_msgs::msg::PoseStamped pose;

  if (!getRobotPose(pose)) {
    RCLCPP_ERROR(get_logger(), "[isGoalReached]Unable to getRobotPose");
    return false;
  }

  // Stuff to test (Transform pose from map frame to global for goal checking) //
  geometry_msgs::msg::PoseStamped transformed_pose;
  rclcpp::Duration tolerance(rclcpp::Duration::from_seconds(occ_map_->getTransformTolerance()));
  nav_2d_utils::transformPose(
    occ_map_->getTfBuffer(), occ_map_->getGlobalFrameID(),
    pose, transformed_pose, tolerance);
  RCLCPP_INFO(get_logger(), "[isGoalReached] pose(map frame): (%.2f, %.2f, %.2f), transformed_pose(global_frame): (%.2f, %.2f, %.2f), end_pose_(global_frame): (%.2f, %.2f, %.2f)",
                              pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
                              transformed_pose.pose.position.x, transformed_pose.pose.position.y, transformed_pose.pose.position.z,
                              end_pose_.pose.position.x, end_pose_.pose.position.y, end_pose_.pose.position.z);
  // Stuff to test (Transform pose from map frame to global for goal checking) //

  return goal_checkers_[current_goal_checker_]->isGoalReached(
    transformed_pose.pose, end_pose_.pose,
    cur_twist_);
}

bool ControllerServer::getRobotPose(geometry_msgs::msg::PoseStamped & pose)
{
  geometry_msgs::msg::PoseStamped current_pose;
  if (!occ_map_->getRobotPose(current_pose)) {
    return false;
  }
  pose = current_pose;
  return true;
}

bool ControllerServer::odomWithinObstacleInflation()
{
  RCLCPP_INFO(get_logger(), "Check if odom is within inflation");
  geometry_msgs::msg::PoseStamped pose;
  if (!getRobotPose(pose)) {
    throw gestelt_core::ControllerTFError("Failed to obtain robot pose");
  }
  const Eigen::Vector3d temp_pose = Eigen::Vector3d(pose.pose.position.x, 
                                                    pose.pose.position.y,
                                                    pose.pose.position.z);

  if(occ_map_->withinObstacleInflation(temp_pose)){
    RCLCPP_WARN(get_logger(), "Odom from occ map (%.2f, %.2f, %.2f) is within obstacle inflation", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
    return true;
  }

  RCLCPP_INFO(get_logger(), "Odom is free from inflation");
  return false;

}

rcl_interfaces::msg::SetParametersResult
ControllerServer::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;

  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    // If we are trying to change the parameter of a plugin we can just skip it at this point
    // as they handle parameter changes themselves and don't need to lock the mutex
    if (name.find('.') != std::string::npos) {
      continue;
    }

    if (!dynamic_params_lock_.try_lock()) {
      RCLCPP_WARN(
        get_logger(),
        "Unable to dynamically change Parameters while the controller is currently running");
      result.successful = false;
      result.reason =
        "Unable to dynamically change Parameters while the controller is currently running";
      return result;
    }

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == "controller_frequency") {
        controller_frequency_ = parameter.as_double();
      } else if (name == "failure_tolerance") {
        failure_tolerance_ = parameter.as_double();
      }
    }

    dynamic_params_lock_.unlock();
  }

  result.successful = true;
  return result;
}

void ControllerServer::odometrySubCB(const nav_msgs::msg::Odometry::UniquePtr msg)
{
  cur_pos_ = Eigen::Vector3d(
    msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z
  );

  cur_vel_ = Eigen::Vector3d(
    msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z
  );

  cur_twist_ = msg->twist.twist;
  
}

void ControllerServer::globalPlanSubCB(const nav_msgs::msg::Path::UniquePtr msg){

  msg->header.frame_id = occ_map_->getMapFrameID();

  curr_path_.clear();
  for (int i = 0; i < msg->poses.size(); i++){
    curr_path_.push_back(Eigen::Vector3d(
      msg->poses[i].pose.position.x,
      msg->poses[i].pose.position.y,
      msg->poses[i].pose.position.z));
  }
  last_path_time_ = now();

  current_path_pub_->publish(*msg);
}

} // namespace gestelt_controller

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(gestelt_controller::ControllerServer)
