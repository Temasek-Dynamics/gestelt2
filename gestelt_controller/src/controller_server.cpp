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
  default_types_{"dwb_core::DWBLocalPlanner"},
  occ_map_update_timeout_(300ms)
{
  RCLCPP_INFO(get_logger(), "Creating controller server");

  declare_parameter("controller_frequency", 20.0);

  declare_parameter("action_server_result_timeout", 10.0);

  declare_parameter("progress_checker_plugin", default_progress_checker_id_);
  declare_parameter("goal_checker_plugins", default_goal_checker_ids_);
  declare_parameter("controller_plugins", default_ids_);

  declare_parameter("speed_limit_topic", rclcpp::ParameterValue("speed_limit"));

  declare_parameter("failure_tolerance", rclcpp::ParameterValue(0.0));
  declare_parameter("publish_zero_velocity", rclcpp::ParameterValue(true));
  declare_parameter("occ_map_update_timeout", 0.30);  // 300ms

  declare_parameter("controller_look_ahead_index", rclcpp::ParameterValue(1));
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

  get_parameter("controller_look_ahead_index", controller_look_ahead_index_);
  get_parameter("print_runtime", print_runtime_);

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

  cmd_pub_ = node->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
    "intermediate_cmd", rclcpp::SensorDataQoS());

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
      true);

  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR(get_logger(), "Error creating action server! %s", e.what());
    on_cleanup(state);
    return nav2_util::CallbackReturn::FAILURE;
  }

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
  action_server_->activate();

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
  occ_map_.reset();
  occ_map_thread_.reset();
  cmd_pub_.reset();

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
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);

  RCLCPP_INFO(get_logger(), "Received a goal, begin computing control effort.");

  try {
    auto goal = action_server_->get_current_goal();
    if (!goal) {
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

      computeAndPublishControl();

      if (isGoalReached()) {
        RCLCPP_INFO(get_logger(), "Reached the goal!");
        break;
      }

      auto cycle_duration = this->now() - start_time;
      if (!loop_rate.sleep()) {
        RCLCPP_WARN(
          get_logger(),
          "Control loop missed its desired rate of %.4f Hz. Current loop rate is %.4f Hz.",
          controller_frequency_, 1 / cycle_duration.seconds());
      }
    }
  } catch (gestelt_core::InvalidController & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    onGoalExit();
    std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
    // result->error_code = Action::Result::INVALID_CONTROLLER;
    action_server_->terminate_current(result);
    return;
  } catch (gestelt_core::ControllerTFError & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    onGoalExit();
    std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
    // result->error_code = Action::Result::TF_ERROR;
    action_server_->terminate_current(result);
    return;
  } catch (gestelt_core::NoValidControl & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    onGoalExit();
    std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
    // result->error_code = Action::Result::NO_VALID_CONTROL;
    action_server_->terminate_current(result);
    return;
  } catch (gestelt_core::FailedToMakeProgress & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    onGoalExit();
    std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
    // result->error_code = Action::Result::FAILED_TO_MAKE_PROGRESS;
    action_server_->terminate_current(result);
    return;
  } catch (gestelt_core::PatienceExceeded & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    onGoalExit();
    std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
    // result->error_code = Action::Result::PATIENCE_EXCEEDED;
    action_server_->terminate_current(result);
    return;
  } catch (gestelt_core::InvalidPath & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    onGoalExit();
    std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
    // result->error_code = Action::Result::INVALID_PATH;
    action_server_->terminate_current(result);
    return;
  } catch (gestelt_core::ControllerTimedOut & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    onGoalExit();
    std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
    // result->error_code = Action::Result::CONTROLLER_TIMED_OUT;
    action_server_->terminate_current(result);
    return;
  } catch (gestelt_core::ControllerException & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    onGoalExit();
    std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
    // result->error_code = Action::Result::UNKNOWN;
    action_server_->terminate_current(result);
    return;
  } catch (std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    onGoalExit();
    std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
    // result->error_code = Action::Result::UNKNOWN;
    action_server_->terminate_current(result);
    return;
  }

  RCLCPP_DEBUG(get_logger(), "Controller succeeded, setting result");

  onGoalExit();

  // TODO(orduno) #861 Handle a pending preemption and set controller name
  action_server_->succeeded_current();
}


void ControllerServer::setPlannerPath(const nav_msgs::msg::Path & path)
{
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
}

void ControllerServer::computeAndPublishControl()
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

  // Trajectory setpoint is in ENU frame 
  // (Transforming to NED frame is done by trajectory server)
  px4_msgs::msg::TrajectorySetpoint traj_sp;

  try {
    std::vector<Eigen::Vector3d> mpc_pred_pos, mpc_pred_vel, mpc_pred_acc, mpc_pred_u;
    Eigen::Vector2d mpc_yaw( NAN, NAN);

    tm_compute_controls_.start();

    controllers_[current_controller_]->computeCommands(
      cur_pos, cur_ori, cur_vel_,
      goal_checkers_[current_goal_checker_].get(),
      mpc_pred_pos, mpc_pred_vel, mpc_pred_acc, mpc_pred_u, mpc_yaw);

    tm_compute_controls_.stop(false);
    tm_compute_controls_.getWallAvg(print_runtime_);

    int idx = controller_look_ahead_index_ < mpc_pred_pos.size() ? controller_look_ahead_index_ : mpc_pred_pos.size()-1 ; 

    traj_sp.position = 
      {(float) mpc_pred_pos[idx](0) , (float) mpc_pred_pos[idx](1), (float) mpc_pred_pos[idx](2)};
    traj_sp.velocity = 
      {(float) mpc_pred_vel[idx](0) , (float) mpc_pred_vel[idx](1), (float) mpc_pred_vel[idx](2)};
    traj_sp.acceleration = 
      {(float) mpc_pred_acc[idx](0) , (float) mpc_pred_acc[idx](1), (float) mpc_pred_acc[idx](2)};
    traj_sp.yaw = mpc_yaw(0);
    traj_sp.yawspeed = mpc_yaw(1);

    last_valid_cmd_time_ = now();
    traj_sp.timestamp = last_valid_cmd_time_.nanoseconds() / 1000; // In microseconds
    // Only no valid control exception types are valid to attempt to have control patience, as
    // other types will not be resolved with more attempts
  } catch (gestelt_core::NoValidControl & e) {
    if (failure_tolerance_ > 0 || failure_tolerance_ == -1.0) {
      RCLCPP_WARN(this->get_logger(), "%s", e.what());
      traj_sp.position = {(float) pose.pose.position.x , (float) pose.pose.position.y, (float) pose.pose.position.z};
      traj_sp.velocity = {0.0, 0.0, 0.0};
      traj_sp.acceleration = {0.0, 0.0, 0.0};
      traj_sp.yaw = NAN;
      traj_sp.yawspeed = NAN;
      traj_sp.timestamp = now().nanoseconds() / 1000; // In microseconds
      if ((now() - last_valid_cmd_time_).seconds() > failure_tolerance_ &&
        failure_tolerance_ != -1.0)
      {
        throw gestelt_core::PatienceExceeded("Controller patience exceeded");
      }
    } else {
      throw gestelt_core::NoValidControl(e.what());
    }
  }

  std::shared_ptr<Action::Feedback> feedback = std::make_shared<Action::Feedback>();
  feedback->speed = Eigen::Vector3d(
    traj_sp.velocity[0], traj_sp.velocity[1], traj_sp.velocity[2]).norm();
    
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

  RCLCPP_DEBUG(get_logger(), "Publishing command at time %.2f", now().seconds());
  
  cmd_pub_->publish(traj_sp);
}

void ControllerServer::updateGlobalPath()
{
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
  }
}

void ControllerServer::onGoalExit()
{
  if (publish_zero_velocity_) {
    px4_msgs::msg::TrajectorySetpoint traj_sp;
    traj_sp.position = {(float) cur_pos_(0) , (float) cur_pos_(1), (float) cur_pos_(2)};
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
    return false;
  }

  geometry_msgs::msg::PoseStamped transformed_end_pose;
  rclcpp::Duration tolerance(rclcpp::Duration::from_seconds(occ_map_->getTransformTolerance()));
  nav_2d_utils::transformPose(
    occ_map_->getTfBuffer(), occ_map_->getGlobalFrameID(),
    end_pose_, transformed_end_pose, tolerance);

  return goal_checkers_[current_goal_checker_]->isGoalReached(
    pose.pose, transformed_end_pose.pose,
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

} // namespace gestelt_controller

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(gestelt_controller::ControllerServer)
