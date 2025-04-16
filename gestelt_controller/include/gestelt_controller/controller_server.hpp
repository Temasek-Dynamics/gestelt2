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

#ifndef GESTELT_CONTROLLER__CONTROLLER_SERVER_HPP_
#define GESTELT_CONTROLLER__CONTROLLER_SERVER_HPP_

#include <Eigen/Eigen>

#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
#include <mutex>

#include "gestelt_core/progress_checker.hpp"
#include "gestelt_core/goal_checker.hpp"

#include "tf2_ros/transform_listener.h"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_msgs/msg/speed_limit.hpp"

#include "px4_msgs/msg/trajectory_setpoint.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "nav2_util/robot_utils.hpp"

#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "occ_map/occ_map.hpp"

#include "gestelt_core/controller.hpp"

namespace gestelt_controller
{
class ProgressChecker;

/**
 * @class gestelt_controller::ControllerServer
 * @brief This class hosts variety of plugins of different algorithms to
 * complete control tasks from the exposed FollowPath action server.
 */
class ControllerServer : public nav2_util::LifecycleNode
{
public:
  using ControllerMap = std::unordered_map<std::string, gestelt_core::Controller::Ptr>;
  using GoalCheckerMap = std::unordered_map<std::string, gestelt_core::GoalChecker::Ptr>;
  using ProgressCheckerMap = std::unordered_map<std::string, gestelt_core::ProgressChecker::Ptr>;

  /**
   * @brief Constructor for gestelt_controller::ControllerServer
   * @param options Additional options to control creation of the node.
   */
  explicit ControllerServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  /**
   * @brief Destructor for gestelt_controller::ControllerServer
   */
  ~ControllerServer();

protected:
  /**
   * @brief Configures controller parameters and member variables
   *
   * Configures controller plugin and occupancy map; Initialize odom subscriber,
   * velocity publisher and follow path action server.
   * @param state LifeCycle Node's state
   * @return Success or Failure
   * @throw pluginlib::PluginlibException When failed to initialize controller
   * plugin
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Activates member variables
   *
   * Activates controller, occupancy map, velocity publisher and follow path action
   * server
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Deactivates member variables
   *
   * Deactivates follow path action server, controller, occupancy map and velocity
   * publisher. Before calling deactivate state, velocity is being set to zero.
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Calls clean up states and resets member variables.
   *
   * Controller and occupancy map clean up state is called, and resets rest of the
   * variables
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Called when in Shutdown state
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  using Action = nav2_msgs::action::FollowPath;
  using ActionServer = nav2_util::SimpleActionServer<Action>;

  // Our action server implements the FollowPath action
  std::unique_ptr<ActionServer> action_server_;

  /**
   * @brief FollowPath action server callback. Handles action server updates and
   * spins server until goal is reached
   *
   * Provides global path to controller received from action client. Twist
   * velocities for the robot are calculated and published using controller at
   * the specified rate till the goal is reached.
   * @throw gestelt_core::PlannerException
   */
  void computeControl();

  /**
   * @brief Find the valid controller ID name for the given request
   *
   * @param c_name The requested controller name
   * @param name Reference to the name to use for control if any valid available
   * @return bool Whether it found a valid controller to use
   */
  bool findControllerId(const std::string & c_name, std::string & name);

  /**
   * @brief Find the valid goal checker ID name for the specified parameter
   *
   * @param c_name The goal checker name
   * @param name Reference to the name to use for goal checking if any valid available
   * @return bool Whether it found a valid goal checker to use
   */
  bool findGoalCheckerId(const std::string & c_name, std::string & name);

  /**
   * @brief Assigns path to controller
   * @param path Path received from action server
   */
  void setPlannerPath(const nav_msgs::msg::Path & path);

  /**
   * @brief Calculates velocity and publishes to intermediate_cmd topic
   */
  void computeAndPublishControl();

  /**
   * @brief Calls setPlannerPath method with an updated path received from
   * action server
   */
  void updateGlobalPath();

  /**
   * @brief Called on goal exit
   */
  void onGoalExit();
  /**
   * @brief Checks if goal is reached
   * @return true or false
   */
  bool isGoalReached();

  /**
   * @brief Obtain current pose of the robot
   * @param pose To store current pose of the robot
   * @return true if able to obtain current pose of the robot, else false
   */
  bool getRobotPose(geometry_msgs::msg::PoseStamped & pose);


  void odometrySubCB(const nav_msgs::msg::Odometry::UniquePtr msg);

  /**
   * @brief Callback executed when a parameter change is detected
   * @param event ParameterEvent message
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  std::mutex dynamic_params_lock_;

  // The controller needs a local occupancy map
  std::shared_ptr<occ_map::OccMap> occ_map_;
  std::unique_ptr<nav2_util::NodeThread> occ_map_thread_;

  // Publishers and subscribers

	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp_lifecycle::LifecyclePublisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr cmd_pub_;


  // Progress Checker Plugin
  pluginlib::ClassLoader<gestelt_core::ProgressChecker> progress_checker_loader_;
  gestelt_core::ProgressChecker::Ptr progress_checker_;
  std::string default_progress_checker_id_;
  std::string default_progress_checker_type_;
  std::string progress_checker_id_;
  std::string progress_checker_type_;

  // Goal Checker Plugin
  pluginlib::ClassLoader<gestelt_core::GoalChecker> goal_checker_loader_;
  GoalCheckerMap goal_checkers_;
  std::vector<std::string> default_goal_checker_ids_;
  std::vector<std::string> default_goal_checker_types_;
  std::vector<std::string> goal_checker_ids_;
  std::vector<std::string> goal_checker_types_;
  std::string goal_checker_ids_concat_, current_goal_checker_;

  // Controller Plugins
  pluginlib::ClassLoader<gestelt_core::Controller> lp_loader_;
  ControllerMap controllers_;
  std::vector<std::string> default_ids_;
  std::vector<std::string> default_types_;
  std::vector<std::string> controller_ids_;
  std::vector<std::string> controller_types_;
  std::string controller_ids_concat_, current_controller_;


  bool publish_zero_velocity_;
  rclcpp::Duration occ_map_update_timeout_;
  double failure_tolerance_{0.0};

  double controller_frequency_{0.0};

  bool use_realtime_priority_;

  rclcpp::Time last_valid_cmd_time_;

  Eigen::Vector3d cur_pos_; // Current position
  Eigen::Vector3d cur_vel_; // Current velocity vector
  geometry_msgs::msg::Twist cur_twist_; // Current twist

  // Whether we've published the single controller warning yet
  geometry_msgs::msg::PoseStamped end_pose_;

  // Current path container
  nav_msgs::msg::Path current_path_;

private:

};


}  // namespace gestelt_controller

#endif  // GESTELT_CONTROLLER__CONTROLLER_SERVER_HPP_
