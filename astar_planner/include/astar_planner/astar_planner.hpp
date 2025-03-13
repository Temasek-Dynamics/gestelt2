// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2018 Simbe Robotics
// Copyright (c) 2019 Samsung Research America
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

#ifndef ASTAR_PLANNER__ASTAR_PLANNER_HPP_
#define ASTAR_PLANNER__ASTAR_PLANNER_HPP_

#include <chrono>
#include <string>
#include <memory>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_core/planner_exceptions.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/geometry_utils.hpp"

#include <occ_map/occ_map.hpp>

namespace astar_planner
{

/**
 * @class GlobalPlanner
 * @brief Abstract interface for global planners to adhere to with pluginlib
 */
class AStarPlanner : public nav2_core::GlobalPlanner
{
public:
  AStarPlanner();

  ~AStarPlanner();

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief Cleanup lifecycle node
   */
  void cleanup() override;

  /**
   * @brief Activate lifecycle node
   */
  void activate() override;

  /**
   * @brief Deactivate lifecycle node
   */
  void deactivate() override;

  /**
   * @brief Creating a plan from start and goal poses
   * @param start Start pose
   * @param goal Goal pose
   * @param cancel_checker Function to check if the task has been canceled
   * @return nav_msgs::Path of the generated path
   */
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker) override;

protected:

  /**
   * @brief Compute a plan given start and goal poses, provided in global world frame.
   * @param start Start pose
   * @param goal Goal pose
   * @param tolerance Relaxation constraint in x and y
   * @param cancel_checker Function to check if the task has been canceled
   * @param plan Path to be computed
   * @return true if can find the path
   */
  bool makePlan(
    const geometry_msgs::msg::Pose & start,
    const geometry_msgs::msg::Pose & goal, double tolerance,
    std::function<bool()> cancel_checker,
    nav_msgs::msg::Path & plan);

  /* Convert */
  // /**
  //  * @brief Set the corresponding cell cost to be free space
  //  * @param pos Position in map frame
  //  */
  // void clearRobotCell(const Eigen::Vector3d& pos);

  // Planner based on ROS1 NavFn algorithm
  std::unique_ptr<AStar> planner_;

  // TF buffer
  std::shared_ptr<tf2_ros::Buffer> tf_;

  std::shared_ptr<OccMap::OccMap> occ_map_;

  /* Variables */

  // The global frame of the costmap
  std::string map_frame_, global_frame_, name_;

  /* Parameters */

  // If the goal is obstructed, the tolerance specifies how many meters the planner
  // can relax the constraint in x and y before failing
  double tolerance_{0.5};

  int max_iterations_{99999};

  double tie_breaker_{1.001};

  int cost_function_type_{2};

  bool print_runtime_{false};

  // parent node weak ptr
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;

  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

  /**
   * @brief Callback executed when a parameter change is detected
   * @param parameters list of changed parameters
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);
};

}  // namespace astar_planner

#endif  // ASTAR_PLANNER__ASTAR_PLANNER_HPP_
