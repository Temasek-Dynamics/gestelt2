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

#ifndef LINEAR_MPC_CONTROLLER__LINEAR_MPC_CONTROLLER_HPP_
#define LINEAR_MPC_CONTROLLER__LINEAR_MPC_CONTROLLER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <mutex>

#include <Eigen/Eigen>

#include "gestelt_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "nav2_util/geometry_utils.hpp"

#include "linear_mpc_controller/poly_sfc_gen.hpp"
#include "linear_mpc_controller/pvaj_mpc.hpp"

#include "occ_map/occ_map.hpp"

// Messages for SFC visualization
#include <decomp_ros_msgs/msg/ellipsoid_array.hpp>
#include <decomp_ros_msgs/msg/polyhedron_array.hpp>

namespace linear_mpc_controller
{

/**
 * @class linear_mpc_controller::LinearMPCController
 * @brief Regulated pure pursuit controller plugin
 */
class LinearMPCController : public gestelt_core::Controller
{
public:
  /**
   * @brief Constructor for linear_mpc_controller::LinearMPCController
   */
  LinearMPCController() = default;

  /**
   * @brief Destrructor for linear_mpc_controller::LinearMPCController
   */
  ~LinearMPCController() override = default;

  /**
   * @brief Configure controller state machine
   * @param parent WeakPtr to node
   * @param name Name of plugin
   * @param tf TF buffer
   * @param costmap_ros Costmap2DROS object of environment
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<occ_map::OccMap> occ_map) override;

  /**
   * @brief Cleanup controller state machine
   */
  void cleanup() override;

  /**
   * @brief Activate controller state machine
   */
  void activate() override;

  /**
   * @brief Deactivate controller state machine
   */
  void deactivate() override;

  /**
   * @brief Compute the best command given the current pose and velocity, with possible debug information
   *
   * Same as above computeCommands, but with debug results.
   * If the results pointer is not null, additional information about the twists
   * evaluated will be in results after the call.
   *
   * @param pose      Current robot pose
   * @param velocity  Current robot velocity
   * @param goal_checker   Ptr to the goal checker for this task in case useful in computing commands
   * @return          Best command
   */
  px4_msgs::msg::TrajectorySetpoint computeCommands(
    const Eigen::Vector3d & pose,
    const Eigen::Quaterniond & orientation,
    const Eigen::Vector3d & velocity,
    gestelt_core::GoalChecker * goal_checker) override;

  /**
   * @brief gestelt_core setPlan - Sets the global plan
   * @param path The global plan
   */
  void setPlan(const nav_msgs::msg::Path & path) override;

protected:
  /**
   * @brief Transforms global plan into same frame as pose and clips poses ineligible for lookaheadPoint
   * Points ineligible to be selected as a lookahead point if they are any of the following:
   * - Outside the local_costmap (collision avoidance cannot be assured)
   * @param pose pose to transform
   * @return Path in new frame
   */
  nav_msgs::msg::Path transformPlanFromGlobalToMap(
    const geometry_msgs::msg::PoseStamped & pose);

  template<typename Iter, typename Getter>
  inline Iter first_after_integrated_distance(Iter begin, Iter end, Getter getCompareVal)
  {
    if (begin == end) {
      return end;
    }
    Getter dist = 0.0;
    for (Iter it = begin; it != end - 1; it++) {
      dist += euclidean_distance(*it, *(it + 1));
      if (dist > getCompareVal) {
        return it + 1;
      }
    }
    return end;
  }

  /**
   * @brief Get the euclidean distance between 2 geometry_msgs::Points
   * @param pos1 First point
   * @param pos1 Second point
   * @param is_3d True if a true L2 distance is desired (default false)
   * @return double L2 distance
   */
  inline double euclidean_distance(
    const geometry_msgs::msg::PoseStamped & pos1,
    const geometry_msgs::msg::PoseStamped & pos2)
  {
    double dx = pos1.pose.position.x - pos2.pose.position.x;
    double dy = pos1.pose.position.y - pos2.pose.position.y;
    double dz = pos1.pose.position.z - pos2.pose.position.z;

    return std::hypot(dx, dy, dz);
  }

  /**
   * Find element in iterator with the minimum calculated value
   */
  template<typename Iter, typename Getter>
  inline Iter min_by(Iter begin, Iter end, Getter getCompareVal)
  {
    if (begin == end) {
      return end;
    }
    auto lowest = getCompareVal(*begin);
    Iter lowest_it = begin;
    for (Iter it = ++begin; it != end; ++it) {
      auto comp = getCompareVal(*it);
      if (comp < lowest) {
        lowest = comp;
        lowest_it = it;
      }
    }
    return lowest_it;
  }
  
  // /**
  //  * @brief Callback executed when a parameter change is detected
  //  * @param event ParameterEvent message
  //  */
  // rcl_interfaces::msg::SetParametersResult
  // dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  std::shared_ptr<occ_map::OccMap> occ_map_;
  rclcpp::Logger logger_ {rclcpp::get_logger("LinearMPCController")};
  rclcpp::Clock::SharedPtr clock_;
  
  // Params
  int yaw_lookahead_dist_{5};
  double max_robot_pose_search_dist_{1.0};
  // tf2::Duration transform_tolerance_;

  // Data
  nav_msgs::msg::Path global_plan_; // Global plan in global frame
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> 
    global_path_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<decomp_ros_msgs::msg::PolyhedronArray>> 
    sfc_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> 
    mpc_traj_pub_;
    

  std::unique_ptr<sfc::PolytopeSFC> sfc_gen_; // Polytope safe flight corridor generator
  std::unique_ptr<pvaj_mpc::MPCController> mpc_controller_; // MPC controller

  // Dynamic parameters handler
  std::mutex mutex_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
};

}  // namespace linear_mpc_controller

#endif  // LINEAR_MPC_CONTROLLER__LINEAR_MPC_CONTROLLER_HPP_
