// Copyright (c) 2020 Shrijit Singh
// Copyright (c) 2020 Samsung Research America
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

#include <algorithm>
#include <string>
#include <limits>
#include <memory>
#include <vector>
#include <utility>

#include "linear_mpc_controller/linear_mpc_controller.hpp"
#include "gestelt_core/planner_exceptions.hpp"
#include "gestelt_core/controller_exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"

using std::hypot;
using std::min;
using std::max;
using std::abs;
using nav2_util::declare_parameter_if_not_declared;
using rcl_interfaces::msg::ParameterType;

namespace linear_mpc_controller
{

void LinearMPCController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<occ_map::OccMap> occ_map)
{
  tf_ = tf;
  plugin_name_ = name;
  occ_map_ = occ_map;

  node_ = parent;
  auto node = parent.lock();
  if (!node) {
    throw gestelt_core::ControllerException("Unable to lock node!");
  }
  clock_ = node->get_clock();
  logger_ = node->get_logger();

  RCLCPP_INFO(
    logger_, "Configuring plugin %s of type AStarPlanner",
    plugin_name_.c_str());

  // double transform_tolerance = 0.1;
  // double control_frequency = 20.0;
  // transform_tolerance_ = tf2::durationFromSec(transform_tolerance);
  // control_duration_ = 1.0 / control_frequency;

  sfc_pub_ = node->create_publisher<decomp_ros_msgs::msg::PolyhedronArray>("sfc", 1);
  global_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);

  // Declare this plugin's parameters
  declare_parameter_if_not_declared(node, name + ".yaw_lookahead_dist", rclcpp::ParameterValue(5));
  node->get_parameter(name + ".yaw_lookahead_dist", yaw_lookahead_dist_);
  declare_parameter_if_not_declared(node, name + ".max_robot_pose_search_dist", rclcpp::ParameterValue(1.0));
  node->get_parameter(name + ".max_robot_pose_search_dist", max_robot_pose_search_dist_);

  // SFC
  sfc::PolytopeSFCParams sfc_params;

  declare_parameter_if_not_declared(node, name + ".sfc.bbox_x", rclcpp::ParameterValue(2.0));
  node->get_parameter(name + ".sfc.bbox_x", sfc_params.bbox_x);
  declare_parameter_if_not_declared(node, name + ".sfc.bbox_y", rclcpp::ParameterValue(2.0));
  node->get_parameter(name + ".sfc.bbox_y", sfc_params.bbox_y);
  declare_parameter_if_not_declared(node, name + ".sfc.bbox_z", rclcpp::ParameterValue(2.0));
  node->get_parameter(name + ".sfc.bbox_z", sfc_params.bbox_z);
  declare_parameter_if_not_declared(node, name + ".sfc.plan_sample_interval", rclcpp::ParameterValue(1));
  node->get_parameter(name + ".sfc.plan_sample_interval", sfc_params.plan_samp_intv);
  
  // MPC
  pvaj_mpc::MPCControllerParams mpc_params;

  declare_parameter_if_not_declared(node, name + ".mpc.ctrl_samp_freq", rclcpp::ParameterValue(30.0));
  node->get_parameter(name + ".mpc.ctrl_samp_freq", mpc_params.ctrl_samp_freq);
  declare_parameter_if_not_declared(node, name + ".mpc.plan_sample_interval", rclcpp::ParameterValue(1));
  node->get_parameter(name + ".mpc.plan_sample_interval", mpc_params.plan_samp_intv);
  declare_parameter_if_not_declared(node, name + ".mpc.yaw_ctrl_flag", rclcpp::ParameterValue(true));
  node->get_parameter(name + ".mpc.yaw_ctrl_flag", mpc_params.yaw_ctrl_flag);

  declare_parameter_if_not_declared(node, name + ".mpc.horizon", rclcpp::ParameterValue(15));
  node->get_parameter(name + ".mpc.horizon", mpc_params.MPC_HORIZON);
  declare_parameter_if_not_declared(node, name + ".mpc.time_step", rclcpp::ParameterValue(0.1));
  node->get_parameter(name + ".mpc.time_step", mpc_params.MPC_STEP);
  declare_parameter_if_not_declared(node, name + ".mpc.R_p", rclcpp::ParameterValue(1000.0));
  node->get_parameter(name + ".mpc.R_p", mpc_params.R_p);
  declare_parameter_if_not_declared(node, name + ".mpc.R_v", rclcpp::ParameterValue(0.0));
  node->get_parameter(name + ".mpc.R_v", mpc_params.R_v);
  declare_parameter_if_not_declared(node, name + ".mpc.R_a", rclcpp::ParameterValue(0.0));
  node->get_parameter(name + ".mpc.R_a", mpc_params.R_a);
  declare_parameter_if_not_declared(node, name + ".mpc.R_u", rclcpp::ParameterValue(0.0));
  node->get_parameter(name + ".mpc.R_u", mpc_params.R_u);
  declare_parameter_if_not_declared(node, name + ".mpc.R_u_con", rclcpp::ParameterValue(0.2));
  node->get_parameter(name + ".mpc.R_u_con", mpc_params.R_u_con);
  declare_parameter_if_not_declared(node, name + ".mpc.R_pN", rclcpp::ParameterValue(2000.0));
  node->get_parameter(name + ".mpc.R_pN", mpc_params.R_pN);
  declare_parameter_if_not_declared(node, name + ".mpc.R_vN", rclcpp::ParameterValue(1000.0));
  node->get_parameter(name + ".mpc.R_vN", mpc_params.R_vN);
  declare_parameter_if_not_declared(node, name + ".mpc.R_aN", rclcpp::ParameterValue(1000.0));
  node->get_parameter(name + ".mpc.R_aN", mpc_params.R_aN);
  
  declare_parameter_if_not_declared(node, name + ".mpc.vx_min", rclcpp::ParameterValue(-0.25));
  node->get_parameter(name + ".mpc.vx_min", mpc_params.v_min(0));
  declare_parameter_if_not_declared(node, name + ".mpc.vy_min", rclcpp::ParameterValue(-0.25));
  node->get_parameter(name + ".mpc.vy_min", mpc_params.v_min(1));
  declare_parameter_if_not_declared(node, name + ".mpc.vz_min", rclcpp::ParameterValue(-0.25));
  node->get_parameter(name + ".mpc.vz_min", mpc_params.v_min(2));
  declare_parameter_if_not_declared(node, name + ".mpc.vx_max", rclcpp::ParameterValue(0.25));
  node->get_parameter(name + ".mpc.vx_max", mpc_params.v_max(0));
  declare_parameter_if_not_declared(node, name + ".mpc.vy_max", rclcpp::ParameterValue(0.25));
  node->get_parameter(name + ".mpc.vy_max", mpc_params.v_max(1));
  declare_parameter_if_not_declared(node, name + ".mpc.vz_max", rclcpp::ParameterValue(0.25));
  node->get_parameter(name + ".mpc.vz_max", mpc_params.v_max(2));

  declare_parameter_if_not_declared(node, name + ".mpc.ax_min", rclcpp::ParameterValue(-30.0));
  node->get_parameter(name + ".mpc.ax_min", mpc_params.a_min(0));
  declare_parameter_if_not_declared(node, name + ".mpc.ay_min", rclcpp::ParameterValue(-30.0));
  node->get_parameter(name + ".mpc.ay_min", mpc_params.a_min(1));
  declare_parameter_if_not_declared(node, name + ".mpc.az_min", rclcpp::ParameterValue(-30.0));
  node->get_parameter(name + ".mpc.az_min", mpc_params.a_min(2));
  declare_parameter_if_not_declared(node, name + ".mpc.ax_max", rclcpp::ParameterValue(30.0));
  node->get_parameter(name + ".mpc.ax_max", mpc_params.a_max(0));
  declare_parameter_if_not_declared(node, name + ".mpc.ay_max", rclcpp::ParameterValue(30.0));
  node->get_parameter(name + ".mpc.ay_max", mpc_params.a_max(1));
  declare_parameter_if_not_declared(node, name + ".mpc.az_max", rclcpp::ParameterValue(30.0));
  node->get_parameter(name + ".mpc.az_max", mpc_params.a_max(2));

  declare_parameter_if_not_declared(node, name + ".mpc.ux_min", rclcpp::ParameterValue(-60.0));
  node->get_parameter(name + ".mpc.ux_min", mpc_params.u_min(0));
  declare_parameter_if_not_declared(node, name + ".mpc.uy_min", rclcpp::ParameterValue(-60.0));
  node->get_parameter(name + ".mpc.uy_min", mpc_params.u_min(1));
  declare_parameter_if_not_declared(node, name + ".mpc.uz_min", rclcpp::ParameterValue(-60.0));
  node->get_parameter(name + ".mpc.uz_min", mpc_params.u_min(2));
  declare_parameter_if_not_declared(node, name + ".mpc.ux_max", rclcpp::ParameterValue(60.0));
  node->get_parameter(name + ".mpc.ux_max", mpc_params.u_max(0));
  declare_parameter_if_not_declared(node, name + ".mpc.uy_max", rclcpp::ParameterValue(60.0));
  node->get_parameter(name + ".mpc.uy_max", mpc_params.u_max(1));
  declare_parameter_if_not_declared(node, name + ".mpc.uz_max", rclcpp::ParameterValue(60.0));
  node->get_parameter(name + ".mpc.uz_max", mpc_params.u_max(2));

  mpc_params.Drag.setZero();

  // Initialize safe flight corridor
  sfc_gen_ = std::make_unique<sfc::PolytopeSFC>(sfc_params);

  // Initialize MPC controller
  mpc_controller_ = std::make_unique<pvaj_mpc::MPCController>(mpc_params);

  // TODO: Initialize timer to publish MPC command
}

void LinearMPCController::cleanup()
{
  RCLCPP_INFO(
    logger_,
    "Cleaning up controller: %s of type"
    " linear_mpc_controller::LinearMPCController",
    plugin_name_.c_str());
  
  global_path_pub_.reset();
  sfc_pub_.reset();

  sfc_gen_.reset();
  mpc_controller_.reset();
}

void LinearMPCController::activate()
{
  RCLCPP_INFO(
    logger_,
    "Activating controller: %s of type "
    "linear_mpc_controller::LinearMPCController",
    plugin_name_.c_str());
  
  global_path_pub_->on_activate();
  sfc_pub_->on_activate();

  // Add callback for dynamic parameters
  auto node = node_.lock();

  // dyn_params_handler_ = node->add_on_set_parameters_callback(
  //   std::bind(
  //     &LinearMPCController::dynamicParametersCallback,
  //     this, std::placeholders::_1));

  RCLCPP_INFO(
    logger_,
    "Activated controller: %s of type "
    "linear_mpc_controller::LinearMPCController",
    plugin_name_.c_str());
}

void LinearMPCController::deactivate()
{
  RCLCPP_INFO(
    logger_,
    "Deactivating controller: %s of type "
    "linear_mpc_controller::LinearMPCController",
    plugin_name_.c_str());

  global_path_pub_->on_deactivate();
  sfc_pub_->on_deactivate();

  dyn_params_handler_.reset();
}


px4_msgs::msg::TrajectorySetpoint LinearMPCController::computeCommands(
  const Eigen::Vector3d & pose,
  const Eigen::Quaterniond & orientation,
  const Eigen::Vector3d &,
  gestelt_core::GoalChecker *)
{
  std::lock_guard<std::mutex> lock_reinit(mutex_);

  // Transform path to map frame
  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = occ_map_->getGlobalFrameID();
  pose_stamped.header.stamp = clock_->now();
  pose_stamped.pose.position.x = pose(0);
  pose_stamped.pose.position.y = pose(1);
  pose_stamped.pose.position.z = pose(2);
  pose_stamped.pose.orientation.x = orientation.x();
  pose_stamped.pose.orientation.y = orientation.y();
  pose_stamped.pose.orientation.z = orientation.z();
  pose_stamped.pose.orientation.w = orientation.w();
  auto plan_map = transformPlanFromGlobalToMap(pose_stamped);
  
  /**
   * Generate safe flight corridor
   */

  // Sample the global path
  std::vector<int> ref_plan_sfc_idx;
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> ref_plan_sfc; // [MAP FRAME] global plan used by safe flight corridor 
  for (size_t i = 0; i < plan_map.poses.size()-1; i += sfc_gen_->getPlanSampleInterval()){
    ref_plan_sfc.push_back(Eigen::Vector3d(
      plan_map.poses[0].pose.position.x, 
      plan_map.poses[0].pose.position.y, 
      plan_map.poses[0].pose.position.z));

      ref_plan_sfc_idx.push_back(i);
  } 
  // Add end of plan
  ref_plan_sfc.push_back(Eigen::Vector3d(
    plan_map.poses.back().pose.position.x, 
    plan_map.poses.back().pose.position.y, 
    plan_map.poses.back().pose.position.z));
  ref_plan_sfc_idx.push_back(plan_map.poses.size()-1);

  if (!sfc_gen_->generateSFC(occ_map_->getLocalPtsInMapFrame(), ref_plan_sfc)){
    throw gestelt_core::ControllerException("Failed to generate Safe Flight Corridor");
  }

  std::vector<Polyhedron3D, Eigen::aligned_allocator<Polyhedron3D>> sfc_polyhedrons 
    = sfc_gen_->getPolyVec();

  sfc_pub_->publish(sfc_gen_->toSFCMsg(occ_map_->getMapFrameID()));

  RCLCPP_INFO(logger_, "[SFC] Number of polyhedrones: %ld", 
    sfc_polyhedrons.size());

  /**
   * Generate MPC controls
   */

  auto polyhedronToPlanes = [&](const Polyhedron3D& poly, 
      Eigen::MatrixX4d& planes) 
  {
    int num_planes = (int) poly.vs_.size();
    planes.resize(num_planes, 4);

    for (int i = 0; i < num_planes; ++i) // for each plane
    {
      Eigen::Vector3d normal = poly.vs_[i].n_; // normal points outward (a,b,c) as in ax+by+cz = d
      Eigen::Vector3d pt = poly.vs_[i].p_; // Point on plane

      double d = normal.dot(pt);   // Scalar d obtained from normal DOT PRODUCT point 

      // Final plane needs to have normal pointing outwards
      // ax + by + cy + d = 0
      planes.row(i) << normal(0), normal(1), normal(2), -d;
    }
  };

  // Sample the global path
  std::vector<int> ref_plan_mpc_idx;
  std::vector<Eigen::Vector3d> ref_plan_mpc; // [MAP FRAME] global plan used by safe flight corridor 
  for (size_t i = 0; i < plan_map.poses.size()-1; i += mpc_controller_->getPlanSampleInterval()){
    ref_plan_mpc.push_back(Eigen::Vector3d(
      plan_map.poses[0].pose.position.x,
      plan_map.poses[0].pose.position.y,
      plan_map.poses[0].pose.position.z));
    ref_plan_mpc_idx.push_back(i);
  } 
  // Add end of plan
  ref_plan_mpc.push_back(Eigen::Vector3d(
    plan_map.poses.back().pose.position.x, 
    plan_map.poses.back().pose.position.y, 
    plan_map.poses.back().pose.position.z));
  ref_plan_mpc_idx.push_back(plan_map.poses.size()-1);
  
  for (int i = 0; i < mpc_controller_->MPC_HORIZON; i++) // for each MPC reference point
  {
    // reference path index
    int ref_idx = i >= (int)ref_plan_mpc.size() ? ref_plan_mpc.size() -1 : i;



  }

  // populate and return message
  px4_msgs::msg::TrajectorySetpoint traj_sp;

  return traj_sp;
}

void LinearMPCController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
}

nav_msgs::msg::Path LinearMPCController::transformPlanFromGlobalToMap(
  const geometry_msgs::msg::PoseStamped & pose)
{
  if (global_plan_.poses.empty()) {
    throw gestelt_core::ControllerException("Received plan with zero length");
  }

  // let's get the pose of the robot in the frame of the plan
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!occ_map_->transformPoseToTargetFrame(global_plan_.header.frame_id, pose, robot_pose)) {
    throw gestelt_core::ControllerException("Unable to transform robot pose into global plan's frame");
  }

  // We'll discard points on the plan that are outside the local costmap
  double max_occ_map_extent = occ_map_->getLocalMapMaxExtent()/2.0;

  auto closest_pose_upper_bound = first_after_integrated_distance(
    global_plan_.poses.begin(), global_plan_.poses.end(), max_robot_pose_search_dist_);

  // First find the closest pose on the path to the robot
  // bounded by when the path turns around (if it does) so we don't get a pose from a later
  // portion of the path
  auto transformation_begin =
    min_by(
    global_plan_.poses.begin(), closest_pose_upper_bound,
    [&](const geometry_msgs::msg::PoseStamped & ps) {
      return euclidean_distance(robot_pose, ps);
    });

  // Find points up to max_transform_dist so we only transform them.
  auto transformation_end = std::find_if(
    transformation_begin, global_plan_.poses.end(),
    [&](const auto & pose) {
      return euclidean_distance(pose, robot_pose) > max_occ_map_extent;
    });

  // Lambda to transform a PoseStamped from global frame to local
  auto transformGlobalPoseToLocal = [&](const auto & global_plan_pose) {
      geometry_msgs::msg::PoseStamped stamped_pose, transformed_pose;
      stamped_pose.header.frame_id = global_plan_.header.frame_id;
      stamped_pose.header.stamp = robot_pose.header.stamp;
      stamped_pose.pose = global_plan_pose.pose;

      occ_map_->transformPoseToTargetFrame(
        occ_map_->getMapFrameID(), stamped_pose, transformed_pose);

      return transformed_pose;
    };

  // Transform the near part of the global plan into the robot's frame of reference.
  nav_msgs::msg::Path transformed_plan;
  std::transform(
    transformation_begin, transformation_end,
    std::back_inserter(transformed_plan.poses),
    transformGlobalPoseToLocal);
  transformed_plan.header.frame_id = occ_map_->getMapFrameID();
  transformed_plan.header.stamp = robot_pose.header.stamp;

  // Remove the portion of the global plan that we've already passed so we don't
  // process it on the next iteration (this is called path pruning)
  global_plan_.poses.erase(begin(global_plan_.poses), transformation_begin);
  global_path_pub_->publish(transformed_plan);

  if (transformed_plan.poses.empty()) {
    throw gestelt_core::ControllerException("Resulting transformed plan has 0 poses in it.");
  }

  return transformed_plan;
}

// rcl_interfaces::msg::SetParametersResult
// LinearMPCController::dynamicParametersCallback(
//   std::vector<rclcpp::Parameter> parameters)
// {
//   rcl_interfaces::msg::SetParametersResult result;
//   std::lock_guard<std::mutex> lock_reinit(mutex_);

//   result.successful = true;
//   return result;
// }

}  // namespace linear_mpc_controller

// Register this controller as a gestelt_core plugin
PLUGINLIB_EXPORT_CLASS(
  linear_mpc_controller::LinearMPCController,
  gestelt_core::Controller)
