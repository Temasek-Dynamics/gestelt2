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

  sfc_pub_ = node->create_publisher<decomp_ros_msgs::msg::PolyhedronArray>(
    "sfc", rclcpp::SensorDataQoS());
  mpc_traj_pub_ = node->create_publisher<nav_msgs::msg::Path>(
    "mpc_traj", rclcpp::SensorDataQoS());
  global_path_pub_ = node->create_publisher<nav_msgs::msg::Path>(
    "received_global_plan", rclcpp::SensorDataQoS());

  mpc_ref_path_pub_ = node->create_publisher<nav_msgs::msg::Path>(
    "mpc_ref_path", rclcpp::SensorDataQoS());

  // Declare this plugin's parameters
  declare_parameter_if_not_declared(node, name + ".control_yaw", rclcpp::ParameterValue(true));
  node->get_parameter(name + ".control_yaw", control_yaw_);
  declare_parameter_if_not_declared(node, name + ".yaw_lookahead_dist", rclcpp::ParameterValue(5));
  node->get_parameter(name + ".yaw_lookahead_dist", yaw_lookahead_dist_);
  declare_parameter_if_not_declared(node, name + ".yawspeed_max", rclcpp::ParameterValue(1.57));
  node->get_parameter(name + ".yawspeed_max", yawspeed_max_);

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

  declare_parameter_if_not_declared(node, name + ".mpc.horizon", rclcpp::ParameterValue(15));
  node->get_parameter(name + ".mpc.horizon", mpc_params.MPC_HORIZON);
  declare_parameter_if_not_declared(node, name + ".mpc.time_step", rclcpp::ParameterValue(0.1));
  node->get_parameter(name + ".mpc.time_step", mpc_params.TIME_STEP);
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
}

void LinearMPCController::cleanup()
{
  RCLCPP_INFO(
    logger_,
    "Cleaning up controller: %s of type"
    " linear_mpc_controller::LinearMPCController",
    plugin_name_.c_str());
  
  mpc_ref_path_pub_.reset();
  global_path_pub_.reset();
  sfc_pub_.reset();
  mpc_traj_pub_.reset();

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
  
  mpc_ref_path_pub_->on_activate();
  global_path_pub_->on_activate();
  sfc_pub_->on_activate();
  mpc_traj_pub_->on_activate();

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

  mpc_ref_path_pub_->on_deactivate();
  global_path_pub_->on_deactivate();
  mpc_traj_pub_->on_deactivate();
  sfc_pub_->on_deactivate();

  dyn_params_handler_.reset();
}

void LinearMPCController::computeCommands(
  const Eigen::Vector3d & position,
  const Eigen::Quaterniond & orientation,
  const Eigen::Vector3d & velocity,
  gestelt_core::GoalChecker *,
  std::vector<Eigen::Vector3d>& mpc_pred_pos,
  std::vector<Eigen::Vector3d>& mpc_pred_vel,
  std::vector<Eigen::Vector3d>& mpc_pred_acc,
  std::vector<Eigen::Vector3d>& mpc_pred_u,
  Eigen::Vector2d& mpc_yaw)
{
  // Reset controller data from previous runs (if any)
  mpc_controller_->reset();

  // RCLCPP_INFO(logger_, "[MPC] computeCommands. Position (%0.2f, %0.2f, %0.2f) Velocity (%0.2f, %0.2f, %0.2f)", 
  //   position(0), position(1), position(2), velocity(0), velocity(1), velocity(2));

  std::lock_guard<std::mutex> lock_reinit(mutex_);

  // Transform path to map frame
  geometry_msgs::msg::PoseStamped pose_stamped; // Current pose in global frame
  pose_stamped.header.frame_id = occ_map_->getGlobalFrameID();
  pose_stamped.header.stamp = clock_->now();
  pose_stamped.pose.position.x = position(0);
  pose_stamped.pose.position.y = position(1);
  pose_stamped.pose.position.z = position(2);
  pose_stamped.pose.orientation.x = orientation.x();
  pose_stamped.pose.orientation.y = orientation.y();
  pose_stamped.pose.orientation.z = orientation.z();
  pose_stamped.pose.orientation.w = orientation.w();
  auto plan_map = transformPlanFromGlobalToMap(pose_stamped);
  
  /**
   * Generate safe flight corridor
   */

  // Sample the global path for SFC Generation

  // Find the idx position on the plan_map closest to current position
  // To get an mpc_ref_path wrt current position
  // To get SFC wrt current position
  auto nearest_gap_dist = std::numeric_limits<double>::max();
  int nearest_plan_idx = (int)plan_map.poses.size() - 1;
  if ((int)plan_map.poses.size() < 3){
    nearest_plan_idx = 0;
  }
  else{
    for (int i = (int)plan_map.poses.size() - 2; i >= 0; i--){ // cannot check from right at the end, as generating SFC needs a start and an end
      auto distGapVector = Eigen::Vector3d(
        plan_map.poses[i].pose.position.x - position(0),
        plan_map.poses[i].pose.position.y - position(1),
        plan_map.poses[i].pose.position.z - position(2)
      );
      auto distGap = distGapVector.squaredNorm();

      if (distGap < nearest_gap_dist){
        nearest_gap_dist = distGap;
        nearest_plan_idx = i;
      }
    }
    if (nearest_plan_idx < (int)plan_map.poses.size() - 2) {
      nearest_plan_idx += 1;
    }
  }

  // [MAP FRAME] Global plan used by safe flight corridor 
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> ref_plan_sfc; 
  // Sampled Indices of original reference plan
  std::vector<int> ref_plan_sfc_idx; 

  for (int i = nearest_plan_idx; i < (int)plan_map.poses.size()-1; i += sfc_gen_->getPlanSampleInterval()){
    ref_plan_sfc.push_back(Eigen::Vector3d(
      plan_map.poses[i].pose.position.x, 
      plan_map.poses[i].pose.position.y, 
      plan_map.poses[i].pose.position.z));

      ref_plan_sfc_idx.push_back(i - nearest_plan_idx);
  } 
  // Add end of plan
  ref_plan_sfc.push_back(Eigen::Vector3d(
    plan_map.poses.back().pose.position.x, 
    plan_map.poses.back().pose.position.y, 
    plan_map.poses.back().pose.position.z));
  ref_plan_sfc_idx.push_back(plan_map.poses.size()-1);

  if (!sfc_gen_->generateSFC(occ_map_->getLocalPtsInMapFrame(), ref_plan_sfc)){
    throw gestelt_core::NoValidControl("Failed to generate Safe Flight Corridor");
  }

  std::vector<Polyhedron3D, Eigen::aligned_allocator<Polyhedron3D>> sfc_polyhedrons 
    = sfc_gen_->getPolyVec();

  sfc_pub_->publish(sfc_gen_->toSFCMsg(occ_map_->getMapFrameID()));

  RCLCPP_INFO(logger_, "[SFC] Generated SFC with %ld polyhedrons", 
    sfc_polyhedrons.size());

  /**
   * Generate MPC controls
   */

  // Sample the global path for MPC reference path

  // [MAP FRAME] global plan used by safe flight corridor 
  std::vector<Eigen::Vector3d> ref_plan_mpc; 

  for (int i = nearest_plan_idx; i < (mpc_controller_->MPC_HORIZON + nearest_plan_idx)&& i < (int)plan_map.poses.size(); i++){
    ref_plan_mpc.push_back(Eigen::Vector3d(
      plan_map.poses[i].pose.position.x,
      plan_map.poses[i].pose.position.y,
      plan_map.poses[i].pose.position.z));
  }
  
  int poly_idx = 0; // current index of SFC polyhedron
  Eigen::Vector3d last_ref_pos = position; // last pos reference

  nav_msgs::msg::Path mpc_ref_path_msg;
  mpc_ref_path_msg.header.frame_id = occ_map_->getMapFrameID();
  mpc_ref_path_msg.header.stamp = clock_->now();

  auto polyhedronToPlanes = [&](const Polyhedron3D& polyhedron, 
    Eigen::MatrixX4d& planes) 
  {
    int num_planes = (int) polyhedron.vs_.size();
    planes.resize(num_planes, 4);

    for (int i = 0; i < num_planes; i++) // for each plane
    {
      Eigen::Vector3d normal = polyhedron.vs_[i].n_; // normal points outward (a,b,c) as in ax+by+cz = d
      Eigen::Vector3d pt = polyhedron.vs_[i].p_; // Point on plane

      double d = normal.dot(pt);   // Scalar d obtained from normal DOT PRODUCT point 

      // Final plane needs to have normal pointing outwards
      // ax + by + cy + d = 0
      planes.row(i) << normal(0), normal(1), normal(2), -d;
    }
  };

  for (int k = 0; k < mpc_controller_->MPC_HORIZON; k++) // for each k-th control iteration
  {
    // k: MPC control iteration
    // ref_idx: index for MPC reference path 
    // ref_pos: reference position from MPC reference path

    // Constrain ref_idx to never exceed the size of reference plan
    int ref_idx = k > (int)ref_plan_mpc.size() -1 ? (int) ref_plan_mpc.size() - 1 : k;
    auto ref_pos = ref_plan_mpc[ref_idx];

    // RCLCPP_INFO(logger_, "[MPC] ref_pos (%0.2f, %0.2f, %0.2f) at ref_idx %d", 
    //   ref_pos(0), ref_pos(1), ref_pos(2), ref_idx);

    // poly_idx: index for SFC Polyhedron
    // sfc_start_idx: End ref path index of current SFC segment
    // sfc_end_idx: End ref path index of current SFC segment

    // get index of current polygon that ref_pos belongs in
    // int sfc_start_idx = ref_plan_sfc_idx[poly_idx];
    int sfc_end_idx = ref_plan_sfc_idx[poly_idx + 1];

    // RCLCPP_INFO(logger_, "[MPC] SFC Polygon (%d) at with segment[%d] (%0.2f, %0.2f, %0.2f) -> [%d](%0.2f, %0.2f, %0.2f)", 
    //   poly_idx, 
    //   sfc_start_idx,
    //   ref_plan_sfc[poly_idx](0), ref_plan_sfc[poly_idx](1), ref_plan_sfc[poly_idx](2), 
    //   sfc_end_idx,
    //   ref_plan_sfc[poly_idx+1](0), ref_plan_sfc[poly_idx+1](1), ref_plan_sfc[poly_idx+1](2));

    if (ref_idx > sfc_end_idx) 
    {
      // RCLCPP_INFO(logger_, "[MPC] Incrementing from polygon %d to polygon %d", poly_idx, poly_idx+1);

      // if MPC ref index exceeds end of current SFC segment, then increment to next polygon
      poly_idx++;

      // Constrain poly_idx to never exceed the number of polygons
      if (poly_idx > (int)sfc_polyhedrons.size() - 1){
        poly_idx = (int)sfc_polyhedrons.size() - 1;
      }
      // RCLCPP_ERROR(logger_, 
      //   "[MPC] ERROR: Ref Path idx %d exceeds SFC Segment end %d", ref_idx, sfc_end_idx );
    }

    // At k-th control iteration, set PVA reference 
    Eigen::Vector3d ref_vel(0, 0, 0); // vel reference
    Eigen::Vector3d ref_acc(0, 0, 0); // acc reference

    if (k == 0) // First iteration
    {
      ref_vel = (ref_pos - position) / mpc_controller_->getTimeStep();
      if (ref_vel.x() > 0.0){
        ref_vel.x() = 3.33;
      } 
      else if (ref_vel.x() < 0.0){
        ref_vel.x() = -3.33;
      }
      else {
        ref_vel.x() = 0.0;
      }
      if (ref_vel.y() > 0.0){
        ref_vel.y() = 3.33;
      } 
      else if (ref_vel.y() < 0.0){
        ref_vel.y() = -3.33;
      }
      else {
        ref_vel.y() = 0.0;
      }
    }
    else // rest of the iteration
    {
      ref_vel = (ref_pos - last_ref_pos) / mpc_controller_->getTimeStep();
      // ref_vel = (ref_plan_mpc[ref_plan_mpc.size() - 1] - last_ref_pos) / mpc_controller_->getTimeStep();
      // if (ref_vel.x() >= 0){
      //   ref_vel.x() = 3.0;
      // } else {
      //   ref_vel.x() = -3.0;
      // }
      // if (ref_vel.y() >= 0){
      //   ref_vel.y() = 3.0;
      // } else {
      //   ref_vel.y() = -3.0;
      // }
    }
    // RCLCPP_INFO(logger_, "[computeCommands]Ref Vel is %0.2f, %0.2f, %0.2f", ref_vel.x(), ref_vel.y(), ref_vel.z());
    mpc_controller_->setReference(ref_pos, ref_vel, ref_acc, k);

    // At k-th control iteration, set SFC linear constraints
    Eigen::MatrixX4d sfc_planes;
    polyhedronToPlanes(sfc_polyhedrons[poly_idx], sfc_planes);
    if (!mpc_controller_->isInFSC(ref_pos, sfc_planes)) { 
      RCLCPP_ERROR(logger_, 
        "[MPC] ERROR: Ref Path idx %d is not in polygon %d", ref_idx, poly_idx );
      throw gestelt_core::NoValidControl("MPC Reference path not in SFC");
    }
    mpc_controller_->assignSFCToRefPt(sfc_planes, k); 

    // Assign pose values for visualizing reference path
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = ref_pos(0);
    pose.pose.position.y = ref_pos(1);
    pose.pose.position.z = ref_pos(2);
    mpc_ref_path_msg.poses.push_back(pose);

    last_ref_pos = ref_pos;
  }

  mpc_ref_path_pub_->publish(mpc_ref_path_msg);

  // Set initial condition
  mpc_controller_->setInitialCondition(position, velocity, Eigen::Vector3d::Zero());
  if (!mpc_controller_->run()){ // Successful MPC solve
    throw gestelt_core::NoValidControl("MPC solve failure!");
  }

  Eigen::MatrixXd A1, B1; // system transition matrix
  mpc_controller_->getSystemModel(A1, B1, mpc_controller_->getTimeStep());
  Eigen::VectorXd x_current = mpc_controller_->X_0_; // Current state
  Eigen::Vector3d u_optimal; 
  bool valid_cmd = true;

  mpc_pred_pos.clear();
  mpc_pred_vel.clear();
  mpc_pred_acc.clear();
  mpc_pred_u.clear();

  // Check if MPC command values are valid
  auto checkValidCmd = [&](const Eigen::Vector3d& vec, const int& min_val, const int& max_val){
    for (const auto& val : vec){
      if (val < min_val || val > max_val){
        return false;
      }
    }
    return true;
  };

  // Get predicted MPC path based on controls and check if valid
  for (int k = 0; k < mpc_controller_->MPC_HORIZON; k++) {
      mpc_controller_->getOptimalControl(u_optimal, k);
      x_current = A1 * x_current + B1 * u_optimal;

      // Check if position, velocity and acceleration at valid values
      if (!checkValidCmd(x_current.segment<3>(0), -100.0, 100.0) ){
        RCLCPP_ERROR(logger_, "Control iteration %d has invalid MPC pos: (%f, %f, %f)", 
          k, 
          x_current.segment<3>(0)(0), 
          x_current.segment<3>(0)(1), 
          x_current.segment<3>(0)(2));
          
        valid_cmd = false;
        break;
      }
      if (!checkValidCmd(x_current.segment<3>(3), -5.0, 5.0) ){
        RCLCPP_ERROR(logger_, "Control iteration %d has invalid MPC vel: (%f, %f, %f)", 
          k, 
          x_current.segment<3>(3)(0), 
          x_current.segment<3>(3)(1), 
          x_current.segment<3>(3)(2));

        valid_cmd = false;
        break;
      }
      if (!checkValidCmd(x_current.segment<3>(6), -60.0, 60.0) ){
        RCLCPP_ERROR(logger_, "Control iteration %d has invalid MPC acc: (%f, %f, %f)", 
          k, 
          x_current.segment<3>(6)(0), 
          x_current.segment<3>(6)(1), 
          x_current.segment<3>(6)(2));

        valid_cmd = false;
        break;
      }

      mpc_pred_pos.push_back(x_current.segment<3>(0));
      mpc_pred_vel.push_back(x_current.segment<3>(3));
      mpc_pred_acc.push_back(x_current.segment<3>(6));
      mpc_pred_u.push_back(u_optimal);
  }

  // // Interpolation method (KIV)
  // int n = 0.02 * 50; // set controller frequency at 50hz

  // Eigen::Vector3d delta_acce = (mpc_pred_acc[1] - mpc_pred_acc[0]) / (mpc_controller_->getTimeStep() / 0.02);
  // Eigen::Vector3d delta_velo = (mpc_pred_vel[1] - mpc_pred_vel[0]) / (mpc_controller_->getTimeStep() / 0.02);
  // Eigen::Vector3d delta_posi = (mpc_pred_pos[1] - mpc_pred_pos[0]) / (mpc_controller_->getTimeStep() / 0.02);

  // Eigen::Vector3d acce = mpc_pred_acc[0];
  // Eigen::Vector3d velo = mpc_pred_vel[0];
  // Eigen::Vector3d posi = mpc_pred_pos[0];
  // while (n < mpc_controller_->getTimeStep() * 50 && mpc_pred_pos.size()){

  //   // Interpolate from trajectory initial position
  //   double curr_timestep = (double)n / 50.0;
  //   // Eigen::Vector3d jerk = mpc_pred_u[n-1];
  //   acce += delta_acce;
  //   velo += delta_velo;
  //   posi += delta_posi;
  //   // insert into mpc path
  //   mpc_pred_pos.insert(mpc_pred_pos.begin() + n, posi);
  //   mpc_pred_vel.insert(mpc_pred_vel.begin() + n, velo);
  //   mpc_pred_acc.insert(mpc_pred_acc.begin() + n, acce);
  //   // mpc_pred_u.insert(mpc_pred_u.begin() + n, jerk);
  //   n++;
  // }

  // for (int i = 0; i < (int)mpc_pred_u.size(); i++){
  //   RCLCPP_INFO(logger_, "[MPC] Pred %d: u (%0.2f, %0.2f, %0.2f) a (%0.2f, %0.2f, %0.2f) v (%0.2f, %0.2f, %0.2f)", 
  //     i, 
  //     mpc_pred_u[i](0), mpc_pred_u[i](1), mpc_pred_u[i](2),
  //     mpc_pred_acc[i](0), mpc_pred_acc[i](1), mpc_pred_acc[i](2),
  //     mpc_pred_vel[i](0), mpc_pred_vel[i](1), mpc_pred_vel[i](2)
  //   );
  // }

  if (!valid_cmd){
    // If MPC command is not valid, then we will use the first point of the reference path
    // pos_corr = Eigen::Vector3d(
    //   mpc_ref_path_msg.poses[0].pose.position.x,
    //   mpc_ref_path_msg.poses[0].pose.position.y,
    //   mpc_ref_path_msg.poses[0].pose.position.z
    // );

    throw gestelt_core::NoValidControl("Invalid controller input from MPC solution");
  }

  // Publish MPC path for visualization
  nav_msgs::msg::Path mpc_traj_msg;

  mpc_traj_msg.header.frame_id = occ_map_->getMapFrameID();
  mpc_traj_msg.header.stamp = clock_->now();
  for (int i = 0; i < (int)mpc_pred_pos.size(); i++) {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = mpc_pred_pos[i](0);
      pose.pose.position.y = mpc_pred_pos[i](1);
      pose.pose.position.z = mpc_pred_pos[i](2);
      mpc_traj_msg.poses.push_back(pose);
  }
  mpc_traj_pub_->publish(mpc_traj_msg);

  static bool end_yaw_flag;

  if (control_yaw_){
    double cmd_yaw = 0.0;
    double dx = pose_stamped.pose.position.x - plan_map.poses.back().pose.position.x,
      dy = pose_stamped.pose.position.y - plan_map.poses.back().pose.position.y,
      dz = pose_stamped.pose.position.z - plan_map.poses.back().pose.position.z;

    // Calculate commanded yaw
    Eigen::Vector2d dir_vec(
      plan_map.poses.back().pose.position.y - position(1),
      plan_map.poses.back().pose.position.x - position(0)
    );
    cmd_yaw = std::atan2(dir_vec(0), dir_vec(1));

    // Yaw = 0 when drone has reached its endpoint
    if (dx * dx + dy * dy + dz * dz < 0.0625){
      if (!end_yaw_flag) {
        mpc_yaw(0) = cmd_yaw;
        mpc_yaw(1) = 0.0;
        end_yaw_flag = true;
      }

    }
    else{
      mpc_yaw(0) = cmd_yaw;
      mpc_yaw(1) = 0.0;
      end_yaw_flag = false;
    }
  }
}

void LinearMPCController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
}

nav_msgs::msg::Path LinearMPCController::transformPlanFromGlobalToMap(
  const geometry_msgs::msg::PoseStamped & pose)
{
  if (global_plan_.poses.empty()) {
    throw gestelt_core::InvalidPath("Received plan with zero length");
  }

  // let's get the pose of the robot in the frame of the plan
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!occ_map_->transformPoseToTargetFrame(global_plan_.header.frame_id, pose, robot_pose)) {
    throw gestelt_core::ControllerTFError("Unable to transform robot pose into global plan's frame");
  }

  // We'll discard points on the plan that are outside the local costmap
  // double max_occ_map_extent = occ_map_->getLocalMapMaxExtent()/2.0;

  // auto closest_pose_upper_bound = first_after_integrated_distance(
  //   global_plan_.poses.begin(), global_plan_.poses.end(), max_robot_pose_search_dist_);

  // First find the closest pose on the path to the robot
  // bounded by when the path turns around (if it does) so we don't get a pose from a later
  // portion of the path
  // auto transformation_begin =
  //   min_by(
  //   global_plan_.poses.begin(), closest_pose_upper_bound,
  //   [&](const geometry_msgs::msg::PoseStamped & ps) {
  //     return euclidean_distance(robot_pose, ps);
  //   });

  // // Find points up to max_transform_dist so we only transform them.
  // auto transformation_end = std::find_if(
  //   transformation_begin, global_plan_.poses.end(),
  //   [&](const auto & pose) {
  //     return euclidean_distance(pose, robot_pose) > max_occ_map_extent;
  //   });

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

  auto transformation_begin = global_plan_.poses.begin();
  auto transformation_end = global_plan_.poses.end();

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
  // global_plan_.poses.erase(begin(global_plan_.poses), transformation_begin);
  
  global_path_pub_->publish(transformed_plan);

  if (transformed_plan.poses.empty()) {
    throw gestelt_core::ControllerTFError("Resulting transformed plan has 0 poses in it.");
  }

  return transformed_plan;
}

}  // namespace linear_mpc_controller

// Register this controller as a gestelt_core plugin
PLUGINLIB_EXPORT_CLASS(
  linear_mpc_controller::LinearMPCController,
  gestelt_core::Controller)
