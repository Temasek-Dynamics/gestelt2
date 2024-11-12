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

#ifndef _VORONOI_PLANNER_HPP_
#define _VORONOI_PLANNER_HPP_

#include <Eigen/Eigen>

#include <limits>
#include <queue>

#include <nlohmann/json.hpp>

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <gestelt_interfaces/msg/plan_request.hpp>
#include <gestelt_interfaces/msg/space_time_path.hpp>
#include <gestelt_interfaces/msg/goals.hpp>

#include <minco_interfaces/msg/polynomial_trajectory.hpp>
#include <minco_interfaces/msg/minco_trajectory.hpp>

#include <px4_msgs/msg/trajectory_setpoint.hpp>

#include <voxel_map/voxel_map.hpp> 

#include <dynamic_voronoi/dynamic_voronoi.hpp>
#include <space_time_astar/space_time_astar.hpp>

#include <minco_traj_gen/minco_traj_gen.hpp>

#include <logger_wrapper/logger_wrapper.hpp>
#include <logger_wrapper/timer.hpp>

#include <viz_helper/viz_helper.hpp>

using json = nlohmann::json;

namespace navigator
{

class Waypoint
{
// Waypoint class is a LIFO queue 

public:
  // Default constructor
  Waypoint(){
  }

  // Reset
  void reset(){
    wp_queue.clear();
  }
  
  /**
   * @brief Add multiple waypoints
   * 
   * @param wp 
   * @return true 
   * @return false 
   */
  bool addMultipleWP(const std::vector<Eigen::Vector3d>& wp_vec){
    // Reverse the waypoints and add on top of the stack
    std::vector<Eigen::Vector3d> wp_vec_reversed = wp_vec;
    std::reverse(wp_vec_reversed.begin(), wp_vec_reversed.end());

    for (auto wp : wp_vec_reversed){
      wp_queue.push_back(wp);
    }

    return true;
  }

  /**
   * @brief Add a single waypoint
   * 
   * @param wp 
   * @return true 
   * @return false 
   */
  bool addWP(const Eigen::Vector3d& wp){
    wp_queue.push_back(wp);
    return true;
  }

  /* Getter methods */

  /**
   * @brief Get the next waypoint
   * 
   * @return Eigen::Vector3d 
   */
  const Eigen::Vector3d& nextWP(){
    return wp_queue.back();
  }

  /**
   * @brief Get all waypoints as a vector
   * 
   * @return const Eigen::Vector3d& 
   */
  const std::vector<Eigen::Vector3d>& getQueue(){
    return wp_queue;
  }

  /**
   * @brief Get the size of the queue
   * 
   * @return Eigen::Vector3d 
   */
  size_t size() const {
    return wp_queue.size();
  }

  /**
   * @brief Get the size of the queue
   * 
   * @return Eigen::Vector3d 
   */
  bool empty() const {
    return wp_queue.empty();
  }

  /* Setter methods */

  /**
   * @brief Pop the last waypoint
   * 
   */
  void popWP() {
    if (wp_queue.empty()){
        return;
    }
    else {
      wp_queue.pop_back();
    }
  }

private:
  std::vector<Eigen::Vector3d> wp_queue;
};

class VoronoiPlanner : public rclcpp::Node
{
public:

  VoronoiPlanner();

  virtual ~VoronoiPlanner();

  /* Initialize planner and map */
  void init();

  void initParams();

  void initPubSubTimer();

  /* Core methods */

  /**
   * @brief Plan a path from start to goal
   * 
   * @param goal 
   * @return true planning succeeded
   * @return false planning failed
   */
  bool plan(const Eigen::Vector3d& goal);

  /**
   * @brief Plan a path from start to goal
   * 
   * @param goal 
   * @return true planning succeeded
   * @return false planning failed
   */
  bool planWithoutComms(const Eigen::Vector3d& goal);

  /**
   * @brief Plan a path from start to goal
   * 
   * @param goal 
   * @return true planning succeeded
   * @return false planning failed
   */
  bool planWithComms(const Eigen::Vector3d& goal);

private:
  /* Timer callbacks */

  /* Timer for front-end planner*/
  void planFETimerCB();

  /* Generate voronoi map timer callback*/
  void genVoroMapTimerCB();

  /* Subscription callbacks */

  /* Front end plan subscription */
  void FEPlanSubCB(const gestelt_interfaces::msg::SpaceTimePath::UniquePtr msg);

  /* Subscription callback to goals */
  void goalsSubCB(const gestelt_interfaces::msg::Goals::UniquePtr msg);

  /* Subscription callback to point goal */
  void pointGoalSubCB(const geometry_msgs::msg::PoseStamped::UniquePtr msg);

  /* Plan request (for debug use)*/
  void planReqDbgSubCB(const gestelt_interfaces::msg::PlanRequest::UniquePtr msg);

  /* Subscription callback to odometry */
  void odomSubCB(const nav_msgs::msg::Odometry::UniquePtr& msg);

  /* Subscription callback to swarm odometry */
  void swarmOdomCB(const nav_msgs::msg::Odometry::UniquePtr& msg, int drone_id);

/* Helper methods */
private:

  pubPVAJCmd(	const Eigen::Vector3d& pos, 
              const Eigen::Vector2d& yaw_yawrate,
              const Eigen::Vector3d& vel,
              const Eigen::Vector3d& acc,
              const Eigen::Vector3d& jerk);

  /* Convert point from world to fixed map origin*/
  Eigen::Vector3d worldToMap(const Eigen::Vector3d& pt)
  {
    return Eigen::Vector3d(
      pt(0) + map_origin_(0), 
      pt(1) + map_origin_(1), 
      pt(2));
  }

  /* Convert point from fixed map origin to world*/
  Eigen::Vector3d mapToWorld(const Eigen::Vector3d& pt)
  {
    return Eigen::Vector3d(
      pt(0) - map_origin_(0), 
      pt(1) - map_origin_(1), 
      pt(2));
  }

  /* Convert point from fixed map origin to local map origin*/
  Eigen::Vector3d mapToLclMap(const Eigen::Vector3d& pt)
  {
    return Eigen::Vector3d(
      pt(0) - bool_map_3d_.origin(0), 
      pt(1) - bool_map_3d_.origin(1), 
      pt(2));
  }

  /* Convert point from local map origin to fixed map origin*/
  Eigen::Vector3d lclMapToMap(const Eigen::Vector3d& pt)
  {
    return Eigen::Vector3d(
      pt(0) + bool_map_3d_.origin(0), 
      pt(1) + bool_map_3d_.origin(1), 
      pt(2));
  }

  /**
   * @brief Sample position, velocity and acceleration at 
   *  a given time on the trajectory
   * 
   * @param traj 
   * @param time_samp 
   * @param pos 
   * @param vel 
   * @param acc 
   * @return true 
   * @return false 
   */
  bool sampleTrajectory(
    const std::shared_ptr<minco::Trajectory>& traj,
    const double& time_samp, 
    Eigen::Vector3d& pos, Eigen::Vector3d& vel, Eigen::Vector3d& acc)
  {
    if (traj == nullptr){
      return false;
    }
    if (traj->getGlobalStartTime() < 0.0){ 
      // Planning has not started
      return false;
    }

    double e_t_start = time_samp - traj->getGlobalStartTime(); // Get time t relative to start of trajectory

    if (e_t_start < 0.0 || e_t_start > traj->getTotalDuration())
    {
      return false;
    }

    pos = traj->getPos(e_t_start);
    vel = traj->getVel(e_t_start);
    acc = traj->getAcc(e_t_start);

    return true;
  }

  /**
   * @brief Get receding horizon planning (RHP) goal from desired goal
   * The RHP goal is the intersection of the P1-P2 line segment with the local map bounds (Cuboid).
   * P1 (current position) is the defined at the centroid of the local map bound 
   * 
   * @param P1 Current position
   * @param P2 Desired goal point
   * @param local_map_min local map minimum bound (corner)
   * @param local_map_max local map maximum bound (corner)
   * @param offset Keep goal within local map bounds by an offset along P1-P2
   * @return Eigen::Vector3d RHP goal
   */
  Eigen::Vector3d getRHPGoal(
    const Eigen::Vector3d& P1, const Eigen::Vector3d& P2, 
    const Eigen::Vector3d& local_map_min, const Eigen::Vector3d& local_map_max);


  // Convert from map to occupancy grid type
  void voronoimapToOccGrid( const dynamic_voronoi::DynamicVoronoi& dyn_voro, 
                            const double& origin_x, const double& origin_y, 
                            nav_msgs::msg::OccupancyGrid& occ_grid);

  // Convert from map to occupancy grid type
  void occmapToOccGrid(const dynamic_voronoi::DynamicVoronoi& dyn_voro, 
                      const double& origin_x, const double& origin_y,
                      nav_msgs::msg::OccupancyGrid& occ_grid);

  // Convert from polynomial trajectory to minco msg
  void polyTrajToMincoMsg(const std::shared_ptr<minco::Trajectory>& traj, 
                const double& traj_start_time,
                minco_interfaces::msg::PolynomialTrajectory &poly_msg, 
                minco_interfaces::msg::MincoTrajectory &MINCO_msg);

  /* Checks */

  /**
   * @brief Check if current position is within goal tolerance
   * 
   * @return true Goal is within tolerance. Goal execution is complete
   * @return false GOal is not within tolerance
   */
  bool isGoalReached(const Eigen::Vector3d& pos, const Eigen::Vector3d& goal);

  /* Helper methods */

private:
  /* MPC Params */
  int ref_samp_intv_{6}; // FE reference path sampling interval
  double path_dis_{0.05};

  /* Params */
  int drone_id_{-1};

  int num_drones_{0};  // If true, enable communicationless planning
  bool commless_{false};  // If true, enable communicationless planning
  bool use_lin_mpc_{false};  // If true, use linear MPC instead of MINCO trajectory generation

  std::string map_frame_;  // Fixed Frame of UAV's origin
  std::string local_map_frame_;  // Frame ID of UAV's local map

  // Planning params
  double t_unit_{0.1}; // [s] Time duration of each space-time A* unit
  double fe_planner_freq_{10}; // [Hz] Frequency for front-end planning
  double gen_voro_map_freq_{20}; // [Hz] Frequency for front-end planning
  double sqr_goal_tol_{0.1}; // [m] Distance to goal before it is considered fulfilled.
  bool plan_once_{false}; // Used for testing, only runs the planner once
  bool verbose_print_{false};  // enables printing of planning time
  std::string output_json_filepath_;  // Output filepath for JSON file

  bool json_output_{false};  // output path to json file

  int fe_stride_;     // stride to sample front-end path for minimum jerk trajectory generation

  // For use when populating reservation table 
  double rsvn_tbl_inflation_{-1.0}; // [m] Inflation of cells in the reservation table
  int rsvn_tbl_window_size_{-1}; // [s] Time buffer in the reservation table
  int t_buffer_{-1}; // [space-time units] Time buffer
  int cells_inf_{-1}; // [voxels] Spatial buffer

  global_planner::AStarParams astar_params_;  // a star planner parameters
  global_planner::VoronoiParams voro_params_; // voronoi map parameters

private:
	/* Callback groups */
	rclcpp::CallbackGroup::SharedPtr planning_cb_group_;
	rclcpp::CallbackGroup::SharedPtr swarm_plan_cb_group_;
	rclcpp::CallbackGroup::SharedPtr mapping_cb_group_;
	rclcpp::CallbackGroup::SharedPtr others_cb_group_;

  /**
   * Periodically running timers
   */
	rclcpp::TimerBase::SharedPtr plan_fe_timer_;	    // Timer for planning front end path
	rclcpp::TimerBase::SharedPtr gen_voro_map_timer_; // Timer for generating discretized voronoi diagram

  /**
   * ROS Publishers
   */
  // rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occ_map_pub_;        // Publishes original occupancy grid
  // rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr voro_occ_grid_pub_;  // Publishes voronoi map occupancy grid

  std::unordered_map<int, rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr> occ_map_pubs_;        // Publishes original occupancy grid
  std::unordered_map<int, rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr> voro_occ_grid_pubs_;  // Publishes voronoi map occupancy grid

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr voro_planning_pub_;  // Publishes voronoi map as modified for planning

  // Planning publishers
  rclcpp::Publisher<gestelt_interfaces::msg::SpaceTimePath>::SharedPtr fe_plan_broadcast_pub_; // Publish front-end plans broadcasted to other agents

  rclcpp::Publisher<minco_interfaces::msg::PolynomialTrajectory>::SharedPtr poly_traj_pub_; // Publish polynomial trajectories for execution

  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr lin_mpc_cmd_pub_; // Publish MPC commands
  rclcpp::Publisher<minco_interfaces::msg::MincoTrajectory>::SharedPtr minco_traj_broadcast_pub_; // Publish MINCO trajectories broadcasted to other agents

  // Visualization
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr plan_req_pub_; // start and goal visualization publisher
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr agent_id_text_pub_; // Agent ID text publisher
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr fe_closed_list_viz_pub_; // Closed list publishers
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr fe_plan_viz_pub_; // Publish front-end plan visualization
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr voronoi_graph_pub_; // publisher of voronoi graph vertices

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr minco_traj_viz_pub_; // Visualize minco trajectory

  /**
   * ROS Subscribers
   */
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;               // Subscriber to odometry
	rclcpp::Subscription<gestelt_interfaces::msg::Goals>::SharedPtr goals_sub_;              // Goal subscriber
	std::vector<rclcpp::Subscription<gestelt_interfaces::msg::SpaceTimePath>::SharedPtr>
    fe_plan_broadcast_subs_;  // Subscription to broadcasted front end plan from other agents
  
	std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> 
    swarm_odom_subs_;  // Subscription to odometry from other agents

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr point_goal_sub_;   // Point Goal subscriber

	rclcpp::Subscription<gestelt_interfaces::msg::PlanRequest>::SharedPtr plan_req_dbg_sub_; // (DEBUG USE) plan request (start and goal) debug subscriber

  /* TF2 */
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

private:
  /* Stored Odometry data */
  Eigen::Vector3d cur_pos_{0.0, 0.0, 0.0}, cur_vel_{0.0, 0.0, 0.0}, cur_acc_{0.0, 0.0, 0.0};   // [MAP FRAME] current state
  Eigen::Vector3d map_origin_{0.0, 0.0, 0.0}; // Fixed Map origin relative to world

  /* Mutexes*/
  std::mutex rsvn_tbl_mtx_;
  std::mutex voro_map_mtx_;
  std::mutex cur_state_mtx_;

  /* Flags*/
  bool init_voro_maps_{false}; // flag to indicate if voronoi map is initialized
  bool plan_complete_{false}; // flag to indicate a plan has been completed

  /* Mapping */
  std::unique_ptr<voxel_map::VoxelMap> voxel_map_;  // Occupancy map object
  voxel_map::BoolMap3D bool_map_3d_; // Bool map slices 
  std::map<int, std::shared_ptr<dynamic_voronoi::DynamicVoronoi>> dyn_voro_arr_; // array of voronoi objects with key of height (cm)

  /* Planner  */
  std::unique_ptr<global_planner::SpaceTimeAStar> fe_planner_; // Front end planner
  std::map<int, RsvnTbl> rsvn_tbl_; // map{drone_id : unordered_set{(x,y,z,t)}}. Reservation table of (x,y,z_cm, t) where x,y are grid positions, z_cm is height in centimeters and t is space time units

  Waypoint waypoints_; // Goal waypoint handler object

  std::vector<Eigen::Vector3d> fe_path_; // [MAP FRAME] Front-end Space path in space coordinates (x,y,z) 
  std::vector<Eigen::Vector4d> fe_path_with_t_; // [MAP FRAME]  Front-end Space time  path in space-time coordinates (x,y,z,t)

  std::unique_ptr<minco::MinJerkOpt> min_jerk_opt_; // Initial minimum jerk trajectory
  std::shared_ptr<minco::Trajectory> poly_traj_{nullptr}; // Front-end MINCO Trajectory

  /* Timers to measure computation time */
  logger_wrapper::Timer tm_front_end_plan_{"front_end_plan"}; // Front end plan generation
  logger_wrapper::Timer tm_voro_map_init_{"voro_map_init"}; // Voronoi map discretization

  /* Visualization */
  std::unique_ptr<viz_helper::VizHelper> viz_helper_; // Class to aid visualization

  /* Logging */
	std::shared_ptr<logger_wrapper::LoggerWrapper> logger_; // Class for logging

}; // class VoronoiPlanner

inline bool VoronoiPlanner::isGoalReached(const Eigen::Vector3d& pos, const Eigen::Vector3d& goal)
{
  return (pos - goal).squaredNorm() < sqr_goal_tol_;
}

inline void VoronoiPlanner::voronoimapToOccGrid( const dynamic_voronoi::DynamicVoronoi& dyn_voro, 
                                                const double& origin_x, const double& origin_y, 
                                                nav_msgs::msg::OccupancyGrid& occ_grid)
{
  occ_grid.header.stamp = this->get_clock()->now();
  occ_grid.header.frame_id = map_frame_;
  occ_grid.info.width = dyn_voro.getSizeX();
  occ_grid.info.height = dyn_voro.getSizeY();
  occ_grid.info.resolution = bool_map_3d_.resolution;
  occ_grid.info.origin.position.x = origin_x;
  occ_grid.info.origin.position.y = origin_y;
  occ_grid.info.origin.position.z = dyn_voro.getOriginZ();
  tf2::Quaternion q;
  q.setRPY(0, 0, 0.0);
  occ_grid.info.origin.orientation.x = q.x();
  occ_grid.info.origin.orientation.y = q.y();
  occ_grid.info.origin.orientation.z = q.z();
  occ_grid.info.origin.orientation.w = q.w();

  occ_grid.data.resize(occ_grid.info.width * occ_grid.info.height);

  auto map2Dto1DIdx = [&](const int& width, const int& x, const int& y){
    return width * y + x;
  };

  // auto map1Dto2DIdx = [&](const int& idx, const int& width, int& x, int& y){
  //   y = idx / width;
  //   x = idx - (y * width);
  // };

  for(int j = 0; j < dyn_voro.getSizeY(); j++)
  {
    for (int i = 0; i < dyn_voro.getSizeX(); i++)
    {
      size_t idx = map2Dto1DIdx(occ_grid.info.width, i, j);
      occ_grid.data[idx] = dyn_voro.isVoronoi(i, j) ? 255: 0;
    }
  }
}

inline void VoronoiPlanner::occmapToOccGrid(const dynamic_voronoi::DynamicVoronoi& dyn_voro, 
                                            const double& origin_x, const double& origin_y,
                                            nav_msgs::msg::OccupancyGrid& occ_grid)
{
  occ_grid.header.stamp = this->get_clock()->now();
  occ_grid.header.frame_id = map_frame_;
  occ_grid.info.width = dyn_voro.getSizeX();
  occ_grid.info.height = dyn_voro.getSizeY();
  occ_grid.info.resolution = bool_map_3d_.resolution;
  occ_grid.info.origin.position.x = origin_x;
  occ_grid.info.origin.position.y = origin_y;
  occ_grid.info.origin.position.z = dyn_voro.getOriginZ();
  tf2::Quaternion q;
  q.setRPY(0, 0, 0.0);
  occ_grid.info.origin.orientation.x = q.x();
  occ_grid.info.origin.orientation.y = q.y();
  occ_grid.info.origin.orientation.z = q.z();
  occ_grid.info.origin.orientation.w = q.w();

  occ_grid.data.resize(occ_grid.info.width * occ_grid.info.height);

  auto map2Dto1DIdx = [&](const int& width, const int& x, const int& y){
    return width * y + x;
  };

  for(int j = 0; j < dyn_voro.getSizeY(); j++)
  {
    for (int i = 0; i < dyn_voro.getSizeX(); i++)
    {
      size_t idx = map2Dto1DIdx(occ_grid.info.width, i, j);

      // Flip the coordinates vertically (Occ grid uses top left as origin but original map data uses bottom left as origin)
      // size_t idx = map2Dto1DIdx(occ_grid.info.width, i, occ_grid.info.height - j - 1);
      occ_grid.data[idx] = dyn_voro.isOccupied(i, j) ? 255: 0;
    }
  }
}

inline void VoronoiPlanner::polyTrajToMincoMsg(const std::shared_ptr<minco::Trajectory>& traj, 
                              const double& traj_start_time,
                              minco_interfaces::msg::PolynomialTrajectory &poly_msg, 
                              minco_interfaces::msg::MincoTrajectory &MINCO_msg)
{

  Eigen::VectorXd durs = traj->getDurations();
  int piece_num = traj->getPieceSize();
  poly_msg.drone_id = drone_id_;
  // poly_msg.traj_id = 0;
  poly_msg.start_time = traj_start_time;
  poly_msg.order = 5; 
  poly_msg.duration.resize(piece_num);
  poly_msg.coef_x.resize(6 * piece_num);
  poly_msg.coef_y.resize(6 * piece_num);
  poly_msg.coef_z.resize(6 * piece_num);

  // For each segment
  for (int i = 0; i < piece_num; ++i)
  {
    // Assign timestamp
    poly_msg.duration[i] = durs(i);

    // Assign coefficient matrix values
    minco::CoefficientMat cMat = traj->getPiece(i).getCoeffMat();
    int i6 = i * 6;
    for (int j = 0; j < 6; j++)
    {
      poly_msg.coef_x[i6 + j] = cMat(0, j);
      poly_msg.coef_y[i6 + j] = cMat(1, j);
      poly_msg.coef_z[i6 + j] = cMat(2, j);
    }
  }

  MINCO_msg.drone_id = drone_id_;
  // MINCO_msg.traj_id = 0;
  MINCO_msg.start_time = traj_start_time;
  MINCO_msg.order = 5; 
  MINCO_msg.duration.resize(piece_num);

  Eigen::Vector3d vec; // Vector representing x,y,z values or their derivatives
  // Start Position
  vec = traj->getPos(0);
  MINCO_msg.start_p[0] = vec(0), MINCO_msg.start_p[1] = vec(1), MINCO_msg.start_p[2] = vec(2);
  // Start Velocity
  vec = traj->getVel(0);
  MINCO_msg.start_v[0] = vec(0), MINCO_msg.start_v[1] = vec(1), MINCO_msg.start_v[2] = vec(2);
  // Start Acceleration
  vec = traj->getAcc(0);
  MINCO_msg.start_a[0] = vec(0), MINCO_msg.start_a[1] = vec(1), MINCO_msg.start_a[2] = vec(2);
  // End position
  vec = traj->getPos(traj->getTotalDuration());
  MINCO_msg.end_p[0] = vec(0), MINCO_msg.end_p[1] = vec(1), MINCO_msg.end_p[2] = vec(2);
  // End velocity
  vec = traj->getVel(traj->getTotalDuration());
  MINCO_msg.end_v[0] = vec(0), MINCO_msg.end_v[1] = vec(1), MINCO_msg.end_v[2] = vec(2);
  // End Acceleration
  vec = traj->getAcc(traj->getTotalDuration());
  MINCO_msg.end_a[0] = vec(0), MINCO_msg.end_a[1] = vec(1), MINCO_msg.end_a[2] = vec(2);

  // Assign inner points
  MINCO_msg.inner_x.resize(piece_num - 1);
  MINCO_msg.inner_y.resize(piece_num - 1);
  MINCO_msg.inner_z.resize(piece_num - 1);
  Eigen::MatrixXd pos = traj->getPositions();
  for (int i = 0; i < piece_num - 1; i++)
  {
    MINCO_msg.inner_x[i] = pos(0, i + 1);
    MINCO_msg.inner_y[i] = pos(1, i + 1);
    MINCO_msg.inner_z[i] = pos(2, i + 1);
  }
  for (int i = 0; i < piece_num; i++){
    MINCO_msg.duration[i] = durs[i];
  }

}

// P1: Start
// P2: global goal
inline Eigen::Vector3d VoronoiPlanner::getRHPGoal(
  const Eigen::Vector3d& P1, const Eigen::Vector3d& P2, 
  const Eigen::Vector3d& local_map_min, const Eigen::Vector3d& local_map_max)
{
  // The subroutine below has been adapted from the RHP function in FASTER by MIT-ACL
  //    https://github.com/mit-acl/faster

  if ((P2(0) > local_map_min(0) && P2(0) < local_map_max(0) ) 
      && (P2(1) > local_map_min(1) && P2(1) < local_map_max(1) ) 
      && (P2(2) > local_map_min(2) && P2(2) < local_map_max(2)))
  {
    // Goal is inside local map bounds

    Eigen::Vector3d P3_gbl;

    // Iterate backwards along the P1-P2 line until an unoccupied point is found
    for (double t = 1.0; t >= 0.05 ; t -= 0.1){
      P3_gbl = P1 + (t)*(P2 - P1);
      Eigen::Vector3d P3 = mapToLclMap(P3_gbl);

      int P3_z = roundToMultInt((int) (P3(2) * 100), 
                                  voro_params_.z_sep_cm, 
                                  voro_params_.min_height_cm, 
                                  voro_params_.max_height_cm);
      // Get node index position
      IntPoint P3_2d; 
      bool in_lcl_map =  dyn_voro_arr_[P3_z]->posToIdx(DblPoint(P3(0), P3(1)), P3_2d); 
      if (!in_lcl_map){
        continue;
      }
      if (!dyn_voro_arr_[P3_z]->isOccupied(P3_2d)){
        break;
      }

      // if (!voxel_map_->isOccupied(P3)){
      //   break;
      // }
    }

    return P3_gbl;
  }

  /**
   * @brief Obtains intersection of line segment P1-P2 with given plane 
   * 
   * @param P1 Start point of line segment 
   * @param P2 End point of line segment
   * @param coeff Coefficients of the plane
   * @param intsc_pt Intersection point of line segment with the plane
   * @param offset Keep goal within local map bounds by an offset along P1-P2
   * @return true 
   * @return false 
   */
  auto getIntersectionWithPlane = [&](
    const Eigen::Vector3d& P1, const Eigen::Vector3d& P2, 
    const Eigen::Vector4d& c, Eigen::Vector3d& intsc_pt) -> bool
  {
    // t: parameterization of distance along vector P1-P2
    double t = -(c[3] + c[0] * P1[0] + c[1] * P1[1] + c[2]* P1[2] ) / 
                (c[0] * (P2[0] - P1[0]) + c[1] * (P2[1] - P1[1]) + c[2] * (P2[2] - P1[2]));

    intsc_pt = P1 + (t)*(P2 - P1);

    bool intsc_w_line_seg = (t < 0 || t > 1) ? false : true;

    if (intsc_w_line_seg){
      for (double t_ = t; t_ >= 0.05 ; t_ -= 0.1){
        intsc_pt = P1 + (t_)*(P2 - P1);
        if (!voxel_map_->isOccupied(intsc_pt)){
          break;
        }
      }
    }

    // if t < 0 || t > 1: The plane does not intersect with the line segment P1-P2, 
    //  but rather it intersects with the vector defined by it.  
    return intsc_w_line_seg;
  };

  std::vector<Eigen::Vector4d> all_planes = {
    Eigen::Vector4d(1, 0, 0, -local_map_max(0)),  // Plane X right
    Eigen::Vector4d(-1, 0, 0, local_map_min(0)),  // Plane X left
    Eigen::Vector4d(0, 1, 0, -local_map_max(1)),  // Plane Y right
    Eigen::Vector4d(0, -1, 0, local_map_min(1)),  // Plane Y left
    Eigen::Vector4d(0, 0, 1, -local_map_max(2)),  // Plane Z up
    Eigen::Vector4d(0, 0, -1, local_map_min(2))   // Plane Z down
  };

  std::vector<Eigen::Vector3d> intsc_pts;
  std::vector<double> distances;

  for (int i = 0; i < 6; i++) // for each plane of the local map bound
  {
    Eigen::Vector3d intsc_pt;
    if (getIntersectionWithPlane(P1, P2, all_planes[i], intsc_pt))
    {
      intsc_pts.push_back(intsc_pt);
      distances.push_back((intsc_pt - P1).norm());
    }
  }

  if (intsc_pts.size() == 0) // There is no intersection between P1-P2 line and bounding box
  {  
    logger_->logError("BUG: Unable to getRHPGoal(), no intersection of current_pose->goal line with local map bounds");
    rclcpp::shutdown();
  }
  
  // Return nearest intersection point 
  int minElementIndex = std::min_element(distances.begin(), distances.end()) - distances.begin();
  Eigen::Vector3d P4 = intsc_pts[minElementIndex];

  // Iterate backwards along the P1-P2 line until an unoccupied point is found
  Eigen::Vector3d P3_gbl;

  for (double t = 1.0; t >= 0.05 ; t -= 0.1){
    P3_gbl = P1 + (t)*(P4 - P1);
    Eigen::Vector3d P3 = mapToLclMap(P3_gbl);

    int P3_z = roundToMultInt((int) (P3(2) * 100), 
                                voro_params_.z_sep_cm, 
                                voro_params_.min_height_cm, 
                                voro_params_.max_height_cm);
    // Get node index position
    IntPoint P3_2d; 
    bool in_lcl_map = dyn_voro_arr_[P3_z]->posToIdx(DblPoint(P3(0), P3(1)), P3_2d); 
    if (!in_lcl_map){
      continue;
    }
    if (!dyn_voro_arr_[P3_z]->isOccupied(P3_2d)){
      break;
    }
  }

  return P3_gbl;
}

inline VoronoiPlanner::pubPVAJCmd(	const Eigen::Vector3d& pos, 
                                    const Eigen::Vector2d& yaw_yawrate,
                                    const Eigen::Vector3d& vel,
                                    const Eigen::Vector3d& acc,
                                    const Eigen::Vector3d& jerk)
{
  // Send msg in ENU frame

	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	msg.position = {(float) pos(0), (float) pos(1), (float) pos(2)};
	msg.velocity = {(float) vel(0), (float) vel(1), (float) vel(2)};
	msg.acceleration = {(float) acc(0), (float) acc(1), (float) acc(2)};
	msg.jerk = {(float) jerk(0), (float) jerk(1), (float) jerk(2)};

	msg.yaw = yaw_yawrate(0); // [-PI:PI]
	msg.yawspeed = yaw_yawrate(1); // angular velocity around NED frame z-axis in radians/second

	lin_mpc_cmd_pub_->publish(msg);
}

} // namespace navigator


#endif // _VORONOI_PLANNER_HPP_


