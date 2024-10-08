#ifndef _VORONOI_PLANNER_HPP_
#define _VORONOI_PLANNER_HPP_

#include <Eigen/Eigen>

#include <limits>
#include <queue>

#include <nlohmann/json.hpp>

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <gestelt_interfaces/msg/plan_request.hpp>
#include <gestelt_interfaces/msg/space_time_path.hpp>
#include <gestelt_interfaces/msg/goals.hpp>

#include <minco_interfaces/msg/polynomial_trajectory.hpp>
#include <minco_interfaces/msg/minco_trajectory.hpp>

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
   * @param start 
   * @param goal 
   * @return true 
   * @return false 
   */
  bool plan(const Eigen::Vector3d& start, const Eigen::Vector3d& goal);


private:
  /* Generate minimum jerk trajectory*/
  std::shared_ptr<minco::Trajectory> genMinJerkTraj(std::unique_ptr<minco::MinJerkOpt>& min_jerk_opt,
                                    const std::vector<Eigen::Vector4d>& space_time_path,
                                    const double& t_plan_start);

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

  /* Plan request (for debug use)*/
  void planReqDbgSubCB(const gestelt_interfaces::msg::PlanRequest::UniquePtr msg);

  /* Subscription callback to odometry */
  void odomSubCB(const nav_msgs::msg::Odometry::UniquePtr msg);

/* Helper methods */
private:

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
   * @return true 
   * @return false 
   */
  bool isGoalReached(const Eigen::Vector3d& pos, const Eigen::Vector3d& goal);

  /* Helper methods */

private:
  /* Params */
  int drone_id_{-1};

  std::string local_map_frame_;  // Frame ID of UAV's local map
  std::string global_frame_;     // Global origin of all UAVs

  // Planning params
  double t_unit_{0.1}; // [s] Time duration of each space-time A* unit
  double fe_planner_freq_{10}; // [Hz] Frequency for front-end planning
  double gen_voro_map_freq_{20}; // [Hz] Frequency for front-end planning
  double sqr_goal_tol_{0.1}; // [m] Distance to goal before it is considered fulfilled.
  bool plan_once_{false}; // Used for testing, only runs the planner once
  bool verbose_print_{false};  // enables printing of planning time
  bool planner_los_smooth_; // Enable line of sight smoothing for planner
  std::string output_json_filepath_;  // Output filepath for JSON file

  bool json_output_{false};  // output path to json file

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
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occ_map_pub_;        // Publishes original occupancy grid
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr voro_occ_grid_pub_;  // Publishes voronoi map occupancy grid

  // Planning publishers
  rclcpp::Publisher<gestelt_interfaces::msg::SpaceTimePath>::SharedPtr fe_plan_pub_; // Publish front-end plans
  rclcpp::Publisher<gestelt_interfaces::msg::SpaceTimePath>::SharedPtr fe_plan_broadcast_pub_; // Publish front-end plans broadcasted to other agents

  rclcpp::Publisher<minco_interfaces::msg::PolynomialTrajectory>::SharedPtr poly_traj_pub_; // Publish polynomial trajectories for execution
  rclcpp::Publisher<minco_interfaces::msg::MincoTrajectory>::SharedPtr minco_traj_broadcast_pub_; // Publish MINCO trajectories broadcasted to other agents

  // Visualization
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr plan_req_pub_; // start and goal visualization publisher
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr fe_closed_list_viz_pub_; // Closed list publishers
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr fe_plan_viz_pub_; // Publish front-end plan visualization
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr voronoi_graph_pub_; // publisher of voronoi graph vertices

  /**
   * ROS Subscribers
   */
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;               // Subscriber to odometry
	rclcpp::Subscription<gestelt_interfaces::msg::Goals>::SharedPtr goals_sub_;              // Goal subscriber
	rclcpp::Subscription<gestelt_interfaces::msg::SpaceTimePath>::SharedPtr fe_plan_broadcast_sub_;  // Subscription to broadcasted front end plan from other agents

	rclcpp::Subscription<gestelt_interfaces::msg::PlanRequest>::SharedPtr plan_req_dbg_sub_; // (DEBUG USE) plan request (start and goal) debug subscriber

private:
  /* Stored sensor data */
  Eigen::Vector3d cur_pos_, cur_vel_;   // [LOCAL FRAME] current state

  /* Mutexes*/
  std::mutex rsvn_tbl_mtx_;
  std::mutex voro_map_mtx_;
  std::mutex cur_state_mtx_;

  /* Flags*/
  bool init_voro_maps_{false}; // flag to indicate if voronoi map is initialized
  bool plan_complete_{false}; // flag to indicate a plan has been completed

  /* Mapping */
  std::shared_ptr<voxel_map::VoxelMap> voxel_map_;  // Occupancy map object
  voxel_map::BoolMap3D bool_map_3d_; // Bool map slices 
  std::map<int, std::shared_ptr<dynamic_voronoi::DynamicVoronoi>> dyn_voro_arr_; // array of voronoi objects with key of height (cm)

  /* Planner  */
  std::unique_ptr<global_planner::SpaceTimeAStar> fe_planner_; // Front end planner
  std::map<int, RsvnTbl> rsvn_tbl_; // map{drone_id : unordered_set{(x,y,z,t)}}. Reservation table of (x,y,z_cm, t) where x,y are grid positions, z_cm is height in centimeters and t is space time units

  Waypoint waypoints_; // Goal waypoint handler object

  std::vector<Eigen::Vector3d> fe_path_; // Front-end Space path in space coordinates (x,y,z) in world frame
  std::vector<Eigen::Vector4d> fe_path_with_t_; // Front-end Space time  path in space-time coordinates (x,y,z,t) in world frame

  std::unique_ptr<minco::MinJerkOpt> min_jerk_opt_; // Initial minimum jerk trajectory
  std::shared_ptr<minco::Trajectory> poly_traj_; // Front-end MINCO Trajectory

  /* Debugging Use */
  logger_wrapper::Timer tm_front_end_plan_{"front_end_plan"}; // Timer to measure runtime
  logger_wrapper::Timer tm_voro_map_init_{"voro_map_init"}; // Timer to measure runtime

  /* Visualization */
  std::shared_ptr<viz_helper::VizHelper> viz_helper_; // Class to aid visualization

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
  occ_grid.header.frame_id = "map";
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
  occ_grid.header.frame_id = "map";
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

} // namespace navigator

// void realignBoolMap(bool ***map, bool ***map_og, int& size_x, int& size_y)
// {
//   for (int x=0; x<size_x; x++) {
//     (*map)[x] = new bool[size_y];
//   }

//   for(int j = 0; j < size_y; j++)
//   {
//     for (int i = 0; i < size_x; i++)
//     {
//       (*map)[i][j] = (*map_og)[i][size_y-j-1];
//     }
//   }
// }

#endif // _VORONOI_PLANNER_HPP_


