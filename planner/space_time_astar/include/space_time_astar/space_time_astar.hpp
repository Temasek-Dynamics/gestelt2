#ifndef SPACE_TIME_ASTAR__SPACE_TIME_ASTAR_HPP_
#define SPACE_TIME_ASTAR__SPACE_TIME_ASTAR_HPP_

#include "space_time_astar/visibility_control.h"

#include <unordered_set>
#include <queue>

#include <Eigen/Eigen>

#include <rclcpp/rclcpp.hpp>

#include <planner_utils/planner_common.hpp>

#include <dynamic_voronoi/dynamic_voronoi.hpp>

namespace global_planner{

  struct AStarParams{
    int drone_id{-1};
    int max_iterations; // Maximum iterations for Astar to run
    double tie_breaker;
    bool debug_viz; // Publish visualization messages for debugging 

    /**
     * 0: Octile
     * 1: L1 Norm 
     * 2: L2 Norm (Euclidean)
     * 3: Chebyshev
     * 4: 
     */
    int cost_function_type; // Type of cost function to use

    double t_unit{0.1};          // [s] Time duration of each space-time A* unit

    // int st_space_diag{3};   // Number of space-time A* units to move from (0,0,0) to (1,1,1) in 26-connected 3D grid
    // int st_face_diag{2};    // Number of space-time A* units to move from (0,0,0) to (0,1,1) in 26-connected 3D grid
    int st_straight{1};     // Number of space-time A* units to move from (0,0,0) to (0,1,0) in 26-connected 3D grid
    // int st_wait{1};         // Number of space-time A* units to wait in the same cell

  }; // struct AStarParams

  struct VoronoiParams{
    int z_separation_cm; // separation between the voronoi planes in units of centimeters

    double local_origin_x; // [m] local map x origin
    double local_origin_y; // [m] local map y origin
    double max_height_cm;  // [cm] max height
    double min_height_cm;  // [cm] min height
    double res;         // [m] resolution
  }; // struct VoronoiParams

class SpaceTimeAStar 
{
public:
  SpaceTimeAStar(const AStarParams& astar_params, rclcpp::Clock::SharedPtr clock);

  virtual ~SpaceTimeAStar();

  /**
   * @brief Clear closed, open list and reset planning_successful flag for 
   * new plan generation
   * 
   */
  void reset();

  /* Assign voronoi map. To be executed when map is updated*/
  void setVoroMap(const std::map<int, std::shared_ptr<dynamic_voronoi::DynamicVoronoi>>& dyn_voro_arr,
                     const VoronoiParams& voro_params);

  /* Update the assignement of the reservation table. TO be executed when the reservation table is updated*/
  void setReservationTable(const std::map<int, RsvnTable>& rsvn_tbl);

  /* Generate space-time plan on voronoi graph  */
  bool generatePlan( const Eigen::Vector3d& start_pos_3d, 
                          const Eigen::Vector3d& goal_pos_3d);

private: 
  /* Generate space-time plan on voronoi graph  */
  bool generatePlan( const Eigen::Vector3d& start_pos_3d, 
                          const Eigen::Vector3d& goal_pos_3d, 
                          std::function<double(const VCell_T&, const VCell_T&)> cost_function);

  // Expand voronio bubble around given cell
  void expandVoronoiBubbleT(const VCell_T& origin_cell);

  /* Trace path from planning results */
  void tracePath(const VCell_T& final_node);

/* Getter methods */
public:
  /**
   * @brief Get successful plan in terms of space i,e. (x,y,z)
   *
   * @return std::vector<Eigen::Vector3d>
   */
  std::vector<Eigen::Vector3d> getPath();

  /**
   * @brief Get successful plan in terms of space and time i.e. (x,y,z,t)
   *
   * @return std::vector<Eigen::Vector3d>
   */
  std::vector<Eigen::Vector4d> getPathWithTime();

  /**
   * @brief Get successful plan in terms of space i,e. (x,y,z)
   *
   * @return std::vector<Eigen::Vector3d>
   */
  std::vector<Eigen::Vector3d> getPath(const Eigen::Vector3d& cur_pos);

  /**
   * @brief Get successful plan in terms of space and time i.e. (x,y,z,t)
   *
   * @return std::vector<Eigen::Vector4d>
   */
  std::vector<Eigen::Vector4d> getPathWithTime(const Eigen::Vector3d& cur_pos);

  /* Get post-smoothed path */
  std::vector<Eigen::Vector4d> getSmoothedPathWithTime();

  /* Get post-smoothed path */
  std::vector<Eigen::Vector3d> getSmoothedPath();

  /**
   * @brief Get successful plan in terms of path positions
   *
   * @return std::vector<Eigen::Vector3d>
   */
  std::vector<Eigen::Vector3d> getClosedList();

/* Helper methods */
private:

  /**
   * @brief Round to nearest multiple 
   * 
   * @param num Number to be rounded
   * @param mult Multiple
   * @return int 
   */
  int roundToMultInt(const int& num, const int& mult);

  /* Convert from time [s] to space-time units */
  long tToSpaceTimeUnits(const double& t);

  /* Check line of sight between 2 points*/
  bool lineOfSight(IntPoint s, IntPoint s_, int z_cm);

private: 

  /* Params */
  AStarParams astar_params_;
  VoronoiParams voro_params_;

  /* ROS */
  rclcpp::Clock::SharedPtr clock_;

  /* Path planner data structures */
  std::map<int, std::shared_ptr<dynamic_voronoi::DynamicVoronoi>> dyn_voro_arr_; // array of voronoi objects with key of height (cm)

  // General voronoi params
  std::map<int, std::unordered_set<IntPoint>> marked_bubble_cells_; // Cells that are marked as part of the voronoi bubble with key of height(cm)

  // Space time voronoi Voronoi search data structures
  std::vector<Eigen::Vector3d> path_pos_; // (WORLD FRAME) Spatial path 
  std::vector<Eigen::Vector4d> path_pos_t_; // (WORLD FRAME) Spatial-temporal path 
  std::vector<Eigen::Vector3d> path_smoothed_; // (WORLD FRAME) post smoothed spatial path 
  std::vector<Eigen::Vector4d> path_smoothed_t_; // (WORLD FRAME) post smoothed spatial-temporal path 
  std::vector<VCell_T> path_idx_vt_; // (LOCAL FRAME) Final planned Path in terms of indices
  std::vector<VCell_T> path_idx_smoothed_t_; // (LOCAL FRAME) Final planned Path in terms of indices

  std::unordered_map<VCell, double> g_cost_v_;  // Cost-to-come hash map
  std::unordered_map<VCell_T, VCell_T> came_from_vt_; // Predecessor hash map
  PriorityQueue<VCell_T, double> open_list_vt_; // Min priority queue 
  std::unordered_set<VCell_T> closed_list_vt_; // All closed nodes

  // map{drone_id : unordered_set{(x,y,z,t)}}
  std::map<int, RsvnTable> rsvn_tbl_; // Reservation table of (x,y,z_cm, t) where x,y are grid positions, z_cm is height in centimeters and t is space time units
};

void SpaceTimeAStar::setReservationTable(const std::map<int, RsvnTable>& rsvn_tbl)
{
  rsvn_tbl_ = rsvn_tbl;
}

std::vector<Eigen::Vector3d> SpaceTimeAStar::getPath()
{
  return path_pos_;
}

std::vector<Eigen::Vector4d> SpaceTimeAStar::getPathWithTime()
{
  return path_pos_t_;
}

std::vector<Eigen::Vector3d> SpaceTimeAStar::getPath(const Eigen::Vector3d& cur_pos)
{
  std::vector<Eigen::Vector3d> path_pos = path_pos_;

  // convert from local frame to world frame
  Eigen::Vector3d start_pos(cur_pos(0)+voro_params_.local_origin_x, 
                            cur_pos(1)+voro_params_.local_origin_y, 
                            cur_pos(2));
  path_pos.insert(path_pos.begin(), start_pos);

  return path_pos;
}

std::vector<Eigen::Vector4d> SpaceTimeAStar::getPathWithTime(const Eigen::Vector3d& cur_pos)
{
  std::vector<Eigen::Vector4d> path_pos_t = path_pos_t_;

  // Transform to world frame
  Eigen::Vector4d cur_pos_t(cur_pos(0)+voro_params_.local_origin_x, 
                          cur_pos(1)+voro_params_.local_origin_y, 
                          cur_pos(2), 
                          0);

  // offset_t: time from current position to start of path
  int offset_t = (int)round((cur_pos_t.segment(0,3) - path_pos_t[0].segment(0,3)).norm()/voro_params_.res) * astar_params_.st_straight; 

  if (offset_t == 0){ // Distance between start and path start is very close
    // Replace start point of path with current point
    path_pos_t[0] = cur_pos_t;
  }
  else {
    // Insert current point at beginning of path
    path_pos_t.insert(path_pos_t.begin(), cur_pos_t);
    // Offset all time 
    for (size_t i = 1; i < path_pos_t.size(); i++){
      path_pos_t[i](3) += offset_t;
    }
  }

  return path_pos_t;
}

std::vector<Eigen::Vector4d> SpaceTimeAStar::getSmoothedPathWithTime()
{
  return path_smoothed_t_;
}

std::vector<Eigen::Vector3d> SpaceTimeAStar::getSmoothedPath()
{
  return path_smoothed_;
}

std::vector<Eigen::Vector3d> SpaceTimeAStar::getClosedList()
{
  std::vector<Eigen::Vector3d> closed_list_pos;
  for (auto itr = closed_list_vt_.begin(); itr != closed_list_vt_.end(); ++itr) {
    DblPoint map_pos;
    IntPoint grid_pos((*itr).x, (*itr).y);

    dyn_voro_arr_[(*itr).z_cm]->idxToPos(grid_pos, map_pos);

    closed_list_pos.push_back(Eigen::Vector3d{map_pos.x, map_pos.y, (*itr).z_m});
  }

  return closed_list_pos; 
}

int SpaceTimeAStar::roundToMultInt(const int& num, const int& mult)
{
  if (mult == 0){
    return num;
  }

  if (num >= voro_params_.max_height_cm){
    return voro_params_.max_height_cm;
  }

  if (num <= voro_params_.min_height_cm){
    return voro_params_.min_height_cm;
  }

  int rem = (int)num % mult;
  if (rem == 0){
    return num;
  }

  return rem < (mult/2) ? (num-rem) : (num-rem) + mult;
}

long SpaceTimeAStar::tToSpaceTimeUnits(const double& t){
  return std::lround(t / astar_params_.t_unit);
}

bool SpaceTimeAStar::lineOfSight(IntPoint s, IntPoint s_, int z_cm)
{
    int x0 = s.x;
    int y0 = s.y;
    int x1 = s_.x;
    int y1 = s_.y;

    int dy = y1 - y0;
    int dx = x1 - x0;

    int f = 0;

    int sx = 1; // Direction taken by x
    int sy = 1; // Direction taken by y

    // Correct for y direction
    if (dy < 0) {
        dy = -dy;
        sy = -1;
    }
    else {
        sy = 1;
    }
    
    // Correct for x direction
    if (dx < 0) {
        dx = -dx;
        sx = -1;
    }
    else {
        sx = 1;
    }

    if (dx >= dy) {
        while (x0 != x1){
            f += dy;
            if (f >= dx){
                if (dyn_voro_arr_[z_cm]->isOccupied(x0 + ((sx-1)/2), y0 + ((sy - 1)/2)))
                {
                    return false;
                }
                y0 = y0 + sy;
                f -= dx;
            }
            if (f != 0 && dyn_voro_arr_[z_cm]->isOccupied(x0 + ((sx-1)/2), y0 + ((sy - 1)/2)))
            {
                return false;
            }
            if (dy == 0 && dyn_voro_arr_[z_cm]->isOccupied(x0 + ((sx-1)/2), y0)
                        && dyn_voro_arr_[z_cm]->isOccupied(x0 + ((sx-1)/2), y0 -1))
            {
                return false;
            }
            x0 = x0 + sx;
        }
    }
    else {
        while (y0 != y1){
            f += dx;
            if (f >= dy){
                if (dyn_voro_arr_[z_cm]->isOccupied(x0 + ((sx-1)/2), y0 + ((sy - 1)/2)))
                {
                    return false;
                }
                x0 += sx;
                f -= dy;
            }
            if (f != 0 && dyn_voro_arr_[z_cm]->isOccupied(x0 + ((sx-1)/2), y0 + ((sy - 1)/2)))
            {
                return false;
            }
            if (dy == 0 && dyn_voro_arr_[z_cm]->isOccupied(x0, y0 + ((sy-1)/2))
                        && dyn_voro_arr_[z_cm]->isOccupied(x0-1, y0+ ((sy-1)/2)))
            {
                return false;
            }
            y0 += sy;
        }
    }

    return true;
}

} // namespace global_planner

#endif  // SPACE_TIME_ASTAR__SPACE_TIME_ASTAR_HPP_
