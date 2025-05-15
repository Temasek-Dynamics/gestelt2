#ifndef ASTAR_PLANNER__ASTAR_HPP_
#define ASTAR_PLANNER__ASTAR_HPP_

#include <memory>
#include <unordered_map>
#include <unordered_set>

#include <Eigen/Eigen>

#include "occ_map/occ_map.hpp"

template <> 
struct std::hash<Eigen::Vector3i> {
    std::size_t operator() (const Eigen::Vector3i& idx) const noexcept {
        std::size_t seed = 0;
        seed ^= std::hash<int>()(idx(0)) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^= std::hash<int>()(idx(1)) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^= std::hash<int>()(idx(2)) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        return seed;
    }
};

namespace astar_planner
{

template<typename T, typename priority_t>
struct PriorityQueue {
    typedef std::pair<priority_t, T> PQElement;

    struct PQComp {
        constexpr bool operator()(
            PQElement const& a,
            PQElement const& b)
            const noexcept
        {
            return a.first > b.first;
        }
    };

    std::priority_queue<PQElement, std::vector<PQElement>, PQComp > elements;

    inline bool empty() const {
        return elements.empty();
    }

    inline void put(T item, priority_t priority) {
        elements.emplace(priority, item);
    }

    T get() {
        T best_item = elements.top().second;
        elements.pop();
        return best_item;
    }

    void clear() {
        elements = std::priority_queue<PQElement, std::vector<PQElement>, PQComp>();
    }
};

class AStar
{
public:

  /**
   * @brief  Constructs the planner
   * @param nx The x size of the map
   * @param ny The y size of the map
   * @param nz The z size of the map
   */
  AStar();

  ~AStar();

  /**
   * @brief  Set up the cost array for the planner, usually from ROS
   * @param occ_map The occupancy map
   * @param allow_unknown Whether or not the planner should be allowed to plan through
   *   unknown space
   */
  void setOccMap(std::shared_ptr<occ_map::OccMap> occ_map, 
      bool allow_unknown = true);

  /**
   * @brief  Sets the goal position for the planner.
   * Note: the navigation cost field computed gives the cost to get to a given point
   * from the goal, not from the start.
   * @param goal the goal position in index coordinates
   */
  void setGoal(const Eigen::Vector3d& goal);

  /**
   * @brief  Sets the start position for the planner.
   * Note: the navigation cost field computed gives the cost to get to a given point
   * from the goal, not from the start.
   * @param start the start position in index coordinates
   */
  void setStart(const Eigen::Vector3d& start);

  int computePath(const int& max_iterations, std::function<bool()> cancelChecker);

  std::vector<Eigen::Vector3i> getNeighbours(const Eigen::Vector3i& idx);

  inline double L1Dist(const Eigen::Vector3i& a, const Eigen::Vector3i& b)
  {
      return (b - a).lpNorm<1>();
  }

  inline double L2Dist(const Eigen::Vector3i& a, const Eigen::Vector3i& b)
  {
      return (b - a).norm();
  }

  /**
   * @brief Return planned path in distance coordinates in map frame
   * 
   * @return std::vector<Eigen::Vector3d> 
   */
  std::vector<Eigen::Vector3d> getPath();

  /**
   * @brief Trace path and return planned path in pixel coordinates in map frame
   * 
   * @return std::vector<Eigen::Vector3d> 
   */
  std::vector<Eigen::Vector3i> tracePath(const Eigen::Vector3i& goal_idx);

private:

  /* Params */
  bool allow_unknown_{true};

  bool set_occ_map_{false};

  double h_weight_{1.0};

  Eigen::Vector3d goal_pos_, start_pos_;  // start and goal in index coordinates

  std::shared_ptr<occ_map::OccMap> occ_map_;

  std::unordered_map<Eigen::Vector3i, double> cost_to_come_; 
  std::unordered_map<Eigen::Vector3i, Eigen::Vector3i> came_from_; // predecessor
  PriorityQueue<Eigen::Vector3i, double> open_list_; // Min priority queue 
  std::unordered_set<Eigen::Vector3i> closed_list_; // All closed nodes

  std::vector<Eigen::Vector3i> planned_path_idx_; // Planned path in pixel coordinates 
  std::vector<Eigen::Vector3d> planned_path_pos_; // Planned path in position (meters) 
      
}; // class AStar

}  // namespace astar_planner

#endif  // ASTAR_PLANNER__ASTAR_HPP_