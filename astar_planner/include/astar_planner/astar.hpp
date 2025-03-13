#ifndef ASTAR_PLANNER__ASTAR_HPP_
#define ASTAR_PLANNER__ASTAR_HPP_

#include <memory>

#include <Eigen/Eigen>

#include "occ_map/occ_map.hpp"

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
 * @param cmap The costmap
 * @param isROS Whether or not the costmap is coming in in ROS format
 * @param allow_unknown Whether or not the planner should be allowed to plan through
 *   unknown space
 */
void setCostmap(std::shared_ptr<occ_map::OccMap> occ_map, 
    bool allow_unknown = true);

/**
 * @brief  Sets the goal position for the planner.
 * Note: the navigation cost field computed gives the cost to get to a given point
 * from the goal, not from the start.
 * @param goal the goal position in index coordinates
 */
void setGoal(const Eigen::Vector3i& goal);

/**
 * @brief  Sets the start position for the planner.
 * Note: the navigation cost field computed gives the cost to get to a given point
 * from the goal, not from the start.
 * @param start the start position in index coordinates
 */
void setStart(const Eigen::Vector3i& start);

int computePath(const int& max_iterations);

std::vector<Eigen::Vector3i> getNeighbours(const Eigen::Vector3i& idx);

private:

/* Params */
bool allow_unknown_{true};

bool set_costmap_{false};

double h_weight_{1.0};

Eigen::Vector3d goal_pos_;  // goal in index coordinates
Eigen::Vector3d start_pos_; // start in index coordinates

std::shared_ptr<occ_map::OccMap> occ_map_{nullptr};

std::unordered_map<Eigen::Vector3i, double> g_cost_; 
std::unordered_map<Eigen::Vector3i, Eigen::Vector3i> came_from_; // predecessor
PriorityQueue<Eigen::Vector3i, double> open_list_; // Min priority queue 
std::unordered_set<Eigen::Vector3i> closed_list_; // All closed nodes

std::vector<Eigen::Vector3i> planned_path_idx_; // Planned path in pixel coordinates 
std::vector<Eigen::Vector3d> planned_path_pos_; // Planned path in position (meters) 
    
}; // class AStar

}  // namespace astar_planner

#endif  // ASTAR_PLANNER__ASTAR_HPP_
