#include "astar_planner/astar.hpp"

#include "gestelt_core/planner_exceptions.hpp"

#include "occ_map/cost_values.hpp"

namespace astar_planner
{

AStar::AStar()
{}

AStar::~AStar()
{}

void AStar::setOccMap(std::shared_ptr<occ_map::OccMap> occ_map, 
                        bool allow_unknown)
{
    allow_unknown_ = allow_unknown;
    occ_map_ = occ_map;

    set_occ_map_ = true;
}

void AStar::setGoal(const Eigen::Vector3d& goal){
    goal_pos_ = goal;
}

void AStar::setStart(const Eigen::Vector3d& start){
    start_pos_ = start;
}

int AStar::computePath(const int& max_iterations, 
    std::function<bool()> cancelChecker) {

    if (!set_occ_map_){
        throw gestelt_core::PlannerException("Occupancy map not set for A* planner");
        return 0;
    }


    const auto start_idx = occ_map_->posToIdx(start_pos_);
    const auto goal_idx = occ_map_->posToIdx(goal_pos_);

    std::cout << "AStar::computePath in map frame from " 
    << start_pos_.transpose() << " to " << goal_pos_.transpose() << std::endl;

    std::cout << "AStar::computePath in idx frame from " 
            << start_idx.transpose() << " to " << goal_idx.transpose() << std::endl;
        
    // RCLCPP_INFO(
    //     logger_, "AStar::computePath in map_frame from (%.2f, %.2f, %.2f) to "
    //     "(%.2f, %.2f, %.2f).", 
    //     start_idx(0), start_idx(1), start_idx(2),
    //     goal_idx(0), goal_idx(1), goal_idx(2));

    came_from_[start_idx] = start_idx;
    cost_to_come_[start_idx] = 0.0;

    open_list_.put(start_idx, 0.0);

    static int num_iter = 0;

    while (!open_list_.empty())
    {
        if (cancelChecker()){
            throw gestelt_core::PlannerCancelled("Planner was cancelled");
            return 0;
        }
        if (num_iter >= max_iterations){
            throw gestelt_core::PlannerTimedOut("Planner reached maximum iterations");
            return 0;
        }
        
        auto cur_idx = open_list_.get();
    
        if (cur_idx == goal_idx)
        {
            planned_path_idx_ = tracePath(cur_idx);

            return planned_path_idx_.size();
        }

        for (auto nb_idx : getNeighbours(cur_idx))
        {
            double tent_g_cost = cost_to_come_[cur_idx] + L1Dist(cur_idx, nb_idx);
         
            if (!cost_to_come_.count(nb_idx) || tent_g_cost < cost_to_come_[nb_idx])
            {
                cost_to_come_[nb_idx] = tent_g_cost;

                double f_cost = cost_to_come_[nb_idx] 
                    + h_weight_ * L1Dist(nb_idx, goal_idx);

                came_from_[nb_idx] = cur_idx;
                open_list_.put(nb_idx, f_cost);

                // if (!closed_list_.count(nb_idx)) 
                // {
                //     came_from_[nb_idx] = cur_idx;
                //     open_list_.put(nb_idx, f_cost);
                // }
            }
        }

        num_iter++;
    }

    std::cout << "num_iter: " << num_iter << std::endl;

    throw gestelt_core::PlannerException("Open list is empty. No more valid nodes to search");

    return 0;
}

std::vector<Eigen::Vector3i> AStar::getNeighbours(const Eigen::Vector3i& idx)
{
    std::vector<Eigen::Vector3i> neighbours;

    // Explore all 26 neighbours of 3d grid
    for (int dx = -1; dx <= 1; dx++)
    {
        for (int dy = -1; dy <= 1; dy++)
        {
            for (int dz = -1; dz <= 1; dz++)
            {
                if (dx == 0 && dy == 0 && dz == 0){
                    // Skip it's own position
                    continue;
                }
                
                Eigen::Vector3i nb_idx(idx(0)+dx, idx(1)+dy, idx(2)+dz);

                if (!occ_map_->inGlobalMapIdx(nb_idx)){
                    continue;
                }

                int cost = occ_map_->getCostIdx(nb_idx);
                    
                if (cost == occ_map::LETHAL_OBSTACLE){
                    continue;
                }

                if (!allow_unknown_ && cost == occ_map::NO_INFORMATION){
                    continue;
                }

                neighbours.push_back(nb_idx);
            }
        }
    }

    return neighbours;
}

std::vector<Eigen::Vector3i> AStar::tracePath(const Eigen::Vector3i& goal_idx)
{
    std::vector<Eigen::Vector3i> planned_path_idx;

    planned_path_idx.push_back(goal_idx);
    
    auto cur_idx = goal_idx;

    while (cur_idx != came_from_[cur_idx])
    {
        planned_path_idx.push_back(cur_idx);
        cur_idx = came_from_[cur_idx];
    }
    planned_path_idx.push_back(cur_idx);


    std::reverse(planned_path_idx.begin(), planned_path_idx.end());

    return planned_path_idx;
}

std::vector<Eigen::Vector3d> AStar::getPath()
{
    std::vector<Eigen::Vector3d> planned_path_pos;

    for (const auto& pt : planned_path_idx_){
        planned_path_pos.push_back(occ_map_->idxToPos(pt));
    }

    return planned_path_pos;
}

}  // namespace astar_planner
