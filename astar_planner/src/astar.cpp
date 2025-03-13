#include "astar_planner/astar.hpp"

#include "occ_map/cost_values.hpp"

namespace astar_planner
{

/**
 * @brief  Constructs the planner
 * @param nx The x size of the map
 * @param ny The y size of the map
 * @param nz The z size of the map
 */
AStar::AStar()
{
}

~AStar::AStar()
{

}

void AStar::setCostmap(std::shared_ptr<occ_map::OccMap> occ_map, 
    bool allow_unknown = true)
{
    allow_unknown_ = allow_unknown;
    occ_map_ = occ_map;

    set_costmap_ = true;
}

void AStar::setGoal(const Eigen::Vector3d& goal){
    goal_pos_ = goal;
}

void AStar::setStart(const Eigen::Vector3d& start){
    start_pos_ = start;
}

int AStar::computePath(const int& max_iterations){
    if (!set_costmap_){
        std::cout << "AStar: Costmap not initialized yet!" << std::endl;
        return 0;
    }

    const auto start_idx = occ_map->posToIdx(start_pos_);
    const auto goal_idx = occ_map->posToIdx(goal_pos_);

    came_from_[start_idx] = start_idx;
    g_cost_[start_idx] = 0.0;

    open_list_.put(start_idx, 0.0);

    static int num_iter = 0;

    while (!open_list_.empty())
    {
        if (num_iter >= max_iterations){
            std::cout << "AStar reached maximum iterations!" << std::endl;
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
            double tent_g_cost = g_cost_[cur_idx] + euclid_dist(cur_idx, nb_idx);
         
            if (!g_cost_.count(nb_idx) || tent_g_cost < g_cost_[nb_idx])
            {
                g_cost_[nb_idx] = tent_g_cost;

                double f_cost = g_cost_[nb_idx] 
                    + h_weight_ * euclid_dist(nb_idx, goal_idx);

                if (!closed_list_.count(nb_idx)) 
                {
                    came_from_[nb_idx] = cur_idx;
                    open_list_.put(nb_idx, f_cost);
                }
            }
        }

        num_iter++;
    }

    return 0;
}

std::vector<Eigen::Vector3i> getNeighbours(const Eigen::Vector3i& idx)
{
    std::vector<Eigen::Vector3i> neighbours;

    // Explore all 26 neighbours of 3d grid
    for (int dx = -1; dx <= 1; dx++)
    {
        for (int dy = -1; dy <= 1; dy++)
        {
            for (int dz = -1; dz <= 1; dz++)
            {
                // Skip it's own position
                if (dx == 0 && dy == 0 && dz == 0){
                    continue;
                }
                
                Eigen::Vector3i nb_idx{ idx(0) + dx*occ_map_->getRes(), 
                                        idx(1) + dy*occ_map_->getRes(), 
                                        idx(2) + dz*occ_map_->getRes()};

                if (!inGlobalMapIdx(nb_idx)){
                    continue;
                }

                int cost = occ_map_->getCostIdx(nb_idx);
                    
                if (cost == occ_map::LETHAL_OBSTACLE){
                    continue;
                }

                if (!allow_unknown && cost == occ_map::NO_INFORMATION){
                    continue;
                }

                neighbours.push_back(nb_idx);
            }
        }
    }

    return neighbours;

}

double euclid_dist(const Eigen::Vector3i& a, const Eigen::Vector3i& b)
{
    double dx = abs(a.x() - b.x());
    double dy = abs(a.y() - b.y());
    double dz = abs(a.z() - b.z());
  
    return sqrt(dx*dx + dy*dy + dz*dz);
}

std::vector<Eigen::Vector3i> tracePath(const Eigen::Vector3i& goal_idx)
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

std::vector<Eigen::Vector3d> getPath()
{
    std::vector<Eigen::Vector3i> planned_path_pos;

    for (const auto& pt : planned_path_idx_){
        planned_path_pos.push_back(occ_map_->idxToPos(pt));
    }

    return planned_path_pos;
}

}  // namespace astar_planner
