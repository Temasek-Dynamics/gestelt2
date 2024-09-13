#include "space_time_astar_planner/space_time_astar_planner.hpp"

AStarPlanner::AStarPlanner( const AStarParams& astar_params): astar_params_(astar_params)
{}

void AStarPlanner::assignVoroMap(const std::map<int, std::shared_ptr<DynamicVoronoi>>& dyn_voro_arr,
                                const int& z_separation_cm,
                                const double& local_origin_x,
                                const double& local_origin_y,
                                const double& max_height,
                                const double& min_height,
                                const double& res)
{
    dyn_voro_arr_ = dyn_voro_arr;

    z_separation_cm_ = z_separation_cm;
    local_origin_x_ = local_origin_x;
    local_origin_y_ = local_origin_y;
    max_height_ = max_height;
    min_height_ = min_height;
    res_ = res;
}

void AStarPlanner::updateReservationTable(const std::map<int, RsvnTable>& rsvn_tbl)
{
    rsvn_tbl_ = rsvn_tbl;
}

void AStarPlanner::reset()
{
    // Voronoi planning data structs
    marked_bubble_cells_.clear();

    // Space time Voronoi planning data structs
    g_cost_v_.clear();
    came_from_vt_.clear();
    open_list_vt_.clear();
    closed_list_vt_.clear();

}

void AStarPlanner::expandVoronoiBubbleT(const VCell_T& origin_cell)
{
    std::queue<IntPoint> q;
    q.push(IntPoint(origin_cell.x, origin_cell.y));

    IntPoint grid(origin_cell.x, origin_cell.y);

    marked_bubble_cells_[origin_cell.z_cm].insert(grid);

    while(!q.empty()) {
        IntPoint cur_cell = q.front();
        q.pop();

        for (int dx=-1; dx<=1; dx++) {
            int nx = cur_cell.x + dx;
            if (nx<0 || nx >= (int) dyn_voro_arr_[origin_cell.z_cm]->getSizeX()) {
                continue; // Skip if outside map
            }
            for (int dy=-1; dy<=1; dy++) {
                int ny = cur_cell.y + dy;
                if (dx && dy) {
                    continue; // skip if not 4 connected connection
                }
                if (ny<0 || ny >= (int) dyn_voro_arr_[origin_cell.z_cm]->getSizeY()){
                    continue; // Skip if outside map
                } 

                IntPoint nb_grid(nx, ny);

                if (marked_bubble_cells_[origin_cell.z_cm].find(nb_grid) != marked_bubble_cells_[origin_cell.z_cm].end()) 
                {
                    continue; // If already added to marked bubble list, then skip
                }

                if (dyn_voro_arr_[origin_cell.z_cm]->getSqrDistance(nx,ny) < 1) 
                {
                    continue;   // Skip if occupied or near obstacle
                }

                // mark as closed bubble cells
                marked_bubble_cells_[origin_cell.z_cm].insert(nb_grid);

                if (!dyn_voro_arr_[origin_cell.z_cm]->isVoronoi(nx,ny)){ 
                    // if 4-con neighbor is not voronoi then push to list
                    q.push(nb_grid); 
                }
            }

        }
    }
}

bool AStarPlanner::generatePlanVoroT(  const Eigen::Vector3d& start_pos_3d, 
                                        const Eigen::Vector3d& goal_pos_3d)
{
    std::function<double(const VCell_T&, const VCell_T&)> cost_function;

    switch ( astar_params_.cost_function_type ) {
        case 0:
            // std::cout << "[AStar]: Using octile distance cost " << std::endl; 
            cost_function = getOctileDistVT;
            break;
        case 1:
            // std::cout << "[AStar]: Using L1 Norm" << std::endl; 
            cost_function = getL1NormVT; 
            break;
        case 2:
            // std::cout << "[AStar]: Using L2 Norm " << std::endl; 
            cost_function = getL2NormVT;
            break;
        case 3:
            // std::cout << "[AStar]: Using Chebyshev Distance" << std::endl;
            cost_function = getChebyshevDistVT;
            break;
        default: 
            // std::cout << "[AStar]: Using Octile Distance" << std::endl;
            cost_function = getOctileDistVT;
            break;
    }

    return generatePlanVoroT(start_pos_3d, goal_pos_3d, cost_function);
}

bool AStarPlanner::generatePlanVoroT(   const Eigen::Vector3d& start_pos_3d, 
                                        const Eigen::Vector3d& goal_pos_3d, 
                                        std::function<double(const VCell_T&, const VCell_T&)> cost_function)
{
    reset();
    
    int start_z_cm = roundToMultInt((int) (start_pos_3d(2) * 100), z_separation_cm_);
    int goal_z_cm = roundToMultInt((int) (goal_pos_3d(2) * 100), z_separation_cm_);
    // std::cout << astar_params_.drone_id << ": start_z: " <<  start_pos_3d(2) << " m rounded to " << start_z_cm << " cm" << std::endl;
    // std::cout << astar_params_.drone_id << ": goal_z: " <<  goal_pos_3d(2) << " m rounded to " << goal_z_cm << " cm" << std::endl;
    
    INTPOINT start_node_2d, goal_node_2d;

    // Search takes place in index space. So we first convert 3d real world positions into indices
    if (!dyn_voro_arr_[start_z_cm]->posToIdx(DblPoint(start_pos_3d(0), start_pos_3d(1)), start_node_2d) 
        || !dyn_voro_arr_[goal_z_cm]->posToIdx(DblPoint(goal_pos_3d(0), goal_pos_3d(1)), goal_node_2d))
    {   
        std::cerr << "[a_star] Start or goal position is not within map bounds!" << std::endl;
        return false;
    }

    if (dyn_voro_arr_[start_z_cm]->isOccupied(start_node_2d)){
        std::cerr << "[space-time A*] Start position in obstacle!" << std::endl;
        return false;
    }
    if (dyn_voro_arr_[goal_z_cm]->isOccupied(goal_node_2d)){
        std::cerr << "[space-time A*] Goal position in obstacle!" << std::endl;
        return false;
    }

    VCell_T start_node(start_node_2d.x, start_node_2d.y, start_z_cm, 0);
    VCell_T goal_node(goal_node_2d.x, goal_node_2d.y, goal_z_cm, -1);

    ////////
    // Method A: use voronoi bubble expansion
    ////////

    // set start and goal cell as obstacle

    dyn_voro_arr_[start_node.z_cm]->setObstacle(start_node.x, start_node.y);
    dyn_voro_arr_[goal_node.z_cm]->setObstacle(goal_node.x, goal_node.y);

    dyn_voro_arr_[start_node.z_cm]->update(); // update distance map and Voronoi diagram
    dyn_voro_arr_[goal_node.z_cm]->update(); // update distance map and Voronoi diagram

    // std::cout << astar_params_.drone_id << ": before expandVoronoiBubbleT" << std::endl;

    // marked_bubble_cells_[start_node.z_cm].clear();
    // marked_bubble_cells_[goal_node.z_cm].clear();
    // Create voronoi bubble around start and goal
    expandVoronoiBubbleT(start_node);
    expandVoronoiBubbleT(goal_node);

    // std::cout << astar_params_.drone_id << ": before removeObstacle" << std::endl;

    dyn_voro_arr_[start_node.z_cm]->removeObstacle(start_node.x, start_node.y);
    dyn_voro_arr_[goal_node.z_cm]->removeObstacle(goal_node.x, goal_node.y);

    ////////
    // Method B: Straight path to nearest voronoi cell
    ////////
    // INTPOINT start_nearest_voro_cell, goal_nearest_voro_cell; 
    // dyn_voro_arr_[start_z_cm]->getNearestVoroCell(start_node_2d, start_nearest_voro_cell);
    // dyn_voro_arr_[goal_z_cm]->getNearestVoroCell(goal_node_2d, goal_nearest_voro_cell);

    came_from_vt_[start_node] = start_node;
    VCell start_node_3d(start_node_2d.x, start_node_2d.y, start_z_cm);
    g_cost_v_[start_node_3d] = 0;

    open_list_vt_.put(start_node, 0); // start_node has 0 f cost

    int num_iter = 0;
    std::vector<Eigen::Vector4i> neighbours; // 3d indices of neighbors

    double t_now = ros::Time::now().toSec();

    while (!open_list_vt_.empty() && num_iter < astar_params_.max_iterations)
    {
        // if (num_iter%100 == 1){
        //     std::cout << "[a_star] Iteration " << num_iter << std::endl;
        //     publishClosedList(getClosedListVoroT(), closed_list_viz_pub_, "local_map_origin");
        //     // ros::Duration(0.1).sleep();
        // }

        VCell_T cur_node = open_list_vt_.get();

        VCell cur_node_3d = VCell(cur_node.x, cur_node.y, cur_node.z_cm);
        closed_list_vt_.insert(cur_node);

        if (cur_node.isSamePositionAs(goal_node))
        {
            // std::cout << astar_params_.drone_id << ":  Found goal after " << num_iter << " iterations" << std::endl;
            // Goal reached, terminate search and obtain path
            tracePathVoroT(cur_node);

            return true;
        }
        

        // Get neighbours that are within the map
        dyn_voro_arr_[cur_node.z_cm]->getVoroNeighbors(
            Eigen::Vector4i(cur_node.x, cur_node.y, cur_node.z_cm, cur_node.t), 
            neighbours, marked_bubble_cells_[cur_node.z_cm]);

        // Explore neighbors of current node. Each neighbor is (grid_x, grid_y, map_z_cm)
        for (const Eigen::Vector4i& nb_grid_4d : neighbours) 
        {   
            
            VCell_T nb_node(nb_grid_4d(0), nb_grid_4d(1), nb_grid_4d(2), nb_grid_4d(3));
            VCell nb_node_3d(nb_grid_4d(0), nb_grid_4d(1), nb_grid_4d(2));

            // Synchronize current planning time to that of reservation table entries
            auto isInReservationTable = [this](const Eigen::Vector4i& grid_4d, const double &t_now) -> bool {
                for (auto const& tbl : rsvn_tbl_){ // for each agent's reservation table
                    // e_t_plan_start: space time units since plan started
                    int e_t_plan_start =  (int) tToSpaceTimeUnits(t_now - tbl.second.t_plan_start);
                    Eigen::Vector4i grid_4d_sync = grid_4d;
                    grid_4d_sync(3) += e_t_plan_start;
                    if (tbl.second.isReserved(grid_4d_sync)){
                        // std::cout << "  grid(" << grid_4d_sync.transpose() << ") is reserved " << std::endl;
                        return true;
                    }
                }
                return false;
            };

            if (isInReservationTable(nb_grid_4d, t_now))
            {
                continue;
            }


            double tent_g_cost = g_cost_v_[cur_node_3d] + cost_function(cur_node, nb_node);

            // If g_cost is not found or tentative cost is better than previously computed cost, then update costs
            if (g_cost_v_.find(nb_node_3d) == g_cost_v_.end() || tent_g_cost < g_cost_v_[nb_node_3d])
            {
                g_cost_v_[nb_node_3d] = tent_g_cost;
                // The tie_breaker is used to assign a larger weight to the h_cost and favour expanding nodes closer towards the goal
                // f_cost = g_cost (cost to come) + h_cost (cost to go)
                double f_cost = g_cost_v_[nb_node_3d] + astar_params_.tie_breaker * cost_function(nb_node, goal_node);

                // If not in closed list: set parent and add to open list
                if (closed_list_vt_.find(nb_node) == closed_list_vt_.end()) 
                {
                    came_from_vt_[nb_node] = cur_node;
                    open_list_vt_.put(nb_node, f_cost);
                }
                // No need to update parents for nodes already in closed list, paths leading up to current node is alr the most optimal
            }
        }
        num_iter++;
    }

    std::cerr   << "Drone " << astar_params_.drone_id << " :Unable to find goal node ("
                << goal_node.x << ", " << goal_node.y 
                << ") with maximum iteration " << num_iter << std::endl;

    return false;
}

bool AStarPlanner::lineOfSight(IntPoint s, IntPoint s_, int z_cm)
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
