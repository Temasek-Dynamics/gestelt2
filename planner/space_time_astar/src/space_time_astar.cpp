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

#include "space_time_astar/space_time_astar.hpp"

namespace global_planner{

SpaceTimeAStar::SpaceTimeAStar(const AStarParams& astar_params, rclcpp::Clock::SharedPtr clock)
: astar_params_(astar_params), clock_(clock)
{}

SpaceTimeAStar::~SpaceTimeAStar(){
}

void SpaceTimeAStar::reset()
{
    // Voronoi planning data structs
    marked_bubble_cells_.clear();

    // Space time Voronoi planning data structs
    cost_to_come_v_.clear();
    came_from_vt_.clear();
    open_list_vt_.clear();
    closed_list_vt_.clear();
}

void SpaceTimeAStar::setVoroMap(const std::map<int, std::shared_ptr<dynamic_voronoi::DynamicVoronoi>>& dyn_voro_arr,
                            const VoronoiParams& voro_params)
{
    voro_params_ = voro_params;
    dyn_voro_arr_ = dyn_voro_arr;
}

void SpaceTimeAStar::expandVoroBubble(const VCell_T& origin_cell)
{
    std::queue<IntPoint> q;
    q.push(IntPoint(origin_cell.x, origin_cell.y));
    marked_bubble_cells_[origin_cell.z_cm].insert(IntPoint(origin_cell.x, origin_cell.y));

    while(!q.empty()) {
        IntPoint cur_cell = q.front();
        q.pop();

        // Iterate through all neighbors
        for (int nx = cur_cell.x - 1; nx <= cur_cell.x + 1; nx++) {
            if (nx<0 || nx >= (int) dyn_voro_arr_[origin_cell.z_cm]->getSizeX()) {
                continue; // Skip if outside map
            }

            for (int ny = cur_cell.y - 1; ny <= cur_cell.y + 1; ny++) {

                if (ny<0 || ny >= (int) dyn_voro_arr_[origin_cell.z_cm]->getSizeY()){
                    continue; // Skip if outside map
                } 

                if (abs(nx - cur_cell.x) == 1 && abs(ny - cur_cell.y) == 1) {
                    continue; // skip if not 4 connected connection
                }

                if (abs(nx-origin_cell.x) <= 1 && abs(ny-origin_cell.y) <= 1) { // around starting cell

                    if (dyn_voro_arr_[origin_cell.z_cm]->getSqrDistToObs(nx,ny) < 1) 
                    {
                        continue;   // Skip if occupied or near obstacle
                    }
                }
                else {

                    if (dyn_voro_arr_[origin_cell.z_cm]->getSqrDistToObs(nx,ny) <= 4) 
                    {
                        continue;   // Skip if occupied or near obstacle
                    }
                }

                IntPoint nb_grid(nx, ny);

                if (marked_bubble_cells_[origin_cell.z_cm].find(nb_grid) != marked_bubble_cells_[origin_cell.z_cm].end()) 
                {
                    continue; // If already added to marked bubble list, then skip
                }

                marked_bubble_cells_[origin_cell.z_cm].insert(nb_grid);

                if (!dyn_voro_arr_[origin_cell.z_cm]->isVoronoiAlternative(nx,ny)){ 
                    // if 4-con neighbor is not voronoi then push to list
                    q.push(nb_grid); 
                }
            }

        }
    }
}

bool SpaceTimeAStar::generatePlan(const Eigen::Vector3d& start_pos_3d, 
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

    return generatePlan(start_pos_3d, goal_pos_3d, cost_function);
}

bool SpaceTimeAStar::generatePlan(const Eigen::Vector3d& start_pos_3d, 
                                const Eigen::Vector3d& goal_pos_3d, 
                                std::function<double(const VCell_T&, const VCell_T&)> cost_function)
{
    reset();

    int start_z_cm = roundToMultInt((int) (start_pos_3d(2) * 100), 
                                        voro_params_.z_sep_cm, 
                                        voro_params_.min_height_cm, 
                                        voro_params_.max_height_cm);
    int goal_z_cm = roundToMultInt((int) (goal_pos_3d(2) * 100), 
                                        voro_params_.z_sep_cm, 
                                        voro_params_.min_height_cm, 
                                        voro_params_.max_height_cm);
    // std::cout << astar_params_.drone_id << ": start_z: " <<  start_pos_3d(2) << " m rounded to " << start_z_cm << " cm" << std::endl;
    // std::cout << astar_params_.drone_id << ": goal_z: " <<  goal_pos_3d(2) << " m rounded to " << goal_z_cm << " cm" << std::endl;
    
    IntPoint start_node_2d, goal_node_2d;

    // Search takes place in index space. So we first convert 3d real world positions into indices
    if (!dyn_voro_arr_[start_z_cm]->posToIdx(
        DblPoint(start_pos_3d(0), start_pos_3d(1)), start_node_2d))
    {   
        std::cerr << "D" << astar_params_.drone_id <<  ": "
            << "[HCA*] Start position (" << start_pos_3d.transpose() 
            << "), Start Node (" << start_node_2d.x << ", " << start_node_2d.y 
            << ") is not within map bounds (" 
            << dyn_voro_arr_[start_z_cm]->getSizeX() << "," 
            << dyn_voro_arr_[start_z_cm]->getSizeY() << ")" << std::endl;
        return false;
    }
    if (!dyn_voro_arr_[goal_z_cm]->posToIdx(
        DblPoint(goal_pos_3d(0), goal_pos_3d(1)), goal_node_2d))
    {   
        std::cerr << "D" << astar_params_.drone_id <<  ": "
            << "[HCA*] Goal position (" << goal_pos_3d.transpose() 
            << "), Goal Node (" << goal_node_2d.x << ", " << goal_node_2d.y 
            << ") is not within map bounds (" 
            << dyn_voro_arr_[goal_z_cm]->getSizeX() << "," 
            << dyn_voro_arr_[goal_z_cm]->getSizeY() << ")" << std::endl;

        return false;
    }

    if (dyn_voro_arr_[start_z_cm]->isOccupied(start_node_2d)){
        std::cerr << "D" << astar_params_.drone_id <<  ": " <<
            "[HCA*] Start position in obstacle!" << std::endl;
        return false;
    }
    if (dyn_voro_arr_[goal_z_cm]->isOccupied(goal_node_2d)){
        std::cerr << "D" << astar_params_.drone_id <<  ": " <<
            "[HCA*] Goal position in obstacle!" << std::endl;
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
    // update distance map and Voronoi diagram
    dyn_voro_arr_[start_node.z_cm]->update(); 
    dyn_voro_arr_[goal_node.z_cm]->update(); 
    dyn_voro_arr_[start_node.z_cm]->updateAlternativePrunedDiagram();
    dyn_voro_arr_[goal_node.z_cm]->updateAlternativePrunedDiagram();
    expandVoroBubble(start_node);
    expandVoroBubble(goal_node);
    // Add cells around start and goal node to the marked bubble cells array
    dyn_voro_arr_[start_node.z_cm]->removeObstacle(start_node.x, start_node.y);
    dyn_voro_arr_[goal_node.z_cm]->removeObstacle(goal_node.x, goal_node.y);

    came_from_vt_[start_node] = start_node; // keep track of parents
    VCell start_node_3d(start_node_2d.x, start_node_2d.y, start_z_cm); 
    cost_to_come_v_[start_node_3d] = 0; // cost to come

    open_list_vt_.put(start_node, 0); // start_node has 0 f-cost

    int num_iter = 0;
    std::vector<Eigen::Vector4i> neighbours; // 3d indices of neighbors

    // double t_now = clock_->now().seconds();

    while (!open_list_vt_.empty() && num_iter < astar_params_.max_iterations)
    {
        // if (num_iter%100 == 1){
        //     std::cout << "[a_star] Iteration " << num_iter << std::endl;
        //     viz_helper_->pubFrontEndClosedList(getClosedList(), closed_list_viz_pub_, local_map_frame_);
        // }

        VCell_T cur_node = open_list_vt_.get();

        closed_list_vt_.insert(cur_node);

        VCell cur_node_3d = VCell(cur_node.x, cur_node.y, cur_node.z_cm);

        if (cur_node.isSamePositionAs(goal_node))
        {
            // Goal reached, terminate search and obtain path
            tracePath(cur_node);

            return true;
        }
        
        auto getVoroNeighbors = [&](const Eigen::Vector4i& grid_pos, 
                                    std::vector<Eigen::Vector4i>& neighbours,
                                    std::map<int, std::unordered_set<IntPoint>>& marked_bubble_cells)
        {
            // Input
            //  grid_pos: (cur_node.x, cur_node.y, cur_node.z_cm, cur_node.t)
            //  neighbours: To be filled with neighbours that are voronoi cells
            //  marked_bubble_cells: bubble cells that are marked
            
            neighbours.clear();

            int cur_t = grid_pos(3);
            int z_cm = grid_pos(2);

            for (int dx = -1; dx <= 1; dx++) {
                for (int dy = -1; dy <= 1; dy++) {

                    int nx = grid_pos(0) + dx;
                    int ny = grid_pos(1) + dy;

                    if (!dyn_voro_arr_[z_cm]->isInMap(nx, ny) 
                        || dyn_voro_arr_[z_cm]->isOccupied(nx, ny)){
                        continue;
                    }

                    if (!(dyn_voro_arr_[z_cm]->isVoronoiAlternative(nx, ny) 
                        || (marked_bubble_cells.find(z_cm) != marked_bubble_cells.end()
                            && marked_bubble_cells[z_cm].find(IntPoint(nx, ny)) != marked_bubble_cells[z_cm].end())))
                    {
                        // if not (voronoi or marked as bubble cell)
                        continue;
                    }

                    if (dyn_voro_arr_[z_cm]->getSqrDistToObs(nx, ny) < astar_params_.min_sqr_dist_obs)
                    {
                        continue;
                    }
                    
                    neighbours.push_back(Eigen::Vector4i{   nx, 
                                                            ny, 
                                                            z_cm, 
                                                            cur_t + 1 });
                    }
            }

            // Check if current cell is voronoi vertex. 
            // IF so, then add neighbors that go up and down
            if (dyn_voro_arr_[z_cm]->getSqrDistToObs(grid_pos(0), grid_pos(1)) > 2) {
            // if (dyn_voro_arr_[z_cm]->isVoronoiAlternative(grid_pos(0), grid_pos(1))){
                int z_cm_top = z_cm + voro_params_.z_sep_cm;
                int z_cm_btm = z_cm - voro_params_.z_sep_cm;

                if (z_cm == voro_params_.min_height_cm){
                    // Check Top layer only
                    if (!dyn_voro_arr_[z_cm_top]->isOccupied(grid_pos(0), grid_pos(1))){
                        neighbours.push_back(Eigen::Vector4i{   grid_pos(0), 
                                                                grid_pos(1), 
                                                                z_cm_top, 
                                                                cur_t + 1 });
                    }
                }
                else if (z_cm == voro_params_.max_height_cm){
                    // Check Bottom layer only
                    if (!dyn_voro_arr_[z_cm_btm]->isOccupied(grid_pos(0), grid_pos(1))){
                        neighbours.push_back(Eigen::Vector4i{   grid_pos(0), 
                                                                grid_pos(1), 
                                                                z_cm_btm, 
                                                                cur_t + 1 });
                    }
                }
                else {
                    // Check both Top and Bottom layer 
                    if (!dyn_voro_arr_[z_cm_top]->isOccupied(grid_pos(0), grid_pos(1))){
                        neighbours.push_back(Eigen::Vector4i{   grid_pos(0), 
                                                                grid_pos(1), 
                                                                z_cm_top, 
                                                                cur_t + 1 });
                    }
                    if (!dyn_voro_arr_[z_cm_btm]->isOccupied(grid_pos(0), grid_pos(1))){
                        neighbours.push_back(Eigen::Vector4i{   grid_pos(0), 
                                                                grid_pos(1), 
                                                                z_cm_btm, 
                                                                cur_t + 1 });
                    }
                }
            }
        };


        // Get neighbours that are within the map
        getVoroNeighbors(
            Eigen::Vector4i(cur_node.x, cur_node.y, cur_node.z_cm, cur_node.t), 
            neighbours, 
            marked_bubble_cells_);

        // Explore neighbors of current node. Each neighbor is (grid_x, grid_y, map_z_cm)
        for (const Eigen::Vector4i& nb_grid_4d : neighbours) 
        {   
            
            VCell_T nb_node(nb_grid_4d(0), nb_grid_4d(1), nb_grid_4d(2), nb_grid_4d(3));
            VCell nb_node_3d(nb_grid_4d(0), nb_grid_4d(1), nb_grid_4d(2));

            // // Synchronize current planning time to that of reservation table entries
            // auto isInReservationTable = [this](const Eigen::Vector4i& grid_4d, const double &t_now) -> bool {
            //     for (auto const& tbl : rsvn_tbl_){ // for each agent's reservation table
            //         // e_t_plan_start: space time units since plan started
            //         int e_t_plan_start =  (int) tToSpaceTimeUnits(t_now - tbl.second.t_plan_start);
            //         Eigen::Vector4i grid_4d_sync = grid_4d;
            //         grid_4d_sync(3) += e_t_plan_start;
            //         if (tbl.second.isReserved(grid_4d_sync)){
            //             // std::cout << "  grid(" << grid_4d_sync.transpose() << ") is reserved " << std::endl;
            //             return true;
            //         }
            //     }
            //     return false;
            // };

            // if (isInReservationTable(nb_grid_4d, t_now))
            // {
            //     continue;
            // }

            double tent_g_cost = cost_to_come_v_[cur_node_3d] + cost_function(cur_node, nb_node);

            // If g_cost is not found or tentative cost is better than previously computed cost, then update costs
            if (cost_to_come_v_.find(nb_node_3d) == cost_to_come_v_.end() 
                || tent_g_cost < cost_to_come_v_[nb_node_3d])
            {
                cost_to_come_v_[nb_node_3d] = tent_g_cost;
                // The tie_breaker is used to assign a larger weight to the h_cost and favour expanding nodes closer towards the goal
                // f_cost = g_cost (cost to come) + h_cost (cost to go)
                double f_cost = tent_g_cost + astar_params_.tie_breaker * cost_function(nb_node, goal_node);

                // If not in closed list: set parent and add to open list
                came_from_vt_[nb_node] = cur_node;
                open_list_vt_.put(nb_node, f_cost);
            }
        }

        num_iter++;
    }

    std::cerr   << "D" << astar_params_.drone_id << ": " <<
                    "Unable to find goal node ("
                << goal_node.x << ", " << goal_node.y 
                << ") with maximum iteration " << num_iter << std::endl;

    return false;
}

void SpaceTimeAStar::tracePath(const VCell_T& final_node)
{
    // Clear existing data structures
    path_idx_vt_.clear();
    path_pos_t_.clear();
    path_pos_.clear();

    path_idx_smoothed_t_.clear();
    path_smoothed_.clear();
    path_smoothed_t_.clear();
    path_smoothed_idx_.clear();

    // Trace back the nodes through the pointer to their parent
    VCell_T cur_node = final_node;
    while (!(cur_node == came_from_vt_[cur_node])) // Start node's parents was initially set as itself, hence this indicates the end of the path
    {
        path_idx_vt_.push_back(cur_node);
        cur_node = came_from_vt_[cur_node];
    }
    // Push back the start node
    path_idx_vt_.push_back(cur_node);

    std::reverse(path_idx_vt_.begin(), path_idx_vt_.end());

    // For each gridnode, get the position and index,
    // So we can obtain a path in terms of indices and positions
    for (size_t i = 0; i < path_idx_vt_.size(); i++)
    {
        DblPoint map_2d_pos;
        int z_cm = path_idx_vt_[i].z_cm;
        double z_m = path_idx_vt_[i].z_m;

        // Convert to map position
        dyn_voro_arr_[z_cm]->idxToPos(IntPoint(path_idx_vt_[i].x, path_idx_vt_[i].y), map_2d_pos);
        // Add space time map position to path 
        path_pos_.push_back(
            Eigen::Vector3d{map_2d_pos.x, map_2d_pos.y, z_m});
        path_pos_t_.push_back(
            Eigen::Vector4d{map_2d_pos.x, map_2d_pos.y, z_m, (double) path_idx_vt_[i].t});

    }

    // /* Get smoothed path */

    // For each gridnode, get the position and index,
    // So as to get a path in terms of indices and positions
    path_idx_smoothed_t_.push_back(path_idx_vt_[0]);
    path_smoothed_idx_.push_back(0);
    for (size_t i = 1; i < path_idx_vt_.size()-1; i++)
    {
        // If different height
        if (path_idx_smoothed_t_.back().z_cm != path_idx_vt_[i].z_cm){ 
            // Add current point to smoothed path
            path_idx_smoothed_t_.push_back(path_idx_vt_[i]);
            path_smoothed_idx_.push_back(i);
        }
        // Else check for line of sight
        else {
            IntPoint a(path_idx_smoothed_t_.back().x, path_idx_smoothed_t_.back().y);
            IntPoint b(path_idx_vt_[i].x, path_idx_vt_[i].y);

            if (!lineOfSight(a, b, path_idx_smoothed_t_.back().z_cm )){ // If no line of sight
                // Add current point to smoothed path
                path_idx_smoothed_t_.push_back(path_idx_vt_[i]);
                path_smoothed_idx_.push_back(i);
            }
        }
    }
    // add goal
    path_idx_smoothed_t_.push_back(path_idx_vt_.back());
    path_smoothed_idx_.push_back(path_idx_vt_.size()-1);

    // For each gridnode, get the position and index,
    // So we can obtain a path in terms of indices and positions
    for (const VCell_T& cell : path_idx_smoothed_t_)
    {
        DblPoint map_2d_pos;

        // Convert to map position
        dyn_voro_arr_[cell.z_cm]->idxToPos(IntPoint(cell.x, cell.y), map_2d_pos);
        // Add space time map position to path and transform it from local map to world frame
        path_smoothed_t_.push_back(
            Eigen::Vector4d{map_2d_pos.x, map_2d_pos.y, cell.z_m, (double) cell.t});
        path_smoothed_.push_back(
            Eigen::Vector3d{map_2d_pos.x, map_2d_pos.y, cell.z_m});
    }
}

} // namespace global_planner