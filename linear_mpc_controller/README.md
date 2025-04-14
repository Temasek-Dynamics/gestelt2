# Linear MPC Controller

Linear MPC Controller with Safe Flight Corridor implemented as a controller plugin.

## Dependencies
- Solvers
    - [OSQP](https://osqp.org/docs/index.html)
    - [OSQP-Eigen](https://github.com/robotology/osqp-eigen)

Installation: 
```bash
# OSQP
git clone https://github.com/osqp/osqp.git
cd osqp 
mkdir build 
cd build 
cmake -G "Unix Makefiles" .. 
sudo cmake --build . --target install

# OSQP-Eigen
git clone https://github.com/robotology/osqp-eigen.git 
cd osqp-eigen 
mkdir build 
cd build 
cmake ../ 
make 
sudo make install 

# Add directory to bash
echo 'export OsqpEigen_DIR=/usr/local/lib/cmake/OsqpEigen/' >> ~/.bashrc 
```

## Configuration

| Parameter | Description | 
|-----|----|
| `desired_linear_vel` | The desired maximum linear velocity to use. | 
| `lookahead_dist` | The lookahead distance to use to find the lookahead point | 
| `min_lookahead_dist` | The minimum lookahead distance threshold when using velocity scaled lookahead distances | 
| `max_lookahead_dist` | The maximum lookahead distance threshold when using velocity scaled lookahead distances | 
| `lookahead_time` | The time to project the velocity by to find the velocity scaled lookahead distance. Also known as the lookahead gain. | 
| `rotate_to_heading_angular_vel` | If rotate to heading is used, this is the angular velocity to use. | 
| `transform_tolerance` | The TF transform tolerance | 
| `use_velocity_scaled_lookahead_dist` | Whether to use the velocity scaled lookahead distances or constant `lookahead_distance` | 
| `min_approach_linear_velocity` | The minimum velocity threshold to apply when approaching the goal | 
| `approach_velocity_scaling_dist` | Integrated distance from end of transformed path at which to start applying velocity scaling. This defaults to the forward extent of the costmap minus one costmap cell length. | 
| `use_collision_detection` | Whether to enable collision detection. |
| `max_allowed_time_to_collision_up_to_carrot` | The time to project a velocity command to check for collisions when `use_collision_detection` is `true`. It is limited to maximum distance of lookahead distance selected. |
| `use_regulated_linear_velocity_scaling` | Whether to use the regulated features for curvature | 
| `use_cost_regulated_linear_velocity_scaling` | Whether to use the regulated features for proximity to obstacles | 
| `cost_scaling_dist` | The minimum distance from an obstacle to trigger the scaling of linear velocity, if `use_cost_regulated_linear_velocity_scaling` is enabled. The value set should be smaller or equal to the `inflation_radius` set in the inflation layer of costmap, since inflation is used to compute the distance from obstacles | 
| `cost_scaling_gain` | A multiplier gain, which should be <= 1.0, used to further scale the speed when an obstacle is within `cost_scaling_dist`. Lower value reduces speed more quickly. | 
| `inflation_cost_scaling_factor` | The value of `cost_scaling_factor` set for the inflation layer in the local costmap. The value should be exactly the same for accurately computing distance from obstacles using the inflated cell values | 
| `regulated_linear_scaling_min_radius` | The turning radius for which the regulation features are triggered. Remember, sharper turns have smaller radii | 
| `regulated_linear_scaling_min_speed` | The minimum speed for which the regulated features can send, to ensure process is still achievable even in high cost spaces with high curvature. | 
| `use_rotate_to_heading` | Whether to enable rotating to rough heading and goal orientation when using holonomic planners. Recommended on for all robot types except ackermann, which cannot rotate in place. | 
| `rotate_to_heading_min_angle` | The difference in the path orientation and the starting robot orientation to trigger a rotate in place, if `use_rotate_to_heading` is enabled. | 
| `max_angular_accel` | Maximum allowable angular acceleration while rotating to heading, if enabled | 
| `max_robot_pose_search_dist` | Maximum integrated distance along the path to bound the search for the closest pose to the robot. This is set by default to the maximum costmap extent, so it shouldn't be set manually unless there are loops within the local costmap. | 
| `use_interpolation` | Enables interpolation between poses on the path for lookahead point selection. Helps sparse paths to avoid inducing discontinuous commanded velocities. Set this to false for a potential performance boost, at the expense of smooth control. | 


## Topics

| Topic  | Type | Description | 
|-----|----|----|
| `lookahead_point`  | `geometry_msgs/PointStamped` | The current lookahead point on the path | 
| `lookahead_arc`  | `nav_msgs/Path` | The drivable arc between the robot and the carrot. Arc length depends on `max_allowed_time_to_collision_up_to_carrot`, forward simulating from the robot pose at the commanded `Twist` by that time. In a collision state, the last published arc will be the points leading up to, and including, the first point in collision. | 

Note: The `lookahead_arc` is also a really great speed indicator, when "full" to carrot or max time, you know you're at full speed. If 20% less, you can tell the robot is approximately 20% below maximum speed. Think of it as the collision checking bounds but also a speed guage.

## Notes to users
