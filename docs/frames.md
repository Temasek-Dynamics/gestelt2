# Frames

[GLOBAL FRAME] -> [MAP FRAME] -> [LOCAL MAP FRAME]
- [GLOBAL FRAME]: Fixed global frame i.e. "world"
- [MAP FRAME]: Fixed drone origin frame i.e. "d0_origin", "d1_origin" etc.
- [LOCAL MAP FRAME]: Moving local map frame (changes with drone's current position relative to [MAP FRAME]) i.e. "d0_lcl_map" 
- Example: ["world"] -> ["d0_origin"] -> ["d0_lcl_map"]

# Modules
- Global Planner
    - SUBSCRIPTIONS
        - [`GLOBAL frame`] start/goal 
            - Convert to [`LOCAL MAP FRAME`] before feeding to planner
        - [`MAP frame`] Odometry
    - PUBLICATIONS
        - [`MAP FRAME`] Minimum Jerk Trajectory
        - [`MAP FRAME`] Front-end plan
- Voxel_map
    - SUBSCRIPTIONS
        - [`MAP frame`] Odometry
    - PUBLICATIONS
        - [`LOCAL MAP FRAME`] Occupancy map visualization
    - TF PUBLICATIONS
        - [`MAP frame`] -> [`LOCAL MAP FRAME`]
- Trajectory Server
    - SUBSCRIPTIONS
        - [`MAP frame`] Minimum Jerk Trajectory
    - PUBLICATIONS
        - [`MAP frame`] Odometry
        - [`MAP frame`] Trajectory setpoints
