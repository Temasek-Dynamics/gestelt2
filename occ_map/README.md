# occ_map
Occupancy map node whose main function is to generate a 3d occupancy grid given point cloud data as input. The occupancy map is used by the PlannerServer and ControllerServer in checking for obstacles.

# ROS Topics

- Publishers:
    - `~/occ_map` <sensor_msgs::msg::PointCloud2>
        - Occupancy map
    - `local_map/bounds` <geometry_msgs::msg::PolygonStamped>
        - Local map boundary. Represents the planning horizon
- Subscribers:
    - `~/odom` <nav_msgs::msg::Odometry>
        - Provides current position and velocity of agent
    - `~/cloud` <sensor_msgs::msg::PointCloud2>
        - Point cloud used for map construction