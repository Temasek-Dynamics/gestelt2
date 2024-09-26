# voronoi_planner
This package contains a voronoi planner which:
1. Performs discrete voronoi tessellation of a 2D map slice 
2. Does Space Time A* search on the resulting discrete voronoi diaram across 3D space

# Components
1. Voxel Map
    - Input:
        - Point cloud and odometry from sensors
    - Output:
        - 2D slices of the map
2. Planner (Provide front-end path)
    - Input:
        - Discretized voronoi diagram
        - Start and goal points
    - Output:
        - Front-end/Global path
3. Navigator (Overarching planner)
    - Input:
        - User-defined goals
        - Other vehicle's trajectories
    - Output:
        - Front-end/Global path

# ROS Topics
1. Voxel Map
    - Function:
        - Provide 3d occupancy grid given point cloud sensor data
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
2. Voronoi Planner 
    - Function:
        - Plan front-end path using Space-Time A* planner
        - Query Voxel Map while planning
    - Publishers:
        - `/fe_plan/broadcast` <gestelt_interfaces::msg::SpaceTimePath>
            - Front-end plan broadcasted to other agents
        - `~/fe_plan`
            - Front-end plan provided to controller 
    - Subscribers:
        - `~/odom` <nav_msgs::msg::Odometry>
            - Provides current position and velocity of agent
        - `/fe_plan/broadcast` <gestelt_interfaces::msg::SpaceTimePath>
            - Front-end plans of other agents used to construct reservation table for space-time A*
        - `~/plan_request_dbg` <gestelt_interfaces::msg::PlanRequest>
            - Sends a plan request with specified start and goal.
        - `~/goals` <gestelt_interfaces::msg::Goals> 
            - Individually published goals
3. Trajectory Server 
    - Function:
        - Send commands to FCU
    - Publishers:
        - `~/odom` <nav_msgs::msg::Odometry>
            - Provides current position and velocity of agent
        - `~/fmu/in/vehicle_command` <px4_msgs::msg::VehicleCommand>
            - Set vehicle mode
        - `~/fmu/in/offboard_control_mode` <px4_msgs::msg::OffboardControlMode>
            - Set offboard mode (trajectory, attitude, rates, thrust_torque, actuator)
        - `~/fmu/in/trajectory_setpoint` <px4_msgs::msg::TrajectorySetpoint>
            - Publish trajectory setpoints: Position, Velocity, Acceleration, Jerk
        - `~/fmu/in/actuator_motors` <px4_msgs::msg::ActuatorMotors>
            - Publish actuator commands directly
        - `~/fmu/in/vehicle_torque_setpoint` <px4_msgs::msg::VehicleTorqueSetpoint>
            - Publish torque setpoints
        - `~/fmu/in/vehicle_thrust_setpoint` <px4_msgs::msg::VehicleThrustSetpoint>
            - Publish thrust setpoints
    - Subscribers:
        - `~/fmu/out/vehicle_odometry` <nav_msgs::msg::VehicleOdometry>
            - Subscribe to vehicle odometry from FCU.
        - `~/fmu/out/vehicle_status` <nav_msgs::msg::VehicleStatus>
            - Subscribe to vehicle status from FCU. Used to check vehicle mode (offboard, landing etc.) and arming state.

# Acknowledgements
1. [dynamicvoronoi code from Boris Lau](http://www2.informatik.uni-freiburg.de/~lau/dynamicvoronoi/)
