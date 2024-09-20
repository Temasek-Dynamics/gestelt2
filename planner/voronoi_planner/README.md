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
# Acknowledgements
1. [dynamicvoronoi code from Boris Lau](http://www2.informatik.uni-freiburg.de/~lau/dynamicvoronoi/)
