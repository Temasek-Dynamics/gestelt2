# pcd_map_generator
Generates a point cloud map used for testing or creating virtual obstacle environments.

# Usage
1. Edit the parameters in [swarm_tools/pcd_map_generator/config/pcd_map_gen.yaml](swarm_tools/pcd_map_generator/config/pcd_map_gen.yaml). Be sure to set the filename parameter. 

2. Run the node
```bash
ros2 launch pcd_map_generator pcd_map_gen.py
```

3. PCD file of generated virtual map is saved to [gestelt_bringup/pcd_maps](gestelt_bringup/pcd_maps)