# gestelt_bringup

Contains bringup files for ROS2 nodes

# Bringup Hierarchy 

## 'launch/gcs'

Bringup nodes for ground control station

- `test_take_off_point_goal.py`   
    - Takes off and enters mission mode. The user can then specify planning goals by using the RVIZ Goal tool. This launches the python script written in the `gestelt_commander` package.
- `record_rosbag.py`
    - Used to record rosbags from actual drone flight. Modify the launch file to select topics to record.
- `play_rosbag.py`
    - Play a ROSBag and visualize in RVIZ. Modifyt the launch file to select specific ROS Bags.

## 'launch/offboard' folder

Bringup nodes for offboard control on actual drones.

- `offboard_launch.py`
    - Launches an instance of `bringup_launch.py` which contains the PlannerServer and ControllerServer
- `vilota_launch.py`
    - Launches the eCAL bridge for the vilota sensor as well as disparity map to point cloud converter 
- `cam_tf_viz.py`
    - Used to visualize the vilota camera transforms.
- `rviz_viz.py`
    - Runs RVIZ Visualization

## 'launch/sim'

Bringup nodes for ground control station

- `multi_drone_sim_launch.py`
    - Reads a given scenario from the `gestelt_commander/scenarios.json` file and spawns up a given number of instances of `bringup_sim_launch.py`.
- `test_point_goal_sim.py`
    - Takes off and enters mission mode. The user can then specify planning goals by using the RVIZ Goal tool. This launches the python script written in the `gestelt_commander` package.
