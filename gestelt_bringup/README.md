# gestelt_bringup

Contains bringup files for ROS2 nodes

# Bringup Hierarchy 

## launch/gcs

Bringup nodes for ground control station

- `test_take_off_goal.py`   
    - Test take off and going to a simple goal
- `record_rosbag.py`
    - Used to record rosbags from actual drone flight
- `play_rosbag.py`
    - Play a ROSBag and visualize in RVIZ 

## launch/offboard

Bringup nodes for offboard control on actual drones.

- `offboard_launch.py`
    - Launches an instance of `bringup_launch.py` which contains the PlannerServer and ControllerServer
- `vilota_launch.py`
    - Launches the eCAL bridge for the vilota sensor as well as disparity map to point cloud converter 

## launch/sim

Bringup nodes for ground control station

- `multi_drone_sim_launch.py`
    - Reads a given scenario from the `gestelt_commander/scenarios.json` file and spawns up a given number of instances of `bringup_sim_launch.py`.
