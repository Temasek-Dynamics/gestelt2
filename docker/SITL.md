# Instructions for SITL simulation

docker run -it --rm --privileged --network host  gestelt/px4_sitl_gz

ros2 launch gestelt_bringup mavros_sitl.py

ros2 launch gestelt_bringup execute_mission_single.py

ros2 launch gestelt_bringup gcs.py