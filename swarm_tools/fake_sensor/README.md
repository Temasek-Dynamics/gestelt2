# fake_sensor
Given a point cloud file and the drone's current position, `fake_sensor` simulates a mounted depth camera (or 3d Lidar) on the drone. This is done by taking in the pose of the drone and outputting a point cloud of the map in the sensor frame.

# Acknowledgements
The point cloud sensor simulator (named as `fake_laser`) is has been modified from the drone simulator in ZheJiang University FAST Lab