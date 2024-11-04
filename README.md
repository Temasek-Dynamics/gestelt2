# Gestelt2
A Receding Horizon Planning (RHP) framework with a focus on multi-UAV navigation in cluttered environments. 

# Architecture
<img src="docs/pictures/gestelt_architecture_24_10.png" alt="Gestelt Architecture" style="width: 1200px;"/>

# Installation and Setup for Simulation

1. Dependencies and Required Repos
- Ubuntu 24.04 
- ROS2 Jazzy
- eProsima/Micro-XRCE-DDS-Agent: v2.4.3
- PX4-msgs: bcb3d020bd2f2a994b0633a6fccf8ae47190d867
- PX4-Autopilot: 3d36c8519de83afd7b4617c3496d0304fb17cc28

2. Install ROS2 and associated dependencies
- ROS 2 Jazzy
```bash
# Install ROS2 at https://docs.ros.org/en/jazzy/Installation.html 

# Install ROS2 dependencies
sudo apt install -y ros-jazzy-pcl-ros ros-jazzy-pcl-conversions ros-jazzy-message-filters
```

3. Install other dependencies
- XRCE DDS installation
```bash
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git --recursive -b v2.4.3
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

- PX4 Firmware
```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive 
cd ~/PX4-Autopilot
git checkout 3d36c8519de83afd7b4617c3496d0304fb17cc28
bash ./Tools/setup/ubuntu.sh 
# Make SITL target for simulation
DONT_RUN=1 make px4_sitl 

# Copy the custom drone model over
cp -r ~/gestelt_ws/src/Gestelt2/gestelt_bringup/models/nuswarm ~/PX4-Autopilot/Tools/simulation/gz/models

# [FOR EMERGENCY USE] IF you screw up the PX4 Autopilot build at any point, clean up the build files via the following command:
make distclean
```

4. Additional ROS packages
```bash
cd ~/gestelt_ws/src
# px4_msgs
git clone  --recursive https://github.com/PX4/px4_msgs.git
cd px4_msgs 
git checkout bcb3d020bd2f2a994b0633a6fccf8ae47190d867
```

5. Building the workspace
```bash
# Assuming your workspace is named as follows
cd ~/gestelt_ws/
colcon build
```

6. Additional checks
```bash
# Check if px4_msg definitions match those in PX4 Firmware
./src/px4-ros2-interface-lib/scripts/check-message-compatibility.py -v ./src/px4_msgs/ ../PX4-Autopilot/
```

# Quick start
