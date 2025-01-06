# Gestelt2
A Receding Horizon Planning (RHP) framework with a focus on multi-UAV navigation in cluttered environments. 

# Architecture 

The architecture below illustrates the high-level architecture of Gestelt using MAVROS as a communication bridge between the flight controller unit and the onboard computer.
<img src="docs/pictures/gestelt2_arch.png" alt="Gestelt Architecture" style="width: 1200px;"/>

# Installation and Setup for Simulation

Dependencies:
- System
    - Ubuntu 24.04 (Noble)
    - ROS2 Jazzy
- Communications
    - eProsima/Micro-XRCE-DDS-Agent: Tag `v2.4.3`
    - PX4-msgs: Commit `bcb3d020bd2f2a994b0633a6fccf8ae47190d867`
- Simulation 
    - PX4-Autopilot: Commit `3d36c8519de83afd7b4617c3496d0304fb17cc28`
    - mavros: Commit `b49095727a6ff160e1e913b90a4ce50e383e8863`

2. Install ROS2 and associated dependencies
- ROS 2 Jazzy
```bash
# Install ROS2 at https://docs.ros.org/en/jazzy/Installation.html 

# Install external package dependencies
sudo apt install -y libeigen3-dev build-essential python3-vcstool tmux

# Install ROS2 Package dependencies
sudo apt install -y ros-jazzy-pcl-ros ros-jazzy-pcl-conversions ros-jazzy-message-filters
```

3. Clone additional dependencies 
```bash
cd ~/gestelt_ws/src/gestelt2
vcs import < simulation.repos --recursive
vcs import < thirdparty.repos --recursive
```

4. Building the workspace
```bash
# Assuming your workspace is named as follows
cd ~/gestelt_ws/ && colcon build --symlink-install
```

5. (OPTIONAL FOR PX4 SITL Simulation) Build PX4-autopilot 
```bash
cd ~/gestelt_ws/PX4-Autopilot/
bash ./Tools/setup/ubuntu.sh 
# Make SITL target for simulation
DONT_RUN=1 make px4_sitl 
```

6. (OPTIONAL FOR Micro-XCRE DDS) Install dependencies for communication with FCU 

(a) XRCE DDS installation
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

(b) Check if px4_msg definitions match those in PX4 Firmware
```bash
./src/px4-ros2-interface-lib/scripts/check-message-compatibility.py -v ./src/px4_msgs/ ../PX4-Autopilot/
```

# Quick start

To enable repeatability of experiments. We make use of scenarios which are configurations of drone spawn locations and environments stored in [gestelt_mission/scenarios.json](gestelt_mission/scenarios.json). Refer to [gestelt_mission/README.md](gestelt_mission/README.md) for more information.

## With mock drones
To run a simulation without a dynamical model i.e. to test the path planning logic.
```bash
ros2 launch gestelt_bringup multi_fake_drones.py
``` 

## With PX4-SITL 
```bash
ros2 launch gestelt_bringup multi_sitl_drones.py
```
