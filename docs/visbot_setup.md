# vistbot_setup

1. Upgrade Cmake
```bash
wget https://github.com/Kitware/CMake/releases/download/v3.31.3/cmake-3.31.3.tar.gz
tar xf cmake-3.31.3.tar.gz
cd cmake-3.31.3
./configure
sudo make install
cmake --version
```

2.  XRCE DDS installation
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

3. Test MicroXRCEAgent
```bash
sudo MicroXRCEAgent serial --dev /dev/ttyS7 -b 921600
```

    sudo MicroXRCEAgent serial --dev /dev/ttyS2 -b 921600
    sudo MicroXRCEAgent serial --dev /dev/ttyS9 -b 921600

## Communications

Drone 6 IP: 

1. GCS connection
    - Look at `VisBot_Drone/visbot_monitor_ws/bringup/scripts/visbot_update/config_update/owl3/owl3_customized/owl3_config.yaml` 
    - Determine what is the GCS UDP port dependending on the drone_id. If drone_id is 6, then set gcs udp port is 14556
    - Add the connection to QGroundControl. More details can be found on the visbot_drone README

2. XRCE Client Connection
    - uxrce_dds_client start -t serial -d /dev/ttyS6 -b 921600
    - uxrce_dds_client start -t serial -d /dev/ttyS1 -b 921600
    - [Reference](https://docs.px4.io/main/en/modules/modules_system.html#uxrce-dds-client)

3. Mavlink Connection
    - `mavlink status streams`

3. FCU Ports
    - TELEM1: /dev/ttyS6 (FCU) <-> /dev/ttyS7 (OBC)
    - TELEM2: UDP 
    - TELEM3: /dev/ttyS1 (FCU) <->
    - ETHERNET: USB

<!-- 
模块输出的主要 ROS topic：
➢/mavros/imu/data_raw
➢/mavros/local_position/odom: 飞控融合后无人机位置
➢/mavros/state ：PX4 状态
模块主要的输入 ROS topic：
➢/mavros/vision_pose/pose ： VIO 定位信息
➢/mavros/setpoint_raw/local ：目标点
➢/mavros/setpoint_position/local ：目标点
➢/mavros/set_mode ： PX4 状态控制 -->


instance #0:
    GCS heartbeat valid
    mavlink chan: #0
    type:        GENERIC LINK OR RADIO
    flow control: ON
    rates:
      tx: 17992.1 B/s
      txerr: 0.0 B/s
      tx rate mult: 1.000
      tx rate max: 115200 B/s
      rx: 10103.7 B/s
      rx loss: 0.0%
    Received Messages:
      sysid:  1, compid:240, Total: 527477 (lost: 2)
        msgid:   84, Rate:  nan Hz, last 0.01s ago
      sysid:255, compid:190, Total: 87816 (lost: 0)
    FTP enabled: YES, TX enabled: YES
    mode: Normal
    Forwarding: On
    MAVLink version: 2
    transport protocol: serial (/dev/ttyS6 @921600)
    ping statistics:
      last: 16.57 ms
      mean: 16.75 ms
      max: 2225.90 ms
      min: 4.52 ms
      dropped packets: 1

instance #1:
    mavlink chan: #1
    type:        GENERIC LINK OR RADIO
    flow control: OFF
    rates:
      tx: 1494.0 B/s
      txerr: 0.0 B/s
      tx rate mult: 1.000
      tx rate max: 5760 B/s
      rx: 1119.8 B/s
      rx loss: 0.0%
    Received Messages:
      sysid:  1, compid:  1, Total: 413996 (lost: 0)
    FTP enabled: YES, TX enabled: YES
    mode: Normal
    Forwarding: Off
    MAVLink version: 1
    transport protocol: serial (/dev/ttyS1 @115200)

instance #2:
    mavlink chan: #2
    type:        GENERIC LINK OR RADIO
    flow control: OFF
    rates:
      tx: 0.0 B/s
      txerr: 52.9 B/s
      tx rate mult: 0.050
      tx rate max: 100000 B/s
      rx: 0.0 B/s
      rx loss: 0.0%
    FTP enabled: YES, TX enabled: YES
    mode: Normal
    Forwarding: Off
    MAVLink version: 1
    transport protocol: UDP (14550, remote port: 14550)
    Broadcast enabled: YES