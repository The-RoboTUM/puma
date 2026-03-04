# PUMA

### 2026.March.4
### 

### PUMA Robot Operation Guide
**Onboard LiDAR + Camera System + PTZ Camera**

Environment:

- **ROS2 Jazzy**
- **Ubuntu 24.04**

This guide explains how to start the **PUMA robotic system**, including:

- Onboard **LiDAR sensors** (Front + Rear)
- Onboard **cameras** (Front + Rear)
- External **PTZ camera** (RGB + Thermal)
- **Robot control system**
- **RViz2 visualization**

---

### 1. Network Architecture

The system uses **two independent networks**.

| Device | Subnet | Network Interface |
|------|------|------|
| PUMA robot & onboard LiDAR | 10.21.31.0/24 | enp7s0 |
| PTZ camera | 192.168.1.0/24 | USB-Ethernet (enx...) |

Therefore the host machine must configure **two network interfaces simultaneously**.


### 2. System Environment

Required environment:

- Ubuntu **24.04**
- **ROS2 Jazzy**
- Python3
- OpenCV
- cv_bridge



#### ROS2 Workspaces

Main workspace:

```
~/Documents/RoboTUM_ws/puma/puma_ros2_ws
```

LiDAR workspace:

```
~/.../ws_rslidar
```



### 3. Runbook (Required Every Time)

The following steps **must be executed every time the robot system starts**.



#### 3.1 Configure Robot Network (LiDAR + Onboard Cameras)

The robot Ethernet **does not provide DHCP**, so a **static IP** must be configured.

Configure Ethernet:

```bash
sudo ip addr flush dev enp7s0
sudo ip addr add 10.21.31.50/24 dev enp7s0
sudo ip link set enp7s0 up
```

Check the IP address:

```bash
ip a show enp7s0
```


##### Verify Robot Connectivity

```bash
ping -c 2 10.21.31.106
ping -c 2 10.21.31.104
ping -c 2 10.21.31.103
```

All three addresses **should respond successfully**.


##### Ensure Traffic Uses Ethernet (Not Wi-Fi)

If the computer is connected to **both Wi-Fi and Ethernet**, check the routing table:

```bash
ip route | grep 10.21.31
```

If you see:

```
dev wlp8s0
```

this means traffic is going through Wi-Fi.

Delete that route:

```bash
sudo ip route del 10.21.31.0/24 via <WIFI_GW> dev wlp8s0
```

#### 3.2 Configure PTZ Camera Network

The PTZ camera uses a **separate subnet**.

Configure the USB-Ethernet interface:

```bash
sudo ip addr flush dev enx001122680b24
sudo ip addr add 192.168.1.100/24 dev enx001122680b24
sudo ip link set enx001122680b24 up
```

Test connectivity:

```bash
ping 192.168.1.108
```

If the ping succeeds, the **PTZ network is working properly**.



#### 3.3 Start Robot Multicast Relay

This step must be executed **on the robot**.

Connect to the robot:

```bash
ssh user@10.21.31.106
```

Enable and start the service:

```bash
sudo systemctl enable multicast-relay.service
sudo systemctl start multicast-relay.service
```

Check service status:

```bash
sudo systemctl status multicast-relay.service --no-pager
```

Expected output:

```
Active: active (running)
```


#### 3.4 Verify LiDAR UDP Packets (Critical Step)

⚠️ **If this step fails, do NOT start debugging ROS.**

Run on the host machine:

```bash
sudo tcpdump -ni enp7s0 -nn 'udp and (port 6691 or port 6692)' -c 20
```

Expected output:

```
10.21.31.106.xxxx > 224.10.10.201.6691: UDP
10.21.31.106.xxxx > 224.10.10.202.6692: UDP
```

If no packets appear, possible causes include:

- Incorrect network configuration
- Multicast relay not running
- Routing errors

These **must be fixed before starting ROS**.


### 4. Start LiDAR Driver

Go to the LiDAR workspace:

```bash
cd ~/.../ws_rslidar
```

Load ROS2 environment:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

Check the driver source:

```bash
ros2 pkg prefix rslidar_sdk
```

It should show:

```
.../ws_rslidar/install/rslidar_sdk
```

and **not**

```
/opt/ros/...
```


#### Run the LiDAR Driver

```bash
ros2 run rslidar_sdk rslidar_sdk_node \
  --ros-args \
  --params-file /home/robotum/Documents/RoboTUM_ws/puma/robot_dds/rslidar_rosparams.yaml
```


#### Verify LiDAR Topics

```bash
ros2 topic list
```

Expected topics:

```
/rslidar_points_6691
/rslidar_points_6692
```

Check the frequency:

```bash
ros2 topic hz /rslidar_points_6691
ros2 topic hz /rslidar_points_6692
```

Expected frequency:

```
~9 Hz
```


### 5. Build the ROS2 Workspace

Go to the workspace:

```bash
cd ~/Documents/RoboTUM_ws/puma/puma_ros2_ws
```

Load ROS2:

```bash
source /opt/ros/jazzy/setup.bash
```

Build the workspace:

```bash
colcon build --symlink-install
```

Source the workspace:

```bash
source install/setup.bash
```


### 6. Launch the PUMA System

Load environments:

```bash
source /opt/ros/jazzy/setup.bash
source ~/Documents/RoboTUM_ws/puma/puma_ros2_ws/install/setup.bash
```

Launch the system:

```bash
ros2 launch puma_nodes puma_bringup.launch.py
```

This launch file automatically starts:

- PUMA control system
- Onboard front camera
- Onboard rear camera
- TF tree
- Basic robot nodes


### 7. Start PTZ Camera Image Publisher

```bash
source /opt/ros/jazzy/setup.bash
cd ~/Documents/RoboTUM_ws/puma/puma_ros2_ws/
source install/setup.bash
```

Run:

```bash
ros2 run puma_nodes ptz_camera_image_publisher.py
```

Published topics:

| Camera | Topic |
|------|------|
| PTZ RGB | `/ptz/rgb/image_raw` |
| PTZ Thermal | `/ptz/thermal/image_raw` |


### 8. PTZ Camera Control

Run the keyboard control script:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/Documents/RoboTUM_ws/puma/puma_ros2_ws/
source install/setup.bash

ros2 run puma_nodes ptz_camera_keyboard
```

Controls:

- Pan
- Tilt
- Zoom

### 9. Robot Keyboard Control

```bash
ros2 run puma_nodes teleop_keyboard
```


### 10. RViz Visualization

Start RViz:

```bash
rviz2
```

Set:

`Fixed Frame` to: `map` or `puma_base_link`

#### 10.1 Add LiDAR Point Clouds

```
Add → PointCloud2
```

Topics:

```
rslidar_points_6691_front
rslidar_points_6692_rear
```

#### 10.2 Add Camera Images

```
Add → Image
```

Topics:

```
/ptz/rgb/image_raw
/ptz/thermal/image_raw
/camera/front/image_raw
/camera/rear/image_raw
```

### 11. QoS Settings (Important)

Different cameras use **different QoS settings**.


#### Onboard Cameras

Topics:

```
/camera/front/image_raw
/camera/rear/image_raw
```

Must use:

```
Best Effort
```

RViz setting:

```
Image → Reliability Policy → Best Effort
```

If **Reliable** is used, images usually **will not appear**.




#### PTZ Cameras

Topics:

```
/ptz/rgb/image_raw
/ptz/thermal/image_raw
```

These can use:

```
Reliable
```

However:

- Higher latency
- Lower real-time performance

For **lowest latency streaming**, it is still recommended to use:

```
Best Effort
```


### 12. Debugging Commands

Check topic QoS:

```bash
ros2 topic info /camera/front/image_raw -v
```

This command displays:

- Publisher QoS
- Subscriber QoS
- Reliability policy

Useful for diagnosing **QoS mismatch issues**.



