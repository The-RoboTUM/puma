# PUMA

### 2026.1.30
### 

### Runbook (Latest Version)
#### Prerequisites

- All required packages are already installed
- If not, please first check:
  - **🧾 Update Logs – [branch: `Pedro's Notes`] Jan 24**
  - **🧾 Update Logs – [branch: `Pedro's Notes`] Jan 26**


#### 🔁 Runbook (Repeat Every Time)


#### 2.1 Configure Host PC Ethernet IP (**Mandatory**)

> The robot Ethernet port does **not** provide DHCP.  
> The host PC **must** use a static IP.

##### Configure the Ethernet interface

```bash
sudo ip addr flush dev enp7s0
sudo ip addr add 10.21.31.50/24 dev enp7s0
ip a show enp7s0
```

#### Verify Connectivity to the Robot

```bash
ping -c 2 10.21.31.106
ping -c 2 10.21.31.104
```

Both IPs must be reachable.


#### Ensure Routing Goes via Ethernet (Not Wi-Fi)

If the host is connected to **both Wi-Fi and Ethernet**, traffic to  
`10.21.31.0/24` **must go through `enp7s0`**.

Check routes:

```bash
ip route | grep 10.21.31
```

If the subnet is routed via Wi-Fi (e.g. `wlp8s0`), remove it:

```bash
sudo ip route del 10.21.31.0/24 via <WIFI_GW> dev wlp8s0
```
(Replace `<WIFI_GW>` with the actual Wi-Fi gateway.)


#### 2.2 Start Multicast Relay on the Robot (10.21.31.106)

> This step is executed **on the robot**

```bash
ssh user@10.21.31.106
```

Enable and start the multicast relay service:

```bash
sudo systemctl enable multicast-relay.service
sudo systemctl start multicast-relay.service
sudo systemctl status multicast-relay.service --no-pager
```

Expected output:

```text
Active: active (running)
```
#### 2.3 Verify Multicast UDP Reaches the Host (**Highly Recommended**)

> ⚠️ If this step fails, **do NOT debug ROS yet**

On the **host PC**, run:

```bash
sudo tcpdump -ni enp7s0 -nn 'udp and (port 6691 or port 6692)' -c 20
```

Expected packets:

```text
10.21.31.106.xxxx > 224.10.10.201.6691: UDP, length 1248
10.21.31.106.xxxx > 224.10.10.202.6692: UDP, length 1248
```

If **no packets appear**:

- Network configuration is incorrect
- Multicast relay is not working
- Routing is wrong

Fix networking first. Do **not** proceed to ROS debugging.


#### 2.4 Start `rslidar_sdk` on the Host  
##### (Decode UDP → ROS2 PointCloud2)

##### Enter the rslidar workspace

```bash
cd ~/.../ws_rslidar
```

##### Source ROS2 and the workspace

###### Source-built workspace

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

###### (Optional) Verify the source-built driver is used

```bash
ros2 pkg prefix rslidar_sdk
```

It should point to:

```text
.../ws_rslidar/install/rslidar_sdk
```

Not:

```text
/opt/ros/...
```

##### Run the rslidar SDK node

```bash
ros2 run rslidar_sdk rslidar_sdk_node \
  --ros-args \
  --params-file /home/robotum/Documents/RoboTUM_ws/puma/robot_dds/rslidar_rosparams.yaml
```

##### Expected ROS topics

```text
/rslidar_points_6691
/rslidar_points_6692
```

##### Verify point cloud frequency (new terminal)

```bash
ros2 topic hz /rslidar_points_6691
ros2 topic hz /rslidar_points_6692
```

In our setup, the frequency is approximately **~9 Hz**.


#### 2.5 Start the Puma System (Bringup)

```bash
source /opt/ros/jazzy/setup.bash
source ~/Documents/RoboTUM_ws/puma/puma_ros2_ws/install/setup.bash

ros2 launch puma_nodes puma_bringup.launch.py
```

#### 2.6 Start Keyboard Teleoperation

```bash
source /opt/ros/jazzy/setup.bash
source ~/Documents/RoboTUM_ws/puma/puma_ros2_ws/install/setup.bash

ros2 run puma_nodes teleop_keyboard
```

#### 2.7 Start RViz Visualization

```bash
source /opt/ros/jazzy/setup.bash
source ~/Documents/RoboTUM_ws/puma/puma_ros2_ws/install/setup.bash

rviz2
```

- Fixed Frame: `map` or `puma_base_link`
##### 2.7.1 Add an LiDAR Visualization

1. In the left panel, click **Add**
2. Select **PointCloud2**
3. Click **OK**

- **PointCloud2 → Topic**  
  Select:
```bash
    rslidar_points_6691_front
```
- **PointCloud2 → Topic**  
```bash
    rslidar_points_6692_rear
```

##### 2.7.2 Add an Image Display

1. In the left panel, click **Add**
2. Select **Image**
3. Click **OK**


 Configure the Image Display:

- **Image → Topic**  
  Select:


```bash
/camera/front/image_raw
```

⚠️ If the topic field is empty, RViz will show:
```text
Error subscribing: Empty topic name
```

##### Note: Set the Correct QoS Policy (Important)

- **Image → Reliability Policy**

```text
Best Effort
```


##### 📌 Many camera streams use `Best Effort` for lower latency.  

If RViz is set to `Reliable`, it may fail to receive images.

---