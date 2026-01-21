# Pedro's Notes

### 2026.1.21
### 🧾 Update Logs

### TF Drift Issue in RViz: Postmortem & Root Cause Analysis

#### Observed Behavior

**Initially**
- Even when the robot was stopped, or only moving forward/backward, the `base_link` in RViz slowly drifted.

**Later**
- After introducing a cut-off (deadband), the drift stopped once forward/backward motion ended.

**However**
- As soon as `q/e` (in-place rotation) was used:
  - After releasing all inputs and stopping,
  - `base_link` started drifting again,
  - Often consistently in the same direction.


#### Root Cause (Core Insight)

The `/odom` pose is obtained by **integrating velocity**:

```text
x += v_x * dt
y += v_y * dt
```
#### Why Drift Happens

Therefore, **any non-zero velocity bias**, no matter how small, will accumulate into continuous position drift over time.

When the robot is *perceived as stationary*, the reported `LinearX / LinearY` may still contain small residual values  
(e.g. `-0.002`, `-0.004`). Over time, these values are integrated into visible drift.

**More importantly**:  
After in-place rotation, the robot’s internal state or filtering often introduces a **more stable and larger bias** in  
`LinearX / LinearY` (e.g. `-0.037 ~ -0.038` as observed).

At this magnitude:

- The deadband can no longer suppress the bias
- The bias is continuously integrated
- Resulting in **obvious and directional drift**


#### Solution Overview: Two Layers of Defense

##### Defense Layer 1: Cut-off / Deadband (Verified Effective)

Treat sufficiently small velocities as pure noise and force them to zero before integration:

```text
if |LinearX| < deadband_v        → LinearX = 0
if |LinearY| < deadband_v        → LinearY = 0
if |OmegaZ|  < deadband_yaw_rate → OmegaZ  = 0
```
This primarily solves:

- Small drift when stationary
- Drift after straight-line motion stops


##### Defense Layer 2: Intent-Gated Integration (Solves Post-Rotation Drift)

When operator input clearly indicates **pure rotation**  
(`/cmd_vel` linear ≈ 0 and angular velocity is significant):

- **Freeze translational integration**
  - Do **not** update `x / y`
  - Even if reported `LinearX / LinearY` are non-zero

When the operator resumes translational commands (`w / s / a / d`):

- **Unfreeze integration**
- Resume normal position updates

This directly addresses:

- Systematic velocity bias introduced after in-place rotation
- Drift caused by biased odometry during rotational maneuvers


#### Optional Enhancement: Continuous-Still Freeze

If the system detects **N consecutive frames** of complete stillness:

- Linear velocity = 0
- Angular velocity = 0

Then:

- **Freeze integration entirely**

This provides additional robustness against:

- Residual sensor noise
- Long-term numerical drift while idle


---

### 2026.1.20
### 🧾 Update Logs

### PUMA Onboard Camera → Host Computer RViz2 Display  
**(RTSP + ROS2 Image Pipeline)**

This Update logs explains how to verify the RTSP video streams from the **PUMA robot’s front/rear cameras** on the host computer, and how to visualize them in **RViz2** using the **Image** display.


#### 1. Background & Principle (Brief)

##### What is RTSP

**RTSP (Real Time Streaming Protocol)** is a protocol commonly used for pulling real-time video streams. It is widely used in surveillance cameras and robotic vision systems.

On the **PUMA robot**, camera images are encoded and exposed via an **RTSP server**, while the **host Computer acts as a client** to pull the stream.

**RTSP stream addresses:**

- **Front camera**

```text
rtsp://10.21.31.103:8554/video1
```

- **Rear camera**

```text
rtsp://10.21.31.103:8554/video2
```

##### Why RViz2 Cannot Play RTSP Directly

RViz2 **cannot subscribe to RTSP streams directly**.

The RViz2 **Image** display only supports ROS2 image topics:
- `sensor_msgs/Image`
- `sensor_msgs/CompressedImage`

Therefore, a **bridge node (camera bridge)** is required to convert RTSP into a ROS2 image topic.

**Data flow:**




```text
RTSP stream
↓
Decode
↓
ROS2 Image Topic (e.g. /camera/front/image_raw)
↓
RViz2 Image Display
```

#### 2. Network & Prerequisites

##### 2.1 Connect the Host Computer to the Robot Onboard Computer

You can connect using **either**:

- **WiFi**  
  Connect to the robot hotspot:


```text
SSID: CA9B_NO.XXX_5G
Password: 12345678
```

- **Wired Ethernet connection**



##### 2.2 Verify Network Connectivity

Run the following command on the host PC:

```bash
ping -c 3 10.21.31.103
```
✅ You should receive replies without packet loss.



##### 2.3 ROS2 Installation

ROS2 must be installed on the host PC.  
This project uses **ROS2 Jazzy**.

Source the ROS2 environment:

```bash
source /opt/ros/jazzy/setup.bash
```

#### 3. Verify the RTSP Stream First (Mandatory)

Before touching ROS2 or RViz2, **always verify the RTSP stream independently**.

##### Method A: ffplay (Fastest)

```bash
ffplay -fflags nobuffer -flags low_delay -rtsp_transport tcp \
rtsp://10.21.31.103:8554/video1
```

##### Method B: GStreamer

```bash
gst-launch-1.0 rtspsrc location=rtsp://10.21.31.103:8554/video1 latency=100 \
! decodebin ! videoconvert ! autovideosink
```

✅ **If video is displayed**, the following are confirmed:
- Network connectivity is correct
- RTSP server on the robot is working
- Camera stream is alive

> ⚠️ If this step fails, **do not proceed** to ROS2/RViz2 debugging.


#### 4. Confirm the ROS2 Image Topic Is Being Published

The **camera bridge node** converts RTSP into a ROS2 image topic.

A successful setup should publish a topic such as:

```text
/camera/front/image_raw
```

##### 4.1 List All ROS2 Topics

```bash
ros2 topic list
```
##### 4.2 Inspect Topic Details (Including QoS)

```bash
ros2 topic info -v /camera/front/image_raw
```

✅ You should see:
- **Type:** `sensor_msgs/msg/Image`
- **Publisher count:** `>= 1`

This confirms the image is being published correctly.


#### 5. Display the Image in RViz2 (Key Step)

##### 5.1 Launch RViz2

```bash
rviz2
```

### 5.2 Add an Image Display

1. In the left panel, click **Add**
2. Select **Image**
3. Click **OK**


##### 5.3 Configure the Image Display

- **Image → Topic**  
  Select:


```bash
/camera/front/image_raw
```

⚠️ If the topic field is empty, RViz will show:
```text
Error subscribing: Empty topic name
```

##### 5.4 Set the Correct QoS Policy (Important)

- **Image → Reliability Policy**

```text
Best Effort
```


📌 Many camera streams use **Best Effort** for lower latency.  
If RViz is set to **Reliable**, it may fail to receive images.


#### ✅ Final Result

When everything is configured correctly:

- RTSP stream is reachable
- ROS2 image topic is published
- RViz2 subscribes with correct QoS

➡️ **The RViz2 Image display will show the live camera feed in real time.**


---

### 2026.1.10
### 🧾 Update Logs

#### ✅ Added

- **Introduced MuJoCo-LiDAR:** Integrated `mujoco-lidar==0.2.5` into the simulation environment for point cloud generation and publishing (dependencies managed via Poetry + `pyproject.toml`).
- **Stable camera & LiDAR streaming:** Improved the robustness of camera and point cloud publishing/visualization in simulation by tuning **ROS 2 QoS** settings (especially for displaying `PointCloud2` in RViz2).

#### 🔧 Changed

- **Updated MuJoCo simulation scripts & MJCF in `sdk_deploy`:** Added camera and LiDAR, and ensured they can run together reliably.
- **Added `env.sh` environment script:** Unified terminal environment setup (e.g., `ROS_DISTRO`, Python, dependencies).

#### 📔 Notes

- Reproducing the setup requires **4 terminals** (MuJoCo sim / `rl_deploy` / `rqt` / `rviz2`).
- In RViz2, it’s recommended to set **PointCloud2** to **Best Effort** to improve real-time display stability (avoids missing point clouds due to QoS mismatch).
- 

#### Reproduction Steps (4 Terminals)

Recommended to run in **VS Code**.  
Before running any commands in each terminal, **source the environment script**.

##### 0) Common setup for every new terminal

After opening a new terminal in VS Code, run:

```bash
source ~/Maxwell\ Robotics/puma/external/sdk_deploy/env.sh
```

##### 1) Verify environment consistency (recommended for every new terminal)

Run:

```bash
python3 -V
python3.12 -V
echo $ROS_DISTRO
python3.12 -c "import rclpy; print('rclpy ok')"
python3.12 -c "import drdds; print('drdds ok')"
```

**Expected:** Python versions and `ROS_DISTRO` are consistent; both `rclpy ok` and `drdds ok` print successfully.


##### Startup order (recommended: 1 → 4)

**Terminal 1:** Start MuJoCo simulation (camera + LiDAR)

```bash
python3.12 src/M20_sdk_deploy/interface/robot/simulation/mujoco_simulation_ros2.py
```

**Terminal 2:** Start RL Deploy node

```bash
ros2 run rl_deploy rl_deploy
```

**Terminal 3:** Open rqt (visualization for front and rear cameras)

```bash
rqt
```

**Terminal 4:** Open RViz2 (visualization for front and rear LiDARs)

```bash
rviz2
```
#### Required RViz2 setting (PointCloud2 QoS)

In RViz2, for the **PointCloud2** display item:

- Set **QoS Reliability** to: **Best Effort**

If not set, the point cloud may not be received or may appear intermittently (a typical QoS mismatch symptom).

---

### 2025.12.15
### 🧾 Update Log & Collaboration Notes

#### What was done

- Integrated `sdk_deploy` as a **Git submodule** instead of copying SDK code into the main repository.
- Forked the official SDK repository to enable team-specific development:
  - **Official repository**: `DeepRoboticsLab/sdk_deploy`
  - **Team fork**: `The-RoboTUM/PUMA_sdk_deploy`
- Implemented SDK customizations:
  - Added front and rear camera definitions
  - Enabled real-time camera visualization in `rqt`
  - Added `odom` TF for visualization in `RViz`

---

#### Why this structure is used

- Keep the official SDK clean and easy to update
- Allow team-specific customization without modifying upstream code
- Lock (pin) the SDK to a specific, known-good commit for reproducibility
- Follow industry-standard workflows used in robotics and large-scale software projects

---

#### How teammates should work with this repository

##### Pulling and updating code

- Always pull the **main PUMA repository** (not the SDK repo directly)
- After pulling, always run:

```bash
git submodule update --init --recursive
```

This ensures the SDK submodule is checked out at the exact version required by the main project.

---

#### ⚠️ Important: How to commit and push changes (read carefully)

##### ✅ Case 1: You modify files inside `external/sdk_deploy`

- `external/sdk_deploy` is a **Git submodule** (a separate repository)
- Changes must be committed and pushed **inside the submodule repository**

**Correct workflow:**

```bash
cd external/sdk_deploy
git add -A
git commit -m "Describe your SDK change"
git push
```

After pushing the SDK changes:

```bash
cd ../..
git add external/sdk_deploy
git commit -m "Update sdk_deploy submodule pointer"
git push
```

> ❗ The main repository only records **which SDK commit is used**.  
> It does **not** contain the SDK code itself.

---

##### ✅ Case 2: You modify files outside `external/` (main repository code)

- These changes belong to the **main PUMA repository**
- You can commit and push directly from the project root:

```bash
git add .
git commit -m "Describe your main repository changes"
git push
```

#### ❌ What NOT to do

- ❌ Do **not** manually clone or pull the SDK repository separately
- ❌ Do **not** run `git pull` inside `external/sdk_deploy` unless you are actively developing the SDK
- ❌ Do **not** expect SDK changes to be pushed via the main repository

---

#### One-sentence takeaway

> **SDK changes → push in the submodule repository**  
> **Main project changes → push in the main repository**


### 2025.12.3
### Updating the Submodule (`external/sdk_deploy` — official SDk from Deep Robotics) to the Latest Official Version

To update the official repository that you added as a submodule, follow these steps:

#### 1. Enter the submodule directory

```bash
cd external/sdk_deploy   # Replace with your actual submodule path
```

#### 2. Pull the latest changes from the official repository
Check whether the official default branch is main or master.
Assuming the branch is main:

```bash
git pull origin main
```

#### 3. Return to the root of your main project

```bash
cd ../..
```
#### 4. Commit the updated submodule reference to your main repository

```bash
git status        # You will see that external/sdk_deploy appears as modified
git add external/sdk_deploy
git commit -m "Update sdk_deploy submodule to latest main"
git push
```
This process updates the commit reference of the submodule within your main repository, ensuring that your project now points to the latest version of the official SDK.

### 2025.11.26
###  current communication structure:

```
        (TCP data)                           (ROS2)
PUMA robot  ───────────────────────────▶ RobotClient.listen_loop
                                                │
                                                ▼
                                     on_message_received callback
                                                │
                                                ▼
                                puma_driver.publish("/puma/status")

```

### Current Overall Data Flow Summary


High-level control (e.g., teleop nodes):

Publish `/cmd_vel` (Twist)

↓

`PumaRosDriver.cmd_vel_callback` writes the incoming values into `RobotClient.target_vx / target_vy / target_vw`

↓

Inside the asynchronous thread, `RobotClient.control_loop()` sends the target velocities to the robot via TCP at a fixed rate 

↓

The robot returns status / feedback data

↓

`RobotClient.listen_loop()` receives the incoming bytes → calls `on_message_received`

↓

`PumaRosDriver.on_robot_message` decodes the payload into a string and publishes it to `/puma/status`

↓

Other ROS nodes subscribe to `/puma/status` and receive the robot’s feedback.

[View detailed changelog](./docs/changelogs/changelog_2025.11.23_Pedro.md)




# 【Future Outlook】PUMA Quadruped ROS2 Communication Framework  
*A complete architecture integrating Jetson, LiDAR, Camera, ROS2, and TCP.*

This document explains how the **PUMA quadruped robot**, **Jetson onboard computer**, **upper-level laptop**, **LiDAR**, **Camera**, and **ROS2** communicate with each other to form a full robot system supporting SLAM, navigation, and perception.

**Core principle:**

- **Only PUMA ↔ Jetson uses TCP.**  
- **All other modules communicate via ROS2 topics + DDS.**

---

# 1️⃣ Physical System Architecture

```
┌──────────────────────────────────────────────┐
│ Laptop (ROS2)                                │
│ • rviz2 visualization                        │
│ • teleop keyboard/joystick control           │
│ • remote monitoring / ros2 bag / tuning      │
└───────────────▲──────────────────────────────┘
│
│ WiFi / Ethernet (ROS2 DDS)
│
┌───────────────┴────────────────────────────┐
│ Jetson / Industrial PC (on PUMA)           │
│ • Runs all major ROS2 nodes                │
│ • LiDAR driver (/scan or /points)          │
│ • Camera driver (/image)                   │
│ • SLAM node (/scan, /odom, /tf)            │
│ • Nav2 stack (/map, /odom, /tf → /cmd_vel) │
│ • ⭐ puma_driver (ROS2 ↔ TCP bridge)       │
└───────────────▼────────────────────────────┘
│
│ TCP (RobotClient protocol)
│               ▼
┌───────────────────────────────────────────────────────────────────┐
│ PUMA Quadruped Controller                                         │
│ • Understands TCP (we need access to PUMA's ROS infrastructure)   │
│ • Receives velocity/pose commands via TCP                         │
│ • Returns state, IMU, odometry, status                            │
└───────────────────────────────────────────────────────────────────┘

```


---

# 2️⃣ ROS2 Graph Inside Jetson

The Jetson runs all high-level robot software. Nodes communicate using ROS2 topics and TF.

```
                 (Laptop joins ROS2 network)
                    ┌───────────────┐
                    │   rviz2       │
                    │ teleop_node   │
                    └─────▲───▲─────┘
                          │   │ /cmd_vel
                /odom     │   │ /puma/status
                          │   │
┌─────────────────────────┼───┼──────────────────────────────┐
│                   Jetson ROS2 Graph                        │
│                                                            │
│  ┌──────────────────┐      ┌──────────────────┐            │
│  │ LiDAR Driver     │      │ Camera Driver    │            │
│  │ publishes /scan  │      │ publishes /image │            │
│  └──────────────────┘      └──────────────────┘            │
│                                                            │
│                 ┌──────────────────────┐                   │
│                 │       SLAM Node      │                   │
│                 │  sub: /scan, /odom   │                   │
│                 │  pub: /map, /tf      │                   │
│                 └────────▲─────────────┘                   │
│                          │                                 │
│             /odom,/tf    │        /map,/tf                 │
│                          │                                 │
│   ┌─────────────┐        │        ┌──────────────────────┐ │
│   │ puma_driver │<───────┼────────┤     Nav2 Stack       │ │
│   │ ROS ↔ TCP   │        │        │  pub: /cmd_vel       │ │
│   │ pub: /odom  │        │        │  sub: /map,/odom,/tf │ │
│   │ pub: /puma/status    │        └──────────────────────┘ │
│   └───────▲─────┘                                          │
│           │  TCP (RobotClient)                             │
└───────────┼────────────────────────────────────────────────┘
            │
            ▼
      PUMA Quadruped Controller (TCP)


```
---

# 3️⃣ Responsibilities of Each Module

### 🟦 **LiDAR Driver**
- Reads data from LiDAR (Ethernet/USB)
- Publishes:
  - `/scan` (LaserScan)  
  - or `/points` (PointCloud2)

---

### 🟩 **Camera Driver**
- Publishes:
  - `/image`
  - `/camera_info`

---

### 🟧 **SLAM Node**
- Subscribes: `/scan`, `/odom`, `/tf`
- Publishes:
  - `/map`
  - TF (`map → odom`)

---

### 🟪 **Nav2 Navigation Stack**
- Subscribes: `/map`, `/odom`, `/tf`
- Publishes: `/cmd_vel`
- Provides:
  - Global path planning  
  - Local obstacle avoidance  
  - Waypoint navigation  

---

### ⭐ **puma_driver (ROS2 ↔ TCP Bridge)**

The **only bridge** connecting ROS2 and the PUMA robot.

#### ROS → PUMA (Control Path)
- Subscribes: `/cmd_vel`
- Updates RobotClient target velocity (`vx`, `vy`, `vw`)
- RobotClient `control_loop()` sends commands to PUMA via TCP

#### PUMA → ROS (Feedback Path)
- Receives TCP packets (state/IMU/odometry)
- Publishes:
  - `/puma/status`
  - `/odom`
  - TF (`base_link → odom`)

---

### 🖥 **Laptop (ROS2 Participant)**
- rviz2 visualization (map, odometry, TF, LiDAR, camera)
- teleop (publishing `/cmd_vel`)
- ros2 bag recording
- remote parameter tuning

---
