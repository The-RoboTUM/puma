# Pedro's Notes


### 2026.1.10
### 🧾 Update Log


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
