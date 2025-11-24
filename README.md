# Pedro's Notes


###  current communication structure:

```
        (TCP data)
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

[View the detailed changelog](./puma/docs/changelogs/changelog_2025.11.23_Pedro.md)



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
