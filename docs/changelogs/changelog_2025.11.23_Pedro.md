## Overview of current ROS2 workspace

Last update: 2025.11.23

This package connects **ROS 2** with the **PUMA robot** over **TCP**.

It is structured in three layers:

1. `driver_node.py` – ROS 2 node layer (talks to `/cmd_vel` and `/puma/status`)
2. `robot_client.py` – async TCP client (talks directly to the robot controller)
3. `protocol_utils.py` – protocol helpers (build/parse headers and JSON payloads)

---

## File Descriptions

### `driver_node.py` : ROS 2 ↔ Robot bridge

Defines the `PumaRosDriver` node:

- Subscribes to `/cmd_vel` (`geometry_msgs/Twist`)
  - `cmd_vel_callback` stores the requested velocities in  
    `self.client.target_vx`, `target_vy`, `target_vw`.
- Publishes `/puma/status` (`std_msgs/String`)
  - `on_robot_message` decodes the bytes from the robot and publishes them as a string.
- Starts the TCP client in a background thread:
  - Creates a separate `asyncio` event loop
  - Runs `RobotClient.heartbeat_loop()`, `control_loop()`, and `listen_loop()` in that loop.

In short: `driver_node.py` exposes the robot to ROS 2 via standard topics.

---

### `protocol_utils.py` : Protocol utilities

Provides configuration and low-level protocol helpers:

- Network config:
  - `ROBOT_IP`, `ROBOT_PORT`, `HEARTBEAT_FREQ`
- Constants:
  - `SYNC_BYTES` – 4-byte magic header for framing packets.
- Functions:
  - `get_timestamp()` – formatted timestamp string.
  - `build_header(payload_len, msg_id, is_json=True)` – builds a 16-byte header.
  - `parse_header(header)` – parses the header and returns `(payload_len, msg_id)`.
  - `create_json_payload(type_code, cmd_code, items=None)` – builds the JSON body and returns UTF-8 encoded bytes.

These helpers are used by `RobotClient` to format and parse packets.

---

### `robot_client.py` : Async TCP client for PUMA

Implements the `RobotClient` class that manages the TCP connection and robot commands.

Main responsibilities:

- **Connection**:
  - `connect()` – opens a TCP connection to `ROBOT_IP:ROBOT_PORT` and sets `running = True`.
  - `send_message(type_code, command_code, items=None)` – builds header + JSON payload and sends a full packet.

- **Loops (run in the asyncio event loop)**:
  - `heartbeat_loop()`  
    - Periodically sends a heartbeat message (`Type=100, Command=100`) at `HEARTBEAT_FREQ`.
  - `control_loop()`  
    - Smoothly updates `current_vx/vy/vw` toward `target_vx/vy/vw` using acceleration limits.  
    - Sends motion commands (`Type=2, Command=21`) at `CONTROL_FREQ` (20 Hz).
  - `listen_loop()`  
    - Reads 16-byte headers, parses payload length, then reads payload.  
    - Calls `self.on_message_received(payload_bytes)` if a callback is set.

- **High-level robot commands**:
  - `set_mode(mode)` – switch usage mode (e.g. 0: Regular, 1: Navigation).
  - `set_motion_state(state)` – change motion state (Idle, Stand, Soft Estop, Sit, etc.).
  - `set_gait(gait)` – set gait type (basic, stair, etc.).
  - `soft_stop()` – convenience wrapper for soft emergency stop.
  - `set_sleep_mode(...)` – configure sleep behavior.
  - `set_flashlight(on)` – control front/back lights.

- **Shutdown**:
  - `close()` – stops loops and closes the TCP connection.

---

## How They Work Together

**Control flow (ROS → PUMA)**

1. A ROS node (e.g. teleop) publishes `/cmd_vel` (Twist).
2. `PumaRosDriver.cmd_vel_callback` writes the desired velocities to  
   `RobotClient.target_vx / target_vy / target_vw`.
3. In the background asyncio loop, `RobotClient.control_loop()`:
   - Smooths the velocities (acceleration-limited),
   - Sends motion commands to the robot via TCP at 20 Hz.

**Feedback flow (PUMA → ROS)**

1. The PUMA controller sends packets back over TCP.
2. `RobotClient.listen_loop()`:
   - Reads and parses the header,
   - Reads the payload,
   - Invokes `on_message_received(payload_bytes)`.
3. In `PumaRosDriver`, this callback is bound to `on_robot_message`, which:
   - Decodes the payload to a string,
   - Publishes it on `/puma/status`.
4. Other ROS nodes subscribe to `/puma/status` to receive robot feedback.

Together, these three files form a clean bridge between **ROS 2 topics** and the robot’s **custom TCP protocol**.
