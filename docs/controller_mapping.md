# Controller Mapping (GameSir T4n Lite)
## Layout Diagram

```text
      [LB: Damping]                               [RB: Unused]
           |                                           |
      .---------.                                 .---------.
     /           \                               /           \
    |   ( L )     |                             |     (Y)     |  Y: Standard Mode
    |   Move      |          [ PUMA ]           |  (X)   (B)  |  X: Obstacles Mode
    |             |                             |     (A)     |  B: Sit
    |      ^      |                             |             |  A: Stand
    |      ^      |                             |             |
    |      |      |                             |    ( R )    |
    |  <---+--->  |                             |   Rotate    |
    |      |      |                             |    <--->    |
     \     v     /                               \           /
      '---------'                                 '---------'
```

## Controls Detail

### Movement (Joysticks)

| Input | Action | Description |
| :--- | :--- | :--- |
| **Left Stick Y** | **Forward / Backward** | Moves the robot linearly forward or backward. |
| **Left Stick X** | **Strafe Left / Right** | Moves the robot sideways (strafing). |
| **Right Stick X** | **Rotate** | Rotates the robot in place (Yaw). |

### Actions (Buttons)

| Button | Action | Description |
| :--- | :--- | :--- |
| **A** (Bottom) | **Stand** | Commands the robot to stand up (Motion State 1). |
| **B** (Right) | **Sit** | Commands the robot to sit down (Motion State 4). |
| **X** (Left) | **Obstacles Mode** | Sets the robot to Obstacles gait (Gait 0x1002). |
| **Y** (Top) | **Standard Mode** | Sets the robot to Standard walking mode (Motion State 6). |
| **LB** (Left Bumper)| **Damping** | Sets the robot to Damping/Passive mode (Motion State 3). |

> **Note:** Ensure the dashboard is in **CONTROLLER** mode for these inputs to take effect.
