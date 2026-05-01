# control_command_split

> A modular ROS 2 package for command-driven robot motion control — integrating MoveIt planning, Servo-based online motion, insertion strategy execution, and robot status publishing.

---

## Table of Contents

- [Overview](#overview)
- [Package Structure](#package-structure)
- [Module Responsibilities](#module-responsibilities)
- [Build](#build)
- [Running](#running)
- [Runtime Dependencies](#runtime-dependencies)
- [Topics](#topics)
- [Parameters](#parameters)
- [Supported Commands](#supported-commands)
- [Example Usage](#example-usage)
- [Typical Usage Flow](#typical-usage-flow)
- [Notes](#notes)

---

## Overview

`control_command_split` is a modularized ROS 2 robot controller, refactored from a monolithic original into independent, maintainable modules. The redesign improves readability, debuggability, and extensibility.

**Two executable nodes are provided:**

| Node | Description |
|------|-------------|
| `robot_control_node` | Main command-driven robot controller |
| `robot_status_publisher_node` | Continuous publisher for end-effector pose and joint info |

**Supported motion modes:**
- MoveIt planning-based motion execution
- Servo-based online closed-loop motion control

---

## Package Structure

```
control_command_split/
├── CMakeLists.txt
├── package.xml
├── README.md
├── include/
│   └── control_command_split/
│       ├── command_dispatcher.hpp
│       ├── insert_strategy.hpp
│       ├── joint_state_cache.hpp
│       ├── motion_primitives.hpp
│       ├── moveit_bootstrap.hpp
│       ├── robot_controller.hpp
│       ├── servo_controller.hpp
│       └── tf_helper.hpp
└── src/
    ├── command_dispatcher.cpp
    ├── insert_strategy.cpp
    ├── joint_state_cache.cpp
    ├── motion_primitives.cpp
    ├── moveit_bootstrap.cpp
    ├── robot_controller.cpp
    ├── robot_controller_node.cpp
    ├── robot_status_publisher_node.cpp
    ├── servo_controller.cpp
    └── tf_helper.cpp
```

---

## Module Responsibilities

### `robot_controller`
Top-level orchestrator node. Initializes all modules, subscribes to the command topic, dispatches parsed commands, and manages the Servo timer.

### `command_dispatcher`
Parses raw string commands into structured `Command` objects. Classifies command type and stores optional arguments (Cartesian step, path fraction, insertion percent/mode, down-pose yaw).

Supported command types: `pose`, `move`, `cart`, `joints`, `insert`, `down`, `current`, `stop`

### `moveit_bootstrap`
Isolates all MoveIt startup logic. Waits for `/move_group`, copies parameters, creates `MoveGroupInterface`, applies common planning setup, and prints planning/end-effector frames.

### `joint_state_cache`
Subscribes to `/joint_states`, stores the latest message thread-safely, and exposes it to other modules. Also provides a utility to build a joint-name→position map.

### `tf_helper`
Unified TF lookup API that hides buffer/listener details and reduces repeated exception-handling boilerplate. Used by the Servo controller and status query.

### `servo_controller`
Servo-based online motion module. Initializes Servo parameters, computes and publishes velocity commands at each cycle, maintains reference orientation, and stops safely with zero-twist publishing on timeout, target reached, or stop command.

### `motion_primitives`
Container for motion-level behaviors:
- Pose-based motion
- Position-only motion
- Cartesian path motion
- Down-pose motion
- Joint-space motion
- World-Z insertion fallback
- Robot status printing
- Insertion command dispatch

### `insert_strategy`
Task-level insertion strategy, kept independent from generic motion primitives. Executes insertion along the tool local Z (`joint` mode) or world Z (`world` mode), verifies linearity and orientation consistency, applies time parameterization, and executes the final trajectory.

### `robot_status_publisher_node`
Standalone lightweight monitoring node. Publishes end-effector pose from TF, subscribes to joint states, and publishes a readable robot status string.

---

## Build

From the workspace root:

```bash
colcon build --packages-select control_command_split
source install/setup.bash
```

> If running inside Docker, execute the same commands within the ROS 2 workspace in the container.

---

## Running

### Start the main robot controller

```bash
ros2 run control_command_split robot_control_node
```

### Start the robot status publisher

```bash
ros2 run control_command_split robot_status_publisher_node
```

---

## Runtime Dependencies

Ensure the following are available **before** launching:

- [ ] `move_group` is running
- [ ] Robot description is properly loaded
- [ ] TF tree is available
- [ ] `/joint_states` is being published
- [ ] MoveIt Servo is running *(only required if `use_servo=true`)*

---

## Topics

### `robot_control_node`

| Direction | Topic | Type |
|-----------|-------|------|
| Subscribed | `/robot_command` | `std_msgs/msg/String` |
| Subscribed | `/joint_states` | `sensor_msgs/msg/JointState` |
| Published *(Servo)* | `/servo_node/delta_twist_cmds` *(default)* | Twist command |

### `robot_status_publisher_node`

| Direction | Topic | Type |
|-----------|-------|------|
| Subscribed | `/joint_states` | `sensor_msgs/msg/JointState` |
| Published | `/robot_current_pose` | Pose |
| Published | `/robot_status_string` | `std_msgs/msg/String` |

---

## Parameters

### Controller

| Parameter | Description |
|-----------|-------------|
| `use_sim_time` | Use simulation clock |
| `servo_rate_hz` | Servo loop rate |

### Servo

| Parameter | Description |
|-----------|-------------|
| `use_servo` | Enable Servo-based motion |
| `servo_twist_topic` | Twist command topic |
| `servo_start_service` | Servo start service name |
| `servo_cmd_frame` | Command frame |
| `servo_lin_kp` | Linear proportional gain |
| `servo_lin_vmax` | Max linear velocity |
| `servo_goal_tolerance` | Goal tolerance |
| `servo_timeout_sec` | Servo timeout |
| `servo_halt_msgs` | Number of halt messages to publish on stop |
| `servo_ang_kp` | Angular proportional gain |
| `servo_ang_wmax` | Max angular velocity |

### Status Publisher

| Parameter | Description |
|-----------|-------------|
| `base_frame` | Base TF frame |
| `ee_frame` | End-effector TF frame |
| `pose_topic` | Output pose topic |
| `status_topic` | Output status string topic |
| `joint_topic` | Input joint states topic |
| `publish_rate` | Publishing rate (Hz) |
| `max_joint_print` | Max number of joints to print in status |

---

## Supported Commands

Commands are sent as `std_msgs/msg/String` to `/robot_command`.

### `pose` — Constrained position motion

Move to a Cartesian position using J4-corridor-constrained MoveIt planning.

```
pose <x> <y> <z>
```
```
pose 0.30 0.10 0.20
```

### `move` — General position motion

Move to a Cartesian position. Uses MoveIt planning when `use_servo=false`, or Servo closed-loop tracking when `use_servo=true`.

```
move <x> <y> <z>
```
```
move 0.32 0.08 0.18
```

### `cart` — Cartesian path motion

Move along a Cartesian path with optional step size and minimum fraction.

```
cart <x> <y> <z> [eef_step] [min_fraction]
```
```
cart 0.30 0.05 0.22
cart 0.30 0.05 0.22 0.003 0.95
```

### `joints` — Joint-space motion

Move to explicit joint positions (radians).

```
joints <j1> <j2> <j3> <j4> <j5> <j6>
```
```
joints 0.0 -0.3 1.0 0.0 0.5 0.0
```

### `insert` — Insertion motion

Execute insertion with optional velocity scaling and mode.

```
insert <distance_m> [percent] [joint|world]
```
```
insert 0.03
insert 0.03 20
insert 0.03 20 joint
insert 0.03 20 world
```

| Argument | Description |
|----------|-------------|
| `distance` | Insertion distance in meters |
| `percent` | Velocity/acceleration scaling (1–100) |
| `joint` | Insert along tool local Z |
| `world` | Insert along world Z |

### `down` — Downward pose motion

Move to a downward-facing pose with optional yaw.

```
down <x> <y> <z> [yaw_rad]
```
```
down 0.30 0.10 0.15
down 0.30 0.10 0.15 1.57
```

### `current` / `status` — Query robot state

Print current end-effector position, joint values, and Servo state.

```
current
```

### `stop` — Stop Servo motion

Immediately halt Servo motion.

```
stop
```

---

## Example Usage

After sourcing the workspace, send commands from a second terminal:

```bash
# Query current state
ros2 topic pub /robot_command std_msgs/msg/String "{data: 'current'}" --once

# Move to position
ros2 topic pub /robot_command std_msgs/msg/String "{data: 'move 0.30 0.10 0.20'}" --once

# Cartesian path motion
ros2 topic pub /robot_command std_msgs/msg/String "{data: 'cart 0.28 0.12 0.18 0.003 0.95'}" --once

# Insertion
ros2 topic pub /robot_command std_msgs/msg/String "{data: 'insert 0.02 20 joint'}" --once

# Stop
ros2 topic pub /robot_command std_msgs/msg/String "{data: 'stop'}" --once
```

---

## Typical Usage Flow

1. Start the robot driver
2. Start MoveIt / `move_group`
3. Start MoveIt Servo *(if `use_servo=true`)*
4. Source the workspace: `source install/setup.bash`
5. Run `robot_control_node`
6. *(Optional)* Run `robot_status_publisher_node`
7. Publish commands to `/robot_command`

---

## Notes

**Package name vs. C++ namespace**
The ROS package name is `control_command_split`, but the internal C++ namespace is `control_command`. This is intentional and does not affect execution.

**Include path convention**
All headers live under `include/control_command_split/` and must be included as:
```cpp
#include "control_command_split/xxx.hpp"
```

**Servo behavior**
When `use_servo=true`, the `move` command switches from planning-based execution to online closed-loop Servo tracking.

**Insertion fallback**
If the insertion strategy fails, the controller automatically falls back to world-Z Cartesian insertion.