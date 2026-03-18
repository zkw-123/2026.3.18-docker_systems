# Stability-aware Ultrasound-guided Robotic Tissue Manipulation System
**(ROS 2 Multi-Container Architecture & Execution Guide)**

---

## 1. System Overview

This project implements a **stability-aware, ultrasound-guided robotic tissue manipulation system** for minimally invasive tissue extraction.

The system is designed as a **strictly modular ROS 2 multi-container architecture**, explicitly separating:

- Perception and state estimation
- Decision making and optimization (Model Predictive Control, MPC)
- Robot motion planning and execution

This separation improves robustness, safety, interpretability, and experimental reproducibility, and allows independent development, debugging, and replacement of each functional layer.

---

## 2. Container-level Architecture

The system runs with **three Docker containers**, each corresponding to one functional layer.

```
┌──────────────────────────────────────────────┐
│  denso_ros2_container                        │
│  Robot execution layer                       │
│  - DENSO ROS2 driver                         │
│  - MoveIt2                                  │
│  - control_command                           │
└──────────────────────────────────────────────┘
                 ▲
                 │  /robot_command (String)
                 │
┌──────────────────────────────────────────────┐
│  sk_mpc_container                            │
│  Decision / optimization layer               │
│  - Stability-aware MPC                       │
│  - Safety gating                             │
│  - dk (lateral step) output                  │
└──────────────────────────────────────────────┘
                 ▲
                 │  state + reliability
                 │
┌──────────────────────────────────────────────┐
│  polaris_ultrasound                          │
│  Perception & state estimation layer         │
│  - Ultrasound imaging                        │
│  - Polaris optical tracking                  │
│  - Time synchronization                     │
│  - Target localization (2D → 3D)             │
│  - Stability index Sk                        │
└──────────────────────────────────────────────┘
```

---

## 3. Container Responsibilities

### 3.1 denso_ros2_container — Robot Execution Layer

**Role**  
This container is the only component that directly interfaces with the robot hardware or simulator.

**Responsibilities**
- Communication with DENSO robot controllers via ROS 2
- Motion planning and collision checking using MoveIt2
- Trajectory execution and controller-level safety handling
- Providing a text-based command interface for upper layers

**Key characteristics**
- No perception
- No optimization or MPC
- Deterministic and debuggable execution behavior

**Primary input topic**
- `/robot_command` (`std_msgs/String`)

**Supported commands**
```
pose x y z
move x y z
joints j1 j2 j3 j4 j5 j6
insert Δz
current
```

---

### 3.2 sk_mpc_container — Decision / MPC Layer

**Role**  
This container implements a **stability-aware model predictive controller** that computes optimal motion adjustments under sensing uncertainty and force constraints.

**Responsibilities**
- Subscribe to multi-source state information:
  - Lateral tracking error
  - Normal insertion step
  - Ultrasound stability index Sk
  - Force / torque measurements
- Perform safety checks and gating
- Solve a constrained MPC problem
- Output intent-level control commands (not trajectories)

**Design principle**
> The MPC determines *what should be done*, not *how the robot executes it*.

**Typical input topics**
- `/mpc/lateral_error`
- `/mpc/normal_step`
- `/mpc/stability_index`
- `/ft_sensor`

**Typical output topics**
- `/mpc/dk_cmd` (`std_msgs/Float64`) — optimal lateral step
- `/mpc/safety_mode` (`std_msgs/Bool`) — protection flag

---

### 3.3 polaris_ultrasound — Perception & State Estimation Layer

**Role**  
This container converts raw sensor streams into time-consistent, control-ready state estimates.

**Responsibilities**
- Ultrasound image acquisition
- Polaris optical tracking
- Precise timestamp synchronization
- Target detection in ultrasound images
- Target localization in 3D world coordinates
- Computation and publication of ultrasound stability index Sk

**Key characteristics**
- Strict time synchronization and quality filtering
- Detection module is fully replaceable (rule-based or learning-based)
- Optional asynchronous data recording and visualization

**Typical outputs**
- Target position (2D / 3D)
- Tracking quality and confidence
- Stability index Sk

---

## 4. End-to-End Data Flow

### 4.1 Perception Flow (polaris_ultrasound)

```
Ultrasound images (~30 Hz)
          +
Polaris tracking (~60 Hz)
          ↓
Time synchronization
          ↓
Target detection (pixel)
          ↓
Target localization (3D)
          ↓
Stability index Sk
```

---

### 4.2 Decision Flow (sk_mpc_container)

```
Target state + Sk + Force/Torque
                ↓
         Stability-aware MPC
                ↓
      dk (lateral step command)
```

---

### 4.3 Execution Flow (denso_ros2_container)

```
dk (Float64)
   ↓
Command bridge
   ↓
/robot_command (String)
   ↓
MoveIt planning & execution
   ↓
DENSO robot
```

---

## 5. Closed-loop Control Concept

The system forms a **hierarchical closed loop**:

- **Inner loop**: Robot-level planning and execution (MoveIt2 and controllers)
- **Outer loop**: Perception-driven MPC adapting motion according to
  - ultrasound image stability (Sk)
  - interaction forces
  - accumulated carry-over effects

This architecture avoids end-to-end black-box control and maintains interpretability, safety, and modularity.

---

## 6. Overall Runtime Workflow (Command-line Guide)

### 6.1 Common Environment Configuration (All Containers)

```
export ROS_DOMAIN_ID=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

---

### 6.2 Start Robot Execution Layer (denso_ros2_container)

```
docker exec -it denso_ros2_container bash
source /opt/ros/humble/setup.bash
sudo apt-get update
rosdep updaterosdep install --ignore-src --from-paths src -y -r
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```
```bash
# Terminal 1: Start robot simulation
ros2 launch denso_robot_bringup denso_robot_bringup.launch.py model:=vp6242 sim:=true

# Terminal 2: Start control node
ros2 run control_command robot_control_node

# Terminal 2(ver_2): Start control node with pose publishing using launch
ros2 launch control_command robot_control_with_status.launch.py
```
---

### 6.3 Start Perception Layer (polaris_ultrasound)

```
docker exec -it polaris_ultrasound bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
```

```
ros2 launch polaris_ultrasound target_localization_launch.py

ros2 run target_localization perception_replay   --ros-args   -p csv_path:=/ros2_ws/src/target_localization/sk_target_results.csv   -p rate_hz:=10.0   -p loop:=true   -p frame_id:=robot_base_link   -p offset_x:=-0.70   -p offset_y:=0.0   -p offset_z:=-1.0

```

```
ros2 topic hz /synchronized_data
ros2 topic echo /target_3d_position
ros2 topic echo /mpc/stability_index
```

---

### 6.4 Start MPC Layer (sk_mpc_container)

```
docker exec -it sk_mpc_container bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
```

```
ros2 run sk_mpc_controller mpc_node
```

```
ros2 topic echo /mpc/dk_cmd
ros2 topic echo /mpc/safety_mode
```

---

### 6.5 Establish Closed Loop

```
ros2 run sk_mpc_controller mpc_to_robot_command_bridge.py   --ros-args   -p cmd_rate_hz:=1.0   -p max_step:=0.002   -p mode:=insert
```

```
ros2 topic echo /mpc/dk_cmd
ros2 topic echo /robot_command
```

---

## 7. Typical Runtime Chain

```
polaris_ultrasound
   → publishes target state + Sk
        ↓
sk_mpc_container
   → computes dk
        ↓
denso_ros2_container
   → executes motion via MoveIt2
```

---

## 8. Repository Structure (Example)

```
project_root/
├── docker/
│   ├── denso_ros2_container/
│   ├── sk_mpc_container/
│   └── polaris_ultrasound/
├── docs/
│   └── figures/
└── README.md
```

---

## 9. Notes for Research and Publication

This architecture is designed to support:
- Clear separation of methodological contributions
- Reproducible experimental evaluation
- Component-wise ablation studies
- Safe deployment on real robotic hardware
