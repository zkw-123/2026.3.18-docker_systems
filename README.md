# Stability-aware Ultrasound-guided Robotic Tissue Manipulation System
**(ROS 2 Multi-Container Architecture & Execution Guide)**

---

## 1. System Overview

This project implements a **stability-aware, ultrasound-guided robotic tissue manipulation system** for minimally invasive tissue extraction.

The system is designed as a **modular ROS 2 multi-container architecture** with five containers:

- perception and state estimation
- decision making and optimization (MPC)
- robot motion planning and execution
- cutter motor control
- force/torque sensing

The original core architecture already consisted of three main layers: `polaris_ultrasound`, `sk_mpc_container`, and `denso_ros2_container`. :contentReference[oaicite:0]{index=0}

---

## 2. Container-level Architecture

```text
┌──────────────────────────────────────────────┐       ┌──────────────────────────────────────────────┐
│  denso_ros2_container                        │       │  faulhaber_motor_container                   │
│  Robot execution layer                       │       │  Cutter motor control                        │
│  - DENSO ROS2 driver                         │       │  - Motor controller node                     │
│  - MoveIt2                                   │       │  - Serial communication                      │
│  - control_command                           │       │  - /cmd_vel input                            │
└──────────────────────────────────────────────┘       └──────────────────────────────────────────────┘
                 ▲                                                              ▲
                 │  /robot_command (String)                                     │   /motor_command (String)   
                 │                                                              │
┌──────────────────────────────────────────────┐                                │
│  sk_mpc_container                            │                                │
│  Decision / optimization layer               │                                │
│  - Stability-aware MPC                       │────────────────────────────────│
│  - Safety gating                             │
│  - dk output                                 │
└──────────────────────────────────────────────┘
            ▲                 ▲
            │                 │
            │                 │  /force_torque
            │                 │
            │          ┌──────────────────────────────┐
            │          │  Leptrino_sensor_container   │
            │          │  Force / torque sensing      │
            │          │  - Leptrino sensor node      │
            │          │  - CSV logging utility       │
            │          └──────────────────────────────┘
            │
            │  state + reliability
            │
┌──────────────────────────────────────────────┐
│  polaris_ultrasound_container                │
│  Perception & state estimation layer         │
│  - Ultrasound imaging                        │
│  - Polaris optical tracking                  │
│  - Time synchronization                      │
│  - Target localization (2D → 3D)             │
│  - Stability index Sk                        │
└──────────────────────────────────────────────┘

```

---

## 3. Container Responsibilities

### 3.1 `denso_ros2_container` — Robot Execution Layer

**Role**  
This container directly interfaces with the robot hardware or simulator. :contentReference[oaicite:1]{index=1}

**Responsibilities**
- communication with DENSO robot controllers via ROS 2
- motion planning and collision checking using MoveIt2
- trajectory execution and controller-level safety handling
- providing a text-based command interface for upper layers :contentReference[oaicite:2]{index=2}

**Primary input topic**
- `/robot_command` (`std_msgs/String`) :contentReference[oaicite:3]{index=3}

**Supported commands**
```text
pose x y z
move x y z
joints j1 j2 j3 j4 j5 j6
insert Δz
current
```
:contentReference[oaicite:4]{index=4}

---

### 3.2 `sk_mpc_container` — Decision / MPC Layer

**Role**  
This container implements the stability-aware MPC and outputs control commands. :contentReference[oaicite:5]{index=5}

**Responsibilities**
- subscribe to multi-source state information
- perform safety checks and gating
- solve the constrained MPC problem
- output intent-level control commands :contentReference[oaicite:6]{index=6}

**Typical input topics**
- `/mpc/lateral_error`
- `/mpc/normal_step`
- `/mpc/stability_index`
- `/ft_sensor` :contentReference[oaicite:7]{index=7}

**Typical output topics**
- `/mpc/dk_cmd` (`std_msgs/Float64`)
- `/mpc/safety_mode` (`std_msgs/Bool`) :contentReference[oaicite:8]{index=8}

---

### 3.3 `polaris_ultrasound_container` — Perception & State Estimation Layer

**Role**  
This container converts raw sensor streams into time-consistent, control-ready state estimates. :contentReference[oaicite:9]{index=9}

**Responsibilities**
- ultrasound image acquisition
- Polaris optical tracking
- timestamp synchronization
- target detection in ultrasound images
- target localization in 3D
- computation and publication of stability index `Sk` :contentReference[oaicite:10]{index=10}

**Typical outputs**
- target position
- tracking quality / confidence
- stability index `Sk` :contentReference[oaicite:11]{index=11}

---

### 3.4 `faulhaber_motor_container` — Cutter Motor Control

**Role**  
This container controls the Faulhaber motor for the cylindrical cutter.

**Responsibilities**
- receive ROS 2 motor commands
- send commands to the motor driver
- drive the motor at the requested speed

**Command topic**
- `/cmd_vel`

**Message type**
- `geometry_msgs/msg/Twist`

**Run**
```bash
ros2 run faulhaber_motor_controller motor_controller_node
```

**Example command**
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 50.0}, angular: {z: 0.0}}" --once
```

**Package structure**
```text
faulhaber_motor_controller/
├── src/
│   └── faulhaber_motor_controller/
├── README.md
├── docker-compose.yml
├── dockerfile
└── 命令行.odt
```

---

### 3.5 `Leptrino_sensor_container` — Force / Torque Sensing

**Role**  
This container reads data from the Leptrino 6-axis force/torque sensor and publishes the measured wrench to ROS 2.

**Responsibilities**
- connect to the sensor
- read force/torque data
- publish the data as a ROS 2 topic
- save the data to CSV if needed

**Output topic**
- `/force_torque`

**Run**
```bash
ros2 launch leptrino_force_torque leptrino.launch.py
```

**Check output**
```bash
ros2 topic echo /force_torque
```

**CSV logging**
```bash
python3 /ros_ws/src/data_manipulation/data_to_csv.py 10 30
```

**Package structure**
```text
Leptrino_sensor_8.22/
├── src/
├── docker-compose.yml
└── dockerfile
```

---

## 4. End-to-End Data Flow

### 4.1 Perception Flow (`polaris_ultrasound`)

```text
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
:contentReference[oaicite:12]{index=12}

---

### 4.2 Decision Flow (`sk_mpc_container`)

```text
Target state + Sk + Force/Torque
                ↓
         Stability-aware MPC
                ↓
      dk (lateral step command)
```
:contentReference[oaicite:13]{index=13}

---

### 4.3 Robot Execution Flow (`denso_ros2_container`)

```text
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
:contentReference[oaicite:14]{index=14}

---

### 4.4 Cutter Motor Flow (`faulhaber_motor_controller`)

```text
Motor command (/cmd_vel)
          ↓
Faulhaber motor controller node
          ↓
Serial communication
          ↓
Cutter motor
```

---

### 4.5 Force / Torque Flow (`Leptrino_sensor_8.22`)

```text
Leptrino sensor
      ↓
Sensor node
      ↓
/force_torque
      ↓
sk_mpc_container / logging
```

---

## 5. Closed-loop Control Concept

The system forms a hierarchical closed loop:

- **Perception loop**: `polaris_ultrasound`
- **Decision loop**: `sk_mpc_container`
- **Robot execution loop**: `denso_ros2_container`
- **Cutter actuation loop**: `faulhaber_motor_controller`
- **Mechanical sensing loop**: `Leptrino_sensor_8.22`

The original 3-container closed-loop relationship between perception, MPC, and robot execution is preserved. :contentReference[oaicite:15]{index=15}  
The motor and force/torque containers extend the system for cutter drive and mechanical sensing.

---

## 6. Overall Runtime Workflow

### 6.1 Common Environment Configuration

```bash
export ROS_DOMAIN_ID=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```
:contentReference[oaicite:16]{index=16}

---

### 6.2 Start `denso_ros2_container`

```bash
docker exec -it denso_ros2_container bash
source /opt/ros/humble/setup.bash
sudo apt-get update
rosdep update
rosdep install --ignore-src --from-paths src -y -r
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

```bash
# Terminal 1
ros2 launch denso_robot_bringup denso_robot_bringup.launch.py model:=vp6242 sim:=true

# Terminal 2
ros2 run control_command robot_control_node

# or
ros2 launch control_command robot_control_with_status.launch.py
```
:contentReference[oaicite:17]{index=17}

---

### 6.3 Start `polaris_ultrasound`

```bash
docker exec -it polaris_ultrasound bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
```

```bash
ros2 launch polaris_ultrasound target_localization_launch.py
```

```bash
ros2 run target_localization perception_replay \
  --ros-args \
  -p csv_path:=/ros2_ws/src/target_localization/sk_target_results.csv \
  -p rate_hz:=10.0 \
  -p loop:=true \
  -p frame_id:=robot_base_link \
  -p offset_x:=-0.70 \
  -p offset_y:=0.0 \
  -p offset_z:=-1.0
```

```bash
ros2 topic hz /synchronized_data
ros2 topic echo /target_3d_position
ros2 topic echo /mpc/stability_index
```
:contentReference[oaicite:18]{index=18}

---

### 6.4 Start `sk_mpc_container`

```bash
docker exec -it sk_mpc_container bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
```

```bash
ros2 run sk_mpc_controller mpc_node
```

```bash
ros2 topic echo /mpc/dk_cmd
ros2 topic echo /mpc/safety_mode
```
:contentReference[oaicite:19]{index=19}

---

### 6.5 Start `faulhaber_motor_controller`

```bash
docker exec -it faulhaber_motor_controller bash
source /opt/ros/foxy/setup.bash
source /ros_ws/install/setup.bash
```

```bash
ros2 run faulhaber_motor_controller motor_controller_node
```

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 50.0}, angular: {z: 0.0}}" --once
```

---

### 6.6 Start `Leptrino_sensor_8.22`

```bash
docker exec -it Leptrino_sensor_8.22 bash
source /opt/ros/humble/setup.bash
source /ros_ws/install/setup.bash
```

```bash
ros2 launch leptrino_force_torque leptrino.launch.py
```

```bash
ros2 topic echo /force_torque
```

```bash
python3 /ros_ws/src/data_manipulation/data_to_csv.py 10 30
```

---

### 6.7 Establish Closed Loop

```bash
ros2 run sk_mpc_controller mpc_to_robot_command_bridge.py \
  --ros-args \
  -p cmd_rate_hz:=1.0 \
  -p max_step:=0.002 \
  -p mode:=insert
```

```bash
ros2 topic echo /mpc/dk_cmd
ros2 topic echo /robot_command
```
:contentReference[oaicite:20]{index=20}

---

## 7. Typical Runtime Chain

```text
polaris_ultrasound
   → publishes target state + Sk
        ↓
Leptrino_sensor_8.22
   → publishes force_torque
        ↓
sk_mpc_container
   → computes dk
        ↓
denso_ros2_container
   → executes robot motion
        ↓
faulhaber_motor_controller
   → drives cutter motor
```

The original 3-container runtime chain is `polaris_ultrasound → sk_mpc_container → denso_ros2_container`. :contentReference[oaicite:21]{index=21}  
The two added containers extend this chain for cutter actuation and wrench sensing.

---

## 8. Repository Structure (Example)

```text
project_root/
├── docker/
│   ├── denso_ros2_container/
│   ├── sk_mpc_container/
│   ├── polaris_ultrasound/
│   ├── faulhaber_motor_controller/
│   └── Leptrino_sensor_8.22/
├── docs/
│   └── figures/
└── README.md
```

The original example repository structure listed the first three containers. :contentReference[oaicite:22]{index=22}

---

## 9. Notes

This architecture supports:

- modular development
- separate debugging of sensing, MPC, robot execution, motor control, and force sensing
- reproducible multi-container experiments
- extension from perception-control experiments to full cutter-actuated tissue manipulation

The original README already highlighted modularity, reproducibility, and component-wise evaluation for the 3-container system. :contentReference[oaicite:23]{index=23}
