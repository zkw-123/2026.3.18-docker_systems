# Stability-aware Ultrasound-guided Robotic Tissue Manipulation System
**(ROS 2 Multi-Container Architecture & Execution Guide)**

---

## 1. System Overview

本项目实现了一个用于微创组织提取的**稳定性感知、超声引导机器人组织操作系统**。

系统采用**模块化 ROS 2 多容器架构**，包含五个容器：
- `polaris_ultrasound_container`: 感知与状态估计（超声 + 追踪）
- `sk_mpc_container`: 决策与优化 (MPC)
- `denso_ros2_container`: 机器人运动规划与执行 (MoveIt2 + Gazebo)
- `faulhaber_motor_container`: 切割电机控制
- `Leptrino_sensor_container`: 力/力矩传感数据采集

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
                 ▲                                               ▲
                 │  /robot_command (String)                      │   /motor_command (String)   
                 │                                               │
┌──────────────────────────────────────────────┐                 │
│  sk_mpc_container                            │                 │
│  Decision / optimization layer               │                 │
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
            └──────────┴──────────────────────────────┘
            │
            │  state + reliability
            │
┌──────────────────────────────────────────────┐
│  polaris_ultrasound_container                │
│  Perception & state estimation layer         │
│  - Ultrasound imaging                        │
│  - Polaris tracking                          │
│  - Target localization (2D → 3D)             │
│  - Stability index Sk                        │
└──────────────────────────────────────────────┘
```

---

## 3. Container Responsibilities

### 3.1 `denso_ros2_container` — Robot Execution Layer
**职责**: 与机器人硬件或仿真器通信，执行 MoveIt2 运动规划与轨迹执行。提供基于文本的命令接口。
**输入**: `/robot_command` (`std_msgs/String`)

### 3.2 `sk_mpc_container` — Decision / MPC Layer
**职责**: 订阅多源状态信息（位姿、稳定性指标 $S_k$、力反馈），进行安全检测并求解受限 MPC 问题，输出控制指令。
**输入**: `/mpc/stability_index`, `/force_torque`
**输出**: `/mpc/dk_cmd`, `/mpc/safety_mode`

### 3.3 `polaris_ultrasound_container` — Perception & State Estimation Layer
**职责**: 超声图像采集、Polaris 光学追踪与时间同步，将 2D 目标转换为 3D 坐标，并发布稳定性指标 $S_k$。
**输出**: `/target_3d_position`, `/mpc/stability_index`

### 3.4 `faulhaber_motor_container` — Cutter Motor Control
**职责**: 接收 ROS 2 速度指令，通过串口驱动 Faulhaber 电机执行切割动作。
**输入**: `/cmd_vel` (`geometry_msgs/Twist`)

### 3.5 `Leptrino_sensor_container` — Force / Torque Sensing
**职责**: 读取 Leptrino 六轴力/力矩传感器数据，发布至 ROS 2 网络，并支持 CSV 数据记录。
**输出**: `/force_torque` (`geometry_msgs/WrenchStamped`)

---

## 4. End-to-End Data Flow

1. **Perception**: 超声 & 追踪流 → `polaris_ultrasound` → 输出目标位姿及 $S_k$。
2. **Sensing**: 传感器硬件 → `Leptrino_sensor` → 输出 `/force_torque`。
3. **Decision**: 位姿 & 传感数据 → `sk_mpc_container` → 计算增量 $d_k$。
4. **Execution**: $d_k$ 转换为指令 → `denso_ros2_container` → 驱动物理/仿真机械臂。
5. **Actuation**: 控制节点指令 → `faulhaber_motor_container` → 驱动末端切割电机。

---

## 5. Critical Environment & Networking Setup

在启动任何容器之前，**必须**确保宿主机和 Docker 配置满足 CycloneDDS 的高频通信要求，以防止节点崩溃。

### 5.1 系统接收缓冲区优化 (Socket Buffer Fix)
CycloneDDS 默认申请较大的 Socket 接收缓冲区（10MB以上）。若系统限制过严，会报 `RCLError` 导致节点闪退。
- **宿主机临时生效**:
  ```bash
  sudo sysctl -w net.core.rmem_max=2147483647
  ```
- **永久生效**: 在宿主机的 `/etc/sysctl.conf` 文件末尾添加 `net.core.rmem_max=2147483647`:
  ```bash
  echo "net.core.rmem_max=2147483647" | sudo tee -a /etc/sysctl.conf
  sudo sysctl -p
  ```

### 5.2 Docker Compose 系统权限
确保核心容器在 `docker-compose.yml` 中继承了宿主机的网络设置和系统权限：
```yaml
services:
  polaris_ultrasound: # (其他 ROS 2 容器同理)
    network_mode: host
    sysctls:
      - net.core.rmem_max=2147483647
```

### 5.3 CycloneDDS 自动发现机制
为了保证跨设备的兼容性，**禁止**在环境中硬编码网卡名称。
- 删除 `docker-compose.yml` 中的 `CYCLONEDDS_NETWORK_INTERFACE_ADDRESS`。
- 删除 `cyclonedds.xml` 中的 `<NetworkInterfaceAddress>` 标签，让 DDS 自动遍历可用网卡。
- 确保所有容器使用相同的 `ROS_DOMAIN_ID=1`。

---

## 6. Overall Runtime Workflow

*请在运行前确保已执行第 5 节的缓冲区优化。*

### 6.1 Start `denso_ros2_container`
```bash
docker exec -it denso_ros2_container bash
# 终端 1：启动底层驱动或 Gazebo 仿真
ros2 launch denso_robot_bringup denso_robot_bringup.launch.py model:=vp6242 sim:=true
# 终端 2：启动控制桥接
ros2 run control_command robot_control_node
```

### 6.2 Start Sensors (`polaris_ultrasound` & `Leptrino`)
```bash
# 超声与追踪
docker exec -it polaris_ultrasound bash
ros2 launch polaris_ultrasound target_localization_launch.py

# 力矩传感器
docker exec -it Leptrino_sensor_8.22 bash
ros2 launch leptrino_force_torque leptrino.launch.py
```

### 6.3 Start `sk_mpc_container`
```bash
docker exec -it sk_mpc_container bash
# 启动 MPC
ros2 run sk_mpc_controller mpc_node
# 启动指令转换桥
ros2 run sk_mpc_controller mpc_to_robot_command_bridge.py --ros-args -p mode:=insert
```

### 6.4 Start `faulhaber_motor_container`
```bash
docker exec -it faulhaber_motor_controller bash
# 需要确保宿主机已赋权串口: sudo chmod 666 /dev/ttyUSB0
ros2 run faulhaber_motor_controller motor_controller_node
```

---

## 7. Troubleshooting (常见问题排查)

| 现象 (Error Message) | 原因 (Root Cause) | 解决方法 (Solution) |
| :--- | :--- | :--- |
| `enp10s0r: does not match an available interface` | XML 或环境变量中写死了不存在的网卡 | 删除 `cyclonedds.xml` 中的网卡绑定配置，让 DDS 自动搜索。 |
| `failed to increase socket receive buffer size` | Linux 系统对 UDP 缓冲区限制过严 | 见 5.1 节，在宿主机执行 `sudo sysctl -w net.core.rmem_max=2147483647`。 |
| `rmw_create_node: failed to create domain` | DDS 初始化失败 (通常是网络或配置问题) | 检查 `ROS_DOMAIN_ID` 是否冲突，执行 `docker compose down` 彻底重置容器网络。 |
| `Permission denied: /dev/ttyUSBX` | 容器没有串口设备的读写权限 | 宿主机执行 `sudo chmod 666 /dev/ttyUSB*`，并在 `docker-compose.yml` 中配置 `privileged: true`。 |

---

## 8. Notes

- 本架构高度模块化，支持独立调试感知、MPC、机器人执行、电机控制和力传感各组件。
- `network_mode: host` 确保了多个容器与宿主机共享同一网络栈，能够实现最低延迟的 ROS 2 多播通信。
- 推荐使用 `tmux` 或 `terminator` 管理这些多容器的终端会话。
