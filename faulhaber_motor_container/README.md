# Faulhaber Motor Controller Container

## What this container does

This container controls the Faulhaber motor for the cylindrical cutter.

It:

- receives ROS 2 motor commands
- sends commands to the motor driver
- drives the motor at the requested speed

It does not handle perception, MPC, robot-arm motion, or force sensing.

---

## Package structure

```bash
faulhaber_motor_controller/
├── ros2_ws/
│        └── src/
│            └── faulhaber_motor_controller/
├── README.md
├── docker-compose.yml
├── dockerfile
└── 命令行.odt
```

---

## How to use

### 1. Enter the container

```bash
docker exec -it <container_name> /bin/bash
```

### 2. Source the environment

```bash
source /opt/ros/foxy/setup.bash
source /ros_ws/install/setup.bash
```

### 3. Run the node

```bash
ros2 run faulhaber_motor_controller motor_controller_node
```

### 4. Send a motor command

Topic:

```bash
/cmd_vel
```

Message type:

```bash
geometry_msgs/msg/Twist
```

Example:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 50.0}, angular: {z: 0.0}}" --once
```

Usually, `linear.x` is used as the motor speed command.