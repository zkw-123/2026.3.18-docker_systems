# Leptrino Force/Torque Sensor Container

## What this container does

This container reads data from the Leptrino 6-axis force/torque sensor and publishes the measured force/torque to ROS 2.

It:

- connects to the sensor
- reads force/torque data
- publishes the data as a ROS 2 topic
- can save the data to CSV

It does not handle perception, MPC, robot motion, or motor control.

---

## Package structure

```bash
Leptrino_sensor_container
├── ros2_ws
│        └── src/
│            └── leptrino_force_torque/
├── docker-compose.yml
└── dockerfile
```

---

## How to use

### 1. Enter the container

```bash
docker exec -it <container_name> /bin/bash
```

### 2. Source the environment

```bash
source /opt/ros/humble/setup.bash
source /ros_ws/install/setup.bash
```

### 3. Start the sensor node

```bash
ros2 launch leptrino_force_torque leptrino.launch.py
```

### 4. Check the output

Topic:

```bash
/force_torque
```

Command:

```bash
ros2 topic echo /force_torque
```

### 5. Save data to CSV

```bash
python3 /ros_ws/src/data_manipulation/data_to_csv.py 10 30
```