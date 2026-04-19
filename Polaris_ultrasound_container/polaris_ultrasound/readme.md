# polaris_ultrasound

## 1. Package Role

`polaris_ultrasound` provides reusable ROS 2 modules for:

- reading Polaris tracking data
- reading ultrasound images
- replaying recorded ultrasound datasets
- computing ultrasound-side image stability
- estimating target position from ultrasound images

This package focuses on **modular perception functions**.  
Task-specific workflows such as calibration or phantom experiments should be handled by dedicated launch files.

---

## 2. Package Structure

The package is organized into two levels:

### Core nodes
These nodes provide reusable low-level functions:

- `polaris_reader`  
  Reads tool poses from the Polaris tracking system.

- `ultrasound_reader`  
  Reads live ultrasound images from the video device.

- `us_dataset_player_node`  
  Replays recorded ultrasound datasets for offline testing.

- `us_stability_node`  
  Computes the ultrasound stability / reliability index from the input image.

- `us_target_estimator_node`  
  Estimates target position from ultrasound images.

### Launch files
These launch files combine the core nodes into complete workflows:

- `live_perception.launch.py`  
  For real-time Polaris + ultrasound input.

- `offline_replay.launch.py`  
  For dataset replay and offline debugging.

- `calibration.launch.py`  
  For calibration-related workflows.

- `phantom_experiment.launch.py`  
  For phantom experiment workflows.

---

## 3. Node Functions

### `polaris_reader`
Reads ROM-based tool tracking data from the Polaris system.

**Main outputs**
- `/polaris/phantom`
- `/polaris/stylus`
- `/polaris/raw`

### `ultrasound_reader`
Reads live ultrasound images from the selected video device.

**Main output**
- `/us_img`

### `us_dataset_player_node`
Publishes recorded ultrasound images and related pose data.

**Main outputs**
- `/us_img`
- `/probe_pose`

### `us_stability_node`
Computes the ultrasound stability index from the ultrasound image and ROI mask.

**Main output**
- `/us_stability`

### `us_target_estimator_node`
Estimates the target position from the ultrasound image stream.

---

## 4. Launch Functions

### `live_perception.launch.py`
Starts the basic real-time perception pipeline using live Polaris and ultrasound input.

Typical use:
- online experiment
- real-time debugging
- live target observation

### `offline_replay.launch.py`
Starts the perception pipeline using recorded datasets instead of live hardware.

Typical use:
- offline analysis
- debugging
- parameter tuning

### `calibration.launch.py`
Starts the nodes required for calibration workflows.

Typical use:
- tool calibration
- probe / phantom calibration
- data collection for calibration

### `phantom_experiment.launch.py`
Starts the nodes required for phantom experiments.

Typical use:
- structured phantom tests
- data recording under fixed experimental settings

---

## 5. How to Use

Before building or launching the package, first confirm that the hardware connection is correct.  
This is important because wrong serial or video device mapping will cause later steps inside the container to fail.

### Step 1: Check hardware connection on the host
Confirm that the required devices are detected correctly.

For Polaris serial device:
```bash
ls /dev/ttyUSB*
dmesg | grep ttyUSB
```

For ultrasound video device:
```bash
ls /dev/video*
v4l2-ctl --list-devices
```

Make sure:
- the Polaris serial port exists and matches the expected device
- the ultrasound device exists and matches the expected video id

### Step 2: Start the container
Build and enter the container after confirming the device mapping.

### Step 3: Build the workspace
```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

### Step 4: Run the required workflow
Use the launch file that matches your purpose.

For live perception:
```bash
ros2 launch polaris_ultrasound live_perception.launch.py
```

For offline replay:
```bash
ros2 launch polaris_ultrasound offline_replay.launch.py
```

For calibration:
```bash
ros2 launch polaris_ultrasound calibration.launch.py
```

For phantom experiment:
```bash
ros2 launch polaris_ultrasound phantom_experiment.launch.py
```

---

## 6. Quick Hardware and Topic Checks

After launching, confirm that data is being published correctly.

```bash
ros2 topic echo /polaris/phantom
ros2 topic echo /polaris/stylus
ros2 topic echo /us_img
ros2 topic echo /us_stability
ros2 topic echo /probe_pose
```

If no data appears:
- re-check the serial device for Polaris
- re-check the video device id for ultrasound
- confirm that the device mapping into the container is correct
- confirm that the selected launch file matches the intended workflow
