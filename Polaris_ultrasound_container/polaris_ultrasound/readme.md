# polaris_ultrasound

## 1. Package Role

`polaris_ultrasound` provides reusable ROS 2 modules for:

- reading Polaris tracking data
- reading live ultrasound images
- replaying recorded ultrasound datasets
- computing ultrasound image stability
- estimating target position from ultrasound images

This package is organized around **modular perception functions**.  
Task-specific workflows such as calibration should be handled through dedicated launch files.

---

## 2. Package Structure

```text
polaris_ultrasound/
├── launch/
│   ├── live_perception.launch.py
│   ├── offline_replay.launch.py
│   └── calibration.launch.py
├── polaris_ultrasound/
│   ├── polaris_reader_node.py
│   ├── us_reader_node.py
│   ├── us_dataset_player_node.py
│   ├── us_stability_node.py
│   ├── us_target_estimator_node.py
│   ├── calibration_recorder.py
│   ├── draw_us_mask.py
│   └── client_node.py
├── package.xml
├── setup.py
└── README.md
```

### Core nodes
These nodes provide reusable low-level functions:

- `polaris_reader`  
  Reads tool poses from the Polaris tracking system.

- `ultrasound_reader`  
  Reads live ultrasound images from the video device.

- `us_dataset_player_node`  
  Replays recorded ultrasound datasets for offline testing.

- `us_stability_node`  
  Computes ultrasound image stability from the input image.

- `us_target_estimator_node`  
  Estimates target position from ultrasound images.

### Workflow-level node
- `calibration_recorder`  
  Used only in the calibration workflow.

---

## 3. Core Node Functions

### `polaris_reader`
Reads tool tracking data from the Polaris system and publishes the result as a JSON string topic.

**Main parameter**
- `rom_path`
- `tool_names`
- `serial_port`
- `output_topic`

**Main output**
- `ndi_transforms`

---

### `ultrasound_reader`
Reads real-time ultrasound images from the selected video device.

**Main parameter**
- `device_id`
- `image_topic`
- `roi_topic`
- `publish_roi`
- `frame_rate`

**Main outputs**
- `/us_img`
- `/us_roi` (optional)

---

### `us_dataset_player_node`
Replays recorded ultrasound image datasets and optional probe pose data.

**Main parameter**
- `data_dir`
- `json_mode`
- `json_dir`
- `json_file`
- `fps`
- `loop`

**Main outputs**
- `/us_img`
- `/probe_pose`

---

### `us_stability_node`
Computes a stability index from the ultrasound image inside the selected ROI.

**Main parameter**
- `image_topic`
- `stability_topic`
- `mask_path`
- `alpha`
- `c_sp_max`
- `c_grad_max`

**Main output**
- `/us_stability`

---

### `us_target_estimator_node`
Estimates a target position from the ultrasound image and optionally transforms it into another coordinate frame.

**Main parameter**
- `image_topic`
- `target_topic`
- `debug_image_topic`
- `mask_path`
- `transform_json_path`

**Main outputs**
- `/target_point`
- `/us_target_debug`

---

## 4. Launch Files

### `live_perception.launch.py`
Starts the online perception pipeline using:

- Polaris tracking
- live ultrasound input
- stability computation
- target estimation

Use this launch for:
- real-time experiments
- live perception debugging
- online target observation

---

### `offline_replay.launch.py`
Starts the offline perception pipeline using:

- recorded ultrasound image dataset
- optional recorded probe pose
- stability computation
- target estimation

Use this launch for:
- offline analysis
- debugging
- parameter tuning

---

### `calibration.launch.py`
Starts the calibration workflow using:

- Polaris tracking
- calibration recorder

Use this launch for:
- calibration data collection
- synchronized ultrasound and tracking recording

---

## 5. Before Use: Check Hardware Connection First

Before building or launching the package, first confirm that the hardware connection is correct.  
This should be done on the host side before entering the container, otherwise later debugging becomes unnecessarily difficult.

### Check Polaris serial device
```bash
ls /dev/ttyUSB*
dmesg | grep ttyUSB
```

### Check ultrasound video device
```bash
ls /dev/video*
v4l2-ctl --list-devices
```

Make sure:
- the Polaris serial port exists and matches the expected device
- the ultrasound device exists and matches the expected video id

---

## 6. Build

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

---

## 7. How to Use

### 7.1 Live perception
```bash
ros2 launch polaris_ultrasound live_perception.launch.py
```

### 7.2 Offline replay
```bash
ros2 launch polaris_ultrasound offline_replay.launch.py
```

### 7.3 Calibration
```bash
ros2 launch polaris_ultrasound calibration.launch.py
```

---

## 8. Quick Checks After Launch

Use the following commands to confirm that the nodes are working correctly.

```bash
ros2 topic echo /us_img
ros2 topic echo /us_stability
ros2 topic echo /probe_pose
ros2 topic echo /target_point
ros2 topic echo ndi_transforms
```

If no data appears:
- re-check the serial device for Polaris
- re-check the video device id for ultrasound
- confirm that the device mapping into the container is correct
- confirm that the selected launch file matches the intended workflow

---

## 9. Notes

- `polaris_ultrasound` is intended to provide reusable perception modules.
- Task-specific workflows should be handled by launch files, not by mixing workflow logic into the core node layer.
- `draw_us_mask.py` is a utility script and is not part of the core runtime pipeline.
