# polaris_ultrasound

## 1. Package Role

`polaris_ultrasound` provides reusable ROS 2 modules for:

- reading Polaris tracking data
- reading live ultrasound images
- replaying recorded ultrasound datasets
- computing ultrasound image stability
- estimating target position from ultrasound images
- recording synchronized ultrasound and Polaris data for calibration

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
  Records synchronized ultrasound and Polaris data for calibration.

---

## 3. Core Node Functions

### `polaris_reader`
Reads tool tracking data from the Polaris system and publishes the result as a JSON string topic.

**Main parameters**
- `rom_path`
- `tool_names`
- `serial_port`
- `output_topic`
- `publish_rate`

**Main output**
- `ndi_transforms`

**Notes**
- `rom_path` should be given as a comma-separated string.
- `tool_names` should also be given as a comma-separated string in the same order.

---

### `ultrasound_reader`
Reads real-time ultrasound images from the selected video device.

**Main parameters**
- `device_id`
- `image_topic`
- `roi_topic`
- `publish_roi`
- `frame_rate`
- `show_image`

**Main outputs**
- `/us_img`
- `/us_roi` (optional)

---

### `us_dataset_player_node`
Replays recorded ultrasound image datasets and optional probe pose data.

**Main parameters**
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

**Main parameters**
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

**Main parameters**
- `image_topic`
- `target_topic`
- `debug_image_topic`
- `mask_path`
- `transform_json_path`

**Main outputs**
- `/target_point`
- `/us_target_debug`

---

### `calibration_recorder`
Records ultrasound frames and matched Polaris transforms for calibration.

**Main parameters**
- `device_id`
- `save_path`
- `frame_rate`
- `tool_names`
- `record_time`
- `polaris_topic`

**Main behavior**
- saves ultrasound images under `ultrasound/<session_timestamp>/`
- saves Polaris transforms under `polaris/<session_timestamp>/`
- prints recording progress during runtime
- prints Polaris status during runtime so marker validity can be checked

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

## 5. Before Running

Before building or launching the package, first confirm that the hardware connection and user permissions are correct.

### 5.1 Check Polaris serial device
On the host:

```bash
ls /dev/ttyUSB*
sudo dmesg | grep ttyUSB
```

### 5.2 Check ultrasound video device
On the host:

```bash
ls /dev/video*
v4l2-ctl --list-devices
```

### 5.3 Check serial permission inside the container
Inside the container:

```bash
ls -l /dev/ttyUSB0
id dockeruser
getent group dialout
```

`dockeruser` should be in the `dialout` group.

If not, add it:

```bash
sudo usermod -aG dialout dockeruser
```

Then open a new shell or re-enter the container.

### 5.4 Confirm ROM file paths
Before launching, make sure the ROM files really exist inside the container:

```bash
ls -l /workspace/src/rom_files
```

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

## 7.1 Live perception

### Command
```bash
ros2 launch polaris_ultrasound live_perception.launch.py \
  rom_path:=/workspace/src/rom_files/8700449_phantom.rom,/workspace/src/rom_files/8700340_stylus.rom \
  tool_names:=phantom,stylus \
  device_id:=0 \
  transform_json_path:=/workspace/config/ultrasound_robot_transform.json
```

### Main launch arguments
- `rom_path`  
  Comma-separated ROM file paths

- `tool_names`  
  Comma-separated tool names corresponding to `rom_path`

- `device_id`  
  Ultrasound video device id

- `transform_json_path`  
  4x4 transform json used by the target estimator

- `mask_path`  
  Optional ROI mask path

- `alpha`, `c_sp_max`, `c_grad_max`  
  Stability computation parameters

---

## 7.2 Offline replay

### Command
```bash
ros2 launch polaris_ultrasound offline_replay.launch.py \
  data_dir:=/workspace/data/sample_dataset \
  json_mode:=fixed_dir \
  json_dir:=/workspace/data/sample_pose \
  loop:=false \
  mask_path:=/workspace/data/us_mask.png \
  transform_json_path:=/workspace/config/ultrasound_robot_transform.json
```

### Main launch arguments
- `data_dir`  
  Directory containing replay images

- `json_mode`  
  `paired`, `fixed_dir`, or `fixed_single`

- `json_dir`  
  Directory containing pose json files when `json_mode=fixed_dir`

- `json_file`  
  Single json file when `json_mode=fixed_single`

- `fps`
- `loop`
- `mask_path`
- `transform_json_path`

---

## 7.3 Calibration

### Command
```bash
ros2 launch polaris_ultrasound calibration.launch.py \
  rom_path:=/workspace/src/rom_files/8700449_phantom.rom,/workspace/src/rom_files/8700340_stylus.rom \
  tool_names:=phantom,stylus \
  device_id:=0 \
  save_path:=/workspace/data/calibration_data \
  frame_rate:=30.0 \
  record_time:=60.0
```

### Main launch arguments
- `rom_path`
- `tool_names`
- `device_id`
- `save_path`
- `frame_rate`
- `record_time`

### Runtime behavior
During calibration, the recorder will:
- print recording progress
- print Polaris tool status
- only save Polaris frames when all requested tools are present and valid

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
- confirm that `dockeruser` has `dialout`
- confirm that the ROM file paths are correct
- confirm that the selected launch file matches the intended workflow

---

## 9. Notes

- `polaris_ultrasound` is intended to provide reusable perception modules.
- Task-specific workflows should be handled by launch files, not by mixing workflow logic into the core node layer.
- `draw_us_mask.py` is a utility script and is not part of the core runtime pipeline.
- `tool_names` and `rom_path` should always use the same order.
