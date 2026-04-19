# polaris_ultrasound

## 1. Package Role

`polaris_ultrasound` is a ROS 2 package for:

- reading Polaris tracking data
- reading ultrasound images
- recording calibration data
- replaying recorded ultrasound datasets
- computing ultrasound stability index `Sk`
- estimating target position in ultrasound images

This package provides the basic perception-side data input for ultrasound-guided experiments.

---

## 2. Main Nodes

### `polaris_reader`
Reads tracking data from the Polaris system.

**Inputs**
- ROM files
- tool names

**Outputs**
- `/polaris/phantom`
- `/polaris/stylus`
- `/polaris/raw`

---

### `ultrasound_reader`
Reads real-time ultrasound images from the video device.

**Output**
- `/us_img`

---

### `calibration_recorder`
Records calibration-related data during experiments.

---

### `phantom_experiment`
Collects stylus / phantom point data for calibration experiments.

**Save path**
- `/ros2_ws/calibration_data/polaris/stylus_phantom/`

---

### `us_dataset_player_node`
Replays recorded ultrasound datasets instead of using a live ultrasound device.

**Outputs**
- `/us_img`
- `/probe_pose`

Use this node for offline testing and debugging.

---

### `us_stability_node`
Computes ultrasound stability index `Sk` from the ultrasound image and ROI mask.

**Inputs**
- ultrasound image topic
- ROI mask

**Output**
- `/us_stability`

---

### `us_target_estimator_node`
Estimates the target position from ultrasound images.

---

## 3. Build

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

---

## 4. Typical Usage

### 4.1 Real-time experiment

**Step 1: Start Polaris reader**
```bash
ros2 run polaris_ultrasound polaris_reader --ros-args \
  -p rom_path:="/opt/ndi/rom_files/8700449_phantom.rom,/opt/ndi/rom_files/8700340_stylus.rom" \
  -p tool_names:="phantom,stylus"
```

**Step 2: Start ultrasound reader**
```bash
ros2 run polaris_ultrasound ultrasound_reader --ros-args \
  -p device_id:=6
```

**Step 3: Start stability estimation if needed**
```bash
ros2 run polaris_ultrasound us_stability_node --ros-args \
  -p image_topic:=/us_img \
  -p stability_topic:=/us_stability \
  -p mask_path:=/ros2_ws/src/silicon_exp1/us_mask.png \
  -p alpha:=0.3 \
  -p c_sp_max:=17.6 \
  -p c_grad_max:=7.1
```

**Step 4: Start target estimation if needed**
```bash
ros2 run polaris_ultrasound us_target_estimator_node
```

---

### 4.2 Offline dataset test

**Step 1: Replay dataset**
```bash
ros2 run polaris_ultrasound us_dataset_player_node --ros-args \
  -p data_dir:=/ros2_ws/src/silicon_exp1/20250810_072526 \
  -p fps:=10.0 \
  -p json_mode:=fixed_dir \
  -p json_dir:=/ros2_ws/src/silicon_exp1 \
  -p loop:=false
```

**Step 2: Run stability estimation if needed**
```bash
ros2 run polaris_ultrasound us_stability_node --ros-args \
  -p image_topic:=/us_img \
  -p stability_topic:=/us_stability \
  -p mask_path:=/ros2_ws/src/silicon_exp1/us_mask.png \
  -p alpha:=0.3 \
  -p c_sp_max:=17.6 \
  -p c_grad_max:=7.1
```

**Step 3: Run target estimation if needed**
```bash
ros2 run polaris_ultrasound us_target_estimator_node
```

---

## 5. Calibration Data Collection

Run:

```bash
ros2 run polaris_ultrasound phantom_experiment
```

The recorded data will be saved to:

```bash
/ros2_ws/calibration_data/polaris/stylus_phantom/
```

---

## 6. ROI Mask Tool

Use `draw_us_mask.py` to create the ROI mask for `us_stability_node`.

```bash
python3 draw_us_mask.py \
  --image /ros2_ws/src/silicon_exp1/20250810_072526/us_000001.png \
  --output /ros2_ws/src/silicon_exp1/us_mask.png
```

**Controls**
- Left click: add vertex
- Right click: close polygon
- `r`: reset
- `s`: save
- `ESC`: quit

---

## 7. Important Parameters

### `polaris_reader`
- `rom_path`: Polaris ROM file paths
- `tool_names`: tool names corresponding to the ROM files

### `ultrasound_reader`
- `device_id`: ultrasound video device id

### `us_dataset_player_node`
- `data_dir`: dataset directory
- `fps`: replay frame rate
- `json_mode`: `paired` / `fixed_dir` / `fixed_single`
- `json_dir`: JSON directory
- `loop`: whether to replay repeatedly

### `us_stability_node`
- `image_topic`: input ultrasound image topic
- `stability_topic`: output stability topic
- `mask_path`: ROI mask path
- `alpha`: smoothing factor
- `c_sp_max`: normalization upper bound for intensity variation
- `c_grad_max`: normalization upper bound for gradient magnitude

---

## 8. Quick Checks

```bash
ros2 topic echo /polaris/phantom
ros2 topic echo /polaris/stylus
ros2 topic echo /us_img
ros2 topic echo /us_stability
ros2 topic echo /probe_pose
```

---

## 9. Notes

- Use `ultrasound_reader` for live ultrasound input.
- Use `us_dataset_player_node` for offline replay.
- Use `us_stability_node` only when `Sk` is needed.
- Use `phantom_experiment` for stylus / phantom point collection.
- Rebuild the workspace after modifying the package.
