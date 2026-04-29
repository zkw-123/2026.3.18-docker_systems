# carryover_validation

> A ROS 2 package for open-loop carry-over model validation and online RLS estimation of the lateral carry-over coefficient `beta_perp`.

---

## 1. Overview

`carryover_validation` is an independent experiment package used to validate the carry-over model during robotic tissue extraction.

The package executes known lateral cutter steps `d_k` during nominal axial insertion, records the resulting target displacement from the NDI-ultrasound perception system, and estimates the lateral carry-over coefficient `beta_perp` online using recursive least squares (RLS).

The basic experiment flow is:

```text
known lateral cutter step d_k
        ↓
robot executes open-loop lateral correction during axial insertion
        ↓
target displacement is measured by the NDI-ultrasound system
        ↓
RLS estimates beta_perp online
        ↓
prediction error is evaluated step by step
```

This package does **not** run the closed-loop MPC controller. It only reuses:

```text
/perception/target_point
/robot_current_pose
/robot_command
```

Do not run the MPC command bridge at the same time, otherwise multiple nodes may publish to `/robot_command`.

---

## 2. Package Structure and Module Responsibilities

```text
carryover_validation/
├── carryover_validation/
│   ├── __init__.py
│   ├── validation_runner_node.py
│   ├── rls_estimator_node.py
│   ├── target_point_adapter_node.py
│   ├── experiment_logger_node.py
│   └── offline_analyzer.py
├── config/
│   └── carryover_validation.yaml
├── launch/
│   └── carryover_validation.launch.py
├── resource/
│   └── carryover_validation
├── package.xml
├── setup.py
├── setup.cfg
└── README.md
```

### `validation_runner`

Top-level experiment runner.

It executes the open-loop validation sequence. At each step, it reads the current robot pose, adds a known lateral correction `d_k`, adds a nominal axial insertion increment, and publishes a new Cartesian command to `/robot_command`.

The command format is:

```text
move x y z
```

The lateral step is therefore not sent as a separate robot command. It is converted into a new Cartesian target:

```text
x_new = x_current + d_k        # if lateral_axis = x
z_new = z_current + axial_increment
```

### `rls_estimator`

Online scalar RLS estimator for `beta_perp`.

The implemented model is:

```text
measured_delta_k = beta_perp * d_k + noise
```

The main prediction error is computed before the current RLS update:

```text
prediction_error_before = measured_delta_k - beta_before * d_k
```

This is the value used to evaluate predictive accuracy.

### `target_point_adapter`

Optional adapter from JSON target position to `geometry_msgs/msg/PointStamped`.

Typical conversion:

```text
/target_3d_position        std_msgs/msg/String, JSON, mm
        ↓
/perception/target_point   geometry_msgs/msg/PointStamped, m
```

Disable this node if `/perception/target_point` is already available.

### `experiment_logger`

Step-level CSV logger.

It records one row per lateral correction step using `/carryover/rls_update`.

The default output directory is:

```text
/workspace/logs/carryover_validation
```

### `offline_analyzer`

Offline CSV analyzer.

It computes:

```text
steady_state_beta
steady_state_beta_std
prediction_error_before_rmse
prediction_error_before_mae
prediction_error_before_max
convergence_step
```

---

## 3. Runtime Interfaces

### 3.1 Runtime Dependencies

Ensure the following are available before launching this package:

| Requirement | Description |
|---|---|
| Robot command node | Must subscribe to `/robot_command` |
| Robot status publisher | Must publish `/robot_current_pose` |
| Perception system | Must publish `/perception/target_point` or `/target_3d_position` |
| Workspace sourced | `source install/setup.bash` |
| MPC command bridge stopped | No other node should publish to `/robot_command` |

If `/target_3d_position` is used instead of `/perception/target_point`, keep `enable_adapter:=true`.

---

### 3.2 Topics

#### `validation_runner`

| Direction | Topic | Type | Description |
|---|---|---|---|
| Subscribed | `/robot_current_pose` | `geometry_msgs/msg/PoseStamped` | Current cutter/EEF pose |
| Subscribed | `/perception/target_point` | `geometry_msgs/msg/PointStamped` | Current target position in meters |
| Published | `/robot_command` | `std_msgs/msg/String` | Robot command, e.g. `move x y z` |
| Published | `/carryover/step_event` | `std_msgs/msg/String` | JSON event for each validation step |
| Published | `/carryover/dk_cmd` | `std_msgs/msg/Float64` | Current lateral step `d_k` |
| Published | `/carryover/step_index` | `std_msgs/msg/Int32` | Current step index |
| Published | `/carryover/state` | `std_msgs/msg/String` | Runner state |

#### `rls_estimator`

| Direction | Topic | Type | Description |
|---|---|---|---|
| Subscribed | `/carryover/step_event` | `std_msgs/msg/String` | JSON step event from runner |
| Published | `/carryover/beta_perp` | `std_msgs/msg/Float64` | Updated carry-over coefficient |
| Published | `/carryover/measured_delta` | `std_msgs/msg/Float64` | Measured lateral target displacement |
| Published | `/carryover/predicted_delta_before` | `std_msgs/msg/Float64` | Prediction using beta before update |
| Published | `/carryover/prediction_error_before` | `std_msgs/msg/Float64` | One-step prediction error |
| Published | `/carryover/predicted_delta_after` | `std_msgs/msg/Float64` | Prediction using beta after update |
| Published | `/carryover/prediction_error_after` | `std_msgs/msg/Float64` | Post-update fitting error |
| Published | `/carryover/rls_update` | `std_msgs/msg/String` | JSON update result for logging |

#### `target_point_adapter`

| Direction | Topic | Type | Description |
|---|---|---|---|
| Subscribed | `/target_3d_position` | `std_msgs/msg/String` | JSON target position, usually in mm |
| Published | `/perception/target_point` | `geometry_msgs/msg/PointStamped` | Target point in meters |

#### `experiment_logger`

| Direction | Topic | Type | Description |
|---|---|---|---|
| Subscribed | `/carryover/rls_update` | `std_msgs/msg/String` | RLS update result |
| Subscribed | `/carryover/step_event` | `std_msgs/msg/String` | Optional raw step event logging |

---

### 3.3 Parameters

#### `validation_runner`

| Parameter | Default | Description |
|---|---:|---|
| `num_steps` | `20` | Number of lateral correction steps |
| `substrate_name` | `substrate_A` | Name of current substrate |
| `lateral_axis` | `x` | Axis used for lateral correction: `x`, `y`, or `z` |
| `axial_axis` | `z` | Axis used for axial insertion |
| `axial_direction_sign` | `-1.0` | Sign of insertion direction |
| `dk_sequence_mode` | `alternating` | Step sequence mode: `alternating`, `positive`, `negative`, or `zero` |
| `dk_magnitude_m` | `0.0005` | Magnitude of lateral step in meters |
| `dk_sequence_m_csv` | empty | Optional explicit sequence, e.g. `0.0005,-0.0005,0.001` |
| `axial_speed_mps` | `0.0005` | Nominal axial insertion speed |
| `step_interval_s` | `2.0` | Time interval per validation step |
| `settle_time_s` | `0.5` | Waiting time after motion command before recording target_after |
| `start_delay_s` | `2.0` | Delay before starting experiment |
| `robot_command_topic` | `/robot_command` | Robot command topic |
| `robot_pose_topic` | `/robot_current_pose` | Current robot pose topic |
| `target_point_topic` | `/perception/target_point` | Target point topic |
| `command_mode` | `move` | Command mode. Current implementation supports `move` |

#### `rls_estimator`

| Parameter | Default | Description |
|---|---:|---|
| `beta0` | `0.0` | Initial value of `beta_perp` |
| `P0` | `1000.0` | Initial scalar covariance |
| `forgetting_factor` | `0.99` | RLS forgetting factor |
| `min_abs_dk_m` | `1.0e-5` | Skip update if `abs(d_k)` is smaller than this |
| `lateral_axis` | `x` | Axis used for measured lateral displacement |
| `enable_beta_clamp` | `false` | Enable beta range clamp |
| `beta_min` | `-2.0` | Minimum beta if clamp is enabled |
| `beta_max` | `2.0` | Maximum beta if clamp is enabled |

#### `target_point_adapter`

| Parameter | Default | Description |
|---|---:|---|
| `input_topic` | `/target_3d_position` | Input JSON target topic |
| `output_topic` | `/perception/target_point` | Output PointStamped topic |
| `frame_id` | `robot_base_link` | Frame ID for output target point |
| `input_unit` | `mm` | Input unit: `mm` or `m` |

#### `experiment_logger`

| Parameter | Default | Description |
|---|---:|---|
| `output_dir` | `/workspace/logs/carryover_validation` | CSV output directory |
| `file_prefix` | `carryover_validation` | CSV filename prefix |
| `substrate_name` | `substrate_A` | Substrate name added to filename |
| `log_raw_step_event` | `false` | If true, also log raw step events |

---

### 3.4 Launch Arguments

The main launch file is:

```text
launch/carryover_validation.launch.py
```

Supported launch arguments:

| Launch argument | Default | Description |
|---|---:|---|
| `config_file` | package config YAML | Path to YAML config file |
| `enable_adapter` | `true` | Start `target_point_adapter` |
| `substrate_name` | `substrate_A` | Substrate name |
| `num_steps` | `20` | Number of validation steps |
| `dk_magnitude_m` | `0.0005` | Lateral step magnitude |
| `axial_speed_mps` | `0.0005` | Axial insertion speed |
| `step_interval_s` | `2.0` | Time interval per step |
| `settle_time_s` | `0.5` | Settling time after command |
| `lateral_axis` | `x` | Lateral correction axis |
| `axial_axis` | `z` | Axial insertion axis |
| `axial_direction_sign` | `-1.0` | Insertion direction sign |
| `output_dir` | `/workspace/logs/carryover_validation` | CSV output directory |

---

## 4. Build and Running

### 4.1 Build

From the workspace root:

```bash
cd /workspace
colcon build --packages-select carryover_validation
source install/setup.bash
```

If building together with other packages:

```bash
cd /workspace
colcon build
source install/setup.bash
```

---

### 4.2 Start Robot System

Start the DENSO robot system first.

Example:

```bash
ros2 launch denso_robot_bringup denso_robot_bringup.launch.py model:=vp6242 sim:=false
```

Start the robot command interface if required:

```bash
ros2 launch control_command robot_control_with_status.launch.py
```

Check required robot topics:

```bash
ros2 topic list | grep robot_command
ros2 topic list | grep robot_current_pose
```

---

### 4.3 Start Perception System

Start the Polaris-ultrasound perception pipeline.

The validation package requires one of the following:

```text
/perception/target_point
```

or:

```text
/target_3d_position
```

If only `/target_3d_position` is available, use the adapter:

```bash
ros2 launch carryover_validation carryover_validation.launch.py enable_adapter:=true
```

If `/perception/target_point` is already published, disable the adapter:

```bash
ros2 launch carryover_validation carryover_validation.launch.py enable_adapter:=false
```

---

### 4.4 Run Carry-over Validation

Default run:

```bash
ros2 launch carryover_validation carryover_validation.launch.py
```

Run with substrate name:

```bash
ros2 launch carryover_validation carryover_validation.launch.py \
  substrate_name:=substrate_A
```

Run with modified lateral step magnitude:

```bash
ros2 launch carryover_validation carryover_validation.launch.py \
  substrate_name:=substrate_A \
  dk_magnitude_m:=0.001 \
  num_steps:=20
```

Run with lateral correction along Y:

```bash
ros2 launch carryover_validation carryover_validation.launch.py \
  lateral_axis:=y
```

Run for the second substrate:

```bash
ros2 launch carryover_validation carryover_validation.launch.py \
  substrate_name:=substrate_B
```

---

### 4.5 Inspect Runtime Topics

Check runner state:

```bash
ros2 topic echo /carryover/state
```

Check lateral step command:

```bash
ros2 topic echo /carryover/dk_cmd
```

Check step event:

```bash
ros2 topic echo /carryover/step_event
```

Check RLS output:

```bash
ros2 topic echo /carryover/beta_perp
ros2 topic echo /carryover/prediction_error_before
ros2 topic echo /carryover/rls_update
```

Check generated CSV:

```bash
ls /workspace/logs/carryover_validation
```

---

### 4.6 Offline Analysis

Run:

```bash
ros2 run carryover_validation offline_analyzer \
  --input /workspace/logs/carryover_validation/<csv_file>
```

Save summary:

```bash
ros2 run carryover_validation offline_analyzer \
  --input /workspace/logs/carryover_validation/<csv_file> \
  --output /workspace/logs/carryover_validation/summary_substrate_A.csv
```

The analyzer reports:

```text
steady_state_beta
steady_state_beta_std
prediction_error_before_rmse
prediction_error_before_mae
prediction_error_before_max
convergence_step
```

---

## 5. Notes

### Do not run MPC command bridge

Do not run the following nodes during this experiment:

```bash
ros2 run sk_mpc_controller mpc_node
ros2 run sk_mpc_controller mpc_node_2d
ros2 run sk_mpc_controller mpc_to_robot_command_bridge
ros2 run sk_mpc_controller mpc_to_robot_command_bridge_2d
```

Otherwise, `/robot_command` may receive commands from multiple sources.

---

### Current package does not use Servo directly

The current validation runner publishes:

```text
std_msgs/msg/String → /robot_command
```

with command format:

```text
move x y z
```

It does not publish directly to:

```text
/servo_node/delta_twist_cmds
```

Whether `move x y z` is executed through MoveIt planning or Servo depends on the downstream robot command package, not on `carryover_validation`.

---

### Current RLS model does not explicitly use force

The current RLS model is:

```text
measured_delta_k = beta_perp * d_k + noise
```

No force/torque topic is subscribed in the current implementation.

The interaction force is therefore not an explicit regressor. Its effect is implicitly included in the estimated local coefficient `beta_perp`.

If force-aware validation is needed later, add a `geometry_msgs/msg/WrenchStamped` subscription and extend the regression model, for example:

```text
measured_delta_k = beta_0 * d_k + beta_F * |F_n,k| * d_k + noise
```

---

### Unit convention

The validation package assumes:

```text
robot pose: meters
/perception/target_point: meters
/target_3d_position: millimeters by default
```

If `/target_3d_position` is already in meters, set:

```yaml
input_unit: "m"
```

---

### Direction convention

The default insertion configuration is:

```yaml
axial_axis: "z"
axial_direction_sign: -1.0
```

If the actual insertion direction is opposite, use:

```yaml
axial_direction_sign: 1.0
```

The default lateral correction axis is:

```yaml
lateral_axis: "x"
```

Use `lateral_axis:=y` if the lateral correction should be along the robot base Y direction.

---

### Fair comparison between substrates

Use the same parameters for both substrates:

```text
num_steps
dk_magnitude_m
dk_sequence_mode
axial_speed_mps
step_interval_s
settle_time_s
beta0
P0
forgetting_factor
```

The most important validation metric is:

```text
prediction_error_before_m
```

because it is computed using `beta_perp` before the current RLS update.