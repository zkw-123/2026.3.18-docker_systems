# 实时目标定位系统 / Realtime Target Localization System

## 📋 系统概述 (System Overview)

这是一个完整的模块化实时目标定位系统，结合超声图像和NDI光学追踪，实时计算目标的3D世界坐标。

This is a complete modular realtime target localization system that combines ultrasound imaging with NDI optical tracking to compute target 3D world coordinates in real-time.

---

## 🏗️ 系统架构 (System Architecture)

```
┌─────────────────────────────────────────────────────────────┐
│                    实时处理流水线                              │
└─────────────────────────────────────────────────────────────┘

US Camera ──→ [1.Ultrasound Publisher] ──→ /us_image_raw ──┐
              (30 Hz)                                       │
                                                           │
                                                           ↓
NDI Device ──→ [2.Polaris Publisher] ──→ /ndi_transforms ──→ [3.Synchronizer]
               (60+ Hz)                                           ↓
                                                         /synchronized_data
                                                           (22 Hz, 配对成功)
                                                                  ↓
                                                         [4.Target Detector]
                                                         (伪代码/可替换)
                                                                  ↓
                                                            /target_pixel
                                                                  ↓
                                                         [5.Target Localizer]
                                                         (3D坐标计算)
                                                                  ↓
                                                          /target_3d_position
                                                                  ↓
                                                         [6.Realtime Visualizer]
                                                         (实时可视化)

┌─────────────────────────────────────────────────────────────┐
│                    异步保存流水线（可选）                       │
└─────────────────────────────────────────────────────────────┘

/us_image_record ──┐
                   ├──→ [7.Data Recorder] ──→ 磁盘保存
/ndi_record ───────┘    (异步、不阻塞)
```

---

## 📦 节点说明 (Node Description)

### 1. ultrasound_publisher_node.py
**超声图像发布节点**

- **功能**: 从视频设备采集超声图像
- **发布**:
  - `/us_image_raw` (实时，BEST_EFFORT)
  - `/us_image_record` (录制，RELIABLE，仅当 `enable_recording=true` 时)
- **参数**:
  - `device_id`: 视频设备ID (default: 6)
  - `frame_rate`: 帧率 (default: 30.0)
  - `enable_recording`: 启用录制 (**default: false**)

### 2. polaris_publisher_node.py
**NDI追踪发布节点**

- **功能**: 从NDI Polaris设备读取追踪数据
- **发布**:
  - `/ndi_transforms` (实时)
  - `/ndi_transforms_record` (录制，仅当 `enable_recording=true` 时)
- **参数**:
  - `rom_paths`: ROM文件路径（逗号分隔）
  - `tool_names`: 工具名称（逗号分隔）
  - `serial_port`: 串口 (default: /dev/ttyUSB0)
  - `enable_recording`: 启用录制 (default: false)

### 3. data_synchronizer_node.py 
**数据同步节点（核心）**

- **功能**: 时间戳匹配、质量检查、配对数据
- **订阅**: `/us_image_raw`, `/ndi_transforms`
- **发布**: `/synchronized_data` (只发布配对成功的)
- **参数**:
  - `marker_name`: 目标marker名称
  - `time_tolerance`: 时间容差 (default: 0.005, 5ms)
  - `quality_threshold`: 质量阈值 (default: 0.5)
- **优势**: 节省28%的下游算力

### 4. target_detector_node.py 
**目标检测节点（可替换）**

- **功能**: 在图像中检测目标像素坐标
- **订阅**: `/synchronized_data`
- **发布**: `/target_pixel` (JSON: x, y, confidence)
- **参数**:
  - `detection_mode`: 检测模式 (center/moving/random)
- **当前实现**: 伪代码（易于替换）
- **替换方法**: 只需修改 `detect_target()` 方法

> ⚠️ **重要限制**: `/synchronized_data` 话题只携带图像元数据（宽高、编码、时间戳），不包含实际像素数据。当前节点在检测时使用的是全零的 mock 图像。替换为真实检测算法时，需要同时订阅 `/us_image_raw` 原始图像话题，并自行实现与 `/synchronized_data` 的时间戳匹配逻辑。

### 5. target_localizer_node.py
**3D定位节点**

- **功能**: 计算目标的3D世界坐标
- **订阅**: `/synchronized_data`, `/target_pixel`
- **发布**: `/target_3d_position` (JSON: x, y, z in mm)
- **参数**:
  - `pixel_to_mm_x/y`: 像素到mm转换
  - `image_origin_x/y`: 图像原点偏移 (mm, default: 0.0)
  - `marker_to_image_transform`: 标定矩阵(4x4，默认单位矩阵)
- **坐标变换链**: Pixel → Image → Marker → World

### 6. realtime_visualizer_node.py
**实时可视化节点**

- **功能**: 显示检测结果和3D位置
- **订阅**: `/us_image_raw`, `/target_pixel`, `/target_3d_position`
- **发布**: `/us_image_annotated` (仅当 `publish_annotated=true` 或 `display_mode=ros` 时)
- **显示模式**:
  - `local`: OpenCV窗口（最快）
  - `ros`: 发布标注图像到 `/us_image_annotated`
  - `none`: 不显示
- **参数**:
  - `window_name`: 窗口标题
  - `publish_annotated`: 是否同时发布标注图像 (default: false)
  - `font_scale`: 字体大小 (default: 0.6)
  - `line_thickness`: 线条粗细 (default: 2)
- **快捷键**: 按 'q' 或 ESC 退出

### 7. data_recorder_node.py（可选）
**数据录制节点**

- **功能**: 异步保存所有数据
- **订阅**: `/us_image_record`, `/ndi_transforms_record`
- **特点**: 完全独立，不影响实时性；仅在 `enable_recording:=true` 时由 launch 文件启动
- **保存**: 按时间戳命名的PNG和JSON文件

---

## 🚀 快速开始 (Quick Start)

### 1. 安装依赖 (Install Dependencies)

```bash
# ROS2依赖
sudo apt-get install ros-<distro>-cv-bridge ros-<distro>-message-filters

# Python依赖
pip install opencv-python numpy scikit-surgerynditracker
```

### 2. 配置参数 (Configure Parameters)

编辑 `launch/target_localization.launch.py`:

```python
# 设置您的设备
us_device_id = '6'          # 超声设备ID
ndi_rom_paths = '/path/to/probe.rom,/path/to/phantom.rom'  # ROM文件
ndi_tool_names = 'probe,phantom'  # 工具名称

# 设置标定参数（重要！）
pixel_to_mm_x = 0.1         # 根据标定结果修改
pixel_to_mm_y = 0.1
marker_to_image_transform = [...]  # 4x4标定矩阵（默认单位矩阵）
```

### 3. 启动系统 (Launch System)

```bash
# 不录制数据（最快）
ros2 launch target_localization target_localization.launch.py

# 启用数据录制
ros2 launch target_localization target_localization.launch.py \
  enable_recording:=true \
  save_path:=/path/to/save

# 自定义参数
ros2 launch target_localization target_localization.launch.py \
  us_device_id:=0 \
  marker_name:=probe \
  detection_mode:=moving \
  display_mode:=local
```

### 4. 监控系统 (Monitor System)

```bash
# 查看话题频率
ros2 topic hz /synchronized_data
ros2 topic hz /target_3d_position

# 查看实时3D位置
ros2 topic echo /target_3d_position

# 查看节点状态
ros2 node list

# 查看节点信息
ros2 node info /data_synchronizer
```

---

## 🔧 替换检测算法 (Replace Detection Algorithm)

检测算法完全独立，易于替换。**替换前请注意**：当前 `target_detector_node.py` 从 `/synchronized_data` 中只能获取图像元数据，不包含真实像素数据。替换为真实算法时，需要同时订阅 `/us_image_raw` 话题获取实际图像。

修改 `detect_target()` 方法：

### 示例1: 使用模板匹配

```python
def detect_target(self, image):
    """使用模板匹配检测"""
    template = cv2.imread('template.png', 0)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    result = cv2.matchTemplate(gray, template, cv2.TM_CCOEFF_NORMED)
    _, max_val, _, max_loc = cv2.minMaxLoc(result)
    
    if max_val > 0.7:
        h, w = template.shape
        x = max_loc[0] + w / 2
        y = max_loc[1] + h / 2
        return x, y, max_val
    
    return None, None, 0.0
```

### 示例2: 使用深度学习

```python
class TargetDetectorNode(Node):
    def __init__(self):
        super().__init__(...)
        # 加载您的模型
        self.model = YourDetectionModel()
        self.model.load_weights('model.pth')
    
    def detect_target(self, image):
        """使用深度学习检测"""
        detections = self.model.predict(image)
        
        if len(detections) > 0:
            best = detections[0]
            x, y = best['center']
            confidence = best['score']
            return x, y, confidence
        
        return None, None, 0.0
```

---

## 📊 性能优化 (Performance Optimization)

### 当前性能

```
处理延迟: ~45ms (端到端)
有效帧率: ~22 Hz (配对成功率 72%)
算力节省: 28% (相比处理所有帧)
```

### 优化建议

1. **降低图像分辨率**
   ```python
   image_width: 960  # 从1920降到960
   image_height: 540  # 从1080降到540
   ```

2. **调整同步参数**
   ```python
   time_tolerance: 0.01  # 放宽到10ms（提高配对率）
   quality_threshold: 0.3  # 降低阈值（更多数据通过）
   ```

3. **GPU加速检测**
   - 使用CUDA版本的OpenCV
   - 使用GPU推理（TensorRT/ONNX Runtime）

---

## 🎯 标定说明 (Calibration Guide)

系统需要两组标定参数：

### 1. 图像标定（像素到物理单位）

在超声图像中放置已知尺寸的标定物：

```python
# 测量方法
实际距离 = 50mm
像素距离 = 500px
pixel_to_mm = 50 / 500 = 0.1 mm/px
```

### 2. Marker到图像变换矩阵

使用手眼标定或点对点配准：

```python
# 需要使用专门的标定程序获取
# 建议使用calibration_tool.py
python3 calibration_tool.py
```

**重要**: 标定精度直接影响3D定位精度！默认参数为单位矩阵，必须替换为实际标定结果。

---

## 📁 文件结构 (File Structure)

```
target_localization/
├── target_localization/
│   ├── ultrasound_publisher_node.py      # 节点1
│   ├── polaris_publisher_node.py         # 节点2
│   ├── data_synchronizer_node.py         # 节点3 
│   ├── target_detector_node.py           # 节点4 
│   ├── target_localizer_node.py          # 节点5
│   ├── realtime_visualizer_node.py       # 节点6 ⚠️ 文件缺失，需补充
│   ├── data_recorder_node.py             # 节点7
│   ├── virtual_point_publisher_node.py   # 虚拟坐标发布（测试用）
│   ├── perception_replay_node.py         # 感知结果回放（仿真用）
│   ├── calibration_tool.py               # 标定工具
│   └── __init__.py
├── launch/
│   ├── target_localization.launch.py     # 主 Launch 文件
│   ├── test_publishers.launch.py
│   └── sync_test.launch.py
├── msg/
│   └── TargetSk.msg
├── package.xml
├── setup.py
├── setup.cfg
└── README.md
```

---

## 🐛 故障排除 (Troubleshooting)

### 问题1: 同步率很低

**现象**: `/synchronized_data` 频率远低于预期

**可能原因**:
- marker不可见
- 追踪质量低
- 时间戳不匹配

**解决方案**:
```bash
# 查看同步统计（统计信息通过 ROS 日志输出，不是独立话题）
ros2 run target_localization data_synchronizer_node.py --ros-args --log-level info

# 调整参数
quality_threshold: 0.3  # 降低质量要求
time_tolerance: 0.01    # 放宽时间容差
```

### 问题2: 可视化延迟大

**解决方案**:
- 关闭可视化: `display_mode:=none`
- 不发布标注图像: `publish_annotated:=false`
- 降低图像分辨率

### 问题3: NDI连接失败

**检查**:
```bash
# 检查设备
ls /dev/ttyUSB*

# 检查权限
sudo chmod 666 /dev/ttyUSB0

# 检查ROM文件
ls /path/to/rom/files
```

### 问题4: 检测不到目标

**当前是伪代码实现**，返回固定/移动/随机位置，且操作的是全零 mock 图像而非真实图像。

要实现真实检测:
1. 修改 `target_detector_node.py`，增加对 `/us_image_raw` 的订阅以获取真实图像数据
2. 修改 `detect_target()` 方法加入您的检测算法
3. 加载您的检测模型
4. 重新测试

### 问题5: realtime_visualizer_node 启动失败

**原因**: 包中缺少 `realtime_visualizer_node.py` 文件。

**解决方案**: 自行实现该节点，或在 launch 文件中将其注释掉（如不需要可视化，改用 `display_mode:=none` 并跳过该节点）。

---

## 📈 数据格式 (Data Format)

### /synchronized_data
```json
{
  "sync_timestamp": 1234567890.123456,
  "image_timestamp": 1234567890.123456,
  "ndi_timestamp": 1234567890.121000,
  "time_difference": 0.002456,
  "image": {
    "width": 1920,
    "height": 1080,
    "encoding": "bgr8"
  },
  "ndi": {
    "marker_name": "probe",
    "marker_quality": 0.95,
    "marker_transform": {
      "matrix": [[...], [...], [...], [...]],
      "translation": [x, y, z]
    }
  },
  "sync_quality": {
    "time_diff_ms": 2.5,
    "quality": 0.95,
    "frame_number": 42
  }
}
```

> ⚠️ **注意**: `/synchronized_data` 不包含图像像素数据，只包含图像元数据（宽高、编码等）。

### /target_pixel
```json
{
  "timestamp": 1234567890.123456,
  "pixel_x": 960.5,
  "pixel_y": 540.2,
  "confidence": 0.98,
  "image_width": 1920,
  "image_height": 1080,
  "detection_time_ms": 1.2,
  "frame_number": 42
}
```

### /target_3d_position
```json
{
  "timestamp": 1234567890.123456,
  "position": {
    "x": 150.5,
    "y": 200.3,
    "z": 50.7
  },
  "unit": "mm",
  "frame_id": "world",
  "computation_time_ms": 0.8,
  "frame_number": 42
}
```

---

## 🔬 应用场景 (Applications)

1. **手术导航**: 实时追踪手术器械位置
2. **超声引导穿刺**: 引导针头到达目标
3. **机器人辅助手术**: 为机器人提供实时定位
4. **运动分析**: 追踪和记录运动轨迹
5. **医学研究**: 数据采集和分析

---

## 📝 开发建议 (Development Tips)

### 调试模式

```bash
# 单独运行节点进行测试
ros2 run target_localization ultrasound_publisher_node.py
ros2 run target_localization data_synchronizer_node.py

# 查看详细日志
ros2 run target_localization target_detector_node.py --ros-args --log-level debug
```

### 性能分析

```bash
# 使用ros2 bag录制数据
ros2 bag record -a

# 回放测试
ros2 bag play your_bag.bag

# 查看延迟
ros2 topic delay /target_3d_position
```

---

## ⚠️ 注意事项 (Important Notes)

1. **标定是关键**: 系统精度完全依赖于标定质量，默认参数为单位矩阵，必须替换为实际标定结果
2. **NDI可见性**: 确保marker始终在追踪器视野内
3. **USB带宽**: 高分辨率+高帧率可能超过USB带宽
4. **实时性权衡**: 录制会略微影响性能，建议关键实验时关闭（`enable_recording` 默认为 `false`）
5. **检测算法**: 当前为伪代码，且操作 mock 图像；替换真实算法时须同时订阅 `/us_image_raw`
