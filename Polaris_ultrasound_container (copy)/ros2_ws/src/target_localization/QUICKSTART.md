# 快速使用指南 / Quick Start Guide

## 📦 文件清单 (File List)

### 核心节点 (Core Nodes) - 7个
1. `ultrasound_publisher_node.py` - 超声图像发布
2. `polaris_publisher_node.py` - NDI追踪发布  
3. `data_synchronizer_node.py` - 数据同步 ⭐
4. `target_detector_node.py` - 目标检测 (可替换)
5. `target_localizer_node.py` - 3D定位
6. `realtime_visualizer_node.py` - 实时可视化
7. `data_recorder_node.py` - 数据录制 (可选)

### Launch文件
- `target_localization_launch.py` - 主启动文件

### 工具
- `calibration_tool.py` - 标定工具

### 文档
- `README.md` - 完整文档

## 🚀 三步启动 (3-Step Launch)

### 步骤1: 修改配置

编辑 `target_localization_launch.py`，修改以下参数：

```python
# 第52-56行：设备配置
'us_device_id', default_value='YOUR_CAMERA_ID'  # 改成你的摄像头ID
'ndi_rom_paths', default_value='/path/to/YOUR.rom'  # 改成你的ROM路径
'ndi_tool_names', default_value='YOUR_TOOL_NAME'  # 改成你的工具名

# 第195-202行：标定参数  
'pixel_to_mm_x': YOUR_VALUE,  # 改成标定值
'marker_to_image_transform': [YOUR_MATRIX]  # 改成标定矩阵
```

### 步骤2: 复制到ROS2工作空间

```bash
# 创建包
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python your_package_name

# 复制文件
cp *.py ~/ros2_ws/src/your_package_name/your_package_name/

# 编译
cd ~/ros2_ws
colcon build --packages-select your_package_name
source install/setup.bash
```

### 步骤3: 启动

```bash
# 基础启动（不录制）
ros2 launch your_package_name target_localization_launch.py

# 完整启动（含录制）
ros2 launch your_package_name target_localization_launch.py \
  enable_recording:=true \
  save_path:=/tmp/tracking_data
```

## 🎯 测试单个节点

```bash
# 测试超声发布
ros2 run your_package_name ultrasound_publisher_node.py

# 测试同步器
ros2 run your_package_name data_synchronizer_node.py

# 查看话题
ros2 topic list
ros2 topic echo /target_3d_position
```

## ⚠️ 重要提示

1. **必须修改配置**: launch文件中的路径需要改成你的实际路径
2. **需要标定**: `marker_to_image_transform` 必须通过标定获得
3. **检测算法**: 当前是伪代码，需要替换为真实算法
4. **权限问题**: 确保有USB设备访问权限

```bash
# 解决USB权限
sudo chmod 666 /dev/ttyUSB0
sudo chmod 666 /dev/video6
```

## 📊 监控命令

```bash
# 查看所有节点
ros2 node list

# 查看话题频率
ros2 topic hz /synchronized_data

# 查看3D位置
ros2 topic echo /target_3d_position

# 查看日志
ros2 log level data_synchronizer debug
```

## 🔧 常见问题

### Q: 找不到设备
A: 检查设备ID和权限
```bash
ls /dev/video* /dev/ttyUSB*
sudo chmod 666 /dev/video* /dev/ttyUSB*
```

### Q: 同步率很低
A: 调整参数
```python
time_tolerance: 0.01  # 放宽到10ms
quality_threshold: 0.3  # 降低阈值
```

### Q: 如何替换检测算法
A: 修改 `target_detector_node.py` 的 `detect_target()` 方法

## 📖 详细文档

查看 `README.md` 获取完整文档。

祝使用顺利！🎉
