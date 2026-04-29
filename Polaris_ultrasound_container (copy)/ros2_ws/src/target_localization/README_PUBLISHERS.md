# Publisher Nodes 使用说明

这是实时目标定位系统的第一部分：数据采集发布节点。

## 📦 包含的文件

1. **ultrasound_publisher_node.py** - 超声图像发布节点
2. **polaris_publisher_node.py** - NDI Polaris追踪数据发布节点
3. **test_publishers_launch.py** - 测试用launch文件

## 🎯 设计特点

### 核心特性

✅ **极简设计** - 只负责采集和发布，不做任何处理  
✅ **双发布策略** - 实时话题 + 录制话题  
✅ **精确时间戳** - 采集后立即打戳  
✅ **QoS优化** - 实时话题用BEST_EFFORT，录制话题用RELIABLE  
✅ **统计监控** - 实时FPS、成功率等统计信息  

### 话题说明

| 话题名 | 消息类型 | QoS | 用途 |
|--------|----------|-----|------|
| `/us_image_raw` | sensor_msgs/Image | BEST_EFFORT, depth=1 | 实时处理 |
| `/us_image_record` | sensor_msgs/Image | RELIABLE, depth=100 | 数据保存 |
| `/ndi_transforms` | std_msgs/String | BEST_EFFORT, depth=1 | 实时处理 |
| `/ndi_transforms_record` | std_msgs/String | RELIABLE, depth=100 | 数据保存 |

## 🚀 快速开始

### 1. 安装依赖

```bash
# ROS2依赖
sudo apt install ros-<distro>-cv-bridge ros-<distro>-sensor-msgs

# Python依赖
pip3 install opencv-python numpy scikit-surgerynditracker
```

### 2. 配置参数

编辑launch文件或通过命令行传参：

#### 超声参数
```python
device_id: 6              # 视频设备ID
frame_rate: 30.0          # 目标帧率(Hz)
image_width: 1920         # 图像宽度
image_height: 1080        # 图像高度
enable_recording: false   # 是否启用录制话题
```

#### Polaris参数
```python
rom_files: '/path/to/tool1.rom,/path/to/tool2.rom'  # ROM文件路径（逗号分隔）
tool_names: 'probe,phantom'                           # 工具名称（逗号分隔）
serial_port: '/dev/ttyUSB0'                          # 串口
enable_recording: false                               # 是否启用录制话题
```

### 3. 单独运行节点

#### 只运行超声节点
```bash
ros2 run your_package ultrasound_publisher_node.py \
  --ros-args \
  -p device_id:=6 \
  -p frame_rate:=30.0 \
  -p enable_recording:=false
```

#### 只运行Polaris节点
```bash
ros2 run your_package polaris_publisher_node.py \
  --ros-args \
  -p rom_files:='/path/to/probe.rom,/path/to/phantom.rom' \
  -p tool_names:='probe,phantom' \
  -p serial_port:='/dev/ttyUSB0' \
  -p enable_recording:=false
```

### 4. 使用Launch文件运行

```bash
# 使用默认参数
ros2 launch your_package test_publishers_launch.py

# 自定义参数
ros2 launch your_package test_publishers_launch.py \
  device_id:=0 \
  frame_rate:=30.0 \
  rom_files:='/home/user/tools/probe.rom,/home/user/tools/phantom.rom' \
  tool_names:='probe,phantom' \
  enable_recording:=true
```

## 🔍 测试和调试

### 1. 查看话题列表
```bash
ros2 topic list
```

应该看到：
```
/us_image_raw
/ndi_transforms
# 如果启用录制：
/us_image_record
/ndi_transforms_record
```

### 2. 查看话题频率
```bash
# 超声图像频率
ros2 topic hz /us_image_raw

# NDI数据频率
ros2 topic hz /ndi_transforms
```

### 3. 查看超声图像
```bash
# 方法1: 使用rqt_image_view
ros2 run rqt_image_view rqt_image_view /us_image_raw

# 方法2: 使用image_view
ros2 run image_view image_view --ros-args -r image:=/us_image_raw
```

### 4. 查看NDI数据内容
```bash
ros2 topic echo /ndi_transforms
```

输出示例：
```json
{
  "timestamp": 1234567890.123456,
  "hw_timestamp": "12345678",
  "frame_number": 42,
  "transforms": [
    {
      "tool_id": 1,
      "tool_name": "probe",
      "quality": 0.9523,
      "matrix": [[1,0,0,100], [0,1,0,200], [0,0,1,300], [0,0,0,1]],
      "translation": [100.0, 200.0, 300.0]
    }
  ]
}
```

### 5. 监控节点状态
```bash
# 查看节点信息
ros2 node info /ultrasound_publisher
ros2 node info /polaris_publisher

# 查看参数
ros2 param list /ultrasound_publisher
ros2 param list /polaris_publisher
```

## 📊 性能监控

### 节点会自动输出统计信息：

#### 超声节点统计
```
[US Stats] Frames: 300, Actual FPS: 29.8, Success rate: 100.0%, Failed: 0
```

#### Polaris节点统计
```
[NDI Stats] Frames: 600, Actual FPS: 59.7, Tracking: Active
```

## ⚙️ 常见问题

### Q1: 无法打开视频设备
```
Error: Failed to open video device 6
```

**解决方案**：
```bash
# 查看可用设备
ls -l /dev/video*

# 测试设备
v4l2-ctl --list-devices

# 修改device_id参数
```

### Q2: NDI追踪器连接失败
```
Error: Failed to initialize NDI tracker
```

**解决方案**：
```bash
# 检查USB连接
lsusb

# 检查串口权限
sudo chmod 666 /dev/ttyUSB0

# 或添加用户到dialout组
sudo usermod -a -G dialout $USER
# 注销重新登录
```

### Q3: ROM文件找不到
```
Error: Cannot find ROM file
```

**解决方案**：
- 确认ROM文件路径正确
- 使用绝对路径
- 检查文件权限

### Q4: 帧率不稳定
```
Actual FPS: 25.3 (目标30)
```

**可能原因**：
- CPU负载过高
- USB带宽不足
- 磁盘IO慢（如果启用录制）

**解决方案**：
- 降低分辨率
- 关闭录制功能
- 使用更快的存储设备

### Q5: Marker不可见
NDI输出的quality为0或数据不更新

**解决方案**：
- 检查marker是否在追踪器视野内
- 确认没有遮挡
- 检查marker反光球是否完好

## 📝 数据格式说明

### 超声图像消息 (sensor_msgs/Image)
```python
header:
  stamp:           # ROS2时间戳
    sec: 1234
    nanosec: 567890
  frame_id: "ultrasound_frame"
height: 1080
width: 1920
encoding: "bgr8"
is_bigendian: 0
step: 5760
data: [...]        # 图像数据
```

### NDI变换消息 (std_msgs/String, JSON格式)
```json
{
  "timestamp": 1234567890.123,        // ROS2时间戳（秒）
  "hw_timestamp": "12345678",         // NDI硬件时间戳
  "frame_number": 42,                 // 帧序号
  "transforms": [                     // 变换列表
    {
      "tool_id": 1,                   // 工具ID
      "tool_name": "probe",           // 工具名称
      "quality": 0.95,                // 追踪质量 (0-1)
      "matrix": [                     // 4x4变换矩阵
        [1.0, 0.0, 0.0, 100.0],      // [R11, R12, R13, Tx]
        [0.0, 1.0, 0.0, 200.0],      // [R21, R22, R23, Ty]
        [0.0, 0.0, 1.0, 300.0],      // [R31, R32, R33, Tz]
        [0.0, 0.0, 0.0, 1.0]         // [0,   0,   0,   1]
      ],
      "translation": [100.0, 200.0, 300.0]  // 平移向量(mm)
    }
  ]
}
```

## 🔧 自定义修改

### 修改图像编码格式
```python
# 在 ultrasound_publisher_node.py 中
image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
# 可改为: "rgb8", "mono8", "mono16" 等
```

### 修改QoS策略
```python
# 如果需要更可靠但稍慢的传输
realtime_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,  # 改为RELIABLE
    history=HistoryPolicy.KEEP_LAST,
    depth=10  # 增加缓冲
)
```

### 添加图像预处理
```python
# 在timer_callback中，发布前添加：
frame = cv2.resize(frame, (960, 540))  # 降低分辨率
frame = cv2.flip(frame, 0)             # 翻转
frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # 转灰度
```

## 📈 下一步

这两个节点只负责数据采集和发布。接下来需要：

1. **Synchronizer节点** - 配对超声和NDI数据
2. **Detector节点** - 检测图像中的目标
3. **Localizer节点** - 计算3D位置
4. **Visualizer节点** - 实时可视化
5. **Recorder节点** - 数据录制（可选）

## 📞 支持

如遇问题，请检查：
1. ROS2环境是否正确配置
2. 依赖包是否安装完整
3. 设备权限是否正确
4. 参数配置是否正确

## 许可证

请根据项目需求添加适当的许可证。
