#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Realtime Target Localization System Launch File
实时目标定位系统启动文件

说明 (Description):
    一键启动完整的实时目标定位系统
    One-click launch of complete realtime target localization system

节点 (Nodes):
    1. ultrasound_publisher - 超声图像发布
    2. polaris_publisher - NDI追踪发布
    3. data_synchronizer - 数据同步
    4. target_detector - 目标检测
    5. target_localizer - 3D定位
    6. realtime_visualizer - 实时可视化
    7. data_recorder (可选) - 数据录制

使用方法 (Usage):
    ros2 launch <package_name> target_localization_launch.py

作者 (Author): Your Name
日期 (Date): 2024
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
import os


def generate_launch_description():
    """
    生成launch描述
    Generate launch description
    """
    
    # ============================================
    # 启动参数声明 (Launch Arguments Declaration)
    # ============================================
    
    # 设备参数 (Device parameters)
    us_device_id_arg = DeclareLaunchArgument(
        'us_device_id',
        default_value='6',
        description='超声视频设备ID (Ultrasound video device ID)'
    )
    
    ndi_rom_paths_arg = DeclareLaunchArgument(
        'ndi_rom_paths',
        default_value='/path/to/probe.rom,/path/to/phantom.rom',
        description='NDI ROM文件路径，逗号分隔 (NDI ROM file paths, comma-separated)'
    )
    
    ndi_tool_names_arg = DeclareLaunchArgument(
        'ndi_tool_names',
        default_value='probe,phantom',
        description='工具名称，逗号分隔 (Tool names, comma-separated)'
    )
    
    # 同步参数 (Synchronization parameters)
    marker_name_arg = DeclareLaunchArgument(
        'marker_name',
        default_value='probe',
        description='要追踪的marker名称 (Marker name to track)'
    )
    
    time_tolerance_arg = DeclareLaunchArgument(
        'time_tolerance',
        default_value='0.005',
        description='时间同步容差(秒) (Time sync tolerance in seconds)'
    )
    
    quality_threshold_arg = DeclareLaunchArgument(
        'quality_threshold',
        default_value='0.5',
        description='最低追踪质量阈值 (Minimum tracking quality threshold)'
    )
    
    # 检测参数 (Detection parameters)
    detection_mode_arg = DeclareLaunchArgument(
        'detection_mode',
        default_value='center',
        description='检测模式: center, moving, random (Detection mode)'
    )
    
    # 标定参数 (Calibration parameters)
    pixel_to_mm_x_arg = DeclareLaunchArgument(
        'pixel_to_mm_x',
        default_value='0.1',
        description='X方向像素到mm转换 (Pixel to mm conversion in X)'
    )
    
    pixel_to_mm_y_arg = DeclareLaunchArgument(
        'pixel_to_mm_y',
        default_value='0.1',
        description='Y方向像素到mm转换 (Pixel to mm conversion in Y)'
    )
    
    # 可视化参数 (Visualization parameters)
    display_mode_arg = DeclareLaunchArgument(
        'display_mode',
        default_value='local',
        description='显示模式: local, ros, none (Display mode)'
    )
    
    # 录制参数 (Recording parameters)
    enable_recording_arg = DeclareLaunchArgument(
        'enable_recording',
        default_value='false',
        description='是否启用数据录制 (Enable data recording)'
    )
    
    save_path_arg = DeclareLaunchArgument(
        'save_path',
        default_value='/tmp/ultrasound_tracking_data',
        description='数据保存路径 (Data save path)'
    )
    
    # ============================================
    # 获取启动配置 (Get Launch Configurations)
    # ============================================
    us_device_id = LaunchConfiguration('us_device_id')
    ndi_rom_paths = LaunchConfiguration('ndi_rom_paths')
    ndi_tool_names = LaunchConfiguration('ndi_tool_names')
    marker_name = LaunchConfiguration('marker_name')
    time_tolerance = LaunchConfiguration('time_tolerance')
    quality_threshold = LaunchConfiguration('quality_threshold')
    detection_mode = LaunchConfiguration('detection_mode')
    pixel_to_mm_x = LaunchConfiguration('pixel_to_mm_x')
    pixel_to_mm_y = LaunchConfiguration('pixel_to_mm_y')
    display_mode = LaunchConfiguration('display_mode')
    enable_recording = LaunchConfiguration('enable_recording')
    save_path = LaunchConfiguration('save_path')
    
    # ============================================
    # 节点1: 超声图像发布器 (Node 1: Ultrasound Publisher)
    # ============================================
    ultrasound_publisher_node = Node(
        package='target_localization',  
        executable='ultrasound_publisher_node.py',
        name='ultrasound_publisher',
        output='screen',
        parameters=[{
            'device_id': us_device_id,
            'frame_rate': 30.0,
            'enable_recording': enable_recording,
            'image_width': 1920,
            'image_height': 1080
        }]
    )
    
    # ============================================
    # 节点2: Polaris追踪发布器 (Node 2: Polaris Publisher)
    # ============================================
    polaris_publisher_node = Node(
        package='target_localization',
        executable='polaris_publisher_node.py',
        name='polaris_publisher',
        output='screen',
        parameters=[{
            'rom_paths': ndi_rom_paths,
            'tool_names': ndi_tool_names,
            'serial_port': '/dev/ttyUSB0',
            'enable_recording': enable_recording
        }]
    )
    
    # ============================================
    # 节点3: 数据同步器 (Node 3: Data Synchronizer)
    # ============================================
    data_synchronizer_node = Node(
        package='target_localization',
        executable='data_synchronizer_node.py',
        name='data_synchronizer',
        output='screen',
        parameters=[{
            'marker_name': marker_name,
            'time_tolerance': time_tolerance,
            'quality_threshold': quality_threshold,
            'enable_statistics': True,
            'statistics_interval': 30
        }]
    )
    
    # ============================================
    # 节点4: 目标检测器 (Node 4: Target Detector)
    # ============================================
    target_detector_node = Node(
        package='target_localization',
        executable='target_detector_node.py',
        name='target_detector',
        output='screen',
        parameters=[{
            'detection_mode': detection_mode,
            'enable_visualization': False
        }]
    )
    
    # ============================================
    # 节点5: 目标定位器 (Node 5: Target Localizer)
    # ============================================
    target_localizer_node = Node(
        package='target_localization',
        executable='target_localizer_node.py',
        name='target_localizer',
        output='screen',
        parameters=[{
            'marker_name': marker_name,
            'pixel_to_mm_x': pixel_to_mm_x,
            'pixel_to_mm_y': pixel_to_mm_y,
            'image_origin_x': 0.0,
            'image_origin_y': 0.0,
            # 默认的marker到图像变换矩阵（单位矩阵）
            # Default marker to image transform (identity matrix)
            # 需要根据实际标定结果修改！
            # Need to modify based on actual calibration!
            'marker_to_image_transform': [
                1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 1.0
            ]
        }]
    )
    
    # ============================================
    # 节点6: 实时可视化器 (Node 6: Realtime Visualizer)
    # ============================================
    realtime_visualizer_node = Node(
        package='target_localization',
        executable='realtime_visualizer_node.py',
        name='realtime_visualizer',
        output='screen',
        parameters=[{
            'display_mode': display_mode,
            'window_name': 'Ultrasound Tracking Visualization',
            'publish_annotated': False,
            'font_scale': 0.6,
            'line_thickness': 2
        }]
    )
    
    # ============================================
    # 节点7: 数据录制器 (可选) (Node 7: Data Recorder - Optional)
    # ============================================
    data_recorder_node = Node(
        package='target_localization',
        executable='data_recorder_node.py',
        name='data_recorder',
        output='screen',
        condition=IfCondition(enable_recording),  # 只有启用录制时才启动
        parameters=[{
            'save_path': save_path,
            'save_us_images': True,
            'save_ndi_data': True,
            'buffer_size': 1000,
            'num_worker_threads': 4
        }]
    )
    
    # ============================================
    # 启动信息 (Launch Information)
    # ============================================
    launch_info = LogInfo(
        msg=[
            '\n',
            '=' * 80, '\n',
            '实时目标定位系统已启动 (Realtime Target Localization System Started)\n',
            '=' * 80, '\n',
            '\n',
            '节点列表 (Node List):\n',
            '  1. ultrasound_publisher - 超声图像发布\n',
            '  2. polaris_publisher - NDI追踪发布\n',
            '  3. data_synchronizer - 数据同步\n',
            '  4. target_detector - 目标检测\n',
            '  5. target_localizer - 3D定位\n',
            '  6. realtime_visualizer - 实时可视化\n',
            '  7. data_recorder (可选) - 数据录制\n',
            '\n',
            '重要话题 (Important Topics):\n',
            '  - /us_image_raw: 实时超声图像\n',
            '  - /ndi_transforms: 实时NDI追踪\n',
            '  - /synchronized_data: 配对数据\n',
            '  - /target_pixel: 像素坐标\n',
            '  - /target_3d_position: 3D位置\n',
            '\n',
            '监控命令 (Monitoring Commands):\n',
            '  ros2 topic hz /synchronized_data  - 查看同步率\n',
            '  ros2 topic echo /target_3d_position - 查看3D位置\n',
            '  ros2 node list - 查看所有节点\n',
            '\n',
            '=' * 80, '\n'
        ]
    )
    
    # ============================================
    # 返回Launch描述 (Return Launch Description)
    # ============================================
    return LaunchDescription([
        # 参数声明 (Argument declarations)
        us_device_id_arg,
        ndi_rom_paths_arg,
        ndi_tool_names_arg,
        marker_name_arg,
        time_tolerance_arg,
        quality_threshold_arg,
        detection_mode_arg,
        pixel_to_mm_x_arg,
        pixel_to_mm_y_arg,
        display_mode_arg,
        enable_recording_arg,
        save_path_arg,
        
        # 启动信息 (Launch info)
        launch_info,
        
        # 节点 (Nodes)
        ultrasound_publisher_node,
        polaris_publisher_node,
        data_synchronizer_node,
        target_detector_node,
        target_localizer_node,
        realtime_visualizer_node,
        data_recorder_node
    ])


if __name__ == '__main__':
    generate_launch_description()
