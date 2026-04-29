#!/usr/bin/env python3

"""
Publisher Nodes Launch File

用于测试超声和Polaris发布节点

使用方法：
    ros2 launch test_publishers_launch.py
    
或指定参数：
    ros2 launch test_publishers_launch.py device_id:=0 enable_recording:=true
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """生成launch描述"""
    
    # ==================== 声明启动参数 ====================
    
    # 超声参数
    us_device_id_arg = DeclareLaunchArgument(
        'device_id',
        default_value='6',
        description='Video device ID for ultrasound'
    )
    
    us_frame_rate_arg = DeclareLaunchArgument(
        'frame_rate',
        default_value='30.0',
        description='Target frame rate for ultrasound capture'
    )
    
    us_width_arg = DeclareLaunchArgument(
        'image_width',
        default_value='1920',
        description='Image width'
    )
    
    us_height_arg = DeclareLaunchArgument(
        'image_height',
        default_value='1080',
        description='Image height'
    )
    
    # Polaris参数
    rom_files_arg = DeclareLaunchArgument(
        'rom_files',
        default_value='/path/to/tool1.rom,/path/to/tool2.rom',
        description='Comma-separated ROM file paths'
    )
    
    tool_names_arg = DeclareLaunchArgument(
        'tool_names',
        default_value='probe,phantom',
        description='Comma-separated tool names'
    )
    
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for NDI Polaris'
    )
    
    # 通用参数
    enable_recording_arg = DeclareLaunchArgument(
        'enable_recording',
        default_value='false',
        description='Enable recording topics (true/false)'
    )
    
    # ==================== 创建节点 ====================
    
    # 超声发布节点
    ultrasound_node = Node(
        package='target_localization',  # 替换为您的包名
        executable='ultrasound_publisher_node.py',
        name='ultrasound_publisher',
        output='screen',
        parameters=[{
            'device_id': LaunchConfiguration('device_id'),
            'frame_rate': LaunchConfiguration('frame_rate'),
            'image_width': LaunchConfiguration('image_width'),
            'image_height': LaunchConfiguration('image_height'),
            'enable_recording': LaunchConfiguration('enable_recording'),
        }],
        # 如果设备打开失败，自动重启
        respawn=False,
        respawn_delay=2.0,
    )
    
    # Polaris发布节点
    polaris_node = Node(
        package='target_localization',  # 替换为您的包名
        executable='polaris_publisher_node.py',
        name='polaris_publisher',
        output='screen',
        parameters=[{
            'rom_files': LaunchConfiguration('rom_files'),
            'tool_names': LaunchConfiguration('tool_names'),
            'serial_port': LaunchConfiguration('serial_port'),
            'enable_recording': LaunchConfiguration('enable_recording'),
        }],
        respawn=False,
        respawn_delay=2.0,
    )
    
    # ==================== 返回Launch描述 ====================
    return LaunchDescription([
        # 参数声明
        us_device_id_arg,
        us_frame_rate_arg,
        us_width_arg,
        us_height_arg,
        rom_files_arg,
        tool_names_arg,
        serial_port_arg,
        enable_recording_arg,
        
        # 节点
        ultrasound_node,
        polaris_node,
    ])
