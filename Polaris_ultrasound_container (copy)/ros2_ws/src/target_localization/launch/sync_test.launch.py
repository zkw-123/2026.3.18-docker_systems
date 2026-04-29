#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
同步测试 Launch 文件
启动超声、Polaris、同步器和虚拟点发布器
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 参数声明
    us_device_id_arg = DeclareLaunchArgument(
        'device_id',
        default_value='6',
        description='Ultrasound video ID'
    )
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='NDI tool path'
    )
    rom_files_arg = DeclareLaunchArgument(
        'rom_files',
        default_value='/path/to/probe.rom,/path/to/phantom.rom',
        description='NDI ROM file path，saparated by comma'
    )
    tool_names_arg = DeclareLaunchArgument(
        'tool_names',
        default_value='probe,phantom',
        description='tool name, saparated by comma'
    )
    frame_rate_arg = DeclareLaunchArgument(
        'frame_rate', default_value='30.0',
        description='Ultrasound capture frame rate (Hz)'
    )
    image_width_arg = DeclareLaunchArgument(
        'image_width', default_value='1920',
        description='Ultrasound image width'
    )
    image_height_arg = DeclareLaunchArgument(
        'image_height', default_value='1080',
        description='Ultrasound image height'
    )
    quality_threshold_arg = DeclareLaunchArgument(
        'quality_threshold', default_value='0.5',
        description='Minimum acceptable tracking quality'
        )

    # 节点1：超声发布

    ultrasound_node = Node(
        package='target_localization',
        executable='ultrasound_publisher',
        name='ultrasound_publisher',
        output='screen',
        parameters=[{
            'device_id':        LaunchConfiguration('device_id'),
            'frame_rate':       LaunchConfiguration('frame_rate'),     # ← 用 launch 参数覆盖
            'image_width':      LaunchConfiguration('image_width'),     # ← 用 launch 参数覆盖
            'image_height':     LaunchConfiguration('image_height'),    # ← 用 launch 参数覆盖
            'enable_recording': False,
        }]
    )

    # 节点2：Polaris发布
    polaris_node = Node(
        package='target_localization',
        executable='polaris_publisher',
        name='polaris_publisher',
        output='screen',
        parameters=[{
            'rom_files': LaunchConfiguration('rom_files'),
            'tool_names': LaunchConfiguration('tool_names'),
            'serial_port': LaunchConfiguration('serial_port'),
            'enable_recording': False
        }]
    )

    # 节点3：同步器
    synchronizer_node = Node(
        package='target_localization',
        executable='data_synchronizer',
        name='data_synchronizer',
        output='screen',
        parameters=[{
            'marker_name':       'probe',
            'time_tolerance':    LaunchConfiguration('time_tolerance'),
            'quality_threshold': LaunchConfiguration('quality_threshold'),  # ← 用 launch 参数覆盖
            'enable_statistics': True,
        }]
    )

    # 节点4：虚拟点发布器
    virtual_point_node = Node(
        package='target_localization',
        executable='virtual_point_publisher',
        name='virtual_point_publisher',
        output='screen'
    )

    return LaunchDescription([
        us_device_id_arg,
        serial_port_arg,
        rom_files_arg,
        tool_names_arg,
        ultrasound_node,
        polaris_node,
        synchronizer_node,
        virtual_point_node
    ])

