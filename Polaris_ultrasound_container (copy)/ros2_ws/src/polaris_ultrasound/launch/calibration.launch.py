from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rom_path = LaunchConfiguration('rom_path')
    tool_names = LaunchConfiguration('tool_names')
    device_id = LaunchConfiguration('device_id')
    save_path = LaunchConfiguration('save_path')
    frame_rate = LaunchConfiguration('frame_rate')
    sync_threshold = LaunchConfiguration('sync_threshold')
    record_time = LaunchConfiguration('record_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'rom_path',
            default_value='/workspace/src/rom_files/8700449_phantom.rom,/workspace/src/rom_files/8700338_probe.rom',
            description='Comma-separated ROM file paths'
        ),
        DeclareLaunchArgument(
            'tool_names',
            default_value='phantom,probe',
            description='Comma-separated tool names'
        ),
        DeclareLaunchArgument(
            'device_id',
            default_value='6',
            description='Ultrasound video device id'
        ),
        DeclareLaunchArgument(
            'save_path',
            default_value='/workspace/data/calibration_data',
            description='Directory to save calibration data'
        ),
        DeclareLaunchArgument(
            'frame_rate',
            default_value='30.0',
            description='Ultrasound capture frame rate'
        ),
        DeclareLaunchArgument(
            'sync_threshold',
            default_value='0.005',
            description='Synchronization threshold in seconds'
        ),
        DeclareLaunchArgument(
            'record_time',
            default_value='60.0',
            description='Total recording duration in seconds'
        ),

        Node(
            package='polaris_ultrasound',
            executable='polaris_reader',
            name='polaris_reader',
            output='screen',
            parameters=[{
                'rom_path': rom_path,
                'tool_names': tool_names,
                'record_time': record_time,
            }]
        ),

        Node(
            package='polaris_ultrasound',
            executable='calibration_recorder',
            name='calibration_recorder',
            output='screen',
            parameters=[{
                'device_id': device_id,
                'save_path': save_path,
                'frame_rate': frame_rate,
                'sync_threshold': sync_threshold,
                'record_time': record_time,
                'tool_names': tool_names,
            }]
        ),
    ])
