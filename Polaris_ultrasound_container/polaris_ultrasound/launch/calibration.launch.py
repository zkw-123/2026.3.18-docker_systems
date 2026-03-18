from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'rom_path',
            default_value='/opt/ndi/rom_files/your_rom_file.rom',
            description='Comma-separated list of ROM file paths'
        ),
        DeclareLaunchArgument(
            'tool_names',
            default_value='probe,phantom',
            description='Comma-separated list of tool names'
        ),
        DeclareLaunchArgument(
            'device_id',
            default_value='6',
            description='Ultrasound video device ID (default /dev/video6)'
        ),
        DeclareLaunchArgument(
            'save_path',
            default_value='/ros2_ws/calibration_data',
            description='Path to save calibration data'
        ),
        DeclareLaunchArgument(
            'frame_rate',
            default_value='30.0',
            description='Frame rate for ultrasound image capture (Hz)'
        ),
        DeclareLaunchArgument(
            'sync_threshold',
            default_value='0.005',
            description='Synchronization threshold in seconds'
        ),
        DeclareLaunchArgument(
            'record_time',
            default_value='60.0',
            description='Total time in seconds to record data'
        ),

        Node(
            package='polaris_ultrasound',
            executable='polaris_reader_node.py',
            name='polaris_reader',
            parameters=[{
                'rom_path': LaunchConfiguration('rom_path'),
                'tool_names': LaunchConfiguration('tool_names'),
                'record_time': LaunchConfiguration('record_time')
            }]
        ),

        Node(
            package='polaris_ultrasound',
            executable='calibration_recorder.py',  
            name='calibration_recorder',
            parameters=[{
                'device_id': LaunchConfiguration('device_id'),
                'save_path': LaunchConfiguration('save_path'),
                'frame_rate': LaunchConfiguration('frame_rate'),
                'sync_threshold': LaunchConfiguration('sync_threshold'),
                'record_time': LaunchConfiguration('record_time'),
                'tool_names': LaunchConfiguration('tool_names')
            }]
        )
    ])
