from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取 config 文件的绝对路径
    config_file = os.path.join(
        get_package_share_directory('faulhaber_motor_controller'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='faulhaber_motor_controller',
            executable='motor_controller_node',
            name='faulhaber_motor_controller',
            parameters=[config_file],
            output='screen'
        ),
    ])

