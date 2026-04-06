from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'comport',
            default_value='/dev/ttyACM0',
            description='The serial port of the sensor'
        ),
        DeclareLaunchArgument(
            'rate',
            default_value='1200',
            description='Rate of the sensor data acquisition'
        ),
        Node(
            package='leptrino_force_torque',
            executable='leptrino_force_torque',
            name='leptrino_force_torque',  
            output='screen',
            parameters=[{
                'com_port': LaunchConfiguration('comport')
            }]
        )
    ])

