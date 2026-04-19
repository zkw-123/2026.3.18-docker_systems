from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rom_path = LaunchConfiguration('rom_path')
    tool_names = LaunchConfiguration('tool_names')
    device_id = LaunchConfiguration('device_id')

    image_topic = LaunchConfiguration('image_topic')
    stability_topic = LaunchConfiguration('stability_topic')
    target_topic = LaunchConfiguration('target_topic')
    debug_image_topic = LaunchConfiguration('debug_image_topic')

    mask_path = LaunchConfiguration('mask_path')
    alpha = LaunchConfiguration('alpha')
    c_sp_max = LaunchConfiguration('c_sp_max')
    c_grad_max = LaunchConfiguration('c_grad_max')
    transform_json_path = LaunchConfiguration('transform_json_path')

    return LaunchDescription([
        DeclareLaunchArgument(
            'rom_path',
            default_value='/opt/ndi/rom_files/8700449_phantom.rom,/opt/ndi/rom_files/8700340_stylus.rom',
            description='Comma-separated ROM file paths'
        ),
        DeclareLaunchArgument(
            'tool_names',
            default_value='phantom,stylus',
            description='Comma-separated tool names'
        ),
        DeclareLaunchArgument(
            'device_id',
            default_value='6',
            description='Ultrasound video device id'
        ),
        DeclareLaunchArgument(
            'image_topic',
            default_value='/us_img',
            description='Ultrasound image topic'
        ),
        DeclareLaunchArgument(
            'stability_topic',
            default_value='/us_stability',
            description='Ultrasound stability output topic'
        ),
        DeclareLaunchArgument(
            'target_topic',
            default_value='/target_point',
            description='Estimated target point topic'
        ),
        DeclareLaunchArgument(
            'debug_image_topic',
            default_value='/us_target_debug',
            description='Debug image topic for target estimator'
        ),
        DeclareLaunchArgument(
            'mask_path',
            default_value='',
            description='ROI mask path for stability / target estimation'
        ),
        DeclareLaunchArgument(
            'alpha',
            default_value='0.3',
            description='Exponential smoothing factor for Sk'
        ),
        DeclareLaunchArgument(
            'c_sp_max',
            default_value='17.6',
            description='Normalization upper bound for C_sp'
        ),
        DeclareLaunchArgument(
            'c_grad_max',
            default_value='7.1',
            description='Normalization upper bound for C_grad'
        ),
        DeclareLaunchArgument(
            'transform_json_path',
            default_value='/ros2_ws/src/polaris_ultrasound/ultrasoud-robot-transformation.json',
            description='Transform JSON for target estimator'
        ),

        Node(
            package='polaris_ultrasound',
            executable='polaris_reader',
            name='polaris_reader',
            output='screen',
            parameters=[{
                'rom_path': rom_path,
                'tool_names': tool_names,
            }]
        ),

        Node(
            package='polaris_ultrasound',
            executable='ultrasound_reader',
            name='ultrasound_reader',
            output='screen',
            parameters=[{
                'device_id': device_id,
            }]
        ),

        Node(
            package='polaris_ultrasound',
            executable='us_stability_node',
            name='us_stability',
            output='screen',
            parameters=[{
                'image_topic': image_topic,
                'stability_topic': stability_topic,
                'mask_path': mask_path,
                'alpha': alpha,
                'c_sp_max': c_sp_max,
                'c_grad_max': c_grad_max,
            }]
        ),

        Node(
            package='polaris_ultrasound',
            executable='us_target_estimator_node',
            name='us_target_estimator',
            output='screen',
            parameters=[{
                'image_topic': image_topic,
                'target_topic': target_topic,
                'debug_image_topic': debug_image_topic,
                'mask_path': mask_path,
                'transform_json_path': transform_json_path,
            }]
        ),
    ])
