from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    data_dir = LaunchConfiguration('data_dir')
    fps = LaunchConfiguration('fps')
    json_mode = LaunchConfiguration('json_mode')
    json_dir = LaunchConfiguration('json_dir')
    json_file = LaunchConfiguration('json_file')
    loop = LaunchConfiguration('loop')

    image_topic = LaunchConfiguration('image_topic')
    probe_topic = LaunchConfiguration('probe_topic')
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
            'data_dir',
            default_value='/ros2_ws/src/silicon_exp1/20250810_072526',
            description='Dataset directory'
        ),
        DeclareLaunchArgument(
            'fps',
            default_value='10.0',
            description='Replay FPS'
        ),
        DeclareLaunchArgument(
            'json_mode',
            default_value='fixed_dir',
            description='paired / fixed_dir / fixed_single'
        ),
        DeclareLaunchArgument(
            'json_dir',
            default_value='/ros2_ws/src/silicon_exp1',
            description='Directory containing pose json files'
        ),
        DeclareLaunchArgument(
            'json_file',
            default_value='',
            description='Single json file path when json_mode=fixed_single'
        ),
        DeclareLaunchArgument(
            'loop',
            default_value='false',
            description='Whether to loop dataset playback'
        ),
        DeclareLaunchArgument(
            'image_topic',
            default_value='/us_img',
            description='Replay image topic'
        ),
        DeclareLaunchArgument(
            'probe_topic',
            default_value='/probe_pose',
            description='Replay probe pose topic'
        ),
        DeclareLaunchArgument(
            'stability_topic',
            default_value='/us_stability',
            description='Stability output topic'
        ),
        DeclareLaunchArgument(
            'target_topic',
            default_value='/target_point',
            description='Estimated target point topic'
        ),
        DeclareLaunchArgument(
            'debug_image_topic',
            default_value='/us_target_debug',
            description='Debug image topic'
        ),
        DeclareLaunchArgument(
            'mask_path',
            default_value='',
            description='ROI mask path'
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
            executable='us_dataset_player_node',
            name='us_dataset_player',
            output='screen',
            parameters=[{
                'data_dir': data_dir,
                'fps': fps,
                'json_mode': json_mode,
                'json_dir': json_dir,
                'json_file': json_file,
                'loop': loop,
                'image_topic': image_topic,
                'probe_topic': probe_topic,
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
