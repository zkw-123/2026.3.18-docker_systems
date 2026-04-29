#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ===== Launch arguments =====
    data_dir = LaunchConfiguration("data_dir")
    fps = LaunchConfiguration("fps")
    json_mode = LaunchConfiguration("json_mode")
    json_dir = LaunchConfiguration("json_dir")
    loop = LaunchConfiguration("loop")

    image_topic = LaunchConfiguration("image_topic")
    stability_topic = LaunchConfiguration("stability_topic")

    mask_path = LaunchConfiguration("mask_path")
    alpha = LaunchConfiguration("alpha")
    c_sp_max = LaunchConfiguration("c_sp_max")
    c_grad_max = LaunchConfiguration("c_grad_max")

    # 如果你的 us_target_estimator_node 支持自定义 image_topic 或其他参数，
    # 可以在这里加 launch argument 并传入。
    # 目前 README 里未明确列出其参数，这里按默认行为启动。:contentReference[oaicite:1]{index=1}

    # ===== Declarations =====
    declare_args = [
        DeclareLaunchArgument(
            "data_dir",
            default_value="/ros2_ws/src/silicon_exp1/20250810_072526",
            description="Directory of ultrasound dataset (png sequence, etc.).",
        ),
        DeclareLaunchArgument(
            "fps",
            default_value="10.0",
            description="Playback FPS for us_dataset_player_node.",
        ),
        DeclareLaunchArgument(
            "json_mode",
            default_value="fixed_dir",
            description="JSON mode: paired / fixed_dir / fixed_single.",
        ),
        DeclareLaunchArgument(
            "json_dir",
            default_value="/ros2_ws/src/silicon_exp1",
            description="Directory containing probe pose JSON(s).",
        ),
        DeclareLaunchArgument(
            "loop",
            default_value="false",
            description="Whether to loop dataset playback.",
        ),
        DeclareLaunchArgument(
            "image_topic",
            default_value="/us_img",
            description="Input image topic for stability/target nodes.",
        ),
        DeclareLaunchArgument(
            "stability_topic",
            default_value="/us_stability",
            description="Output Sk topic.",
        ),
        DeclareLaunchArgument(
            "mask_path",
            default_value="/ros2_ws/src/silicon_exp1/us_mask.png",
            description="ROI mask path for us_stability_node.",
        ),
        DeclareLaunchArgument(
            "alpha",
            default_value="0.3",
            description="Exponential smoothing factor for Sk.",
        ),
        DeclareLaunchArgument(
            "c_sp_max",
            default_value="17.6",
            description="Normalization upper bound for C_sp.",
        ),
        DeclareLaunchArgument(
            "c_grad_max",
            default_value="7.1",
            description="Normalization upper bound for C_grad.",
        ),
    ]

    # ===== Nodes =====
    dataset_player = Node(
        package="polaris_ultrasound",
        executable="us_dataset_player_node",
        name="us_dataset_player",
        output="screen",
        parameters=[
            {
                "data_dir": data_dir,
                "fps": fps,
                "json_mode": json_mode,
                "json_dir": json_dir,
                "loop": loop,
            }
        ],
    )

    stability_node = Node(
        package="polaris_ultrasound",
        executable="us_stability_node",
        name="us_stability",
        output="screen",
        parameters=[
            {
                "image_topic": image_topic,
                "stability_topic": stability_topic,
                "mask_path": mask_path,
                "alpha": alpha,
                "c_sp_max": c_sp_max,
                "c_grad_max": c_grad_max,
            }
        ],
    )

    target_estimator = Node(
        package="polaris_ultrasound",
        executable="us_target_estimator_node",
        name="us_target_estimator",
        output="screen",
        # 如果你的目标检测节点支持参数（比如 image_topic、mask_path、transform_json_path 等），
        # 你可以按你代码实际参数名在这里补上：
        # parameters=[{"image_topic": image_topic}]
    )

    # ===== LaunchDescription =====
    return LaunchDescription(
        declare_args
        + [
            dataset_player,
            stability_node,
            target_estimator,
        ]
    )

