#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    base_frame = LaunchConfiguration("base_frame")
    ee_frame = LaunchConfiguration("ee_frame")
    publish_rate = LaunchConfiguration("publish_rate")
    max_joint_print = LaunchConfiguration("max_joint_print")

    declare_args = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time if true",
        ),
        DeclareLaunchArgument(
            "base_frame",
            default_value="base_link",
            description="Base frame for TF lookup",
        ),
        DeclareLaunchArgument(
            "ee_frame",
            default_value="J6",
            description="End-effector frame for TF lookup",
        ),
        DeclareLaunchArgument(
            "publish_rate",
            default_value="20.0",
            description="Status publish rate (Hz)",
        ),
        DeclareLaunchArgument(
            "max_joint_print",
            default_value="6",
            description="Max number of joints to print in status string",
),

    ]

    controller = Node(
        package="control_command",
        executable="robot_control_node",
        name="robot_control_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    status = Node(
        package="control_command",
        executable="robot_status_publisher_node",
        name="robot_status_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "base_frame": base_frame,
                "ee_frame": ee_frame,
                "publish_rate": publish_rate,
                "max_joint_print": max_joint_print,
                
            }
        ],
    )

    return LaunchDescription(declare_args + [controller, status])

