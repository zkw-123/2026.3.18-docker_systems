#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = "carryover_validation"

    default_config_file = os.path.join(
        FindPackageShare(package_name).find(package_name),
        "config",
        "carryover_validation.yaml",
    )

    # Launch arguments
    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=default_config_file,
        description="Path to the carry-over validation YAML config file.",
    )

    enable_adapter_arg = DeclareLaunchArgument(
        "enable_adapter",
        default_value="true",
        description=(
            "Whether to start target_point_adapter. "
            "Set false if /perception/target_point is already available."
        ),
    )

    substrate_name_arg = DeclareLaunchArgument(
        "substrate_name",
        default_value="substrate_A",
        description="Name of the substrate used in this validation experiment.",
    )

    num_steps_arg = DeclareLaunchArgument(
        "num_steps",
        default_value="20",
        description="Number of lateral correction steps.",
    )

    dk_magnitude_arg = DeclareLaunchArgument(
        "dk_magnitude_m",
        default_value="0.0005",
        description="Magnitude of each lateral correction step in meters.",
    )

    axial_speed_arg = DeclareLaunchArgument(
        "axial_speed_mps",
        default_value="0.0005",
        description="Nominal axial insertion speed in m/s.",
    )

    step_interval_arg = DeclareLaunchArgument(
        "step_interval_s",
        default_value="2.0",
        description="Time interval corresponding to one validation step.",
    )

    settle_time_arg = DeclareLaunchArgument(
        "settle_time_s",
        default_value="0.5",
        description="Waiting time after each robot command before recording target_after.",
    )

    lateral_axis_arg = DeclareLaunchArgument(
        "lateral_axis",
        default_value="x",
        description="Lateral correction axis in the robot/base frame: x, y, or z.",
    )

    axial_axis_arg = DeclareLaunchArgument(
        "axial_axis",
        default_value="z",
        description="Axial insertion axis in the robot/base frame: x, y, or z.",
    )

    axial_direction_sign_arg = DeclareLaunchArgument(
        "axial_direction_sign",
        default_value="-1.0",
        description=(
            "Sign of axial insertion direction. "
            "Use -1.0 if insertion decreases the axial coordinate."
        ),
    )

    output_dir_arg = DeclareLaunchArgument(
        "output_dir",
        default_value="/workspace/logs/carryover_validation",
        description="Directory for step-level CSV logs.",
    )

    # Launch configurations
    config_file = LaunchConfiguration("config_file")
    enable_adapter = LaunchConfiguration("enable_adapter")
    substrate_name = LaunchConfiguration("substrate_name")
    num_steps = LaunchConfiguration("num_steps")
    dk_magnitude_m = LaunchConfiguration("dk_magnitude_m")
    axial_speed_mps = LaunchConfiguration("axial_speed_mps")
    step_interval_s = LaunchConfiguration("step_interval_s")
    settle_time_s = LaunchConfiguration("settle_time_s")
    lateral_axis = LaunchConfiguration("lateral_axis")
    axial_axis = LaunchConfiguration("axial_axis")
    axial_direction_sign = LaunchConfiguration("axial_direction_sign")
    output_dir = LaunchConfiguration("output_dir")

    target_point_adapter_node = Node(
        package=package_name,
        executable="target_point_adapter",
        name="target_point_adapter",
        output="screen",
        condition=IfCondition(enable_adapter),
        parameters=[
            config_file,
        ],
    )

    validation_runner_node = Node(
        package=package_name,
        executable="validation_runner",
        name="carryover_validation_runner",
        output="screen",
        parameters=[
            config_file,
            {
                "substrate_name": substrate_name,
                "num_steps": num_steps,
                "dk_magnitude_m": dk_magnitude_m,
                "axial_speed_mps": axial_speed_mps,
                "step_interval_s": step_interval_s,
                "settle_time_s": settle_time_s,
                "lateral_axis": lateral_axis,
                "axial_axis": axial_axis,
                "axial_direction_sign": axial_direction_sign,
            },
        ],
    )

    rls_estimator_node = Node(
        package=package_name,
        executable="rls_estimator",
        name="rls_estimator",
        output="screen",
        parameters=[
            config_file,
            {
                "lateral_axis": lateral_axis,
            },
        ],
    )

    experiment_logger_node = Node(
        package=package_name,
        executable="experiment_logger",
        name="experiment_logger",
        output="screen",
        parameters=[
            config_file,
            {
                "substrate_name": substrate_name,
                "output_dir": output_dir,
            },
        ],
    )

    return LaunchDescription([
        config_file_arg,
        enable_adapter_arg,
        substrate_name_arg,
        num_steps_arg,
        dk_magnitude_arg,
        axial_speed_arg,
        step_interval_arg,
        settle_time_arg,
        lateral_axis_arg,
        axial_axis_arg,
        axial_direction_sign_arg,
        output_dir_arg,

        target_point_adapter_node,
        validation_runner_node,
        rls_estimator_node,
        experiment_logger_node,
    ])