#!/usr/bin/env python3
import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, FindExecutable


def load_yaml(package_name: str, relative_path: str):
    pkg_share = os.path.join(
        os.getenv("AMENT_PREFIX_PATH", "").split(":")[0], "share"
    )
    try:
        from ament_index_python.packages import get_package_share_directory
        pkg_share = get_package_share_directory(package_name)
    except Exception:
        pass

    abs_path = os.path.join(pkg_share, relative_path)
    try:
        with open(abs_path, "r") as f:
            return yaml.safe_load(f)
    except OSError:
        return None


def generate_launch_description():
    # Core arguments for DENSO bringup
    model = LaunchConfiguration("model")
    sim = LaunchConfiguration("sim")  
    use_sim_time = LaunchConfiguration("use_sim_time")  # 'true' / 'false' as string
    ip_address = LaunchConfiguration("ip_address")
    namespace = LaunchConfiguration("namespace")
    use_rviz = LaunchConfiguration("use_rviz")

    # Arguments aligned with denso_robot_bringup.launch.py
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    moveit_config_file = LaunchConfiguration("moveit_config_file")
    send_format = LaunchConfiguration("send_format")
    recv_format = LaunchConfiguration("recv_format")
    verbose = LaunchConfiguration("verbose")

   
    # Explicit bool conversion for runtime nodes (clock source)
    use_sim_time_bool = ParameterValue(use_sim_time, value_type=bool)

    # Servo arguments
    use_servo = LaunchConfiguration("use_servo")
    servo_start_delay = LaunchConfiguration("servo_start_delay")
    servo_params_file = LaunchConfiguration("servo_params_file")

    # Control node arguments
    start_control_node = LaunchConfiguration("start_control_node")
    control_use_servo = LaunchConfiguration("control_use_servo")

    # Include denso_robot_bringup.launch.py
    denso_bringup_launch = PathJoinSubstitution([
        FindPackageShare("denso_robot_bringup"),
        "launch",
        "denso_robot_bringup.launch.py"
    ])

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(denso_bringup_launch),
        launch_arguments={
            "model": model,
            "sim": sim,
            "use_sim_time": use_sim_time,
            "ip_address": ip_address,
            "namespace": namespace,
            "use_rviz": use_rviz,
            "description_package": description_package,
            "description_file": description_file,
            "moveit_config_package": moveit_config_package,
            "moveit_config_file": moveit_config_file,
            "send_format": send_format,
            "recv_format": recv_format,
            "verbose": verbose,
        }.items(),
    )

    # Generate robot_description (URDF) and robot_description_semantic (SRDF)
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]), " ",
        "ip_address:=", ip_address, " ",
        "model:=", model, " ",
        "send_format:=", send_format, " ",
        "recv_format:=", recv_format, " ",
        "namespace:=", namespace, " ",
        "verbose:=", verbose, " ",
        "sim:=", sim, " ",
    ])
    robot_description = {"robot_description": robot_description_content}

    robot_description_semantic_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([FindPackageShare(moveit_config_package), "srdf", moveit_config_file]), " ",
        "model:=", model, " ",
        "namespace:=", namespace, " ",
    ])
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}

    # Load kinematics.yaml (same file as bringup uses)
    kinematics_yaml = load_yaml("denso_robot_moveit_config", "config/kinematics.yaml")
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    # MoveIt Servo node (robot model parameters injected explicitly)
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        output="screen",
        parameters=[
            servo_params_file,
            {"use_sim_time": use_sim_time_bool},
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
        ],
        arguments=['--ros-args', '--log-level', 'servo_node:=debug', '--log-level', 'moveit_servo:=debug'],
        condition=IfCondition(use_servo),
    )

    servo_node_delayed = TimerAction(
        period=servo_start_delay,
        actions=[servo_node],
    )

    # Command interface node (control_command)
    robot_control_node = Node(
        package="control_command",
        executable="robot_control_node",
        name="robot_controller",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time_bool},
            {"use_servo": ParameterValue(control_use_servo, value_type=bool)},
        ],
        condition=IfCondition(start_control_node),
    )

    # Default servo yaml path stored inside this package
    default_servo_yaml = PathJoinSubstitution([
        FindPackageShare("denso_bringup_overall"),
        "config",
        "moveit_servo.yaml"
    ])

    return LaunchDescription([
        DeclareLaunchArgument("model", default_value=TextSubstitution(text="vp6242")),
        DeclareLaunchArgument("sim", default_value=TextSubstitution(text="true")),
        DeclareLaunchArgument("use_sim_time", default_value=sim),
        DeclareLaunchArgument("ip_address", default_value=TextSubstitution(text="192.168.1.100")),
        DeclareLaunchArgument("namespace", default_value=TextSubstitution(text="")),
        DeclareLaunchArgument("use_rviz", default_value=TextSubstitution(text="true")),

        DeclareLaunchArgument("description_package", default_value=TextSubstitution(text="denso_robot_descriptions")),
        DeclareLaunchArgument("description_file", default_value=TextSubstitution(text="denso_robot.urdf.xacro")),
        DeclareLaunchArgument("moveit_config_package", default_value=TextSubstitution(text="denso_robot_moveit_config")),
        DeclareLaunchArgument("moveit_config_file", default_value=TextSubstitution(text="denso_robot.srdf.xacro")),
        DeclareLaunchArgument("send_format", default_value=TextSubstitution(text="288")),
        DeclareLaunchArgument("recv_format", default_value=TextSubstitution(text="292")),
        DeclareLaunchArgument("verbose", default_value=TextSubstitution(text="false")),

        DeclareLaunchArgument("use_servo", default_value=TextSubstitution(text="true")),
        DeclareLaunchArgument("servo_start_delay", default_value=TextSubstitution(text="2.0")),
        DeclareLaunchArgument("servo_params_file", default_value=default_servo_yaml),

        DeclareLaunchArgument("start_control_node", default_value=TextSubstitution(text="true")),
        DeclareLaunchArgument("control_use_servo", default_value=TextSubstitution(text="true")),

        bringup,
        servo_node_delayed,
        robot_control_node,
    ])

