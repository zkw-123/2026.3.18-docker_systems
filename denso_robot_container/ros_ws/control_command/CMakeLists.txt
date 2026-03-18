cmake_minimum_required(VERSION 3.8)
project(control_command)

# C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)        # [FIX] used by robot_status_publisher_node
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tf2_ros REQUIRED)            # [FIX] used by robot_status_publisher_node
find_package(tf2_geometry_msgs REQUIRED)

# 这两个是关键：提供 robot_trajectory / trajectory_processing / robot_model 等头文件与链接
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)

include_directories(include)

# ----------------------------
# Main control node (original)
# ----------------------------
add_executable(robot_control_node
  src/robot_controller_node.cpp
  src/insert_strategy.cpp
)

ament_target_dependencies(robot_control_node
  rclcpp
  std_msgs
  geometry_msgs
  moveit_ros_planning_interface
  moveit_msgs
  tf2_geometry_msgs
  moveit_core
  moveit_ros_planning
)

# --------------------------------
# Timing version control node (NEW)
# --------------------------------
add_executable(robot_control_node_timing
  src/robot_controller_node_timing.cpp
  src/insert_strategy.cpp
)

ament_target_dependencies(robot_control_node_timing
  rclcpp
  std_msgs
  geometry_msgs
  moveit_ros_planning_interface
  moveit_msgs
  tf2_geometry_msgs
  moveit_core
  moveit_ros_planning
)

# ----------------------------
# Status publisher node
# ----------------------------
add_executable(robot_status_publisher_node
  src/robot_status_publisher_node.cpp
)

ament_target_dependencies(robot_status_publisher_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2_ros
  tf2_geometry_msgs
)

# ----------------------------
# Install
# ----------------------------
install(TARGETS
  robot_control_node
  robot_control_node_timing          # [NEW]
  robot_status_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py" PATTERN "*.launch.py"
)

ament_package()

