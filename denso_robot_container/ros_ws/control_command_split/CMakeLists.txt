cmake_minimum_required(VERSION 3.8)
project(control_command_split)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

set(MODULAR_SOURCES
  src/robot_controller.cpp
  src/command_dispatcher.cpp
  src/moveit_bootstrap.cpp
  src/joint_state_cache.cpp
  src/tf_helper.cpp
  src/servo_controller.cpp
  src/motion_primitives.cpp
  src/insert_strategy.cpp
)

add_executable(robot_control_node
  src/robot_controller_node.cpp
  ${MODULAR_SOURCES}
)
ament_target_dependencies(robot_control_node
  rclcpp rclcpp_action std_msgs std_srvs sensor_msgs geometry_msgs
  tf2 tf2_ros tf2_geometry_msgs
  moveit_ros_planning_interface moveit_msgs moveit_core
)
target_link_libraries(robot_control_node Eigen3::Eigen)

add_executable(robot_status_publisher_node
  src/robot_status_publisher_node.cpp
)
ament_target_dependencies(robot_status_publisher_node
  rclcpp std_msgs sensor_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs
)

install(TARGETS
  robot_control_node
  robot_status_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
