^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package denso_robot_bringup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
1.3.1 (2026-02-01)
------------------
* Refactored launch file with the following improvements (Kunkun):

  1. Added dedicated ``use_sim_time`` launch argument with explicit bool type casting
     via ``ParameterValue``, decoupled from the ``sim`` argument to allow independent
     override; propagated consistently to all nodes including move_group, control_node,
     rviz2, and static_transform_publisher (active in both sim and hardware modes:
     ``True`` follows Gazebo ``/clock``, ``False`` uses system wall clock)
  2. Split controller spawners into separate hardware and simulation variants using
     ``IfCondition``/``UnlessCondition``; simulation spawners are wrapped in a
     ``TimerAction`` (3s delay) to wait for Gazebo and spawn_entity to become ready
     before connecting to ``/controller_manager``
  3. Added missing Gazebo plugins ``libgazebo_ros_init.so`` and
     ``libgazebo_ros_api_plugin.so`` alongside ``libgazebo_ros_factory.so`` to ensure
     ``/clock`` publication and full ROS 2 API availability in simulation mode
  4. Added ``use_sim_time`` parameter to ``control_node`` to ensure consistent clock
     behaviour in hardware mode
  5. Propagated ``use_sim_time_bool`` to ``rviz2`` and ``static_transform_publisher``,
     ensuring correct TF timestamp synchronisation in both sim and hardware modes
  6. Cleaned up residual commented-out code (MongoDB server node, ``launch_rviz``
     argument, stale TODO blocks) and clarified node launch ordering to reduce
     potential race conditions

1.2.0 (2024-03-15)
------------------
* Implemented support for ROS2 Humble
1.1.1 (2024-02-09)
------------------
1.1.0 (2023-03-24)
------------------
Initial release
