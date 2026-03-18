# control_command modular split

这是基于 `robot_controller_node.cpp` 做的真实拆分版本，不是通用模板。

## 现在的拆分边界

- `robot_controller.*`
  - 只保留 ROS2 Node 壳、模块装配、topic/timer wiring、command dispatch
- `command_dispatcher.*`
  - 负责把字符串命令解析成结构化 `Command`
- `moveit_bootstrap.*`
  - 负责等待 `/move_group`、复制参数、创建 `MoveGroupInterface`、通用 setup
- `joint_state_cache.*`
  - 负责 `/joint_states` 缓存
- `tf_helper.*`
  - 统一 TF 查询出口
- `servo_controller.*`
  - 把原文件中的 Servo 参数、启动、非阻塞 halt、tick 闭环全部独立
- `motion_primitives.*`
  - 放置 `moveToPose / moveToPositionPlan / cartesianToPosition / moveToDownPose / moveToJoints / insert / current`
- `insert_strategy.*`
  - 保留为独立文件，沿用原有实现

## 与原文件的关系

原始 `robot_controller_node.cpp` 中的主要函数映射如下：

- `waitForMoveGroup / copyParametersFromMoveGroup / applyCommonSetup / printRuntimeFrames`
  -> `moveit_bootstrap.*`
- `getLatestJointState / buildJointPosMap`
  -> `joint_state_cache.*`
- `setServoTargetAndActivate / requestServoHalt / publishZeroTwistOnce / startServoAsync / servoControlTick`
  -> `servo_controller.*`
- `planPositionWithJ4Corridor / moveToPose / moveToPositionPlan / cartesianToPosition / moveToDownPose / moveToJoints / insertAlongWorldZ / getCurrentStatus`
  -> `motion_primitives.*`
- `commandCallback`
  -> 解析部分进入 `command_dispatcher.*`，调度部分留在 `robot_controller.*`

## 建议的替换方式

1. 用这里的 `include/control_command/*` 和 `src/*` 替换原 package 中对应文件
2. 保留你的 launch / package.xml
3. 用新的 `CMakeLists.txt` 对照修改
4. 先编译 `robot_control_node`
5. 编译通过后，再决定是否继续拆 timing 版本