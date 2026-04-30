# robot_controller

## 职责
机器人运动控制的核心 C++ 库与 ROS2 节点。提供 IK/FK 求解、七段式 S 曲线轨迹规划、
Jog 点动控制、100Hz 闭环控制循环，以及完整的 ROS2 Service 接口。
架构上分为 3 个 CMake Target（零 ROS 依赖的核心库 → 运动控制器 → ROS2 节点），
实现 ROS 无关的核心逻辑与通信层解耦。

## 节点清单
| 节点 | 可执行文件 | 功能 |
|------|-----------|------|
| robot_controller_node | `ros2 run robot_controller robot_controller_node` | 独立控制器节点（standalone_main.cpp） |

此包同时编译为共享库（robot_kinematics / robot_motion / robot_nodes），
可被 `robot_hmi`、`robot_api_cpp`、`robot_api_python` 链接使用，也可作为独立节点运行。

## CMake Target 分层

```
robot_kinematics (共享库)    ← IK + 轨迹规划（零 ROS 依赖）
       ▲
robot_motion (共享库)        ← 运动控制器 + Jog + 接口（依赖 kinematics）
       ▲
robot_nodes (共享库)         ← ROS2 控制节点（依赖 motion）
```

## 话题 / 服务接口

### 话题

| 名称 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/joint_command` | sensor_msgs/JointState | Pub | 关节指令（9 值：7 臂 + 2 爪），100Hz 发布（唯一发布者） |
| `/joint_states` | sensor_msgs/JointState | Sub | Isaac Sim 关节反馈 |
| `~/status` | robot_msgs/RobotStatus | Pub (10Hz) | 机器人状态遥测 |
| `~/jog_command` | robot_msgs/JogCommand | Sub | Jog 点动命令（sensor_data QoS） |
| `~/joint_target` | sensor_msgs/JointState | Sub | 外部关节目标流（来自示教器，200ms 超时） |

### Service（兼容层，Python 脚本用）

| Service | 类型 | 说明 |
|---------|------|------|
| `~/solve_ik` | SolveIK | IK 求解（xyz, rpy → joint_angles） |
| `~/move_joint` | MoveJoint | 关节空间运动（moveJ） |
| `~/move_pose` | MovePose | 笛卡尔运动（mode: 0=moveJ, 1=moveL） |
| `~/move_linear` | MoveLinear | 线性增量运动 |
| `~/control_gripper` | ControlGripper | 夹爪控制（0=open, 1=close, 2=set_width） |
| `~/go_home` | GoHome | 回安全位 |
| `~/set_speed` | SetSpeed | 设置模式速度 |
| `~/get_state` | GetRobotState | 查询完整状态 |

### Service（示教器专用）

| Service | 类型 | 说明 |
|---------|------|------|
| `~/set_tcp` | SetTCP | 设置 TCP 工具坐标系 |
| `~/set_speed_ratio` | SetSpeedRatio | 全局速度比（0.0-1.0） |
| `~/robot_cmd` | RobotCmd | STOP / EMERGENCY_STOP / CLEAR_FAULT |

## 关键参数（硬编码在 control_constants.hpp）

| 参数 | 值 | 说明 |
|------|----|------|
| kJointTolerance | 0.05 rad | 关节到达容差 |
| kFingerTolerance | 0.002 m | 夹爪到达容差 |
| kMotionTimeout | 10.0 s | 运动超时 |
| kTrajectoryTimeout | 15.0 s | 轨迹执行超时 |
| kControlLoopHz | 100 Hz | 主控循环频率 |
| kFollowingErrorLimit | 0.50 rad | 轨迹跟随误差上限（Isaac Sim 物理延迟需宽松阈值） |
| kTeachingFollowErrorLimit | 0.50 rad | Jog 跟随误差上限 |
| kArrivalTolerance | 0.01 rad | 到达判定容差 |
| kArrivalSettleTime | 0.2 s | 到达稳定等待 |
| kReadyTimeout | 5.0 s | 等待关节反馈超时 |

## 核心类与修改入口

### kinematics 层（robot_kinematics target）

| 文件 | 类/函数 | 职责 |
|------|---------|------|
| `include/.../kinematics/robot_profile.hpp` | RobotProfile, GripperProfile, TcpConfig, MotionLimits, MotionMode, rpy_to_rotation() | 数据结构 + 工具函数 |
| `include/.../kinematics/ik_solver.hpp` | IKSolver | IK/FK 求解（KDL + DLS 阻尼） |
| `src/kinematics/ik_solver.cpp` | IKSolver::solve(), forward(), velocity_ik() | KDL 运动链 + 伪逆雅可比 |
| `include/.../kinematics/trajectory_planner.hpp` | SCurvePlanner, TrajectoryPlanner | 七段式 S 曲线轨迹规划 |
| `src/kinematics/trajectory_planner.cpp` | SCurvePlanner::plan() | 7 相位解析计算 + 多轴同步 |
| `include/.../profiles/panda_profile.hpp` | profiles::panda(), profiles::panda_gripper() | Panda 机器人参数 |

### motion 层（robot_motion target）

| 文件 | 类/函数 | 职责 |
|------|---------|------|
| `include/.../motion/i_robot_controller.hpp` | IRobotController | 运动控制抽象接口 |
| `include/.../motion/motion_io_bridge.hpp` | MotionIOBridge | IO 桥接抽象接口 |
| `include/.../motion/robot_motion_controller.hpp` | RobotMotionController | 运动控制器实现 |
| `src/motion/robot_motion_controller.cpp` | moveJ(), moveL(), set_arm() 等 | 运动原语实现 |
| `include/.../motion/jog_controller.hpp` | JogController | Jog 点动（S-curve 速度规划 + 雅可比速度 IK） |
| `src/motion/jog_controller.cpp` | JogController::tick(), start(), stop() | 50Hz Jog 控制 |
| `include/.../motion/topic_config.hpp` | TopicConfig, CameraExtrinsics | ROS2 话题配置（motion 层，无 ROS 依赖） |

### nodes 层（robot_nodes target）

| 文件 | 类/函数 | 职责 |
|------|---------|------|
| `include/.../nodes/robot_controller_node.hpp` | RobotControllerNode, RosMotionBridge | ROS2 主控制节点 |
| `src/nodes/robot_controller_node.cpp` | control_loop_tick(), handle_jog_command() | 100Hz 闭环（READ→PLAN→MONITOR→WRITE）+ Jog |
| `src/nodes/robot_controller_node_services.cpp` | handle_*() | 11 个 Service 回调实现 |
| `src/nodes/ros_motion_bridge.cpp` | RosMotionBridge | ROS2 通信适配（发布/订阅/TF/轨迹） |
| `include/.../nodes/robot_state.hpp` | RobotStateMachine | 状态机（IDLE/MOVING/TEACHING/STOPPING/FAULT） |
| `src/nodes/robot_state.cpp` | transition_to(), force_state() | 状态转换验证 |
| `include/.../nodes/motion_owner.hpp` | MotionOwner | 运动控制权枚举（NONE/PENDANT/SCRIPT） |
| `include/.../nodes/robot_state_model.hpp` | RobotStateModel | 线程安全的目标/实际状态数据 |
| `include/.../nodes/setpoint_generator.hpp` | SetpointGenerator | tick 式轨迹回放 |
| `src/nodes/setpoint_generator.cpp` | tick(), start(), cancel() | 轨迹插值 + 进度计算 |

## 测试与演示

### 离线测试（无需 Isaac Sim）
| 测试文件 | 覆盖范围 |
|---------|---------|
| test/test_ik_solver.cpp | URDF 加载、FK、IK 闭环一致性、不可达位姿处理 |
| test/test_trajectory_planner.cpp | 零位移、完整 7 相位、负位移、无巡航、短距离、Jerk 连续性、多轴同步 |
| test/test_motion_controller.cpp | 速度设置、moveJ/moveL 轨迹、抓取状态（使用 MockMotionBridge） |

注：集成测试（test_robot_node.cpp）已迁移至 `robot_demos/test/`。test_camera_tf.cpp 仍位于 `robot_vision/test/`。

### 演示
所有演示已迁移至 `robot_demos` 包（demo_grasp_tcp 等）。

## 启动方式
```bash
# 独立运行控制器节点
ros2 run robot_controller robot_controller_node

# 或通过 launch 文件
ros2 launch robot_bringup controller.launch.py
```

## 包内依赖
- **内部依赖**: robot_msgs, robot_description, robot_logger
- **外部依赖**: rclcpp, sensor_msgs, geometry_msgs, tf2_ros, tf2, tf2_geometry_msgs, ament_index_cpp, orocos_kdl, urdfdom, urdf, kdl_parser, eigen

## 修改指南
- **修改 IK 算法** → 编辑 `src/kinematics/ik_solver.cpp` 的 `IKSolver::solve()` 和 `velocity_ik()`
- **修改轨迹规划** → 编辑 `src/kinematics/trajectory_planner.cpp` 的 `SCurvePlanner::plan()`
- **修改运动原语（moveJ/moveL）** → 编辑 `src/motion/robot_motion_controller.cpp`
- **修改 Jog 行为** → 编辑 `src/motion/jog_controller.cpp`（速度 ramp、坐标变换、关节限速）
- **修改控制循环** → 编辑 `src/nodes/robot_controller_node.cpp` 的 `control_loop_tick()`
- **修改状态机** → 编辑 `src/nodes/robot_state.cpp` 的 `is_valid_transition()`
- **修改控制常量** → 编辑 `include/.../motion/control_constants.hpp`
- **新增机器人** → 在 `include/.../profiles/` 下添加 `xxx_profile.hpp`，定义 `RobotProfile`
- **新增 Service** → 在 `robot_controller_node_services.cpp` 中添加 handler，在 `robot_controller_node.cpp` 的 `init()` 中注册

## 注意事项
- **100Hz 闭环** 是系统核心，所有运动（轨迹、Jog、外部目标）都由此循环驱动，`/joint_command` 仅由控制器发布
- **外部关节目标** 通过 `~/joint_target` 话题接收，200ms 超时后自动回退到保持当前位置
- **状态机强制转换** 仅用于 EMERGENCY_STOP，其他转换必须通过 `transition_to()` 验证
- **速度复合**: 有效速度 = 模式速度 × 全局速度比，影响轨迹规划的时间计算
- **工厂模式**: `RobotControllerNode::create()` 使用两阶段初始化，避免 `shared_from_this()` 问题
- **线程安全**: `RobotStateModel` 使用 `shared_mutex`（读写锁），高频读不受阻塞
- **moveJ/moveL** 通过 S 曲线规划器以 50Hz 生成轨迹点，不依赖 Isaac Sim 内部规划器
