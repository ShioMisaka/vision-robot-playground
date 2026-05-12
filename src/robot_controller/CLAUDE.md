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

此包同时编译为共享库（robot_kinematics / robot_motion / robot_nodes / robot_client），
可被 `robot_hmi`、`robot_tasks`、`robot_api_python`、`robot_demos` 链接使用，也可作为独立节点运行。

## CMake Target 分层

```
robot_kinematics (共享库)    ← IK + 轨迹规划（零 ROS 依赖）
       ▲
robot_motion (共享库)        ← 运动控制器 + Jog + 接口（依赖 kinematics）
       ▲
robot_nodes (共享库)         ← ROS2 控制节点（依赖 motion）

robot_client (共享库)        ← Action/Lease 客户端（ActionRobotController + RobotClient，依赖 robot_motion + robot_msgs，不依赖 robot_nodes）
```

## 话题 / 服务 / Action 接口

### 话题

| 名称 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/joint_command` | sensor_msgs/JointState | Pub | 关节指令（9 值：7 臂 + 2 爪），100Hz 发布（唯一发布者） |
| `/joint_states` | sensor_msgs/JointState | Sub | Isaac Sim 关节反馈 |
| `~/status` | robot_msgs/RobotStatus | Pub (10Hz) | 机器人状态遥测 |
| `~/jog_command` | robot_msgs/JogCommand | Sub | Jog 点动命令（sensor_data QoS） |
| `~/joint_target` | sensor_msgs/JointState | Sub | 外部关节目标流（来自示教器，200ms 超时） |

### Service

| Service | 类型 | 说明 |
|---------|------|------|
| `~/solve_ik` | SolveIK | IK 求解（xyz, rpy → joint_angles） |
| `~/control_gripper` | ControlGripper | 夹爪控制（0=open, 1=close, 2=set_width） |
| `~/set_speed` | SetSpeed | 设置模式速度 |
| `~/get_state` | GetRobotState | 查询完整状态 |
| `~/set_tcp` | SetTCP | 设置 TCP 工具坐标系 |
| `~/set_speed_ratio` | SetSpeedRatio | 全局速度比（0.0-1.0） |
| `~/robot_cmd` | RobotCmd | STOP / EMERGENCY_STOP / CLEAR_FAULT |
| `~/acquire_control` | AcquireControl | 获取控制权 Lease（返回 session_id） |
| `~/release_control` | ReleaseControl | 释放控制权 |
| `~/renew_lease` | RenewLease | 续约 Lease |
| `~/request_teaching_mode` | RequestTeachingMode | 请求示教模式（需持有 Lease） |

### Action

| Action | 类型 | 说明 |
|--------|------|------|
| `~/move_j` | MoveJ | 关节空间/笛卡尔运动（需 session_id，带进度反馈） |
| `~/move_l` | MoveL | 线性运动（需 session_id，带进度反馈） |
| `~/go_home` | GoHome | 回安全位（需 session_id，带进度反馈） |

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
| `src/kinematics/ik_solver.cpp` | solve(), solve_from_frame(), forward(), velocity_ik() | KDL 运动链 + 伪逆雅可比；solve_from_frame 直传旋转矩阵避免欧拉角转换 |
| `include/.../kinematics/trajectory_planner.hpp` | SCurvePlanner, TrajectoryPlanner | 七段式 S 曲线轨迹规划 |
| `src/kinematics/trajectory_planner.cpp` | SCurvePlanner::plan() | 7 相位解析计算 + 多轴同步 |
| `include/.../kinematics/profile_loader.hpp` | ProfileLoader, RobotConfig | 从 YAML 加载 RobotProfile/GripperProfile |

### motion 层（robot_motion target）

| 文件 | 类/函数 | 职责 |
|------|---------|------|
| `include/.../motion/i_robot_controller.hpp` | IRobotController | 运动控制抽象接口 |
| `include/.../motion/motion_io_bridge.hpp` | MotionIOBridge | IO 桥接抽象接口 |
| `include/.../motion/robot_motion_controller.hpp` | RobotMotionController | 运动控制器实现 |
| `src/motion/robot_motion_controller.cpp` | moveJ(), moveL(), set_arm() 等 | 运动原语实现 |
| `include/.../motion/jog_controller.hpp` | JogController | Jog 点动（S-curve 速度规划 + 解析 IK） |
| `src/motion/jog_controller.cpp` | tick(), update_velocity_ramp(), enforce_joint_limits(), start(), stop() | 50Hz Jog 控制（tick 仅计算，发布由 node 层负责） |

### nodes 层（robot_nodes target）

| 文件 | 类/函数 | 职责 |
|------|---------|------|
| `include/.../nodes/robot_controller_node.hpp` | RobotControllerNode, RosMotionBridge | ROS2 主控制节点 |
| `include/.../nodes/topic_config.hpp` | TopicConfig | ROS2 话题配置（nodes 层） |
| `src/nodes/robot_controller_node.cpp` | control_loop_tick(), handle_jog_command() | 100Hz 闭环（READ→PLAN→MONITOR→WRITE）+ Jog |
| `src/nodes/robot_controller_node_services.cpp` | handle_*() | Service 回调实现 |
| `src/nodes/ros_motion_bridge.cpp` | RosMotionBridge | ROS2 通信适配（发布/订阅/TF/轨迹） |
| `include/.../nodes/robot_state.hpp` | RobotStateMachine | 状态机（IDLE/MOVING/TEACHING/STOPPING/FAULT） |
| `src/nodes/robot_state.cpp` | transition_to(), force_state() | 状态转换验证 |
| `include/.../nodes/robot_state_model.hpp` | RobotStateModel | 线程安全的目标/实际状态数据 |
| `include/.../nodes/setpoint_generator.hpp` | SetpointGenerator | tick 式轨迹回放 |
| `src/nodes/setpoint_generator.cpp` | tick(), start(), cancel() | 轨迹插值 + 进度计算 |
| `include/.../nodes/lease_manager.hpp` | LeaseManager | Lease 管理（acquire/release/renew + 超时自动释放） |
| `src/nodes/lease_manager.cpp` | acquire_control(), release_control(), renew_lease() | Lease 生命周期管理 |
| `include/.../nodes/action_handlers.hpp` | ActionHandlers | Action Server 处理器（MoveJ/MoveL/GoHome） |
| `src/nodes/action_handlers.cpp` | handle_move_j(), handle_move_l(), handle_go_home() | Action 回调实现 |

### client 层（robot_client target）

| 文件 | 类/函数 | 职责 |
|------|---------|------|
| `include/.../client/robot_client.hpp` | RobotClient | 轻量 Action/Lease 客户端（`robot_control::RobotClient`） |
| `include/.../client/action_robot_controller.hpp` | ActionRobotController | Action 代理控制器（`robot_control::ActionRobotController`） |
| `src/client/robot_client.cpp` | create(), acquire_control(), release_control(), wait_for_services() | 客户端实现 |
| `src/client/action_robot_controller.cpp` | moveJ(), moveL(), go_home(), open/close_gripper() | Action 调用封装 |

## 测试与演示

### 离线测试（无需 Isaac Sim）
| 测试文件 | 覆盖范围 |
|---------|---------|
| test/test_ik_solver.cpp | URDF 加载、FK、IK 闭环一致性、不可达位姿处理 |
| test/test_trajectory_planner.cpp | 零位移、完整 7 相位、负位移、无巡航、短距离、Jerk 连续性、多轴同步 |
| test/test_motion_controller.cpp | 速度设置、moveJ/moveL 轨迹、抓取状态（使用 MockMotionBridge） |
| test/test_profile_loader.cpp | YAML 加载、RobotProfile/GripperProfile 字段验证 |

注：集成测试（test_robot_node.cpp）已迁移至 `robot_demos/test/`。test_camera_tf.cpp 已迁移至 `robot_tasks/test/`。

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
- **外部依赖**: rclcpp, sensor_msgs, geometry_msgs, tf2_ros, tf2, tf2_geometry_msgs, ament_index_cpp, orocos_kdl, urdfdom, urdf, kdl_parser, eigen, yaml-cpp

## 修改指南
- **修改 IK 算法** → 编辑 `src/kinematics/ik_solver.cpp` 的 `IKSolver::solve()` 和 `velocity_ik()`
- **修改轨迹规划** → 编辑 `src/kinematics/trajectory_planner.cpp` 的 `SCurvePlanner::plan()`
- **修改运动原语（moveJ/moveL）** → 编辑 `src/motion/robot_motion_controller.cpp`
- **修改 Jog 行为** → 编辑 `src/motion/jog_controller.cpp`（速度 ramp → update_velocity_ramp()、关节限速 → enforce_joint_limits()、坐标变换）
- **修改控制循环** → 编辑 `src/nodes/robot_controller_node.cpp` 的 `control_loop_tick()`
- **修改状态机** → 编辑 `src/nodes/robot_state.cpp` 的 `is_valid_transition()`
- **修改控制常量** → 编辑 `include/.../motion/control_constants.hpp`
- **修改 Lease 策略** → 编辑 `src/nodes/lease_manager.cpp`（超时、续约策略）
- **修改 Action 处理** → 编辑 `src/nodes/action_handlers.cpp`（MoveJ/MoveL/GoHome 的 Action 回调）
- **修改客户端库** → 编辑 `src/client/` 下的 RobotClient 和 ActionRobotController
- **新增机器人** → 在 `robot_description/config/` 下添加 `xxx_profile.yaml`，定义 `RobotProfile`，通过 `ProfileLoader::load()` 加载
- **新增 Service** → 在 `robot_controller_node_services.cpp` 中添加 handler，在 `robot_controller_node.cpp` 的 `init()` 中注册
- **新增 Action** → 在 `action_handlers.cpp` 中添加 handler，在 `robot_controller_node.cpp` 的 `init()` 中注册 Action Server

## 注意事项
- **100Hz 闭环** 是系统核心，所有运动（轨迹、Jog、外部目标）都由此循环驱动，`/joint_command` 仅由控制器发布
- **外部关节目标** 通过 `~/joint_target` 话题接收，200ms 超时后自动回退到保持当前位置
- **状态机强制转换** 仅用于 EMERGENCY_STOP，其他转换必须通过 `transition_to()` 验证
- **速度复合**: 有效速度 = 模式速度 × 全局速度比，影响轨迹规划的时间计算
- **工厂模式**: `RobotControllerNode::create()` 使用两阶段初始化，避免 `shared_from_this()` 问题
- **线程安全**: `RobotStateModel` 使用 `shared_mutex`（读写锁），高频读不受阻塞
- **moveJ/moveL** 通过 S 曲线规划器以 50Hz 生成轨迹点，不依赖 Isaac Sim 内部规划器
