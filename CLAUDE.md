# Claude Code 项目指南

## 项目概述
基于 ROS2 + Isaac Sim 的机械臂视觉引导抓取系统（多机器人可扩展架构）。
使用 ZEN_X_Mini 双目深度相机的左目 + 深度通道进行目标检测与 3D 定位。
核心逻辑已用 C++ 重写，通过分层架构实现 ROS 无关的核心库与 ROS2 节点解耦。
Python 脚本通过 pybind11 调用 C++ 后端。

## 技术栈
- **语言**: C++17（核心库 + ROS2 节点）, Python 3.12（脚本，通过 pybind11 调用 C++）
- **ROS2**: Jazzy
- **仿真**: NVIDIA Isaac Sim（通过 ROS2 bridge 通信）
- **机器人**: Franka Panda 7-DOF + 二指夹爪（架构支持扩展新机器人）
- **相机**: ZEN_X_Mini（左目 RGB + 深度）
- **轨迹规划**: 七段式 S 曲线（Jerk 连续），多轴同步时间对齐
- **IK 求解**: KDL 伪逆 + 阻尼最小二乘法（DLS）奇异位形保护
- **C++ 核心依赖**: rclcpp, Eigen3, Orocos KDL, OpenCV, tf2_ros, cv_bridge, message_filters
- **Python 绑定**: pybind11 2.11
- **示教器 GUI**: Qt5 Widgets（C++ ROS2 包，通过 ROS2 Service 与控制节点解耦）

## 分层架构
```
Layer 3: Python Binding (pybind11)           ← script/ 通过 C++ 后端控制机器人
Layer 2: ROS 2 C++ Wrapper Nodes            ← rclcpp 节点（通信适配层）
Layer 1: Pure C++ Core Library (无 ROS 依赖) ← 4 个 CMake target
```

### CMake Target 分层
```
robot_kinematics   ← IK + 轨迹规划（零 ROS 依赖，仅 KDL + Eigen）
       ▲
robot_vision       ← 颜色检测 + 视觉接口（零 ROS 依赖，仅 OpenCV + Eigen）
       ▲
robot_motion       ← 运动控制器 + 接口（依赖 kinematics）
       ▲
robot_nodes        ← ROS2 节点 + 任务管理（依赖 motion + vision）
```

- Layer 1 通过 `MotionIOBridge` 抽象接口与通信解耦，ROS2 节点实现该接口
- `robot_kinematics` 和 `robot_vision` 是叶子节点，可独立复用和测试
- `robot_nodes` 是唯一有 ROS 依赖的 target
- 新增机器人只需定义 `RobotProfile`，核心库无需修改
- Python 脚本使用 `rclcpp`（通过 pybind11），不能同时使用 `rclpy`

## 项目结构
```
src/
  # === Python 原始模块（保留供参考/对比）===
  config.py            # Python 常量配置
  ik_solver.py         # Python IK 求解器（ikpy）
  vision.py            # Python HSV 颜色检测器
  robot.py             # Python RobotController 节点
  vision_processor.py  # Python VisionProcessor 节点
  task_manager.py      # Python GraspTaskManager

  # === C++ 核心库包（纯库，无可执行文件）===
  robot_control_cpp/
    include/robot_control_cpp/
      profiles/
        panda_profile.hpp                         # Panda 专用配置
      kinematics/
        robot_profile.hpp                         # RobotProfile / GripperProfile / TcpConfig / MotionLimits
        ik_solver.hpp                              # IK/FK 求解器（KDL + DLS）
        trajectory_planner.hpp                     # S 曲线轨迹规划器（七段式，Jerk 连续）
      motion/
        control_constants.hpp                     # 通用控制常量
        i_robot_controller.hpp                    # 运动控制抽象接口 IRobotController
        motion_io_bridge.hpp                      # IO 桥接接口 MotionIOBridge
        robot_motion_controller.hpp               # 通用运动控制器
      vision/
        i_vision_processor.hpp                    # 视觉处理抽象接口 IVisionProcessor
        camera_interface.hpp                      # CameraInterface 基类
        color_detector.hpp                        # ColorDetector（OpenCV HSV）
      nodes/
        topic_config.hpp                          # ROS2 话题配置 TopicConfig + CameraExtrinsics
        grasp_task_manager.hpp                    # 抓取状态机 GraspTaskManager
        robot_controller_node.hpp                 # ROS2 机器人控制节点
        vision_processor_node.hpp                 # ROS2 视觉处理节点
    src/
      kinematics/    ik_solver.cpp / trajectory_planner.cpp
      motion/        robot_motion_controller.cpp
      vision/        color_detector.cpp
      nodes/         robot_controller_node.cpp / vision_processor_node.cpp / grasp_task_manager.cpp
    CMakeLists.txt                # 4 target: robot_kinematics / robot_motion / robot_vision / robot_nodes
    package.xml

  # === pybind11 Python 绑定包 ===
  robot_control_cpp_py/
    src/bindings.cpp              # pybind11 绑定代码
    robot_control_cpp_py/
      __init__.py                 # Python 包，导入 _core 并重新导出
    CMakeLists.txt
    package.xml

  # === C++ 测试与演示包 ===
  robot_control_test/
    test/
      test_ik_solver.cpp          # 独立 IK 测试（离线可运行）
      test_robot_node.cpp         # 集成测试（需 Isaac Sim）
    demo/
      demo_grasp_tcp.cpp          # TCP 抓取演示
    CMakeLists.txt                # find_package(robot_control_cpp) 链接对应 target
    package.xml                   # depend on robot_control_cpp

  # === ROS2 自定义消息包 ===
  robot_control_msgs/              # 原始服务接口（保留，向后兼容）
    srv/
      SolveIK.srv                 # IK 求解服务
      MoveJoint.srv               # 关节运动服务
      MovePose.srv                # 笛卡尔运动服务（moveJ/moveL）
      MoveLinear.srv              # 线性增量运动服务
      ControlGripper.srv          # 夹爪控制服务
      GoHome.srv                  # 回 Home 服务
      SetSpeed.srv                # 速度设置服务
      GetRobotState.srv           # 状态查询服务
    CMakeLists.txt
    package.xml

  # === 示教器接口包 ===
  arm_control_interfaces/         # 标准化示教器接口（Actions + Services + Messages）
    action/
      MoveJ.action                # 关节/笛卡尔运动 Action（带反馈）
      MoveL.action                # 线性运动 Action（带反馈）
    srv/
      SetTCP.srv                  # 设置 TCP 工具坐标系
      SetSpeedRatio.srv           # 设置全局速度比（0.0-1.0）
      RobotCmd.srv                # 机器人命令（STOP/EMERGENCY_STOP/CLEAR_FAULT）
    msg/
      RobotStatus.msg             # 机器人状态（状态机 + 遥测）
      JogCommand.msg              # Jog 点动速度命令
    CMakeLists.txt
    package.xml

  # === Qt5 示教器包 ===
  teaching_pendant/
    include/teaching_pendant/
      pendant_node.hpp            # ROS2 节点（service client + subscriber）
      main_window.hpp             # Qt5 主窗口
    src/
      pendant_node.cpp            # 通信后端实现
      main_window.cpp             # GUI 实现
      main.cpp                    # 入口（Qt + ROS2 线程整合）
    CMakeLists.txt                # Qt5 + ROS2
    package.xml

script/
  test_move.py         # Python 演示（纯 Python 后端）：IK 位姿控制
  test_vision.py       # Python 演示（纯 Python 后端）：视觉伺服引导抓取
  test_grasp_tcp.py    # Python 演示（纯 Python 后端）：TCP 抓取
  test_move_cpp.py     # Python 演示（C++ 后端）：IK 位姿控制
  test_vision_cpp.py   # Python 演示（C++ 后端）：视觉伺服引导抓取
  test_grasp_tcp_cpp.py # Python 演示（C++ 后端）：TCP 抓取
  test_joint_state.py  # Python 演示：关节状态读取
  test_camera.py       # Python 演示：相机图像显示
urdf/
  panda.urdf           # Franka Panda URDF 模型
```

## 编码规范 — Python
- 遵循 PEP8，完整 Type Hints + 中文 Docstring
- C++ 后端脚本使用 `rclcpp_init()` / `rclcpp_shutdown()`，禁止使用 `rclpy.init()`
- 线程同步使用 `threading.Event`（就绪等待）和 `threading.Lock`（共享数据保护）

## 编码规范 — C++
- 遵循 Google C++ Style Guide，类名 PascalCase，方法/变量 snake_case
- 禁止裸指针，全面使用智能指针 (`std::shared_ptr`, `std::unique_ptr`)
- 所有公开头文件 `.hpp` 必须包含 Doxygen 注释（`@brief`, `@param`, `@return`）
- 接口隔离：ROS2 通信逻辑（Layer 2）不得污染纯核心逻辑（Layer 1）
- 异常处理：C++ 层捕获通信/超时异常，通过 pybind11 抛出到 Python
- 线程同步：`std::mutex` + `std::condition_variable`
- 回调组：状态订阅用 `MutuallyExclusiveCallbackGroup`，发布/TF 用 `ReentrantCallbackGroup`
- 节点构造使用工厂模式（`create()` 静态方法 + 两阶段初始化），避免 `shared_from_this()` 问题
- 轨迹执行使用 `sleep_until` 绝对时间调度（非 `sleep_for`），避免 OS 调度误差累积
- moveJ/moveL 通过 S 曲线规划器以 50Hz 逐步发送关节位置，不依赖 Isaac Sim 内部轨迹规划

## ROS2 话题
| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/joint_command` | sensor_msgs/JointState | Pub | 关节指令（9值：7臂+2爪） |
| `/joint_states` | sensor_msgs/JointState | Sub | Isaac Sim 关节反馈 |
| `/camera/image_raw/left` | sensor_msgs/Image | Sub | 左目 RGB |
| `/camera/image_raw/depth` | sensor_msgs/Image | Sub | 深度图（uint16 mm 或 float32 m） |

## ROS2 Services（robot_controller_node - 兼容层）

**保留用于向后兼容和 Python 脚本**。示教器应使用 Actions。

| Service | 类型 | 说明 |
|---------|------|------|
| `~/solve_ik` | SolveIK | IK 求解（xyz, rpy → joint_angles） |
| `~/move_joint` | MoveJoint | 关节空间运动（moveJ） |
| `~/move_pose` | MovePose | 笛卡尔运动（mode: 0=moveJ, 1=moveL） |
| `~/move_linear` | MoveLinear | 线性增量运动 |
| `~/control_gripper` | ControlGripper | 夹爪控制（0=open, 1=close, 2=set_width） |
| `~/go_home` | GoHome | 回安全位 |
| `~/set_speed` | SetSpeed | 设置速度（mode: 0=moveJ, 1=moveL, percent: 0-100） |
| `~/get_state` | GetRobotState | 查询状态（关节角、位姿、夹爪、TCP） |

## ROS2 Actions（robot_controller_node - 示教器接口）

**来自 arm_control_interfaces 包**，提供带进度反馈的非阻塞运动控制。

| Action | 说明 |
|--------|------|
| `~/move_j` | MoveJ — 关节空间/笛卡尔空间运动（支持 IK） |
| `~/move_l` | MoveL — 线性插值运动 |

**Action 反馈字段：**
- `progress` — 0.0~1.0 进度百分比
- `current_joint_angles` — 当前关节角
- `estimated_time_remaining` — 预计剩余时间（秒）

## ROS2 Services（robot_controller_node - 示教器专用）

**来自 arm_control_interfaces 包**，供示教器调用。

| Service | 类型 | 说明 |
|---------|------|------|
| `~/set_tcp` | SetTCP | 设置当前 TCP 工具坐标系名称 |
| `~/set_speed_ratio` | SetSpeedRatio | 设置全局速度比（0.0-1.0），与模式速度复合 |
| `~/robot_cmd` | RobotCmd | 机器人命令：STOP（停止）、EMERGENCY_STOP（急停）、CLEAR_FAULT（清除故障） |

## ROS2 Messages（arm_control_interfaces）

| Message | 类型 | 说明 |
|---------|------|------|
| `RobotStatus` | Pub（10Hz） | 机器人状态（状态机、速度比、错误码、关节角、位姿、夹爪、TCP） |
| `JogCommand` | Sub | Jog 点动命令（6 轴速度：vx,vy,vz,vroll,vpitch,vyaw） |

## 状态机（RobotStateMachine）

**状态定义：**
- `kIdle` — 空闲，可接受新命令
- `kMoving` — 运动中（Action 执行）
- `kTeaching` — 示教模式（Jog 点动）
- `kStopping` — 停止中（减速到停止）
- `kFault` — 故障状态（需 CLEAR_FAULT 恢复）

**有效转换：**
- kIdle → kMoving/kTeaching（命令触发）
- kMoving → kIdle/kStopping/kFault（完成/停止/错误）
- kTeaching → kIdle/kStopping/kFault（停止/超时/急停）
- kStopping → kIdle/kFault（停止完成/急停）
- kFault → kIdle（CLEAR_FAULT）
- **任意状态 → kFault**（EMERGENCY_STOP，最高优先级）

**Jog 看门狗：** 200ms 无 JogCommand 则自动停止（kTeaching → kIdle）。

## 速度复合规则

有效速度 = 模式速度（SetSpeed 设置） × 全局速度比（SetSpeedRatio 设置）

示例：模式速度 50% × 全局比 0.8 = 有效速度 40%

## 常用命令
```bash
# === 编译消息定义 ===
# 原始服务接口（保留，向后兼容）
colcon build --base-paths src --packages-select robot_control_msgs

# 示教器接口包（Actions + Services + Messages）
colcon build --base-paths src --packages-select arm_control_interfaces

# === 编译核心库 ===
colcon build --base-paths src --packages-select robot_control_cpp

# === 编译示教器（需先编译 msgs + interfaces + core）===
source install/setup.zsh
colcon build --base-paths src --packages-select teaching_pendant

# === 编译 pybind11 绑定包 ===
source install/setup.zsh
colcon build --base-paths src --packages-select robot_control_cpp_py

# === 编译测试与演示 ===
colcon build --base-paths src --packages-select robot_control_test

# === 一键编译全部 ===
colcon build --base-paths src

# === 编译示教器及其依赖（推荐）===
colcon build --base-paths src --packages-up-to teaching_pendant

# === Action CLI 测试 ===
# 发送 MoveJ Goal（笛卡尔模式，带 IK）
ros2 action send /move_j arm_control_interfaces/action/MoveJ "
  mode: 1
  position: {x: 0.5, y: 0.0, z: 0.3}
  orientation: {x: 0.0, y: -1.57, z: -1.57}
  speed_ratio: 0.5
  finger_width: 0.04
"

# 取消 Action
ros2 action cancel /move_j

# 查看 Action 反馈
ros2 action feedback /move_j

# === Service CLI 测试 ===
# 设置全局速度比
ros2 service call /robot_controller_node/set_speed_ratio arm_control_interfaces/srv/SetSpeedRatio "{ratio: 0.8}"

# 急停
ros2 service call /robot_controller_node/robot_cmd arm_control_interfaces/srv/RobotCmd "{command: 1}"

# 清除故障
ros2 service call /robot_controller_node/robot_cmd arm_control_interfaces/srv/RobotCmd "{command: 2}"

# === 运行演示 ===
# C++ 演示（需要 Isaac Sim）
ros2 run robot_control_test demo_grasp_tcp

# Python 演示（C++ 后端，需要 Isaac Sim）
python3 script/test_move_cpp.py
python3 script/test_grasp_tcp_cpp.py
python3 script/test_vision_cpp.py

# Python 演示（纯 Python 后端）
python3 script/test_move.py
python3 script/test_vision.py

# === 运行示教器 ===
# 需要 Isaac Sim + robot_controller_node 运行中
ros2 run teaching_pendant teaching_pendant

# === 查看系统状态 ===
ros2 topic list
ros2 topic echo /robot_controller_node/status
ros2 action list
```

## Python 绑定使用模式
```python
import threading
import robot_control_cpp_py as rc

rc.rclcpp_init()

robot = rc.RobotControllerNode.create(
    rc.profiles.panda(), rc.profiles.panda_gripper(), rc.TopicConfig())

executor = rc.MultiThreadedExecutor()
executor.add_node(robot)
spin_thread = threading.Thread(target=executor.spin, daemon=True)
spin_thread.start()

robot.wait_for_ready()
ctrl = robot.get_controller()
ctrl.open_gripper()
ctrl.move_to_pose([0.5, 0, 0.3], [0, -3.14, -3.14])

executor.cancel()
spin_thread.join()
rc.rclcpp_shutdown()
```

## 架构约定
- RobotController 不依赖 VisionProcessor，二者通过 GraspTaskManager 编排
- VisionProcessor/CameraInterface::process_image() 是桩代码，子类重写以接入 YOLO/GraspNet
- 视觉伺服参数定义在 script 中，非节点级参数
- TF2: RobotControllerNode 自动发布 base → hand → tcp 变换链
- 新增机器人：在 `include/robot_control_cpp/profiles/` 下添加 `xxx_profile.hpp`，定义 `RobotProfile`
- pybind11 绑定中所有阻塞方法必须释放 GIL（`py::call_guard<py::gil_scoped_release>()`）
- Eigen 类型在 Python 侧转换为原生 list/tuple，不使用 numpy
