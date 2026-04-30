# Robot Vision Playground

基于 ROS2 + Isaac Sim 的机械臂视觉引导抓取系统。C++ 核心库 + pybind11 Python 绑定的多语言分层架构，支持多机器人扩展。

## 功能

- **机械臂控制**: 关节角度指令、IK 位姿控制、相对平移/旋转、TCP 工具坐标系
- **S 曲线轨迹规划**: 七段式 Jerk 连续轨迹，moveJ 关节空间平滑运动，moveL 笛卡尔直线平滑运动
- **奇异位形保护**: IK 求解器内置阻尼最小二乘法（DLS），自动检测并抑制奇异位形附近的关节速度爆炸
- **视觉处理**: 双目深度相机左目+深度同步订阅，HSV 颜色目标检测
- **视觉伺服抓取**: 图像反馈闭环 — 检测 → 居中 → 下探 → 夹取 → 提起
- **TF2 集成**: 自动发布末端执行器 TF 变换链，支持坐标系变换查询
- **多机器人扩展**: 通过 `RobotProfile` 配置驱动，核心库无需修改
- **Python 绑定**: pybind11 将 C++ 核心库暴露给 Python，脚本可直接调用 C++ 后端
- **示教器接口**: 标准化 ROS2 Actions（MoveJ/MoveL）带进度反馈，状态机保护，Jog 点动（S-curve 速度规划 + Jacobian 速度 IK），急停与看门狗

## 环境要求

- **操作系统**: Ubuntu 24.04
- **ROS2**: Jazzy Jalisco
- **C++**: C++17 编译器（GCC 13+）
- **Python**: 3.12
- **仿真**: NVIDIA Isaac Sim（提供 `/joint_states` 和相机话题）
- **相机**: ZEN_X_Mini 双目深度相机

## 依赖安装

### C++ 依赖（ROS2 包，通过 apt 安装）

```bash
sudo apt install ros-jazzy-rclcpp ros-jazzy-sensor-msgs ros-jazzy-geometry-msgs \
  ros-jazzy-tf2-ros ros-jazzy-cv-bridge ros-jazzy-message-filters \
  ros-jazzy-orocos-kdl ros-jazzy-urdfdom ros-jazzy-kdl-parser \
  ros-jazzy-eigen ros-jazzy-image-transport ros-jazzy-urdf \
  ros-jazzy-ament-index-cpp \
  pybind11-dev
```

### Qt5 示教器依赖（可选）

```bash
sudo apt install qtbase5-dev
```

## 项目结构

```
isaac_ros_project/
├── src/
│   ├── robot_msgs/                # 数据定义层（Services + Actions + Messages）
│   │   ├── srv/                   # 11 个 Service 定义
│   │   │   ├── SolveIK.srv
│   │   │   ├── MoveJoint.srv / MovePose.srv / MoveLinear.srv
│   │   │   ├── ControlGripper.srv / GoHome.srv
│   │   │   ├── SetSpeed.srv / GetRobotState.srv
│   │   │   ├── SetTCP.srv / SetSpeedRatio.srv / RobotCmd.srv
│   │   ├── action/                # 2 个 Action 定义（带进度反馈）
│   │   │   ├── MoveJ.action
│   │   │   └── MoveL.action
│   │   └── msg/                   # 2 个 Message 定义
│   │       ├── RobotStatus.msg    # 状态 + 遥测（10Hz 发布）
│   │       └── JogCommand.msg     # Jog 点动命令
│   │
│   ├── robot_description/         # 机器人模型（URDF/Mesh）
│   │   └── urdf/panda.urdf        # Franka Panda URDF
│   │
│   ├── robot_bringup/             # 启动项（Launch 文件、全局参数）
│   │   └── launch/
│   │       ├── controller.launch.py    # 控制器节点 launch
│   │       └── full_system.launch.py   # 完整系统 launch
│   │
│   ├── robot_controller/          # 核心控制层（C++）
│   │   ├── include/robot_controller/
│   │   │   ├── profiles/
│   │   │   │   └── panda_profile.hpp           # Panda 专用配置
│   │   │   ├── kinematics/                     # robot_kinematics target（零 ROS 依赖）
│   │   │   │   ├── robot_profile.hpp           # RobotProfile / GripperProfile / TcpConfig
│   │   │   │   ├── ik_solver.hpp               # IK/FK 求解器（KDL + DLS）
│   │   │   │   └── trajectory_planner.hpp      # S 曲线轨迹规划器
│   │   │   ├── motion/                         # robot_motion target（依赖 kinematics）
│   │   │   │   ├── control_constants.hpp       # 通用控制常量
│   │   │   │   ├── i_robot_controller.hpp      # IRobotController 抽象接口
│   │   │   │   ├── motion_io_bridge.hpp        # MotionIOBridge 接口
│   │   │   │   ├── robot_motion_controller.hpp
│   │   │   │   └── jog_controller.hpp          # Jog 点动控制器（S-curve + 速度 IK）
│   │   │   └── nodes/                          # robot_nodes target（依赖 motion）
│   │   │       ├── topic_config.hpp            # TopicConfig + CameraExtrinsics
│   │   │       ├── robot_controller_node.hpp
│   │   │       ├── robot_state.hpp             # RobotStateMachine
│   │   │       └── trajectory_executor.hpp     # 非阻塞轨迹执行器
│   │   ├── src/
│   │   │   ├── kinematics/  ik_solver.cpp / trajectory_planner.cpp
│   │   │   ├── motion/      robot_motion_controller.cpp / jog_controller.cpp
│   │   │   └── nodes/       robot_controller_node.cpp / robot_state.cpp / setpoint_generator.cpp / standalone_main.cpp
│   │   ├── test/                                 # 离线测试
│   │   │   ├── test_ik_solver.cpp
│   │   │   ├── test_trajectory_planner.cpp
│   │   │   └── test_motion_controller.cpp
│   │   └── demo/
│   │       └── demo_grasp_tcp.cpp
│   │
│   ├── robot_vision/             # 感知层（视觉处理）
│   │   ├── include/robot_vision/
│   │   │   ├── vision/
│   │   │   │   ├── i_vision_processor.hpp      # IVisionProcessor 抽象接口
│   │   │   │   ├── camera_interface.hpp        # CameraInterface 基类
│   │   │   │   └── color_detector.hpp          # ColorDetector（OpenCV HSV）
│   │   │   └── nodes/
│   │   │       ├── vision_processor_node.hpp   # ROS2 视觉处理节点
│   │   │       └── grasp_task_manager.hpp      # 抓取状态机 GraspTaskManager
│   │   ├── src/
│   │   │   ├── vision/      color_detector.cpp
│   │   │   └── nodes/       vision_processor_node.cpp / grasp_task_manager.cpp
│   │   ├── test/                                 # 集成测试（需 Isaac Sim）
│   │   │   ├── test_robot_node.cpp
│   │   │   └── test_camera_tf.cpp
│   │   └── demo/
│   │       └── demo_camera.cpp
│   │
│   ├── robot_hmi/                # 示教器界面（Qt5）
│   │   ├── include/robot_hmi/
│   │   │   ├── pendant_node.hpp              # ROS2 节点（发布 JogCommand，关节流控）
│   │   │   ├── main_window.hpp               # Qt5 主窗口（薄编排层）
│   │   │   └── panels/                       # Panel 组件（UI + 逻辑自包含）
│   │   │       ├── connection_bar.hpp        # 连接状态显示
│   │   │       ├── camera_panel.hpp          # 相机画面 + RGB/Depth
│   │   │       ├── robot_state_bar.hpp       # 位姿/夹爪/TCP 显示
│   │   │       ├── joint_control_panel.hpp   # 关节滑块 + 编辑 + 流控
│   │   │       ├── cartesian_panel.hpp       # XYZ/RPY + Jog + 运动模式
│   │   │       └── function_panel.hpp        # 速度/夹爪/GoHome/E-STOP
│   │   └── src/
│   │       ├── pendant_node.cpp
│   │       ├── main_window.cpp
│   │       ├── panels/                       # Panel 实现
│   │       └── main.cpp                      # 入口
│   │
│   └── robot_api_python/          # Python API 封装 + C++ 客户端库
│       ├── include/robot_api_python/
│       │   ├── robot_client_node.hpp          # C++ 客户端节点（连接外部控制器）
│       │   └── service_robot_controller.hpp   # C++ Service 代理控制器
│       ├── src/
│       │   ├── bindings.cpp                   # pybind11 绑定代码（~560 行）
│       │   ├── robot_client_node.cpp
│       │   └── service_robot_controller.cpp
│       ├── demo/
│       │   └── demo_vision_grasp.cpp          # C++ 视觉抓取演示（客户端模式）
│       └── robot_api_python/
│           └── __init__.py                    # Python 包入口
│
├── script/
│   ├── test_move_cpp.py            # Python + C++ 后端：IK 位姿控制
│   ├── test_grasp_tcp_cpp.py       # Python + C++ 后端：TCP 抓取
│   ├── test_vision_cpp.py          # Python + C++ 后端：视觉伺服抓取
│   └── test_camera_tf.py           # Python + C++ 后端：相机 TF 链验证
│
├── docs/
│   ├── superpowers/                # 开发流程文档
│   └── api.md                      # 完整 API 接口文档
├── CLAUDE.md
└── README.md
```

## 系统架构

### 单控制器多客户端模式

整个系统的核心设计原则是 **「只有一个进程拥有机器人控制权」**。所有前端（示教器、Python 脚本、视觉抓取 demo）都不直接控制机器人硬件，而是通过 ROS2 话题/服务向 **`robot_controller_node`** 发送请求，由它统一决策和执行。

```
                         ┌──────────────────────────────────┐
                         │   Isaac Sim (物理仿真引擎)        │
                         │   发布 /joint_states (关节反馈)   │
                         │   接收 /joint_command (关节指令)   │
                         └────────▲──────────────┬──────────┘
                                  │              │
                         /joint_states    /joint_command
                          (反馈)            (100Hz 指令，唯一发布者)
                                  │              │
┌─────────────────────────────────┼──────────────┼──────────────────┐
│                                 │              │                  │
│  robot_controller_node          │              │                  │
│  (独立进程，系统核心)           │              │                  │
│                                 │              │                  │
│  ┌──────────────────────────────┴──────────────┴───────────────┐  │
│  │  100Hz 控制循环：READ → PLAN → MONITOR → WRITE              │  │
│  │  11 个 ROS2 Service（move_joint, move_pose, go_home...）    │  │
│  │  订阅 ~/joint_target（示教器 50Hz 关节流）                  │  │
│  │  订阅 ~/jog_command（示教器 50Hz Jog 心跳）                 │  │
│  │  状态机（IDLE / MOVING / TEACHING / STOPPING / FAULT）      │  │
│  │  运动控制权（MotionOwner: NONE / PENDANT / SCRIPT）         │  │
│  └─────────────────────────────────────────────────────────────┘  │
│                                 ▲                                 │
└─────────────────────────────────┼─────────────────────────────────┘
                                  │
                    ROS2 Service / Topic (请求，不是指令)
                                  │
          ┌───────────────────────┼───────────────────────┐
          │                       │                       │
┌─────────┴─────────┐  ┌──────────┴─────────┐  ┌──────────┴────────┐
│  robot_hmi        │  │  Python 脚本       │  │  demo_vision_grasp│
│  (Qt5 示教器)     │  │ (robot_api_python) │  │  (C++ 可执行文件) │
│                   │  │                    │  │                   │
│  • Service 调用   │  │  • Service 调用    │  │  • Service 调用   │
│  • 50Hz 关节流    │  │                    │  │                   │
│  • 50Hz Jog 心跳  │  │  RobotClientNode   │  │  RobotClientNode  │
│  (PendantNode)    │  │                    │  │                   │
└───────────────────┘  └────────────────────┘  └───────────────────┘
```

### 为什么多个客户端不会冲突？

关键在于 **`/joint_command` 只有一个发布者**（`robot_controller_node`），所有客户端发送的是「请求」而非「指令」：

1. **单一指令源**：只有 `robot_controller_node` 的 100Hz 循环发布 `/joint_command`，不存在两个控制器竞争写入的情况
2. **运动控制权（MotionOwner）**：控制器内部维护当前谁在控制（`NONE` / `PENDANT` / `SCRIPT`），同一时刻只有一个所有者
3. **状态机保护**：所有状态转换经过 `is_valid_transition()` 验证，非法请求被拒绝
4. **200ms 看门狗**：示教器的关节流 200ms 无更新自动失效，防止卡死

### 包间依赖

```
robot_msgs          ← 叶子包（无项目内依赖）
     │
     ▼
robot_logger        ← 统一日志（被所有非叶子包依赖）
robot_description   ← 叶子包（URDF 模型）
     │
     ▼
robot_controller    ← robot_kinematics / robot_motion / robot_nodes
     │
     ▼
robot_vision        ← robot_vision_core / robot_vision_nodes
     │
     ├──▶ robot_hmi           ← Qt5 示教器（PendantNode 连接外部节点）
     └──▶ robot_api_python    ← pybind11 Python API + C++ 客户端库 + demo

robot_bringup       ← launch 文件（controller + full_system）
```

注意：`robot_controller` → `robot_vision` 单向依赖，不可反向。
`robot_hmi` 和 `robot_api_python` 中的客户端代码 **不嵌入** `RobotControllerNode`，而是通过 Service / Topic 连接独立运行的 `robot_controller_node` 进程。

### 代码分层

```
┌──────────────────────────────────────────────────────────┐
│  Layer 3: Python Binding (pybind11)                      │
│  script/ 代码通过 C++ 后端控制机器人                     │
├──────────────────────────────────────────────────────────┤
│  Layer 2: ROS2 Wrapper Nodes（唯一有 ROS 依赖的 target） │
│  ┌──────────────────────┐ ┌───────────────────────────┐  │
│  │ RobotControllerNode  │ │ VisionProcessorNode       │  │
│  │ + RosMotionBridge    │ │ + GraspTaskManager        │  │
│  └──────────┬───────────┘ └──────────┬────────────────┘  │
├─────────────┼────────────────────────┼───────────────────┤
│  Layer 1: Pure C++ Core (无 ROS 依赖)│                   │
│  ┌──────────▼──────────┐ ┌───────────▼────────────────┐  │
│  │ robot_kinematics    │ │ robot_vision_core          │  │
│  │ IKSolver (KDL + DLS)│ │ ColorDetector (OpenCV HSV) │  │
│  │ SCurvePlanner       │ │ CameraInterface            │  │
│  │ RobotProfile        │ │ IVisionProcessor           │  │
│  └──────────┬──────────┘ └────────────────────────────┘  │
│             │                                            │
│  ┌──────────▼──────────────────────────────────────────┐ │
│  │ robot_motion                                        │ │
│  │ RobotMotionController + IRobotController            │ │
│  │ JogController (S-curve + Jacobian velocity IK)      │ │
│  │ MotionIOBridge                                      │ │
│  └─────────────────────────────────────────────────────┘ │
└──────────────────────────────────────────────────────────┘
```

## 编译

```bash
source /opt/ros/jazzy/setup.zsh

# 一键编译全部
colcon build --base-paths src

# 选择性编译单个包（需先 source install/setup.zsh 解析已安装依赖）
source install/setup.zsh
colcon build --base-paths src --packages-select robot_controller

# 编译示教器及其依赖
colcon build --base-paths src --packages-up-to robot_hmi
```

**编译顺序：** robot_msgs → robot_logger + robot_description（可并行）→ robot_controller → robot_vision + robot_hmi（可并行）→ robot_api_python

**编译产物：**
- `install/robot_msgs/` — ROS2 接口定义（Services + Actions + Messages）
- `install/robot_description/share/robot_description/urdf/panda.urdf` — URDF 模型
- `install/robot_controller/lib/librobot_kinematics.so` — IK + 轨迹规划
- `install/robot_controller/lib/librobot_motion.so` — 运动控制器 + Jog 控制器
- `install/robot_controller/lib/librobot_nodes.so` — ROS2 节点
- `install/robot_controller/lib/robot_controller/robot_controller_node` — 独立控制器可执行文件
- `install/robot_vision/lib/librobot_vision_core.so` — 视觉处理核心
- `install/robot_vision/lib/librobot_vision_nodes.so` — 视觉 ROS2 节点
- `install/robot_api_python/lib/librobot_api_client_lib.so` — C++ 客户端库（ServiceRobotController + RobotClientNode）
- `install/robot_api_python/` — pybind11 Python 模块
- `install/robot_api_python/lib/robot_api_python/demo_vision_grasp` — C++ 视觉抓取演示（客户端模式）
- `install/robot_hmi/lib/robot_hmi/robot_hmi` — Qt5 示教器可执行文件

### 导出编译数据库（IDE 代码补全）

```bash
colcon build --base-paths src --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
ln -s build/robot_controller/compile_commands.json ./
```

## 运行

### 启动顺序总览

系统采用 **「先启控制器，后启前端」** 的启动模式。所有前端（示教器、脚本、demo）都是 `robot_controller_node` 的客户端，不创建自己的控制器实例。

```
Step 1: Isaac Sim           ← 物理仿真，提供关节反馈和相机图像
Step 2: robot_controller_node ← 100Hz 控制循环（系统核心，唯一发布 /joint_command）
Step 3: 前端（可同时启动多个）
        ├── robot_hmi       ← Qt5 示教器
        ├── Python 脚本      ← robot_api_python RobotClient
        └── demo_vision_grasp ← C++ 视觉抓取演示
```

### 1. 启动 Isaac Sim

在 Isaac Sim 中加载 Franka Panda 场景，确保 ROS2 bridge 已启动。验证话题：

```bash
ros2 topic list
# 应看到: /joint_states, /joint_command, /camera/image_raw/left, /camera/image_raw/depth
```

### 2. 启动控制器节点（必须先于所有前端）

控制器节点是系统的唯一控制核心，负责 100Hz 闭环控制、状态机管理、Service 响应。

```bash
source install/setup.zsh

# 方式一：仅启动控制器
ros2 run robot_controller robot_controller_node

# 方式二：通过 launch 文件启动
ros2 launch robot_bringup controller.launch.py
```

启动后控制器会：
- 订阅 `/joint_states`（Isaac Sim 关节反馈）
- 发布 `/joint_command` @ 100Hz（唯一发布者）
- 提供 11 个 ROS2 Service（move_joint, move_pose, go_home 等）
- 接受 `~/joint_target`（示教器关节流）和 `~/jog_command`（Jog 命令）

### 3. 启动前端（可同时启动多个，互不冲突）

所有前端通过 ROS2 Service / Topic 向 `robot_controller_node` 发送请求，由控制器统一执行。多个前端可以安全共存，因为 `/joint_command` 只有控制器一个发布者。

#### Qt5 示教器

```bash
source install/setup.zsh

# 方式一：launch 一键启动（控制器 + 示教器）
ros2 launch robot_bringup full_system.launch.py

# 方式二：手动启动（控制器已在运行时）
ros2 run robot_hmi robot_hmi
```

示教器通过 `PendantNode` 连接 `robot_controller_node`，使用 Service 调用 + 50Hz 关节流 + 50Hz Jog 心跳。

#### Python 脚本

```bash
source install/setup.zsh
python3 script/test_move_cpp.py         # IK 位姿控制
python3 script/test_grasp_tcp_cpp.py    # TCP 抓取
python3 script/test_vision_cpp.py       # 视觉伺服抓取
```

Python 脚本使用 `RobotClient`（底层是 pybind11 包装的 C++ `RobotClientNode`），通过 ROS2 Service 调用控制机器人。

#### C++ 视觉抓取演示

```bash
source install/setup.zsh
# 注意：demo 已迁移至 robot_api_python 包（不再在 robot_vision 中）
ros2 run robot_api_python demo_vision_grasp
```

此 demo 使用 `RobotClientNode` 连接外部 `robot_controller_node`，不会创建自己的控制器实例。可以和示教器同时运行。

### 4. C++ 抓取演示（独立模式，需单独运行）

```bash
source install/setup.zsh
# 此 demo 内嵌 RobotControllerNode，会自己启动控制器循环
# 不要和其他前端同时运行（会竞争 /joint_command）
ros2 run robot_controller demo_grasp_tcp
```

> **注意**：`demo_grasp_tcp`（在 `robot_controller` 包中）是旧式 demo，内嵌了控制器实例。
> 如果需要和其他前端共存，请使用上面第 3 节中的客户端模式 demo。

### 离线测试（无需 Isaac Sim）

```bash
source install/setup.zsh

# C++ IK 测试
ros2 run robot_controller test_ik_solver
# 预期输出: 结果: 12/12 通过

# Python IK 测试
python3 -c "
import robot_api_python as rc
ik = rc.IKSolver(rc.profiles.panda())
print('FK:', ik.forward([0, 0, 0, -1.57, 0, 1.57, 0]))
print('IK:', ik.solve([0.5, 0, 0.2]))
"
```

## Python 快速开始

### 推荐模式：RobotClient（连接外部节点）

```python
import threading
import robot_api_python as rc

rc.rclcpp_init()

# 创建轻量客户端（连接独立运行的 robot_controller_node）
robot = rc.RobotClient.create("robot_controller_node")

# 后台 spin（rclcpp 节点需要 executor 驱动）
executor = rc.MultiThreadedExecutor()
executor.add_node(robot)
thread = threading.Thread(target=executor.spin, daemon=True)
thread.start()

# 等待服务就绪（确保 robot_controller_node 已启动）
robot.wait_for_services()

# 运动控制（全部通过 ROS2 Service 调用，不直接控制硬件）
ctrl = robot.get_controller()
ctrl.open_gripper()
ctrl.move_to_pose([0.5, 0, 0.3], [0, -3.14, -3.14])
ctrl.close_gripper()
ctrl.move_linear([0, 0, 0.2])

# 清理
executor.cancel()
thread.join()
rc.rclcpp_shutdown()
```

> **为什么 Python 也要用 rclcpp？** 此项目使用 pybind11 将 C++ 核心（IK、轨迹规划、视觉检测）暴露给 Python。
> Python 端通过 `rclcpp`（C++ ROS2 客户端库）而非 `rclpy` 与 ROS2 通信。
> 两者不能共存，因此所有 Python 脚本必须使用 `rc.rclcpp_init()` 而非 `rclpy.init()`。

### ~~已弃用模式：RobotControllerNode（内嵌 C++ 库）~~

> **不要使用此模式**。它会创建自己的 100Hz 控制循环，与已运行的 `robot_controller_node` 竞争 `/joint_command`，
> 导致机器人不可预测运动。仅在没有外部控制器节点的单进程场景下可用。

## 接口隔离

| 抽象接口 | 实现类 | 用途 |
|----------|--------|------|
| `MotionIOBridge` | `RosMotionBridge` | 运动控制与通信解耦 |
| `IRobotController` | `RobotMotionController` / `ServiceRobotController` | 上层只依赖接口 |
| `IVisionProcessor` | `VisionProcessorNode` | 视觉结果查询抽象 |
| `CameraInterface` | `ColorDetector` | 可替换为 YOLO/GraspNet |

## ROS2 话题

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/joint_command` | sensor_msgs/JointState | Pub | 关节指令（9值：7臂+2爪） |
| `/joint_states` | sensor_msgs/JointState | Sub | Isaac Sim 关节反馈 |
| `/camera/image_raw/left` | sensor_msgs/Image | Sub | 左目 RGB |
| `/camera/image_raw/depth` | sensor_msgs/Image | Sub | 深度图 |
| `/robot_controller_node/status` | robot_msgs/RobotStatus | Pub | 机器人状态（10Hz） |
| `/robot_controller_node/jog_command` | robot_msgs/JogCommand | Sub | Jog 点动命令 |
| `/robot_controller_node/joint_target` | sensor_msgs/JointState | Sub | 外部关节目标流（示教器 50Hz） |

## 示教器接口（robot_msgs）

标准化 ROS2 接口，用于连接示教器与机器人控制节点。

### Actions

**MoveJ** — 关节/笛卡尔空间运动，带进度反馈

```bash
ros2 action send /move_j robot_msgs/action/MoveJ "
  mode: 1
  position: {x: 0.5, y: 0.0, z: 0.3}
  orientation: {x: 0.0, y: -1.57, z: -1.57}
  speed_ratio: 0.5
  finger_width: 0.04
"

# 查看反馈
ros2 action feedback /move_j

# 取消运动
ros2 action cancel /move_j
```

**MoveL** — 线性插值运动

```bash
ros2 action send /move_l robot_msgs/action/MoveL "
  position: {x: 0.4, y: 0.1, z: 0.25}
  orientation: {x: 0.0, y: -1.57, z: -1.57}
  frame: 'base'
  speed_ratio: 0.3
  finger_width: 0.04
"
```

### Services

```bash
# 设置全局速度比（与模式速度复合）
ros2 service call /robot_controller_node/set_speed_ratio robot_msgs/srv/SetSpeedRatio "{ratio: 0.8}"

# 急停（任意状态 → kFault）
ros2 service call /robot_controller_node/robot_cmd robot_msgs/srv/RobotCmd "{command: 1}"

# 停止（减速停止）
ros2 service call /robot_controller_node/robot_cmd robot_msgs/srv/RobotCmd "{command: 0}"

# 清除故障（kFault → kIdle）
ros2 service call /robot_controller_node/robot_cmd robot_msgs/srv/RobotCmd "{command: 2}"

# 切换 TCP
ros2 service call /robot_controller_node/set_tcp robot_msgs/srv/SetTCP "{name: 'grasptarget'}"
```

### 状态机

**状态：** kIdle → kMoving → kStopping → kFault（循环）

- kIdle：空闲，可接受命令
- kMoving：Action 执行中
- kTeaching：Jog 点动模式
- kStopping：减速停止
- kFault：故障，需 CLEAR_FAULT

**安全特性：**
- Jog 看门狗：200ms 无命令自动停止（50Hz tick 正常运行时自动刷新看门狗）
- EMERGENCY_STOP：立即进入 kFault
- Action 支持取消
- **运动控制权（MotionOwner）**：NONE / PENDANT / SCRIPT，防止多客户端冲突

## 详细文档

完整 API 接口文档见 [docs/api.md](docs/api.md)，涵盖：
- C++ API 参考（所有公开类、方法签名）
- Python 绑定 API 参考（类型映射、使用示例）
- 示教器接口（Actions、Services、Messages、状态机）
- 开发指南（添加新机器人、替换视觉算法）
