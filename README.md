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
- **示教器接口**: 标准化 ROS2 Actions（MoveJ/MoveL）带进度反馈，状态机保护，Jog 点动，急停与看门狗

## 环境要求

- **操作系统**: Ubuntu 24.04
- **ROS2**: Jazzy Jalisco
- **C++**: C++17 编译器（GCC 13+）
- **Python**: 3.12
- **仿真**: NVIDIA Isaac Sim（提供 `/joint_states` 和相机话题）
- **相机**: ZEN_X_Mini 双目深度相机

## 依赖安装

### Python 依赖（仅纯 Python 后端需要）

```bash
pip install ikpy scipy opencv-python
```

### C++ 依赖（ROS2 包，通过 apt 安装）

```bash
sudo apt install ros-jazzy-rclcpp ros-jazzy-sensor-msgs ros-jazzy-geometry-msgs \
  ros-jazzy-tf2-ros ros-jazzy-cv-bridge ros-jazzy-message-filters \
  ros-jazzy-orocos-kdl ros-jazzy-urdfdom ros-jazzy-kdl-parser \
  ros-jazzy-eigen ros-jazzy-image-transport ros-jazzy-urdf \
  pybind11-dev
```

## 项目结构

```
isaac_ros_project/
├── src/
│   ├── robot_control_msgs/          # 原始服务接口（保留，向后兼容）
│   │   └── srv/                     # 8 个 Service（SolveIK, MoveJoint, MovePose 等）
│   │
│   ├── arm_control_interfaces/      # 示教器接口包（Actions + Services + Messages）
│   │   ├── action/                  # Actions（带进度反馈）
│   │   │   ├── MoveJ.action         # 关节/笛卡尔运动 Action
│   │   │   └── MoveL.action         # 线性运动 Action
│   │   ├── srv/                     # Services
│   │   │   ├── SetTCP.srv           # 设置 TCP
│   │   │   ├── SetSpeedRatio.srv    # 全局速度比
│   │   │   └── RobotCmd.srv         # STOP/EMERGENCY_STOP/CLEAR_FAULT
│   │   └── msg/                     # Messages
│   │       ├── RobotStatus.msg      # 状态 + 遥测（10Hz 发布）
│   │       └── JogCommand.msg       # Jog 点动命令
│   │
│   ├── robot_control_cpp/          # C++ 核心库包（纯库，4 个 CMake target）
│   │   ├── include/robot_control_cpp/
│   │   │   ├── profiles/                   # 机器人配置文件
│   │   │   │   └── panda_profile.hpp       # Panda 专用配置
│   │   │   ├── kinematics/                 # robot_kinematics target（零 ROS 依赖）
│   │   │   │   ├── robot_profile.hpp       # RobotProfile / GripperProfile / TcpConfig
│   │   │   │   ├── ik_solver.hpp           # IK/FK 求解器（KDL + DLS）
│   │   │   │   └── trajectory_planner.hpp  # S 曲线轨迹规划器
│   │   │   ├── motion/                     # robot_motion target（依赖 kinematics）
│   │   │   │   ├── control_constants.hpp   # 通用控制常量
│   │   │   │   ├── i_robot_controller.hpp  # IRobotController 抽象接口
│   │   │   │   ├── motion_io_bridge.hpp    # MotionIOBridge 接口
│   │   │   │   └── robot_motion_controller.hpp
│   │   │   ├── vision/                     # robot_vision target（零 ROS 依赖）
│   │   │   │   ├── i_vision_processor.hpp  # IVisionProcessor 抽象接口
│   │   │   │   ├── camera_interface.hpp    # CameraInterface 基类
│   │   │   │   └── color_detector.hpp      # ColorDetector（OpenCV HSV）
│   │   │   └── nodes/                      # robot_nodes target（依赖 motion + vision）
│   │   │       ├── topic_config.hpp        # TopicConfig + CameraExtrinsics
│   │   │       ├── robot_state.hpp         # RobotStateMachine 状态机
│   │   │       ├── trajectory_executor.hpp # 非阻塞轨迹执行器
│   │   │       ├── grasp_task_manager.hpp  # GraspTaskManager 状态机
│   │   │       ├── robot_controller_node.hpp
│   │   │       └── vision_processor_node.hpp
│   │   └── src/
│   │       ├── kinematics/  ik_solver.cpp / trajectory_planner.cpp
│   │       ├── motion/      robot_motion_controller.cpp
│   │       ├── vision/      color_detector.cpp
│   │       └── nodes/       robot_controller_node.cpp / vision_processor_node.cpp / grasp_task_manager.cpp
│   │
│   ├── robot_control_cpp_py/       # pybind11 Python 绑定包
│   │   ├── src/bindings.cpp                # 绑定代码
│   │   └── robot_control_cpp_py/
│   │       └── __init__.py                 # Python 包入口
│   │
│   ├── robot_control_test/         # C++ 测试与演示包
│   │   ├── test/
│   │   │   ├── test_ik_solver.cpp          # 独立 IK 测试（离线）
│   │   │   ├── test_trajectory_planner.cpp # S 曲线规划器测试（离线）
│   │   │   └── test_robot_node.cpp         # 集成测试（需 Isaac Sim）
│   │   └── demo/
│   │       └── demo_grasp_tcp.cpp          # TCP 抓取演示
│   │
│   └── teaching_pendant/           # Qt5 示教器包
│       ├── include/teaching_pendant/
│       │   ├── pendant_node.hpp            # ROS2 节点（Action/Service 客户端）
│       │   └── main_window.hpp             # Qt5 主窗口
│       └── src/
│           ├── pendant_node.cpp            # 通信后端实现
│           ├── main_window.cpp             # GUI 实现
│           └── main.cpp                    # 入口
│
├── script/
│   ├── test_move_cpp.py            # Python + C++ 后端：IK 位姿控制
│   ├── test_grasp_tcp_cpp.py       # Python + C++ 后端：TCP 抓取
│   ├── test_vision_cpp.py          # Python + C++ 后端：视觉伺服抓取
│   ├── test_move.py                # 纯 Python 后端：IK 位姿控制
│   ├── test_vision.py              # 纯 Python 后端：视觉伺服
│   ├── test_grasp_tcp.py           # 纯 Python 后端：TCP 抓取
│   ├── test_joint_state.py         # 关节状态读取
│   └── test_camera.py              # 相机图像显示
│
├── docs/
│   ├── superpowers/                # 开发流程文档
│   │   ├── specs/                  # 设计规格
│   │   │   └── 2026-04-10-pendant-interface-design.md
│   │   └── plans/                  # 实现计划
│   │       └── 2026-04-10-pendant-interface.md
│   └── api.md                      # 完整 API 接口文档
├── urdf/
│   └── panda.urdf                  # Franka Panda URDF 模型
├── CLAUDE.md
└── README.md
```

## 分层架构

```
┌──────────────────────────────────────────────────────────┐
│  Layer 3: Python Binding (pybind11)                      │
│  script/ 代码通过 C++ 后端控制机器人                     │
├──────────────────────────────────────────────────────────┤
│  Layer 2: robot_nodes（唯一有 ROS 依赖的 target）        │
│  ┌──────────────────────┐ ┌───────────────────────────┐  │
│  │ RobotControllerNode  │ │ VisionProcessorNode       │  │
│  │ + RosMotionBridge    │ │ + GraspTaskManager        │  │
│  └──────────┬───────────┘ └──────────┬────────────────┘  │
├─────────────┼────────────────────────┼───────────────────┤
│  Layer 1: Pure C++ Core (无 ROS 依赖)│                   │
│  ┌──────────▼──────────┐ ┌───────────▼────────────────┐  │
│  │ robot_kinematics    │ │ robot_vision               │  │
│  │ IKSolver (KDL + DLS)│ │ ColorDetector (OpenCV HSV) │  │
│  │ SCurvePlanner       │ │ CameraInterface            │  │
│  │ RobotProfile        │ │ IVisionProcessor           │  │
│  └──────────┬──────────┘ └────────────────────────────┘  │
│             │                                            │
│  ┌──────────▼──────────────────────────────────────────┐ │
│  │ robot_motion                                        │ │
│  │ RobotMotionController + IRobotController            │ │
│  │ MotionIOBridge                                      │ │
│  └─────────────────────────────────────────────────────┘ │
└──────────────────────────────────────────────────────────┘
```

### 包间依赖

```
robot_control_test ──→ robot_control_cpp（链接 robot_kinematics / robot_motion / robot_nodes）
robot_control_cpp_py ──→ robot_control_cpp（链接全部 4 个 target）
script/*.py ──→ robot_control_cpp_py（运行时）
```

## 编译

```bash
source /opt/ros/jazzy/setup.zsh

# 一键编译全部（核心库 + 绑定 + 测试 + 示教器）
colcon build --base-paths src

# 或分步编译（按依赖顺序）
colcon build --base-paths src --packages-select robot_control_msgs        # 原始服务接口
colcon build --base-paths src --packages-select arm_control_interfaces    # 示教器接口
colcon build --base-paths src --packages-select robot_control_cpp         # C++ 核心库
source install/setup.zsh
colcon build --base-paths src --packages-select robot_control_cpp_py      # Python 绑定
colcon build --base-paths src --packages-select robot_control_test        # C++ 测试
colcon build --base-paths src --packages-select teaching_pendant          # Qt5 示教器
```

**编译产物：**
- `install/robot_control_msgs/` — ROS2 服务定义
- `install/arm_control_interfaces/` — ROS2 Actions/Services/Messages 定义
- `install/robot_control_cpp/lib/librobot_kinematics.so` — IK + 轨迹规划
- `install/robot_control_cpp/lib/librobot_motion.so` — 运动控制器
- `install/robot_control_cpp/lib/librobot_vision.so` — 视觉处理
- `install/robot_control_cpp/lib/librobot_nodes.so` — ROS2 节点
- `install/robot_control_cpp_py/` — pybind11 Python 模块
- `install/teaching_pendant/lib/teaching_pendant` — Qt5 示教器可执行文件

## 运行

### 1. 启动 Isaac Sim

在 Isaac Sim 中加载 Franka Panda 场景，确保 ROS2 bridge 已启动。验证话题：

```bash
ros2 topic list
# 应看到: /joint_states, /joint_command, /camera/image_raw/left, /camera/image_raw/depth
```

### 2. C++ IK 独立测试（无需 Isaac Sim）

```bash
source install/setup.zsh
ros2 run robot_control_test test_ik_solver
# 预期输出: 结果: 12/12 通过
```

### 3. Python IKSolver 离线测试（无需 Isaac Sim）

```bash
source install/setup.zsh
python3 -c "
import robot_control_cpp_py as rc
ik = rc.IKSolver(rc.profiles.panda())
print('FK:', ik.forward([0, 0, 0, -1.57, 0, 1.57, 0]))
print('IK:', ik.solve([0.5, 0, 0.2]))
"
```

### 4. Python 演示（C++ 后端，需要 Isaac Sim）

```bash
source install/setup.zsh
python3 script/test_move_cpp.py         # IK 位姿控制
python3 script/test_grasp_tcp_cpp.py    # TCP 抓取
python3 script/test_vision_cpp.py       # 视觉伺服抓取
```

### 5. C++ 抓取演示（需要 Isaac Sim）

```bash
source install/setup.zsh
ros2 run robot_control_test demo_grasp_tcp
```

## Python 快速开始

```python
import threading
import robot_control_cpp_py as rc

# 初始化 rclcpp（替代 rclpy.init，二者不能共存）
rc.rclcpp_init()

# 创建机器人控制节点
robot = rc.RobotControllerNode.create(
    rc.profiles.panda(), rc.profiles.panda_gripper(), rc.TopicConfig())

# 后台 spin
executor = rc.MultiThreadedExecutor()
executor.add_node(robot)
thread = threading.Thread(target=executor.spin, daemon=True)
thread.start()

# 等待连接
robot.wait_for_ready()

# 运动控制
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

## 接口隔离

| 抽象接口 | 实现类 | 用途 |
|----------|--------|------|
| `MotionIOBridge` | `RosMotionBridge` | 运动控制与通信解耦 |
| `IRobotController` | `RobotMotionController` | 上层只依赖接口 |
| `IVisionProcessor` | `VisionProcessorNode` | 视觉结果查询抽象 |
| `CameraInterface` | `ColorDetector` | 可替换为 YOLO/GraspNet |

## ROS2 话题

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/joint_command` | sensor_msgs/JointState | Pub | 关节指令（9值：7臂+2爪） |
| `/joint_states` | sensor_msgs/JointState | Sub | Isaac Sim 关节反馈 |
| `/camera/image_raw/left` | sensor_msgs/Image | Sub | 左目 RGB |
| `/camera/image_raw/depth` | sensor_msgs/Image | Sub | 深度图 |
| `/robot_controller_node/status` | arm_control_interfaces/RobotStatus | Pub | 机器人状态（10Hz） |
| `/jog_command` | arm_control_interfaces/JogCommand | Sub | Jog 点动命令 |

## 示教器接口（arm_control_interfaces）

标准化 ROS2 接口，用于连接示教器与机器人控制节点。

### Actions

**MoveJ** — 关节/笛卡尔空间运动，带进度反馈

```bash
# 发送 Goal（笛卡尔模式，IK 自动求解）
ros2 action send /move_j arm_control_interfaces/action/MoveJ "
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
ros2 action send /move_l arm_control_interfaces/action/MoveL "
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
ros2 service call /robot_controller_node/set_speed_ratio arm_control_interfaces/srv/SetSpeedRatio "{ratio: 0.8}"

# 急停（任意状态 → kFault）
ros2 service call /robot_controller_node/robot_cmd arm_control_interfaces/srv/RobotCmd "{command: 1}"

# 停止（减速停止）
ros2 service call /robot_controller_node/robot_cmd arm_control_interfaces/srv/RobotCmd "{command: 0}"

# 清除故障（kFault → kIdle）
ros2 service call /robot_controller_node/robot_cmd arm_control_interfaces/srv/RobotCmd "{command: 2}"

# 切换 TCP
ros2 service call /robot_controller_node/set_tcp arm_control_interfaces/srv/SetTCP "{name: 'grasptarget'}"
```

### 状态机

**状态：** kIdle → kMoving → kStopping → kFault（循环）

- kIdle：空闲，可接受命令
- kMoving：Action 执行中
- kTeaching：Jog 点动模式
- kStopping：减速停止
- kFault：故障，需 CLEAR_FAULT

**安全特性：**
- Jog 看门狗：200ms 无命令自动停止
- EMERGENCY_STOP：立即进入 kFault
- Action 支持取消

## 详细文档

完整 API 接口文档见 [docs/api.md](docs/api.md)，涵盖：
- C++ API 参考（所有公开类、方法签名）
- Python 绑定 API 参考（类型映射、使用示例）
- 示教器接口（Actions、Services、Messages、状态机）
- 开发指南（添加新机器人、替换视觉算法）
