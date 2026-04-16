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
│   │   │   └── nodes/       robot_controller_node.cpp / robot_state.cpp / trajectory_executor.cpp
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
│   └── robot_api_python/          # Python API 封装
│       ├── src/bindings.cpp                # pybind11 绑定代码
│       └── robot_api_python/
│           └── __init__.py                 # Python 包入口
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

## 分层架构

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

### 包间依赖

```
robot_msgs          ← 叶子包（无项目内依赖）
     │
     ▼
robot_description   ← 叶子包（URDF 模型）
     │
     ▼
robot_controller    ← robot_kinematics / robot_motion / robot_nodes
     │
     ▼
robot_vision        ← robot_vision_core / robot_vision_nodes
     │
     ├──▶ robot_hmi           ← Qt5 示教器
     └──▶ robot_api_python    ← pybind11 Python API
```

注意：`robot_controller` → `robot_vision` 单向依赖，不可反向。依赖 `robot_vision` 的集成测试放在 `robot_vision/` 下。

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

**编译顺序：** robot_msgs → robot_description → robot_controller → robot_vision → robot_hmi / robot_api_python

**编译产物：**
- `install/robot_msgs/` — ROS2 接口定义（Services + Actions + Messages）
- `install/robot_description/share/robot_description/urdf/panda.urdf` — URDF 模型
- `install/robot_controller/lib/librobot_kinematics.so` — IK + 轨迹规划
- `install/robot_controller/lib/librobot_motion.so` — 运动控制器 + Jog 控制器
- `install/robot_controller/lib/librobot_nodes.so` — ROS2 节点
- `install/robot_vision/lib/librobot_vision_core.so` — 视觉处理核心
- `install/robot_vision/lib/librobot_vision_nodes.so` — 视觉 ROS2 节点
- `install/robot_api_python/` — pybind11 Python 模块
- `install/robot_hmi/lib/robot_hmi/robot_hmi` — Qt5 示教器可执行文件

### 导出编译数据库（IDE 代码补全）

```bash
colcon build --base-paths src --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
ln -s build/robot_controller/compile_commands.json ./
```

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
ros2 run robot_controller test_ik_solver
# 预期输出: 结果: 12/12 通过
```

### 3. Python IKSolver 离线测试（无需 Isaac Sim）

```bash
source install/setup.zsh
python3 -c "
import robot_api_python as rc
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
ros2 run robot_controller demo_grasp_tcp
```

### 6. Qt5 示教器（需要 Isaac Sim + robot_controller_node 运行中）

```bash
source install/setup.zsh
ros2 run robot_hmi robot_hmi
```

## Python 快速开始

```python
import threading
import robot_api_python as rc

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
| `/robot_controller_node/status` | robot_msgs/RobotStatus | Pub | 机器人状态（10Hz） |
| `/jog_command` | robot_msgs/JogCommand | Sub | Jog 点动命令 |

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

## 详细文档

完整 API 接口文档见 [docs/api.md](docs/api.md)，涵盖：
- C++ API 参考（所有公开类、方法签名）
- Python 绑定 API 参考（类型映射、使用示例）
- 示教器接口（Actions、Services、Messages、状态机）
- 开发指南（添加新机器人、替换视觉算法）
