# Isaac ROS Project

基于 ROS2 Jazzy + NVIDIA Isaac Sim 的机械臂视觉引导抓取系统。

Franka Panda 7-DOF + 二指夹爪，ZED_X_Mini 双目深度相机，Qt5 示教器 GUI。
核心控制逻辑使用 C++17 实现，Python 通过 pybind11 调用 C++ 后端。

## 功能

- **机械臂控制**: IK 位姿控制、关节空间/笛卡尔空间运动、TCP 工具坐标系
- **S 曲线轨迹规划**: 七段式 Jerk 连续轨迹，MoveJ 关节空间 / MoveL 笛卡尔直线
- **奇异位形保护**: IK 求解器内置 DLS 阻尼最小二乘法，自动抑制奇异位形
- **视觉处理**: 双目深度相机 HSV 颜色检测 + 深度 3D 定位
- **视觉引导抓取**: 检测 → 居中 → 下探 → 夹取 → 提起 完整抓取流程
- **Action + Lease 模型**: 多客户端安全共存，Lease 授权 + 自动续约 + 超时释放
- **示教器 GUI**: Qt5 6-Panel 架构，关节/笛卡尔 Jog，夹爪控制，急停
- **Python API**: pybind11 绑定 C++ 核心，通过 `rclcpp` 控制机器人

## 环境要求

- **操作系统**: Ubuntu 24.04
- **ROS2**: Jazzy Jalisco
- **C++**: C++17（GCC 13+）
- **Python**: 3.12
- **仿真**: NVIDIA Isaac Sim
- **相机**: ZED_X_Mini 双目深度相机

## 依赖安装

```bash
# ROS2 核心包 + C++ 依赖
sudo apt install ros-jazzy-rclcpp ros-jazzy-sensor-msgs ros-jazzy-geometry-msgs \
  ros-jazzy-tf2-ros ros-jazzy-cv-bridge ros-jazzy-message-filters \
  ros-jazzy-orocos-kdl ros-jazzy-urdfdom ros-jazzy-kdl-parser \
  ros-jazzy-eigen ros-jazzy-image-transport ros-jazzy-urdf \
  ros-jazzy-ament-index-cpp \
  pybind11-dev

# Qt5 示教器（可选）
sudo apt install qtbase5-dev
```

## 编译

```bash
source /opt/ros/jazzy/setup.zsh

# 编译全部
colcon build --base-paths src

# 编译完成后 source 环境
source install/setup.zsh

# 选择性编译单个包
colcon build --base-paths src --packages-select robot_controller

# 编译示教器及其依赖
colcon build --base-paths src --packages-up-to robot_hmi

# 导出编译数据库（IDE 补全）
colcon build --base-paths src --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
ln -s build/robot_controller/compile_commands.json ./
```

**编译顺序：** robot_msgs → robot_logger + robot_description（可并行）→ robot_controller + robot_vision（可并行）→ robot_tasks + robot_hmi（可并行）→ robot_api_python + robot_demos（可并行）→ robot_bringup

## 运行

### 启动顺序

```
Step 1: Isaac Sim              ← 物理仿真，提供 /joint_states 和相机图像
Step 2: robot_controller_node  ← 100Hz 控制循环（系统核心，唯一发布 /joint_command）
Step 3: 前端（可同时启动多个）
        ├── robot_hmi           ← Qt5 示教器
        ├── robot_tasks_node    ← 抓取任务 Action Server
        ├── Python 脚本          ← robot_api_python RobotClient
        └── demo_vision_grasp   ← C++ 视觉抓取演示
```

### 1. 启动 Isaac Sim

在 Isaac Sim 中加载 Franka Panda 场景，确保 ROS2 bridge 已启动。验证话题：

```bash
ros2 topic list
# 应看到: /joint_states, /camera/image_raw/left, /camera/image_raw/depth
```

### 2. 启动控制器节点（必须先于所有前端）

```bash
source install/setup.zsh

# 方式一：仅启动控制器
ros2 run robot_controller robot_controller_node

# 方式二：通过 launch 文件
ros2 launch robot_bringup controller.launch.py
```

启动后控制器会：
- 订阅 `/joint_states`（Isaac Sim 关节反馈）
- 发布 `/joint_command` @ 100Hz（唯一发布者）
- 提供 11 个 ROS2 Service + 3 个 Action（MoveJ / MoveL / GoHome）
- 提供 Lease 管理（AcquireControl / ReleaseControl / RenewLease）
- 发布 `/robot_controller_node/status` @ 10Hz（状态遥测）

### 3. 启动前端（可同时运行多个）

所有前端通过 ROS2 Action / Service / Topic 向 `robot_controller_node` 发送请求，
由控制器统一执行。多个前端通过 Lease 机制安全共存。

#### 3a. 控制器 + 示教器（一键启动）

```bash
source install/setup.zsh
ros2 launch robot_bringup full_system.launch.py
```

或手动分别启动：

```bash
# 终端 1：控制器
ros2 run robot_controller robot_controller_node

# 终端 2：示教器
ros2 run robot_hmi robot_hmi
```

#### 3b. 抓取任务节点

```bash
source install/setup.zsh
ros2 run robot_tasks robot_tasks_node
```

#### 3c. Python 脚本

```bash
source install/setup.zsh
python3 script/test_move_cpp.py         # IK 位姿控制
python3 script/test_grasp_tcp_cpp.py    # TCP 抓取
python3 script/test_vision_cpp.py       # 视觉引导抓取
python3 script/test_camera_tf.py        # 相机 TF 验证
```

#### 3d. C++ 演示

```bash
source install/setup.zsh

# 视觉引导抓取演示（客户端模式，可和示教器共存）
ros2 run robot_demos demo_vision_grasp

# 相机画面显示
ros2 run robot_demos demo_camera

# 视觉诊断工具（不运动机器人，打印坐标变换链）
ros2 run robot_demos demo_vision_diagnostic

# 集成测试（11 项）
ros2 run robot_demos test_robot_node
```

### 离线测试（无需 Isaac Sim）

```bash
source install/setup.zsh

# C++ IK 测试
ros2 run robot_controller test_ik_solver

# Python IK 测试
python3 -c "
import robot_api_python as rc
ik = rc.IKSolver(rc.profiles.panda())
print('FK:', ik.forward([0, 0, 0, -1.57, 0, 1.57, 0]))
print('IK:', ik.solve([0.5, 0, 0.2]))
"
```

## 系统架构

### 单控制器多客户端模式

```
                         ┌──────────────────────────────────┐
                         │   Isaac Sim (物理仿真引擎)        │
                         │   发布 /joint_states (关节反馈)   │
                         │   接收 /joint_command (关节指令)   │
                         └────────▲──────────────┬──────────┘
                                  │              │
                         /joint_states    /joint_command
                          (反馈)            (100Hz，唯一发布者)
                                  │              │
┌─────────────────────────────────┼──────────────┼──────────────────┐
│  robot_controller_node          │              │                  │
│  (独立进程，系统核心)           │              │                  │
│                                                                 │
│  100Hz 控制循环 + 状态机 + Lease 管理                          │
│  11 个 Service + 3 个 Action (MoveJ/MoveL/GoHome)              │
│  订阅 ~/joint_target + ~/jog_command                           │
└─────────────────────────┬───────────────────────────────────────┘
                          │
             ROS2 Action / Service / Topic
                          │
    ┌─────────────────────┼─────────────────────┐
    │                     │                     │
┌───┴──────────┐  ┌───────┴───────┐  ┌─────────┴──────────┐
│  robot_hmi   │  │  robot_tasks  │  │  Python / C++ Demo │
│  Qt5 示教器  │  │  抓取任务节点 │  │  RobotClient       │
│              │  │               │  │                     │
│ Lease 客户端 │  │ Lease 客户端  │  │ Lease 客户端        │
│ Action 客户端│  │ Action 客户端 │  │ Action 客户端       │
└──────────────┘  └───────────────┘  └─────────────────────┘
```

### Action + Lease 模型

所有运动操作需要 Lease 授权：

1. 客户端调用 `AcquireControl` 获取 Lease（返回 `session_id`）
2. 所有 Action（MoveJ / MoveL / GoHome）需携带 `session_id`
3. Lease 超时自动释放（默认 30s），客户端需定期 RenewLease
4. 同一时刻只有一个客户端持有 Lease，防止多客户端冲突

### 代码分层

```
Layer 4: Python Binding (pybind11)                    ← script/ 通过 C++ 后端控制
Layer 3: C++ Action/Lease Client (robot_client)       ← 轻量客户端，连接外部节点
Layer 2: ROS2 C++ Wrapper Nodes                       ← rclcpp 节点（通信适配层）
Layer 1: Pure C++ Core Library (无 ROS 依赖)           ← kinematics / motion / vision_core
```

## 包结构

| 包名 | 职责 |
|------|------|
| robot_msgs | ROS2 自定义接口（11 Service + 4 Action + 2 Message） |
| robot_logger | 统一日志系统（spdlog，宏接口，文件轮转） |
| robot_description | URDF 机器人模型 + Profile/Camera YAML 配置 |
| robot_controller | 核心运动控制 + Action Server + Lease 管理 + robot_client 客户端库 |
| robot_vision | 视觉处理：HSV 检测 + 深度 3D 定位 + 相机 TF 发布 |
| robot_tasks | 视觉+运动联合任务编排（GraspTask Action Server） |
| robot_hmi | Qt5 示教器 GUI（6 Panel 架构） |
| robot_api_python | pybind11 Python API 绑定 |
| robot_demos | 演示与集成测试 |
| robot_bringup | 启动文件配置 |

### 包间依赖

```
robot_description          robot_logger
    ▲           ▲              ▲
    │           │              │
robot_controller  robot_vision
    ▲           ▲         (平行后端，互不依赖)
    │           │
    └── robot_tasks ──┐
    ▲                 │
robot_hmi         robot_api_python
    ▲                 │
    └── robot_demos ──┘
```

## ROS2 话题

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/joint_command` | sensor_msgs/JointState | Pub | 关节指令（9值：7臂+2爪），100Hz |
| `/joint_states` | sensor_msgs/JointState | Sub | Isaac Sim 关节反馈 |
| `/camera/image_raw/left` | sensor_msgs/Image | Sub | 左目 RGB |
| `/camera/image_raw/depth` | sensor_msgs/Image | Sub | 深度图 |
| `/robot_controller_node/status` | robot_msgs/RobotStatus | Pub | 机器人状态遥测（10Hz） |
| `/robot_controller_node/jog_command` | robot_msgs/JogCommand | Sub | Jog 点动命令 |
| `/robot_controller_node/joint_target` | sensor_msgs/JointState | Sub | 外部关节目标流（示教器 50Hz） |

## Python 快速开始

```python
import threading
import robot_api_python as rc

rc.rclcpp_init()

# 创建客户端（连接独立运行的 robot_controller_node）
robot = rc.RobotClient.create("robot_controller_node")

# 后台 spin
executor = rc.MultiThreadedExecutor()
executor.add_node(robot)
thread = threading.Thread(target=executor.spin, daemon=True)
thread.start()

robot.wait_for_services()

# 获取控制权
session_id = robot.acquire_control("my_script")
ctrl = robot.get_controller()

# 运动控制（通过 Action 调用）
ctrl.open_gripper()
ctrl.move_to_pose([0.5, 0, 0.3], [0, -3.14, -3.14])
ctrl.close_gripper()

# 释放控制权
robot.release_control()

executor.cancel()
thread.join()
rc.rclcpp_shutdown()
```

> Python 使用 `rclcpp`（通过 pybind11），禁止同时使用 `rclpy`。

## 坐标系

- 基坐标系: `panda_link0`
- 末端坐标系: `panda_hand`（法兰）/ 自定义 TCP
- 相机坐标系: `camera_color_optical_frame`（eye-in-hand 配置）
- TF 链: `panda_link0` → ... → `panda_hand` → `camera_link` → `camera_color_optical_frame`

## 状态机

- 状态: IDLE → MOVING / TEACHING → STOPPING → FAULT
- 任意状态可通过 EMERGENCY_STOP 进入 FAULT
- FAULT 通过 CLEAR_FAULT 恢复到 IDLE
- Jog 看门狗: 200ms 无命令自动停止
