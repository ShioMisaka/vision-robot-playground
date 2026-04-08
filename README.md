# Robot Vision Grasp

基于 ROS2 + Isaac Sim 的机械臂视觉引导抓取系统。C++ 核心库 + pybind11 Python 绑定的多语言分层架构，支持多机器人扩展。

## 功能

- **机械臂控制**: 关节角度指令、IK 位姿控制、相对平移/旋转、TCP 工具坐标系
- **视觉处理**: 双目深度相机左目+深度同步订阅，HSV 颜色目标检测
- **视觉伺服抓取**: 图像反馈闭环 — 检测 → 居中 → 下探 → 夹取 → 提起
- **TF2 集成**: 自动发布末端执行器 TF 变换链，支持坐标系变换查询
- **多机器人扩展**: 通过 `RobotProfile` 配置驱动，核心库无需修改
- **Python 绑定**: pybind11 将 C++ 核心库暴露给 Python，脚本可直接调用 C++ 后端

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
│   ├── robot_control_cpp/          # C++ 核心库包（纯库）
│   │   ├── include/robot_control_cpp/
│   │   │   ├── config.hpp                  # TopicConfig + ControlConstants
│   │   │   ├── robot_profile.hpp           # RobotProfile / GripperProfile / TcpConfig
│   │   │   ├── i_robot_controller.hpp      # IRobotController 抽象接口
│   │   │   ├── i_vision_processor.hpp      # IVisionProcessor 抽象接口
│   │   │   ├── ik_solver.hpp               # IK/FK 求解器（KDL）
│   │   │   ├── color_detector.hpp          # CameraInterface + ColorDetector
│   │   │   ├── robot_motion_controller.hpp # RobotMotionController + MotionIOBridge
│   │   │   ├── grasp_task_manager.hpp      # GraspTaskManager 状态机
│   │   │   ├── robot_controller_node.hpp   # RobotControllerNode（ROS2）
│   │   │   ├── vision_processor_node.hpp   # VisionProcessorNode（ROS2）
│   │   │   └── panda_profile.hpp           # Panda 专用配置
│   │   └── src/                            # 实现文件
│   │
│   ├── robot_control_cpp_py/       # pybind11 Python 绑定包
│   │   ├── src/bindings.cpp                # 绑定代码
│   │   └── robot_control_cpp_py/
│   │       └── __init__.py                 # Python 包入口
│   │
│   ├── robot_control_test/         # C++ 测试与演示包
│   │   ├── test/
│   │   │   ├── test_ik_solver.cpp          # 独立 IK 测试（离线）
│   │   │   └── test_robot_node.cpp         # 集成测试（需 Isaac Sim）
│   │   └── demo/
│   │       └── demo_grasp_tcp.cpp          # TCP 抓取演示
│   │
│   └── *.py                        # Python 原始模块（保留供参考）
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
├── doc/
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
│  Layer 2: ROS 2 C++ Wrapper Nodes                        │
│  ┌──────────────────────┐ ┌───────────────────────────┐  │
│  │ RobotControllerNode  │ │ VisionProcessorNode       │  │
│  │ + RosMotionBridge    │ │ + ApproximateTime Sync    │  │
│  └──────────┬───────────┘ └──────────┬────────────────┘  │
├─────────────┼────────────────────────┼───────────────────┤
│  Layer 1: Pure C++ Core(无 ROS 依赖) │                   │
│  ┌──────────▼────────────────────────▼───────────────┐   │
│  │ IKSolver (KDL)  │ RobotMotionController           │   │
│  │ ColorDetector   │ GraspTaskManager                │   │
│  └───────────────────────────────────────────────────┘   │
└──────────────────────────────────────────────────────────┘
```

### 包间依赖

```
robot_control_test ──→ robot_control_cpp
robot_control_cpp_py ──→ robot_control_cpp
script/*.py ──→ robot_control_cpp_py（运行时）
```

## 编译

```bash
source /opt/ros/jazzy/setup.zsh

# 一键编译全部（核心库 + 绑定 + 测试）
colcon build --base-paths src --packages-up-to robot_control_cpp_py

# 或分步编译
colcon build --base-paths src --packages-select robot_control_cpp     # 核心库
source install/setup.zsh
colcon build --base-paths src --packages-select robot_control_cpp_py  # Python 绑定
colcon build --base-paths src --packages-select robot_control_test    # C++ 测试
```

**编译产物：**
- `install/robot_control_cpp/lib/librobot_control_core.so` — Layer 1 核心库
- `install/robot_control_cpp/lib/librobot_control_nodes.so` — Layer 2 ROS2 节点库
- `install/robot_control_cpp_py/` — pybind11 Python 模块

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

## 详细文档

完整 API 接口文档见 [doc/api.md](doc/api.md)，涵盖：
- C++ API 参考（所有公开类、方法签名）
- Python 绑定 API 参考（类型映射、使用示例）
- 开发指南（添加新机器人、替换视觉算法）
