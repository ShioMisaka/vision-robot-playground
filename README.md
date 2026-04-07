# Robot Vision Grasp

基于 ROS2 + Isaac Sim 的机械臂视觉引导抓取系统，采用 C++ 核心库 + Python 上层脚本的多语言分层架构，支持多机器人扩展。

## 功能

- **机械臂控制**: 关节角度指令、IK 位姿控制、相对平移/旋转、TCP 工具坐标系
- **视觉处理**: 双目深度相机左目+深度同步订阅，HSV 颜色目标检测
- **视觉伺服抓取**: 图像反馈闭环 — 检测 → 居中 → 下探 → 夹取 → 提起
- **TF2 集成**: 自动发布末端执行器 TF 变换链，支持坐标系变换查询
- **多机器人扩展**: 通过 `RobotProfile` 配置驱动，核心库无需修改

## 环境要求

- Python 3.10+
- ROS2 Jazzy
- C++17 编译器（GCC 9+ / Clang 10+）
- NVIDIA Isaac Sim（提供 `/joint_states` 和相机话题）
- ZEN_X_Mini 双目深度相机

## 依赖安装

### Python 依赖

```bash
pip install ikpy scipy opencv-python
```

### C++ 依赖（ROS2 包，通过 apt 安装）

```bash
sudo apt install ros-jazzy-rclcpp ros-jazzy-sensor-msgs ros-jazzy-geometry-msgs \
  ros-jazzy-tf2-ros ros-jazzy-cv-bridge ros-jazzy-message-filters \
  ros-jazzy-orocos-kdl ros-jazzy-urdfdom ros-jazzy-kdl-parser \
  ros-jazzy-eigen ros-jazzy-image-transport
```

## 项目结构

```
src/
  # Python 原始模块
  config.py            # 话题名、关节名、TF Frame、预设姿态
  ik_solver.py         # IK/FK 求解器（基于 URDF + ikpy）
  vision.py            # HSV 颜色检测器
  robot.py             # 机械臂控制节点
  vision_processor.py  # 相机同步节点（左目 + 深度）
  task_manager.py      # 抓取任务状态机

  # C++ 核心包（ament_cmake）
  robot_control_cpp/
    include/robot_control_cpp/
      config.hpp                  # 话题配置 + 通用常量
      robot_profile.hpp           # 机器人/夹爪/TCP 参数描述
      i_robot_controller.hpp      # 运动控制抽象接口
      i_vision_processor.hpp      # 视觉处理抽象接口
      ik_solver.hpp               # KDL IK/FK 求解器
      color_detector.hpp          # OpenCV HSV 检测器
      robot_motion_controller.hpp # 通用运动控制器
      grasp_task_manager.hpp      # 抓取状态机
      robot_controller_node.hpp   # ROS2 机器人控制节点
      vision_processor_node.hpp   # ROS2 视觉处理节点
      panda_profile.hpp           # Franka Panda 配置
    src/  *.cpp
    CMakeLists.txt / package.xml

script/
  test_move.py / test_vision.py / test_joint_state.py / test_camera.py / test_grasp_tcp.py
urdf/
  panda.urdf
```

## 快速开始

### 1. 编译 C++ 包

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --packages-select robot_control_cpp
source install/setup.bash
```

### 2. 启动 Isaac Sim

在 Isaac Sim 中加载 Franka Panda 场景，确保 ROS2 bridge 已启动，以下话题可用：

```
/joint_states          # 关节状态反馈
/joint_command         # 关节指令（订阅）
/camera/image_raw/left # 左目相机
/camera/image_raw/depth# 深度图
```

### 3. 运行 Python 演示

```bash
# 位姿控制
python3 script/test_move.py

# 视觉引导抓取（红色物块）
python3 script/test_vision.py
```

## 架构说明

### 分层设计

```
┌─────────────────────────────────────────────┐
│  Layer 3: Python Binding (pybind11)         │  ← 待实现
├─────────────────────────────────────────────┤
│  Layer 2: ROS 2 C++ Wrapper Nodes          │  ← rclcpp 通信适配
│  ├─ RobotControllerNode                     │
│  └─ VisionProcessorNode                     │
├─────────────────────────────────────────────┤
│  Layer 1: Pure C++ Core (无 ROS 依赖)       │  ← 可独立编译测试
│  ├─ IKSolver (KDL)                          │
│  ├─ ColorDetector (OpenCV)                  │
│  ├─ RobotMotionController                   │
│  └─ GraspTaskManager                        │
└─────────────────────────────────────────────┘
```

### 接口隔离

- `MotionIOBridge`: 抽离运动控制逻辑与底层通信，ROS 节点实现该接口
- `IRobotController` / `IVisionProcessor`: 上层编排只依赖接口
- `CameraInterface`: 图像处理算法基类，子类接入 YOLO/GraspNet 等

### 扩展新机器人

只需添加一个 `xxx_profile.hpp` 定义 `RobotProfile`：

```cpp
// include/robot_control_cpp/my_robot_profile.hpp
namespace robot_control::profiles {

inline RobotProfile my_robot() {
  RobotProfile p;
  p.name = "my_robot";
  p.urdf_path = "urdf/my_robot.urdf";
  p.dof = 6;
  // ... 关节名、限位、home 位等
  return p;
}

}  // namespace robot_control::profiles
```

核心库（IKSolver、RobotMotionController、GraspTaskManager）无需任何修改。

## ROS2 话题一览

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/joint_command` | sensor_msgs/JointState | 发布 | 9 值：7 臂关节 + 2 夹爪 |
| `/joint_states` | sensor_msgs/JointState | 订阅 | Isaac Sim 关节反馈 |
| `/camera/image_raw/left` | sensor_msgs/Image | 订阅 | 左目 RGB 图像 |
| `/camera/image_raw/depth` | sensor_msgs/Image | 订阅 | 深度图 |

## 视觉伺服参数调参

`script/test_vision.py` 顶部的常量控制伺服行为：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `DIRECTION_X` | -1.0 | 图像右偏时机器人 X 移动方向 |
| `DIRECTION_Y` | 1.0 | 图像下偏时机器人 Y 移动方向 |
| `MOVE_STEP` | 0.02 | 每次居中调整步长（米） |
| `CENTER_THRESHOLD` | 0.03 | 居中判定阈值（归一化偏移） |
| `DESCEND_DISTANCE` | 0.15 | 下探距离（米） |
| `LIFT_DISTANCE` | 0.25 | 提起距离（米） |
