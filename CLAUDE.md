# Claude Code 项目指南

## 项目概述
基于 ROS2 + Isaac Sim 的机械臂视觉引导抓取系统（多机器人可扩展架构）。
使用 ZEN_X_Mini 双目深度相机的左目 + 深度通道进行目标检测与 3D 定位。
核心逻辑已用 C++ 重写，通过分层架构实现 ROS 无关的核心库与 ROS2 节点解耦。

## 技术栈
- **语言**: C++17（核心库 + ROS2 节点）, Python 3.10+（测试脚本、上层算法）
- **ROS2**: Jazzy
- **仿真**: NVIDIA Isaac Sim（通过 ROS2 bridge 通信）
- **机器人**: Franka Panda 7-DOF + 二指夹爪（架构支持扩展新机器人）
- **相机**: ZEN_X_Mini（左目 RGB + 深度）
- **C++ 核心依赖**: rclcpp, Eigen3, Orocos KDL, OpenCV, tf2_ros, cv_bridge, message_filters

## 分层架构
```
Layer 3: Python Binding (pybind11)           ← 待实现：script/ 代码无缝迁移
Layer 2: ROS 2 C++ Wrapper Nodes            ← rclcpp 节点（通信适配层）
Layer 1: Pure C++ Core Library (无 ROS 依赖) ← IK、运动控制、颜色检测、状态机
```

- Layer 1 通过 `MotionIOBridge` 抽象接口与通信解耦，ROS2 节点实现该接口
- 新增机器人只需定义 `RobotProfile`，核心库无需修改

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

  # === C++ 核心包 ===
  robot_control_cpp/
    include/robot_control_cpp/
      config.hpp                  # 话题配置 TopicConfig + 通用常量 ControlConstants
      robot_profile.hpp           # RobotProfile / GripperProfile / TcpConfig（多机器人）
      i_robot_controller.hpp      # 运动控制抽象接口 IRobotController
      i_vision_processor.hpp      # 视觉处理抽象接口 IVisionProcessor
      ik_solver.hpp               # IK/FK 求解器（KDL，不依赖 rclcpp）
      color_detector.hpp          # CameraInterface 基类 + ColorDetector（OpenCV HSV）
      robot_motion_controller.hpp # 通用运动控制器 + MotionIOBridge 接口
      grasp_task_manager.hpp      # 抓取状态机 GraspTaskManager
      robot_controller_node.hpp   # ROS2 机器人控制节点 RobotControllerNode
      vision_processor_node.hpp   # ROS2 视觉处理节点 VisionProcessorNode
      panda_profile.hpp           # Panda 专用配置（扩展新机器人参考）
    src/
      ik_solver.cpp / color_detector.cpp / robot_motion_controller.cpp
      grasp_task_manager.cpp / robot_controller_node.cpp / vision_processor_node.cpp
    CMakeLists.txt
    package.xml

script/
  test_move.py         # Python 演示：IK 位姿控制
  test_vision.py       # Python 演示：视觉伺服引导抓取（红色物块）
  test_joint_state.py  # Python 演示：关节状态读取
  test_camera.py       # Python 演示：相机图像显示
  test_grasp_tcp.py    # Python 演示：TCP 抓取
urdf/
  panda.urdf           # Franka Panda URDF 模型
```

## 编码规范 — Python
- 遵循 PEP8，完整 Type Hints + 中文 Docstring
- 所有 ROS2 节点通过 MultiThreadedExecutor 运行，禁止使用 `rclpy.spin_once`
- 线程同步使用 `threading.Event`（就绪等待）和 `threading.Lock`（共享数据保护）

## 编码规范 — C++
- 遵循 Google C++ Style Guide，类名 PascalCase，方法/变量 snake_case
- 禁止裸指针，全面使用智能指针 (`std::shared_ptr`, `std::unique_ptr`)
- 所有公开头文件 `.hpp` 必须包含 Doxygen 注释（`@brief`, `@param`, `@return`）
- 接口隔离：ROS2 通信逻辑（Layer 2）不得污染纯核心逻辑（Layer 1）
- 异常处理：C++ 层捕获通信/超时异常，通过 pybind11 抛出到 Python
- 线程同步：`std::mutex` + `std::condition_variable`
- 回调组：状态订阅用 `MutuallyExclusiveCallbackGroup`，发布/TF 用 `ReentrantCallbackGroup`

## ROS2 话题
| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/joint_command` | sensor_msgs/JointState | Pub | 关节指令（9值：7臂+2爪） |
| `/joint_states` | sensor_msgs/JointState | Sub | Isaac Sim 关节反馈 |
| `/camera/image_raw/left` | sensor_msgs/Image | Sub | 左目 RGB |
| `/camera/image_raw/depth` | sensor_msgs/Image | Sub | 深度图（uint16 mm 或 float32 m） |

## 常用命令
```bash
# 编译 C++ 包
source /opt/ros/jazzy/setup.bash && colcon build --base-paths src --packages-select robot_control_cpp

# 运行 Python 位姿控制演示
python3 script/test_move.py

# 运行 Python 视觉引导抓取演示
python3 script/test_vision.py

# 查看 ROS2 话题
ros2 topic list
```

## 架构约定
- RobotController 不依赖 VisionProcessor，二者通过 GraspTaskManager 编排
- VisionProcessor/CameraInterface::process_image() 是桩代码，子类重写以接入 YOLO/GraspNet
- 视觉伺服参数定义在 script 中，非节点级参数
- TF2: RobotControllerNode 自动发布 base → hand → tcp 变换链
- 新增机器人：在 `include/robot_control_cpp/` 下添加 `xxx_profile.hpp`，定义 `RobotProfile`
