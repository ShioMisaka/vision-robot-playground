# robot_control_cpp 解耦重构设计

## 目标

将 `robot_control_cpp` 从当前的 2 target（core + nodes）重构为 4 target，实现运动学、运动控制、视觉、ROS2 节点的完全解耦，提升独立复用性、可测试性和代码可维护性。

## Target 分层架构

```
robot_vision       (零外部依赖，仅 OpenCV + Eigen)
       ▲
       │
robot_kinematics   (零外部依赖，仅 KDL + Eigen)
       ▲
       │
robot_motion       ← 依赖 kinematics
       ▲
       │
robot_nodes        ← 依赖 motion + vision
```

- `robot_vision` 和 `robot_kinematics` 是叶子节点，互相不依赖，零 ROS 依赖
- `robot_motion` 只依赖 `robot_kinematics`，不依赖 vision
- `robot_nodes` 组合 motion + vision，是唯一有 ROS 依赖的 target
- 编译顺序：`vision` 和 `kinematics` 可并行 → `motion` → `nodes`

## 各模块内容

### robot_kinematics

```
include/robot_control_cpp/kinematics/
  ik_solver.hpp              # IKSolver（KDL + DLS）
  trajectory_planner.hpp     # TrajectoryPlanner + SCurvePlanner
  robot_profile.hpp          # RobotProfile, GripperProfile, TcpConfig, MotionLimits
src/kinematics/
  ik_solver.cpp
  trajectory_planner.cpp
```

外部依赖：Orocos KDL, urdfdom, urdf, kdl_parser, Eigen3

### robot_motion

```
include/robot_control_cpp/motion/
  i_robot_controller.hpp     # IRobotController 接口 + MotionMode 枚举
  motion_io_bridge.hpp       # MotionIOBridge 接口
  robot_motion_controller.hpp  # RobotMotionController 实现
  control_constants.hpp      # 控制频率、超时等常量
src/motion/
  robot_motion_controller.cpp
```

外部依赖：无新增（继承 kinematics）
- IKSolver 封装在 RobotMotionController 内部，不对外暴露
- ControlConstants 从原 config.hpp 迁出

### robot_vision

```
include/robot_control_cpp/vision/
  camera_interface.hpp       # CameraInterface 基类
  i_vision_processor.hpp     # IVisionProcessor 接口 + DetectionResult
  color_detector.hpp         # ColorDetector（OpenCV HSV）
src/vision/
  color_detector.cpp
```

外部依赖：OpenCV, Eigen3

### robot_nodes

```
include/robot_control_cpp/nodes/
  robot_controller_node.hpp  # RobotControllerNode（rclcpp::Node）
  vision_processor_node.hpp  # VisionProcessorNode（rclcpp::Node）
  grasp_task_manager.hpp     # GraspTaskManager 状态机
  topic_config.hpp           # TopicConfig + CameraExtrinsics
src/nodes/
  robot_controller_node.cpp
  vision_processor_node.cpp
  grasp_task_manager.cpp
```

外部依赖：rclcpp, sensor_msgs, geometry_msgs, tf2_ros, cv_bridge, message_filters, image_transport
- TopicConfig + CameraExtrinsics 从原 config.hpp 迁入
- GraspTaskManager 作为上层编排放在这里
- RosMotionBridge 是 RobotControllerNode 的内部实现细节，不单独暴露头文件

## 现有耦合问题修复

### 1. GraspTaskManager 硬编码 frame 名

```cpp
// 修复：frame 名作为构造参数传入
GraspTaskManager(std::shared_ptr<IRobotController> robot,
                 std::shared_ptr<IVisionProcessor> vision,
                 const std::string& base_frame,
                 const std::string& camera_frame);
```

### 2. RosMotionBridge 硬编码夹爪关节名

```cpp
// 修复：从 GripperProfile 中读取 finger_joint_names
```

### 3. IRobotController::lookup_transform 返回类型

```cpp
// 修复：返回通用 Eigen::Matrix4d，不依赖 ROS TF 类型
std::optional<Eigen::Matrix4d> lookup_transform(
    const std::string& parent_frame,
    const std::string& child_frame,
    double timeout_sec = 1.0);
```

## 删除的文件

- `include/robot_control_cpp/config.hpp` — 拆分为 `control_constants.hpp`（motion）和 `topic_config.hpp`（nodes）
- 旧的 `src/*.cpp` 文件 — 迁移到对应子目录

## pybind11 绑定影响

- `robot_control_cpp_py` 的 include 路径更新为新子目录结构
- CMake link 依赖从 `robot_control_core robot_control_nodes` 变为 `robot_kinematics robot_motion robot_vision robot_nodes`
- Python 侧 API 无变化，脚本无需修改

## panda_profile.hpp 归属

`panda_profile.hpp`（Panda 专用配置）继续留在 `include/robot_control_cpp/` 顶层，因为它是对 kinematics 层 `RobotProfile` 的具体实例化，不属于任何模块的通用接口。
