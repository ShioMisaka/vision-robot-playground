# robot_control_cpp 解耦重构实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将 robot_control_cpp 从 2 target 重构为 4 target（kinematics, motion, vision, nodes），实现模块完全解耦。

**Architecture:** 在同一个 catkin 包内创建 4 个 CMake target，通过子目录组织头文件。叶子节点（kinematics, vision）零 ROS 依赖，motion 仅依赖 kinematics，nodes 组合 motion + vision 并承载 ROS2 通信。同时修复现有耦合问题（硬编码 frame 名、夹爪关节名）。

**Tech Stack:** C++17, CMake (ament), KDL, Eigen3, OpenCV, rclcpp, pybind11

---

### Task 1: 创建目录结构 + 拆分 kinematics 模块

**Files:**
- Create: `include/robot_control_cpp/kinematics/ik_solver.hpp`
- Create: `include/robot_control_cpp/kinematics/trajectory_planner.hpp`
- Create: `include/robot_control_cpp/kinematics/robot_profile.hpp`
- Create: `src/kinematics/ik_solver.cpp`
- Create: `src/kinematics/trajectory_planner.cpp`
- Modify: `CMakeLists.txt` — 添加 robot_kinematics target

- [ ] **Step 1: 创建子目录并迁移 kinematics 头文件**

创建 `include/robot_control_cpp/kinematics/` 目录。将以下文件复制到新位置并更新 include 路径：

**`include/robot_control_cpp/kinematics/robot_profile.hpp`** — 从原 `robot_profile.hpp` 复制，内容不变（无内部 include 需要更新）。

**`include/robot_control_cpp/kinematics/trajectory_planner.hpp`** — 从原 `trajectory_planner.hpp` 复制，更新 include：
```cpp
#pragma once

#include <vector>

#include "robot_control_cpp/kinematics/robot_profile.hpp"

namespace robot_control {
// ... 内容完全不变 ...
}  // namespace robot_control
```

**`include/robot_control_cpp/kinematics/ik_solver.hpp`** — 从原 `ik_solver.hpp` 复制，内容不变（它只前向声明 `RobotProfile`，include 路径在 cpp 中）。

- [ ] **Step 2: 迁移 kinematics 源文件**

创建 `src/kinematics/` 目录。复制并更新 include 路径：

**`src/kinematics/ik_solver.cpp`** — 从原 `src/ik_solver.cpp` 复制，更新第一行 include：
```cpp
#include "robot_control_cpp/kinematics/ik_solver.hpp"
```
其余内容不变。

**`src/kinematics/trajectory_planner.cpp`** — 从原 `src/trajectory_planner.cpp` 复制，更新第一行 include：
```cpp
#include "robot_control_cpp/kinematics/trajectory_planner.hpp"
```
其余内容不变。

- [ ] **Step 3: 在 CMakeLists.txt 中添加 robot_kinematics target**

在现有 CMakeLists.txt 的 `# ===== Layer 1: 核心库 =====` 注释之前，添加：

```cmake
# ===== robot_kinematics: IK + 轨迹规划（零 ROS 依赖）=====
add_library(robot_kinematics SHARED
  src/kinematics/ik_solver.cpp
  src/kinematics/trajectory_planner.cpp
)

target_include_directories(robot_kinematics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(robot_kinematics
  orocos_kdl
  urdfdom
  urdf
  kdl_parser
  Eigen3
)
```

- [ ] **Step 4: 编译验证 kinematics target**

Run: `colcon build --base-paths src --packages-select robot_control_cpp`
Expected: 编译成功，`robot_kinematics` target 生成

- [ ] **Step 5: Commit**

```bash
git add src/robot_control_cpp/include/robot_control_cpp/kinematics/ src/robot_control_cpp/src/kinematics/ src/robot_control_cpp/CMakeLists.txt
git commit -m "refactor: extract robot_kinematics target (IK + trajectory planner)"
```

---

### Task 2: 拆分 vision 模块

**Files:**
- Create: `include/robot_control_cpp/vision/i_vision_processor.hpp`
- Create: `include/robot_control_cpp/vision/camera_interface.hpp`
- Create: `include/robot_control_cpp/vision/color_detector.hpp`
- Create: `src/vision/color_detector.cpp`
- Modify: `CMakeLists.txt` — 添加 robot_vision target

- [ ] **Step 1: 创建子目录并迁移 vision 头文件**

创建 `include/robot_control_cpp/vision/` 目录。

**`include/robot_control_cpp/vision/i_vision_processor.hpp`** — 从原 `i_vision_processor.hpp` 复制，内容不变（无内部 include 需要更新）。

**`include/robot_control_cpp/vision/camera_interface.hpp`** — 从原 `camera_interface.hpp` 复制，更新 include：
```cpp
#include "robot_control_cpp/vision/i_vision_processor.hpp"
```

**`include/robot_control_cpp/vision/color_detector.hpp`** — 从原 `color_detector.hpp` 复制，更新 include：
```cpp
#include "robot_control_cpp/vision/camera_interface.hpp"
```

- [ ] **Step 2: 迁移 vision 源文件**

创建 `src/vision/` 目录。

**`src/vision/color_detector.cpp`** — 从原 `src/color_detector.cpp` 复制，更新 include：
```cpp
#include "robot_control_cpp/vision/color_detector.hpp"
```

- [ ] **Step 3: 在 CMakeLists.txt 中添加 robot_vision target**

在 `robot_kinematics` target 之后添加：

```cmake
# ===== robot_vision: 视觉处理（零 ROS 依赖）=====
add_library(robot_vision SHARED
  src/vision/color_detector.cpp
)

target_include_directories(robot_vision PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(robot_vision
  OpenCV
  Eigen3
)
```

- [ ] **Step 4: 编译验证 vision target**

Run: `colcon build --base-paths src --packages-select robot_control_cpp`
Expected: 编译成功

- [ ] **Step 5: Commit**

```bash
git add src/robot_control_cpp/include/robot_control_cpp/vision/ src/robot_control_cpp/src/vision/ src/robot_control_cpp/CMakeLists.txt
git commit -m "refactor: extract robot_vision target (color detector + vision interfaces)"
```

---

### Task 3: 拆分 motion 模块 + 拆分 config.hpp

**Files:**
- Create: `include/robot_control_cpp/motion/control_constants.hpp`
- Create: `include/robot_control_cpp/motion/i_robot_controller.hpp`
- Create: `include/robot_control_cpp/motion/motion_io_bridge.hpp`
- Create: `include/robot_control_cpp/motion/robot_motion_controller.hpp`
- Create: `src/motion/robot_motion_controller.cpp`
- Modify: `CMakeLists.txt` — 添加 robot_motion target

- [ ] **Step 1: 创建 control_constants.hpp**

从 `config.hpp` 中提取 `ControlConstants`，创建 `include/robot_control_cpp/motion/control_constants.hpp`：
```cpp
#pragma once

namespace robot_control {

/// 通用控制常量
struct ControlConstants {
  /// 关节运动到位判定阈值（弧度）
  static constexpr double kJointTolerance = 0.05;
  /// 夹爪运动到位判定阈值（米）
  static constexpr double kFingerTolerance = 0.002;
  /// 运动等待超时（秒）
  static constexpr double kMotionTimeout = 10.0;
  /// 运动轮询间隔（秒）
  static constexpr double kPollInterval = 0.02;
  /// 到位后稳定等待时间（秒）
  static constexpr double kSettleTime = 0.2;
  /// 默认插值步数
  static constexpr int kDefaultSteps = 10;
  /// 默认插值步间隔（秒）
  static constexpr double kDefaultStepTime = 0.08;
  /// 图像同步队列大小
  static constexpr int kImageSyncQueueSize = 10;
  /// 图像同步时间容差（秒）
  static constexpr double kImageSyncSlop = 0.1;
  /// 夹爪稳定检测次数
  static constexpr int kFingerStableCount = 5;
  /// 夹爪稳定容差（米）
  static constexpr double kFingerStableTol = 0.001;
  /// 就绪等待超时（秒）
  static constexpr double kReadyTimeout = 5.0;
  /// 轨迹规划时间步长（50 Hz）
  static constexpr double kTrajectoryDt = 0.02;
};

}  // namespace robot_control
```

注意：`kImageSyncQueueSize` 和 `kImageSyncSlop` 目前只有 `VisionProcessorNode` 使用。但由于它们是"控制/处理"相关常量而非 ROS 概念，放在 motion 中是合理的。后续如果觉得不合适可以移到 vision。

- [ ] **Step 2: 迁移 motion 头文件**

创建 `include/robot_control_cpp/motion/` 目录。

**`include/robot_control_cpp/motion/i_robot_controller.hpp`** — 从原 `i_robot_controller.hpp` 复制，内容不变。

**`include/robot_control_cpp/motion/motion_io_bridge.hpp`** — 从原 `motion_io_bridge.hpp` 复制，内容不变。

**`include/robot_control_cpp/motion/robot_motion_controller.hpp`** — 从原 `robot_motion_controller.hpp` 复制，更新 includes：
```cpp
#include "robot_control_cpp/motion/i_robot_controller.hpp"
#include "robot_control_cpp/motion/motion_io_bridge.hpp"
#include "robot_control_cpp/kinematics/robot_profile.hpp"
#include "robot_control_cpp/kinematics/trajectory_planner.hpp"
```

- [ ] **Step 3: 迁移 motion 源文件**

创建 `src/motion/` 目录。

**`src/motion/robot_motion_controller.cpp`** — 从原 `src/robot_motion_controller.cpp` 复制，更新 includes：
```cpp
#include "robot_control_cpp/motion/robot_motion_controller.hpp"
#include "robot_control_cpp/kinematics/ik_solver.hpp"
#include "robot_control_cpp/motion/control_constants.hpp"
```

其余内容不变（所有 `ControlConstants::` 引用保持不变）。

- [ ] **Step 4: 在 CMakeLists.txt 中添加 robot_motion target**

在 `robot_vision` target 之后添加：

```cmake
# ===== robot_motion: 运动控制器（依赖 kinematics）=====
add_library(robot_motion SHARED
  src/motion/robot_motion_controller.cpp
)

target_include_directories(robot_motion PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(robot_motion robot_kinematics)
```

- [ ] **Step 5: 编译验证 motion target**

Run: `colcon build --base-paths src --packages-select robot_control_cpp`
Expected: 编译成功

- [ ] **Step 6: Commit**

```bash
git add src/robot_control_cpp/include/robot_control_cpp/motion/ src/robot_control_cpp/src/motion/ src/robot_control_cpp/CMakeLists.txt
git commit -m "refactor: extract robot_motion target (controller + interfaces) and split ControlConstants"
```

---

### Task 4: 拆分 nodes 模块 + 修复耦合问题

**Files:**
- Create: `include/robot_control_cpp/nodes/topic_config.hpp`
- Create: `include/robot_control_cpp/nodes/grasp_task_manager.hpp`
- Create: `include/robot_control_cpp/nodes/robot_controller_node.hpp`
- Create: `include/robot_control_cpp/nodes/vision_processor_node.hpp`
- Create: `src/nodes/grasp_task_manager.cpp`
- Create: `src/nodes/robot_controller_node.cpp`
- Create: `src/nodes/vision_processor_node.cpp`
- Modify: `CMakeLists.txt` — 添加 robot_nodes target，移除旧的 core/nodes target

- [ ] **Step 1: 创建 topic_config.hpp**

从 `config.hpp` 中提取 `TopicConfig` + `CameraExtrinsics`，创建 `include/robot_control_cpp/nodes/topic_config.hpp`：
```cpp
#pragma once

#include <array>
#include <string>

namespace robot_control {

/// 相机外参（相对于 hand_frame 的固定偏移）
struct CameraExtrinsics {
  /// camera_link 相对于 hand 的位置偏移（米）
  std::array<double, 3> xyz = {0.015, 0.0, 0.03};
  /// camera_link 相对于 hand 的旋转（ZYX 惯性欧拉角，弧度）
  std::array<double, 3> rpy = {0.0, 1.57079632679, 3.14159265359};
};

/// ROS 2 话题配置，支持按机器人实例自定义
struct TopicConfig {
  std::string joint_command = "/joint_command";
  std::string joint_state = "/joint_states";
  std::string camera_left = "/camera/image_raw/left";
  std::string camera_depth = "/camera/image_raw/depth";
  std::string camera_frame = "camera_color_optical_frame";
  CameraExtrinsics camera_extrinsics;
};

}  // namespace robot_control
```

- [ ] **Step 2: 迁移并修复 grasp_task_manager**

创建 `include/robot_control_cpp/nodes/` 和 `src/nodes/` 目录。

**`include/robot_control_cpp/nodes/grasp_task_manager.hpp`** — 从原 `grasp_task_manager.hpp` 复制，更新 includes 并修复硬编码 frame 名：

```cpp
#pragma once

#include <array>
#include <chrono>
#include <memory>
#include <optional>
#include <string>

#include "robot_control_cpp/motion/i_robot_controller.hpp"
#include "robot_control_cpp/vision/i_vision_processor.hpp"

namespace robot_control {

/// 抓取任务状态
enum class GraspState {
  kIdle,
  kDetecting,
  kApproaching,
  kDescending,
  kGrasping,
  kLifting,
  kDone,
  kError
};

/// 视觉引导抓取任务管理器
/// 在主线程中运行状态机，协调视觉与运动控制
class GraspTaskManager {
public:
  /// @brief 构造抓取任务管理器
  /// @param robot 运动控制接口
  /// @param vision 视觉处理接口
  /// @param base_frame 基座坐标系名称（如 "panda_link0"）
  /// @param camera_frame 相机坐标系名称（如 "camera_color_optical_frame"）
  /// @param approach_height 接近时目标上方偏移高度（米）
  /// @param grasp_height_offset 抓取时高度偏移（米）
  /// @param grasp_rpy 抓取姿态 [roll, pitch, yaw]（弧度）
  GraspTaskManager(std::shared_ptr<IRobotController> robot,
                   std::shared_ptr<IVisionProcessor> vision,
                   const std::string& base_frame = "panda_link0",
                   const std::string& camera_frame = "camera_color_optical_frame",
                   double approach_height = 0.15,
                   double grasp_height_offset = 0.02,
                   const std::array<double, 3>& grasp_rpy = {
                       3.14159265, 0.0, 3.14159265});

  /// @brief 运行完整抓取流程（阻塞）
  /// @param timeout 整体超时（秒）
  /// @return true=成功
  bool run(double timeout = 30.0);

  /// @brief 获取当前状态
  GraspState get_state() const { return state_; }

  /// @brief 将相机坐标系 3D 点转换到基座坐标系
  /// @param camera_xyz 相机光学坐标系下的 3D 点
  /// @return 基座坐标系下的 3D 点，失败返回 nullopt
  std::optional<std::array<double, 3>> transform_to_base(
      const Eigen::Vector3d& camera_xyz);

private:
  bool step_detect();
  void step_approach();
  void step_descend();
  void step_grasp();
  void step_lift();

  std::shared_ptr<IRobotController> robot_;
  std::shared_ptr<IVisionProcessor> vision_;

  std::string base_frame_;
  std::string camera_frame_;

  double approach_height_;
  double grasp_height_offset_;
  std::array<double, 3> grasp_rpy_;

  GraspState state_ = GraspState::kIdle;
  std::optional<std::array<double, 3>> target_xyz_;
};

}  // namespace robot_control
```

**`src/nodes/grasp_task_manager.cpp`** — 从原 `src/grasp_task_manager.cpp` 复制，更新 includes 和修复硬编码：

```cpp
#include "robot_control_cpp/nodes/grasp_task_manager.hpp"

#include <Eigen/Geometry>

#include <chrono>
#include <cmath>
#include <thread>
#include <vector>

namespace robot_control {

GraspTaskManager::GraspTaskManager(
    std::shared_ptr<IRobotController> robot,
    std::shared_ptr<IVisionProcessor> vision,
    const std::string& base_frame,
    const std::string& camera_frame,
    double approach_height, double grasp_height_offset,
    const std::array<double, 3>& grasp_rpy)
    : robot_(std::move(robot)),
      vision_(std::move(vision)),
      base_frame_(base_frame),
      camera_frame_(camera_frame),
      approach_height_(approach_height),
      grasp_height_offset_(grasp_height_offset),
      grasp_rpy_(grasp_rpy) {}

// run(), step_detect(), step_approach(), step_descend(), step_grasp(), step_lift()
// 保持不变（这些方法不引用 frame 名）

std::optional<std::array<double, 3>> GraspTaskManager::transform_to_base(
    const Eigen::Vector3d& camera_xyz) {
  auto tf = robot_->lookup_transform(
      base_frame_, camera_frame_, 1.0);  // 修复：使用成员变量而非硬编码
  if (!tf.has_value()) {
    return std::nullopt;
  }

  Eigen::AngleAxisd roll((*tf)[3], Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitch((*tf)[4], Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yaw((*tf)[5], Eigen::Vector3d::UnitZ());
  Eigen::Matrix3d R = (yaw * pitch * roll).toRotationMatrix();

  Eigen::Vector3d t((*tf)[0], (*tf)[1], (*tf)[2]);
  Eigen::Vector3d base_point = R * camera_xyz + t;

  return std::array<double, 3>{base_point.x(), base_point.y(),
                                base_point.z()};
}

}  // namespace robot_control
```

- [ ] **Step 3: 迁移并修复 robot_controller_node**

**`include/robot_control_cpp/nodes/robot_controller_node.hpp`** — 从原 `robot_controller_node.hpp` 复制，更新 includes：
```cpp
#include "robot_control_cpp/motion/robot_motion_controller.hpp"
#include "robot_control_cpp/kinematics/robot_profile.hpp"
#include "robot_control_cpp/nodes/topic_config.hpp"
```
其余内容不变。

**`src/nodes/robot_controller_node.cpp`** — 从原 `src/robot_controller_node.cpp` 复制，更新 includes 并修复硬编码夹爪关节名：

```cpp
#include "robot_control_cpp/nodes/robot_controller_node.hpp"
#include "robot_control_cpp/kinematics/ik_solver.hpp"
#include "robot_control_cpp/nodes/topic_config.hpp"
```

在 `RosMotionBridge::publish_command` 中，修复硬编码夹爪关节名。需要从 `GripperProfile` 获取夹爪关节名。为此在 `RosMotionBridge` 构造函数中增加 `GripperProfile` 参数：

```cpp
// RosMotionBridge 构造函数签名更新：
RosMotionBridge(rclcpp::Node::SharedPtr node,
                const TopicConfig& topics,
                std::shared_ptr<IKSolver> ik,
                const RobotProfile& profile,
                const GripperProfile& gripper);  // 新增参数
```

```cpp
// RosMotionBridge 新增成员：
GripperProfile gripper_;
```

```cpp
// publish_command 修复：
void RosMotionBridge::publish_command(const std::vector<double>& arm,
                                      double finger) {
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = node_->now();

  for (size_t i = 0; i < arm.size(); ++i) {
    msg.name.push_back(profile_.joint_names[i]);
    msg.position.push_back(arm[i]);
  }
  // 夹爪：使用 all_joint_names 中 dof 之后的关节
  for (size_t i = profile_.dof; i < profile_.all_joint_names.size(); ++i) {
    msg.name.push_back(profile_.all_joint_names[i]);
    msg.position.push_back(finger);
  }

  cmd_pub_->publish(msg);
}
```

```cpp
// update_joint_state 修复夹爪读取：
auto it = name_to_pos.find(
    profile_.all_joint_names.empty() ? "panda_finger_joint1"
                                     : profile_.all_joint_names[profile_.dof]);
if (it != name_to_pos.end()) {
  current_finger_ = it->second;
}
```

在 `RobotControllerNode` 构造和 `init()` 中传递 `gripper_` 给 `RosMotionBridge`：
```cpp
bridge_ = std::make_shared<RosMotionBridge>(
    shared_from_this(), topics_, ik_, profile_, gripper_);
```

- [ ] **Step 4: 迁移 vision_processor_node**

**`include/robot_control_cpp/nodes/vision_processor_node.hpp`** — 从原 `vision_processor_node.hpp` 复制，更新 includes：
```cpp
#include "robot_control_cpp/vision/camera_interface.hpp"
#include "robot_control_cpp/vision/i_vision_processor.hpp"
#include "robot_control_cpp/nodes/topic_config.hpp"
```
其余内容不变。

**`src/nodes/vision_processor_node.cpp`** — 从原 `src/vision_processor_node.cpp` 复制，更新 includes：
```cpp
#include "robot_control_cpp/nodes/vision_processor_node.hpp"
#include "robot_control_cpp/nodes/topic_config.hpp"
#include "robot_control_cpp/motion/control_constants.hpp"
```

注意：`VisionProcessorNode::init()` 中使用了 `ControlConstants::kImageSyncQueueSize` 和 `ControlConstants::kImageSyncSlop`，这些现在从 `motion/control_constants.hpp` 引入。

- [ ] **Step 5: 更新 CMakeLists.txt — 添加 robot_nodes target**

在 `robot_motion` target 之后添加，同时替换旧的 `robot_control_core` 和 `robot_control_nodes`：

```cmake
# ===== robot_nodes: ROS2 节点（依赖 motion + vision）=====
add_library(robot_nodes SHARED
  src/nodes/robot_controller_node.cpp
  src/nodes/vision_processor_node.cpp
  src/nodes/grasp_task_manager.cpp
)

target_include_directories(robot_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(robot_nodes robot_motion robot_vision)

ament_target_dependencies(robot_nodes
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2
  tf2_geometry_msgs
  cv_bridge
  message_filters
  image_transport
)
```

删除旧的 `robot_control_core` 和 `robot_control_nodes` target 定义。

- [ ] **Step 6: 更新安装和导出**

更新 `install` 和 `ament_export_targets`：

```cmake
# ===== 安装 =====
install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS robot_kinematics robot_motion robot_vision robot_nodes
  EXPORT robot_control_cpp_export
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ===== 导出（供下游包 find_package 使用）=====
ament_export_targets(robot_control_cpp_export HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2
  tf2_geometry_msgs
  cv_bridge
  message_filters
  image_transport
  orocos_kdl
  urdfdom
  urdf
  kdl_parser
  Eigen3
  OpenCV
)
```

- [ ] **Step 7: 编译验证 nodes target**

Run: `colcon build --base-paths src --packages-select robot_control_cpp`
Expected: 编译成功

- [ ] **Step 8: Commit**

```bash
git add src/robot_control_cpp/include/robot_control_cpp/nodes/ src/robot_control_cpp/src/nodes/ src/robot_control_cpp/CMakeLists.txt
git commit -m "refactor: extract robot_nodes target + fix hardcoded frame/gripper names"
```

---

### Task 5: 更新 panda_profile.hpp include 路径

**Files:**
- Modify: `include/robot_control_cpp/panda_profile.hpp`

- [ ] **Step 1: 更新 include 路径**

```cpp
#pragma once

#include <map>
#include <string>
#include <vector>
#include "robot_control_cpp/kinematics/robot_profile.hpp"
```

其余内容不变。

- [ ] **Step 2: 编译验证**

Run: `colcon build --base-paths src --packages-select robot_control_cpp`
Expected: 编译成功

- [ ] **Step 3: Commit**

```bash
git add src/robot_control_cpp/include/robot_control_cpp/panda_profile.hpp
git commit -m "refactor: update panda_profile.hpp include path"
```

---

### Task 6: 删除旧文件 + 保留兼容性头文件

**Files:**
- Delete: `include/robot_control_cpp/config.hpp`
- Delete: `include/robot_control_cpp/ik_solver.hpp`
- Delete: `include/robot_control_cpp/trajectory_planner.hpp`
- Delete: `include/robot_control_cpp/robot_profile.hpp`
- Delete: `include/robot_control_cpp/i_robot_controller.hpp`
- Delete: `include/robot_control_cpp/motion_io_bridge.hpp`
- Delete: `include/robot_control_cpp/robot_motion_controller.hpp`
- Delete: `include/robot_control_cpp/camera_interface.hpp`
- Delete: `include/robot_control_cpp/i_vision_processor.hpp`
- Delete: `include/robot_control_cpp/color_detector.hpp`
- Delete: `include/robot_control_cpp/grasp_task_manager.hpp`
- Delete: `include/robot_control_cpp/robot_controller_node.hpp`
- Delete: `include/robot_control_cpp/vision_processor_node.hpp`
- Delete: `src/ik_solver.cpp`
- Delete: `src/trajectory_planner.cpp`
- Delete: `src/color_detector.cpp`
- Delete: `src/robot_motion_controller.cpp`
- Delete: `src/grasp_task_manager.cpp`
- Delete: `src/robot_controller_node.cpp`
- Delete: `src/vision_processor_node.cpp`

- [ ] **Step 1: 删除所有旧头文件和源文件**

删除 `include/robot_control_cpp/` 下的旧 `.hpp` 文件（保留 `panda_profile.hpp` 和子目录）。
删除 `src/` 下的旧 `.cpp` 文件（保留子目录）。

- [ ] **Step 2: 编译验证**

Run: `colcon build --base-paths src --packages-select robot_control_cpp`
Expected: 编译成功

- [ ] **Step 3: Commit**

```bash
git add -A src/robot_control_cpp/
git commit -m "refactor: remove old flat header/source files after target migration"
```

---

### Task 7: 更新 pybind11 绑定

**Files:**
- Modify: `src/robot_control_cpp_py/src/bindings.cpp`
- Modify: `src/robot_control_cpp_py/CMakeLists.txt`

- [ ] **Step 1: 更新 bindings.cpp 的 include 路径**

将所有 include 从旧路径更新为新路径：

```cpp
// 旧：
#include "robot_control_cpp/config.hpp"
#include "robot_control_cpp/robot_profile.hpp"
#include "robot_control_cpp/ik_solver.hpp"
#include "robot_control_cpp/color_detector.hpp"
#include "robot_control_cpp/i_robot_controller.hpp"
#include "robot_control_cpp/i_vision_processor.hpp"
#include "robot_control_cpp/robot_motion_controller.hpp"
#include "robot_control_cpp/grasp_task_manager.hpp"
#include "robot_control_cpp/robot_controller_node.hpp"
#include "robot_control_cpp/vision_processor_node.hpp"
#include "robot_control_cpp/panda_profile.hpp"

// 新：
#include "robot_control_cpp/kinematics/robot_profile.hpp"
#include "robot_control_cpp/kinematics/ik_solver.hpp"
#include "robot_control_cpp/vision/i_vision_processor.hpp"
#include "robot_control_cpp/vision/camera_interface.hpp"
#include "robot_control_cpp/vision/color_detector.hpp"
#include "robot_control_cpp/motion/i_robot_controller.hpp"
#include "robot_control_cpp/motion/control_constants.hpp"
#include "robot_control_cpp/motion/robot_motion_controller.hpp"
#include "robot_control_cpp/nodes/grasp_task_manager.hpp"
#include "robot_control_cpp/nodes/robot_controller_node.hpp"
#include "robot_control_cpp/nodes/vision_processor_node.hpp"
#include "robot_control_cpp/nodes/topic_config.hpp"
#include "robot_control_cpp/panda_profile.hpp"
```

绑定代码中所有类名、方法名保持不变，Python 侧 API 无变化。

- [ ] **Step 2: 更新 pybind11 CMakeLists.txt**

```cmake
target_link_libraries(_core PRIVATE
  robot_control_cpp::robot_kinematics
  robot_control_cpp::robot_motion
  robot_control_cpp::robot_vision
  robot_control_cpp::robot_nodes
)
```

- [ ] **Step 3: 编译验证 pybind11 包**

Run:
```bash
source install/setup.zsh
colcon build --base-paths src --packages-up-to robot_control_cpp_py
```
Expected: 编译成功

- [ ] **Step 4: Commit**

```bash
git add src/robot_control_cpp_py/src/bindings.cpp src/robot_control_cpp_py/CMakeLists.txt
git commit -m "refactor: update pybind11 bindings for new target structure"
```

---

### Task 8: 更新 robot_control_test 包

**Files:**
- Modify: `src/robot_control_test/CMakeLists.txt`
- Modify: `src/robot_control_test/test/test_trajectory_planner.cpp` (如有旧 include)
- Modify: `src/robot_control_test/test/test_motion_controller.cpp` (如有旧 include)
- Modify: `src/robot_control_test/test/test_ik_solver.cpp` (如有旧 include)
- Modify: `src/robot_control_test/test/test_robot_node.cpp` (如有旧 include)
- Modify: `src/robot_control_test/test/test_camera_tf.cpp` (如有旧 include)
- Modify: `src/robot_control_test/demo/demo_grasp_tcp.cpp` (如有旧 include)
- Modify: `src/robot_control_test/demo/demo_camera.cpp` (如有旧 include)

- [ ] **Step 1: 更新 test CMakeLists.txt 的 link 依赖**

将 `robot_control_cpp::robot_control_core` 替换为 `robot_control_cpp::robot_kinematics` + `robot_control_cpp::robot_motion`，将 `robot_control_cpp::robot_control_nodes` 替换为 `robot_control_cpp::robot_nodes`：

```cmake
# 独立测试（无 ROS 依赖）
target_link_libraries(test_trajectory_planner robot_control_cpp::robot_kinematics)
target_link_libraries(test_motion_controller robot_control_cpp::robot_motion)
target_link_libraries(test_ik_solver robot_control_cpp::robot_kinematics)

# 集成测试（需要 ROS2）
target_link_libraries(test_robot_node robot_control_cpp::robot_nodes)
target_link_libraries(test_camera_tf robot_control_cpp::robot_nodes)

# 演示
target_link_libraries(demo_grasp_tcp robot_control_cpp::robot_nodes)
target_link_libraries(demo_camera robot_control_cpp::robot_nodes)
```

- [ ] **Step 2: 更新所有测试/演示文件的 include 路径**

对 `robot_control_test/test/` 和 `robot_control_test/demo/` 下所有 `.cpp` 文件执行 include 路径替换。根据各文件的 include 需要替换为对应的新路径。具体替换规则：

| 旧 include | 新 include |
|---|---|
| `robot_control_cpp/ik_solver.hpp` | `robot_control_cpp/kinematics/ik_solver.hpp` |
| `robot_control_cpp/trajectory_planner.hpp` | `robot_control_cpp/kinematics/trajectory_planner.hpp` |
| `robot_control_cpp/robot_profile.hpp` | `robot_control_cpp/kinematics/robot_profile.hpp` |
| `robot_control_cpp/i_robot_controller.hpp` | `robot_control_cpp/motion/i_robot_controller.hpp` |
| `robot_control_cpp/motion_io_bridge.hpp` | `robot_control_cpp/motion/motion_io_bridge.hpp` |
| `robot_control_cpp/robot_motion_controller.hpp` | `robot_control_cpp/motion/robot_motion_controller.hpp` |
| `robot_control_cpp/config.hpp` | `robot_control_cpp/motion/control_constants.hpp` 或 `robot_control_cpp/nodes/topic_config.hpp`（视内容而定） |
| `robot_control_cpp/color_detector.hpp` | `robot_control_cpp/vision/color_detector.hpp` |
| `robot_control_cpp/camera_interface.hpp` | `robot_control_cpp/vision/camera_interface.hpp` |
| `robot_control_cpp/i_vision_processor.hpp` | `robot_control_cpp/vision/i_vision_processor.hpp` |
| `robot_control_cpp/grasp_task_manager.hpp` | `robot_control_cpp/nodes/grasp_task_manager.hpp` |
| `robot_control_cpp/robot_controller_node.hpp` | `robot_control_cpp/nodes/robot_controller_node.hpp` |
| `robot_control_cpp/vision_processor_node.hpp` | `robot_control_cpp/nodes/vision_processor_node.hpp` |
| `robot_control_cpp/panda_profile.hpp` | `robot_control_cpp/panda_profile.hpp`（不变） |

注意：如果某些文件同时引用了 `TopicConfig` 和 `ControlConstants`，需要同时 include 两个新头文件。

- [ ] **Step 3: 编译验证 test 包**

Run:
```bash
source install/setup.zsh
colcon build --base-paths src --packages-up-to robot_control_test
```
Expected: 编译成功

- [ ] **Step 4: Commit**

```bash
git add src/robot_control_test/
git commit -m "refactor: update test/demo includes and CMake for new target structure"
```

---

### Task 9: 全量编译验证 + 运行离线测试

- [ ] **Step 1: 全量编译**

Run: `colcon build --base-paths src`
Expected: 所有包编译成功，无警告

- [ ] **Step 2: 运行离线测试**

Run:
```bash
source install/setup.zsh
ros2 run robot_control_test test_ik_solver
ros2 run robot_control_test test_trajectory_planner
ros2 run robot_control_test test_motion_controller
```
Expected: 全部 PASS

- [ ] **Step 3: 最终 Commit**

如有修复则提交，否则此步骤跳过。

---

### Task 10: 更新 CLAUDE.md 项目文档

**Files:**
- Modify: `CLAUDE.md`

- [ ] **Step 1: 更新分层架构描述**

将 CLAUDE.md 中的分层架构从 2 target 更新为 4 target，更新项目结构树和常用命令中的 target 名称。

- [ ] **Step 2: Commit**

```bash
git add CLAUDE.md
git commit -m "docs: update CLAUDE.md for 4-target architecture"
```
