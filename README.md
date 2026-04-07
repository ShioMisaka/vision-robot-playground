# Robot Vision Grasp

基于 ROS2 + Isaac Sim 的机械臂视觉引导抓取系统，采用 C++ 核心库 + Python 上层脚本的多语言分层架构，支持多机器人扩展。

## 功能

- **机械臂控制**: 关节角度指令、IK 位姿控制、相对平移/旋转、TCP 工具坐标系
- **视觉处理**: 双目深度相机左目+深度同步订阅，HSV 颜色目标检测
- **视觉伺服抓取**: 图像反馈闭环 — 检测 → 居中 → 下探 → 夹取 → 提起
- **TF2 集成**: 自动发布末端执行器 TF 变换链，支持坐标系变换查询
- **多机器人扩展**: 通过 `RobotProfile` 配置驱动，核心库无需修改

## 环境要求

- **操作系统**: Ubuntu 24.04
- **ROS2**: Jazzy Jalisco
- **C++**: C++17 编译器（GCC 13+）
- **Python**: 3.10+
- **仿真**: NVIDIA Isaac Sim（提供 `/joint_states` 和相机话题）
- **相机**: ZEN_X_Mini 双目深度相机

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
  ros-jazzy-eigen ros-jazzy-image-transport ros-jazzy-urdf
```

## 项目结构

```
isaac_ros_project/
├── src/
│   ├── robot_control_cpp/          # C++ 核心库包（纯库，无可执行文件）
│   │   ├── include/robot_control_cpp/
│   │   │   ├── config.hpp                  # 话题配置 TopicConfig + 常量 ControlConstants
│   │   │   ├── robot_profile.hpp           # RobotProfile / GripperProfile / TcpConfig
│   │   │   ├── i_robot_controller.hpp      # 运动控制抽象接口 IRobotController
│   │   │   ├── i_vision_processor.hpp      # 视觉处理抽象接口 IVisionProcessor
│   │   │   ├── ik_solver.hpp               # IK/FK 求解器（KDL）
│   │   │   ├── color_detector.hpp          # CameraInterface 基类 + ColorDetector
│   │   │   ├── robot_motion_controller.hpp # 通用运动控制器 + MotionIOBridge
│   │   │   ├── grasp_task_manager.hpp      # 抓取状态机 GraspTaskManager
│   │   │   ├── robot_controller_node.hpp   # ROS2 机器人控制节点
│   │   │   ├── vision_processor_node.hpp   # ROS2 视觉处理节点
│   │   │   └── panda_profile.hpp           # Panda 专用配置
│   │   ├── src/                            # 6 个实现文件
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   ├── robot_control_test/         # C++ 测试与演示包
│   │   ├── test/
│   │   │   ├── test_ik_solver.cpp          # 独立 IK 测试（离线可运行）
│   │   │   └── test_robot_node.cpp         # 集成测试（需 Isaac Sim）
│   │   ├── demo/
│   │   │   └── demo_grasp_tcp.cpp          # TCP 抓取演示
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   ├── config.py / ik_solver.py / vision.py / robot.py    # Python 原始模块
│   ├── vision_processor.py / task_manager.py
│
├── script/                         # Python 演示脚本
│   ├── test_move.py                        # IK 位姿控制
│   ├── test_vision.py                      # 视觉伺服抓取（红色物块）
│   ├── test_joint_state.py                 # 关节状态读取
│   ├── test_camera.py                      # 相机图像显示
│   └── test_grasp_tcp.py                   # TCP 抓取
├── urdf/
│   └── panda.urdf                  # Franka Panda URDF 模型
├── CLAUDE.md
└── README.md
```

---

## 编译

### 编译 C++ 核心库

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --packages-select robot_control_cpp
```

编译产物：
- `install/robot_control_cpp/lib/librobot_control_core.so` — Layer 1 核心库
- `install/robot_control_cpp/lib/librobot_control_nodes.so` — Layer 2 ROS2 节点库
- `install/robot_control_cpp/include/robot_control_cpp/` — 公开头文件
- `install/robot_control_cpp/share/robot_control_cpp/cmake/` — CMake 导出配置

### 编译测试与演示

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash          # 必须先 source 核心库
colcon build --base-paths src --packages-select robot_control_test
```

### 一键编译全部 C++ 包

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --packages-up-to robot_control_test
```

---

## 运行

### 1. 启动 Isaac Sim

在 Isaac Sim 中加载 Franka Panda 场景，确保 ROS2 bridge 已启动。验证话题：

```bash
ros2 topic list
# 应看到:
#   /joint_states
#   /joint_command
#   /camera/image_raw/left
#   /camera/image_raw/depth
```

### 2. 运行 C++ IK 独立测试（无需 Isaac Sim）

```bash
source install/setup.bash
ros2 run robot_control_test test_ik_solver
```

预期输出：`结果: 12/12 通过`

### 3. 运行 C++ 抓取演示（需要 Isaac Sim）

```bash
source install/setup.bash
ros2 run robot_control_test demo_grasp_tcp
```

执行流程：切换 TCP → 张开夹爪 → 移动到目标上方 → 下降 → 闭合夹爪 → 提起

### 4. 运行 Python 演示

```bash
python3 script/test_move.py       # 位姿控制
python3 script/test_vision.py     # 视觉引导抓取
python3 script/test_grasp_tcp.py  # TCP 抓取
```

---

## 架构说明

### 三层架构

```
┌──────────────────────────────────────────────────────────┐
│  Layer 3: Python Binding (pybind11)                      │  ← 待实现
│  script/ 代码无缝迁移到 C++ 后端                           │
├──────────────────────────────────────────────────────────┤
│  Layer 2: ROS 2 C++ Wrapper Nodes                       │  ← rclcpp 通信适配
│  ┌──────────────────────┐ ┌───────────────────────────┐  │
│  │ RobotControllerNode  │ │ VisionProcessorNode       │  │
│  │ + RosMotionBridge    │ │ + ApproximateTime Sync    │  │
│  └──────────┬───────────┘ └──────────┬────────────────┘  │
├─────────────┼────────────────────────┼───────────────────┤
│  Layer 1: Pure C++ Core (无 ROS 依赖) │                   │
│  ┌──────────▼───────────────────────▼────────────────┐   │
│  │ IKSolver (KDL)  │ RobotMotionController           │   │
│  │ ColorDetector    │ GraspTaskManager               │   │
│  └──────────────────────────────────────────────────┘    │
└──────────────────────────────────────────────────────────┘
```

### 包间依赖关系

```
robot_control_test
    │
    ├── find_package(robot_control_cpp)
    │       │
    │       ├── robot_control_cpp::robot_control_core  (Layer 1)
    │       │     ik_solver.cpp, color_detector.cpp,
    │       │     robot_motion_controller.cpp, grasp_task_manager.cpp
    │       │
    │       └── robot_control_cpp::robot_control_nodes  (Layer 2)
    │             robot_controller_node.cpp, vision_processor_node.cpp
    │             依赖 robot_control_core
    │
    └── find_package(rclcpp)
```

### 接口隔离

| 抽象接口 | 实现类 | 用途 |
|----------|--------|------|
| `MotionIOBridge` | `RosMotionBridge` | 运动控制与通信解耦 |
| `IRobotController` | `RobotMotionController` | 上层只依赖接口，不依赖具体实现 |
| `IVisionProcessor` | `VisionProcessorNode` | 视觉结果查询抽象 |
| `CameraInterface` | `ColorDetector` | 可替换为 YOLO/GraspNet 等 |

---

## C++ API 参考

### 数据结构

#### RobotProfile — 机器人描述

```cpp
struct RobotProfile {
  std::string name;                           // 机器人名称
  std::string urdf_path;                      // URDF 文件路径
  int dof = 7;                                // 自由度
  std::vector<std::string> joint_names;       // 活动关节名
  std::vector<std::string> all_joint_names;   // 所有关节名（含夹爪）
  std::vector<double> joint_limits_lower;     // 关节下限 (rad)
  std::vector<double> joint_limits_upper;     // 关节上限 (rad)
  std::vector<double> home_joints;            // 归位姿态 (dof+2 值)
  std::vector<double> ik_default_guess;       // IK 默认初始猜测
  std::string base_frame;                     // 基座坐标系名
  std::string hand_frame;                     // 末端坐标系名
  std::map<std::string, TcpConfig> tcp_frames; // TCP 工具坐标系集合
  std::string default_tcp = "hand";           // 默认 TCP
};
```

#### TcpConfig — TCP 工具偏移

```cpp
struct TcpConfig {
  std::array<double, 3> offset_xyz = {0, 0, 0};  // 位置偏移 (m)
  std::array<double, 3> offset_rpy = {0, 0, 0};  // 姿态偏移 (rad)
};
```

#### GripperProfile — 夹爪参数

```cpp
struct GripperProfile {
  std::string type = "parallel";   // 类型
  double min_width = 0.0;          // 最小开口 (m)
  double max_width = 0.04;         // 最大开口 (m)
  int dof = 1;                     // 自由度
};
```

#### TopicConfig — ROS2 话题配置

```cpp
struct TopicConfig {
  std::string joint_command = "/joint_command";
  std::string joint_state   = "/joint_states";
  std::string camera_left   = "/camera/image_raw/left";
  std::string camera_depth  = "/camera/image_raw/depth";
  std::string camera_frame  = "camera_color_optical_frame";
};
```

#### DetectionResult — 视觉检测结果

```cpp
struct DetectionResult {
  bool detected = false;
  Eigen::Vector3d xyz = Eigen::Vector3d::Zero();   // 相机坐标系 3D 位置
  Eigen::Vector2i uv = Eigen::Vector2i::Zero();     // 图像像素坐标
  double confidence = 0.0;
};
```

### IKSolver — 运动学求解器

头文件：`ik_solver.hpp` | 依赖：KDL、Eigen3、urdf、kdl_parser（**不依赖 rclcpp**）

```cpp
// 构造：加载 URDF 并初始化 KDL 链
explicit IKSolver(const RobotProfile& profile);

// 逆运动学：目标位姿 → 关节角
// xyz: 目标位置, rpy: 目标姿态（nullopt = 仅位置约束）
// 返回: dof 个关节角，失败返回 nullopt
// 自动缓存上次结果作为下次初始猜测
std::optional<std::vector<double>> solve(
    const std::array<double, 3>& xyz,
    const std::optional<std::array<double, 3>>& rpy = std::nullopt) const;

// 正运动学：关节角 → 位姿 [x, y, z, roll, pitch, yaw]
std::array<double, 6> forward(const std::vector<double>& joint_angles) const;

// 正运动学：返回 4x4 齐次变换矩阵
Eigen::Matrix4d forward_matrix(const std::vector<double>& joint_angles) const;

// 获取自由度
int get_dof() const;
```

### RobotMotionController — 运动控制器

头文件：`robot_motion_controller.hpp` | 实现 `IRobotController` 接口

```cpp
// 构造
RobotMotionController(
    std::shared_ptr<IKSolver> ik,
    const RobotProfile& profile,
    const GripperProfile& gripper,
    std::shared_ptr<MotionIOBridge> bridge);

// === 关节空间控制 ===
void set_arm(const std::vector<double>& angles, bool block = true);
void rotate_joint(int index, double delta_angle, bool block = true);
void go_home(bool block = true);

// === 笛卡尔空间控制 ===
// 移动到目标位姿（自动处理 TCP 偏移）
void move_to_pose(
    const std::array<double, 3>& xyz,
    const std::optional<std::array<double, 3>>& rpy,
    double finger = -1.0,    // -1 = 保持当前
    int steps = 0,           // 0 = 不插值
    double step_time = 0.08,
    bool block = true);

// 相对平移（base 或 end_effector 坐标系）
void move_linear(
    const std::array<double, 3>& delta,
    const std::string& frame = "base",
    double finger = -1.0,
    bool block = true);

// === 夹爪控制 ===
void set_gripper(double width, bool block = true);
void open_gripper(bool block = true);
void close_gripper(bool block = true);  // 含稳定检测

// === 状态查询 ===
std::vector<double> get_joint_angles() const;
std::array<double, 6> get_end_effector_pose() const;
double get_finger_width() const;

// === TCP 工具坐标系 ===
void set_tcp(const std::string& name);
std::string get_current_tcp() const;

// === TF 查询 ===
std::optional<std::array<double, 6>> lookup_transform(
    const std::string& target_frame,
    const std::string& source_frame,
    double timeout = 1.0);
```

### ColorDetector — 颜色检测器

头文件：`color_detector.hpp` | 继承 `CameraInterface`

```cpp
// 构造：HSV 颜色范围
ColorDetector(const std::array<int, 3>& lower_hsv,
              const std::array<int, 3>& upper_hsv);

// 检测目标像素坐标（BGR 图像）
std::optional<Eigen::Vector2i> detect(const cv::Mat& bgr_image) const;

// 完整处理：检测 + 深度估计 3D 位置
DetectionResult process_image(const cv::Mat& rgb, const cv::Mat& depth) const;

// 设置相机内参（默认 fx=fy=614, cx=320, cy=240）
void set_camera_intrinsics(double fx, double fy, double cx, double cy);

// 可视化：在图像上绘制检测结果
void draw_target(cv::Mat& image, int cx, int cy, const std::string& label) const;
```

### GraspTaskManager — 抓取状态机

头文件：`grasp_task_manager.hpp`

```cpp
// 构造
GraspTaskManager(
    std::shared_ptr<IRobotController> robot,
    std::shared_ptr<IVisionProcessor> vision,
    double approach_height = 0.15,       // 接近高度 (m)
    double grasp_height_offset = 0.02,   // 抓取高度微调 (m)
    const std::array<double, 3>& grasp_rpy = {M_PI, 0, M_PI});  // 抓取姿态

// 执行完整抓取流程（阻塞）
bool run(double timeout = 30.0);
```

**状态转移：**

```
kIdle → kDetecting → kApproaching → kDescending → kGrasping → kLifting → kDone
                                      ↘ kError (异常/超时)
```

### RobotControllerNode — ROS2 机器人控制节点

头文件：`robot_controller_node.hpp`

```cpp
// 构造（内部创建 IKSolver + RosMotionBridge + RobotMotionController）
RobotControllerNode(
    const RobotProfile& profile,
    const GripperProfile& gripper,
    const TopicConfig& topics);

// 等待关节状态就绪（阻塞）
bool wait_for_ready(double timeout = 5.0);

// 获取底层运动控制器
std::shared_ptr<RobotMotionController> get_controller();

// 获取 TF 缓冲（用于坐标系查询）
std::shared_ptr<tf2_ros::Buffer> get_tf_buffer();
```

**TF2 广播：** 每次收到 `/joint_states` 时自动发布：
- `panda_link0` → `panda_hand`（FK 计算）
- `panda_hand` → `<tcp_name>`（TCP 偏移）

**回调组：**
- 状态订阅：`MutuallyExclusiveCallbackGroup`
- 发布/TF：`ReentrantCallbackGroup`

### VisionProcessorNode — ROS2 视觉处理节点

头文件：`vision_processor_node.hpp`

```cpp
// 构造
VisionProcessorNode(
    std::shared_ptr<CameraInterface> processor,
    const TopicConfig& topics);

// 获取最新检测结果
std::optional<DetectionResult> get_latest_result() const;

// 阻塞等待检测结果
std::optional<DetectionResult> wait_for_detection(double timeout = 10.0);
```

**图像同步：** 使用 `message_filters::ApproximateTime` 策略同步 RGB + 深度图像，时间容差 0.1s，队列大小 10。

---

## 开发指南

### 添加新机器人

1. 创建 Profile 文件 `include/robot_control_cpp/my_robot_profile.hpp`：

```cpp
#pragma once
#include "robot_control_cpp/robot_profile.hpp"
#include "robot_control_cpp/color_detector.hpp"

namespace robot_control::profiles {

inline RobotProfile my_robot() {
  RobotProfile p;
  p.name = "my_robot";
  p.urdf_path = "urdf/my_robot.urdf";
  p.dof = 6;

  p.joint_names = {"joint1", "joint2", "joint3",
                   "joint4", "joint5", "joint6"};
  p.all_joint_names = {"joint1", "joint2", "joint3",
                       "joint4", "joint5", "joint6",
                       "finger_left", "finger_right"};

  p.joint_limits_lower = {/* ... */};
  p.joint_limits_upper = {/* ... */};
  p.home_joints = {/* 6 arm + 2 gripper */};
  p.ik_default_guess = {/* 6 values */};

  p.base_frame = "base_link";
  p.hand_frame = "end_effector";

  p.tcp_frames = {
      {"hand", TcpConfig{{0, 0, 0}, {0, 0, 0}}},
  };
  p.default_tcp = "hand";

  return p;
}

inline GripperProfile my_robot_gripper() {
  return {"parallel", 0.0, 0.08, 1};
}

}  // namespace robot_control::profiles
```

2. 将对应的 URDF 文件放到 `urdf/` 目录下。

3. 在使用处引用新 Profile：

```cpp
#include "robot_control_cpp/my_robot_profile.hpp"

auto profile = robot_control::profiles::my_robot();
auto gripper = robot_control::profiles::my_robot_gripper();
auto node = robot_control::RobotControllerNode::create(
    profile, gripper, robot_control::TopicConfig{});
```

**核心库（IKSolver、RobotMotionController、GraspTaskManager）无需任何修改。**

### 替换视觉算法

`CameraInterface` 是抽象基类，实现 `process_image()` 即可替换检测算法：

```cpp
class YoloDetector : public robot_control::CameraInterface {
public:
  DetectionResult process_image(const cv::Mat& rgb,
                                const cv::Mat& depth) const override {
    // YOLO 推理 + 深度图 3D 定位
    // ...
  }
};

// 使用
auto detector = std::make_shared<YoloDetector>();
auto vision = std::make_shared<robot_control::VisionProcessorNode>(
    detector, topics);
```

### 编写新的测试/演示

在 `src/robot_control_test/` 下添加源文件，并在其 `CMakeLists.txt` 中注册：

```cmake
# test/test_my_feature.cpp
add_executable(test_my_feature test/test_my_feature.cpp)
target_link_libraries(test_my_feature robot_control_cpp::robot_control_core)
# 或需要 ROS2 节点:
# target_link_libraries(test_my_feature robot_control_cpp::robot_control_nodes)
install(TARGETS test_my_feature RUNTIME DESTINATION lib/${PROJECT_NAME})
```

重新编译：

```bash
colcon build --base-paths src --packages-select robot_control_test
```

### 编码规范

| 规范 | 说明 |
|------|------|
| 命名 | 类名 PascalCase，方法/变量 snake_case |
| 智能指针 | 禁止裸指针，使用 `shared_ptr` / `unique_ptr` |
| 注释 | 公开头文件必须有 Doxygen 注释（`@brief`, `@param`, `@return`） |
| 接口隔离 | ROS2 通信逻辑（Layer 2）不得污染核心逻辑（Layer 1） |
| 线程同步 | `std::mutex` + `std::condition_variable` |
| 异常 | 核心库抛 `std::runtime_error`，ROS 节点层捕获并日志 |

### 线程安全

| 类 | 线程安全 | 机制 |
|----|----------|------|
| `IKSolver` | 否 | 构造后无状态修改 |
| `RobotMotionController` | 是 | Bridge 内部 mutex |
| `RosMotionBridge` | 是 | `state_mutex_` 保护关节状态 |
| `RobotControllerNode` | 是 | 回调组 + mutex |
| `GraspTaskManager` | 否 | 单线程状态机 |
| `ColorDetector` | 是 | 无可变状态 |
| `VisionProcessorNode` | 是 | `result_mutex_` + condition_variable |

---

## ROS2 话题一览

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/joint_command` | sensor_msgs/JointState | 发布 | 9 值：7 臂关节 + 2 夹爪 |
| `/joint_states` | sensor_msgs/JointState | 订阅 | Isaac Sim 关节反馈 |
| `/camera/image_raw/left` | sensor_msgs/Image | 订阅 | 左目 RGB 图像 |
| `/camera/image_raw/depth` | sensor_msgs/Image | 订阅 | 深度图 |

## Python 视觉伺服参数

`script/test_vision.py` 顶部的常量控制伺服行为：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `DIRECTION_X` | -1.0 | 图像右偏时机器人 X 移动方向 |
| `DIRECTION_Y` | 1.0 | 图像下偏时机器人 Y 移动方向 |
| `MOVE_STEP` | 0.02 | 每次居中调整步长（米） |
| `CENTER_THRESHOLD` | 0.03 | 居中判定阈值（归一化偏移） |
| `DESCEND_DISTANCE` | 0.15 | 下探距离（米） |
| `LIFT_DISTANCE` | 0.25 | 提起距离（米） |
