# API 接口文档

本文档涵盖 C++ 核心库（Layer 1 & 2）和 Python 绑定（Layer 3）的完整接口参考。

---

## 目录

- [数据结构](#数据结构)
- [控制常量](#控制常量)
- [IKSolver — 运动学求解器](#iksolver--运动学求解器)
- [RobotMotionController — 运动控制器](#robotmotioncontroller--运动控制器)
- [ColorDetector — 颜色检测器](#colordetector--颜色检测器)
- [GraspTaskManager — 抓取状态机](#grasptaskmanager--抓取状态机)
- [RobotControllerNode — ROS2 控制节点](#robotcontrollernode--ros2-控制节点)
- [VisionProcessorNode — ROS2 视觉节点](#visionprocessornode--ros2-视觉节点)
- [Profile 工厂函数](#profile-工厂函数)
- [Python 绑定完整参考](#python-绑定完整参考)
- [C++ / Python 类型映射](#c--python-类型映射)
- [线程安全](#线程安全)
- [开发指南](#开发指南)

---

## 数据结构

### TcpConfig — TCP 工具偏移

**头文件**: `robot_profile.hpp`

```cpp
struct TcpConfig {
  std::array<double, 3> offset_xyz = {0, 0, 0};  // 位置偏移 (m)
  std::array<double, 3> offset_rpy = {0, 0, 0};  // 姿态偏移 (rad)
};
```

**Python**: `rc.TcpConfig` — 可读写属性 `offset_xyz`, `offset_rpy`（`list[float]`）

---

### RobotProfile — 机器人描述

**头文件**: `robot_profile.hpp`

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

**Python**: `rc.RobotProfile` — 所有字段均可读写

---

### GripperProfile — 夹爪参数

**头文件**: `robot_profile.hpp`

```cpp
struct GripperProfile {
  std::string type = "parallel";   // 类型
  double min_width = 0.0;          // 最小开口 (m)
  double max_width = 0.04;         // 最大开口 (m)
  int dof = 1;                     // 自由度
};
```

**Python**: `rc.GripperProfile` — 可读写 `type`, `min_width`, `max_width`, `dof`

---

### TopicConfig — ROS2 话题配置

**头文件**: `config.hpp`

```cpp
struct TopicConfig {
  std::string joint_command = "/joint_command";
  std::string joint_state   = "/joint_states";
  std::string camera_left   = "/camera/image_raw/left";
  std::string camera_depth  = "/camera/image_raw/depth";
  std::string camera_frame  = "camera_color_optical_frame";
};
```

**Python**: `rc.TopicConfig` — 可读写所有字段

---

### DetectionResult — 视觉检测结果

**头文件**: `i_vision_processor.hpp`

```cpp
struct DetectionResult {
  bool detected = false;
  Eigen::Vector3d xyz = Eigen::Vector3d::Zero();   // 相机坐标系 3D 位置
  Eigen::Vector2i uv = Eigen::Vector2i::Zero();     // 图像像素坐标
  double confidence = 0.0;
};
```

**Python**: `rc.DetectionResult`

| 属性 | Python 类型 | 读写 | 说明 |
|------|-------------|------|------|
| `detected` | `bool` | 读写 | 是否检测到目标 |
| `xyz` | `list[float]` | 只读 | 相机坐标系 3D 位置 `[x, y, z]` |
| `uv` | `list[int]` | 只读 | 像素坐标 `[u, v]` |
| `confidence` | `float` | 读写 | 置信度 |

---

### GraspState — 抓取任务状态

**头文件**: `grasp_task_manager.hpp`

```cpp
enum class GraspState {
  kIdle, kDetecting, kApproaching, kDescending,
  kGrasping, kLifting, kDone, kError
};
```

**Python**: `rc.GraspState` 枚举

```python
rc.GraspState.IDLE          # 空闲
rc.GraspState.DETECTING     # 检测中
rc.GraspState.APPROACHING   # 接近中
rc.GraspState.DESCENDING    # 下降中
rc.GraspState.GRASPING      # 抓取中
rc.GraspState.LIFTING       # 提升中
rc.GraspState.DONE          # 完成
rc.GraspState.ERROR         # 错误
```

---

## 控制常量

**头文件**: `config.hpp` · **Python**: `rc` 模块级常量

| 常量 | 值 | 单位 | 说明 |
|------|----|------|------|
| `JOINT_TOLERANCE` | 0.05 | rad | 关节到位判定阈值 |
| `FINGER_TOLERANCE` | 0.002 | m | 夹爪到位判定阈值 |
| `MOTION_TIMEOUT` | 10.0 | s | 运动等待超时 |
| `POLL_INTERVAL` | 0.02 | s | 运动轮询间隔 |
| `SETTLE_TIME` | 0.2 | s | 到位后稳定等待 |
| `DEFAULT_STEPS` | 10 | — | 默认插值步数 |
| `DEFAULT_STEP_TIME` | 0.08 | s | 默认插值步间隔 |
| `IMAGE_SYNC_QUEUE_SIZE` | 10 | — | 图像同步队列大小 |
| `IMAGE_SYNC_SLOP` | 0.1 | s | 图像同步时间容差 |
| `FINGER_STABLE_COUNT` | 5 | — | 夹爪稳定检测次数 |
| `FINGER_STABLE_TOL` | 0.001 | m | 夹爪稳定容差 |
| `READY_TIMEOUT` | 5.0 | s | 就绪等待超时 |

---

## IKSolver — 运动学求解器

**头文件**: `ik_solver.hpp` · **依赖**: KDL, Eigen3, urdf（**不依赖 rclcpp**）

### 构造

```cpp
explicit IKSolver(const RobotProfile& profile);
```

```python
ik = rc.IKSolver(rc.profiles.panda())
```

### 方法

#### `solve` — 逆运动学

```cpp
std::optional<std::vector<double>> solve(
    const std::array<double, 3>& xyz,
    const std::optional<std::array<double, 3>>& rpy = std::nullopt) const;
```

```python
result: list[float] | None = ik.solve(
    xyz: list[float],           # [x, y, z] 目标位置 (m)
    rpy: list[float] | None = None  # [roll, pitch, yaw] 目标姿态 (rad)，None = 仅位置约束
)
# 返回: 7 个关节角 (rad)，求解失败返回 None
# 自动缓存上次结果作为下次初始猜测
```

#### `forward` — 正运动学

```cpp
std::array<double, 6> forward(const std::vector<double>& joint_angles) const;
```

```python
pose: list[float] = ik.forward(
    joint_angles: list[float]  # 7 个关节角 (rad)
)
# 返回: [x, y, z, roll, pitch, yaw]
```

#### `forward_matrix` — 正运动学齐次矩阵

```cpp
Eigen::Matrix4d forward_matrix(const std::vector<double>& joint_angles) const;
```

```python
matrix: list[list[float]] = ik.forward_matrix(
    joint_angles: list[float]  # 7 个关节角 (rad)
)
# 返回: 4x4 齐次变换矩阵（嵌套列表）
```

#### `get_dof` — 获取自由度

```python
dof: int = ik.get_dof()  # 返回 7
```

---

## RobotMotionController — 运动控制器

**头文件**: `robot_motion_controller.hpp` · 实现 `IRobotController` 接口

### 获取方式

不直接构造，通过 `RobotControllerNode::get_controller()` 获取：

```python
robot = rc.RobotControllerNode.create(profile, gripper, topics)
ctrl = robot.get_controller()
```

### 关节空间控制

#### `set_arm` — 设置关节角度

```python
ctrl.set_arm(
    angles: list[float],   # 7 个关节角 (rad)
    block: bool = True     # 是否阻塞等待到位
)
```

#### `rotate_joint` — 旋转单关节

```python
ctrl.rotate_joint(
    index: int,            # 关节索引 0~6
    delta_angle: float,    # 增量 (rad)
    block: bool = True
)
```

#### `go_home` — 回归 home 位

```python
ctrl.go_home(block: bool = True)
```

### 笛卡尔空间控制

#### `move_to_pose` — IK 位姿控制

```python
ctrl.move_to_pose(
    xyz: list[float],              # [x, y, z] 目标位置 (m)，当前 TCP 坐标系
    rpy: list[float] | None,       # [roll, pitch, yaw] 目标姿态 (rad)，None = 仅位置
    finger: float = -1.0,          # 夹爪宽度，-1 = 保持当前
    steps: int = 0,                # 插值步数，0 = 不插值直接发送
    step_time: float = 0.08,       # 插值步间隔 (s)
    block: bool = True             # 是否阻塞等待到位
)
```

**finger 参数说明：**
- `-1.0`（默认）：非抓取时保持当前夹爪宽度，抓取时保持目标宽度以维持夹持力
- `0.0~0.04`：设置夹爪到指定宽度

**抓取流程中的 finger 用法：**
```python
# 1. 接近/下降时保持夹爪张开
ctrl.move_to_pose(approach_pos, rpy, finger=gripper.max_width, steps=15, step_time=0.06)

# 2. 闭合夹爪
ctrl.close_gripper()

# 3. 提起时保持夹爪闭合（发送 min_width 维持夹持力）
ctrl.move_to_pose(lift_pos, rpy, finger=gripper.min_width, steps=15, step_time=0.06)
```

#### `move_linear` — 相对平移

```python
ctrl.move_linear(
    delta: list[float],            # [dx, dy, dz] 相对位移 (m)
    frame: str = "base",           # "base" = 基座坐标系, "end_effector" = 末端坐标系
    finger: float = -1.0,
    block: bool = True
)
```

### 夹爪控制

#### `set_gripper` — 设置夹爪宽度

```python
ctrl.set_gripper(width: float, block: bool = True)
```

#### `open_gripper` — 张开夹爪

```python
ctrl.open_gripper(block: bool = True)
```

#### `close_gripper` — 闭合夹爪（含力保持）

```python
ctrl.close_gripper(block: bool = True)
```

闭合后自动设置 `grasping_` 标志，后续运动命令会持续发送目标夹爪宽度（0.0）以维持夹持力。

### 状态查询

#### `get_joint_angles` — 获取当前关节角度

```python
angles: list[float] = ctrl.get_joint_angles()  # 7 个值 (rad)
```

#### `get_end_effector_pose` — 获取当前 TCP 位姿

```python
pose: list[float] = ctrl.get_end_effector_pose()
# 返回: [x, y, z, roll, pitch, yaw]，当前 TCP 坐标系
```

#### `get_finger_width` — 获取夹爪宽度

```python
width: float = ctrl.get_finger_width()  # 单位: m
```

### TCP 工具坐标系

#### `set_tcp` — 切换 TCP

```python
ctrl.set_tcp(name: str)  # "hand" 或 "grasptarget"
```

#### `get_current_tcp` — 获取当前 TCP 名称

```python
name: str = ctrl.get_current_tcp()
```

### TF 查询

#### `lookup_transform` — 查询坐标系变换

```python
result: list[float] | None = ctrl.lookup_transform(
    target_frame: str,     # 目标坐标系
    source_frame: str,     # 源坐标系
    timeout: float = 1.0   # 超时 (s)
)
# 返回: [x, y, z, roll, pitch, yaw] 或 None
```

---

## ColorDetector — 颜色检测器

**头文件**: `color_detector.hpp` · 继承 `CameraInterface`

### 构造

```python
detector = rc.ColorDetector(
    lower_hsv: list[int],  # [h_min, s_min, v_min]
    upper_hsv: list[int]   # [h_max, s_max, v_max]
)
```

### 方法

#### `set_camera_intrinsics` — 设置相机内参

```python
detector.set_camera_intrinsics(fx: float, fy: float, cx: float, cy: float)
# 默认: fx=fy=614, cx=320, cy=240
```

### 模块级函数

#### `pixel_to_3d` — 像素坐标反投影

```python
xyz: list[float] = rc.pixel_to_3d(
    u: int, v: int,       # 像素坐标
    depth: float,          # 深度值 (m)
    fx: float, fy: float,  # 焦距
    cx: float, cy: float   # 光心
)
# 返回: [x, y, z] 相机坐标系 3D 位置
```

**注意**: ColorDetector 的 `detect()`, `process_image()`, `draw_target()` 方法涉及 `cv::Mat`，未暴露给 Python。图像处理由 `VisionProcessorNode` 在 C++ 内部完成。

---

## GraspTaskManager — 抓取状态机

**头文件**: `grasp_task_manager.hpp`

### 构造

```python
task = rc.GraspTaskManager(
    robot: rc.RobotMotionController,       # 运动控制器
    vision: rc.VisionProcessorNode,         # 视觉处理器
    approach_height: float = 0.15,          # 接近高度 (m)
    grasp_height_offset: float = 0.02,      # 抓取高度微调 (m)
    grasp_rpy: list[float] = [3.14159, 0.0, 3.14159]  # 抓取姿态 (rad)
)
```

### 方法

#### `run` — 执行完整抓取流程

```python
success: bool = task.run(timeout: float = 30.0)
# 阻塞执行，返回 True=成功
```

#### `get_state` — 获取当前状态

```python
state: rc.GraspState = task.get_state()
```

### 状态转移

```
IDLE → DETECTING → APPROACHING → DESCENDING → GRASPING → LIFTING → DONE
                                   ↘ ERROR (异常/超时)
```

---

## RobotControllerNode — ROS2 控制节点

**头文件**: `robot_controller_node.hpp`

### 工厂方法

```python
robot = rc.RobotControllerNode.create(
    profile: rc.RobotProfile,      # 机器人参数
    gripper: rc.GripperProfile,     # 夹爪参数
    topics: rc.TopicConfig          # 话题配置
)
```

**注意**: 使用工厂方法而非直接构造，因为内部需要 `shared_from_this()`。

### 方法

#### `wait_for_ready` — 等待关节状态就绪

```python
ready: bool = robot.wait_for_ready(timeout: float = 5.0)
# 阻塞等待首次收到 /joint_states，超时返回 False
```

#### `get_controller` — 获取运动控制器

```python
ctrl: rc.RobotMotionController = robot.get_controller()
```

#### `get_logger` — 获取日志器

```python
logger: rc.Logger = robot.get_logger()
logger.info("消息")
logger.warn("警告")
logger.error("错误")
```

### TF2 广播

每次收到 `/joint_states` 时自动发布：
- `panda_link0` → `panda_hand`（FK 计算）
- `panda_hand` → `<tcp_name>`（TCP 偏移）

---

## VisionProcessorNode — ROS2 视觉节点

**头文件**: `vision_processor_node.hpp`

### 工厂方法

```python
vision = rc.VisionProcessorNode.create(
    processor: rc.CameraInterface,  # 检测器（如 ColorDetector）
    topics: rc.TopicConfig           # 话题配置
)
```

### 方法

#### `get_latest_result` — 获取最新检测结果

```python
result: rc.DetectionResult | None = vision.get_latest_result()
# 非阻塞，返回最新结果或 None
```

#### `wait_for_detection` — 阻塞等待检测结果

```python
result: rc.DetectionResult | None = vision.wait_for_detection(
    timeout: float = 10.0  # 超时 (s)
)
# 阻塞直到检测到目标或超时
```

#### `get_logger` — 获取日志器

```python
logger: rc.Logger = vision.get_logger()
```

### 图像同步

内部使用 `message_filters::ApproximateTime` 策略同步 RGB + 深度图像，时间容差 0.1s，队列大小 10。

---

## Profile 工厂函数

**Python 子模块**: `rc.profiles`

```python
profile = rc.profiles.panda()              # → RobotProfile
gripper = rc.profiles.panda_gripper()       # → GripperProfile
detector = rc.profiles.panda_red_detector() # → ColorDetector (红色 HSV 检测)
```

**Panda 默认配置：**

| 参数 | 值 |
|------|----|
| DOF | 7 |
| Home | `[0, 0, 0, -1.57, 0, 1.57, 0]` + 夹爪 `[0.4, 0.4]` |
| TCP | `hand`: 无偏移；`grasptarget`: Z +0.1m |
| 夹爪范围 | 0.0 ~ 0.04 m |
| 关节限位 | ±2.8973 rad（各轴不同） |

---

## Python 绑定完整参考

### 导入

```python
import robot_control_cpp_py as rc
```

### ROS2 生命周期

```python
rc.rclcpp_init()       # 初始化 rclcpp（替代 rclpy.init）
rc.rclcpp_shutdown()   # 关闭 rclcpp（替代 rclpy.shutdown）
```

**重要**: `rclcpp_init()` 和 `rclpy.init()` 不能在同一进程中共存，因为底层 `rcl_init()` 只能调用一次。

### Executor

```python
executor = rc.MultiThreadedExecutor()
executor.add_node(robot)     # 添加 ROS2 节点
executor.spin()              # 阻塞处理回调（自动释放 GIL）
executor.cancel()            # 取消 spin
```

### 完整使用模板

```python
import threading
import robot_control_cpp_py as rc

def main():
    # 1. 初始化
    rc.rclcpp_init()

    # 2. 创建节点
    profile = rc.profiles.panda()
    gripper = rc.profiles.panda_gripper()
    robot = rc.RobotControllerNode.create(profile, gripper, rc.TopicConfig())

    # 3. 后台 spin
    executor = rc.MultiThreadedExecutor()
    executor.add_node(robot)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # 4. 等待就绪
    robot.wait_for_ready()

    # 5. 使用控制器
    ctrl = robot.get_controller()
    # ... 运动控制 ...

    # 6. 清理（必须 join 线程再 shutdown）
    executor.cancel()
    spin_thread.join()
    rc.rclcpp_shutdown()

if __name__ == "__main__":
    main()
```

### 带视觉的完整模板

```python
import threading
import robot_control_cpp_py as rc

def main():
    rc.rclcpp_init()

    profile = rc.profiles.panda()
    gripper = rc.profiles.panda_gripper()
    detector = rc.profiles.panda_red_detector()
    topics = rc.TopicConfig()

    robot = rc.RobotControllerNode.create(profile, gripper, topics)
    vision = rc.VisionProcessorNode.create(detector, topics)

    executor = rc.MultiThreadedExecutor()
    executor.add_node(robot)
    executor.add_node(vision)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    robot.wait_for_ready()
    ctrl = robot.get_controller()

    # 等待检测结果
    result = vision.wait_for_detection(timeout=10.0)
    if result:
        print(f"检测到目标: xyz={result.xyz}, uv={result.uv}")

    executor.cancel()
    spin_thread.join()
    rc.rclcpp_shutdown()
```

### 使用 GraspTaskManager 一键抓取

```python
rc.rclcpp_init()
# ... 创建 robot 和 vision 节点 ...

task = rc.GraspTaskManager(ctrl, vision)
success = task.run(timeout=30.0)
print(f"抓取结果: {success}, 状态: {task.get_state()}")

# ... 清理 ...
```

---

## C++ / Python 类型映射

| C++ 类型 | Python 类型 | 说明 |
|----------|-------------|------|
| `std::vector<double>` | `list[float]` | 自动转换 |
| `std::array<double, 3>` | `list[float]` | 3 元素列表 |
| `std::array<double, 6>` | `list[float]` | 6 元素列表 |
| `std::array<int, 3>` | `list[int]` | HSV 值 |
| `std::optional<T>` | `T \| None` | 自动转换 |
| `std::map<K, V>` | `dict[K, V]` | 自动转换 |
| `std::string` | `str` | 自动转换 |
| `Eigen::Vector3d` | `list[float]` | 3 元素，绑定层转换 |
| `Eigen::Vector2i` | `list[int]` | 2 元素，绑定层转换 |
| `Eigen::Matrix4d` | `list[list[float]]` | 4x4 嵌套列表 |
| `bool` | `bool` | 直接映射 |
| `double` | `float` | 直接映射 |
| `int` | `int` | 直接映射 |

---

## 线程安全

| 类 | 线程安全 | 机制 |
|----|----------|------|
| `IKSolver` | 是 | 构造后无状态修改 |
| `RobotMotionController` | 是 | Bridge 内部 mutex |
| `RosMotionBridge` | 是 | `state_mutex_` 保护关节状态 |
| `RobotControllerNode` | 是 | 回调组 + mutex |
| `GraspTaskManager` | 否 | 单线程状态机 |
| `ColorDetector` | 是 | 无可变状态 |
| `VisionProcessorNode` | 是 | `result_mutex_` + condition_variable |

### GIL 注意事项

pybind11 绑定中，所有阻塞方法自动释放 Python GIL：

- `spin()` — `py::call_guard<py::gil_scoped_release>()`
- `wait_for_ready()`, `wait_for_detection()` — 同上
- 所有 `block=True` 的运动方法 — 同上

这意味着 Python 主线程在等待机器人运动时，不会阻塞其他 Python 线程。

---

## 开发指南

### 添加新机器人

1. 创建 Profile 文件 `include/robot_control_cpp/my_robot_profile.hpp`：

```cpp
#pragma once
#include "robot_control_cpp/robot_profile.hpp"

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
  p.joint_limits_lower = { /* ... */ };
  p.joint_limits_upper = { /* ... */ };
  p.home_joints = { /* 6 + 2 */ };
  p.ik_default_guess = { /* 6 */ };
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
3. 在 `bindings.cpp` 中添加 Profile 绑定函数。
4. 核心库（IKSolver、RobotMotionController、GraspTaskManager）无需修改。

### 替换视觉算法

`CameraInterface` 是抽象基类，实现 `process_image()` 即可替换检测算法：

```cpp
class YoloDetector : public robot_control::CameraInterface {
public:
  DetectionResult process_image(const cv::Mat& rgb,
                                const cv::Mat& depth) const override {
    // YOLO 推理 + 深度图 3D 定位
  }
};
```

C++ 中使用：
```cpp
auto detector = std::make_shared<YoloDetector>();
auto vision = VisionProcessorNode::create(detector, topics);
```

### 编写新的 Python 脚本

使用 C++ 后端的脚本模板：

```python
import threading
import robot_control_cpp_py as rc

rc.rclcpp_init()

robot = rc.RobotControllerNode.create(
    rc.profiles.panda(), rc.profiles.panda_gripper(), rc.TopicConfig())

executor = rc.MultiThreadedExecutor()
executor.add_node(robot)
spin_thread = threading.Thread(target=executor.spin, daemon=True)
spin_thread.start()

try:
    robot.wait_for_ready()
    ctrl = robot.get_controller()
    # ... 自定义逻辑 ...
finally:
    executor.cancel()
    spin_thread.join()
    rc.rclcpp_shutdown()
```

### 编写新的 C++ 测试/演示

在 `src/robot_control_test/` 下添加源文件，并在其 `CMakeLists.txt` 中注册：

```cmake
add_executable(test_my_feature test/test_my_feature.cpp)
target_link_libraries(test_my_feature robot_control_cpp::robot_control_core)
# 或需要 ROS2 节点:
# target_link_libraries(test_my_feature robot_control_cpp::robot_control_nodes)
install(TARGETS test_my_feature RUNTIME DESTINATION lib/${PROJECT_NAME})
```

### 编码规范

| 规范 | 说明 |
|------|------|
| 命名 | 类名 PascalCase，方法/变量 snake_case |
| 智能指针 | 禁止裸指针，使用 `shared_ptr` / `unique_ptr` |
| 注释 | 公开头文件必须包含 Doxygen 注释 |
| 接口隔离 | ROS2 通信逻辑不得污染核心逻辑 |
| 线程同步 | `std::mutex` + `std::condition_variable` |
| 异常 | 核心库抛 `std::runtime_error`，ROS 节点层捕获并日志 |
| 节点构造 | 使用工厂模式 `create()` + 两阶段初始化 |
| Python 脚本 | 使用 `rclcpp_init()` / `rclcpp_shutdown()`，禁止 `rclpy` |
