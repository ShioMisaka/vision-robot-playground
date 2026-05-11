# robot_api_python

## 职责
通过 pybind11 将 C++ 核心（robot_controller + robot_vision + robot_api_cpp + robot_tasks）暴露为 Python API。
使 Python 脚本能够使用 `rclcpp`（而非 `rclpy`）控制机器人，支持运动控制、视觉检测和抓取任务。
提供两种使用模式：直接链接 C++ 库（RobotControllerNode）和通过 Service 调用外部节点（RobotClient）。

## 节点清单
无可执行节点。编译为 Python 扩展模块 `_core.cpython-*.so`，通过 `import robot_api_python` 使用。

## CMake Target 分层

```
robot_api_cpp::robot_api_client_lib  ← ServiceRobotController + RobotClient（来自 robot_api_cpp 包）
       ▲
_core (pybind11 模块)                ← Python 绑定 + 依赖 robot_api_cpp + robot_controller + robot_vision
```

注：`robot_api_client_lib`（ServiceRobotController + RobotClient）已提取至 `robot_api_cpp` 包，
此包通过链接 `robot_api_cpp::robot_api_client_lib` 使用。

## 话题 / 服务 / Action 接口
此包不直接定义接口，而是暴露底层 `robot_controller` 和 `robot_vision` 的完整接口。

## Python API 表面

### ROS2 生命周期

| 函数/类 | 说明 |
|---------|------|
| `rclcpp_init()` | 初始化 rclcpp（替代 rclpy.init） |
| `rclcpp_shutdown()` | 关闭 rclcpp |
| `MultiThreadedExecutor` | 多线程执行器：add_node(), spin()（释放 GIL）, cancel() |
| `Logger` | 日志接口：info(), warn(), error() |

### 配置数据结构

| 类 | 说明 |
|----|------|
| `RobotProfile` | 机器人参数（DOF、关节限位、Home 位、TCP、运动限值） |
| `GripperProfile` | 夹爪参数（type, min/max_width, DOF） |
| `TcpConfig` | TCP 偏移（offset_xyz, offset_rpy） |
| `TopicConfig` | ROS2 话题名称 + 相机外参（来自 robot_controller） |
| `VisionTopicConfig` | 视觉节点相机话题配置（camera_left, camera_depth, sync_queue_size, sync_max_slop） |
| `CameraConfig` | 相机完整配置（intrinsics + extrinsics + optical frame + frame names） |
| `MotionLimits` | 运动限值（max_vel, max_acc, max_jerk） |
| `DetectionResult` | 视觉检测结果（detected, xyz, uv, confidence, label） |
| `MotionMode` | 枚举：MOVE_J, MOVE_L |
| `GraspState` | 枚举：IDLE, DETECTING, APPROACHING, RE_DETECTING, DESCENDING, GRASPING, LIFTING, DONE, ERROR |

### 核心控制类（Layer 1）

| 类 | 关键方法 | 说明 |
|----|---------|------|
| `IKSolver` | solve(xyz, rpy), forward(angles), forward_matrix(angles), velocity_ik() | IK/FK 求解器 |
| `RobotMotionController` | moveJ(), moveL(), set_arm(), open/close_gripper(), go_home(), move_to_pose(), move_linear(), rotate_joint(), lookup_transform(), set_speed(), get_speed(), set_tcp(), get_current_tcp() | 运动控制（所有阻塞方法释放 GIL） |
| `ColorDetector` | set_camera_intrinsics(fx, fy, cx, cy) | HSV 颜色检测器（仅暴露内参设置，detect/process_image 未绑定） |
| `GraspTaskManager` | run(timeout), get_state(), transform_to_base(xyz) | 抓取任务状态机 |
| `pixel_to_3d(u, v, depth, fx, fy, cx, cy)` | 像素坐标 → 3D 点 | 工具函数 |

### ROS2 节点（Layer 2）

#### RobotClient（推荐，连接外部节点，来自 robot_api_cpp）
| 类 | 工厂方法 | 说明 |
|----|---------|------|
| `RobotClient` | `create(service_prefix="robot_controller_node")` | 轻量客户端（`robot_api::RobotClient`）：通过 ROS2 Service 调用外部 robot_controller_node |
| `ServiceRobotController` | `robot.get_controller()` | Service 代理控制器（`robot_api::ServiceRobotController`），与 RobotMotionController 接口一致 |

#### RobotControllerNode（deprecated，直接链接 C++ 库）
| 类 | 工厂方法 | 说明 |
|----|---------|------|
| `RobotControllerNode` | `create(profile, gripper, topics)` | 内嵌 C++ 控制节点（已弃用，推荐使用 RobotClient） |
| `VisionProcessorNode` | `create(processor, config)` | 视觉节点：接受 VisionTopicConfig 参数，get_latest_result(), wait_for_detection(), get_logger() |

### 预定义 Profile

| 函数/类 | 说明 |
|---------|------|
| `load_profile(name="panda")` | 从 YAML 加载机器人配置，返回 `RobotConfig`（含 `.robot` 和 `.gripper`） |
| `RobotConfig` | 配置数据结构：`.robot` (RobotProfile) + `.gripper` (GripperProfile) |

### 模块常量

| 常量 | 值 | 说明 |
|------|----|------|
| JOINT_TOLERANCE | 0.05 rad | 关节容差 |
| FINGER_TOLERANCE | 0.002 m | 夹爪容差 |
| MOTION_TIMEOUT | 10.0 s | 运动超时 |
| TRAJECTORY_DT | 0.02 s | 轨迹采样间隔 |
| READY_TIMEOUT | 5.0 s | 就绪超时 |
| CONTROL_LOOP_HZ | 100.0 Hz | 控制循环频率 |
| POLL_INTERVAL | 0.02 s | 反馈轮询间隔 |
| SETTLE_TIME | 0.2 s | 稳定等待 |
| DEFAULT_STEPS | 10 | 默认插值步数 |
| DEFAULT_STEP_TIME | 0.08 s | 默认步进时间 |
| IMAGE_SYNC_QUEUE_SIZE | 10 | 图像同步队列 |
| IMAGE_SYNC_SLOP | 0.1 s | 图像同步容差 |
| FINGER_STABLE_COUNT | 5 | 夹爪稳定采样数 |
| FINGER_STABLE_TOL | 0.001 m | 夹爪稳定容差 |

### robot_logger 子模块

```python
from robot_api_python._core import robot_logger

# Python 不支持 fmt 格式参数，必须用 f-string
robot_logger.info(f"Count: {count}")
robot_logger.warn("Timeout")
robot_logger.error(f"Failed: {msg}")
robot_logger.set_level("debug")              # 全局
robot_logger.set_level("vision", "debug")    # 按模块
```

## 关键参数
无 YAML 配置。所有参数通过 Python API 传递。

## 使用模式

### 模式一：RobotClient（推荐，连接外部节点）

```python
import threading
import robot_api_python as rc

rc.rclcpp_init()

# 创建轻量客户端（连接独立运行的 robot_controller_node）
robot = rc.RobotClient.create("robot_controller_node")
robot.wait_for_services()

# 直接控制（通过 Service 调用）
ctrl = robot.get_controller()
ctrl.open_gripper()
ctrl.move_to_pose([0.5, 0, 0.3], [0, -3.14, -3.14])
ctrl.close_gripper()

rc.rclcpp_shutdown()
```

### 模式二：RobotControllerNode（内嵌 C++ 库，deprecated）

```python
import threading
import robot_api_python as rc

rc.rclcpp_init()

# 创建内嵌节点（直接链接 C++ 库）
config = rc.load_profile("panda")
robot = rc.RobotControllerNode.create(
    config.robot, config.gripper, rc.TopicConfig())

# 后台 spin
executor = rc.MultiThreadedExecutor()
executor.add_node(robot)
thread = threading.Thread(target=executor.spin, daemon=True)
thread.start()

robot.wait_for_ready()
ctrl = robot.get_controller()
ctrl.open_gripper()

executor.cancel()
thread.join()
rc.rclcpp_shutdown()
```

## 启动方式
此包为 Python 库，通过 `import robot_api_python` 使用：
```bash
python3 script/test_move_cpp.py         # IK 位姿控制（RobotClient）
python3 script/test_grasp_tcp_cpp.py    # TCP 抓取（RobotClient）
python3 script/test_vision_cpp.py       # 视觉引导抓取（RobotClient）
python3 script/test_camera_tf.py        # 相机 TF 验证（RobotControllerNode）
```

## 包内依赖
- **内部依赖**: robot_api_cpp, robot_controller, robot_vision, robot_tasks, robot_msgs, robot_logger
- **外部依赖**: rclcpp, tf2_ros, pybind11-dev

## 修改指南
- **新增 C++ 绑定** → 编辑 `src/bindings.cpp`，参考现有 `py::class_` 写法
- **新增 Profile** → 在 `src/bindings.cpp` 的 profiles 子模块中添加绑定函数
- **新增常量** → 在 `src/bindings.cpp` 的常量导出区域添加 `m.attr("xxx") = value`
- **修改 Python 导出** → 编辑 `robot_api_python/__init__.py`
- **新增 Python 便捷方法** → 在 `robot_api_python/` 目录下创建新模块

## 注意事项
- **禁止使用 rclpy**: 此包使用 `rclcpp`（通过 pybind11），不能同时使用 `rclpy.init()`
- **GIL 释放**: 所有阻塞方法（spin, moveJ, wait_for_detection 等）自动释放 GIL，支持 Python 多线程
- **Eigen → list**: C++ 的 Eigen 类型自动转换为 Python 原生 list/tuple，不使用 numpy
- **std::optional → None**: C++ 的 `std::optional` 自动映射为 Python `None`
- **ColorDetector 限制**: Python 侧仅暴露 `set_camera_intrinsics()`，`detect()` 和 `process_image()` 未绑定（需要 cv::Mat，pybind11 不支持直接传递）
- **RobotControllerNode 已弃用**: 推荐使用 `RobotClient` 连接外部节点，避免在 Python 进程中内嵌 C++ 控制器
- **类型转换**: pybind11 绑定代码在 `src/bindings.cpp` 中集中管理（约 560 行）
- **ServiceRobotController**: 与 RobotMotionController 拥有相同接口，但通过 ROS2 Service 调用实现，状态通过话题缓存
- **robot_api 命名空间**: `ServiceRobotController` 和 `RobotClient` 使用 `robot_api::` 命名空间（来自 robot_api_cpp 包），Python 侧导出时不带命名空间前缀
