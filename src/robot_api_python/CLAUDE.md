# robot_api_python

## 职责
通过 pybind11 将 C++ 核心（robot_controller + robot_vision）暴露为 Python API。
使 Python 脚本能够使用 `rclcpp`（而非 `rclpy`）控制机器人，支持运动控制、视觉检测和抓取任务。

## 节点清单
无可执行节点。编译为 Python 扩展模块 `_core.cpython-*.so`，通过 `import robot_api_python` 使用。

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
| `TopicConfig` | ROS2 话题名称 + 相机外参 |
| `CameraExtrinsics` | 相机安装位姿（xyz, rpy） |
| `MotionLimits` | 运动限值（max_vel, max_acc, max_jerk） |
| `DetectionResult` | 视觉检测结果（detected, xyz, uv, confidence） |
| `MotionMode` | 枚举：MOVE_J, MOVE_L |
| `GraspState` | 枚举：IDLE, DETECTING, ..., DONE, ERROR |

### 核心控制类（Layer 1）

| 类 | 关键方法 | 说明 |
|----|---------|------|
| `IKSolver` | solve(xyz, rpy), forward(angles), forward_matrix(angles), velocity_ik() | IK/FK 求解器 |
| `RobotMotionController` | moveJ(), moveL(), set_arm(), open/close_gripper(), go_home(), move_to_pose(), move_linear() | 运动控制（所有阻塞方法释放 GIL） |
| `ColorDetector` | detect(image), process_image(rgb, depth), set_camera_intrinsics() | HSV 颜色检测器 |
| `GraspTaskManager` | run(timeout), get_state(), transform_to_base(xyz) | 抓取任务状态机 |
| `pixel_to_3d(u, v, depth, fx, fy, cx, cy)` | 像素坐标 → 3D 点 | 工具函数 |

### ROS2 节点（Layer 2）

| 类 | 工厂方法 | 说明 |
|----|---------|------|
| `RobotControllerNode` | `create(profile, gripper, topics)` | 控制节点：wait_for_ready(), get_controller(), get_logger() |
| `VisionProcessorNode` | `create(processor, topics)` | 视觉节点：get_latest_result(), wait_for_detection(), get_logger() |

### 预定义 Profile

| 函数 | 说明 |
|------|------|
| `profiles.panda()` | 返回 Franka Panda RobotProfile |
| `profiles.panda_gripper()` | 返回 Panda 夹爪 GripperProfile |

### 模块常量

| 常量 | 值 | 说明 |
|------|----|------|
| JOINT_TOLERANCE | 0.05 rad | 关节容差 |
| FINGER_TOLERANCE | 0.002 m | 夹爪容差 |
| MOTION_TIMEOUT | 10.0 s | 运动超时 |
| TRAJECTORY_DT | 0.02 s | 轨迹采样间隔 |
| READY_TIMEOUT | 5.0 s | 就绪超时 |
| CONTROL_LOOP_HZ | 100.0 Hz | 控制循环频率 |

## 关键参数
无 YAML 配置。所有参数通过 Python API 传递。

## 使用模式

```python
import threading
import robot_api_python as rc

rc.rclcpp_init()

# 创建节点
robot = rc.RobotControllerNode.create(
    rc.profiles.panda(), rc.profiles.panda_gripper(), rc.TopicConfig())
detector = rc.ColorDetector([0, 100, 100], [10, 255, 255])
vision = rc.VisionProcessorNode.create(detector, rc.TopicConfig())

# 后台 Executor
executor = rc.MultiThreadedExecutor()
executor.add_node(robot)
executor.add_node(vision)
thread = threading.Thread(target=executor.spin, daemon=True)
thread.start()

# 控制机器人
robot.wait_for_ready()
ctrl = robot.get_controller()
ctrl.open_gripper()
ctrl.move_to_pose([0.5, 0, 0.3], [0, -3.14, -3.14])

# 抓取任务
task = rc.GraspTaskManager(ctrl, vision)
task.run()

executor.cancel()
thread.join()
rc.rclcpp_shutdown()
```

## 启动方式
此包为 Python 库，通过 `import robot_api_python` 使用：
```bash
python3 script/test_move_cpp.py       # IK 位姿控制演示
python3 script/test_vision_cpp.py     # 视觉引导抓取演示
python3 script/test_grasp_tcp_cpp.py  # TCP 抓取演示
python3 script/test_camera_tf.py      # 相机 TF 验证
```

## 包内依赖
- **内部依赖**: robot_controller, robot_vision
- **外部依赖**: rclcpp, pybind11-dev

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
- **类型转换**: `pybind11` 绑定代码在 `src/bindings.cpp` 中集中管理（约 390 行）
