# robot_demos

## 职责
集中管理所有演示可执行文件和集成测试。从 `robot_controller`、`robot_vision`、`robot_api_python`
迁移而来，避免核心包包含演示代码。所有演示需要 Isaac Sim 运行并提供 `/joint_states` 反馈。

## 节点清单
无可导出库。此包仅编译可执行文件。

## 可执行文件

| 可执行文件 | 来源 | 功能 |
|-----------|------|------|
| `demo_grasp_tcp` | 原 robot_controller | TCP 抓取：切换 grasptarget TCP → 接近 → 下降 → 抓取 → 抬起 |
| `demo_camera` | 原 robot_vision | 显示同步 RGB + 深度图（JET colormap） |
| `demo_vision_grasp` | 原 robot_api_python | C++ 两阶段视觉引导抓取（客户端模式，连接外部 robot_controller_node） |
| `demo_vision_diagnostic` | 原 robot_api_python | 视觉坐标变换诊断工具（不运动机器人，打印变换链+误差分析） |
| `test_robot_node` | 原 robot_vision | 11 项集成测试：关节控制、夹爪、Home、IK、TCP、TF |

## 话题 / 服务 / Action 接口
此包不定义新接口，使用各依赖包的接口。

## 关键参数
无 YAML 配置。所有参数在源文件中硬编码或通过构造函数传递。

## 启动方式
```bash
# 前提：Isaac Sim + robot_controller_node 运行中

# TCP 抓取演示
ros2 run robot_demos demo_grasp_tcp

# 相机画面显示
ros2 run robot_demos demo_camera

# 视觉引导抓取演示
ros2 run robot_demos demo_vision_grasp

# 集成测试（11 项）
ros2 run robot_demos test_robot_node
```

## 包内依赖
- **内部依赖**: robot_controller, robot_vision, robot_api_cpp, robot_tasks, robot_logger, robot_description
- **外部依赖**: rclcpp, sensor_msgs, cv_bridge, message_filters, OpenCV

## 各演示链接关系

| 可执行文件 | 链接的 CMake Target |
|-----------|-------------------|
| `demo_grasp_tcp` | robot_controller::robot_nodes, robot_logger::robot_logger_lib |
| `demo_camera` | robot_controller::robot_nodes, robot_vision::robot_vision_nodes, robot_logger::robot_logger_lib |
| `demo_vision_grasp` | robot_api_cpp::robot_api_client_lib, robot_tasks::robot_tasks_lib, robot_vision::robot_vision_nodes, robot_logger::robot_logger_lib |
| `demo_vision_diagnostic` | robot_api_cpp::robot_api_client_lib, robot_tasks::robot_tasks_lib, robot_vision::robot_vision_nodes, robot_logger::robot_logger_lib |
| `test_robot_node` | robot_controller::robot_nodes, robot_vision::robot_vision_nodes, robot_logger::robot_logger_lib |

## 修改指南
- **修改 TCP 抓取流程** → 编辑 `demo/demo_grasp_tcp.cpp`
- **修改相机显示** → 编辑 `demo/demo_camera.cpp`
- **修改视觉引导抓取** → 编辑 `demo/demo_vision_grasp.cpp`
- **修改集成测试** → 编辑 `test/test_robot_node.cpp`
- **新增演示** → 在 `demo/` 或 `test/` 下创建源文件，在 `CMakeLists.txt` 中添加 `add_executable` 和 `install`

## 注意事项
- **需 Isaac Sim**: 所有演示和测试需要 Isaac Sim 运行并发布 `/joint_states`
- **需 robot_controller_node**: 大多数演示需要独立运行的 `robot_controller_node`
- **demo_vision_grasp 使用客户端模式**: 通过 `robot_api::RobotClient` 连接外部控制器，不内嵌 C++ 库
- **GraspTaskManager 来自 robot_tasks**: 命名空间为 `robot_tasks`，非 `robot_vision`
- **test_robot_node 为集成测试**: 覆盖关节控制、夹爪、Home、IK、TCP、TF 等 11 项功能，不是单元测试
- **日志模块名**: 所有可执行文件使用 `ROBOT_LOGGER_MODULE_NAME="demos"`
