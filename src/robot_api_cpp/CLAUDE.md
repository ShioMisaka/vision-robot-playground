# robot_api_cpp

## 职责
C++ Service 客户端库，提供轻量级 ROS2 节点（`RobotClient`）和 Service 代理控制器
（`ServiceRobotController`），用于连接外部运行的 `robot_controller_node`。
无需链接完整的 C++ 控制器库，适用于 C++ 演示程序和集成测试。

## 节点清单
无可执行节点。编译为共享库 `robot_api_client_lib`，供下游包链接使用。

## CMake Target 分层

```
robot_api_client_lib (共享库)  ← ServiceRobotController + RobotClient（依赖 robot_motion）
```

## 话题 / 服务 / Action 接口
此包不直接定义接口，而是通过 Service 客户端调用 `robot_controller_node` 的 Service。

### Service 客户端（ServiceRobotController）

| Service | 类型 | 说明 |
|---------|------|------|
| `~/move_joint` | MoveJoint | 关节空间运动 |
| `~/move_pose` | MovePose | 笛卡尔运动（mode: 0=moveJ, 1=moveL） |
| `~/move_linear` | MoveLinear | 线性增量运动 |
| `~/control_gripper` | ControlGripper | 夹爪控制 |
| `~/go_home` | GoHome | 回安全位 |
| `~/set_speed` | SetSpeed | 设置模式速度 |
| `~/get_state` | GetRobotState | 查询状态 |
| `~/set_tcp` | SetTCP | 设置 TCP |

## 核心类与修改入口

### robot_api 命名空间

| 文件 | 类 | 职责 |
|------|-----|------|
| `include/robot_api/service_robot_controller.hpp` | ServiceRobotController | IRobotController 的 Service 客户端实现，实现完整运动控制接口 |
| `src/service_robot_controller.cpp` | ServiceRobotController 各方法 | ROS2 Service 调用 + 状态缓存 + TF 查询 |
| `include/robot_api/robot_client.hpp` | RobotClient | 轻量 ROS2 客户端节点，持有 ServiceRobotController |
| `src/robot_client.cpp` | RobotClient::create(), wait_for_services() | 工厂创建 + Service 就绪等待 |

### ServiceRobotController 关键设计
- 实现 `robot_control::IRobotController` 接口，与 `RobotMotionController` 接口一致
- 所有运动命令通过 ROS2 Service 同步调用实现
- 状态通过 `refresh_state_cache()` 缓存（关节角、末端位姿、夹爪宽度、TCP、速度）
- TF 查询通过 `tf2_ros::Buffer` + `TransformListener` 实现
- 线程安全：`cache_mutex_` 保护状态缓存

## 关键参数
无 YAML 配置。所有参数通过 API 传递。

## 启动方式
此包为共享库，通过链接使用：
```cpp
#include "robot_api/robot_client.hpp"

auto client = robot_api::RobotClient::create("robot_controller_node");
client->wait_for_services();
auto ctrl = client->get_controller();
ctrl->go_home();
```

前提：`robot_controller_node` 需已独立运行。

## 包内依赖
- **内部依赖**: robot_controller（使用 IRobotController, robot_motion）, robot_logger
- **外部依赖**: rclcpp, robot_msgs, tf2_ros

## 修改指南
- **新增 Service 客户端** → 在 `ServiceRobotController` 中添加新的 `rclcpp::Client` 成员和调用方法
- **修改状态缓存策略** → 编辑 `src/service_robot_controller.cpp` 的 `refresh_state_cache()`
- **修改客户端创建逻辑** → 编辑 `src/robot_client.cpp` 的 `create()` 工厂方法

## 注意事项
- **命名空间**: 此包所有类使用 `namespace robot_api`
- **无 Python**: 纯 C++ 库，无 pybind11 绑定（Python 绑定在 `robot_api_python` 中）
- **无 robot_vision 依赖**: 此包不依赖 `robot_vision`，仅提供运动控制客户端
- **ServiceRobotController 与 RobotMotionController 接口一致**: 可在需要时无缝切换
- **Service 前缀**: 通过 `service_prefix` 参数指定目标控制器节点的命名空间
