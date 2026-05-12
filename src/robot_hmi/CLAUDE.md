# robot_hmi

## 职责
Qt5 示教器 GUI，提供机器人关节控制、笛卡尔 Jog、夹爪操作、相机画面显示和急停功能。
采用 Panel 架构，将 UI 拆分为 6 个自包含 widget。通过 `PendantNode` 连接外部运行的
`robot_controller_node`，使用 ROS2 Action、Lease Service 和 Topic 进行通信。
Jog 操作需要先获取 Lease 并进入示教模式。

## 节点清单
| 节点 | 源文件 | 功能 |
|------|--------|------|
| PendantNode | `src/pendant_node.cpp` | ROS2 通信后端（Service 客户端 + Topic 发布/订阅），连接外部 robot_controller_node |
| robot_hmi（可执行入口） | `src/main.cpp` | Qt + ROS2 线程整合 |

## 话题 / 服务 / Action 接口

### 话题

| 名称 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `~/joint_target` | sensor_msgs/JointState | Pub | 50Hz 关节目标流（发送到 RobotControllerNode，由 100Hz 控制循环统一执行） |
| `~/jog_command` | robot_msgs/JogCommand | Pub | Jog 点动命令（50Hz 心跳） |
| `/joint_states` | sensor_msgs/JointState | Sub | 关节反馈 |
| `~/status` | robot_msgs/RobotStatus | Sub | 机器人状态（检测 Jog 完成和 Fault） |
| `/camera/image_raw/left` | sensor_msgs/Image | Sub | 相机 RGB（message_filters 同步） |
| `/camera/image_raw/depth` | sensor_msgs/Image | Sub | 相机深度（message_filters 同步） |

### Service 客户端

| Service | 类型 | 说明 |
|---------|------|------|
| ~/control_gripper | ControlGripper | 夹爪控制 |
| ~/set_speed | SetSpeed | 设置模式速度 |
| ~/get_state | GetRobotState | 查询状态 |
| ~/robot_cmd | RobotCmd | STOP / E-STOP / CLEAR_FAULT |
| ~/acquire_control | AcquireControl | 获取控制权 Lease |
| ~/release_control | ReleaseControl | 释放控制权 |
| ~/request_teaching_mode | RequestTeachingMode | 请求示教模式（Jog 前置条件） |

### Action 客户端

| Action | 类型 | 说明 |
|--------|------|------|
| ~/move_j | MoveJ | 关节空间运动（替代原 MoveJoint/MovePose Service） |
| ~/move_l | MoveL | 线性运动（替代原 MoveLinear Service） |
| ~/go_home | GoHome | 回安全位（替代原 GoHome Service） |

## 核心类与修改入口

### 入口与编排

| 文件 | 类 | 职责 |
|------|-----|------|
| `src/main.cpp` | main() | ROS2 初始化 → 创建 PendantNode → 后台 Executor → Qt 事件循环 |
| `src/main_window.cpp` | MainWindow | 薄编排层：面板布局 + 回调中继 + 5Hz 状态轮询 |
| `include/robot_hmi/main_window.hpp` | MainWindow | 窗口定义（1100x800） |

### 通信后端

| 文件 | 类 | 职责 |
|------|-----|------|
| `src/pendant_node.cpp` | PendantNode | 所有 ROS2 通信：Action + Lease Service 客户端、Topic 发布/订阅、关节流控线程 |
| `include/robot_hmi/pendant_node.hpp` | PendantNode | 接口定义 |

**PendantNode 关键方法：**
- `create(service_prefix, joint_names)` → 工厂构造函数（接受关节名称列表）
- `acquire_lease()` → 获取控制权 Lease（返回 session_id）
- `release_lease()` → 释放控制权
- `request_teaching_mode()` → 请求示教模式（Jog 前置条件）
- `async_get_state()` → 异步查询状态（后台线程，2s 超时）
- `async_move_j()`, `async_move_l()` → 异步运动 Action（通过 MoveJ/MoveL Action）
- `async_go_home()` → 异步回安全位 Action
- `start_jog(axis, mode, frame)` → 开始 Jog（需持有 Lease + 示教模式，50Hz 定时器发送 JogCommand）
- `stop_jog()` → 停止 Jog（发送零速度）
- `start_joint_stream(initial)` → 启动 50Hz 关节目标流控线程（发布到 `~/joint_target`）
- `emergency_stop()`, `clear_fault()` → 急停/恢复
- `build_jog_command(axis, frame)` → 构建 JogCommand（线性 50mm/s，角速度 11°/s）

### Panel 组件

| 文件 | 类 | 职责 | 需要 PendantNode |
|------|-----|------|-----------------|
| `src/panels/connection_bar.cpp` | ConnectionBar | Robot/Camera 连接状态（红/橙/绿） | 否 |
| `src/panels/camera_panel.cpp` | CameraPanel | 相机画面显示 + RGB/Depth 切换 + E-STOP 遮罩 | 否 |
| `src/panels/robot_state_bar.cpp` | RobotStateBar | 位姿(XYZ/RPY)/夹爪/TCP 数值显示 | 否 |
| `src/panels/joint_control_panel.cpp` | JointControlPanel | 7 关节滑块 + 角度编辑 + 50Hz 流控 | 是 |
| `src/panels/cartesian_panel.cpp` | CartesianPanel | XYZ/RPY 输入 + Jog 12 轴按钮 + 运动模式 | 是 |
| `src/panels/function_panel.cpp` | FunctionPanel | moveJ/moveL 速度 + 夹爪/GoHome/E-STOP | 是 |

### 跨 Panel 通信（Qt 信号）

```
FunctionPanel::estopChanged(bool) ──→ JointControlPanel::onEstopChanged
                                  ├──→ CartesianPanel::onEstopChanged
                                  └──→ CameraPanel::onEstopChanged

JointControlPanel::jointStreamReady(array) → PendantNode::start_joint_stream()
```

### 线程架构

| 线程 | 职责 |
|------|------|
| Qt 主线程 | GUI 事件循环 + 5Hz 状态轮询 |
| ROS2 Executor 线程 | MultiThreadedExecutor 处理回调 |
| PendantNode Task 线程 | 后台 Service 异步调用 |
| PendantNode Joint Stream 线程 | 50Hz 关节指令发布（20ms 间隔） |
| PendantNode Discovery 定时器 | 2Hz 服务发现（500ms） |
| Jog Repeat 定时器 | 50Hz Jog 命令心跳 |

## 关键参数

| 参数 | 值 | 定义位置 |
|------|-----|---------|
| 状态轮询频率 | 5Hz (200ms) | MainWindow::refresh_timer_ |
| 关节流控频率 | 50Hz (20ms) | PendantNode 关节流控线程 |
| 滑块锁定时间 | 2s | JointControlPanel::lock_timer_ |
| Jog 线性速度 | 50 mm/s | PendantNode::build_jog_command() |
| Jog 角速度 | 11°/s | PendantNode::build_jog_command() |
| 故障轮询频率 | 5Hz (200ms) | FunctionPanel::fault_check_timer_ |
| 服务发现频率 | 2Hz (500ms) | PendantNode::discovery_timer_ |
| 图像帧率限制 | ~30fps | PendantNode::image_callback() |

## 关节流控机制

`JointControlPanel` 实现低延迟关节指令流：

1. **首帧同步**: 首次 `onStateUpdated` 触发 → 发出 `jointStreamReady` 信号 → `PendantNode::start_joint_stream()`
2. **50Hz 定时器**: `joint_follow_timer_` 以 20ms 间隔检测滑块变化 → 调用 `update_joint_target()`
3. **后台发布线程**: `PendantNode` 后台线程以 50Hz 发布到 `~/joint_target`（由 RobotControllerNode 的 100Hz 控制循环统一执行，避免双发布竞争）
4. **交互锁机制**: 用户拖动滑块/编辑角度时暂停自动更新 2 秒（`lock_timer_`）

## 启动方式
```bash
# 需要 Isaac Sim + robot_controller_node 运行中
ros2 run robot_hmi robot_hmi
```

## 包内依赖
- **内部依赖**: robot_msgs, robot_logger
- **外部依赖**: rclcpp, sensor_msgs, geometry_msgs, cv_bridge, message_filters, qtbase5-dev, OpenCV, tf2_ros, tf2_geometry_msgs

## 修改指南
- **修改 UI 布局** → 编辑 `src/main_window.cpp` 的 `setupUi()`
- **修改关节滑块行为** → 编辑 `src/panels/joint_control_panel.cpp`（限位、分辨率、锁定时间）
- **修改 Jog 按钮** → 编辑 `src/panels/cartesian_panel.cpp`（按钮布局、速度、坐标系）
- **修改速度/夹爪/E-STOP** → 编辑 `src/panels/function_panel.cpp`
- **修改 ROS2 通信逻辑** → 编辑 `src/pendant_node.cpp`（Service 超时、Topic QoS）
- **新增 Panel** → 创建 `include/robot_hmi/panels/xxx_panel.hpp` + `src/panels/xxx_panel.cpp`，继承 QWidget + Q_OBJECT 宏，在 `MainWindow::setupUi()` 中创建并布局，连接 `estopChanged` 信号（如需 E-STOP 通知），更新 `CMakeLists.txt` 的 `add_executable` 源文件列表
- **修改窗口大小** → 编辑 `src/main_window.cpp` 构造函数中的 `resize()` 调用

## 注意事项
- **AUTOMOC 要求**: 新增含 Q_OBJECT 宏的头文件必须在 `CMakeLists.txt` 的源文件列表中包含，否则 MOC 不生成元对象代码
- **线程安全**: ROS2 回调在 Executor 线程中执行，必须通过 `QMetaObject::invokeMethod` 跨线程更新 UI
- **PendantNode** 同时管理 Action 客户端、Lease Service 客户端和 Topic 发布，Action/Service 调用通过后台任务线程排队执行
- **E-STOP 机制**: FunctionPanel 发出 `estopChanged` 信号 → JointControlPanel 暂停流控 → CartesianPanel 禁用 Jog → CameraPanel 显示红色遮罩
- **Jog 需要 Lease**: Jog 操作需要先 acquire_lease() 获取控制权，再 request_teaching_mode() 进入示教模式
- **Jog 停止同步**: RobotStatus 检测到 kTeaching→kIdle 转换时，通过 `jog_stopped_cb` 通知 JointControlPanel 重新同步滑块到实际位置
