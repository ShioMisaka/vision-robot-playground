# Isaac ROS Project

## 工程概述
基于 ROS2 Jazzy + NVIDIA Isaac Sim 的机械臂视觉引导抓取系统。
Franka Panda 7-DOF + 二指夹爪，ZED_X_Mini 双目深度相机，Qt5 示教器 GUI。
核心控制逻辑使用 C++17 实现，Python 通过 pybind11 调用 C++ 后端。

## 技术栈
- **ROS2**: Jazzy
- **构建工具**: colcon + ament_cmake
- **中间件**: DDS（默认 FastDDS）
- **语言**: C++17（核心库 + ROS2 节点 + Qt5 GUI）, Python 3.12（脚本，通过 pybind11 调用 C++）
- **仿真**: NVIDIA Isaac Sim（通过 ROS2 bridge 通信）
- **依赖**: Eigen3, Orocos KDL, OpenCV, Qt5, pybind11 2.11

## 包结构总览
| 包名 | 职责 |
|------|------|
| robot_msgs | ROS2 自定义接口（11 Service + 2 Action + 2 Message） |
| robot_logger | 统一日志系统（spdlog，宏接口，文件轮转） |
| robot_description | URDF 机器人模型（Panda + 夹爪 + 相机） |
| robot_bringup | 启动文件配置（当前为空壳） |
| robot_controller | 核心运动控制：IK/FK、S 曲线轨迹规划、Jog、100Hz 闭环 |
| robot_vision | 视觉处理：HSV 检测 + 深度 3D 定位 + 抓取状态机 |
| robot_hmi | Qt5 示教器 GUI（6 Panel 架构） |
| robot_api_python | pybind11 Python API 绑定 |

## 常用命令
```bash
# 所有 colcon build 使用 --base-paths src（src/ 不在 workspace 根目录下）
# 选择性编译前需 source install/setup.zsh 以解析已安装的依赖

# 一键编译全部
colcon build --base-paths src

# 选择性编译单个包
source install/setup.zsh
colcon build --base-paths src --packages-select <package_name>

# 编译示教器及其依赖
colcon build --base-paths src --packages-up-to robot_hmi

# 运行示教器（需要 Isaac Sim + robot_controller_node）
ros2 run robot_hmi robot_hmi

# 查看机器人状态
ros2 topic echo /robot_controller_node/status
```

## 文档体系
采用「根目录轻量 + 包级详细」两层结构。根目录只保留全局信息（技术栈、依赖关系、约定），
每个包的 CLAUDE.md 包含接口定义、核心类、修改入口和注意事项。不要将包级详情合并回根目录。

## 包间依赖关系
```
robot_msgs
    ▲
    │
robot_logger ─────────────────────────────────────┬──────────────┬───────────────┐
    ▲                                             │              │               │
    │                                             │              │               │
robot_description ──→ robot_controller ───────────┼─→ robot_vision ─→ robot_api_python
                                                    │
                                                    └─→ robot_hmi
```

**编译顺序**: robot_msgs → robot_logger + robot_description（可并行）→ robot_controller → robot_vision + robot_hmi（可并行）→ robot_api_python

## 全局约定

### 话题命名
- `/joint_command` — 关节指令（sensor_msgs/JointState，9 值：7 臂 + 2 爪）
- `/joint_states` — 关节反馈（来自 Isaac Sim）
- `/camera/image_raw/left` — 左目 RGB
- `/camera/image_raw/depth` — 深度图

### 坐标系
- 基坐标系: `panda_link0`
- 末端坐标系: `panda_hand`（法兰）/ 自定义 TCP
- 相机坐标系: `camera_color_optical_frame`（eye-in-hand 配置）
- TF 链: `panda_link0` → ... → `panda_hand` → `camera_link` → `camera_color_optical_frame`

### 状态机
- 状态: IDLE → MOVING / TEACHING → STOPPING → FAULT
- 任意状态可通过 EMERGENCY_STOP 进入 FAULT
- FAULT 通过 CLEAR_FAULT 恢复到 IDLE
- Jog 看门狗: 200ms 无命令自动停止

### 日志规范
所有日志输出必须使用 `robot_logger` 的 `LOG_*` 宏（C++）或 `robot_logger` 模块（Python）。
禁止使用 `std::cout`、`std::cerr`、`printf`、`RCLCPP_*` 直接输出日志。测试代码（test/）允许使用 `std::cout`。

### 速度复合
- 有效速度 = 模式速度（SetSpeed） × 全局速度比（SetSpeedRatio）

### 代码风格
- **C++**: Google Style Guide，类名 PascalCase，方法/变量 snake_case
- **Python**: PEP8，完整 Type Hints + 中文 Docstring
- 智能指针优先（禁止裸指针），Doxygen 注释公开头文件
- ROS2 回调组: 状态订阅用 MutuallyExclusiveCallbackGroup，发布用 ReentrantCallbackGroup
- 节点构造使用工厂模式（`create()` 静态方法 + 两阶段初始化）

### 分层架构
```
Layer 3: Python Binding (pybind11)           ← script/ 通过 C++ 后端控制
Layer 2: ROS 2 C++ Wrapper Nodes            ← rclcpp 节点（通信适配层）
Layer 1: Pure C++ Core Library (无 ROS 依赖) ← robot_kinematics / robot_motion / robot_vision_core
```

- Layer 1 通过 `MotionIOBridge` 抽象接口与通信解耦
- **依赖方向约束**: robot_controller → robot_vision 单向依赖，不可反向
- Python 脚本使用 `rclcpp`（通过 pybind11），禁止同时使用 `rclpy`
- pybind11 绑定中所有阻塞方法必须释放 GIL
- Eigen 类型在 Python 侧转换为原生 list/tuple，不使用 numpy

### AI 协作规则
在修改任何包之前，必须先读取该包目录下的 CLAUDE.md。
步骤：
1. 确认涉及哪个/哪些包
2. 读取对应包的 CLAUDE.md
3. 再开始分析或修改代码
4. 大规模任务（如批量生成/修改多个文件）先列出计划，等用户确认后再执行

### 各包详细文档
- [robot_msgs](src/robot_msgs/CLAUDE.md)
- [robot_logger](src/robot_logger/CLAUDE.md)
- [robot_description](src/robot_description/CLAUDE.md)
- [robot_bringup](src/robot_bringup/CLAUDE.md)
- [robot_controller](src/robot_controller/CLAUDE.md)
- [robot_vision](src/robot_vision/CLAUDE.md)
- [robot_hmi](src/robot_hmi/CLAUDE.md)
- [robot_api_python](src/robot_api_python/CLAUDE.md)
