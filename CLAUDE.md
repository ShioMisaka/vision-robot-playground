# Claude Code 项目指南

## 项目概述
基于 ROS2 + Isaac Sim 的 Franka Panda 7 轴机械臂视觉引导抓取系统。
使用 ZEN_X_Mini 双目深度相机的左目 + 深度通道进行目标检测与 3D 定位。

## 技术栈
- **语言**: Python 3.10+
- **ROS2**: Humble
- **仿真**: NVIDIA Isaac Sim（通过 ROS2 bridge 通信）
- **机器人**: Franka Panda 7-DOF + 二指夹爪
- **相机**: ZEN_X_Mini（左目 RGB + 深度）
- **核心依赖**: rclpy, ikpy, scipy, opencv, cv_bridge, message_filters

## 项目结构
```
src/
  config.py            # 常量：话题名、关节名、TF Frame、预设姿态
  ik_solver.py         # IK/FK 求解器（ikpy，基于 URDF）
  vision.py            # 纯 CV 工具：HSV 颜色检测器 ColorDetector
  robot.py             # RobotController 节点：关节控制、IK 位姿、TF2
  vision_processor.py  # VisionProcessor 节点：左目+深度同步、检测桩代码
  task_manager.py      # GraspTaskManager：抓取状态机（检测→居中→抓取→提起）
script/
  test_move.py         # 演示：IK 位姿控制
  test_vision.py       # 演示：视觉伺服引导抓取（红色物块）
  test_joint_state.py  # 演示：关节状态读取
urdf/
  panda.urdf           # Franka Panda URDF 模型
```

## 编码规范
- 遵循 PEP8，完整 Type Hints + 中文 Docstring
- 所有 ROS2 节点通过 MultiThreadedExecutor 运行，禁止使用 `rclpy.spin_once`
- 线程同步使用 `threading.Event`（就绪等待）和 `threading.Lock`（共享数据保护）
- 回调组划分：状态订阅用 `MutuallyExclusiveCallbackGroup`，发布/TF 用 `ReentrantCallbackGroup`

## ROS2 话题
| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/joint_command` | sensor_msgs/JointState | Pub | 关节指令（9值：7臂+2爪） |
| `/joint_states` | sensor_msgs/JointState | Sub | Isaac Sim 关节反馈 |
| `/camera/image_raw/left` | sensor_msgs/Image | Sub | 左目 RGB |
| `/camera/image_raw/depth` | sensor_msgs/Image | Sub | 深度图（uint16 mm 或 float32 m） |

## 常用命令
```bash
# 运行位姿控制演示
python3 script/test_move.py

# 运行视觉引导抓取演示
python3 script/test_vision.py

# 查看 ROS2 话题
ros2 topic list

# 查看话题数据
ros2 topic echo /joint_states
```

## 架构约定
- RobotController 不依赖 VisionProcessor，二者通过 GraspTaskManager 编排
- VisionProcessor.process_image() 是桩代码，子类重写以接入 YOLO/GraspNet
- 视觉伺服参数（DIRECTION_X/Y, MOVE_STEP, DESCEND_DISTANCE）定义在 script 中，非节点级参数
- TF2: RobotController 自动发布 panda_link0 → panda_hand 变换，Buffer 支持查询相机→基座变换
