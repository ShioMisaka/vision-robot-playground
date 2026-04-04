# Franka Vision Grasp

基于 ROS2 + Isaac Sim 的 Franka Panda 机械臂视觉引导抓取系统。

## 功能

- **机械臂控制**: 关节角度指令、IK 位姿控制、相对平移/旋转
- **视觉处理**: 双目深度相机左目+深度同步订阅，HSV 颜色目标检测
- **视觉伺服抓取**: 图像反馈闭环 — 检测 → 居中 → 下探 → 夹取 → 提起
- **TF2 集成**: 自动发布末端执行器 TF，支持坐标系变换查询

## 环境要求

- Python 3.10+
- ROS2 Humble
- NVIDIA Isaac Sim（提供 `/joint_states` 和相机话题）
- ZEN_X_Mini 双目深度相机

## 依赖安装

```bash
pip install ikpy scipy opencv-python cv-bridge
```

## 项目结构

```
src/
  config.py            # 话题名、关节名、TF Frame、预设姿态
  ik_solver.py         # IK/FK 求解器（基于 URDF + ikpy）
  vision.py            # HSV 颜色检测器
  robot.py             # 机械臂控制节点
  vision_processor.py  # 相机同步节点（左目 + 深度）
  task_manager.py      # 抓取任务状态机
script/
  test_move.py         # 位姿控制演示
  test_vision.py       # 视觉引导抓取演示（红色物块）
urdf/
  panda.urdf           # Franka Panda URDF
```

## 快速开始

### 1. 启动 Isaac Sim

在 Isaac Sim 中加载 Franka Panda 场景，确保 ROS2 bridge 已启动，以下话题可用：

```
/joint_states          # 关节状态反馈
/joint_command         # 关节指令（订阅）
/camera/image_raw/left # 左目相机
/camera/image_raw/depth# 深度图
```

### 2. 运行位姿控制演示

```bash
python3 script/test_move.py
```

机械臂将依次执行：张开夹爪 → 移动到指定位姿 → 闭合夹爪 → 平移 → 旋转关节。

### 3. 运行视觉引导抓取

```bash
python3 script/test_vision.py
```

流程：
1. 机械臂移动到观察位（俯视桌面）
2. 检测红色物块，连续锁定 15 帧后确认
3. 视觉伺服居中：根据像素偏移调整机械臂 XY，使目标移到画面中心
4. 居中后下探 → 闭合夹爪 → 提起

按 `q` 键可随时退出。

## 视觉伺服参数调参

`script/test_vision.py` 顶部的常量控制伺服行为：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `DIRECTION_X` | -1.0 | 图像右偏时机器人 X 移动方向（-1 或 1） |
| `DIRECTION_Y` | 1.0 | 图像下偏时机器人 Y 移动方向（-1 或 1） |
| `MOVE_STEP` | 0.02 | 每次居中调整步长（米） |
| `CENTER_THRESHOLD` | 0.03 | 居中判定阈值（归一化偏移） |
| `DESCEND_DISTANCE` | 0.15 | 下探距离（米） |
| `LIFT_DISTANCE` | 0.25 | 提起距离（米） |

## 扩展指南

### 接入自定义检测网络

继承 `VisionProcessor`，重写 `process_image()` 方法：

```python
from src.vision_processor import VisionProcessor

class MyDetector(VisionProcessor):
    def process_image(self, rgb_image, depth_image):
        # 你的检测逻辑
        results = my_model(rgb_image)
        return {
            "detected": True,
            "xyz": [x, y, z],
            "uv": [u, v],
            "confidence": 0.95,
            "grasp_pose": None,
        }
```

### 使用 GraspTaskManager

```python
from src.task_manager import GraspTaskManager

task_mgr = GraspTaskManager(robot, vision, approach_height=0.15)
success = task_mgr.run(timeout=30.0)
```

## ROS2 话题一览

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/joint_command` | sensor_msgs/JointState | 发布 | 9 值：7 臂关节 + 2 夹爪 |
| `/joint_states` | sensor_msgs/JointState | 订阅 | Isaac Sim 关节反馈 |
| `/camera/image_raw/left` | sensor_msgs/Image | 订阅 | 左目 RGB 图像 |
| `/camera/image_raw/depth` | sensor_msgs/Image | 订阅 | 深度图 |

## 架构说明

所有 ROS2 节点通过 `MultiThreadedExecutor` 运行，回调在后台线程处理，主线程执行阻塞式业务逻辑，完全避免 `spin_once` 滥用。

```
                    ┌──────────────┐
                    │ GraspTask    │  状态机编排
                    │ Manager      │
                    └──┬───────┬───┘
                       │       │
              ┌────────▼┐  ┌──▼───────────┐
              │  Robot  │  │   Vision     │
              │Controller│  │  Processor   │
              └───┬─────┘  └──┬───────────┘
                  │           │
           ┌──────▼──────┐  ┌─▼──────────────┐
           │ /joint_     │  │ ApproximateTime │
           │ command     │  │ Synchronizer    │
           │ /joint_     │  │ (left + depth)  │
           │ states      │  └────────────────┘
           └─────────────┘
```
