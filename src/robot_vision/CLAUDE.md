# robot_vision

## 职责
提供视觉处理能力与视觉引导抓取任务管理。分为两层：纯 C++ 视觉核心库（零 ROS 依赖）
和 ROS2 视觉节点层。当前实现基于 OpenCV HSV 颜色检测 + 深度图 3D 定位，
`CameraInterface::process_image()` 为桩代码接口，可子类化接入 YOLO/GraspNet 等算法。

## 节点清单
| 节点 | 源文件 | 功能 |
|------|--------|------|
| VisionProcessorNode | `src/nodes/vision_processor_node.cpp` | 同步 RGB+Depth 订阅，调用检测器，发布结果 |
| _(无独立可执行节点)_ | | 通过 robot_api_python 内嵌启动 |

## CMake Target 分层

```
robot_vision_core (共享库)   ← 颜色检测 + 视觉接口（零 ROS 依赖，仅 OpenCV + Eigen）
       ▲
robot_vision_nodes (共享库)  ← ROS2 视觉节点 + 抓取任务管理器（依赖 vision_core + robot_motion）
```

## 话题 / 服务 / Action 接口

### 话题（VisionProcessorNode）

| 名称 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/camera/image_raw/left` | sensor_msgs/Image | Sub | 左目 RGB（message_filters 同步） |
| `/camera/image_raw/depth` | sensor_msgs/Image | Sub | 深度图（message_filters 同步） |

无 Service / Action。视觉结果通过 `IVisionProcessor` 接口（`get_latest_result()` / `wait_for_detection()`）供 `GraspTaskManager` 调用。

## 核心类与修改入口

### vision 层（robot_vision_core target）

| 文件 | 类 | 职责 |
|------|-----|------|
| `include/.../vision/i_vision_processor.hpp` | IVisionProcessor | 视觉处理抽象接口（DetectionResult 结构体） |
| `include/.../vision/camera_interface.hpp` | CameraInterface | 图像处理基类（`pixel_to_3d()` 针孔相机投影） |
| `include/.../vision/color_detector.hpp` | ColorDetector | HSV 颜色检测器（继承 CameraInterface） |
| `src/vision/color_detector.cpp` | ColorDetector::detect(), process_image() | BGR→HSV + 轮廓面积筛选 + 深度 3D 投影 |

### nodes 层（robot_vision_nodes target）

| 文件 | 类 | 职责 |
|------|-----|------|
| `include/.../nodes/vision_processor_node.hpp` | VisionProcessorNode | ROS2 视觉节点（ApproximateTime 同步 + 线程安全结果缓存） |
| `src/nodes/vision_processor_node.cpp` | create(), on_synced_image() | 工厂创建 + cv_bridge 转换 + 检测结果通知 |
| `include/.../nodes/grasp_task_manager.hpp` | GraspTaskManager | 抓取状态机（8 状态：IDLE→DETECTING→APPROACHING→...→DONE） |
| `src/nodes/grasp_task_manager.cpp` | run(), step_*() | 阻塞式抓取流程 + TF 坐标变换 |

### GraspTaskManager 状态机

```
kIdle → kDetecting → kApproaching → kDescending → kGrasping → kLifting → kDone
                                        ↓ (任何阶段异常)
                                      kError
```

| 状态 | 动作 |
|------|------|
| kDetecting | 调用 vision->get_latest_result()，转换到 base 坐标系 |
| kApproaching | move_to_pose() 到目标上方 approach_height（默认 0.15m） |
| kDescending | move_to_pose() 到抓取高度（grasp_height_offset 默认 0.02m） |
| kGrasping | close_gripper() |
| kLifting | move_linear() 抬起 approach_height + 0.1m |

## 关键参数（GraspTaskManager 构造函数）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| base_frame | "panda_link0" | 基坐标系 |
| camera_frame | "camera_color_optical_frame" | 相机光学坐标系 |
| approach_height | 0.15 m | 接近阶段高度偏移 |
| grasp_height_offset | 0.02 m | 抓取高度偏移 |
| grasp_rpy | [π, 0, π] | 抓取朝向 |

## ColorDetector 相机内参

| 参数 | 默认值 | 说明 |
|------|--------|------|
| fx, fy | 614 | 焦距（像素） |
| cx, cy | 320, 240 | 光心（像素） |

通过 `set_camera_intrinsics(fx, fy, cx, cy)` 修改。

## 测试与演示

### 集成测试（需 Isaac Sim）

| 测试文件 | 覆盖范围 |
|---------|---------|
| test/test_robot_node.cpp | 11 项测试：关节控制、夹爪、Home、IK、TCP、TF |
| test/test_camera_tf.cpp | 相机 TF 链验证、坐标变换精度（< 1mm） |

### 演示

| 演示文件 | 功能 |
|---------|------|
| demo/demo_camera.cpp | 显示同步 RGB + 深度图（JET colormap） |

## 启动方式
此包无可独立运行的节点。通过 `robot_api_python` 或 `robot_hmi` 内嵌启动。

## 包内依赖
- **内部依赖**: robot_controller（使用 IRobotController, RobotControllerNode, TopicConfig, ControlConstants）
- **外部依赖**: rclcpp, sensor_msgs, geometry_msgs, cv_bridge, message_filters, image_transport, robot_msgs, eigen

## 修改指南
- **替换检测算法** → 创建 `CameraInterface` 子类，实现 `process_image()`，参考 `ColorDetector` 写法
- **修改 HSV 颜色参数** → 修改创建 `ColorDetector` 时的 `lower_hsv` / `upper_hsv` 参数（在 script/ 中）
- **修改抓取流程** → 编辑 `src/nodes/grasp_task_manager.cpp` 的 `step_*()` 方法
- **修改抓取参数** → 修改 `GraspTaskManager` 构造参数（在 script/ 中）
- **修改相机内参** → 调用 `ColorDetector::set_camera_intrinsics()`
- **新增视觉节点** → 参考 `VisionProcessorNode::create()` 工厂模式
- **修改图像同步策略** → 编辑 `src/nodes/vision_processor_node.cpp` 的 `init()` 中 ApproximateTime 配置

## 注意事项
- **依赖方向**: robot_vision → robot_controller 单向依赖，不可反向（避免循环依赖）
- `process_image()` 为虚函数，ColorDetector 是一个实现示例，生产环境应替换为 YOLO/GraspNet
- `GraspTaskManager::run()` 是阻塞调用，需在独立线程中执行
- 深度图支持两种格式：CV_16U（单位 mm）和 CV_32F（单位 m），`process_image()` 自动检测
- `VisionProcessorNode` 使用 `message_filters::ApproximateTime` 同步 RGB 和深度图，队列大小 10，最大时间差 0.1s
