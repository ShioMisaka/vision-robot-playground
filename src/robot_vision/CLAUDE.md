# robot_vision

## 职责
提供视觉处理能力和相机配置管理。分为两层：纯 C++ 视觉核心库（零 ROS 依赖）
和 ROS2 视觉节点层。当前实现基于 OpenCV HSV 颜色检测 + 深度图 3D 定位，
`CameraInterface::process_image()` 为桩代码接口，可子类化接入 YOLO/GraspNet 等算法。
VisionProcessorNode 启动时自动发布相机静态 TF（mount_frame → camera_frame → optical_frame）。

## 节点清单
| 节点 | 源文件 | 功能 |
|------|--------|------|
| VisionProcessorNode | `src/nodes/vision_processor_node.cpp` | 同步 RGB+Depth 订阅，调用检测器，发布结果 |
| _(无独立可执行节点)_ | | 通过 robot_api_python 或 robot_demos 内嵌启动 |

## CMake Target 分层

```
robot_vision_core (共享库)   ← 颜色检测 + 视觉接口（零 ROS 依赖，仅 OpenCV + Eigen）
       ▲
robot_vision_nodes (共享库)  ← ROS2 视觉节点（依赖 vision_core）
```

## 话题 / 服务 / Action 接口

### 话题（VisionProcessorNode）

| 名称 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/camera/image_raw/left` | sensor_msgs/Image | Sub | 左目 RGB（message_filters 同步） |
| `/camera/image_raw/depth` | sensor_msgs/Image | Sub | 深度图（message_filters 同步） |

无 Service / Action。视觉结果通过 `IVisionProcessor` 接口（`get_latest_result()` / `wait_for_detection()` / `average_detections()`）供外部调用。

## 核心类与修改入口

### vision 层（robot_vision_core target）

| 文件 | 类 | 职责 |
|------|-----|------|
| `include/.../vision/i_vision_processor.hpp` | IVisionProcessor | 视觉处理抽象接口（DetectionResult 结构体：detected, xyz, uv, confidence, label） |
| `include/.../vision/camera_interface.hpp` | CameraInterface | 图像处理基类（`pixel_to_3d()` 针孔相机投影） |
| `include/.../vision/color_detector.hpp` | ColorDetector | HSV 颜色检测器（继承 CameraInterface） |
| `include/.../vision/vision_topic_config.hpp` | VisionTopicConfig | 视觉节点相机话题配置（独立于 robot_controller::TopicConfig） |
| `include/.../vision/camera_config.hpp` | CameraConfig, CameraIntrinsicsConfig, CameraExtrinsicsConfig, CameraOpticalFrameConfig | 运行时相机配置结构体 |
| `include/.../vision/camera_config_loader.hpp` | CameraConfigLoader | 从 YAML 加载相机配置（`config/zed_x_mini_camera.yaml`） |
| `src/vision/camera_config_loader.cpp` | CameraConfigLoader::load() | yaml-cpp 解析 + 校验 |
| `src/vision/color_detector.cpp` | ColorDetector::detect(), process_image() | BGR→HSV + 轮廓面积筛选 + 深度 3D 投影 |

### nodes 层（robot_vision_nodes target）

| 文件 | 类 | 职责 |
|------|-----|------|
| `include/.../nodes/vision_processor_node.hpp` | VisionProcessorNode | ROS2 视觉节点（ApproximateTime 同步 + 线程安全结果缓存 + 相机静态 TF 发布） |
| `src/nodes/vision_processor_node.cpp` | create(), on_synced_image(), publish_camera_tf() | 工厂创建 + cv_bridge 转换 + 检测结果通知 + TF 发布 |

## ColorDetector 相机内参

| 参数 | 默认值 | 说明 |
|------|--------|------|
| fx, fy | 490.667 | 焦距（像素，ZED_X_Mini 默认） |
| cx, cy | 640.0, 360.0 | 光心（像素） |

通过构造函数或 `set_camera_intrinsics(fx, fy, cx, cy)` 修改。
推荐通过 `CameraConfigLoader` 从 `config/zed_x_mini_camera.yaml` 加载。

## 相机 TF 发布

VisionProcessorNode 初始化时根据 `CameraConfig` 自动发布两条静态 TF：
- `mount_frame → camera_frame`（由 extrinsics: offset_xyz + rpy 定义）
- `camera_frame → optical_frame`（由 optical_frame_rotation.pitch 定义）

当 `CameraConfig.mount_frame` 非空时启用，使用 `tf2_ros::StaticTransformBroadcaster`。

## 测试与演示

### 演示

所有演示（demo_camera、demo_vision_grasp、test_robot_node）已迁移至 `robot_demos` 包。
集成测试（test_camera_tf）已迁移至 `robot_tasks` 包。

## 启动方式
此包无可独立运行的节点。通过 `robot_api_python` 或 `robot_demos` 内嵌启动。

## 包内依赖
- **内部依赖**: robot_logger
- **外部依赖**: rclcpp, sensor_msgs, geometry_msgs, cv_bridge, message_filters, tf2_ros, Eigen3, OpenCV, yaml-cpp

注：VisionProcessorNode 已解耦，不再依赖 `robot_controller::TopicConfig` 和 `ControlConstants`，改用自有的 `VisionTopicConfig`。GraspTaskManager 已迁移至 `robot_tasks` 包。

## 修改指南
- **替换检测算法** → 创建 `CameraInterface` 子类，实现 `process_image()`，参考 `ColorDetector` 写法
- **修改 HSV 颜色参数** → 修改创建 `ColorDetector` 时的 `lower_hsv` / `upper_hsv` 参数（在 script/ 中）
- **修改相机内参** → 调用 `ColorDetector::set_camera_intrinsics()`
- **修改视觉话题配置** → 修改 `VisionTopicConfig` 参数（camera_left, camera_depth, sync_queue_size, sync_max_slop）
- **修改相机参数** → 编辑 `robot_description/config/zed_x_mini_camera.yaml`
- **新增视觉节点** → 参考 `VisionProcessorNode::create()` 工厂模式
- **修改图像同步策略** → 编辑 `src/nodes/vision_processor_node.cpp` 的 `init()` 中 ApproximateTime 配置

## 注意事项
- **命名空间**: 此包所有类使用 `namespace robot_vision`
- `process_image()` 为虚函数，ColorDetector 是一个实现示例，生产环境应替换为 YOLO/GraspNet
- 深度图支持两种格式：CV_16U（单位 mm）和 CV_32F（单位 m），`process_image()` 自动检测
- `VisionProcessorNode` 使用 `message_filters::ApproximateTime` 同步 RGB 和深度图，队列大小 10，最大时间差 0.1s
