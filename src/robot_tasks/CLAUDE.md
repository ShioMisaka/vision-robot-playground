# robot_tasks

## 职责
视觉+运动联合任务编排层。提供 GraspTaskManager 抓取状态机等高级任务，
依赖 robot_controller::robot_client（Action/Lease 客户端）+ robot_vision（视觉处理），
不直接依赖 robot_controller 内部库（通过 Action 调用）。

## 节点清单
| 节点 | 可执行文件 | 功能 |
|------|-----------|------|
| robot_tasks_node | `ros2 run robot_tasks robot_tasks_node` | GraspTask Action Server 节点 |

## CMake Target

```
robot_tasks_lib (共享库)        ← GraspTaskManager（依赖 robot_controller::robot_client + robot_vision_core）
robot_tasks_node (可执行文件)   ← GraspTaskNode（依赖 robot_tasks_lib）
```

集成测试 `test_camera_tf` 需要 Isaac Sim 运行中，链接 robot_controller::robot_nodes。

## 话题 / 服务 / Action 接口

### Action（robot_tasks_node 提供）

| Action | 类型 | 说明 |
|--------|------|------|
| `~/grasp_task` | GraspTask | 视觉引导抓取任务（需 session_id，带状态反馈） |

### GraspState 状态机

```
kIdle → kDetecting → kApproaching → kDescending → kGrasping → kLifting → kDone
                    ↘ kReDetecting (near-distance refine)
任意状态 → kError
```

### GraspTaskManager 构造参数

| 参数 | 说明 |
|------|------|
| ctrl | IRobotController 接口（来自 robot_controller::robot_client） |
| vision | IVisionProcessor 接口（来自 robot_vision） |
| base_frame, optical_frame | TF 坐标系名称 |
| approach_height | 接近高度（m） |
| grasp_height_offset | 抓取高度补偿（m） |
| grasp_rpy | 抓取姿态 [roll, pitch, yaw] |
| camera_offset, camera_rpy | 相机外参 |
| optical_frame_pitch | 光学帧旋转角度 |
| max_reach, approach params | 运动约束参数 |

### GraspTaskNode

| 文件 | 类 | 职责 |
|------|-----|------|
| `include/robot_tasks/grasp_task_node.hpp` | GraspTaskNode | GraspTask Action Server 节点 |
| `src/grasp_task_node.cpp` | handle_grasp() | Action 回调实现 |
| `src/grasp_task_main.cpp` | main() | 可执行入口 |

## 测试

### 集成测试（需要 Isaac Sim）
| 测试文件 | 覆盖范围 |
|---------|---------|
| test/test_camera_tf.cpp | 相机 TF 链验证（从 robot_vision 迁移） |

## 启动方式
```bash
# 运行 GraspTask Action Server 节点（需要 robot_controller_node + robot_vision_node）
ros2 run robot_tasks robot_tasks_node
```

## 包内依赖
- **内部依赖**: robot_controller (robot_client target), robot_vision, robot_logger
- **外部依赖**: rclcpp, Eigen3
- **测试依赖**: robot_controller（仅 test_camera_tf 需要链接 robot_nodes）

## 修改指南
- **修改抓取流程** → 编辑 `src/grasp_task_manager.cpp` 的状态机步骤
- **修改 GraspTask Action** → 编辑 `src/grasp_task_node.cpp` 的 Action 回调
- **新增任务类型** → 在 `include/robot_tasks/` 下创建新类，遵循 IRobotController + IVisionProcessor 接口
- **修改抓取参数** → 修改构造 GraspTaskManager 时的参数（在调用方）

## 注意事项
- **命名空间**: 此包所有类使用 `namespace robot_tasks`
- **通过 Action 控制**: GraspTaskManager 通过 IRobotController 接口（ActionRobotController）控制机器人，不直接链接 robot_controller 内部库
- **transform_to_base**: 内部手动计算坐标变换（base ← hand ← camera ← optical），optical_frame_pitch 参数控制光学帧旋转
