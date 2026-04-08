# S 曲线运动控制接口设计

## 概述

为 `RobotMotionController` 新增 `moveJ`（关节空间）和 `moveL`（笛卡尔空间）运动接口，使用七段式 S 曲线（Jerk 连续）轨迹规划替代手动传入插值参数。速度通过百分比（1-100%）描述，moveJ 和 moveL 各自独立保存速度设置，百分比相对于 `RobotProfile` 中定义的运动极限按比例缩放。

## 需求

1. 新增 `moveJ(target_angles, block)` — 关节空间 S 曲线运动
2. 新增 `moveL(xyz, rpy, finger, block)` — 笛卡尔空间直线 S 曲线运动
3. 新增 `set_speed(mode, percent)` / `get_speed(mode)` — 速度设置（moveJ/movel 独立）
4. 七段式 S 曲线轨迹规划算法（Jerk 连续）
5. 旧接口（`set_arm`, `move_to_pose`, `move_linear`）保留不动
6. 速度百分比（1-100%）同时按比例缩放 max_vel / max_acc / max_jerk

## 架构

### 新增 TrajectoryPlanner 类（Layer 1，无 ROS 依赖）

文件位置：
- `src/robot_control_cpp/include/robot_control_cpp/trajectory_planner.hpp`
- `src/robot_control_cpp/src/trajectory_planner.cpp`

```
SCurvePlanner（单轴七段式 S 曲线）
  - plan(q0, q1, cfg, dt) → vector<TrajectoryPoint>

TrajectoryPlanner（多轴同步）
  - plan_joint(q_start, q_end, axis_configs, dt) → vector<vector<double>>
```

**多轴同步策略：** 对每个轴独立规划，取最长运动时间 `T_max`，对短轴用 `T_max` 作为总时间重新规划（降低 max_vel 使轨迹适配 `T_max`）。重新规划保持七段式结构，保证 Jerk 连续性。

### 速度模型

```cpp
struct MotionLimits {
  double max_vel;   // moveJ: rad/s, moveL: m/s
  double max_acc;   // moveJ: rad/s², moveL: m/s²
  double max_jerk;  // moveJ: rad/s³, moveL: m/s³
};
```

`RobotProfile` 新增：
- `MotionLimits joint_limits` — 关节空间运动极限
- `MotionLimits cartesian_limits` — 笛卡尔空间运动极限

百分比映射：
```
actual_max_vel  = percent/100 * limits.max_vel
actual_max_acc  = percent/100 * limits.max_acc
actual_max_jerk = percent/100 * limits.max_jerk
```

**范围验证：** `set_speed` 接受 1-100，传入 0 抛出 `std::invalid_argument`，传入 >100 被钳位到 100。新速度立即生效，影响下一次 moveJ/moveL 调用。

### 速度模式枚举

```cpp
enum class MotionMode { kMoveJ, kMoveL };
```

接口使用 `MotionMode` 而非裸字符串，与现有 `GraspState` 枚举风格一致。Python 绑定中导出为枚举值 `MotionMode.MOVE_J` / `MotionMode.MOVE_L`。

### 轨迹时间步长

`dt` 定义在 `ControlConstants` 中：

```cpp
static constexpr double kTrajectoryDt = 0.01;  // 100 Hz 轨迹发布频率
```

所有 moveJ/moveL 调用共用此常量。选择 100 Hz 是 Isaac Sim 典型控制频率。

### 接口设计

```cpp
// IRobotController 新增纯虚方法
virtual void moveJ(const std::vector<double>& target_angles, bool block = true) = 0;
virtual void moveL(const std::array<double, 3>& xyz,
                   const std::optional<std::array<double, 3>>& rpy = std::nullopt,
                   double finger = -1.0, bool block = true) = 0;
virtual void set_speed(MotionMode mode, double percent) = 0;
virtual double get_speed(MotionMode mode) const = 0;
```

### 夹爪状态处理

moveJ 和 moveL 均遵循现有 `grasping_` 标志逻辑：
- **moveJ**：无 `finger` 参数，始终按 `grasping_` 状态决定夹爪行为（抓取时保持 `gripper_.min_width`，非抓取时保持当前夹爪宽度）
- **moveL**：`finger` 参数保持 `finger < 0` 表示"跟随 grasping_ 状态"的约定，与现有 `move_to_pose` 一致

### moveJ 行为

1. 获取当前关节角度 `q_start`
2. 根据 `movej_speed_` 百分比和 `profile_.joint_limits` 计算实际运动极限
3. 对每个轴调用 `SCurvePlanner::plan()` 独立规划
4. 多轴同步（取最长轴时间，其他轴用该时间重新规划）
5. 按 `kTrajectoryDt` 逐步 `publish_command`，每步 `sleep_for(dt)`
6. 夹爪按 `grasping_` 状态处理（同 `set_arm` 逻辑）
7. 如果 `block=true`，结束后 `wait_for_motion`

### moveL 行为

1. 获取当前 TCP 位姿作为起点
2. 计算笛卡尔空间总距离，根据 `movel_speed_` 和 `cartesian_limits` 确定总运动时间 `T`
3. 按 `kTrajectoryDt` 在笛卡尔空间采样中间点：
   - 位置：线性插值（直线）
   - 姿态：RPY 转四元数后球面线性插值（slerp），再转回 RPY
4. 每个中间点 IK 求解得到关节角度序列
5. 对关节角度序列应用 S 曲线速度规划（平滑关节空间速度，保持笛卡尔直线路径的关节跟踪平滑）
6. 按 `kTrajectoryDt` 逐步 `publish_command`
7. 如果 `block=true`，结束后 `wait_for_motion`

**IK 失败处理：** 如果中间点 IK 失败，抛出 `std::runtime_error` 提前中止，机器人停在最后一个成功的位置。

### 七段式 S 曲线算法

经典七段式时间最优轨迹规划，7 个阶段：

1. 加加速段（jerk = +j_max）
2. 匀加速段（acc = +a_max）
3. 减加速段（jerk = -j_max）
4. 匀速段（vel = v_max）
5. 减加速段（jerk = -j_max，减速）
6. 匀减速段（acc = -a_max）
7. 加减速段（jerk = +j_max，减速到 0）

需要处理的退化情况：
- 最大速度不可达（无匀速段）
- 最大加速度不可达（无匀加速/匀减速段）
- 极短距离（仅加加速/减加速段）

## 文件影响

| 文件 | 改动 |
|------|------|
| `src/robot_control_cpp/include/robot_control_cpp/robot_profile.hpp` | 新增 `MotionLimits` 结构体和字段 |
| `src/robot_control_cpp/include/robot_control_cpp/panda_profile.hpp` | 填充 Panda 的运动极限参数 |
| `src/robot_control_cpp/include/robot_control_cpp/trajectory_planner.hpp` | **新增**，SCurvePlanner + TrajectoryPlanner 声明 |
| `src/robot_control_cpp/src/trajectory_planner.cpp` | **新增**，七段式算法实现 |
| `src/robot_control_cpp/include/robot_control_cpp/i_robot_controller.hpp` | 新增 moveJ/moveL/set_speed/get_speed，新增 MotionMode 枚举 |
| `src/robot_control_cpp/include/robot_control_cpp/robot_motion_controller.hpp` | 新增方法声明 + 速度状态成员 |
| `src/robot_control_cpp/src/robot_motion_controller.cpp` | 新增方法实现 |
| `src/robot_control_cpp/include/robot_control_cpp/config.hpp` | 新增 `kTrajectoryDt` 常量 |
| `src/robot_control_cpp_py/src/bindings.cpp` | 绑定新方法、MotionLimits、MotionMode 枚举 |
| `src/robot_control_cpp/CMakeLists.txt` | 添加 trajectory_planner.cpp |

## 不包含

- 不修改/删除旧接口（set_arm, move_to_pose, move_linear）
- 不实现碰撞检测或路径规划
- 不修改 GraspTaskManager（后续可迁移到 moveJ/moveL）
- 不实现加速度/加加速度的独立设置接口（统一跟随百分比缩放）
