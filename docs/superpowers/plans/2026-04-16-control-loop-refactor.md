# 100Hz Control Loop Refactor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Refactor robot_controller to use a single 100Hz Read→Plan→Monitor→Write control loop, eliminating action execution threads and decoupling Action servers from direct command publishing.

**Architecture:** Replace the current multi-threaded trajectory execution model (TrajectoryExecutor thread + separate jog timer) with a unified 100Hz timer-driven control loop in RobotControllerNode. Introduce RobotStateModel (thread-safe state), SetpointGenerator (tick-based trajectory provider), and closed-loop monitoring (following error + arrival detection). Action servers become thin: validate → plan → submit → done. The 100Hz loop handles all publishing, feedback, and result signaling.

**Tech Stack:** C++17, rclcpp (Jazzy), Eigen3, Orocos KDL, std::shared_mutex

---

## Current Architecture (Problems)

```
Python moveJ() → RobotMotionController → while+sleep_until loop → bridge->publish()
Action MoveJ   → action thread → TrajectoryExecutor (own thread) → while polling → bridge->publish()
Jog            → 50Hz timer → JogController::tick() → bridge->publish()
```

**Issues:**
1. Multiple threads publish to `/joint_command` concurrently (no coordination)
2. Action handlers contain while-loops mixing business logic with ROS2 feedback
3. No closed-loop monitoring (no following error detection)
4. TrajectoryExecutor runs in its own thread with sleep_until — duplicates what a timer could do
5. No unified state model (target/actual joints scattered across bridge and controller)

## Target Architecture

```
                        ┌─────────────────────────┐
  Python API ──────────►│                         │
  Action Server ───────►│   SetpointGenerator     │◄── 100Hz Control Loop
  JogController ───────►│   (trajectory queue)    │    (Read→Plan→Monitor→Write)
                        └──────────┬──────────────┘
                                   │ target_joints
                                   ▼
                        ┌─────────────────────────┐
                        │   RobotStateModel        │ ◄── joint_states callback
                        │   (target + actual)      │      (actual_joints)
                        │   shared_mutex protected │
                        └──────────┬──────────────┘
                                   │
                                   ▼
                        ┌─────────────────────────┐
                        │   Monitor               │
                        │   • following error      │
                        │   • arrival detection    │
                        │   • timeout              │
                        └──────────┬──────────────┘
                                   │
                                   ▼
                        ┌─────────────────────────┐
                        │   Write                 │
                        │   bridge->publish_command│
                        └─────────────────────────┘
```

## File Changes Summary

| File | Action | Responsibility |
|------|--------|---------------|
| `include/robot_controller/nodes/robot_state_model.hpp` | **Create** | Thread-safe target/actual state with shared_mutex |
| `include/robot_controller/nodes/setpoint_generator.hpp` | **Create** | Tick-based trajectory provider (replaces TrajectoryExecutor threading) |
| `src/nodes/setpoint_generator.cpp` | **Create** | Implementation |
| `include/robot_controller/motion/control_constants.hpp` | **Modify** | Add monitoring constants |
| `include/robot_controller/motion/motion_io_bridge.hpp` | **Modify** | Add trajectory submission interface |
| `src/motion/robot_motion_controller.cpp` | **Modify** | moveJ_internal/moveL use submit_trajectory |
| `include/robot_controller/nodes/robot_controller_node.hpp` | **Modify** | Add 100Hz timer, state model, active action tracking; remove action threads |
| `src/nodes/robot_controller_node.cpp` | **Modify** | Major: control loop, refactored actions, jog integration |
| `include/robot_controller/nodes/robot_state.hpp` | **Modify** | Add kStopping→kFault transition |
| `src/nodes/robot_state.cpp` | **Modify** | Update transition table |
| `CMakeLists.txt` | **Modify** | Replace trajectory_executor with setpoint_generator |

---

## Task 1: Create RobotStateModel

**Files:**
- Create: `src/robot_controller/include/robot_controller/nodes/robot_state_model.hpp`

**Purpose:** Thread-safe container for target/actual joint and gripper state, using `std::shared_mutex` for read-heavy workload.

- [ ] **Step 1: Write the header**

```cpp
// include/robot_controller/nodes/robot_state_model.hpp
#pragma once

#include <shared_mutex>
#include <vector>
#include <cmath>
#include <algorithm>

namespace robot_control {

/// @brief 线程安全的机器人状态数据模型（The Model）
/// 存储 target（指令发生器输出）和 actual（关节反馈）。
/// 使用 shared_mutex 允许多个读者并发读取（100Hz 读 + 10Hz 状态发布）。
class RobotStateModel {
public:
  /// @brief 构造
  /// @param dof 关节数（arm DOF）
  /// @param default_gripper 默认夹爪宽度
  explicit RobotStateModel(int dof, double default_gripper = 0.04)
      : target_joints_(dof, 0.0),
        actual_joints_(dof, 0.0),
        target_gripper_(default_gripper),
        actual_gripper_(default_gripper) {}

  // --- Actual (written by joint_states callback, read by control loop) ---

  /// @brief 更新实际关节反馈（由订阅回调调用）
  void update_actual(const std::vector<double>& joints, double gripper) {
    std::unique_lock lock(mutex_);
    actual_joints_ = joints;
    actual_gripper_ = gripper;
  }

  /// @brief 获取实际关节角度（共享读锁）
  std::vector<double> get_actual_joints() const {
    std::shared_lock lock(mutex_);
    return actual_joints_;
  }

  /// @brief 获取实际夹爪宽度
  double get_actual_gripper() const {
    std::shared_lock lock(mutex_);
    return actual_gripper_;
  }

  // --- Target (written by control loop, read by publish) ---

  /// @brief 更新目标关节指令（由控制循环调用）
  void update_target(const std::vector<double>& joints, double gripper) {
    std::unique_lock lock(mutex_);
    target_joints_ = joints;
    target_gripper_ = gripper;
  }

  /// @brief 获取目标关节角度
  std::vector<double> get_target_joints() const {
    std::shared_lock lock(mutex_);
    return target_joints_;
  }

  /// @brief 获取目标夹爪宽度
  double get_target_gripper() const {
    std::shared_lock lock(mutex_);
    return target_gripper_;
  }

  // --- Monitoring helpers ---

  /// @brief 计算最大跟踪误差（target vs actual 的最大单轴差值）
  /// @return 最大误差（rad）
  double max_following_error() const {
    std::shared_lock lock(mutex_);
    double max_err = 0.0;
    size_t n = std::min(target_joints_.size(), actual_joints_.size());
    for (size_t i = 0; i < n; ++i) {
      max_err = std::max(max_err, std::abs(target_joints_[i] - actual_joints_[i]));
    }
    return max_err;
  }

  /// @brief 检查到位（所有轴误差 < tolerance）
  bool is_on_target(double tolerance) const {
    std::shared_lock lock(mutex_);
    size_t n = std::min(target_joints_.size(), actual_joints_.size());
    for (size_t i = 0; i < n; ++i) {
      if (std::abs(target_joints_[i] - actual_joints_[i]) > tolerance) {
        return false;
      }
    }
    return true;
  }

  /// @brief 将 target 对齐到 actual（用于 CLEAR_FAULT 后恢复）
  void align_target_to_actual() {
    std::unique_lock lock(mutex_);
    target_joints_ = actual_joints_;
    target_gripper_ = actual_gripper_;
  }

  /// @brief 获取关节数
  size_t dof() const {
    std::shared_lock lock(mutex_);
    return target_joints_.size();
  }

private:
  mutable std::shared_mutex mutex_;
  std::vector<double> target_joints_;
  std::vector<double> actual_joints_;
  double target_gripper_;
  double actual_gripper_;
};

}  // namespace robot_control
```

- [ ] **Step 2: Verify compilation**

```bash
# 单独验证头文件语法
cd /home/cll/workspace/isaac_ros_project
cat > /tmp/test_state_model.cpp << 'EOF'
#include "robot_controller/nodes/robot_state_model.hpp"
int main() {
  robot_control::RobotStateModel m(7);
  m.update_actual({0,0,0,0,0,0,0}, 0.04);
  m.update_target({0.1,0,0,0,0,0,0}, 0.04);
  double err = m.max_following_error();
  bool on_target = m.is_on_target(0.01);
  m.align_target_to_actual();
  return 0;
}
EOF
```

---

## Task 2: Create SetpointGenerator

**Files:**
- Create: `src/robot_controller/include/robot_controller/nodes/setpoint_generator.hpp`
- Create: `src/robot_controller/src/nodes/setpoint_generator.cpp`

**Purpose:** Tick-based trajectory provider. Replaces `TrajectoryExecutor`'s threading model — the 100Hz loop calls `tick()` instead of the executor running its own thread.

- [ ] **Step 1: Write the header**

```cpp
// include/robot_controller/nodes/setpoint_generator.hpp
#pragma once

#include <atomic>
#include <chrono>
#include <mutex>
#include <vector>

namespace robot_control {

/// @brief 轨迹步骤（与原 TrajectoryExecutor 一致）
struct TrajectoryStep {
  std::vector<double> joint_positions;
  double time_from_start;  ///< seconds
};

/// @brief tick() 调用结果
struct SetpointResult {
  std::vector<double> joint_positions;  ///< 当前时刻的目标关节角
  double finger_width;                  ///< 夹爪宽度
  double progress;                      ///< 0.0 ~ 1.0
  double time_remaining;                ///< 预计剩余时间（秒）
  bool done;                            ///< 轨迹所有点已发送完毕
};

/// @brief Tick-based 轨迹指令发生器（零 ROS 依赖，零线程）
///
/// 使用模式:
///   1. start(trajectory, finger) — 提交预计算轨迹
///   2. 每次 100Hz tick 调用 tick() 获取当前目标
///   3. tick().done == true 表示轨迹播放完成
///   4. cancel() 立即标记为已完成（done=true）
class SetpointGenerator {
public:
  SetpointGenerator() = default;

  /// @brief 提交轨迹（覆盖当前轨迹）
  /// @param trajectory 预计算轨迹步骤
  /// @param finger_width 夹爪宽度
  void start(const std::vector<TrajectoryStep>& trajectory, double finger_width);

  /// @brief 取消当前轨迹（标记为 done）
  void cancel();

  /// @brief 获取当前时刻的指令目标
  /// @param now 当前时间点
  /// @return SetpointResult，done=true 表示轨迹播放完毕
  SetpointResult tick(std::chrono::steady_clock::time_point now);

  /// @brief 是否有活跃轨迹
  bool is_active() const { return active_.load(); }

  /// @brief 获取最终目标关节角（轨迹最后一个点）
  std::vector<double> final_target() const;

  /// @brief 获取夹爪宽度
  double finger_width() const;

  /// @brief 获取总时长
  double total_duration() const;

private:
  mutable std::mutex mutex_;
  std::vector<TrajectoryStep> trajectory_;
  double finger_width_ = 0.04;
  double total_duration_ = 0.0;
  std::chrono::steady_clock::time_point start_time_;
  std::atomic<bool> active_{false};
  std::atomic<bool> cancelled_{false};
};

}  // namespace robot_control
```

- [ ] **Step 2: Write the implementation**

```cpp
// src/nodes/setpoint_generator.cpp
#include "robot_controller/nodes/setpoint_generator.hpp"

#include <algorithm>

namespace robot_control {

void SetpointGenerator::start(
    const std::vector<TrajectoryStep>& trajectory, double finger_width) {
  std::lock_guard lock(mutex_);
  trajectory_ = trajectory;
  finger_width_ = finger_width;
  total_duration_ = trajectory.empty() ? 0.0 : trajectory.back().time_from_start;
  start_time_ = std::chrono::steady_clock::now();
  active_ = true;
  cancelled_ = false;
}

void SetpointGenerator::cancel() {
  cancelled_ = true;
  active_ = false;
}

SetpointGenerator::SetpointResult SetpointGenerator::tick(
    std::chrono::steady_clock::time_point now) {
  std::lock_guard lock(mutex_);

  SetpointResult result;
  result.finger_width = finger_width_;
  result.done = false;

  if (!active_ || cancelled_) {
    result.done = true;
    result.progress = cancelled_ ? 0.0 : 1.0;
    result.time_remaining = 0.0;
    if (!trajectory_.empty()) {
      result.joint_positions = trajectory_.back().joint_positions;
    }
    active_ = false;
    return result;
  }

  // 计算经过时间
  double elapsed = std::chrono::duration<double>(now - start_time_).count();

  // 二分查找当前时间对应的轨迹点
  size_t idx = 0;
  while (idx < trajectory_.size() - 1 &&
         trajectory_[idx + 1].time_from_start <= elapsed) {
    ++idx;
  }

  result.joint_positions = trajectory_[idx].joint_positions;
  result.progress = (total_duration_ > 1e-12)
                        ? std::clamp(elapsed / total_duration_, 0.0, 1.0)
                        : 1.0;
  result.time_remaining = std::max(0.0, total_duration_ - elapsed);

  // 检查轨迹是否播放完毕
  if (elapsed >= total_duration_) {
    result.joint_positions = trajectory_.back().joint_positions;
    result.progress = 1.0;
    result.time_remaining = 0.0;
    result.done = true;
    active_ = false;
  }

  return result;
}

std::vector<double> SetpointGenerator::final_target() const {
  std::lock_guard lock(mutex_);
  if (trajectory_.empty()) return {};
  return trajectory_.back().joint_positions;
}

double SetpointGenerator::finger_width() const {
  std::lock_guard lock(mutex_);
  return finger_width_;
}

double SetpointGenerator::total_duration() const {
  std::lock_guard lock(mutex_);
  return total_duration_;
}

}  // namespace robot_control
```

- [ ] **Step 3: Verify compilation**

```bash
cd /home/cll/workspace/isaac_ros_project
source install/setup.zsh
colcon build --base-paths src --packages-select robot_controller 2>&1 | tail -20
```

---

## Task 3: Extend ControlConstants

**Files:**
- Modify: `src/robot_controller/include/robot_controller/motion/control_constants.hpp`

- [ ] **Step 1: Add monitoring constants**

在 `ControlConstants` 中添加:

```cpp
static constexpr double kFollowingErrorLimit = 0.1;    ///< 跟踪误差阈值（rad），超过触发急停
static constexpr double kArrivalTolerance = 0.01;       ///< 到位判定阈值（rad）
static constexpr double kArrivalSettleTime = 0.2;       ///< 到位稳定等待时间（秒）
static constexpr double kTrajectoryTimeout = 15.0;      ///< 轨迹执行超时（秒）
static constexpr double kControlLoopHz = 100.0;         ///< 控制循环频率（Hz）
static constexpr double kControlLoopDt = 1.0 / 100.0;   ///< 控制循环周期（秒）
```

---

## Task 4: Extend MotionIOBridge Interface

**Files:**
- Modify: `src/robot_controller/include/robot_controller/motion/motion_io_bridge.hpp`

**Purpose:** Add trajectory submission interface so `RobotMotionController` (Layer 1) can submit trajectories without knowing about the 100Hz loop.

- [ ] **Step 1: Add new virtual methods**

在 `MotionIOBridge` 类末尾、`public` 区域添加:

```cpp
/// 提交轨迹给指令发生器（由控制循环执行）
virtual void submit_trajectory(
    const std::vector<TrajectoryStep>& steps, double finger) = 0;

/// 阻塞等待提交的轨迹执行完成
/// @return true 正常完成，false 被取消或超时
virtual bool wait_trajectory_completion(double timeout) = 0;

/// 取消当前轨迹
virtual void cancel_trajectory() = 0;
```

需要在文件顶部添加 include:
```cpp
#include "robot_controller/nodes/setpoint_generator.hpp"  // TrajectoryStep
```

Wait — 这会引入 Layer 2 的头文件到 Layer 1 的 `MotionIOBridge`。需要将 `TrajectoryStep` 移到独立头文件。

**更好的方案:** 将 `TrajectoryStep` 定义移到 `motion_io_bridge.hpp` 中（它是纯数据结构，不依赖 ROS）。

在 `MotionIOBridge` 类定义之前添加:
```cpp
/// @brief 轨迹步骤
struct TrajectoryStep {
  std::vector<double> joint_positions;
  double time_from_start;  ///< seconds
};
```

`setpoint_generator.hpp` 改为引用此定义:
```cpp
#include "robot_controller/motion/motion_io_bridge.hpp"  // TrajectoryStep
```

- [ ] **Step 2: Update setpoint_generator.hpp to use shared TrajectoryStep**

删除 `setpoint_generator.hpp` 中的 `TrajectoryStep` 定义，改为 include `motion_io_bridge.hpp`。

---

## Task 5: Implement RosMotionBridge Trajectory Methods

**Files:**
- Modify: `src/robot_controller/include/robot_controller/nodes/robot_controller_node.hpp`
- Modify: `src/robot_controller/src/nodes/robot_controller_node.cpp`

**Purpose:** `RosMotionBridge` 持有 `SetpointGenerator`，实现 `submit_trajectory` / `wait_trajectory_completion`。

- [ ] **Step 1: Add members to RosMotionBridge (in robot_controller_node.hpp)**

在 `RosMotionBridge` 的 `private` 区域添加:

```cpp
SetpointGenerator setpoint_gen_;
std::mutex trajectory_mutex_;
std::condition_variable trajectory_cv_;
```

在 `public` 区域添加:
```cpp
void submit_trajectory(
    const std::vector<TrajectoryStep>& steps, double finger) override;
bool wait_trajectory_completion(double timeout) override;
void cancel_trajectory() override;

/// 获取 SetpointGenerator（供控制循环使用）
SetpointGenerator& setpoint_generator() { return setpoint_gen_; }
```

需要额外 include:
```cpp
#include "robot_controller/nodes/setpoint_generator.hpp"
```

- [ ] **Step 2: Implement the methods (in robot_controller_node.cpp)**

在 `RosMotionBridge` 实现区域添加:

```cpp
void RosMotionBridge::submit_trajectory(
    const std::vector<TrajectoryStep>& steps, double finger) {
  setpoint_gen_.start(steps, finger);
}

bool RosMotionBridge::wait_trajectory_completion(double timeout) {
  auto deadline = std::chrono::steady_clock::now() +
                  std::chrono::duration<double>(timeout);
  std::unique_lock lock(trajectory_mutex_);
  return trajectory_cv_.wait_until(lock, deadline, [this] {
    return !setpoint_gen_.is_active();
  });
}

void RosMotionBridge::cancel_trajectory() {
  setpoint_gen_.cancel();
  trajectory_cv_.notify_all();
}
```

在 `update_joint_state()` 末尾（TF 发布后），添加轨迹完成信号通知:

```cpp
// 在 update_joint_state 的末尾添加:
// 如果轨迹刚刚完成（setpoint_gen_ 标记 done），通知等待者
if (!setpoint_gen_.is_active()) {
  trajectory_cv_.notify_all();
}
```

实际上轨迹完成信号应该由控制循环发出（更精确）。将 `trajectory_cv_.notify_all()` 放在控制循环的 Monitor 阶段。

---

## Task 6: Modify RobotMotionController

**Files:**
- Modify: `src/robot_controller/src/motion/robot_motion_controller.cpp`

**Purpose:** `moveJ_internal` 和 `moveL` 改用 `bridge_->submit_trajectory()` + `bridge_->wait_trajectory_completion()`，不再自己循环发布。

- [ ] **Step 1: Refactor moveJ_internal**

将 `moveJ_internal` 中从 `// 基于绝对时间调度发送轨迹点` 开始的循环替换为:

```cpp
void RobotMotionController::moveJ_internal(
    const std::vector<double>& target_angles, double finger, bool block) {
  auto current = bridge_->get_current_arm();

  double max_delta = 0.0;
  for (int i = 0; i < profile_.dof; ++i) {
    max_delta = std::max(max_delta, std::abs(target_angles[i] - current[i]));
  }
  if (max_delta < 1e-6) {
    return;
  }

  double speed_factor = movej_speed_ / 100.0;
  std::vector<MotionLimits> configs;
  configs.reserve(profile_.dof);
  for (int i = 0; i < profile_.dof; ++i) {
    configs.push_back({
        profile_.joint_limits.max_vel * speed_factor,
        profile_.joint_limits.max_acc * speed_factor,
        profile_.joint_limits.max_jerk * speed_factor});
  }

  auto trajectory = TrajectoryPlanner::plan_joint(
      current, target_angles, configs, ControlConstants::kTrajectoryDt);

  // 构造 TrajectoryStep 序列
  double dt = ControlConstants::kTrajectoryDt;
  std::vector<TrajectoryStep> steps;
  steps.reserve(trajectory.size());
  for (size_t i = 0; i < trajectory.size(); ++i) {
    steps.push_back({trajectory[i], static_cast<double>(i) * dt});
  }

  // 提交给指令发生器（由 100Hz 控制循环执行）
  bridge_->submit_trajectory(steps, finger);

  if (block) {
    double timeout = ControlConstants::kTrajectoryTimeout +
                     steps.back().time_from_start;
    bridge_->wait_trajectory_completion(timeout);

    // 等待实际到位
    bridge_->wait_for_motion(
        target_angles, finger,
        ControlConstants::kJointTolerance,
        ControlConstants::kFingerTolerance,
        ControlConstants::kMotionTimeout,
        ControlConstants::kPollInterval,
        ControlConstants::kSettleTime,
        !grasping_);
  }
}
```

- [ ] **Step 2: Refactor moveL similarly**

将 `moveL` 中轨迹发送循环替换为 submit + wait:

```cpp
// 在 moveL 方法中，将以下代码段:
//   auto start = std::chrono::steady_clock::now();
//   for (size_t i = 1; i < joint_traj.size(); ++i) {
//     ...sleep_until...
//     bridge_->publish_command(joint_traj[i], actual_finger);
//   }
// 替换为:

  // 提交给指令发生器
  std::vector<TrajectoryStep> steps;
  steps.reserve(joint_traj.size());
  for (size_t i = 0; i < joint_traj.size(); ++i) {
    // 使用 cart_traj 的时间
    double t = (i < cart_traj.size()) ? cart_traj[i].t
               : (i > 0 ? steps[i-1].time_from_start + ControlConstants::kTrajectoryDt : 0.0);
    steps.push_back({joint_traj[i], t});
  }

  bridge_->submit_trajectory(steps, actual_finger);

  if (block && !steps.empty()) {
    double timeout = ControlConstants::kTrajectoryTimeout +
                     steps.back().time_from_start;
    bridge_->wait_trajectory_completion(timeout);

    const auto& final_angles = steps.back().joint_positions;
    bridge_->wait_for_motion(
        final_angles, actual_finger,
        ControlConstants::kJointTolerance,
        ControlConstants::kFingerTolerance,
        ControlConstants::kMotionTimeout,
        ControlConstants::kPollInterval,
        ControlConstants::kSettleTime,
        !grasping_);
  }
```

---

## Task 7: Add 100Hz Control Loop to RobotControllerNode

**Files:**
- Modify: `src/robot_controller/include/robot_controller/nodes/robot_controller_node.hpp`
- Modify: `src/robot_controller/src/nodes/robot_controller_node.cpp`

**Purpose:** The core change — add `RobotStateModel` and the 100Hz `control_loop_timer_` that performs Read→Plan→Monitor→Write.

### Header Changes

- [ ] **Step 1: Add new members to RobotControllerNode**

在 `private` 区域添加:

```cpp
// === 100Hz 控制循环 ===
rclcpp::TimerBase::SharedPtr control_loop_timer_;
RobotStateModel state_model_;  // 构造时初始化 dof

// === 活跃 Action 追踪 ===
struct ActiveMotion {
  std::variant<
    std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_msgs::action::MoveJ>>,
    std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_msgs::action::MoveL>>
  > goal_handle;
  std::vector<double> target_angles;  // 最终目标（用于到位判定）
  double target_finger;
  std::chrono::steady_clock::time_point start_time;
  double total_duration;
};
std::unique_ptr<ActiveMotion> active_motion_;
std::mutex active_motion_mutex_;
std::chrono::steady_clock::time_point trajectory_done_time_;  // 轨迹播放完成时刻
bool waiting_settle_ = false;  // 正在等待到位稳定

// === 控制循环方法 ===
void control_loop_tick();
bool check_action_completion();  // 返回 true 表示动作已完成并已处理
```

需要添加 include:
```cpp
#include <variant>
#include "robot_controller/nodes/robot_state_model.hpp"
```

移除（不再需要 action 执行线程）:
```cpp
// 移除:
std::thread movej_thread_;
std::thread movel_thread_;
```

Wait — 先不移除。分阶段来，先加 100Hz 循环，action 线程在 Task 8 再移除。

### Implementation Changes

- [ ] **Step 2: Initialize state_model_ and create control loop timer**

在 `init()` 方法中，`RobotControllerNode` 构造后、创建 action server 之前:

```cpp
// 初始化状态模型
state_model_ = RobotStateModel(profile_.dof);

// === 100Hz 控制循环 ===
control_loop_timer_ = create_wall_timer(
    std::chrono::microseconds(static_cast<int64_t>(1e6 / ControlConstants::kControlLoopHz)),
    [this]() { control_loop_tick(); }, pub_cbg_);
```

- [ ] **Step 3: Implement control_loop_tick()**

这是核心方法:

```cpp
void RobotControllerNode::control_loop_tick() {
  // === 1. READ ===
  auto actual = bridge_->get_current_arm();
  double actual_finger = bridge_->get_current_finger();
  state_model_.update_actual(actual, actual_finger);

  // === 2. PLAN (Setpoint Generator) ===
  auto state = state_machine_.state();
  auto target = state_model_.get_target_joints();
  double target_finger = state_model_.get_target_gripper();

  switch (state) {
    case RobotState::kMoving: {
      auto& gen = bridge_->setpoint_generator();
      if (gen.is_active()) {
        auto result = gen.tick(std::chrono::steady_clock::now());
        target = result.joint_positions;
        target_finger = result.finger_width;

        // 发布 Action feedback
        std::lock_guard lock(active_motion_mutex_);
        if (active_motion_ && result.progress > 0) {
          std::visit([&](auto& gh) {
            auto fb = std::make_shared<
                std::decay_t<decltype(*gh->get_goal())>::Feedback>();
            // 使用 ROS2 action feedback消息类型
            // MoveJ/MoveL 的 feedback 结构一致
            // 通过模板或 variant 处理
            publish_action_feedback(gh, result);
          }, active_motion_->goal_handle);
        }

        // 轨迹播放完成
        if (result.done) {
          trajectory_done_time_ = std::chrono::steady_clock::now();
          waiting_settle_ = true;
        }
      }
      break;
    }
    case RobotState::kTeaching: {
      // Jog: 由 jog 控制器计算增量
      if (jog_controller_ && jog_controller_->is_active()) {
        std::array<double, 7> fb{};
        auto actual_j = state_model_.get_actual_joints();
        for (size_t i = 0; i < 7 && i < actual_j.size(); ++i) fb[i] = actual_j[i];

        jog_controller_->set_finger_width(actual_finger);
        jog_controller_->tick(fb, [](const std::vector<double>&, double) {});

        auto jog_target = jog_controller_->get_commanded_joints();
        target = std::vector<double>(jog_target.begin(), jog_target.end());
        target_finger = actual_finger;

        // Jog 减速完成检测
        if (!jog_controller_->is_active()) {
          state_machine_.transition_to(RobotState::kIdle);
        }
      }
      break;
    }
    case RobotState::kIdle:
    case RobotState::kFault:
    case RobotState::kStopping:
      // 保持 target 不变
      break;
  }

  state_model_.update_target(target, target_finger);

  // === 3. MONITOR ===
  double error = state_model_.max_following_error();

  // 动态误差监控（仅运动状态检查）
  if ((state == RobotState::kMoving || state == RobotState::kTeaching) &&
      error > ControlConstants::kFollowingErrorLimit) {
    RCLCPP_ERROR(this->get_logger(),
                 "Following error %.4f rad exceeds limit %.4f rad — EMERGENCY STOP",
                 error, ControlConstants::kFollowingErrorLimit);
    emergency_stop();
    return;
  }

  // 轨迹完成 + 到位判定
  if (state == RobotState::kMoving && waiting_settle_) {
    if (check_action_completion()) {
      return;  // action 已完成，状态已转换
    }
  }

  // === 4. WRITE ===
  bridge_->publish_command(target, target_finger);
}
```

- [ ] **Step 4: Implement check_action_completion()**

```cpp
bool RobotControllerNode::check_action_completion() {
  std::lock_guard lock(active_motion_mutex_);
  if (!active_motion_) return false;

  // 检查是否稳定到位
  auto& gen = bridge_->setpoint_generator();
  bool on_target = state_model_.is_on_target(ControlConstants::kArrivalTolerance);
  bool finger_ok = std::abs(state_model_.get_target_gripper() -
                            state_model_.get_actual_gripper()) < ControlConstants::kFingerTolerance;

  // 检查超时
  auto elapsed = std::chrono::steady_clock::now() - trajectory_done_time_;
  bool timed_out = std::chrono::duration<double>(elapsed).seconds() >
                   ControlConstants::kArrivalSettleTime +
                   ControlConstants::kTrajectoryTimeout;

  // 检查是否被急停
  if (state_machine_.state() == RobotState::kFault) {
    // Action 被 abort（由 emergency_stop 处理）
    active_motion_.reset();
    waiting_settle_ = false;
    return true;
  }

  if (on_target && finger_ok) {
    // 稳定到位 → 成功
    auto final_angles = state_model_.get_actual_joints();
    auto final_pose = controller_->get_end_effector_pose();

    std::visit([&](auto& gh) {
      using GoalHandle = std::decay_t<decltype(*gh)>;
      auto result = std::make_shared<typename GoalHandle::Result>();
      result->success = true;
      result->message = "completed";
      // 填充 final_joint_angles / final_tcp_pose（如果 Result 有这些字段）
      fill_action_result(result, final_angles, final_pose);
      gh->succeed(result);
    }, active_motion_->goal_handle);

    state_machine_.transition_to(RobotState::kIdle);
    bridge_->cancel_trajectory();  // 确保 generator 状态清理
    active_motion_.reset();
    waiting_settle_ = false;

    // 通知 Python API 等待者
    // (trajectory_cv_ 在 RosMotionBridge 中)
    return true;
  }

  if (timed_out) {
    // 超时 → 失败
    std::visit([&](auto& gh) {
      using GoalHandle = std::decay_t<decltype(*gh)>;
      auto result = std::make_shared<typename GoalHandle::Result>();
      result->success = false;
      result->message = "trajectory timeout — robot did not reach target";
      gh->abort(result);
    }, active_motion_->goal_handle);

    state_machine_.transition_to(RobotState::kFault);
    state_machine_.set_error(10, "Trajectory timeout");
    active_motion_.reset();
    waiting_settle_ = false;
    return true;
  }

  return false;  // 还在等待
}
```

> **注意:** `publish_action_feedback` 和 `fill_action_result` 是辅助函数，需要根据 `MoveJ::Feedback`/`MoveL::Feedback` 和 `MoveJ::Result`/`MoveL::Result` 的实际 msg 定义来实现。由于 MoveJ 和 MoveL 的 feedback/result 结构不同，可以用模板或 `std::visit` + lambda 处理。

---

## Task 8: Refactor Action Servers (Remove Threads)

**Files:**
- Modify: `src/robot_controller/src/nodes/robot_controller_node.cpp`

**Purpose:** Action handlers become thin: validate → plan trajectory → submit to SetpointGenerator → set state. No threads. The 100Hz loop handles execution, feedback, and result.

- [ ] **Step 1: Refactor handle_movej_accepted**

```cpp
void RobotControllerNode::handle_movej_accepted(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_msgs::action::MoveJ>> goal_handle) {
  if (!state_machine_.transition_to(RobotState::kMoving)) {
    auto result = std::make_shared<robot_msgs::action::MoveJ::Result>();
    result->success = false;
    result->message = "MoveJ rejected: cannot transition to MOVING";
    goal_handle->abort(result);
    return;
  }

  try {
    auto goal = goal_handle->get_goal();
    double effective_speed = goal->speed_ratio * global_speed_ratio_.load();

    std::vector<double> target_angles;

    if (goal->mode == robot_msgs::action::MoveJ::Goal::CARTESIAN) {
      // IK 求解（与现有逻辑相同）
      std::array<double, 3> xyz = {goal->position.x, goal->position.y, goal->position.z};
      std::array<double, 3> rpy = {goal->orientation.x, goal->orientation.y, goal->orientation.z};

      auto tcp_cfg = profile_.tcp_frames.at(controller_->get_current_tcp());
      Eigen::Matrix4d T_tcp = Eigen::Matrix4d::Identity();
      Eigen::AngleAxisd roll_a(tcp_cfg.offset_rpy[0], Eigen::Vector3d::UnitX());
      Eigen::AngleAxisd pitch_a(tcp_cfg.offset_rpy[1], Eigen::Vector3d::UnitY());
      Eigen::AngleAxisd yaw_a(tcp_cfg.offset_rpy[2], Eigen::Vector3d::UnitZ());
      T_tcp.block<3,3>(0,0) = (yaw_a * pitch_a * roll_a).toRotationMatrix();
      T_tcp(0,3) = tcp_cfg.offset_xyz[0]; T_tcp(1,3) = tcp_cfg.offset_xyz[1]; T_tcp(2,3) = tcp_cfg.offset_xyz[2];

      Eigen::Matrix4d T_target = Eigen::Matrix4d::Identity();
      Eigen::AngleAxisd e_roll(rpy[0], Eigen::Vector3d::UnitX());
      Eigen::AngleAxisd e_pitch(rpy[1], Eigen::Vector3d::UnitY());
      Eigen::AngleAxisd e_yaw(rpy[2], Eigen::Vector3d::UnitZ());
      T_target.block<3,3>(0,0) = (e_yaw * e_pitch * e_roll).toRotationMatrix();
      T_target(0,3) = xyz[0]; T_target(1,3) = xyz[1]; T_target(2,3) = xyz[2];

      Eigen::Matrix4d hand_target = T_target * T_tcp.inverse();
      Eigen::Vector3d h_xyz = hand_target.block<3,1>(0,3);
      Eigen::Vector3d h_rpy = hand_target.block<3,3>(0,0).eulerAngles(0,1,2);

      auto ik_result = ik_->solve(
          std::array<double,3>{h_xyz.x(), h_xyz.y(), h_xyz.z()},
          std::array<double,3>{h_rpy.x(), h_rpy.y(), h_rpy.z()});
      if (!ik_result) {
        auto result = std::make_shared<robot_msgs::action::MoveJ::Result>();
        result->success = false;
        result->message = "IK solver failed";
        goal_handle->abort(result);
        state_machine_.transition_to(RobotState::kFault);
        state_machine_.set_error(1, "MoveJ: IK failed");
        return;
      }
      target_angles = *ik_result;
    } else {
      target_angles = {goal->joint_angles.begin(), goal->joint_angles.end()};
    }

    // S 曲线轨迹规划
    auto current = bridge_->get_current_arm();
    double speed_factor = effective_speed * (controller_->get_speed(MotionMode::kMoveJ) / 100.0);
    std::vector<MotionLimits> configs(profile_.dof);
    for (int i = 0; i < profile_.dof; ++i) {
      configs[i] = {profile_.joint_limits.max_vel * speed_factor,
                    profile_.joint_limits.max_acc * speed_factor,
                    profile_.joint_limits.max_jerk * speed_factor};
    }

    auto trajectory = TrajectoryPlanner::plan_joint(
        current, target_angles, configs, ControlConstants::kTrajectoryDt);

    double dt = ControlConstants::kTrajectoryDt;
    std::vector<TrajectoryStep> steps;
    steps.reserve(trajectory.size());
    for (size_t i = 0; i < trajectory.size(); ++i) {
      steps.push_back({trajectory[i], static_cast<double>(i) * dt});
    }

    double finger = (goal->finger_width >= 0) ? goal->finger_width
                    : bridge_->get_current_finger();

    // 提交给指令发生器（不启动线程！）
    bridge_->setpoint_generator().start(steps, finger);

    // 记录活跃动作
    {
      std::lock_guard lock(active_motion_mutex_);
      active_motion_ = std::make_unique<ActiveMotion>();
      active_motion_->goal_handle = goal_handle;
      active_motion_->target_angles = target_angles;
      active_motion_->target_finger = finger;
      active_motion_->start_time = std::chrono::steady_clock::now();
      active_motion_->total_duration = steps.back().time_from_start;
    }
    waiting_settle_ = false;

    RCLCPP_INFO(this->get_logger(),
                "MoveJ accepted: %zu steps, %.2fs duration",
                steps.size(), steps.back().time_from_start);

  } catch (const std::exception& e) {
    auto result = std::make_shared<robot_msgs::action::MoveJ::Result>();
    result->success = false;
    result->message = std::string("MoveJ error: ") + e.what();
    goal_handle->abort(result);
    state_machine_.transition_to(RobotState::kFault);
    state_machine_.set_error(2, result->message);
  }
  // 不再创建线程！直接返回。100Hz 循环接管执行。
}
```

- [ ] **Step 2: Refactor handle_movel_accepted (same pattern)**

MoveL 的重构与 MoveJ 完全对称:
1. 验证 + IK 规划（保留现有笛卡尔插值 + IK 逻辑）
2. 构建 `TrajectoryStep` 序列
3. `bridge_->setpoint_generator().start(steps, finger)`
4. 填充 `active_motion_`
5. 返回（无线程）

MoveL 取消处理也类似。

- [ ] **Step 3: Remove action thread members**

在 `robot_controller_node.hpp` 中移除:
```cpp
// 删除:
std::thread movej_thread_;
std::thread movel_thread_;
```

在析构函数中移除:
```cpp
// 删除:
if (movej_thread_.joinable()) movej_thread_.join();
if (movel_thread_.joinable()) movel_thread_.join();
```

- [ ] **Step 4: Remove TrajectoryExecutor dependency**

在 `robot_controller_node.hpp` 中移除:
```cpp
// 删除:
#include "robot_controller/nodes/trajectory_executor.hpp"
// 删除:
std::unique_ptr<TrajectoryExecutor> trajectory_executor_;
```

在 `init()` 中移除:
```cpp
// 删除:
trajectory_executor_ = std::make_unique<TrajectoryExecutor>(...);
```

所有 `trajectory_executor_->` 调用替换为 `bridge_->setpoint_generator().` 调用。

---

## Task 9: Integrate Jog into Control Loop + Remove Separate Jog Timer

**Files:**
- Modify: `src/robot_controller/src/nodes/robot_controller_node.cpp`

**Purpose:** Jog tick 现在在 100Hz 控制循环中执行（Task 7 已包含），移除单独的 50Hz jog tick timer。

- [ ] **Step 1: Remove jog_tick_timer_**

在 `init()` 中移除:
```cpp
// 删除:
jog_tick_timer_ = create_wall_timer(
    std::chrono::milliseconds(20),
    [this]() { jog_tick_callback(); }, pub_cbg_);
```

在头文件中移除:
```cpp
// 删除:
rclcpp::TimerBase::SharedPtr jog_tick_timer_;
void jog_tick_callback();
```

- [ ] **Step 2: Update JogController dt**

Jog 从 50Hz → 100Hz，需要更新 `JogConfig`:
```cpp
JogConfig jog_cfg;
jog_cfg.dof = profile_.dof;
jog_cfg.dt = ControlConstants::kControlLoopDt;  // 0.01s (100Hz)
jog_controller_ = std::make_unique<JogController>(ik_, jog_cfg);
```

- [ ] **Step 3: Keep jog_watchdog_timer_ (still needed)**

200ms watchdog timer 保持不变，它检测 JogCommand 超时。

---

## Task 10: Update State Machine Transitions

**Files:**
- Modify: `src/robot_controller/include/robot_controller/nodes/robot_state.hpp`
- Modify: `src/robot_controller/src/nodes/robot_state.cpp`

- [ ] **Step 1: Add kStopping → kFault transition**

在 `is_valid_transition` 中:
```cpp
case RobotState::kStopping:
  return to == RobotState::kIdle || to == RobotState::kFault;
```

这允许急停在停止过程中触发。

---

## Task 11: Update Emergency Stop and CLEAR_FAULT

**Files:**
- Modify: `src/robot_controller/src/nodes/robot_controller_node.cpp`

- [ ] **Step 1: Update emergency_stop()**

```cpp
void RobotControllerNode::emergency_stop() {
  // 取消轨迹
  bridge_->setpoint_generator().cancel();

  // 急停 Jog
  if (jog_controller_ && jog_controller_->is_active()) {
    auto actual = state_model_.get_actual_joints();
    std::array<double, 7> fb{};
    for (size_t i = 0; i < 7 && i < actual.size(); ++i) fb[i] = actual[i];
    jog_controller_->emergency_stop(fb);
  }

  // 中止活跃 Action
  {
    std::lock_guard lock(active_motion_mutex_);
    if (active_motion_) {
      std::visit([](auto& gh) {
        // 使用合适的结果类型
        using GH = std::decay_t<decltype(*gh)>;
        auto result = std::make_shared<typename GH::Result>();
        result->success = false;
        result->message = "EMERGENCY_STOP activated";
        gh->abort(result);
      }, active_motion_->goal_handle);
      active_motion_.reset();
    }
  }
  waiting_settle_ = false;

  // 锁死 target 为当前 actual
  state_model_.align_target_to_actual();

  state_machine_.force_state(RobotState::kFault);
  state_machine_.set_error(100, "EMERGENCY_STOP activated");
  RCLCPP_ERROR(this->get_logger(), "EMERGENCY_STOP activated");
}
```

- [ ] **Step 2: Update CLEAR_FAULT handler**

```cpp
case robot_msgs::srv::RobotCmd::Request::CLEAR_FAULT:
  if (state_machine_.state() == RobotState::kFault) {
    state_machine_.clear_error();
    // 恢复前将 target 对齐到 actual，防止跳变
    state_model_.align_target_to_actual();
    state_machine_.transition_to(RobotState::kIdle);
    res->success = true;
    res->message = "FAULT cleared";
  } else {
    res->success = false;
    res->message = "CLEAR_FAULT: robot not in FAULT state";
  }
  break;
```

---

## Task 12: Update CMakeLists.txt

**Files:**
- Modify: `src/robot_controller/CMakeLists.txt`

- [ ] **Step 1: Replace trajectory_executor with setpoint_generator**

```cmake
# robot_nodes: ROS2 节点（依赖 motion）
add_library(robot_nodes SHARED
  src/nodes/robot_controller_node.cpp
  src/nodes/robot_state.cpp
  src/nodes/setpoint_generator.cpp       # 替换 trajectory_executor.cpp
)
```

---

## Task 13: Verify Compilation and Test

- [ ] **Step 1: Full build**

```bash
cd /home/cll/workspace/isaac_ros_project
source install/setup.zsh
colcon build --base-paths src --packages-select robot_controller
```

- [ ] **Step 2: Run offline tests**

```bash
colcon test --packages-select robot_controller
```

- [ ] **Step 3: Verify Python bindings still work**

```bash
colcon build --base-paths src --packages-select robot_api_python
python3 -c "import robot_api_python as rc; print('OK')"
```

- [ ] **Step 4: Commit**

```bash
git add -A
git commit -m "refactor: replace action threads with 100Hz control loop

- Add RobotStateModel (thread-safe target/actual state)
- Add SetpointGenerator (tick-based trajectory provider)
- Add 100Hz Read→Plan→Monitor→Write control loop
- Action servers now submit trajectories (no threads)
- Add following error monitoring and arrival detection
- Jog integrated into control loop (100Hz)
- Emergency stop aligns target to actual
- CLEAR_FAULT forces target alignment"
```

---

## Review Fixes (Critical/Important Issues)

The following issues were identified during plan review and must be addressed during implementation.

### Fix 1: Python Blocking Path Deadlock (Critical)

**Problem:** When Python calls `controller_->moveJ(angles, block=true)`, it submits a trajectory via `bridge_->submit_trajectory()` and then calls `bridge_->wait_trajectory_completion()`. But `trajectory_cv_` is only notified in `check_action_completion()`, which only runs when `active_motion_` exists (Action path). For the Python path, `active_motion_` is null, so the CV is never signaled → deadlock.

**Fix:** In `control_loop_tick()`, unconditionally notify `trajectory_cv_` when the setpoint generator transitions from active to done:

```cpp
// In control_loop_tick(), Plan section, kMoving case:
if (result.done) {
  trajectory_done_time_ = std::chrono::steady_clock::now();
  waiting_settle_ = true;
  // Notify Python/blocking waiters regardless of active_motion_
  bridge_->notify_trajectory_complete();  // calls trajectory_cv_.notify_all()
}
```

Add to `RosMotionBridge`:
```cpp
void notify_trajectory_complete() {
  trajectory_cv_.notify_all();
}
```

Also remove the redundant `wait_for_motion()` call after `wait_trajectory_completion()` in the Python path (Task 6). Instead, have `wait_trajectory_completion()` include the settle wait:

```cpp
bool RosMotionBridge::wait_trajectory_completion(double timeout) {
  auto deadline = std::chrono::steady_clock::now() +
                  std::chrono::duration<double>(timeout);
  std::unique_lock lock(trajectory_mutex_);
  bool completed = trajectory_cv_.wait_until(lock, deadline, [this] {
    return !setpoint_gen_.is_active();
  });
  return completed;
}
```

And in `RobotMotionController::moveJ_internal()`, the `block` path becomes:
```cpp
bridge_->submit_trajectory(steps, finger);
if (block) {
  double total_time = steps.back().time_from_start;
  double timeout = total_time + ControlConstants::kTrajectoryTimeout;
  bool ok = bridge_->wait_trajectory_completion(timeout);
  if (ok) {
    bridge_->wait_for_motion(target_angles, finger,
        ControlConstants::kJointTolerance, ControlConstants::kFingerTolerance,
        ControlConstants::kMotionTimeout, ControlConstants::kPollInterval,
        ControlConstants::kSettleTime, !grasping_);
  }
}
```

### Fix 2: waiting_settle_ Race Condition (Critical)

**Problem:** `waiting_settle_` and `trajectory_done_time_` are written from `control_loop_tick()` (timer callback on `pub_cbg_`) and read/written from `emergency_stop()` (service callback, potentially different thread). No lock protects them.

**Fix:** Protect with `active_motion_mutex_`:

```cpp
// In control_loop_tick():
{
  std::lock_guard lock(active_motion_mutex_);
  trajectory_done_time_ = std::chrono::steady_clock::now();
  waiting_settle_ = true;
}

// In check_action_completion() — already holds active_motion_mutex_

// In emergency_stop():
{
  std::lock_guard lock(active_motion_mutex_);
  waiting_settle_ = false;
  // ... abort active motion ...
}
```

### Fix 3: TrajectoryStep Single Definition (Important)

**Problem:** Plan defines `TrajectoryStep` in both `setpoint_generator.hpp` (Task 2) and `motion_io_bridge.hpp` (Task 4), causing ODR violation.

**Fix:** Define `TrajectoryStep` ONLY in `motion_io_bridge.hpp` (Layer 1, already included everywhere). In Task 2, `setpoint_generator.hpp` includes `motion_io_bridge.hpp` instead of defining its own `TrajectoryStep`.

```cpp
// setpoint_generator.hpp
#pragma once
#include "robot_controller/motion/motion_io_bridge.hpp"  // TrajectoryStep
#include <chrono>
#include <mutex>
#include <vector>
// ... rest of the header, without TrajectoryStep definition
```

### Fix 4: Template Action Feedback/Result Helpers (Important)

**Problem:** Plan references `publish_action_feedback` and `fill_action_result` but never defines them.

**Fix:** Add template helpers in `robot_controller_node.cpp`:

```cpp
// Template helper: publish action feedback from SetpointResult
template<typename GoalHandleT>
void publish_feedback(
    const std::shared_ptr<GoalHandleT>& gh,
    const SetpointResult& sp) {
  auto fb = std::make_shared<typename GoalHandleT::Feedback>();
  fb->progress = sp.progress;
  const auto& angles = sp.joint_positions;
  std::copy_n(angles.begin(),
              std::min(angles.size(), size_t(7)),
              fb->current_joint_angles.begin());
  fb->estimated_time_remaining = sp.time_remaining;
  gh->publish_feedback(fb);
}

// Template helper: succeed action with final state
template<typename GoalHandleT>
void succeed_action(
    const std::shared_ptr<GoalHandleT>& gh,
    const std::vector<double>& angles,
    const std::array<double, 6>& pose,
    const std::string& msg = "completed") {
  auto result = std::make_shared<typename GoalHandleT::Result>();
  result->success = true;
  result->message = msg;
  std::copy_n(angles.begin(),
              std::min(angles.size(), size_t(7)),
              result->final_joint_angles.begin());
  result->final_tcp_pose = {pose[0], pose[1], pose[2],
                            pose[3], pose[4], pose[5]};
  gh->succeed(result);
}

// Template helper: abort action
template<typename GoalHandleT>
void abort_action(
    const std::shared_ptr<GoalHandleT>& gh,
    const std::string& msg) {
  auto result = std::make_shared<typename GoalHandleT::Result>();
  result->success = false;
  result->message = msg;
  gh->abort(result);
}
```

Usage in `control_loop_tick()`:
```cpp
std::visit([&](auto& gh) {
  publish_feedback(gh, result);
}, active_motion_->goal_handle);
```

Usage in `check_action_completion()`:
```cpp
std::visit([&](auto& gh) {
  succeed_action(gh, final_angles, final_pose);
}, active_motion_->goal_handle);
```

### Fix 5: SetpointGenerator::cancel() Mutex (Important)

**Problem:** `cancel()` sets atomic flags without acquiring mutex, causing ordering issue with concurrent `start()`.

**Fix:**
```cpp
void SetpointGenerator::cancel() {
  std::lock_guard lock(mutex_);  // Add lock
  cancelled_ = true;
  active_ = false;
}
```

### Fix 6: STOP Command kStopping→kIdle Transition (Important)

**Problem:** After removing action threads, nothing handles the `kStopping → kIdle` transition. The old code relied on the action thread to do this after the trajectory was cancelled.

**Fix:** In `control_loop_tick()`, add handling for `kStopping` state:

```cpp
case RobotState::kStopping: {
  // 轨迹已取消，等待机器人稳定后切回 kIdle
  auto& gen = bridge_->setpoint_generator();
  if (!gen.is_active()) {
    // SetpointGenerator 已停止，检查到位
    if (state_model_.is_on_target(ControlConstants::kArrivalTolerance)) {
      state_machine_.transition_to(RobotState::kIdle);
      RCLCPP_INFO(this->get_logger(), "STOP complete, transitioned to IDLE");
    }
  }
  // 保持当前 target（stop 时已锁死）
  break;
}
```

Also update the STOP handler to NOT rely on action threads:
```cpp
case robot_msgs::srv::RobotCmd::Request::STOP:
  if (state_machine_.state() == RobotState::kMoving ||
      state_machine_.state() == RobotState::kTeaching) {
    bridge_->setpoint_generator().cancel();
    if (jog_controller_ && jog_controller_->is_active()) {
      jog_controller_->stop();
    }
    state_machine_.transition_to(RobotState::kStopping);
    res->success = true;
    res->message = "STOP executed";
  }
  break;
```

---

## Compatibility Notes

### Python API (pybind11)
- `RobotMotionController` 的公开方法签名**不变**
- `moveJ()`, `moveL()` 等方法内部改用 `submit_trajectory` + `wait`
- `block=true` 行为保持一致
- **无需修改 bindings.cpp**

### HMI (示教器)
- Action server 接口（MoveJ/MoveL）不变
- JogCommand 话题和协议不变
- RobotStatus 话题不变
- **无需修改 robot_hmi 代码**

### Service API
- 所有 Service 接口（兼容层）不变
- `handle_move_joint` 等仍调用 `controller_->moveJ()`
- **完全向后兼容**

### Key Invariants Preserved
1. `MotionIOBridge` 的旧方法（`publish_command`, `wait_for_motion`）仍然可用
2. `IRobotController` 接口不变
3. `TrajectoryStep` 结构体可复用
4. `JogController` 接口不变（只是调用频率从 50Hz → 100Hz）
