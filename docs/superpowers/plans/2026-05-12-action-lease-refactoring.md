# Action + Lease + Client Library Merge 实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将运动指令从 Service 迁移到 Action，引入租约模型替代 MotionOwner，将 robot_api_cpp 合并进 robot_controller。

**Architecture:** 在 robot_controller 中新增 Action Server 和租约管理器，新增 robot_client CMake target 提供 Action/Service Client 封装。robot_api_cpp 包被删除，所有下游改为依赖 robot_controller::robot_client。

**Tech Stack:** ROS2 Jazzy (rclcpp_action), C++17, colcon + ament_cmake, pybind11

**Spec:** `docs/superpowers/specs/2026-05-12-action-lease-refactoring-design.md`

---

## 文件变更总览

### Phase 1: robot_msgs（接口层）

| 操作 | 文件 |
|------|------|
| 新建 | `src/robot_msgs/srv/AcquireControl.srv` |
| 新建 | `src/robot_msgs/srv/ReleaseControl.srv` |
| 新建 | `src/robot_msgs/srv/RenewLease.srv` |
| 新建 | `src/robot_msgs/srv/RequestTeachingMode.srv` |
| 新建 | `src/robot_msgs/action/GoHome.action` |
| 新建 | `src/robot_msgs/action/GraspTask.action` |
| 修改 | `src/robot_msgs/action/MoveJ.action`（加 session_id） |
| 修改 | `src/robot_msgs/action/MoveL.action`（加 session_id） |
| 修改 | `src/robot_msgs/msg/RobotStatus.msg`（motion_owner → session 信息） |
| 删除 | `src/robot_msgs/srv/MoveJoint.srv` |
| 删除 | `src/robot_msgs/srv/MovePose.srv` |
| 删除 | `src/robot_msgs/srv/MoveLinear.srv` |
| 删除 | `src/robot_msgs/srv/GoHome.srv` |
| 修改 | `src/robot_msgs/CMakeLists.txt` |

### Phase 2: robot_controller（后端层）

| 操作 | 文件 |
|------|------|
| 新建 | `src/robot_controller/include/robot_controller/nodes/lease_manager.hpp` |
| 新建 | `src/robot_controller/src/nodes/lease_manager.cpp` |
| 新建 | `src/robot_controller/include/robot_controller/nodes/action_handlers.hpp` |
| 新建 | `src/robot_controller/src/nodes/action_handlers.cpp` |
| 修改 | `src/robot_controller/include/robot_controller/nodes/robot_controller_node.hpp` |
| 修改 | `src/robot_controller/src/nodes/robot_controller_node.cpp` |
| 修改 | `src/robot_controller/src/nodes/robot_controller_node_services.cpp` |
| 修改 | `src/robot_controller/include/robot_controller/motion/i_robot_controller.hpp` |
| 删除 | `src/robot_controller/include/robot_controller/nodes/motion_owner.hpp` |
| 新建 | `src/robot_controller/include/robot_controller/client/action_robot_controller.hpp` |
| 新建 | `src/robot_controller/src/client/action_robot_controller.cpp` |
| 新建 | `src/robot_controller/include/robot_controller/client/robot_client.hpp` |
| 新建 | `src/robot_controller/src/client/robot_client.cpp` |
| 修改 | `src/robot_controller/CMakeLists.txt` |
| 新建 | `src/robot_controller/test/test_lease_manager.cpp` |
| 新建 | `src/robot_controller/test/test_action_integration.cpp` |

### Phase 3: robot_tasks（编排层）

| 操作 | 文件 |
|------|------|
| 修改 | `src/robot_tasks/CMakeLists.txt` |
| 修改 | `src/robot_tasks/package.xml` |
| 新建 | `src/robot_tasks/include/robot_tasks/grasp_task_node.hpp` |
| 新建 | `src/robot_tasks/src/grasp_task_node.cpp` |
| 新建 | `src/robot_tasks/src/grasp_task_main.cpp` |
| 修改 | `src/robot_tasks/include/robot_tasks/grasp_task_manager.hpp` |
| 修改 | `src/robot_tasks/src/grasp_task_manager.cpp` |

### Phase 4: 前端迁移

| 操作 | 文件 |
|------|------|
| 修改 | `src/robot_hmi/CMakeLists.txt` |
| 修改 | `src/robot_hmi/package.xml` |
| 修改 | `src/robot_hmi/include/robot_hmi/pendant_node.hpp` |
| 修改 | `src/robot_hmi/src/pendant_node.cpp` |
| 修改 | `src/robot_api_python/CMakeLists.txt` |
| 修改 | `src/robot_api_python/package.xml` |
| 修改 | `src/robot_api_python/src/bindings.cpp` |
| 修改 | `src/robot_demos/CMakeLists.txt` |
| 修改 | `src/robot_demos/package.xml` |
| 修改 | `src/robot_demos/demo/demo_vision_grasp.cpp` |
| 修改 | `src/robot_demos/demo/demo_vision_diagnostic.cpp` |
| 修改 | `src/robot_demos/demo/demo_grasp_tcp.cpp`（验证 motion_owner.hpp 引用） |
| 修改 | `src/robot_demos/test/test_robot_node.cpp`（go_home 调用适配） |

### Phase 5: 清理

| 操作 | 文件 |
|------|------|
| 删除 | `src/robot_api_cpp/`（整个目录） |
| 修改 | `src/robot_bringup/launch/`（launch 文件） |
| 修改 | 各包 CLAUDE.md |

---

## Phase 1: robot_msgs 接口层

### Task 1: 创建租约 Service 定义

**Files:**
- Create: `src/robot_msgs/srv/AcquireControl.srv`
- Create: `src/robot_msgs/srv/ReleaseControl.srv`
- Create: `src/robot_msgs/srv/RenewLease.srv`
- Create: `src/robot_msgs/srv/RequestTeachingMode.srv`

- [ ] **Step 1: 创建 4 个 .srv 文件**

`src/robot_msgs/srv/AcquireControl.srv`:
```
string client_name
float64 lease_duration
---
bool success
string session_id
float64 lease_timeout
string message
```

`src/robot_msgs/srv/ReleaseControl.srv`:
```
string session_id
---
bool success
string message
```

`src/robot_msgs/srv/RenewLease.srv`:
```
string session_id
float64 lease_extension
---
bool success
float64 new_timeout
string message
```

`src/robot_msgs/srv/RequestTeachingMode.srv`:
```
string session_id
---
bool success
string message
```

- [ ] **Step 2: Commit**

```bash
git add src/robot_msgs/srv/AcquireControl.srv src/robot_msgs/srv/ReleaseControl.srv src/robot_msgs/srv/RenewLease.srv src/robot_msgs/srv/RequestTeachingMode.srv
git commit -m "feat(robot_msgs): 新增租约管理 Service 定义"
```

---

### Task 2: 修改 Action 定义 + 新建 GoHome/GraspTask

**Files:**
- Modify: `src/robot_msgs/action/MoveJ.action`
- Modify: `src/robot_msgs/action/MoveL.action`
- Create: `src/robot_msgs/action/GoHome.action`
- Create: `src/robot_msgs/action/GraspTask.action`

- [ ] **Step 1: 修改 MoveJ.action — 在 Goal 末尾添加 session_id**

在 `src/robot_msgs/action/MoveJ.action` 的 Goal 部分，`float64 finger_width` 之后添加 `string session_id`。

- [ ] **Step 2: 修改 MoveL.action — 在 Goal 末尾添加 session_id**

在 `src/robot_msgs/action/MoveL.action` 的 Goal 部分，`float64 finger_width` 之后添加 `string session_id`。

- [ ] **Step 3: 创建 GoHome.action**

`src/robot_msgs/action/GoHome.action`:
```
# Goal
float64 speed_ratio
string session_id
---
# Result
bool success
string message
float64[7] final_joint_angles
float64[6] final_tcp_pose
---
# Feedback
float64 progress
float64[7] current_joint_angles
float64 estimated_time_remaining
```

- [ ] **Step 4: 创建 GraspTask.action**

`src/robot_msgs/action/GraspTask.action`:
```
# Goal
uint8 IDLE = 0
uint8 DETECTING = 1
uint8 APPROACHING = 2
uint8 RE_DETECTING = 3
uint8 DESCENDING = 4
uint8 GRASPING = 5
uint8 LIFTING = 6
uint8 DONE = 7
uint8 ERROR = 8

float64 approach_height
float64[3] grasp_rpy
string session_id
---
# Result
bool success
string message
uint8 final_state
float64[7] final_joint_angles
float64[6] final_tcp_pose
---
# Feedback
uint8 current_state
string state_description
float64 progress
```

- [ ] **Step 5: Commit**

```bash
git add src/robot_msgs/action/
git commit -m "feat(robot_msgs): Action 定义添加 session_id + 新增 GoHome/GraspTask"
```

---

### Task 3: 修改 RobotStatus.msg + 删除旧 Service + 更新 CMakeLists

**Files:**
- Modify: `src/robot_msgs/msg/RobotStatus.msg`
- Delete: `src/robot_msgs/srv/MoveJoint.srv`
- Delete: `src/robot_msgs/srv/MovePose.srv`
- Delete: `src/robot_msgs/srv/MoveLinear.srv`
- Delete: `src/robot_msgs/srv/GoHome.srv`
- Modify: `src/robot_msgs/CMakeLists.txt`

- [ ] **Step 1: 修改 RobotStatus.msg**

将末尾的 `motion_owner` 字段替换为：

```
# 删除以下行：
# uint8 OWNER_NONE=0
# uint8 OWNER_PENDANT=1
# uint8 OWNER_SCRIPT=2
# uint8 motion_owner

# 替换为：
string active_session_id
string active_client_name
bool teaching_mode_active
```

- [ ] **Step 2: 删除 4 个旧 Service 文件**

```bash
rm src/robot_msgs/srv/MoveJoint.srv
rm src/robot_msgs/srv/MovePose.srv
rm src/robot_msgs/srv/MoveLinear.srv
rm src/robot_msgs/srv/GoHome.srv
```

- [ ] **Step 3: 更新 CMakeLists.txt**

在 `rosidl_generate_interfaces()` 中：
- 删除 `"srv/MoveJoint.srv"`, `"srv/MovePose.srv"`, `"srv/MoveLinear.srv"`, `"srv/GoHome.srv"`
- 添加 `"srv/AcquireControl.srv"`, `"srv/ReleaseControl.srv"`, `"srv/RenewLease.srv"`, `"srv/RequestTeachingMode.srv"`
- 添加 `"action/GoHome.action"`, `"action/GraspTask.action"`

- [ ] **Step 4: 编译 robot_msgs**

```bash
source /opt/ros/jazzy/setup.zsh
colcon build --base-paths src --packages-select robot_msgs
```

- [ ] **Step 5: Commit**

```bash
git add -A src/robot_msgs/
git commit -m "feat(robot_msgs): 接口重构完成 — 删除运动 Service，新增租约 Service + GoHome/GraspTask Action + 更新 RobotStatus"
```

---

## Phase 2: robot_controller 后端层

### Task 4: 创建 LeaseManager 类

**Files:**
- Create: `src/robot_controller/include/robot_controller/nodes/lease_manager.hpp`
- Create: `src/robot_controller/src/nodes/lease_manager.cpp`
- Create: `src/robot_controller/test/test_lease_manager.cpp`

LeaseManager 管理 session_id 分配、租约超时检测、Teaching Mode 状态。独立于 ROS2 通信层，方便单元测试。

- [ ] **Step 1: 编写 LeaseManager 单元测试**

`src/robot_controller/test/test_lease_manager.cpp`:
```cpp
#include <gtest/gtest.h>
#include "robot_controller/nodes/lease_manager.hpp"

using robot_control::LeaseManager;

TEST(LeaseManagerTest, AcquireAndRelease) {
  LeaseManager lm(std::chrono::seconds(10));
  auto result = lm.acquire("pendant", 0);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(lm.active_session_id(), result.value());
  EXPECT_EQ(lm.active_client_name(), "pendant");
  EXPECT_TRUE(lm.release(result.value()));
  EXPECT_EQ(lm.active_session_id(), "");
}

TEST(LeaseManagerTest, DoubleAcquireFails) {
  LeaseManager lm(std::chrono::seconds(10));
  auto s1 = lm.acquire("pendant", 0);
  ASSERT_TRUE(s1.has_value());
  auto s2 = lm.acquire("python", 0);
  EXPECT_FALSE(s2.has_value());  // 已被占用
}

TEST(LeaseManagerTest, ReleaseAllowsNewAcquire) {
  LeaseManager lm(std::chrono::seconds(10));
  auto s1 = lm.acquire("pendant", 0);
  lm.release(s1.value());
  auto s2 = lm.acquire("python", 0);
  ASSERT_TRUE(s2.has_value());
}

TEST(LeaseManagerTest, RenewExtendsLease) {
  LeaseManager lm(std::chrono::seconds(1));
  auto s = lm.acquire("pendant", 0);
  std::this_thread::sleep_for(std::chrono::milliseconds(800));
  EXPECT_TRUE(lm.renew(s.value(), 0));
}

TEST(LeaseManagerTest, ExpiredLeaseReleased) {
  LeaseManager lm(std::chrono::milliseconds(100));
  auto s = lm.acquire("pendant", 0);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  lm.check_expiry();
  EXPECT_EQ(lm.active_session_id(), "");
}

TEST(LeaseManagerTest, ValidateSession) {
  LeaseManager lm(std::chrono::seconds(10));
  EXPECT_FALSE(lm.is_valid_session("nonexistent"));
  auto s = lm.acquire("pendant", 0);
  EXPECT_TRUE(lm.is_valid_session(s.value()));
  EXPECT_FALSE(lm.is_valid_session("wrong_id"));
}

TEST(LeaseManagerTest, TeachingModeRequiresSession) {
  LeaseManager lm(std::chrono::seconds(10));
  EXPECT_FALSE(lm.request_teaching_mode("nonexistent"));
  auto s = lm.acquire("pendant", 0);
  EXPECT_TRUE(lm.request_teaching_mode(s.value()));
  EXPECT_TRUE(lm.teaching_mode_active());
}

TEST(LeaseManagerTest, TeachingModeExclusive) {
  LeaseManager lm(std::chrono::seconds(10));
  auto s1 = lm.acquire("pendant", 0);
  lm.request_teaching_mode(s1.value());
  lm.release(s1.value());
  // 释放租约后 Teaching Mode 自动退出
  EXPECT_FALSE(lm.teaching_mode_active());
  auto s2 = lm.acquire("python", 0);
  EXPECT_TRUE(lm.request_teaching_mode(s2.value()));
}
```

- [ ] **Step 2: 在 CMakeLists.txt 中添加 test_lease_manager 目标**

在 `src/robot_controller/CMakeLists.txt` 的测试区域添加：

```cmake
ament_add_gtest(test_lease_manager test/test_lease_manager.cpp)
target_link_libraries(test_lease_manager robot_nodes robot_logger::robot_logger_lib)
```

注意：LeaseManager 定义在 `robot_nodes` 中（因为它被 RobotControllerNode 使用），或者如果拆分为独立 target 则链接对应 target。

`src/robot_controller/include/robot_controller/nodes/lease_manager.hpp`:
```cpp
#pragma once
#include <chrono>
#include <optional>
#include <string>
#include <mutex>

namespace robot_control {

struct LeaseInfo {
  std::string session_id;
  std::string client_name;
  std::chrono::steady_clock::time_point expiry;
};

class LeaseManager {
public:
  explicit LeaseManager(std::chrono::seconds default_duration);

  // 租约管理
  std::optional<std::string> acquire(const std::string& client_name, double duration_sec);
  bool release(const std::string& session_id);
  bool renew(const std::string& session_id, double extension_sec);
  void check_expiry();  // 由 10Hz 定时器调用

  // 校验
  bool is_valid_session(const std::string& session_id) const;
  bool has_active_lease() const;

  // 查询
  std::string active_session_id() const;
  std::string active_client_name() const;

  // Teaching Mode
  bool request_teaching_mode(const std::string& session_id);
  bool teaching_mode_active() const;

private:
  std::chrono::seconds default_duration_;
  mutable std::mutex mutex_;
  std::optional<LeaseInfo> active_lease_;
  bool teaching_mode_{false};
  uint64_t next_id_{1};

  std::string generate_session_id();
};

}  // namespace robot_control
```

- [ ] **Step 3: 编写 LeaseManager 实现**

`src/robot_controller/src/nodes/lease_manager.cpp`:
```cpp
#include "robot_controller/nodes/lease_manager.hpp"
#include "robot_logger/logger.hpp"

namespace robot_control {

LeaseManager::LeaseManager(std::chrono::seconds default_duration)
    : default_duration_(default_duration) {}

std::string LeaseManager::generate_session_id() {
  return std::to_string(next_id_++);
}

std::optional<std::string> LeaseManager::acquire(
    const std::string& client_name, double duration_sec) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (active_lease_.has_value()) {
    LOG_WARN("Lease acquire rejected: held by '{}'", active_lease_->client_name);
    return std::nullopt;
  }
  auto duration = duration_sec > 0
      ? std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::duration<double>(duration_sec))
      : default_duration_;
  LeaseInfo info{
      generate_session_id(),
      client_name,
      std::chrono::steady_clock::now() + duration,
  };
  active_lease_ = info;
  LOG_INFO("Lease acquired: session={} client={} duration={}s",
           info.session_id, info.client_name, duration.count());
  return info.session_id;
}

bool LeaseManager::release(const std::string& session_id) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!active_lease_.has_value() || active_lease_->session_id != session_id) {
    return false;
  }
  LOG_INFO("Lease released: session={} client={}",
           active_lease_->session_id, active_lease_->client_name);
  teaching_mode_ = false;
  active_lease_.reset();
  return true;
}

bool LeaseManager::renew(const std::string& session_id, double extension_sec) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!active_lease_.has_value() || active_lease_->session_id != session_id) {
    return false;
  }
  auto extension = extension_sec > 0
      ? std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::duration<double>(extension_sec))
      : default_duration_;
  active_lease_->expiry = std::chrono::steady_clock::now() + extension;
  return true;
}

void LeaseManager::check_expiry() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (active_lease_.has_value() &&
      std::chrono::steady_clock::now() > active_lease_->expiry) {
    LOG_WARN("Lease expired: session={} client={}",
             active_lease_->session_id, active_lease_->client_name);
    teaching_mode_ = false;
    active_lease_.reset();
  }
}

bool LeaseManager::is_valid_session(const std::string& session_id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return active_lease_.has_value() && active_lease_->session_id == session_id;
}

bool LeaseManager::has_active_lease() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return active_lease_.has_value();
}

std::string LeaseManager::active_session_id() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return active_lease_.has_value() ? active_lease_->session_id : "";
}

std::string LeaseManager::active_client_name() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return active_lease_.has_value() ? active_lease_->client_name : "";
}

bool LeaseManager::request_teaching_mode(const std::string& session_id) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!active_lease_.has_value() || active_lease_->session_id != session_id) {
    return false;
  }
  if (teaching_mode_) {
    return false;  // 已有其他 session 在 teaching mode
  }
  teaching_mode_ = true;
  return true;
}

bool LeaseManager::teaching_mode_active() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return teaching_mode_;
}

}  // namespace robot_control
```

- [ ] **Step 4: 编译并运行测试**

```bash
source install/setup.zsh
colcon build --base-paths src --packages-select robot_controller
colcon test --base-paths src --packages-select robot_controller --event-handlers console_direct+
```

- [ ] **Step 5: Commit**

```bash
git add src/robot_controller/include/robot_controller/nodes/lease_manager.hpp src/robot_controller/src/nodes/lease_manager.cpp src/robot_controller/test/test_lease_manager.cpp
git commit -m "feat(robot_controller): 新增 LeaseManager 租约管理器"
```

---

### Task 5: 改造 robot_controller_node — 移除 MotionOwner + 重构 Service handler（原子提交）

**Files:**
- Modify: `src/robot_controller/include/robot_controller/nodes/robot_controller_node.hpp`
- Modify: `src/robot_controller/src/nodes/robot_controller_node.cpp`
- Modify: `src/robot_controller/src/nodes/robot_controller_node_services.cpp`
- Delete: `src/robot_controller/include/robot_controller/nodes/motion_owner.hpp`
- Modify: `src/robot_controller/include/robot_controller/motion/motion_io_bridge.hpp`（submit_trajectory 移除 MotionSource）
- Modify: `src/robot_controller/src/nodes/ros_motion_bridge.cpp`（同步修改）

此任务将 MotionOwner 替换和 Service handler 重构合并为一次原子提交，避免中间编译失败状态。

- [ ] **Step 1: 替换 motion_owner_ 为 LeaseManager**

在 `robot_controller_node.hpp` 中：
- 移除 `#include "robot_controller/nodes/motion_owner.hpp"`
- 添加 `#include "robot_controller/nodes/lease_manager.hpp"`
- 将成员 `std::atomic<MotionOwner> motion_owner_{MotionOwner::kNone}` 替换为 `LeaseManager lease_manager_{std::chrono::seconds{10}}`
- 移除 `claim_ownership()` / `release_ownership()` / `get_motion_owner()` 方法
- 在 Service handler 声明中：删除 `handle_move_joint` / `handle_move_pose` / `handle_move_linear` / `handle_go_home`
- 添加 4 个租约 handler 声明：`handle_acquire_control` / `handle_release_control` / `handle_renew_lease` / `handle_request_teaching_mode`

- [ ] **Step 2: 更新 RosMotionBridge — 移除 MotionSource**

`motion_io_bridge.hpp`:
- 将 `submit_trajectory(...)` 的 `MotionSource source = MotionSource::kApi` 参数移除
- 将 `TrajectoryStartedCallback` 从 `std::function<void(MotionSource)>` 改为 `std::function<void()>`

`ros_motion_bridge.cpp`:
- 对应修改 `submit_trajectory()` 实现

- [ ] **Step 3: 更新 control_loop_tick() 中的所有权检查**

在 `robot_controller_node.cpp` 的 `control_loop_tick()` 中：
- 删除所有 `motion_owner_.store(...)` 调用
- 将 `motion_owner_.load() == MotionOwner::kScript` 检查替换为 `!lease_manager_.teaching_mode_active()`
- 在 100Hz 循环中添加 `lease_manager_.check_expiry()` 调用
- 租约超时时触发减速停车（同 STOP 逻辑）
- `handle_jog_command()`: 检查 `lease_manager_.teaching_mode_active()` 替代原 `motion_owner_ == kScript`
- `external_joint_sub_` 回调: 同样检查 teaching_mode_active

- [ ] **Step 4: 重构 Service handler**

在 `robot_controller_node_services.cpp` 中：

**删除** 4 个运动 handler：`handle_move_joint`, `handle_move_pose`, `handle_move_linear`, `handle_go_home`

**新增** 4 个租约 handler：

```cpp
void RobotControllerNode::handle_acquire_control(
    const robot_msgs::srv::AcquireControl::Request::SharedPtr req,
    robot_msgs::srv::AcquireControl::Response::SharedPtr res) {
  auto session = lease_manager_.acquire(req->client_name, req->lease_duration);
  if (session.has_value()) {
    res->success = true;
    res->session_id = session.value();
    res->lease_timeout = 10.0;
    res->message = "OK";
  } else {
    res->success = false;
    res->message = "Controller is held by: " + lease_manager_.active_client_name();
  }
}

void RobotControllerNode::handle_release_control(
    const robot_msgs::srv::ReleaseControl::Request::SharedPtr req,
    robot_msgs::srv::ReleaseControl::Response::SharedPtr res) {
  res->success = lease_manager_.release(req->session_id);
  res->message = res->success ? "OK" : "Invalid session";
}

void RobotControllerNode::handle_renew_lease(
    const robot_msgs::srv::RenewLease::Request::SharedPtr req,
    robot_msgs::srv::RenewLease::Response::SharedPtr res) {
  res->success = lease_manager_.renew(req->session_id, req->lease_extension);
  res->new_timeout = 10.0;
  res->message = res->success ? "OK" : "Invalid session";
}

void RobotControllerNode::handle_request_teaching_mode(
    const robot_msgs::srv::RequestTeachingMode::Request::SharedPtr req,
    robot_msgs::srv::RequestTeachingMode::Response::SharedPtr res) {
  res->success = lease_manager_.request_teaching_mode(req->session_id);
  res->message = res->success ? "OK" : "No valid lease or already in teaching mode";
}
```

**保留** 的 handler 不变：`handle_solve_ik`, `handle_control_gripper`, `handle_set_speed`, `handle_get_state`, `handle_pendant_set_tcp`, `handle_set_speed_ratio`, `handle_robot_cmd`

**保留 Service 的租约校验**：当前保留的 Service（set_speed, set_tcp 等）的 .srv 定义中没有 session_id 字段，本轮暂不添加租约校验。后续迭代中可在 Request header 中传入 session_id 或为这些 Service 新增带 session_id 的版本。

- [ ] **Step 5: 更新 init() 中的 Service 注册**

- 删除 `move_joint`, `move_pose`, `move_linear`, `go_home` 的 Service Server 创建
- 添加 `acquire_control`, `release_control`, `renew_lease`, `request_teaching_mode` 的 Service Server 创建

- [ ] **Step 6: 更新 publish_status()**

将 RobotStatus 消息中 `motion_owner` 字段替换为 `active_session_id`, `active_client_name`, `teaching_mode_active`。

- [ ] **Step 7: 删除 motion_owner.hpp**

```bash
rm src/robot_controller/include/robot_controller/nodes/motion_owner.hpp
```

- [ ] **Step 8: 编译验证**

```bash
source install/setup.zsh
colcon build --base-paths src --packages-select robot_controller
```

- [ ] **Step 9: 运行现有测试**

```bash
colcon test --base-paths src --packages-select robot_controller --event-handlers console_direct+
```

- [ ] **Step 10: 原子 Commit**

```bash
git add -A src/robot_controller/
git commit -m "refactor(robot_controller): MotionOwner → LeaseManager + 重构 Service handler（原子提交）"
```

---

### Task 6: 实现 Action Server（MoveJ / MoveL / GoHome）

**Files:**
- Create: `src/robot_controller/include/robot_controller/nodes/action_handlers.hpp`
- Create: `src/robot_controller/src/nodes/action_handlers.cpp`
- Modify: `src/robot_controller/src/nodes/robot_controller_node.cpp`（注册 Action Server）

这是最复杂的任务。Action Server 需要在单独线程中执行长时间运动，通过 Feedback 报告进度，支持 Cancel。

- [ ] **Step 1: 创建 Action handler 头文件**

`src/robot_controller/include/robot_controller/nodes/action_handlers.hpp`:
```cpp
#pragma once
#include <rclcpp_action/rclcpp_action.hpp>
#include "robot_msgs/action/move_j.hpp"
#include "robot_msgs/action/move_l.hpp"
#include "robot_msgs/action/go_home.hpp"

namespace robot_control {

class RobotControllerNode;  // forward declare

class ActionHandlers {
public:
  using MoveJ = robot_msgs::action::MoveJ;
  using MoveL = robot_msgs::action::MoveL;
  using GoHome = robot_msgs::action::GoHome;

  explicit ActionHandlers(RobotControllerNode* node);

  // Action Server 回调
  rclcpp_action::GoalResponse handle_movej_goal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const MoveJ::Goal> goal);
  rclcpp_action::CancelResponse handle_movej_cancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveJ>> goal_handle);
  void handle_movej_accepted(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveJ>> goal_handle);

  // MoveL 和 GoHome 同理...
  rclcpp_action::GoalResponse handle_movel_goal(...);
  rclcpp_action::CancelResponse handle_movel_cancel(...);
  void handle_movel_accepted(...);

  rclcpp_action::GoalResponse handle_gohome_goal(...);
  rclcpp_action::CancelResponse handle_gohome_cancel(...);
  void handle_gohome_accepted(...);

private:
  RobotControllerNode* node_;
};

}  // namespace robot_control
```

- [ ] **Step 2: 实现 Action handler**

关键实现逻辑（以 MoveJ 为例）：

1. **Goal 回调**: 校验 session_id（`lease_manager_.is_valid_session(goal->session_id)`），检查状态机为 IDLE
2. **Accept 回调**: 启动执行线程
3. **执行线程**:
   - 调用 `controller_->moveJ()`（现有运动控制器）
   - 在轨迹执行期间，通过 `SetpointGenerator` 的进度信息发布 Feedback
   - 检查 `goal_handle->is_canceling()`，若取消则减速停车
   - 完成时发布 Result
4. **Cancel 回调**: 设置取消标志，让执行线程自行减速停车

Feedback 发布策略：利用现有 `SetpointGenerator::progress()` 返回的进度信息，在 100Hz 控制循环中或独立线程中发布 Feedback。

**RobotCmd STOP 与 Action 取消的联动**：
- `handle_robot_cmd()` 中的 STOP 命令需同时取消正在执行的 Action（调用 `goal_handle->cancel()`）
- E-STOP 走现有 FAULT 流程，不走 Action 取消
- Action cancel 回调中执行平滑减速停车（同 STOP 逻辑）

- [ ] **Step 3: 在 RobotControllerNode::init() 中注册 Action Server**

```cpp
// Action Server 注册
action_handlers_ = std::make_unique<ActionHandlers>(this);

movej_action_server_ = rclcpp_action::create_server<MoveJ>(
    shared_from_this(), "~/move_j",
    [this](auto uuid, auto goal) { return action_handlers_->handle_movej_goal(uuid, goal); },
    [this](auto goal_handle) { return action_handlers_->handle_movej_cancel(goal_handle); },
    [this](auto goal_handle) { action_handlers_->handle_movej_accepted(goal_handle); });

movel_action_server_ = rclcpp_action::create_server<MoveL>(
    shared_from_this(), "~/move_l", ...);

gohome_action_server_ = rclcpp_action::create_server<GoHome>(
    shared_from_this(), "~/go_home", ...);
```

- [ ] **Step 4: 编译验证**

```bash
source install/setup.zsh
colcon build --base-paths src --packages-select robot_controller
```

- [ ] **Step 5: Commit**

```bash
git add src/robot_controller/include/robot_controller/nodes/action_handlers.hpp src/robot_controller/src/nodes/action_handlers.cpp
git commit -m "feat(robot_controller): 实现 MoveJ/MoveL/GoHome Action Server"
```

---

### Task 7: 更新 IRobotController 接口 — 移除 MotionSource，添加租约方法

**Files:**
- Modify: `src/robot_controller/include/robot_controller/motion/i_robot_controller.hpp`
- Modify: `src/robot_controller/include/robot_controller/motion/robot_motion_controller.hpp`
- Modify: `src/robot_controller/src/motion/robot_motion_controller.cpp`

- [ ] **Step 1: 修改 i_robot_controller.hpp**

精确变更清单（对照当前代码行号）：

| 行号 | 变更 |
|------|------|
| 8 | `#include "robot_controller/nodes/motion_owner.hpp"` → 删除（不再需要 MotionSource 枚举） |
| 32-37 | `move_to_pose(...)` 签名不变（保留 `steps` 和 `step_time` 参数） |
| 40-43 | `move_linear(...)` 签名不变（保留 `finger = -1.0`） |
| 68-71 | `lookup_transform(...)` 签名不变（保留 `timeout = 1.0`） |
| 78-81 | `moveJ(xyz, rpy, ...)` 移除末尾 `MotionSource source = MotionSource::kApi` 参数 |
| 84-87 | `moveL(xyz, rpy, ...)` 移除末尾 `MotionSource source = MotionSource::kApi` 参数 |

新增方法（在 `get_speed()` 之后）：

```cpp
  // ===== 租约管理 =====
  virtual bool acquire_control(const std::string& client_name, double lease_duration = 0) = 0;
  virtual void release_control() = 0;
  virtual bool renew_lease() = 0;
  virtual std::string session_id() const = 0;

  // ===== Action 进度回调 =====
  using ProgressCallback = std::function<void(double progress)>;
  virtual void set_progress_callback(ProgressCallback cb) = 0;
```

- [ ] **Step 2: 同步修改 RobotMotionController 声明和实现**

在 `robot_motion_controller.hpp` 和 `.cpp` 中：
- `moveJ(xyz, rpy, ...)` 和 `moveL(xyz, rpy, ...)` 移除 `MotionSource source` 参数
- `moveJ_internal(...)` 移除 `MotionSource source` 参数
- 添加租约和进度回调方法的空实现（throw "not supported by motion controller" — MotionController 是嵌入式实现，不支持远程租约）

- [ ] **Step 2: 编译验证（预期部分失败 — RobotMotionController 也需更新）**

更新 `robot_motion_controller.hpp/cpp` 中对 `moveJ`/`moveL` 的实现签名，移除 `MotionSource` 参数。

- [ ] **Step 3: Commit**

```bash
git add src/robot_controller/include/robot_controller/motion/i_robot_controller.hpp src/robot_controller/src/motion/robot_motion_controller.cpp src/robot_controller/include/robot_controller/motion/robot_motion_controller.hpp
git commit -m "refactor(robot_controller): IRobotController 移除 MotionSource，新增租约/进度回调方法"
```

---

### Task 8: 创建 robot_client CMake Target（ActionRobotController + RobotClient）

**Files:**
- Create: `src/robot_controller/include/robot_controller/client/action_robot_controller.hpp`
- Create: `src/robot_controller/src/client/action_robot_controller.cpp`
- Create: `src/robot_controller/include/robot_controller/client/robot_client.hpp`
- Create: `src/robot_controller/src/client/robot_client.cpp`
- Modify: `src/robot_controller/CMakeLists.txt`

- [ ] **Step 1: 实现 ActionRobotController**

`ActionRobotController` 实现更新后的 `IRobotController` 接口：
- 运动方法（moveJ/moveL/go_home/set_arm 等）通过 Action Client 调用
- 配置/查询方法（set_speed/get_state/set_tcp 等）通过 Service Client 调用
- 租约方法通过 Service Client 调用 AcquireControl/ReleaseControl/RenewLease
- 后台线程持续续租（每 3 秒）

- [ ] **Step 2: 实现 RobotClient**

`RobotClient` 是 `rclcpp::Node` 子类，持有 `ActionRobotController` 实例。工厂模式 `create()`。

- [ ] **Step 3: 更新 CMakeLists.txt 添加 robot_client target**

在 `src/robot_controller/CMakeLists.txt` 中添加第 4 个 library target：

```cmake
# robot_client - Action/Service Client 封装
add_library(robot_client SHARED
  src/client/action_robot_controller.cpp
  src/client/robot_client.cpp
)
target_include_directories(robot_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(robot_client PUBLIC
  robot_motion
  ${rclcpp_TARGETS}
  ${robot_msgs_TARGETS}
)
# 注意：不链接 robot_nodes，避免循环依赖
```

并将 `robot_client` 添加到现有 install 和 export 块中：

```cmake
# 在现有 install(TARGETS ...) 列表中添加 robot_client
install(TARGETS robot_kinematics robot_motion robot_nodes robot_client
  EXPORT robot_controller_export
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
```

确保 `ament_export_targets(robot_controller_export HAS_LIBRARY_TARGET)` 已存在（覆盖所有 4 个 target）。下游包使用 `robot_controller::robot_client` 引用。

- [ ] **Step 4: 编译验证**

```bash
source install/setup.zsh
colcon build --base-paths src --packages-select robot_controller
```

- [ ] **Step 5: Commit**

```bash
git add src/robot_controller/include/robot_controller/client/ src/robot_controller/src/client/ src/robot_controller/CMakeLists.txt
git commit -m "feat(robot_controller): 新增 robot_client CMake target（ActionRobotController + RobotClient）"
```

---

### Task 9: 全量编译验证 Phase 2

- [ ] **Step 1: 运行全部 robot_controller 测试**

```bash
source install/setup.zsh
colcon build --base-paths src --packages-select robot_controller
colcon test --base-paths src --packages-select robot_controller --event-handlers console_direct+
```

- [ ] **Step 2: 全量编译（robot_msgs + robot_controller）**

```bash
colcon build --base-paths src --packages-up-to robot_controller
```

注意：此时 robot_tasks, robot_hmi 等下游包会编译失败（接口变更），这是预期行为，在 Phase 3-4 修复。

---

## Phase 3: robot_tasks 编排层

### Task 10: 更新 robot_tasks 依赖 + 适配 IRobotController 变更

**Files:**
- Modify: `src/robot_tasks/CMakeLists.txt`
- Modify: `src/robot_tasks/package.xml`
- Modify: `src/robot_tasks/include/robot_tasks/grasp_task_manager.hpp`
- Modify: `src/robot_tasks/src/grasp_task_manager.cpp`

- [ ] **Step 1: 更新 CMakeLists.txt**

- 将 `find_package(robot_api_cpp REQUIRED)` 替换为 `find_package(robot_controller REQUIRED)`
- 将 `robot_api_cpp::robot_api_client_lib` 替换为 `robot_controller::robot_client`
- 添加 `find_package(robot_msgs REQUIRED)` 和链接

- [ ] **Step 2: 更新 package.xml**

- 将 `<depend>robot_api_cpp</depend>` 替换为 `<depend>robot_controller</depend>`

- [ ] **Step 3: 更新 GraspTaskManager**

- 头文件 include 从 `robot_api/...` 改为 `robot_controller/client/...`
- 如果 `moveJ`/`moveL` 的 `MotionSource` 参数在 GraspTaskManager 中有使用，移除它
- 确认所有 `IRobotController` 方法调用与新接口兼容

- [ ] **Step 4: 编译验证**

```bash
source install/setup.zsh
colcon build --base-paths src --packages-select robot_tasks
```

- [ ] **Step 5: Commit**

```bash
git add src/robot_tasks/
git commit -m "refactor(robot_tasks): 依赖从 robot_api_cpp 切换到 robot_controller::robot_client"
```

---

### Task 11: 创建 GraspTask Action Server（robot_tasks_node）

**Files:**
- Create: `src/robot_tasks/include/robot_tasks/grasp_task_node.hpp`
- Create: `src/robot_tasks/src/grasp_task_node.cpp`
- Create: `src/robot_tasks/src/grasp_task_main.cpp`
- Modify: `src/robot_tasks/CMakeLists.txt`

- [ ] **Step 1: 创建 GraspTaskNode**

`grasp_task_node.hpp`:
```cpp
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "robot_msgs/action/grasp_task.hpp"
#include "robot_controller/client/robot_client.hpp"
#include "robot_tasks/grasp_task_manager.hpp"

namespace robot_tasks {

class GraspTaskNode : public rclcpp::Node {
public:
  static std::shared_ptr<GraspTaskNode> create();

private:
  GraspTaskNode();
  void init();

  using GraspTask = robot_msgs::action::GraspTask;

  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const GraspTask::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<GraspTask>> goal_handle);
  void handle_accepted(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<GraspTask>> goal_handle);
  void execute(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<GraspTask>> goal_handle);

  rclcpp_action::Server<GraspTask>::SharedPtr action_server_;
  std::shared_ptr<robot_control::RobotClient> robot_client_;
  std::unique_ptr<GraspTaskManager> task_manager_;
};

}  // namespace robot_tasks
```

- [ ] **Step 2: 实现执行逻辑**

核心逻辑：在 `execute()` 中调用 `GraspTaskManager::run()`，通过轮询检查 `goal_handle->is_canceling()` 支持取消，每阶段切换时发布 Feedback。

需要适配 `GraspTaskManager::run()` 使其支持取消检查回调。可选方案：
1. 传入 `std::function<bool()> cancel_check` 回调
2. 或在 `IRobotController` 层面通过 Action cancel 自动取消底层运动

- [ ] **Step 3: 添加可执行入口和 CMake target**

`grasp_task_main.cpp`:
```cpp
#include <rclcpp/rclcpp.hpp>
#include "robot_tasks/grasp_task_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = robot_tasks::GraspTaskNode::create();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

在 CMakeLists.txt 中添加 `robot_tasks_node` 可执行 target。

- [ ] **Step 4: 编译验证**

```bash
source install/setup.zsh
colcon build --base-paths src --packages-select robot_tasks
```

- [ ] **Step 5: Commit**

```bash
git add src/robot_tasks/
git commit -m "feat(robot_tasks): 新增 GraspTask Action Server（robot_tasks_node）"
```

---

## Phase 4: 前端迁移（三个包可并行）

### Task 12: robot_hmi — PendantNode 改用 RobotClient

**Files:**
- Modify: `src/robot_hmi/CMakeLists.txt`
- Modify: `src/robot_hmi/package.xml`
- Modify: `src/robot_hmi/include/robot_hmi/pendant_node.hpp`
- Modify: `src/robot_hmi/src/pendant_node.cpp`

- [ ] **Step 1: 更新依赖**

CMakeLists.txt:
- 添加 `find_package(robot_controller REQUIRED)`
- 添加 `robot_controller::robot_client` 到 target_link_libraries

package.xml:
- 添加 `<depend>robot_controller</depend>`

- [ ] **Step 2: 重构 PendantNode**

核心变化：
- PendantNode 内部持有 `std::shared_ptr<robot_api::RobotClient>` 替代直接创建 Service Client
- Service 调用委托给 `RobotClient::get_controller()`
- 启动时 `acquire_control("pendant")`，3 秒续租，关闭时 `release_control()`
- Action 调用通过 `ActionRobotController` 的 `moveJ()`/`moveL()` 方法
- 进度反馈通过 `set_progress_callback()` 回调到 Qt 主线程
- Jog/流控 Topic 和图像订阅仍由 PendantNode 直接管理
- 调用 `RequestTeachingMode` 进入 Jog 模式

- [ ] **Step 3: 编译验证**

```bash
source install/setup.zsh
colcon build --base-paths src --packages-select robot_hmi
```

- [ ] **Step 4: Commit**

```bash
git add src/robot_hmi/
git commit -m "refactor(robot_hmi): PendantNode 改用 RobotClient + 租约管理"
```

---

### Task 13: robot_api_python — pybind11 重绑

**Files:**
- Modify: `src/robot_api_python/CMakeLists.txt`
- Modify: `src/robot_api_python/package.xml`
- Modify: `src/robot_api_python/src/bindings.cpp`

- [ ] **Step 1: 更新依赖**

- `robot_api_cpp` → `robot_controller`
- 链接 `robot_controller::robot_client`

- [ ] **Step 2: 更新 bindings.cpp**

- 将 `#include "robot_api/robot_client.hpp"` 改为 `#include "robot_controller/client/robot_client.hpp"`
- 将 `#include "robot_api/service_robot_controller.hpp"` 改为 `#include "robot_controller/client/action_robot_controller.hpp"`
- 移除已弃用 `RobotControllerNode` 的 Python 绑定（如果 bindings.cpp 中有暴露）
- 绑定新的 `ActionRobotController` 方法（acquire_control, release_control, renew_lease, session_id, set_progress_callback）

- [ ] **Step 3: 编译验证**

```bash
source install/setup.zsh
colcon build --base-paths src --packages-select robot_api_python
```

- [ ] **Step 4: Commit**

```bash
git add src/robot_api_python/
git commit -m "refactor(robot_api_python): pybind11 重绑到 robot_controller::robot_client"
```

---

### Task 14: robot_demos — 更新调用方式

**Files:**
- Modify: `src/robot_demos/CMakeLists.txt`
- Modify: `src/robot_demos/package.xml`
- Modify: `src/robot_demos/demo/demo_vision_grasp.cpp`
- Modify: `src/robot_demos/demo/demo_vision_diagnostic.cpp`
- Verify: `src/robot_demos/demo/demo_grasp_tcp.cpp`（内嵌节点，需验证是否引用了 motion_owner.hpp）
- Verify: `src/robot_demos/test/test_robot_node.cpp`（调用 go_home，需适配）

- [ ] **Step 1: 更新依赖和 include**

CMakeLists.txt:
- `find_package(robot_api_cpp REQUIRED)` → `find_package(robot_controller REQUIRED)`
- `robot_api_cpp::robot_api_client_lib` → `robot_controller::robot_client`

package.xml:
- `<depend>robot_api_cpp</depend>` → `<depend>robot_controller</depend>`

demo_vision_grasp.cpp / demo_vision_diagnostic.cpp:
- `#include "robot_api/robot_client.hpp"` → `#include "robot_controller/client/robot_client.hpp"`

- [ ] **Step 2: 更新 demo 代码**

- 使用新接口前先调用 `acquire_control("demo")`
- 退出前调用 `release_control()`
- 移除 `MotionSource` 参数（如有使用）

- [ ] **Step 3: 验证 demo_grasp_tcp 和 test_robot_node**

`demo_grasp_tcp.cpp` 直接链接 `robot_controller::robot_nodes`（内嵌控制器），不走 Service/Action。需要验证：
- 是否引用了 `motion_owner.hpp`（已删除），如有则移除
- `go_home()` 调用签名是否兼容

`test_robot_node.cpp` 同理，调用 `controller->go_home()` 需确认签名兼容。

- [ ] **Step 4: 编译验证**

```bash
source install/setup.zsh
colcon build --base-paths src --packages-select robot_demos
```

- [ ] **Step 5: Commit**

```bash
git add src/robot_demos/
git commit -m "refactor(robot_demos): 改用 robot_controller::robot_client + 适配接口变更"
```

---

## Phase 5: 清理

### Task 15: 删除 robot_api_cpp + 更新 launch 文件

**Files:**
- Delete: `src/robot_api_cpp/`（整个目录）
- Modify: `src/robot_bringup/launch/*.py`（如果引用了 robot_api_cpp 相关启动）
- Modify: 各包 `CLAUDE.md`

- [ ] **Step 1: 删除 robot_api_cpp**

```bash
rm -rf src/robot_api_cpp/
```

- [ ] **Step 2: 全量编译验证**

```bash
source install/setup.zsh
colcon build --base-paths src
```

- [ ] **Step 3: 运行全部测试**

```bash
colcon test --base-paths src --event-handlers console_direct+
```

- [ ] **Step 4: 更新 CLAUDE.md 文件**

更新以下文件的包结构、依赖关系、接口描述：
- `CLAUDE.md`（根目录）
- `src/robot_controller/CLAUDE.md`
- `src/robot_tasks/CLAUDE.md`
- `src/robot_hmi/CLAUDE.md`
- `src/robot_api_python/CLAUDE.md`
- `src/robot_demos/CLAUDE.md`

删除 `src/robot_api_cpp/CLAUDE.md`。

- [ ] **Step 5: Commit**

```bash
git add -A
git commit -m "chore: 删除 robot_api_cpp，更新 CLAUDE.md，重构完成"
```
