# Pendant Interface Layer Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Create a standardized ROS2 interface package and extend the controller node with Action servers, state machine, jog watchdog, and status publishing for teaching pendant integration.

**Architecture:** A new `arm_control_interfaces` package defines Actions (MoveJ, MoveL), Services (SetTCP, SetSpeedRatio, RobotCmd), and Messages (RobotStatus, JogCommand). The existing `RobotControllerNode` is extended with a `RobotState` state machine, action servers, jog subscription with 200ms watchdog timer, and a 10Hz status publisher. A non-blocking `TrajectoryExecutor` is added to the `robot_nodes` target for Action progress feedback. Both MoveJ and MoveL use the `TrajectoryExecutor` for non-blocking execution with STOP/E-STOP interruption support.

**Tech Stack:** ROS2 Jazzy, rclcpp_action, C++17, ament_cmake

---

## File Map

### New files (created in order)

| File | Responsibility |
|------|---------------|
| `src/arm_control_interfaces/action/MoveJ.action` | MoveJ action definition |
| `src/arm_control_interfaces/action/MoveL.action` | MoveL action definition |
| `src/arm_control_interfaces/srv/SetTCP.srv` | Set TCP frame service |
| `src/arm_control_interfaces/srv/SetSpeedRatio.srv` | Global speed limit service |
| `src/arm_control_interfaces/srv/RobotCmd.srv` | Stop/e-stop/clear fault commands |
| `src/arm_control_interfaces/msg/RobotStatus.msg` | Robot state + telemetry |
| `src/arm_control_interfaces/msg/JogCommand.msg` | Cartesian jog velocity command |
| `src/arm_control_interfaces/CMakeLists.txt` | Build config for interface package |
| `src/arm_control_interfaces/package.xml` | Package manifest |
| `src/robot_control_cpp/include/robot_control_cpp/nodes/robot_state.hpp` | RobotState enum + transition validation |
| `src/robot_control_cpp/src/nodes/robot_state.cpp` | State machine logic |
| `src/robot_control_cpp/include/robot_control_cpp/nodes/trajectory_executor.hpp` | Non-blocking trajectory executor |
| `src/robot_control_cpp/src/nodes/trajectory_executor.cpp` | Trajectory executor implementation |

### Modified files

| File | Changes |
|------|---------|
| `src/robot_control_cpp/include/robot_control_cpp/nodes/robot_controller_node.hpp` | Add state machine, action servers, services, jog, status publisher members |
| `src/robot_control_cpp/src/nodes/robot_controller_node.cpp` | Implement all new callbacks, wire up init() |
| `src/robot_control_cpp/CMakeLists.txt` | Add `arm_control_interfaces` dep, `rclcpp_action`, new source files |
| `src/robot_control_cpp/package.xml` | Add `arm_control_interfaces` and `rclcpp_action` deps |

---

### Task 1: Create `arm_control_interfaces` package

**Files:**
- Create: `src/arm_control_interfaces/action/MoveJ.action`
- Create: `src/arm_control_interfaces/action/MoveL.action`
- Create: `src/arm_control_interfaces/srv/SetTCP.srv`
- Create: `src/arm_control_interfaces/srv/SetSpeedRatio.srv`
- Create: `src/arm_control_interfaces/srv/RobotCmd.srv`
- Create: `src/arm_control_interfaces/msg/RobotStatus.msg`
- Create: `src/arm_control_interfaces/msg/JogCommand.msg`
- Create: `src/arm_control_interfaces/CMakeLists.txt`
- Create: `src/arm_control_interfaces/package.xml`

- [ ] **Step 1: Create package directory structure**

```bash
mkdir -p src/arm_control_interfaces/{action,srv,msg}
```

- [ ] **Step 2: Create MoveJ.action**

```action
# Goal
uint8 JOINT_SPACE = 0
uint8 CARTESIAN = 1
uint8 mode
float64[7] joint_angles
geometry_msgs/Point position
geometry_msgs/Vector3 orientation
float64 speed_ratio
float64 finger_width
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

- [ ] **Step 3: Create MoveL.action**

```action
# Goal
geometry_msgs/Point position
geometry_msgs/Vector3 orientation
string frame
float64 speed_ratio
float64 finger_width
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

- [ ] **Step 4: Create SetTCP.srv**

```srv
string name
---
bool success
string message
```

- [ ] **Step 5: Create SetSpeedRatio.srv**

```srv
float64 ratio
---
bool success
string message
```

- [ ] **Step 6: Create RobotCmd.srv**

```srv
uint8 STOP = 0
uint8 EMERGENCY_STOP = 1
uint8 CLEAR_FAULT = 2
uint8 command
---
bool success
string message
```

- [ ] **Step 7: Create RobotStatus.msg**

```msg
uint8 IDLE = 0
uint8 MOVING = 1
uint8 TEACHING = 2
uint8 STOPPING = 3
uint8 FAULT = 4
uint8 state
float64 speed_ratio
int32 error_code
string error_message
float64[7] joint_angles
float64[6] tcp_pose
float64 finger_width
string tcp_name
bool is_connected
```

- [ ] **Step 8: Create JogCommand.msg**

```msg
uint8 CARTESIAN_JOG = 0
uint8 mode
float64[6] velocity
builtin_interfaces/Time stamp
```

- [ ] **Step 9: Create CMakeLists.txt**

```cmake
cmake_minimum_required(VERSION 3.8)
project(arm_control_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveJ.action"
  "action/MoveL.action"
  "srv/SetTCP.srv"
  "srv/SetSpeedRatio.srv"
  "srv/RobotCmd.srv"
  "msg/RobotStatus.msg"
  "msg/JogCommand.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs action_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
```

- [ ] **Step 10: Create package.xml**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>arm_control_interfaces</name>
  <version>0.1.0</version>
  <description>Standardized ROS2 interfaces for robot arm teaching pendant</description>
  <maintainer email="user@example.com">user</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>

  <depend>builtin_interfaces</depend>
  <depend>geometry_msgs</depend>
  <depend>action_msgs</depend>

  <exec_depend>rosidl_default_runtime</exec_depend>

  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 11: Build and verify**

```bash
colcon build --base-paths src --packages-select arm_control_interfaces
```

Expected: Build succeeds with no errors.

- [ ] **Step 12: Verify generated headers exist**

```bash
source install/setup.zsh
ls install/arm_control_interfaces/include/arm_control_interfaces/action/movej.hpp
ls install/arm_control_interfaces/include/arm_control_interfaces/action/movel.hpp
ls install/arm_control_interfaces/include/arm_control_interfaces/srv/set_tcp.hpp
ls install/arm_control_interfaces/include/arm_control_interfaces/srv/set_speed_ratio.hpp
ls install/arm_control_interfaces/include/arm_control_interfaces/srv/robot_cmd.hpp
ls install/arm_control_interfaces/include/arm_control_interfaces/msg/robot_status.hpp
ls install/arm_control_interfaces/include/arm_control_interfaces/msg/jog_command.hpp
```

Expected: All files exist.

- [ ] **Step 13: Commit**

```bash
git add src/arm_control_interfaces/
git commit -m "feat: add arm_control_interfaces package (Actions, Services, Messages)"
```

---

### Task 2: Create RobotState state machine (ROS-free, compiled into robot_nodes target)

**Note:** `robot_state.hpp/cpp` and `trajectory_executor.hpp/cpp` have zero ROS includes and are functionally Layer 1 code. They are compiled into the `robot_nodes` target (which has ROS deps) rather than creating a separate CMake target, to avoid unnecessary CMake restructuring. If these are later needed by `robot_motion` (Layer 1), they should be moved to a new `robot_common` target at that time.

**Files:**
- Create: `src/robot_control_cpp/include/robot_control_cpp/nodes/robot_state.hpp`
- Create: `src/robot_control_cpp/src/nodes/robot_state.cpp`

```cpp
#pragma once

#include <cstdint>
#include <mutex>
#include <string>
#include <vector>
#include <array>

namespace robot_control {

/// @brief 机器人运行状态
enum class RobotState : uint8_t {
  kIdle = 0,
  kMoving = 1,
  kTeaching = 2,
  kStopping = 3,
  kFault = 4,
};

/// @brief 状态机管理器（零 ROS 依赖，线程安全）
class RobotStateMachine {
public:
  RobotStateMachine() = default;

  /// @brief 获取当前状态
  RobotState state() const;

  /// @brief 尝试状态转换
  /// @return true 转换成功，false 转换被拒绝
  bool transition_to(RobotState target);

  /// @brief 强制设置状态（仅用于 EMERGENCY_STOP）
  void force_state(RobotState target);

  /// @brief 获取状态名（用于日志）
  static const char* state_name(RobotState s);

  /// @brief 获取错误码
  int32_t error_code() const;

  /// @brief 获取错误信息
  std::string error_message() const;

  /// @brief 设置错误
  void set_error(int32_t code, const std::string& message);

  /// @brief 清除错误
  void clear_error();

private:
  mutable std::mutex mutex_;
  RobotState state_{RobotState::kIdle};
  int32_t error_code_{0};
  std::string error_message_;

  static bool is_valid_transition(RobotState from, RobotState to);
};

}  // namespace robot_control
```

- [ ] **Step 2: Create robot_state.cpp**

```cpp
#include "robot_control_cpp/nodes/robot_state.hpp"
#include <stdexcept>

namespace robot_control {

const char* RobotStateMachine::state_name(RobotState s) {
  switch (s) {
    case RobotState::kIdle:     return "IDLE";
    case RobotState::kMoving:   return "MOVING";
    case RobotState::kTeaching: return "TEACHING";
    case RobotState::kStopping: return "STOPPING";
    case RobotState::kFault:    return "FAULT";
    default:                    return "UNKNOWN";
  }
}

bool RobotStateMachine::is_valid_transition(RobotState from, RobotState to) {
  if (to == RobotState::kFault) return true;  // EMERGENCY_STOP from any state
  switch (from) {
    case RobotState::kIdle:
      return to == RobotState::kMoving || to == RobotState::kTeaching;
    case RobotState::kMoving:
      return to == RobotState::kIdle || to == RobotState::kStopping;
    case RobotState::kTeaching:
      return to == RobotState::kIdle || to == RobotState::kStopping;
    case RobotState::kStopping:
      return to == RobotState::kIdle;
    case RobotState::kFault:
      return to == RobotState::kIdle;
    default:
      return false;
  }
}

RobotState RobotStateMachine::state() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

bool RobotStateMachine::transition_to(RobotState target) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!is_valid_transition(state_, target)) {
    return false;
  }
  state_ = target;
  return true;
}

void RobotStateMachine::force_state(RobotState target) {
  std::lock_guard<std::mutex> lock(mutex_);
  state_ = target;
}

int32_t RobotStateMachine::error_code() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return error_code_;
}

std::string RobotStateMachine::error_message() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return error_message_;
}

void RobotStateMachine::set_error(int32_t code, const std::string& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  error_code_ = code;
  error_message_ = message;
}

void RobotStateMachine::clear_error() {
  std::lock_guard<std::mutex> lock(mutex_);
  error_code_ = 0;
  error_message_.clear();
}

}  // namespace robot_control
```

- [ ] **Step 3: Add robot_state.cpp to CMakeLists.txt**

In `src/robot_control_cpp/CMakeLists.txt`, add `src/nodes/robot_state.cpp` to the `robot_nodes` target source list (line 77-80):

```cmake
add_library(robot_nodes SHARED
  src/nodes/robot_controller_node.cpp
  src/nodes/vision_processor_node.cpp
  src/nodes/grasp_task_manager.cpp
  src/nodes/robot_state.cpp
)
```

- [ ] **Step 4: Build to verify**

```bash
source install/setup.zsh
colcon build --base-paths src --packages-select robot_control_cpp
```

Expected: Build succeeds.

- [ ] **Step 5: Commit**

```bash
git add src/robot_control_cpp/include/robot_control_cpp/nodes/robot_state.hpp
git add src/robot_control_cpp/src/nodes/robot_state.cpp
git add src/robot_control_cpp/CMakeLists.txt
git commit -m "feat: add RobotStateMachine with thread-safe transition validation"
```

---

### Task 3: Create non-blocking trajectory executor (Layer 1, no ROS deps)

This enables Action progress feedback by running the trajectory loop in a separate thread with a pollable progress interface.

**Files:**
- Create: `src/robot_control_cpp/include/robot_control_cpp/nodes/trajectory_executor.hpp`
- Create: `src/robot_control_cpp/src/nodes/trajectory_executor.cpp`

- [ ] **Step 1: Create trajectory_executor.hpp**

```cpp
#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

namespace robot_control {

/// @brief 轨迹执行步骤
struct TrajectoryStep {
  std::vector<double> joint_positions;
  double time_from_start;  // seconds
};

/// @brief 非阻塞轨迹执行器（零 ROS 依赖）
/// 在独立线程中执行预计算的轨迹点序列，支持进度查询和取消。
class TrajectoryExecutor {
public:
  /// @brief 构造
  /// @param publish_fn 每步调用：发送关节指令 (positions, finger)
  TrajectoryExecutor(
      std::function<void(const std::vector<double>&, double)> publish_fn);

  ~TrajectoryExecutor();

  /// @brief 启动轨迹执行（非阻塞）
  /// @param trajectory 预计算的轨迹步骤序列
  /// @param finger_width 夹爪宽度
  void start(const std::vector<TrajectoryStep>& trajectory,
             double finger_width);

  /// @brief 查询执行进度
  /// @param progress 输出：0.0-1.0
  /// @param current_angles 输出：当前关节角度
  /// @param time_remaining 输出：预计剩余时间（秒）
  /// @return true 轨迹正在执行
  bool get_progress(double& progress,
                    std::vector<double>& current_angles,
                    double& time_remaining) const;

  /// @brief 取消当前轨迹
  void cancel();

  /// @brief 是否正在执行
  bool is_active() const;

  /// @brief 阻塞等待执行完成
  /// @param timeout 超时时间（秒），0 表示无限等待
  /// @return true 正常完成，false 超时或被取消
  bool wait_for_completion(double timeout = 0.0);

private:
  void execute_loop();

  std::function<void(const std::vector<double>&, double)> publish_fn_;

  std::vector<TrajectoryStep> trajectory_;
  double finger_width_ = 0.0;
  double total_duration_ = 0.0;

  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::thread thread_;
  std::atomic<bool> active_{false};
  std::atomic<bool> cancelled_{false};
  std::atomic<bool> completed_{false};
  size_t current_step_{0};
};

}  // namespace robot_control
```

- [ ] **Step 2: Create trajectory_executor.cpp**

```cpp
#include "robot_control_cpp/nodes/trajectory_executor.hpp"

#include <algorithm>

namespace robot_control {

TrajectoryExecutor::TrajectoryExecutor(
    std::function<void(const std::vector<double>&, double)> publish_fn)
    : publish_fn_(std::move(publish_fn)) {}

TrajectoryExecutor::~TrajectoryExecutor() {
  cancel();
  if (thread_.joinable()) {
    thread_.join();
  }
}

void TrajectoryExecutor::start(const std::vector<TrajectoryStep>& trajectory,
                               double finger_width) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (active_) {
    throw std::runtime_error("TrajectoryExecutor: already active");
  }

  trajectory_ = trajectory;
  finger_width_ = finger_width;
  total_duration_ = trajectory.empty() ? 0.0 : trajectory.back().time_from_start;
  current_step_ = 0;
  active_ = true;
  cancelled_ = false;
  completed_ = false;

  thread_ = std::thread(&TrajectoryExecutor::execute_loop, this);
}

void TrajectoryExecutor::execute_loop() {
  auto start = std::chrono::steady_clock::now();

  for (size_t i = 0; i < trajectory_.size(); ++i) {
    if (cancelled_) break;

    double t = trajectory_[i].time_from_start;
    auto target_time = start + std::chrono::duration<double>(t);
    std::this_thread::sleep_until(target_time);

    if (cancelled_) break;

    publish_fn_(trajectory_[i].joint_positions, finger_width_);

    std::lock_guard<std::mutex> lock(mutex_);
    current_step_ = i;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!cancelled_) {
      completed_ = true;
    }
    active_ = false;
  }
  cv_.notify_all();
}

bool TrajectoryExecutor::get_progress(double& progress,
                                      std::vector<double>& current_angles,
                                      double& time_remaining) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!active_ && !completed_) return false;
  if (trajectory_.empty()) return false;

  size_t step = current_step_;
  double elapsed = (step < trajectory_.size()) ? trajectory_[step].time_from_start : total_duration_;

  progress = (total_duration_ > 1e-12) ? elapsed / total_duration_ : 1.0;
  progress = std::clamp(progress, 0.0, 1.0);

  if (step < trajectory_.size()) {
    current_angles = trajectory_[step].joint_positions;
  } else if (!trajectory_.empty()) {
    current_angles = trajectory_.back().joint_positions;
  }

  time_remaining = std::max(0.0, total_duration_ - elapsed);
  return active_;
}

void TrajectoryExecutor::cancel() {
  cancelled_ = true;
  cv_.notify_all();
}

bool TrajectoryExecutor::is_active() const {
  return active_.load();
}

bool TrajectoryExecutor::wait_for_completion(double timeout) {
  std::unique_lock<std::mutex> lock(mutex_);
  if (timeout <= 0.0) {
    cv_.wait(lock, [this] { return !active_; });
    return completed_.load();
  }
  return cv_.wait_for(lock, std::chrono::duration<double>(timeout),
                      [this] { return !active_; }) &&
         completed_.load();
}

}  // namespace robot_control
```

- [ ] **Step 3: Add trajectory_executor.cpp to CMakeLists.txt**

In `src/robot_control_cpp/CMakeLists.txt`, add to `robot_nodes` source list:

```cmake
add_library(robot_nodes SHARED
  src/nodes/robot_controller_node.cpp
  src/nodes/vision_processor_node.cpp
  src/nodes/grasp_task_manager.cpp
  src/nodes/robot_state.cpp
  src/nodes/trajectory_executor.cpp
)
```

- [ ] **Step 4: Build to verify**

```bash
source install/setup.zsh
colcon build --base-paths src --packages-select robot_control_cpp
```

Expected: Build succeeds.

- [ ] **Step 5: Commit**

```bash
git add src/robot_control_cpp/include/robot_control_cpp/nodes/trajectory_executor.hpp
git add src/robot_control_cpp/src/nodes/trajectory_executor.cpp
git add src/robot_control_cpp/CMakeLists.txt
git commit -m "feat: add non-blocking TrajectoryExecutor for Action progress feedback"
```

---

### Task 4: Add build dependencies for `arm_control_interfaces` and `rclcpp_action`

**Files:**
- Modify: `src/robot_control_cpp/CMakeLists.txt`
- Modify: `src/robot_control_cpp/package.xml`

- [ ] **Step 1: Add CMake dependencies**

In `src/robot_control_cpp/CMakeLists.txt`, after line 22 (`find_package(robot_control_msgs REQUIRED)`), add:

```cmake
find_package(arm_control_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)
```

In the `ament_target_dependencies(robot_nodes ...)` block (line 90-101), add:

```cmake
  rclcpp_action
  arm_control_interfaces
```

- [ ] **Step 2: Add package.xml dependencies**

In `src/robot_control_cpp/package.xml`, after the `<depend>robot_control_msgs</depend>` line, add:

```xml
  <depend>rclcpp_action</depend>
  <depend>arm_control_interfaces</depend>
```

- [ ] **Step 3: Build to verify**

```bash
source install/setup.zsh
colcon build --base-paths src --packages-select arm_control_interfaces robot_control_cpp
```

Expected: Build succeeds.

- [ ] **Step 4: Commit**

```bash
git add src/robot_control_cpp/CMakeLists.txt src/robot_control_cpp/package.xml
git commit -m "build: add arm_control_interfaces and rclcpp_action dependencies"
```

---

### Task 5: Extend RobotControllerNode header with state machine, actions, services, jog, status

**Files:**
- Modify: `src/robot_control_cpp/include/robot_control_cpp/nodes/robot_controller_node.hpp`

- [ ] **Step 1: Add includes**

After the existing `#include <robot_control_msgs/srv/get_robot_state.hpp>` (line 24), add:

```cpp
#include <rclcpp_action/rclcpp_action.hpp>

#include <arm_control_interfaces/action/move_j.hpp>
#include <arm_control_interfaces/action/move_l.hpp>
#include <arm_control_interfaces/srv/set_tcp.hpp>
#include <arm_control_interfaces/srv/set_speed_ratio.hpp>
#include <arm_control_interfaces/srv/robot_cmd.hpp>
#include <arm_control_interfaces/msg/robot_status.hpp>
#include <arm_control_interfaces/msg/jog_command.hpp>

#include "robot_control_cpp/nodes/robot_state.hpp"
#include "robot_control_cpp/nodes/trajectory_executor.hpp"
```

- [ ] **Step 2: Add state machine, action servers, service servers, jog, status members**

After the existing `rclcpp::Service<robot_control_msgs::srv::GetRobotState>::SharedPtr srv_state_;` (line 177), before the `ready_mutex_` block, add:

```cpp
  // === State machine ===
  RobotStateMachine state_machine_;

  // === Trajectory executor ===
  std::unique_ptr<TrajectoryExecutor> trajectory_executor_;

  // === Global speed ratio (atomic for thread-safe access from action threads) ===
  std::atomic<double> global_speed_ratio_{1.0};

  // === Action servers ===
  rclcpp_action::Server<arm_control_interfaces::action::MoveJ>::SharedPtr movej_action_;
  rclcpp_action::Server<arm_control_interfaces::action::MoveL>::SharedPtr movel_action_;

  // === Pendant service servers ===
  rclcpp::Service<arm_control_interfaces::srv::SetTCP>::SharedPtr pendant_set_tcp_srv_;
  rclcpp::Service<arm_control_interfaces::srv::SetSpeedRatio>::SharedPtr set_speed_ratio_srv_;
  rclcpp::Service<arm_control_interfaces::srv::RobotCmd>::SharedPtr robot_cmd_srv_;

  // === Jog + watchdog ===
  rclcpp::Subscription<arm_control_interfaces::msg::JogCommand>::SharedPtr jog_sub_;
  rclcpp::TimerBase::SharedPtr jog_watchdog_timer_;
  rclcpp::Time last_jog_time_;

  // === Status publisher ===
  rclcpp::Publisher<arm_control_interfaces::msg::RobotStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
```

- [ ] **Step 3: Add action/service/jog/status method declarations**

After the existing `handle_get_state` declaration (line 156), add:

```cpp
  // === Action callbacks (MoveJ) ===
  rclcpp_action::GoalResponse handle_movej_goal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const arm_control_interfaces::action::MoveJ::Goal> goal);
  rclcpp_action::CancelResponse handle_movej_cancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<arm_control_interfaces::action::MoveJ>> goal_handle);
  void handle_movej_accepted(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<arm_control_interfaces::action::MoveJ>> goal_handle);

  // === Action callbacks (MoveL) ===
  rclcpp_action::GoalResponse handle_movel_goal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const arm_control_interfaces::action::MoveL::Goal> goal);
  rclcpp_action::CancelResponse handle_movel_cancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<arm_control_interfaces::action::MoveL>> goal_handle);
  void handle_movel_accepted(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<arm_control_interfaces::action::MoveL>> goal_handle);

  // === Pendant service callbacks ===
  void handle_pendant_set_tcp(
      const std::shared_ptr<arm_control_interfaces::srv::SetTCP::Request> req,
      std::shared_ptr<arm_control_interfaces::srv::SetTCP::Response> res);
  void handle_set_speed_ratio(
      const std::shared_ptr<arm_control_interfaces::srv::SetSpeedRatio::Request> req,
      std::shared_ptr<arm_control_interfaces::srv::SetSpeedRatio::Response> res);
  void handle_robot_cmd(
      const std::shared_ptr<arm_control_interfaces::srv::RobotCmd::Request> req,
      std::shared_ptr<arm_control_interfaces::srv::RobotCmd::Response> res);

  // === Jog + watchdog ===
  void handle_jog_command(const arm_control_interfaces::msg::JogCommand::SharedPtr msg);
  void jog_watchdog_callback();

  // === Status publisher ===
  void publish_status();

  // === Emergency stop helper ===
  void emergency_stop();

  // === Action execution threads (joined on destruction) ===
  std::thread movej_thread_;
  std::thread movel_thread_;
  std::atomic<bool> shutdown_{false};
```

- [ ] **Step 6: Add destructor declaration**

After the constructor declaration (line 125-127), add:

```cpp
  ~RobotControllerNode();
```

- [ ] **Step 4: Add public state accessor**

After `get_bridge()` (line 122), add:

```cpp
  /// @brief 获取状态机（只读）
  const RobotStateMachine& state_machine() const { return state_machine_; }
```

- [ ] **Step 5: Build to verify**

```bash
source install/setup.zsh
colcon build --base-paths src --packages-select robot_control_cpp
```

Expected: Build succeeds (compilation will have linker errors for unimplemented methods, that's OK — next task implements them).

- [ ] **Step 6: Commit**

```bash
git add src/robot_control_cpp/include/robot_control_cpp/nodes/robot_controller_node.hpp
git commit -m "feat: extend RobotControllerNode header with state machine, actions, jog, status"
```

---

### Task 6: Implement action callbacks, service callbacks, jog watchdog, and status publisher

This is the largest task. It implements all new callbacks in `robot_controller_node.cpp`.

**Files:**
- Modify: `src/robot_control_cpp/src/nodes/robot_controller_node.cpp`

- [ ] **Step 1: Add includes at top of file**

After `#include "robot_control_cpp/nodes/topic_config.hpp"` (line 3), add:

```cpp
#include "robot_control_cpp/nodes/robot_state.hpp"
#include "robot_control_cpp/nodes/trajectory_executor.hpp"

#include <rclcpp_action/rclcpp_action.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <arm_control_interfaces/action/move_j.hpp>
#include <arm_control_interfaces/action/move_l.hpp>
#include <arm_control_interfaces/srv/set_tcp.hpp>
#include <arm_control_interfaces/srv/set_speed_ratio.hpp>
#include <arm_control_interfaces/srv/robot_cmd.hpp>
#include <arm_control_interfaces/msg/robot_status.hpp>
#include <arm_control_interfaces/msg/jog_command.hpp>
```

- [ ] **Step 2: Initialize TrajectoryExecutor in init()**

After `controller_ = std::make_shared<RobotMotionController>(...)` (line 373-374), add:

```cpp
  // Trajectory executor: wraps bridge_->publish_command for non-blocking execution
  trajectory_executor_ = std::make_unique<TrajectoryExecutor>(
      [this](const std::vector<double>& arm, double finger) {
        bridge_->publish_command(arm, finger);
      });
```

- [ ] **Step 3: Add pendant interface setup at end of init()**

After the existing `RCLCPP_INFO(this->get_logger(), "RobotControllerNode started (with services)");` (line 449), add:

```cpp
  // === Action servers ===
  movej_action_ = rclcpp_action::create_server<arm_control_interfaces::action::MoveJ>(
      this, "~/movej",
      std::bind(&RobotControllerNode::handle_movej_goal, this,
                std::placeholders::_1, std::placeholders::_2),
      std::bind(&RobotControllerNode::handle_movej_cancel, this,
                std::placeholders::_1),
      std::bind(&RobotControllerNode::handle_movej_accepted, this,
                std::placeholders::_1));

  movel_action_ = rclcpp_action::create_server<arm_control_interfaces::action::MoveL>(
      this, "~/movel",
      std::bind(&RobotControllerNode::handle_movel_goal, this,
                std::placeholders::_1, std::placeholders::_2),
      std::bind(&RobotControllerNode::handle_movel_cancel, this,
                std::placeholders::_1),
      std::bind(&RobotControllerNode::handle_movel_accepted, this,
                std::placeholders::_1));

  // === Pendant service servers ===
  pendant_set_tcp_srv_ = create_service<arm_control_interfaces::srv::SetTCP>(
      "~/set_tcp",
      [this](const std::shared_ptr<arm_control_interfaces::srv::SetTCP::Request> req,
             std::shared_ptr<arm_control_interfaces::srv::SetTCP::Response> res) {
        handle_pendant_set_tcp(req, res);
      });

  set_speed_ratio_srv_ = create_service<arm_control_interfaces::srv::SetSpeedRatio>(
      "~/set_speed_ratio",
      [this](const std::shared_ptr<arm_control_interfaces::srv::SetSpeedRatio::Request> req,
             std::shared_ptr<arm_control_interfaces::srv::SetSpeedRatio::Response> res) {
        handle_set_speed_ratio(req, res);
      });

  robot_cmd_srv_ = create_service<arm_control_interfaces::srv::RobotCmd>(
      "~/robot_cmd",
      [this](const std::shared_ptr<arm_control_interfaces::srv::RobotCmd::Request> req,
             std::shared_ptr<arm_control_interfaces::srv::RobotCmd::Response> res) {
        handle_robot_cmd(req, res);
      });

  // === Jog subscription ===
  rclcpp::SubscriptionOptions jog_opts;
  jog_opts.callback_group = state_cbg_;
  jog_sub_ = create_subscription<arm_control_interfaces::msg::JogCommand>(
      "~/jog_command", rclcpp::SensorDataQoS(),
      [this](const arm_control_interfaces::msg::JogCommand::SharedPtr msg) {
        handle_jog_command(msg);
      }, jog_opts);

  // === Jog watchdog (200ms) ===
  jog_watchdog_timer_ = create_wall_timer(
      std::chrono::milliseconds(200),
      [this]() { jog_watchdog_callback(); }, pub_cbg_);

  // === Status publisher (10Hz) ===
  status_pub_ = create_publisher<arm_control_interfaces::msg::RobotStatus>(
      "~/status", 10);
  status_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() { publish_status(); }, pub_cbg_);

  RCLCPP_INFO(this->get_logger(),
              "RobotControllerNode: pendant interface ready (actions, jog, status)");
```

- [ ] **Step 4: Implement MoveJ action callbacks**

Before the closing `}  // namespace robot_control`, add all the following implementations. Starting with MoveJ:

```cpp
// ===== MoveJ Action =====

rclcpp_action::GoalResponse RobotControllerNode::handle_movej_goal(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const arm_control_interfaces::action::MoveJ::Goal> goal) {
  if (state_machine_.state() != RobotState::kIdle) {
    RCLCPP_WARN(this->get_logger(), "MoveJ rejected: robot not IDLE (state=%s)",
                RobotStateMachine::state_name(state_machine_.state()));
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (goal->speed_ratio < 0.0 || goal->speed_ratio > 1.0) {
    RCLCPP_WARN(this->get_logger(), "MoveJ rejected: speed_ratio out of [0,1]");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RobotControllerNode::handle_movej_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<arm_control_interfaces::action::MoveJ>>) {
  if (state_machine_.state() == RobotState::kMoving) {
    state_machine_.transition_to(RobotState::kStopping);
    trajectory_executor_->cancel();
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  return rclcpp_action::CancelResponse::REJECT;
}

void RobotControllerNode::handle_movej_accepted(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<arm_control_interfaces::action::MoveJ>> goal_handle) {
  state_machine_.transition_to(RobotState::kMoving);

  // Run in a detached thread to avoid blocking the executor
  std::thread([this, goal_handle]() {
    auto result = std::make_shared<arm_control_interfaces::action::MoveJ::Result>();
    auto feedback = std::make_shared<arm_control_interfaces::action::MoveJ::Feedback>();

    try {
      auto goal = goal_handle->get_goal();
      double effective_speed = goal->speed_ratio * global_speed_ratio_;

      // Plan trajectory
      std::vector<TrajectoryStep> steps;
      std::vector<double> target_angles;

      if (goal->mode == arm_control_interfaces::action::MoveJ::Goal::CARTESIAN) {
        // Cartesian → IK → joint trajectory
        std::array<double, 3> xyz = {goal->position.x, goal->position.y, goal->position.z};
        std::optional<std::array<double, 3>> rpy;
        rpy = std::array<double, 3>{goal->orientation.x, goal->orientation.y, goal->orientation.z};

        // Use controller's IK (respects TCP offset)
        auto tcp_cfg = profile_.tcp_frames.at(controller_->get_current_tcp());
        Eigen::Matrix4d T_tcp = Eigen::Matrix4d::Identity();
        Eigen::AngleAxisd roll_a(tcp_cfg.offset_rpy[0], Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitch_a(tcp_cfg.offset_rpy[1], Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yaw_a(tcp_cfg.offset_rpy[2], Eigen::Vector3d::UnitZ());
        T_tcp.block<3,3>(0,0) = (yaw_a * pitch_a * roll_a).toRotationMatrix();
        T_tcp(0,3) = tcp_cfg.offset_xyz[0]; T_tcp(1,3) = tcp_cfg.offset_xyz[1]; T_tcp(2,3) = tcp_cfg.offset_xyz[2];

        Eigen::Matrix4d T_target = Eigen::Matrix4d::Identity();
        Eigen::AngleAxisd e_roll(rpy->at(0), Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd e_pitch(rpy->at(1), Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd e_yaw(rpy->at(2), Eigen::Vector3d::UnitZ());
        T_target.block<3,3>(0,0) = (e_yaw * e_pitch * e_roll).toRotationMatrix();
        T_target(0,3) = xyz[0]; T_target(1,3) = xyz[1]; T_target(2,3) = xyz[2];

        Eigen::Matrix4d hand_target = T_target * T_tcp.inverse();
        Eigen::Vector3d h_xyz = hand_target.block<3,1>(0,3);
        Eigen::Vector3d h_rpy = hand_target.block<3,3>(0,0).eulerAngles(0,1,2);
        std::array<double,3> h_xyz_arr = {h_xyz.x(), h_xyz.y(), h_xyz.z()};
        std::array<double,3> h_rpy_arr = {h_rpy.x(), h_rpy.y(), h_rpy.z()};

        auto ik_result = ik_->solve(h_xyz_arr, h_rpy_arr);
        if (!ik_result) {
          result->success = false;
          result->message = "IK solver failed";
          goal_handle->abort(result);
          state_machine_.transition_to(RobotState::kFault);
          state_machine_.set_error(1, "MoveJ: IK failed");
          return;
        }
        target_angles = *ik_result;
      } else {
        // Joint space
        target_angles = {goal->joint_angles.begin(), goal->joint_angles.end()};
      }

      // Plan with speed factor
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

      // Convert to TrajectoryStep
      double dt = ControlConstants::kTrajectoryDt;
      for (size_t i = 0; i < trajectory.size(); ++i) {
        steps.push_back({trajectory[i], static_cast<double>(i) * dt});
      }

      double finger = (goal->finger_width >= 0) ? goal->finger_width
                      : bridge_->get_current_finger();

      trajectory_executor_->start(steps, finger);

      // Feedback loop
      while (trajectory_executor_->is_active()) {
        double progress;
        std::vector<double> cur_angles;
        double time_rem;
        trajectory_executor_->get_progress(progress, cur_angles, time_rem);

        feedback->progress = progress;
        feedback->current_joint_angles = cur_angles;
        feedback->estimated_time_remaining = time_rem;
        goal_handle->publish_feedback(feedback);

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }

      if (trajectory_executor_->wait_for_completion(2.0)) {
        // Confirm the robot actually reached the target
        bridge_->wait_for_motion(
            target_angles, finger,
            ControlConstants::kJointTolerance,
            ControlConstants::kFingerTolerance,
            ControlConstants::kMotionTimeout,
            ControlConstants::kPollInterval,
            ControlConstants::kSettleTime,
            true);

        // Check if emergency stop was triggered during execution
        if (state_machine_.state() == RobotState::kFault) {
          result->success = false;
          result->message = "MoveJ interrupted by EMERGENCY_STOP";
          goal_handle->abort(result);
          return;
        }

        auto final_angles = controller_->get_joint_angles();
        auto final_pose = controller_->get_end_effector_pose();
        result->success = true;
        result->message = "MoveJ completed";
        result->final_joint_angles = final_angles;
        result->final_tcp_pose = {final_pose[0], final_pose[1], final_pose[2],
                                  final_pose[3], final_pose[4], final_pose[5]};
        goal_handle->succeed(result);
        state_machine_.transition_to(RobotState::kIdle);
      } else {
        // Cancelled or timed out — check if it was an emergency stop
        if (state_machine_.state() == RobotState::kFault) {
          result->success = false;
          result->message = "MoveJ interrupted by EMERGENCY_STOP";
          goal_handle->abort(result);
          return;
        }
        result->success = false;
        result->message = "MoveJ cancelled";
        goal_handle->canceled(result);
        state_machine_.transition_to(RobotState::kIdle);
      }

    } catch (const std::exception& e) {
      result->success = false;
      result->message = std::string("MoveJ error: ") + e.what();
      goal_handle->abort(result);
      state_machine_.transition_to(RobotState::kFault);
      state_machine_.set_error(2, result->message);
    }
  }).detach();
}
```

**Note:** The IK calculation in MoveJ duplicates logic from `RobotMotionController`. A cleaner approach is to call `controller_->moveJ()` in blocking mode from the action thread. However, that prevents progress feedback. The above approach calls the planner directly and uses `TrajectoryExecutor`. This is intentionally verbose to keep the bridge pattern intact — the action callback uses Layer 1 primitives (IK + planner + executor), not the controller's blocking methods.

**Important deviation from spec:** The spec proposed refactoring `TrajectoryPlanner::plan_joint` to return `std::vector<TrajectoryPoint>` with timing. This plan does NOT modify the existing planner. Instead, it computes `time_from_start` as `i * dt` in the callback, which is correct because `plan_joint` produces uniform dt samples. This avoids a breaking change to the Layer 1 API.

- [ ] **Step 5: Implement MoveL action callbacks**

```cpp
// ===== MoveL Action =====

rclcpp_action::GoalResponse RobotControllerNode::handle_movel_goal(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const arm_control_interfaces::action::MoveL::Goal> goal) {
  if (state_machine_.state() != RobotState::kIdle) {
    RCLCPP_WARN(this->get_logger(), "MoveL rejected: robot not IDLE (state=%s)",
                RobotStateMachine::state_name(state_machine_.state()));
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (goal->speed_ratio < 0.0 || goal->speed_ratio > 1.0) {
    RCLCPP_WARN(this->get_logger(), "MoveL rejected: speed_ratio out of [0,1]");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RobotControllerNode::handle_movel_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<arm_control_interfaces::action::MoveL>>) {
  if (state_machine_.state() == RobotState::kMoving) {
    state_machine_.transition_to(RobotState::kStopping);
    trajectory_executor_->cancel();
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  return rclcpp_action::CancelResponse::REJECT;
}

void RobotControllerNode::handle_movel_accepted(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<arm_control_interfaces::action::MoveL>> goal_handle) {
  state_machine_.transition_to(RobotState::kMoving);

  std::thread([this, goal_handle]() {
    auto result = std::make_shared<arm_control_interfaces::action::MoveL::Result>();
    auto feedback = std::make_shared<arm_control_interfaces::action::MoveL::Feedback>();

    try {
      auto goal = goal_handle->get_goal();
      double effective_speed = goal->speed_ratio * global_speed_ratio_.load();

      std::array<double, 3> xyz = {goal->position.x, goal->position.y, goal->position.z};
      std::optional<std::array<double, 3>> rpy;
      rpy = std::array<double, 3>{goal->orientation.x, goal->orientation.y, goal->orientation.z};

      // Plan Cartesian linear trajectory using S-curve planner (same as RobotMotionController::moveL)
      auto current_pose = controller_->get_end_effector_pose();
      Eigen::Vector3d start_pos(current_pose[0], current_pose[1], current_pose[2]);
      Eigen::Vector3d end_pos(xyz[0], xyz[1], xyz[2]);
      double total_dist = (end_pos - start_pos).norm();

      if (total_dist < 1e-6) {
        result->success = true;
        result->message = "MoveL: already at target";
        goal_handle->succeed(result);
        state_machine_.transition_to(RobotState::kIdle);
        return;
      }

      // Plan Cartesian S-curve
      double speed_factor = effective_speed * (controller_->get_speed(MotionMode::kMoveL) / 100.0);
      MotionLimits cart_cfg{
          profile_.cartesian_limits.max_vel * speed_factor,
          profile_.cartesian_limits.max_acc * speed_factor,
          profile_.cartesian_limits.max_jerk * speed_factor};

      auto cart_traj = SCurvePlanner::plan(
          0.0, total_dist, cart_cfg, ControlConstants::kTrajectoryDt);

      // Orientation interpolation
      Eigen::Vector3d start_rpy(current_pose[3], current_pose[4], current_pose[5]);
      Eigen::Quaterniond start_quat =
          (Eigen::AngleAxisd(start_rpy.x(), Eigen::Vector3d::UnitX()) *
           Eigen::AngleAxisd(start_rpy.y(), Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(start_rpy.z(), Eigen::Vector3d::UnitZ()));
      Eigen::Quaterniond end_quat =
          (Eigen::AngleAxisd(rpy->at(0), Eigen::Vector3d::UnitX()) *
           Eigen::AngleAxisd(rpy->at(1), Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(rpy->at(2), Eigen::Vector3d::UnitZ()));

      // TCP offset for IK
      auto tcp_cfg = profile_.tcp_frames.at(controller_->get_current_tcp());
      Eigen::Matrix4d T_tcp = Eigen::Matrix4d::Identity();
      Eigen::AngleAxisd e_roll(tcp_cfg.offset_rpy[0], Eigen::Vector3d::UnitX());
      Eigen::AngleAxisd e_pitch(tcp_cfg.offset_rpy[1], Eigen::Vector3d::UnitY());
      Eigen::AngleAxisd e_yaw(tcp_cfg.offset_rpy[2], Eigen::Vector3d::UnitZ());
      T_tcp.block<3,3>(0,0) = (e_yaw * e_pitch * e_roll).toRotationMatrix();
      T_tcp(0,3) = tcp_cfg.offset_xyz[0]; T_tcp(1,3) = tcp_cfg.offset_xyz[1]; T_tcp(2,3) = tcp_cfg.offset_xyz[2];

      // Pre-compute all IK solutions
      std::vector<TrajectoryStep> steps;
      double finger = (goal->finger_width >= 0) ? goal->finger_width : bridge_->get_current_finger();

      for (const auto& pt : cart_traj) {
        double alpha = (total_dist > 1e-12) ? pt.pos / total_dist : 1.0;
        alpha = std::clamp(alpha, 0.0, 1.0);

        Eigen::Vector3d interp_pos = start_pos + alpha * (end_pos - start_pos);
        Eigen::Quaterniond interp_quat = start_quat.slerp(alpha, end_quat);

        Eigen::Matrix4d T_target = Eigen::Matrix4d::Identity();
        T_target.block<3,3>(0,0) = interp_quat.toRotationMatrix();
        T_target(0,3) = interp_pos.x(); T_target(1,3) = interp_pos.y(); T_target(2,3) = interp_pos.z();

        Eigen::Matrix4d hand_target = T_target * T_tcp.inverse();
        Eigen::Vector3d h_xyz = hand_target.block<3,1>(0,3);
        Eigen::Vector3d h_rpy = hand_target.block<3,3>(0,0).eulerAngles(0,1,2);

        auto ik_result = ik_->solve(
            std::array<double,3>{h_xyz.x(), h_xyz.y(), h_xyz.z()},
            std::array<double,3>{h_rpy.x(), h_rpy.y(), h_rpy.z()});
        if (!ik_result) {
          throw std::runtime_error("MoveL: IK failed at trajectory point");
        }
        steps.push_back({*ik_result, pt.t});
      }

      trajectory_executor_->start(steps, finger);

      // Feedback loop
      while (trajectory_executor_->is_active()) {
        double progress;
        std::vector<double> cur_angles;
        double time_rem;
        trajectory_executor_->get_progress(progress, cur_angles, time_rem);

        feedback->progress = progress;
        feedback->current_joint_angles = cur_angles;
        feedback->estimated_time_remaining = time_rem;
        goal_handle->publish_feedback(feedback);

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }

      if (trajectory_executor_->wait_for_completion(2.0)) {
        // Confirm the robot reached the target
        if (!steps.empty()) {
          bridge_->wait_for_motion(
              steps.back().joint_positions, finger,
              ControlConstants::kJointTolerance,
              ControlConstants::kFingerTolerance,
              ControlConstants::kMotionTimeout,
              ControlConstants::kPollInterval,
              ControlConstants::kSettleTime,
              true);
        }

        // Check if emergency stop was triggered during execution
        if (state_machine_.state() == RobotState::kFault) {
          result->success = false;
          result->message = "MoveL interrupted by EMERGENCY_STOP";
          goal_handle->abort(result);
          return;
        }

        auto final_angles = controller_->get_joint_angles();
        auto final_pose = controller_->get_end_effector_pose();
        result->success = true;
        result->message = "MoveL completed";
        result->final_joint_angles = final_angles;
        result->final_tcp_pose = {final_pose[0], final_pose[1], final_pose[2],
                                  final_pose[3], final_pose[4], final_pose[5]};
        goal_handle->succeed(result);
        state_machine_.transition_to(RobotState::kIdle);
      } else {
        result->success = false;
        result->message = "MoveL cancelled";
        goal_handle->canceled(result);
        state_machine_.transition_to(RobotState::kIdle);
      }

    } catch (const std::exception& e) {
      result->success = false;
      result->message = std::string("MoveL error: ") + e.what();
      goal_handle->abort(result);
      state_machine_.transition_to(RobotState::kFault);
      state_machine_.set_error(3, result->message);
    }
  }).detach();
}
```

- [ ] **Step 6: Implement pendant service callbacks**

```cpp
// ===== Pendant Service Callbacks =====

void RobotControllerNode::handle_pendant_set_tcp(
    const std::shared_ptr<arm_control_interfaces::srv::SetTCP::Request> req,
    std::shared_ptr<arm_control_interfaces::srv::SetTCP::Response> res) {
  try {
    controller_->set_tcp(req->name);
    res->success = true;
    res->message = "TCP set to " + req->name;
  } catch (const std::exception& e) {
    res->success = false;
    res->message = std::string("SetTCP failed: ") + e.what();
  }
}

void RobotControllerNode::handle_set_speed_ratio(
    const std::shared_ptr<arm_control_interfaces::srv::SetSpeedRatio::Request> req,
    std::shared_ptr<arm_control_interfaces::srv::SetSpeedRatio::Response> res) {
  if (req->ratio < 0.0 || req->ratio > 1.0) {
    res->success = false;
    res->message = "ratio must be in [0.0, 1.0]";
    return;
  }
  global_speed_ratio_ = req->ratio;
  res->success = true;
  res->message = "global speed ratio set to " + std::to_string(req->ratio);
}

void RobotControllerNode::handle_robot_cmd(
    const std::shared_ptr<arm_control_interfaces::srv::RobotCmd::Request> req,
    std::shared_ptr<arm_control_interfaces::srv::RobotCmd::Response> res) {
  switch (req->command) {
    case arm_control_interfaces::srv::RobotCmd::Request::STOP:
      if (state_machine_.state() == RobotState::kMoving ||
          state_machine_.state() == RobotState::kTeaching) {
        trajectory_executor_->cancel();
        // Hold current position
        bridge_->publish_command(bridge_->get_current_arm(),
                                bridge_->get_current_finger());
        // Transition: kMoving/kTeaching → kStopping → kIdle
        state_machine_.transition_to(RobotState::kStopping);
        state_machine_.transition_to(RobotState::kIdle);
        res->success = true;
        res->message = "STOP executed";
      } else if (state_machine_.state() == RobotState::kStopping) {
        // Already stopping
        bridge_->publish_command(bridge_->get_current_arm(),
                                bridge_->get_current_finger());
        res->success = true;
        res->message = "STOP: already stopping";
      } else {
        res->success = true;
        res->message = "STOP: no motion to stop";
      }
      break;

    case arm_control_interfaces::srv::RobotCmd::Request::EMERGENCY_STOP:
      emergency_stop();
      res->success = true;
      res->message = "EMERGENCY_STOP executed";
      break;

    case arm_control_interfaces::srv::RobotCmd::Request::CLEAR_FAULT:
      if (state_machine_.state() == RobotState::kFault) {
        state_machine_.clear_error();
        state_machine_.transition_to(RobotState::kIdle);
        res->success = true;
        res->message = "FAULT cleared";
      } else {
        res->success = false;
        res->message = "CLEAR_FAULT: robot not in FAULT state";
      }
      break;

    default:
      res->success = false;
      res->message = "Unknown command: " + std::to_string(req->command);
      break;
  }
}
```

- [ ] **Step 7: Implement jog, watchdog, emergency_stop, and status publisher**

```cpp
// ===== Jog + Watchdog =====

void RobotControllerNode::handle_jog_command(
    const arm_control_interfaces::msg::JogCommand::SharedPtr msg) {
  auto state = state_machine_.state();
  if (state != RobotState::kIdle && state != RobotState::kTeaching) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Jog ignored: robot state=%s",
                         RobotStateMachine::state_name(state));
    return;
  }

  if (state == RobotState::kIdle) {
    state_machine_.transition_to(RobotState::kTeaching);
  }

  last_jog_time_ = this->now();

  // velocity: [vx, vy, vz, vroll, vpitch, vyaw] in m/s and rad/s
  // Convert to incremental joint motion via differential IK
  double dt = 0.02;  // 50Hz jog cycle
  auto current = bridge_->get_current_arm();
  auto pose = controller_->get_end_effector_pose();

  // Apply velocity as Cartesian delta
  std::array<double, 3> delta = {
      msg->velocity[0] * dt,
      msg->velocity[1] * dt,
      msg->velocity[2] * dt
  };
  std::array<double, 3> delta_rpy = {
      msg->velocity[3] * dt,
      msg->velocity[4] * dt,
      msg->velocity[5] * dt
  };

  // Compute new target pose
  Eigen::Vector3d pos(pose[0], pose[1], pose[2]);
  Eigen::Vector3d rpy(pose[3], pose[4], pose[5]);
  Eigen::Matrix3d rot = (Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX()) *
                         Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()))
                            .toRotationMatrix();
  Eigen::Vector3d new_pos = pos + rot * Eigen::Vector3d(delta[0], delta[1], delta[2]);

  Eigen::AngleAxisd d_roll(delta_rpy[0], Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd d_pitch(delta_rpy[1], Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd d_yaw(delta_rpy[2], Eigen::Vector3d::UnitZ());
  Eigen::Matrix3d d_rot = (d_yaw * d_pitch * d_roll).toRotationMatrix();
  Eigen::Matrix3d new_rot = d_rot * rot;
  Eigen::Vector3d new_rpy = new_rot.eulerAngles(0, 1, 2);

  std::array<double, 3> target_xyz = {new_pos.x(), new_pos.y(), new_pos.z()};
  std::array<double, 3> target_rpy = {new_rpy.x(), new_rpy.y(), new_rpy.z()};

  // Solve IK for new pose
  auto tcp_cfg = profile_.tcp_frames.at(controller_->get_current_tcp());
  Eigen::Matrix4d T_tcp = Eigen::Matrix4d::Identity();
  Eigen::AngleAxisd e_roll(tcp_cfg.offset_rpy[0], Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd e_pitch(tcp_cfg.offset_rpy[1], Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd e_yaw(tcp_cfg.offset_rpy[2], Eigen::Vector3d::UnitZ());
  T_tcp.block<3,3>(0,0) = (e_yaw * e_pitch * e_roll).toRotationMatrix();
  T_tcp(0,3) = tcp_cfg.offset_xyz[0]; T_tcp(1,3) = tcp_cfg.offset_xyz[1]; T_tcp(2,3) = tcp_cfg.offset_xyz[2];

  Eigen::Matrix4d T_target = Eigen::Matrix4d::Identity();
  T_target.block<3,3>(0,0) = new_rot;
  T_target(0,3) = new_pos.x(); T_target(1,3) = new_pos.y(); T_target(2,3) = new_pos.z();

  Eigen::Matrix4d hand_target = T_target * T_tcp.inverse();
  Eigen::Vector3d h_xyz = hand_target.block<3,1>(0,3);
  Eigen::Vector3d h_rpy = hand_target.block<3,3>(0,0).eulerAngles(0,1,2);

  auto ik_result = ik_->solve(
      std::array<double,3>{h_xyz.x(), h_xyz.y(), h_xyz.z()},
      std::array<double,3>{h_rpy.x(), h_rpy.y(), h_rpy.z()});

  if (ik_result) {
    bridge_->publish_command(*ik_result, bridge_->get_current_finger());
  } else {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Jog: IK failed, skipping step");
  }
}

void RobotControllerNode::jog_watchdog_callback() {
  if (state_machine_.state() != RobotState::kTeaching) return;

  auto elapsed = (this->now() - last_jog_time_).seconds();
  if (elapsed > 0.2) {
    RCLCPP_WARN(this->get_logger(), "Jog watchdog: no command for %.2fs, stopping", elapsed);
    bridge_->publish_command(bridge_->get_current_arm(), bridge_->get_current_finger());
    state_machine_.transition_to(RobotState::kIdle);
  }
}

// ===== Emergency Stop =====

void RobotControllerNode::emergency_stop() {
  trajectory_executor_->cancel();
  bridge_->publish_command(bridge_->get_current_arm(), bridge_->get_current_finger());
  state_machine_.force_state(RobotState::kFault);
  state_machine_.set_error(100, "EMERGENCY_STOP activated");
  RCLCPP_ERROR(this->get_logger(), "EMERGENCY_STOP activated");
}

// ===== Status Publisher =====

void RobotControllerNode::publish_status() {
  auto msg = std::make_unique<arm_control_interfaces::msg::RobotStatus>();
  msg->state = static_cast<uint8_t>(state_machine_.state());
  msg->speed_ratio = global_speed_ratio_;
  msg->error_code = state_machine_.error_code();
  msg->error_message = state_machine_.error_message();

  auto angles = controller_->get_joint_angles();
  msg->joint_angles = angles;

  auto pose = controller_->get_end_effector_pose();
  msg->tcp_pose = {pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]};

  msg->finger_width = controller_->get_finger_width();
  msg->tcp_name = controller_->get_current_tcp();
  msg->is_connected = ready_;

  status_pub_->publish(std::move(msg));
}
```

- [ ] **Step 8: Build to verify**

```bash
source install/setup.zsh
colcon build --base-paths src --packages-select arm_control_interfaces robot_control_cpp
```

Expected: Build succeeds with no errors.

- [ ] **Step 9: Commit**

```bash
git add src/robot_control_cpp/src/nodes/robot_controller_node.cpp
git commit -m "feat: implement MoveJ/MoveL actions, jog watchdog, status publisher, RobotCmd"
```

---

### Task 7: Full integration build

**Files:** None (build verification only)

- [ ] **Step 1: Clean build all packages**

```bash
rm -rf build/ install/ log/
colcon build --base-paths src --packages-up-to robot_control_cpp
```

Expected: All packages build successfully.

- [ ] **Step 2: Verify downstream packages still build**

```bash
source install/setup.zsh
colcon build --base-paths src --packages-up-to teaching_pendant
colcon build --base-paths src --packages-up-to robot_control_cpp_py
```

Expected: All build successfully.

- [ ] **Step 3: Verify interface introspection**

```bash
source install/setup.zsh
ros2 interface show arm_control_interfaces/action/MoveJ
ros2 interface show arm_control_interfaces/action/MoveL
ros2 interface show arm_control_interfaces/srv/SetTCP
ros2 interface show arm_control_interfaces/srv/SetSpeedRatio
ros2 interface show arm_control_interfaces/srv/RobotCmd
ros2 interface show arm_control_interfaces/msg/RobotStatus
ros2 interface show arm_control_interfaces/msg/JogCommand
```

Expected: All interfaces print correctly.

- [ ] **Step 4: Final commit (if any fixes needed)**

```bash
git add -A
git commit -m "chore: integration build verification"
```
