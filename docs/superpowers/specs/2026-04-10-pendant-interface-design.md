# Pendant Interface Layer Design

**Date:** 2026-04-10
**Status:** Approved
**Scope:** Standardized ROS2 interface layer for teaching pendant integration

## Context

The project has an existing robot control system with:
- 8 ROS2 Services in `robot_control_msgs` (SolveIK, MoveJoint, MovePose, MoveLinear, ControlGripper, GoHome, SetSpeed, GetRobotState)
- A Qt5 teaching pendant that calls Services directly
- Clean Layer 1/2 separation: `IRobotController` + `MotionIOBridge` abstract the pure C++ core

The pendant needs richer interaction: progress feedback during long motions, jog streaming with safety watchdog, centralized state machine, and emergency stop.

## Decisions

- **Approach A: Action-First** — All pendant-facing motion goes through Actions. Services handle configuration. Central state machine guards every command.
- **New package** `arm_control_interfaces` created separately from `robot_control_msgs` (backward compatible).
- **Jog mode:** Cartesian only (6-axis: vx,vy,vz,vroll,vpitch,vyaw).
- **MoveJ:** Supports both joint-space (angle array) and Cartesian-space (xyz+rpy via IK) targets.

## Interface Definitions

### Package: `arm_control_interfaces`

#### Actions

**MoveJ.action** — Joint-space motion with Cartesian fallback

```
# Goal
uint8 JOINT_SPACE = 0
uint8 CARTESIAN = 1
uint8 mode                      # 0=joint_angles, 1=position+orientation
float64[7] joint_angles         # used when mode=JOINT_SPACE
geometry_msgs/Point position    # used when mode=CARTESIAN
geometry_msgs/Vector3 orientation # RPY (rad), used when mode=CARTESIAN
float64 speed_ratio             # 0.0-1.0
float64 finger_width            # gripper target width (m)
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

**MoveL.action** — Linear interpolation

```
# Goal
geometry_msgs/Point position
geometry_msgs/Vector3 orientation
string frame                   # reference frame (default: "base")
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

#### Services

**SetTCP.srv**

```
string name
---
bool success
string message
```

**SetSpeedRatio.srv**

```
# ratio: 0.0-1.0 global speed limit
# Acts as an upper bound multiplier on all motion.
# Composition rule: effective_speed = per_mode_speed * ratio
# Example: per_mode=50% (from SetSpeed), ratio=0.8 -> effective=40%
float64 ratio                   # 0.0-1.0
---
bool success
string message
```

**RobotCmd.srv**

```
uint8 STOP = 0
uint8 EMERGENCY_STOP = 1
uint8 CLEAR_FAULT = 2
uint8 command
---
bool success
string message
```

#### Messages

**RobotStatus.msg**

```
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

**JogCommand.msg**

```
uint8 CARTESIAN_JOG = 0
uint8 mode
float64[6] velocity
builtin_interfaces/Time stamp
```

## State Machine

### RobotState Enum

```cpp
enum class RobotState : uint8_t {
  kIdle = 0,
  kMoving = 1,
  kTeaching = 2,
  kStopping = 3,
  kFault = 4,
};
```

### Valid Transitions

| From | To | Trigger |
|------|----|---------|
| kIdle | kMoving | MoveJ/MoveL goal accepted |
| kIdle | kTeaching | First JogCommand received |
| kMoving | kIdle | Motion completed successfully |
| kMoving | kStopping | STOP or Action cancel received |
| kMoving | kFault | Execution error or EMERGENCY_STOP |
| kTeaching | kIdle | Zero-velocity jog or explicit stop |
| kTeaching | kStopping | Watchdog timeout (200ms) |
| kTeaching | kFault | EMERGENCY_STOP |
| kStopping | kIdle | Stopped successfully |
| kStopping | kFault | Deceleration failure or EMERGENCY_STOP |
| kFault | kIdle | CLEAR_FAULT command |
| Any | kFault | EMERGENCY_STOP (highest priority, bypasses all) |

### Transition Diagram

```
       CLEAR_FAULT              goal accepted
  +--------------+        +------------------+
  |              |        |                  |
  v              |        v                  |
+------+  goal +-------+  error  +-------+ |
| IDLE |------->|MOVING |------->| FAULT | |
+------+        +-------+        +-------+ |
  ^  ^              |                  ^    |
  |  |  STOP/Cancel  |                  |    |
  |  |               v                  |    |
  |  |          +---------+             |    |
  |  +--------->|STOPPING |-------------+    |
  |             +---------+                  |
  |    first JogCommand                      |
  |         |                                |
  |         v                                |
  |     +---------+  watchdog timeout        |
  |     |TEACHING |--------------------------+
  |     +---------+
  +-----------------------------------------------+
```

## Layer 1 Refactoring: Non-Blocking Trajectory Execution

The existing `RobotMotionController::moveJ/moveL` are synchronous blocking calls. To support Action feedback (progress, estimated_time_remaining), we need a non-blocking trajectory execution interface in Layer 1.

### Approach: Step-by-Step Executor

Add a non-blocking trajectory execution mode to `IRobotController`:

```cpp
// New methods on IRobotController
struct TrajectoryStep {
  std::vector<double> joint_positions;
  double time_from_start;  // seconds
};

// Start a pre-planned trajectory without blocking
// Returns a handle that can be polled for progress
void start_trajectory(const std::vector<TrajectoryStep>& trajectory,
                      double finger_width);

// Get current execution progress. Returns false if no trajectory active.
bool get_trajectory_progress(double& progress,
                             std::vector<double>& current_angles,
                             double& time_remaining) const;

// Cancel active trajectory (controlled deceleration)
void cancel_trajectory();

// Check if trajectory is still executing
bool is_trajectory_active() const;
```

### Implementation

The existing `moveJ` internally calls `TrajectoryPlanner::plan()` then iterates through points with `sleep_until`. The refactored version extracts the trajectory into a `vector<TrajectoryStep>`, then the Action callback runs the loop in a separate thread, publishing feedback at each step.

The `TrajectoryPlanner` already has per-point timing information internally. We expose it by adding timing to the planner output:

```cpp
// In trajectory_planner.hpp, extend the output
struct TrajectoryPoint {
  std::vector<double> positions;
  double time_from_start;
};
// plan() returns std::vector<TrajectoryPoint> instead of std::vector<std::vector<double>>
```

This is a Layer 1 change (zero ROS dependency). The existing blocking `moveJ/moveL` methods can be preserved as wrappers that call `start_trajectory` + poll until completion.

## Controller Skeleton

Added to existing `RobotControllerNode`:

### New Members

```cpp
// State machine
RobotState state_{RobotState::kIdle};
std::mutex state_mutex_;
rclcpp::Time last_jog_time_;
double global_speed_ratio_{1.0};
int32_t error_code_{0};
std::string error_message_;

// Action servers
rclcpp_action::Server<MoveJ>::SharedPtr movej_action_;
rclcpp_action::Server<MoveL>::SharedPtr movel_action_;

// Service servers
rclcpp::Service<SetTCP>::SharedPtr set_tcp_srv_;
rclcpp::Service<SetSpeedRatio>::SharedPtr set_speed_srv_;
rclcpp::Service<RobotCmd>::SharedPtr robot_cmd_srv_;

// Subscriber + timers
rclcpp::Subscription<JogCommand>::SharedPtr jog_sub_;
rclcpp::TimerBase::SharedPtr jog_watchdog_timer_;
rclcpp::TimerBase::SharedPtr status_timer_;

// Publisher
rclcpp::Publisher<RobotStatus>::SharedPtr status_pub_;
```

### Key Methods

```cpp
// State machine
bool transition_to(RobotState target);

// Action callbacks (MoveJ / MoveL)
rclcpp_action::GoalResponse handle_movej_goal(...);
rclcpp_action::CancelResponse handle_movej_cancel(...);
void handle_movej_accepted(...);

// Service callbacks
void handle_set_tcp(...);
void handle_set_speed_ratio(...);
void handle_robot_cmd(...);

// Jog + watchdog
void handle_jog_command(const JogCommand::SharedPtr msg);
void jog_watchdog_callback();  // 200ms timer

// Status publisher
void publish_status();  // 10Hz timer
```

### Callback Behavior

**MoveJ/MoveL Goal:** Check `state_ == kIdle`, transition to `kMoving`, execute via existing `IRobotController::moveJ/moveL`, publish progress feedback at trajectory rate, transition to `kIdle` on completion or `kFault` on error.

**STOP:** Transition current state to `kStopping`, send zero-velocity command, transition to `kIdle` when stopped.

**EMERGENCY_STOP:** Publishes current joint positions (hold position, not literal zero angles) to halt motion immediately. Forces `kFault` state from any other state. Requires `CLEAR_FAULT` to resume.

**CLEAR_FAULT:** Only valid from `kFault`, resets error codes, transitions to `kIdle`.

**Jog:** Check `state_ == kIdle || state_ == kTeaching`, transition to `kTeaching`, compute incremental motion from velocity * dt, call existing motion interface, reset watchdog timer.

**Watchdog (200ms):** If no JogCommand received within 200ms during `kTeaching`, send zero-velocity command, transition to `kIdle`.

## File Layout

### New files

```
src/arm_control_interfaces/
  action/MoveJ.action
  action/MoveL.action
  srv/SetTCP.srv
  srv/SetSpeedRatio.srv
  srv/RobotCmd.srv
  msg/RobotStatus.msg
  msg/JogCommand.msg
  CMakeLists.txt
  package.xml
```

### Modified files

- `src/robot_control_cpp/include/robot_control_cpp/nodes/robot_controller_node.hpp`
- `src/robot_control_cpp/src/nodes/robot_controller_node.cpp`
- `src/robot_control_cpp/CMakeLists.txt`
- `src/robot_control_cpp/package.xml`
- `src/teaching_pendant/` (migrate to new Actions/Messages)

### Build order

```
1. robot_control_msgs        (existing, unchanged)
2. arm_control_interfaces    (NEW)
3. robot_control_cpp         (modified, depends on both interface packages)
4. teaching_pendant          (migrated to new interfaces)
5. robot_control_cpp_py      (unchanged)
6. robot_control_test        (unchanged)
```

### CMakeLists.txt (arm_control_interfaces)

```cmake
cmake_minimum_required(VERSION 3.8)
project(arm_control_interfaces)

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
```

### package.xml (arm_control_interfaces)

```xml
<?xml version="1.0"?>
<package format="3">
  <name>arm_control_interfaces</name>
  <version>0.1.0</version>
  <description>Standardized ROS2 interfaces for robot arm teaching pendant</description>
  <maintainer email="todo@todo.com">maintainer</maintainer>
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

### CMake linking (robot_control_cpp)

```cmake
find_package(arm_control_interfaces REQUIRED)
ament_target_dependencies(robot_nodes
  rclcpp rclcpp_action
  robot_control_msgs arm_control_interfaces
  ...
)
```

## Safety Considerations

1. **Watchdog timer** — 200ms timeout on jog commands prevents runaway motion if pendant disconnects.
2. **State machine guards** — Every command callback checks current state; invalid transitions are rejected.
3. **Emergency stop** — Immediate zero-velocity command bypasses state machine, forces `kFault` state.
4. **Action cancellation** — MoveJ/MoveL support cancel via ROS2 Action protocol, triggering controlled deceleration.
5. **Global speed ratio** — `SetSpeedRatio` applies an upper bound on all motion, enforced at trajectory planning level.

## Out of Scope

- Existing `robot_control_msgs` Services remain for Python scripts and backward compatibility.
- `robot_control_cpp_py` bindings unchanged — Python scripts continue using existing Services.
- Vision-guided grasping pipeline unchanged.
- No real-time guarantees beyond ROS2 default scheduling.
