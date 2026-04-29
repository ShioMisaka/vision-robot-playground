# Motion Arbitration Design: Explicit Ownership Model

**Date:** 2026-04-29
**Status:** Approved (revised after review)
**Scope:** robot_msgs, robot_controller, robot_vision, robot_hmi, robot_api_python

## Problem

Multiple frontends (HMI pendant, vision grasp scripts, Python API) can command the robot simultaneously with no mutual exclusion. The `kIdle` state in the 100Hz control loop blindly accepts `external_joint_target_` from the pendant's 50Hz joint stream, causing:

1. **With pendant + script**: Oscillation/twitching between trajectory targets and pendant stale targets during trajectory transitions through `kIdle`.
2. **Without pendant**: Potential race conditions between trajectory submission and the control loop's `kIdle` hold-position behavior.

## Solution: MotionOwner Model

### Core Types

```cpp
// robot_controller/nodes/motion_owner.hpp (NEW FILE)
enum class MotionOwner : uint8_t {
  kNone = 0,      // No active owner - kIdle holds position
  kPendant = 1,   // HMI pendant controls via joint stream / jog / services
  kScript = 2,    // External script (C++ API / Python) controls
};
```

```cpp
// robot_controller/nodes/motion_owner.hpp (same file, nodes layer only)
enum class MotionSource : uint8_t {
  kService = 0,   // Trajectory submitted via ROS2 service handler
  kApi = 1,       // Trajectory submitted via C++ API (moveJ/moveL)
};
```

Both enums live in the **nodes layer** (Layer 2) since ownership is a ROS2 communication concern, not a pure-motion concern.

### Thread Safety

`motion_owner_` is `std::atomic<MotionOwner>` since it is read/written from multiple threads:
- 100Hz control loop (read in kIdle)
- `external_joint_sub_` callback (conditional write)
- `on_trajectory_started_` callback (write)
- `emergency_stop()` (write)
- `claim_ownership()` / `release_ownership()` (write)

### Ownership Transitions

| Event | From | To | Notes |
|-------|------|----|-------|
| `submit_trajectory` with `kApi` source | any | `kScript` | Via `on_trajectory_started_` callback |
| `submit_trajectory` with `kService` source | any | `kPendant` | Via `on_trajectory_started_` callback |
| Pendant `joint_target` received | `kNone` only | `kPendant` | Rejected if owner != kNone (server-side guard) |
| Pendant stream timeout (200ms) | `kPendant` | `kNone` | Checked in kIdle |
| Script calls `release_ownership()` | `kScript` | `kNone` | |
| Emergency stop | any | `kNone` | |
| STOP command (`~/robot_cmd`) | any | `kNone` | User-initiated stop |
| CLEAR_FAULT command | any | `kNone` | Reset after fault |

### Jog Ownership Rules

- Jog is always owned by the pendant (only pendant sends `JogCommand`)
- If `motion_owner_ == kScript`, jog commands are **rejected** (no state transition to kTeaching)
- If jog is active and script claims ownership, jog is stopped first (emergency_stop on jog controller)

### kIdle Behavior Change

Before (current, broken):
```cpp
case RobotState::kIdle:
  if (!external_joint_target_.empty() && elapsed < 0.2) {
    target = external_joint_target_;  // Always accepts pendant stream
  } else {
    target = actual;
  }
```

After (with ownership):
```cpp
case RobotState::kIdle:
case RobotState::kFault:
  {
    std::lock_guard<std::mutex> lock(external_target_mutex_);
    auto elapsed = (this->now() - external_target_time_).seconds();
    if (motion_owner_.load() == MotionOwner::kPendant &&
        !external_joint_target_.empty() && elapsed < 0.2) {
      target = external_joint_target_;  // Only when pendant owns control
    } else {
      target = actual;  // Hold position otherwise
    }
    // Auto-release pendant on stream timeout
    if (motion_owner_.load() == MotionOwner::kPendant && elapsed >= 0.2) {
      motion_owner_.store(MotionOwner::kNone);
    }
  }
```

### Server-Side Guard: Reject Pendant Messages When Not Owner

In `external_joint_sub_` callback, reject pendant messages when another owner holds control:

```cpp
external_joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "~/joint_target", 10,
    [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
      // Reject pendant stream if another owner holds control
      auto owner = motion_owner_.load();
      if (owner != MotionOwner::kNone && owner != MotionOwner::kPendant) {
        return;  // Drop message - script has control
      }
      std::lock_guard<std::mutex> lock(external_target_mutex_);
      external_joint_target_.assign(msg->position.begin(), msg->position.end());
      external_target_time_ = this->now();
      // Claim ownership on first message (only from kNone)
      if (owner == MotionOwner::kNone) {
        motion_owner_.store(MotionOwner::kPendant);
      }
    }, ext_sub_opts);
```

## Interface Changes

### 1. RobotControllerNode (Layer 2)

Add ownership methods to the **node** (not IRobotController):

```cpp
// robot_controller_node.hpp
std::atomic<MotionOwner> motion_owner_{MotionOwner::kNone};

void claim_ownership(MotionOwner owner);
void release_ownership();
```

Implementation:
```cpp
void RobotControllerNode::claim_ownership(MotionOwner owner) {
  motion_owner_.store(owner);
}

void RobotControllerNode::release_ownership() {
  motion_owner_.store(MotionOwner::kNone);
}
```

### 2. MotionIOBridge (Layer 1 → Layer 2 Adapter)

Add `MotionSource` parameter to `submit_trajectory`:

```cpp
// motion_io_bridge.hpp
void submit_trajectory(const std::vector<TrajectoryStep>& steps,
                       double finger,
                       MotionSource source = MotionSource::kApi);
```

Change `on_trajectory_started_` callback signature to include source:

```cpp
std::function<void(MotionSource)> on_trajectory_started_;
```

Store the source so it's available when the callback fires:
```cpp
void RosMotionBridge::submit_trajectory(
    const std::vector<TrajectoryStep>& steps, double finger,
    MotionSource source) {
  setpoint_gen_.start(steps, finger);
  if (on_trajectory_started_) {
    on_trajectory_started_(source);
  }
}
```

### 3. RobotMotionController (Layer 1)

Pass `MotionSource` through to bridge. No ownership methods on this class.

```cpp
void moveJ_internal(const std::vector<double>& target_angles,
                    double finger, bool block,
                    MotionSource source = MotionSource::kApi);
```

The `moveJ()`/`moveL()` public methods default to `MotionSource::kApi`.

### 4. Service Handlers (Layer 2)

Service handlers in `robot_controller_node_services.cpp` call controller methods with `MotionSource::kService`:

```cpp
void RobotControllerNode::handle_move_joint(...) {
  // ...
  controller_->moveJ(req->joint_angles, req->block, MotionSource::kService);
  // ... (requires adding MotionSource parameter to moveJ)
}
```

This requires adding an optional `MotionSource` parameter to the public `moveJ`/`moveL` signatures on `IRobotController` and `RobotMotionController`.

### 5. RobotStatus Message

Add field to `robot_msgs/msg/RobotStatus.msg` (appended at end for ABI compatibility):

```
# ... existing fields ...
uint8 OWNER_NONE=0
uint8 OWNER_PENDANT=1
uint8 OWNER_SCRIPT=2
uint8 motion_owner  # Current motion owner (OWNER_NONE/PENDANT/SCRIPT)
```

Named constants avoid magic numbers in PendantNode.

### 6. Publish Status

```cpp
void RobotControllerNode::publish_status() {
  // ... existing fields ...
  msg->motion_owner = static_cast<uint8_t>(motion_owner_.load());
}
```

## Frontend Changes

### GraspTaskManager

Proper RAII ownership guard using destructor-based cleanup:

```cpp
// In grasp_task_manager.hpp or a utility header
struct OwnershipGuard {
  std::shared_ptr<IRobotController> robot;
  explicit OwnershipGuard(std::shared_ptr<IRobotController> r) : robot(std::move(r)) {
    robot->claim_ownership();
  }
  ~OwnershipGuard() {
    if (robot) robot->release_ownership();
  }
  OwnershipGuard(const OwnershipGuard&) = delete;
  OwnershipGuard& operator=(const OwnershipGuard&) = delete;
};
```

Usage in `GraspTaskManager::run()`:

```cpp
bool GraspTaskManager::run(double timeout) {
  OwnershipGuard owner_guard(robot_);  // Claims ownership, releases on any exit

  // ... existing grasp logic (success/error/exception/abort) ...
  // release_ownership() called automatically by destructor
}
```

### PendantNode

Status-driven stream control in `on_robot_status`:

```cpp
void PendantNode::on_robot_status(const RobotStatus::SharedPtr msg) {
  // ... existing fault tracking ...

  if (msg->motion_owner == robot_msgs::msg::RobotStatus::OWNER_SCRIPT) {
    if (!script_active_) {
      pause_joint_stream();
      script_active_ = true;
    }
  } else if (script_active_ &&
             msg->motion_owner == robot_msgs::msg::RobotStatus::OWNER_NONE) {
    script_active_ = false;
    // Sync stream target to current actual position (existing pattern)
    {
      std::lock_guard<std::mutex> jlock(latest_joints_mutex_);
      std::lock_guard<std::mutex> slock(joint_stream_mutex_);
      joint_stream_target_ = latest_joints_;
      joint_stream_dirty_ = true;
    }
    resume_joint_stream();
  }

  // ... existing jog/fault handling ...
}
```

### Jog Command Guard

In `handle_jog_command`:

```cpp
void RobotControllerNode::handle_jog_command(...) {
  auto owner = motion_owner_.load();
  if (owner == MotionOwner::kScript) {
    RCLCPP_WARN(this->get_logger(),
                "Jog rejected: script owns motion control");
    return;
  }
  // ... existing jog handling ...
}
```

### Emergency Stop / STOP / CLEAR_FAULT

```cpp
void RobotControllerNode::emergency_stop() {
  // ... existing code ...
  motion_owner_.store(MotionOwner::kNone);  // Reset ownership
}

// In handle_robot_cmd service handler:
void RobotControllerNode::handle_robot_cmd(...) {
  if (req->command == RobotCmd::Request::STOP) {
    motion_owner_.store(MotionOwner::kNone);
    // ... existing stop logic ...
  } else if (req->command == RobotCmd::Request::EMERGENCY_STOP) {
    emergency_stop();  // Already sets owner to kNone
  } else if (req->command == RobotCmd::Request::CLEAR_FAULT) {
    motion_owner_.store(MotionOwner::kNone);
    // ... existing clear fault logic ...
  }
}
```

## Packages Changed

| Package | Changes |
|---------|---------|
| robot_msgs | Add `motion_owner` field + named constants to RobotStatus.msg |
| robot_controller | Add `motion_owner.hpp` (MotionOwner + MotionSource enums), modify kIdle behavior, add ownership methods to node, guard external_joint_sub_, guard jog, add MotionSource to submit_trajectory and moveJ_internal, reset owner on STOP/ESTOP/CLEAR_FAULT |
| robot_vision | GraspTaskManager uses OwnershipGuard RAII to claim/release ownership |
| robot_hmi | PendantNode pauses/resumes stream based on owner status in RobotStatus |
| robot_api_python | Expose `claim_ownership()` / `release_ownership()` in Python bindings |

**Build note:** RobotStatus.msg change requires full rebuild of all dependent packages.

## Testing

- **Unit test**: Verify kIdle behavior with each MotionOwner state (kNone holds, kPendant follows stream, kScript holds)
- **Unit test**: Verify pendant message rejection when owner is kScript
- **Unit test**: Verify ownership transitions on STOP/ESTOP/CLEAR_FAULT
- **Integration test**: Run demo_vision_grasp alone (no pendant) - should hold position between trajectories
- **Integration test**: Run demo_vision_grasp with HMI open - pendant pauses stream during grasp
- **Manual test**: Pendant joint control still works normally when no script is running
- **Manual test**: Pendant jog rejected during script ownership
