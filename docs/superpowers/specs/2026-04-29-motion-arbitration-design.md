# Motion Arbitration Design: Explicit Ownership Model

**Date:** 2026-04-29
**Status:** Approved
**Scope:** robot_msgs, robot_controller, robot_vision, robot_hmi

## Problem

Multiple frontends (HMI pendant, vision grasp scripts, Python API) can command the robot simultaneously with no mutual exclusion. The `kIdle` state in the 100Hz control loop blindly accepts `external_joint_target_` from the pendant's 50Hz joint stream, causing:

1. **With pendant + script**: Oscillation/twitching between trajectory targets and pendant stale targets during trajectory transitions through `kIdle`.
2. **Without pendant**: Potential race conditions between trajectory submission and the control loop's `kIdle` hold-position behavior.

## Solution: MotionOwner Model

### Core Type

```cpp
// robot_controller/nodes/motion_owner.hpp
enum class MotionOwner : uint8_t {
  kNone = 0,      // No active owner - kIdle holds position
  kPendant = 1,   // HMI pendant controls via joint stream / jog / services
  kScript = 2,    // External script (C++ API / Python) controls
};
```

### Trajectory Source

```cpp
// robot_controller/motion/motion_io_bridge.hpp
enum class MotionSource : uint8_t {
  kService = 0,   // Trajectory submitted via ROS2 service handler
  kApi = 1,       // Trajectory submitted via C++ API (moveJ/moveL)
};
```

### Ownership Transitions

| Event | From | To |
|-------|------|----|
| `submit_trajectory` with `kApi` source | any | `kScript` |
| Service handler calls moveJ/moveL | any | `kPendant` |
| Pendant sends `joint_target` message | `kNone` | `kPendant` |
| Pendant stream timeout (200ms no msg) | `kPendant` | `kNone` |
| Script calls `release_ownership()` | `kScript` | `kNone` |
| Emergency stop | any | `kNone` |

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
  if (motion_owner_ == MotionOwner::kPendant &&
      !external_joint_target_.empty() && elapsed < 0.2) {
    target = external_joint_target_;  // Only when pendant owns control
  } else {
    target = actual;  // Hold position otherwise
  }
  // Auto-release pendant on stream timeout
  if (motion_owner_ == MotionOwner::kPendant && elapsed >= 0.2) {
    motion_owner_ = MotionOwner::kNone;
  }
```

## Interface Changes

### 1. MotionIOBridge

Add `MotionSource` parameter to `submit_trajectory`:

```cpp
void submit_trajectory(const std::vector<TrajectoryStep>& steps,
                       double finger,
                       MotionSource source = MotionSource::kApi);
```

The `on_trajectory_started_` callback reads the source and sets `motion_owner_` accordingly.

### 2. IRobotController

Add ownership methods:

```cpp
virtual void claim_ownership() = 0;
virtual void release_ownership() = 0;
```

### 3. RobotMotionController

Pass `MotionSource` through to bridge:

- `moveJ_internal(..., MotionSource source)` forwards source to `submit_trajectory`
- `claim_ownership()` sets owner to `kScript` via bridge callback
- `release_ownership()` resets owner to `kNone` via bridge callback

### 4. RobotStatus Message

Add field to `robot_msgs/msg/RobotStatus.msg`:

```
uint8 motion_owner  # 0=NONE, 1=PENDANT, 2=SCRIPT
```

### 5. Service Handlers

Service handlers in `robot_controller_node_services.cpp` pass `MotionSource::kService` when calling controller methods, so the controller knows the source is the pendant.

## Frontend Changes

### GraspTaskManager

RAII ownership guard:

```cpp
bool GraspTaskManager::run(double timeout) {
  robot_->claim_ownership();  // Claim at start
  auto owner_guard = [](){ robot_->release_ownership(); };  // RAII on exit

  // ... existing grasp logic ...

  // release_ownership called on any exit (success/error/abort)
}
```

### PendantNode

Status-driven stream control in `on_robot_status`:

```cpp
void PendantNode::on_robot_status(const RobotStatus::SharedPtr msg) {
  if (msg->motion_owner == 2) {  // kScript
    if (!script_active_) {
      pause_joint_stream();
      script_active_ = true;
    }
  } else if (script_active_ && msg->motion_owner == 0) {  // kNone
    script_active_ = false;
    sync_stream_to_actual();  // Update stream target to current position
    resume_joint_stream();
  }
  // ... existing jog/fault handling ...
}
```

### Emergency Stop

Add `motion_owner_ = MotionOwner::kNone` to `emergency_stop()` method.

## Packages Changed

| Package | Changes |
|---------|---------|
| robot_msgs | Add `motion_owner` field to RobotStatus.msg |
| robot_controller | Add MotionOwner enum, MotionSource enum, modify kIdle behavior, modify submit_trajectory, add ownership methods |
| robot_vision | GraspTaskManager claims/releases ownership |
| robot_hmi | PendantNode pauses/resumes stream based on owner status |

## Testing

- **Unit test**: Verify kIdle behavior with each MotionOwner state
- **Integration test**: Run demo_vision_grasp alone (no pendant) - should hold position between trajectories
- **Integration test**: Run demo_vision_grasp with HMI open - pendant pauses stream during grasp
- **Manual test**: Pendant joint control still works normally when no script is running
