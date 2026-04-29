# Motion Arbitration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement explicit MotionOwner arbitration to prevent control conflicts between HMI pendant and vision grasp scripts.

**Architecture:** Add MotionOwner/MotionSource enums, thread-safe ownership tracking in the controller node, ownership-aware kIdle behavior, server-side rejection of pendant messages during script ownership, and status-driven stream pause in PendantNode. Ownership lives on `RobotControllerNode` (Layer 2) only -- not on `IRobotController` (Layer 1).

**Tech Stack:** C++17, ROS2 Jazzy, std::atomic, colcon build

**Spec:** `docs/superpowers/specs/2026-04-29-motion-arbitration-design.md`

---

## File Structure

| Action | File | Responsibility |
|--------|------|---------------|
| Create | `src/robot_controller/include/robot_controller/nodes/motion_owner.hpp` | MotionOwner + MotionSource enums |
| Modify | `src/robot_msgs/msg/RobotStatus.msg` | Add `motion_owner` field + constants |
| Modify | `src/robot_controller/include/robot_controller/motion/motion_io_bridge.hpp` | Add MotionSource to submit_trajectory |
| Modify | `src/robot_controller/include/robot_controller/nodes/robot_controller_node.hpp` | Add motion_owner_ atomic, claim/release, change callback signature |
| Modify | `src/robot_controller/src/nodes/ros_motion_bridge.cpp` | Implement MotionSource-aware submit_trajectory |
| Modify | `src/robot_controller/src/nodes/robot_controller_node.cpp` | kIdle behavior, guards, emergency_stop, publish_status |
| Modify | `src/robot_controller/src/nodes/robot_controller_node_services.cpp` | Reset owner on STOP/CLEAR_FAULT |
| Modify | `src/robot_controller/include/robot_controller/motion/robot_motion_controller.hpp` | Add MotionSource to moveJ_internal + public moveJ/moveL |
| Modify | `src/robot_controller/src/motion/robot_motion_controller.cpp` | Pass MotionSource through (moveJ_internal + moveL) |
| Modify | `src/robot_vision/include/robot_vision/nodes/grasp_task_manager.hpp` | Add OwnershipGuard + node parameter |
| Modify | `src/robot_vision/src/nodes/grasp_task_manager.cpp` | Use OwnershipGuard in run() |
| Modify | `src/robot_vision/demo/demo_vision_grasp.cpp` | Pass node to GraspTaskManager |
| Modify | `src/robot_hmi/include/robot_hmi/pendant_node.hpp` | Add script_active_ flag |
| Modify | `src/robot_hmi/src/pendant_node.cpp` | Status-driven stream control |

> **Note:** Line numbers are approximate. Search for the code pattern to find exact locations.

---

### Task 1: Add MotionOwner/MotionSource enums

**Files:**
- Create: `src/robot_controller/include/robot_controller/nodes/motion_owner.hpp`

- [ ] **Step 1: Create the enum header**

```cpp
#pragma once

#include <cstdint>

namespace robot_control {

/// 当前运动控制权的持有者
enum class MotionOwner : uint8_t {
  kNone = 0,      ///< 无活跃持有者 — kIdle 保持当前位置
  kPendant = 1,   ///< 示教器 — 通过 joint_target 流 / jog / service 控制
  kScript = 2,    ///< 外部脚本 — 通过 C++ API / Python 控制
};

/// 轨迹提交来源（决定所有权归属）
enum class MotionSource : uint8_t {
  kService = 0,   ///< 来自 ROS2 Service 回调（示教器发起）
  kApi = 1,       ///< 来自 C++ API 直接调用（脚本发起）
};

}  // namespace robot_control
```

- [ ] **Step 2: Commit**

```bash
git add src/robot_controller/include/robot_controller/nodes/motion_owner.hpp
git commit -m "feat(controller): add MotionOwner and MotionSource enums"
```

---

### Task 2: Update RobotStatus message

**Files:**
- Modify: `src/robot_msgs/msg/RobotStatus.msg`

- [ ] **Step 1: Append motion_owner field with named constants**

After the last line (`bool is_connected`), add:

```
uint8 OWNER_NONE=0
uint8 OWNER_PENDANT=1
uint8 OWNER_SCRIPT=2
uint8 motion_owner
```

- [ ] **Step 2: Build robot_msgs**

Run: `colcon build --base-paths src --packages-select robot_msgs`
Expected: Build succeeds

- [ ] **Step 3: Commit**

```bash
git add src/robot_msgs/msg/RobotStatus.msg
git commit -m "feat(msgs): add motion_owner field to RobotStatus"
```

---

### Task 3: Update MotionIOBridge + RosMotionBridge + RobotControllerNode (atomic commit)

These three files are tightly coupled -- the callback signature change in the bridge header breaks the existing node code. They must be changed together.

**Files:**
- Modify: `src/robot_controller/include/robot_controller/motion/motion_io_bridge.hpp`
- Modify: `src/robot_controller/include/robot_controller/nodes/robot_controller_node.hpp`
- Modify: `src/robot_controller/src/nodes/ros_motion_bridge.cpp`
- Modify: `src/robot_controller/src/nodes/robot_controller_node.cpp`

- [ ] **Step 1: Update MotionIOBridge interface**

In `motion_io_bridge.hpp`:
- Add include: `#include "robot_controller/nodes/motion_owner.hpp"`
- Change `submit_trajectory` signature to add `MotionSource source = MotionSource::kApi`

- [ ] **Step 2: Update RosMotionBridge in header**

In `robot_controller_node.hpp`, `RosMotionBridge` class:
- Change callback type: `using TrajectoryStartedCallback = std::function<void(MotionSource)>;`
- Change `submit_trajectory` declaration to add `MotionSource source = MotionSource::kApi`

- [ ] **Step 3: Update RosMotionBridge implementation**

In `ros_motion_bridge.cpp`, `submit_trajectory`:
- Add `MotionSource source` parameter
- Pass `source` to `on_trajectory_started_(source)`
- Add source to log message

- [ ] **Step 4: Add ownership members to RobotControllerNode**

In `robot_controller_node.hpp`, `RobotControllerNode` class:
- Add public methods: `claim_ownership(MotionOwner)`, `release_ownership()`, `get_motion_owner()`
- Add private member: `std::atomic<MotionOwner> motion_owner_{MotionOwner::kNone};`
- Add include: `#include "robot_controller/nodes/motion_owner.hpp"`

- [ ] **Step 5: Update on_trajectory_started_ callback**

In `robot_controller_node.cpp`, `init()`:
- Change lambda to accept `MotionSource source`
- Set `motion_owner_` based on source (`kApi` → `kScript`, `kService` → `kPendant`)

- [ ] **Step 6: Update kIdle behavior**

In `control_loop_tick()`, `kIdle`/`kFault` case:
- Only accept `external_joint_target_` when `motion_owner_ == kPendant`
- Auto-release pendant ownership on 200ms stream timeout

- [ ] **Step 7: Add server-side guard in external_joint_sub_**

In `init()`:
- Reject messages when `motion_owner_` is not `kNone` and not `kPendant`
- Claim `kPendant` ownership on first message from `kNone`

- [ ] **Step 8: Add jog rejection guard**

In `handle_jog_command()`:
- Return early if `motion_owner_ == kScript`

- [ ] **Step 9: Update emergency_stop()**

- Add `motion_owner_.store(MotionOwner::kNone)` at the start

- [ ] **Step 10: Update publish_status()**

- Add `msg->motion_owner = static_cast<uint8_t>(motion_owner_.load())`

- [ ] **Step 11: Build**

Run: `source install/setup.zsh && colcon build --base-paths src --packages-select robot_controller`
Expected: Build succeeds

- [ ] **Step 12: Commit**

```bash
git add src/robot_controller/include/robot_controller/motion/motion_io_bridge.hpp \
        src/robot_controller/include/robot_controller/nodes/robot_controller_node.hpp \
        src/robot_controller/src/nodes/ros_motion_bridge.cpp \
        src/robot_controller/src/nodes/robot_controller_node.cpp
git commit -m "feat(controller): implement MotionOwner arbitration in control loop"
```

---

### Task 4: Update service handlers (ownership reset on STOP/CLEAR_FAULT)

**Files:**
- Modify: `src/robot_controller/src/nodes/robot_controller_node_services.cpp`

- [ ] **Step 1: Read the service handlers file**

Read `robot_controller_node_services.cpp` to find `handle_robot_cmd`.

- [ ] **Step 2: Add ownership reset**

In `handle_robot_cmd`:
- For STOP command: add `motion_owner_.store(MotionOwner::kNone)`
- For CLEAR_FAULT command: add `motion_owner_.store(MotionOwner::kNone)`
- EMERGENCY_STOP already calls `emergency_stop()` which resets ownership

- [ ] **Step 3: Build**

Run: `source install/setup.zsh && colcon build --base-paths src --packages-select robot_controller`
Expected: Build succeeds

- [ ] **Step 4: Commit**

```bash
git add src/robot_controller/src/nodes/robot_controller_node_services.cpp
git commit -m "feat(controller): reset ownership on STOP and CLEAR_FAULT commands"
```

---

### Task 5: Pass MotionSource through RobotMotionController

**Files:**
- Modify: `src/robot_controller/include/robot_controller/motion/robot_motion_controller.hpp`
- Modify: `src/robot_controller/src/motion/robot_motion_controller.cpp`

- [ ] **Step 1: Update moveJ_internal signature**

In header, add `MotionSource source = MotionSource::kApi` parameter.

In implementation, pass `source` to `bridge_->submit_trajectory(steps, finger, source)`.

- [ ] **Step 2: Update moveL to pass MotionSource**

In `robot_motion_controller.cpp`, `moveL()` method:
- Add `MotionSource source = MotionSource::kApi` parameter to `moveL` signature (both header and implementation)
- Pass `source` to `bridge_->submit_trajectory(steps, actual_finger, source)` (around line 469)

- [ ] **Step 3: Update public moveJ signature (pose overload)**

In header and implementation, the pose-overload `moveJ(xyz, rpy, finger, block)`:
- Add `MotionSource source = MotionSource::kApi` parameter
- Pass to `moveJ_internal(target_angles, actual_finger, block, source)`

- [ ] **Step 4: Build**

Run: `source install/setup.zsh && colcon build --base-paths src --packages-select robot_controller`
Expected: Build succeeds

- [ ] **Step 5: Commit**

```bash
git add src/robot_controller/include/robot_controller/motion/robot_motion_controller.hpp \
        src/robot_controller/src/motion/robot_motion_controller.cpp
git commit -m "feat(controller): pass MotionSource through moveJ and moveL"
```

---

### Task 6: Add OwnershipGuard to GraspTaskManager

**Files:**
- Modify: `src/robot_vision/include/robot_vision/nodes/grasp_task_manager.hpp`
- Modify: `src/robot_vision/src/nodes/grasp_task_manager.cpp`
- Modify: `src/robot_vision/demo/demo_vision_grasp.cpp`

- [ ] **Step 1: Add OwnershipGuard struct using concrete RobotControllerNode type**

In `grasp_task_manager.hpp`, add before the `GraspTaskManager` class:

```cpp
#include "robot_controller/nodes/robot_controller_node.hpp"

namespace robot_vision {

/// RAII guard: claims script ownership on construction, releases on destruction.
/// Stores shared_ptr<RobotControllerNode> directly (no dynamic_pointer_cast needed).
struct OwnershipGuard {
  std::shared_ptr<robot_control::RobotControllerNode> node_;
  bool active_ = false;

  explicit OwnershipGuard(
      std::shared_ptr<robot_control::RobotControllerNode> node)
      : node_(std::move(node)) {
    if (node_) {
      node_->claim_ownership(robot_control::MotionOwner::kScript);
      active_ = true;
    }
  }

  ~OwnershipGuard() {
    if (active_ && node_) {
      node_->release_ownership();
    }
  }

  OwnershipGuard(const OwnershipGuard&) = delete;
  OwnershipGuard& operator=(const OwnershipGuard&) = delete;
};
```

- [ ] **Step 2: Add node parameter to GraspTaskManager constructor**

In the header, add as the **last** constructor parameter (with default `nullptr`):

```cpp
    std::shared_ptr<robot_control::RobotControllerNode> controller_node = nullptr);
```

Add member:
```cpp
  std::shared_ptr<robot_control::RobotControllerNode> controller_node_;
```

In the implementation, store it:
```cpp
  , controller_node_(controller_node)
```

- [ ] **Step 3: Use OwnershipGuard in run()**

In `grasp_task_manager.cpp`, at the start of `run()`:

```cpp
bool GraspTaskManager::run(double timeout) {
  // RAII: claims ownership, auto-releases on any exit
  std::optional<OwnershipGuard> owner_guard;
  if (controller_node_) {
    owner_guard.emplace(controller_node_);
  }

  auto start = std::chrono::steady_clock::now();
  state_ = GraspState::kDetecting;
  // ... rest of existing code ...
```

- [ ] **Step 4: Update demo_vision_grasp.cpp**

Pass `robot_node` as the last argument to `GraspTaskManager` constructor:

```cpp
    GraspTaskManager task(
        ctrl, vision_node,
        "panda_link0", "camera_color_optical_frame",
        kApproachHeight, kGraspHeightOffset, kGraspRpy,
        kRedetectSamples, kRedetectInterval,
        0.85, 0.05, 0.01, 100, 3,
        "panda_hand",
        std::array<double, 3>{0.0, 0.0, 0.0},
        std::array<double, 3>{-1.57079632679, 0.0, -1.57079632679},
        robot_node);  // Pass node for ownership
```

Note: Check the existing constructor call in `demo_vision_grasp.cpp` -- it may already use defaults for some parameters. The node parameter is added at the end with default `nullptr`, so existing callers are unaffected.

- [ ] **Step 5: Build**

Run: `source install/setup.zsh && colcon build --base-paths src --packages-up-to robot_vision`
Expected: Build succeeds

- [ ] **Step 6: Commit**

```bash
git add src/robot_vision/include/robot_vision/nodes/grasp_task_manager.hpp \
        src/robot_vision/src/nodes/grasp_task_manager.cpp \
        src/robot_vision/demo/demo_vision_grasp.cpp
git commit -m "feat(vision): add OwnershipGuard to GraspTaskManager"
```

---

### Task 7: Update PendantNode for status-driven stream control

**Files:**
- Modify: `src/robot_hmi/include/robot_hmi/pendant_node.hpp`
- Modify: `src/robot_hmi/src/pendant_node.cpp`

- [ ] **Step 1: Add script_active_ flag to header**

In `pendant_node.hpp`, add member:
```cpp
  std::atomic<bool> script_active_{false};
```

- [ ] **Step 2: Update on_robot_status callback**

In `pendant_node.cpp`, in `on_robot_status()`, add script ownership handling **before** the existing jog/fault logic:

```cpp
  // === Script ownership handling ===
  if (msg->motion_owner == robot_msgs::msg::RobotStatus::OWNER_SCRIPT) {
    if (!script_active_.load()) {
      RCLCPP_INFO(get_logger(), "Script took motion ownership, pausing joint stream");
      pause_joint_stream();
      script_active_.store(true);
    }
  } else if (script_active_.load() &&
             msg->motion_owner == robot_msgs::msg::RobotStatus::OWNER_NONE) {
    RCLCPP_INFO(get_logger(), "Script released motion ownership, resuming joint stream");
    script_active_.store(false);
    // Sync stream target to current actual position (existing pattern)
    {
      std::lock_guard<std::mutex> jlock(latest_joints_mutex_);
      std::lock_guard<std::mutex> slock(joint_stream_mutex_);
      joint_stream_target_ = latest_joints_;
      joint_stream_dirty_ = true;
    }
    resume_joint_stream();
  }
```

- [ ] **Step 3: Build**

Run: `source install/setup.zsh && colcon build --base-paths src --packages-up-to robot_hmi`
Expected: Build succeeds

- [ ] **Step 4: Commit**

```bash
git add src/robot_hmi/include/robot_hmi/pendant_node.hpp \
        src/robot_hmi/src/pendant_node.cpp
git commit -m "feat(hmi): pause joint stream when script owns motion control"
```

---

### Task 8: Full build verification + test

- [ ] **Step 1: Source and build all packages**

Run:
```bash
source install/setup.zsh
colcon build --base-paths src
```
Expected: All packages build successfully

- [ ] **Step 2: Run existing unit tests**

Run:
```bash
colcon test --base-paths src --packages-select robot_controller
colcon test-result --verbose
```
Expected: All existing tests pass. If MockMotionBridge needs updating for the new `submit_trajectory` signature, fix it.

- [ ] **Step 3: Fix any test/build issues and commit**

---

### Task 9: Integration test with Isaac Sim

This requires a running Isaac Sim instance and manual verification.

- [ ] **Step 1: Test demo_vision_grasp alone (no pendant)**

1. Launch Isaac Sim with the robot scene
2. Run `ros2 run robot_vision demo_vision_grasp`
3. Verify: motion is smooth, no oscillation between trajectory segments
4. Verify: `ros2 topic echo /robot_controller_node/status` shows `motion_owner: 2` during grasp, `motion_owner: 0` after

- [ ] **Step 2: Test demo_vision_grasp with pendant open**

1. Launch Isaac Sim
2. Launch `ros2 run robot_hmi robot_hmi`
3. Wait for connection
4. In another terminal, run `ros2 run robot_vision demo_vision_grasp`
5. Verify: pendant sliders freeze during grasp (stream paused)
6. Verify: no oscillation/twitching
7. Verify: after grasp completes, pendant sliders resume control

- [ ] **Step 3: Test pendant joint control alone**

1. Launch Isaac Sim
2. Launch `ros2 run robot_hmi robot_hmi`
3. Move joint sliders
4. Verify: robot follows sliders smoothly
5. Verify: `motion_owner: 1` while streaming

- [ ] **Step 4: Test E-STOP during grasp**

1. While demo_vision_grasp is running, press E-STOP on pendant
2. Verify: robot stops immediately
3. Verify: `motion_owner: 0` in status
4. Clear fault
5. Verify: pendant regains control

---

## Dependency Graph

```
Task 1 (enums) ─────────┐
Task 2 (msg) ───────────┤
                          ├──→ Task 3 (bridge + bridge impl + node) [ATOMIC]
                          │         │
                          │         ├──→ Task 4 (service handlers)
                          │         │
                          │         └──→ Task 5 (MotionController pass-through)
                          │                    │
                          │                    └──→ Task 6 (GraspTaskManager + OwnershipGuard)
                          │
Task 2 (msg) ────────────┼──→ Task 7 (PendantNode)
Task 3 (status publish) ─┘
                          │
                          └──→ Task 8 (full build + tests)
                                   │
                                   └──→ Task 9 (Isaac Sim integration)
```

## Key Design Decisions

1. **Ownership lives on `RobotControllerNode` only** -- NOT on `IRobotController`. This keeps Layer 1 (pure C++) clean of ROS2 communication concerns. `GraspTaskManager` accesses the node via a concrete `shared_ptr<RobotControllerNode>` parameter.

2. **`motion_owner_` is `std::atomic<MotionOwner>`** -- lock-free reads from the 100Hz control loop. `MotionOwner` is an enum with `uint8_t` underlying type, which is trivially copyable and guaranteed to work with `std::atomic`.

3. **Tasks 3 (bridge + node) is atomic** -- the callback signature change in `RosMotionBridge` breaks existing code until `RobotControllerNode` is updated. These must be committed together.

4. **OwnershipGuard uses concrete `shared_ptr<RobotControllerNode>`** -- no `dynamic_pointer_cast` needed since `demo_vision_grasp` already has the concrete type.

5. **Server-side rejection of pendant messages** -- the controller drops incoming `joint_target` messages when `motion_owner_` is `kScript`, even before the pendant sees the status update. This eliminates the 100ms latency window.

6. **Named constants in `RobotStatus.msg`** -- `OWNER_NONE=0`, `OWNER_PENDANT=1`, `OWNER_SCRIPT=2` -- avoids magic numbers in PendantNode.
