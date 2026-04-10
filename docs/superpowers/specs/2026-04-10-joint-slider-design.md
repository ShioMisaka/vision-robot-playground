# Joint Control Slider Design

## Goal

Replace the current `QDoubleSpinBox`-based joint angle controls in the teaching pendant with `QSlider` + editable `QLineEdit` (degrees), enabling real-time robot following as the user drags the slider.

## Current State

- Joint control: 7x `QDoubleSpinBox` (radians) + 7x read-only `QLabel` (radians) + "Move Joints" button
- `onRefreshState()` polls at 5Hz, updates `label_joints_[]` with real joint angles from `async_get_state`
- `onMoveJoint()` reads all 7 spinboxes and calls `async_move_joint()` with `block=true`
- `post_task` queue is single-threaded serial, shared by all operations (state poll, moves, gripper, etc.)

## Design

### UI Layout per Joint (7 rows)

```
[J1:] [========QSlider========] [QLineEdit: 0.0°]
[J2:] [========QSlider========] [QLineEdit: 0.0°]
...
[J7:] [========QSlider========] [QLineEdit: 0.0°]
```

- **QSlider**: horizontal, range mapped per-joint to Panda joint limits. Integer steps = hundredths of a degree (`int_slider = round(deg * 100)`), giving 0.01° resolution. QSlider tracking enabled (value changes continuously during drag).
- **QLineEdit**: displays current angle in **degrees** (1 decimal place). Editable — press Enter to execute moveJoint to that exact angle.
- **"Move Joints" button removed** — slider follow and line edit Enter replace it.

### Per-Joint Slider Ranges (from panda_profile.hpp)

| Joint | Lower (deg) | Upper (deg) | Slider int range |
|-------|-------------|-------------|------------------|
| J1    | -166.0      | 166.0       | -16600 ~ 16600   |
| J2    | -101.0      | 101.0       | -10100 ~ 10100   |
| J3    | -166.0      | 166.0       | -16600 ~ 16600   |
| J4    | -176.0      | -4.0        | -17600 ~ -400    |
| J5    | -166.0      | 166.0       | -16600 ~ 16600   |
| J6    | -1.0        | 215.0       | -100 ~ 21500     |
| J7    | -166.0      | 166.0       | -16600 ~ 16600   |

Each slider's range is set individually per joint. Limits are hardcoded from `panda_profile.hpp` values (rounded to nearest 0.01°).

### Data Flow

#### 1. Robot → UI (state sync)

`onRefreshState()` callback receives real joint angles (radians). For each joint:
- Convert rad → deg
- If slider is NOT user-controlled (see interaction lock below), update slider position and line edit text
- Sync `last_synced_joints_[i]` (radians) to match what was just written to sliders. This prevents the follow timer from detecting the state-refresh update as a "user change" and sending a redundant move command.

#### 2. Slider drag → Robot (real-time follow via dedicated streaming path)

**Problem with using `post_task` + `block=true`:** The existing `async_move_joint` sets `block=true`, which blocks until trajectory completion (seconds). The single-threaded `post_task` queue would starve all other operations. A 50Hz send rate is impossible with blocking calls.

**Solution: Dedicated joint streaming thread in PendantNode.**

New PendantNode members:
```cpp
std::mutex joint_stream_mutex_;
std::array<double, 7> joint_stream_target_;  // latest target (rad)
bool joint_stream_dirty_ = false;             // new target since last send
std::thread joint_stream_thread_;
std::atomic<bool> joint_stream_running_{false};
```

New PendantNode methods:
- `start_joint_stream(const std::array<double, 7>& initial)` — starts streaming thread
- `update_joint_target(const std::array<double, 7>& target)` — stores latest target (mutex-protected), sets dirty flag
- `stop_joint_stream()` — stops streaming thread

Streaming thread loop (runs at 50Hz = 20ms interval):
```
while (running) {
  sleep 20ms
  lock mutex
  if (!dirty) continue
  copy target, clear dirty
  unlock mutex
  send MoveJoint service call with block=false, timeout=200ms
}
```

With `block=false`, the service call returns immediately after the controller accepts the target. The controller handles trajectory replanning (cancel previous, start new). This allows true 50Hz streaming without queue starvation.

**MainWindow follow timer (20ms QTimer):**
- Reads all 7 slider values, converts to radians
- Compares with `last_streamed_joints_[i]`
- If any joint delta > 0.003 rad (~0.17°), calls `node_->update_joint_target(...)` and updates `last_streamed_joints_`

**Lifecycle:**
- Stream thread starts when MainWindow is constructed, stops in destructor
- E-stop: MainWindow sets `estop_active_`, follow timer skips sends. PendantNode's `emergency_stop()` also stops the stream thread.
- When estop is cleared (requires separate action), user must re-drag sliders to restart streaming.

#### 3. Line edit → Robot (direct input)

User types a degree value, presses Enter:
- Convert deg → rad, clamp to per-joint limits
- Move slider to matching position
- Call `update_joint_target()` immediately (streams to robot)
- Start interaction lock (2s) to prevent state refresh from overwriting

### Interaction Lock

Prevents robot state feedback from overriding the slider while user is actively dragging:

- `slider_is_controlled_[7]` — per-joint bool array
- `QSlider::sliderPressed` → set `slider_is_controlled_[i] = true`
- `QSlider::sliderReleased` → start 2-second per-joint lock timer
- Lock timer fires → set `slider_is_controlled_[i] = false`
- `QLineEdit::editingFinished` (Enter pressed) → also clears the lock for that joint immediately (so state refresh resumes normally after the commanded position is reached)
- While `slider_is_controlled_[i] == true`, `onRefreshState` does NOT update that slider or lineedit
- **E-stop check:** follow timer and line edit Enter handler both check `estop_active_` before sending

### Unit Conversion

- Slider works in integer hundredths of a degree (0.01° steps): `int_slider = round(deg * 100)`
- Display shows degrees with 1 decimal: `QString::number(deg, 'f', 1) + "°"`
- Conversion helpers: `degToSlider(double deg, int joint)`, `sliderToDeg(int val)`, `degToRad(double deg)`, `radToDeg(double rad)`
- All conversion is in MainWindow only; PendantNode always works in radians

### E-STOP Integration

- Follow timer checks `estop_active_` — if true, skips sending entirely
- Line edit Enter also checks `estop_active_`
- `PendantNode::emergency_stop()` calls `stop_joint_stream()` to halt the streaming thread
- State refresh (`onRefreshState`) continues running to show robot state even during fault

## Files to Modify

1. **`main_window.hpp`** — Replace `QDoubleSpinBox* spin_joint_[7]` with `QSlider* slider_joint_[7]` + `QLineEdit* edit_joint_[7]`; add follow timer, interaction lock state, `last_streamed_joints_[]`, per-joint timer arrays, conversion helpers
2. **`main_window.cpp`** — Rewrite `createControlPanel()` joint section (remove spinboxes and Move Joints button), add follow timer logic, add line edit Enter handler, update `onRefreshState()` sync logic, add E-stop checks
3. **`pendant_node.hpp`** — Add joint streaming members (thread, mutex, target buffer, dirty flag), add `start_joint_stream()`, `update_joint_target()`, `stop_joint_stream()` methods
4. **`pendant_node.cpp`** — Implement streaming thread loop with `block=false` MoveJoint calls, implement lifecycle methods, update `emergency_stop()` to stop stream, update destructor
