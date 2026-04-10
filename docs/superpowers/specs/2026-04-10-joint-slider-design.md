# Joint Control Slider Design

## Goal

Replace the current `QDoubleSpinBox`-based joint angle controls in the teaching pendant with `QSlider` + editable `QLineEdit` (degrees), enabling real-time robot following as the user drags the slider.

## Current State

- Joint control: 7x `QDoubleSpinBox` (radians) + 7x read-only `QLabel` (radians) + "Move Joints" button
- `onRefreshState()` polls at 5Hz, updates `label_joints_[]` with real joint angles from `async_get_state`
- `onMoveJoint()` reads all 7 spinboxes and calls `async_move_joint()`

## Design

### UI Layout per Joint (7 rows)

```
[J1:] [========QSlider========] [QLineEdit: 0.00°] 
[J2:] [========QSlider========] [QLineEdit: 0.00°]
...
[J7:] [========QSlider========] [QLineEdit: 0.00°]
```

- **QSlider**: horizontal, range mapped to Panda joint limits (integer × 100 for 0.01° resolution)
- **QLineEdit**: displays current angle in **degrees** (1 decimal place). Editable — press Enter to execute moveJoint to that exact angle.

### Data Flow

#### 1. Robot → UI (state sync)

`onRefreshState()` callback receives real joint angles (radians). For each joint:
- Convert rad → deg
- If slider is NOT user-controlled (see interaction lock below), update slider position and line edit
- This keeps UI in sync with actual robot state

#### 2. Slider drag → Robot (real-time follow)

A dedicated **50Hz QTimer** (`joint_follow_timer_`) checks each slider:
- Compare slider value (converted to rad) with `last_sent_joints_[i]`
- If delta > 0.005 rad (~0.3°), send one `async_move_joint()` with all 7 current slider positions
- Update `last_sent_joints_` after send

#### 3. Line edit → Robot (direct input)

User types a degree value, presses Enter:
- Convert deg → rad, clamp to joint limits
- Move slider to matching position
- Send `async_move_joint()` immediately
- Same interaction lock applies

### Interaction Lock

Prevents robot state feedback from overriding the slider while user is actively dragging:

- `slider_is_controlled_[7]` — per-joint bool array
- `QSlider::sliderPressed` → set `slider_is_controlled_[i] = true`, start 2-second lock timer for this joint
- `QSlider::sliderReleased` → start 2-second lock timer for this joint
- Lock timer fires → set `slider_is_controlled_[i] = false`
- While `slider_is_controlled_[i] == true`, `onRefreshState` does NOT update that slider/lineedit

### Unit Conversion

- Slider works in integer "decidegrees" (0.1° steps): `int_slider = round(deg * 10)`
- Display shows degrees with 1 decimal: `QString::number(deg, 'f', 1)`
- Conversion helpers: `degToSlider(double deg)`, `sliderToDeg(int val)`, `degToRad(double deg)`, `radToDeg(double rad)`
- Panda joint limits (rad) are stored and used for clamping

### Throttling

- 50Hz follow timer checks all 7 sliders in one pass
- Only sends `async_move_joint()` when at least one joint changed beyond threshold (0.005 rad)
- Each send goes through the existing `post_task` queue (serial, no overlap)

## Files to Modify

1. **`main_window.hpp`** — Replace `QDoubleSpinBox* spin_joint_[7]` with `QSlider* slider_joint_[7]` + `QLineEdit* edit_joint_[7]`; add follow timer, interaction lock state, conversion helpers
2. **`main_window.cpp`** — Rewrite `createControlPanel()` joint section, add follow timer logic, add line edit Enter handler, update `onRefreshState()` sync logic

No changes to `pendant_node.hpp`/`.cpp` — the existing `async_move_joint()` interface is sufficient.
