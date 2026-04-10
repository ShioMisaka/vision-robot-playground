# Joint Control Slider Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace QDoubleSpinBox joint controls with QSlider + QLineEdit (degrees) for real-time robot following.

**Architecture:** Dedicated streaming thread in PendantNode sends `block=false` MoveJoint at 50Hz. MainWindow 20ms QTimer polls sliders and feeds targets to PendantNode. Interaction lock prevents state-refresh from overwriting sliders during user drag. Streaming thread pauses (not destroys) on e-stop for clean resume.

**Tech Stack:** Qt5 (QSlider, QLineEdit, QTimer), C++17, ROS2 Jazzy services, std::thread

**Testing note:** This project has no unit test framework. Verification relies on build compilation (`colcon build`) + manual integration testing (Task 5).

---

## Task 1: Add joint streaming to PendantNode

**Files:**
- Modify: `src/teaching_pendant/include/teaching_pendant/pendant_node.hpp`
- Modify: `src/teaching_pendant/src/pendant_node.cpp`

### pendant_node.hpp changes

Add these members and methods to the `PendantNode` class:

```cpp
// After the existing "急停" section (public), add:

// === 关节实时流控 ===

void start_joint_stream(const std::array<double, 7>& initial);
void update_joint_target(const std::array<double, 7>& target);
void stop_joint_stream();
void pause_joint_stream();
void resume_joint_stream();

// Add to private section:
std::mutex joint_stream_mutex_;
std::array<double, 7> joint_stream_target_{};
bool joint_stream_dirty_ = false;
std::thread joint_stream_thread_;
std::atomic<bool> joint_stream_running_{false};
std::atomic<bool> joint_stream_paused_{false};
```

### pendant_node.cpp — implement streaming

Add `#include <cmath>` at the top if not already present.

**`start_joint_stream`:**
```cpp
void PendantNode::start_joint_stream(const std::array<double, 7>& initial) {
  if (joint_stream_running_.load()) return;
  {
    std::lock_guard<std::mutex> lock(joint_stream_mutex_);
    joint_stream_target_ = initial;
    joint_stream_dirty_ = false;  // don't send until user moves slider
  }
  joint_stream_running_ = true;
  joint_stream_paused_ = false;
  joint_stream_thread_ = std::thread([this]() {
    while (joint_stream_running_.load()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));  // 50Hz
      if (joint_stream_paused_.load()) continue;
      std::array<double, 7> target;
      bool should_send = false;
      {
        std::lock_guard<std::mutex> lock(joint_stream_mutex_);
        if (!joint_stream_dirty_) continue;
        target = joint_stream_target_;
        joint_stream_dirty_ = false;
        should_send = true;
      }
      if (!should_send) continue;
      if (!cli_move_joint_->service_is_ready()) continue;
      auto req = std::make_shared<robot_control_msgs::srv::MoveJoint::Request>();
      req->joint_angles.assign(target.begin(), target.end());
      req->block = false;
      auto future = cli_move_joint_->async_send_request(req);
      if (future.wait_for(std::chrono::milliseconds(200)) == std::future_status::timeout) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Joint stream send timeout");
      }
    }
  });
}
```

**`update_joint_target`:**
```cpp
void PendantNode::update_joint_target(const std::array<double, 7>& target) {
  std::lock_guard<std::mutex> lock(joint_stream_mutex_);
  joint_stream_target_ = target;
  joint_stream_dirty_ = true;
}
```

**`stop_joint_stream`:**
```cpp
void PendantNode::stop_joint_stream() {
  joint_stream_running_ = false;
  if (joint_stream_thread_.joinable()) {
    joint_stream_thread_.join();
  }
}
```

**`pause_joint_stream`:**
```cpp
void PendantNode::pause_joint_stream() {
  joint_stream_paused_ = true;
}
```

**`resume_joint_stream`:**
```cpp
void PendantNode::resume_joint_stream() {
  joint_stream_paused_ = false;
}
```

**Update destructor:**
```cpp
PendantNode::~PendantNode() {
  stop_joint_stream();    // stop streaming thread
  stop_jog();             // stop jog thread
  task_running_ = false;
  task_cv_.notify_all();
  if (task_thread_.joinable()) {
    task_thread_.join();
  }
}
```

**Update `emergency_stop`:** Replace the entire method body:
```cpp
void PendantNode::emergency_stop() {
  pause_joint_stream();  // pause, don't destroy — allows resume after fault clear
  stop_jog();
}
```

- [ ] **Step 1:** Add member declarations to `pendant_node.hpp` (public methods + private members)
- [ ] **Step 2:** Add `#include <cmath>` to `pendant_node.cpp` if missing
- [ ] **Step 3:** Implement `start_joint_stream()`, `update_joint_target()`, `stop_joint_stream()`, `pause_joint_stream()`, `resume_joint_stream()` in `pendant_node.cpp`
- [ ] **Step 4:** Update destructor and `emergency_stop()` in `pendant_node.cpp`
- [ ] **Step 5:** Build to verify compilation: `colcon build --base-paths src --packages-select teaching_pendant`
- [ ] **Step 6:** Commit: `feat(pendant): add joint streaming thread for real-time slider follow`

---

## Task 2: Replace SpinBox with QSlider + QLineEdit in MainWindow header

**Files:**
- Modify: `src/teaching_pendant/include/teaching_pendant/main_window.hpp`

### Remove
- `QDoubleSpinBox* spin_joint_[7];` (line 83)

### Add to includes
```cpp
#include <QLineEdit>
```

### Replace private members

Replace `QDoubleSpinBox* spin_joint_[7];` with:
```cpp
// Joint slider controls
QSlider* slider_joint_[7];
QLineEdit* edit_joint_[7];
QTimer* joint_follow_timer_;
QTimer* lock_timer_[7];  // per-joint interaction lock timer
bool slider_is_controlled_[7] = {};
std::array<double, 7> last_streamed_joints_{};
```

### Add private helper methods
```cpp
// Unit conversion helpers
static double radToDeg(double rad);
static double degToRad(double deg);
static int degToSlider(double deg);
static double sliderToDeg(int val);
void syncSliderToState(int joint, double rad);
void onJointSliderPressed(int joint);
void onJointSliderReleased(int joint);
void onJointEditFinished(int joint);
void onJointFollowTick();
```

### Remove slot
Remove `void onMoveJoint();` from the private slots section (no longer needed).

- [ ] **Step 1:** Edit `main_window.hpp` — add `#include <QLineEdit>`, remove `QDoubleSpinBox* spin_joint_[7]`, remove `onMoveJoint()` slot, add new members and helper declarations
- [ ] **Step 2:** Commit: `refactor(pendant): replace SpinBox members with QSlider/QLineEdit declarations`

---

## Task 3: Rewrite joint control UI and wire up logic in MainWindow

**Files:**
- Modify: `src/teaching_pendant/src/main_window.cpp`

This is the largest task. Breakdown:

### 3a. Add joint limit constants at top of file (after `#include` block)

```cpp
namespace {

// Panda joint limits in degrees (from panda_profile.hpp, converted)
constexpr double kJointLowerDeg[7] = {
    -166.0, -101.0, -166.0, -176.0, -166.0, -1.0, -166.0};
constexpr double kJointUpperDeg[7] = {
    166.0, 101.0, 166.0, -4.0, 166.0, 215.0, 166.0};

}  // namespace
```

### 3b. Add conversion helpers at bottom of `teaching_pendant` namespace

```cpp
double MainWindow::radToDeg(double rad) {
  return rad * 180.0 / M_PI;
}

double MainWindow::degToRad(double deg) {
  return deg * M_PI / 180.0;
}

int MainWindow::degToSlider(double deg) {
  return static_cast<int>(std::round(deg * 100.0));
}

double MainWindow::sliderToDeg(int val) {
  return val / 100.0;
}
```

### 3c. Rewrite `createControlPanel()` joint section

Replace the entire "左侧：关节控制" block (lines 151-175 in current code) with:

```cpp
// === 左侧：关节控制（Slider + 角度输入）===
auto* joint_group = new QGroupBox("Joint Control", this);
auto* joint_layout = new QGridLayout(joint_group);
joint_layout->setColumnStretch(1, 1);  // slider stretches

for (int i = 0; i < 7; ++i) {
  joint_layout->addWidget(new QLabel(QString("J%1:").arg(i + 1)), i, 0);

  // Slider: range in hundredths of a degree
  slider_joint_[i] = new QSlider(Qt::Horizontal, this);
  slider_joint_[i]->setRange(
      degToSlider(kJointLowerDeg[i]),
      degToSlider(kJointUpperDeg[i]));
  slider_joint_[i]->setValue(0);  // will be synced from robot state
  slider_joint_[i]->setTracking(true);
  joint_layout->addWidget(slider_joint_[i], i, 1);

  // Line edit: degrees, editable
  edit_joint_[i] = new QLineEdit("0.0°", this);
  edit_joint_[i]->setFixedWidth(70);
  edit_joint_[i]->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  QFont mono("Monospace");
  edit_joint_[i]->setFont(mono);
  joint_layout->addWidget(edit_joint_[i], i, 2);

  // Per-joint interaction lock timer
  lock_timer_[i] = new QTimer(this);
  lock_timer_[i]->setSingleShot(true);
  lock_timer_[i]->setInterval(2000);

  // Capture i by value in lambdas
  const int joint_idx = i;

  connect(slider_joint_[joint_idx], &QSlider::sliderPressed, this,
          [this, joint_idx]() { onJointSliderPressed(joint_idx); });
  connect(slider_joint_[joint_idx], &QSlider::sliderReleased, this,
          [this, joint_idx]() { onJointSliderReleased(joint_idx); });
  connect(edit_joint_[joint_idx], &QLineEdit::editingFinished, this,
          [this, joint_idx]() { onJointEditFinished(joint_idx); });
  connect(lock_timer_[joint_idx], &QTimer::timeout, this,
          [this, joint_idx]() { slider_is_controlled_[joint_idx] = false; });
}
```

**Remove** the old `btn_move_joint` button creation, its stylesheet, its `connect`, and its `addWidget` line.

### 3d. Add new slot implementations

```cpp
void MainWindow::syncSliderToState(int joint, double rad) {
  if (slider_is_controlled_[joint]) return;
  double deg = radToDeg(rad);
  slider_joint_[joint]->blockSignals(true);
  slider_joint_[joint]->setValue(degToSlider(deg));
  slider_joint_[joint]->blockSignals(false);
  edit_joint_[joint]->setText(QString::number(deg, 'f', 1) + "°");
}

void MainWindow::onJointSliderPressed(int joint) {
  slider_is_controlled_[joint] = true;
  lock_timer_[joint]->stop();  // cancel any pending unlock
}

void MainWindow::onJointSliderReleased(int joint) {
  // Lock persists for 2 seconds after release
  lock_timer_[joint]->start();
}

void MainWindow::onJointEditFinished(int joint) {
  if (estop_active_.load()) return;

  QString text = edit_joint_[joint]->text();
  text.remove('°');  // strip degree symbol
  bool ok = false;
  double deg = text.toDouble(&ok);
  if (!ok) return;

  // Clamp to joint limits
  deg = std::max(kJointLowerDeg[joint], std::min(kJointUpperDeg[joint], deg));

  // Update slider
  slider_joint_[joint]->blockSignals(true);
  slider_joint_[joint]->setValue(degToSlider(deg));
  slider_joint_[joint]->blockSignals(false);

  // Update display
  edit_joint_[joint]->setText(QString::number(deg, 'f', 1) + "°");

  // Send target immediately
  std::array<double, 7> target{};
  for (int i = 0; i < 7; ++i) {
    target[i] = degToRad(sliderToDeg(slider_joint_[i]->value()));
  }
  node_->update_joint_target(target);
  last_streamed_joints_ = target;

  // Start interaction lock
  slider_is_controlled_[joint] = true;
  lock_timer_[joint]->start();
}

void MainWindow::onJointFollowTick() {
  if (estop_active_.load()) return;

  std::array<double, 7> target{};
  bool changed = false;
  for (int i = 0; i < 7; ++i) {
    target[i] = degToRad(sliderToDeg(slider_joint_[i]->value()));
    if (std::abs(target[i] - last_streamed_joints_[i]) > 0.003) {
      changed = true;
    }
  }
  if (changed) {
    node_->update_joint_target(target);
    last_streamed_joints_ = target;
  }
}
```

### 3e. Start joint stream and follow timer in constructor

After `refresh_timer_->start(200);` in the constructor, add:

```cpp
// 关节跟随定时器（50Hz）
joint_follow_timer_ = new QTimer(this);
connect(joint_follow_timer_, &QTimer::timeout, this, &MainWindow::onJointFollowTick);
joint_follow_timer_->start(20);

// 启动关节流控线程
std::array<double, 7> initial{};
node_->start_joint_stream(initial);
```

### 3f. Update `onRefreshState()` — fix e-stop check and replace label_joints_ update

Replace the `if (estop_active_.load()) return;` guard at the top of `onRefreshState()` with a guard that only skips the `async_get_state` call, not the connection status update:

```cpp
void MainWindow::onRefreshState() {
  // Connection status should update even during e-stop
  bool robot_topic = node_->is_robot_connected();
  bool services = node_->are_services_ready();
  if (robot_topic && services) {
    label_robot_status_->setText("Connected");
    label_robot_status_->setStyleSheet("color: green; font-weight: bold;");
  } else if (robot_topic || services) {
    label_robot_status_->setText("Connecting...");
    label_robot_status_->setStyleSheet("color: orange; font-weight: bold;");
  } else {
    label_robot_status_->setText("Disconnected");
    label_robot_status_->setStyleSheet("color: red; font-weight: bold;");
  }

  // Skip state polling during e-stop (but allow it to show robot state)
  if (!services) return;
  if (estop_active_.load()) {
    // Still poll state so sliders show actual robot position during fault
    // but don't send any commands
  }

  node_->async_get_state(
      [this](bool success,
             const std::vector<double>& joints,
             const std::array<double, 6>& pose,
             double finger,
             const std::string& tcp) {
        if (!success) return;
        QMetaObject::invokeMethod(this, [this, joints, pose, finger, tcp]() {
          // Sync sliders instead of labels
          for (int i = 0; i < 7 && i < (int)joints.size(); ++i) {
            syncSliderToState(i, joints[i]);
            last_streamed_joints_[i] = joints[i];
          }
          for (int i = 0; i < 6; ++i) {
            label_pose_[i]->setText(QString::number(pose[i], 'f', 4));
          }
          label_finger_->setText(QString::number(finger, 'f', 4));
          label_tcp_->setText(QString::fromStdString(tcp));
        });
      });
}
```

### 3g. Remove `onMoveJoint()` implementation

Delete the entire `onMoveJoint()` method body from `main_window.cpp`.

### 3h. Remove `label_joints_[]` usage

The old `label_joints_[7]` array is no longer updated or needed. Leave it declared in the header for now (cleaned up in Task 4).

- [ ] **Step 1:** Add joint limit constants and conversion helpers to `main_window.cpp`
- [ ] **Step 2:** Rewrite joint section in `createControlPanel()` — replace SpinBox with QSlider + QLineEdit, remove Move Joints button
- [ ] **Step 3:** Add `syncSliderToState()`, `onJointSliderPressed()`, `onJointSliderReleased()`, `onJointEditFinished()`, `onJointFollowTick()` implementations
- [ ] **Step 4:** Start `joint_follow_timer_` and `start_joint_stream()` in constructor
- [ ] **Step 5:** Rewrite `onRefreshState()` — allow state polling during e-stop, replace label_joints_ with slider sync
- [ ] **Step 6:** Remove `onMoveJoint()` method body
- [ ] **Step 7:** Build: `colcon build --base-paths src --packages-select teaching_pendant`
- [ ] **Step 8:** Fix any compilation errors
- [ ] **Step 9:** Commit: `feat(pendant): replace SpinBox joint control with QSlider + degree input`

---

## Task 4: Cleanup unused members

**Files:**
- Modify: `src/teaching_pendant/include/teaching_pendant/main_window.hpp`
- Modify: `src/teaching_pendant/src/main_window.cpp`

- [ ] **Step 1:** Remove `QLabel* label_joints_[7];` from `main_window.hpp` private members
- [ ] **Step 2:** Remove any remaining references to `label_joints_` in `main_window.cpp` (should be none after Task 3)
- [ ] **Step 3:** Keep `#include <QDoubleSpinBox>` — still used by `spin_xyz_` and `spin_rpy_`
- [ ] **Step 4:** Build: `colcon build --base-paths src --packages-select teaching_pendant`
- [ ] **Step 5:** Commit: `chore(pendant): remove unused label_joints members`

---

## Task 5: Manual integration test

This project requires Isaac Sim for runtime testing. Verify:

- [ ] **Step 1:** Start Isaac Sim with robot controller node running
- [ ] **Step 2:** Run `ros2 run teaching_pendant teaching_pendant`
- [ ] **Step 3:** Verify sliders sync to current robot joint positions on startup
- [ ] **Step 4:** Drag a slider — robot should follow in real time
- [ ] **Step 5:** Type a degree value in a line edit, press Enter — robot should move to that position
- [ ] **Step 6:** Press E-STOP during slider drag — robot should stop, slider sends should cease
- [ ] **Step 7:** Clear fault (if button available) — verify streaming resumes and sliders work again
- [ ] **Step 8:** Verify state refresh updates sliders when user is NOT dragging (auto-sync works)
- [ ] **Step 9:** Verify state refresh does NOT override sliders during drag (interaction lock works)
