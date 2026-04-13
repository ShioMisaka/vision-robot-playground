# Teaching Pendant Panel Refactor

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Split monolithic `MainWindow` into 6 independent Panel widgets for maintainability and future growth.

**Architecture:** Each Panel is a self-contained QWidget subclass with its own UI creation + logic. Panels receive a `PendantNode` pointer for direct async calls. Cross-panel communication (E-STOP) uses Qt signals. MainWindow becomes a thin orchestrator (~50 lines) that creates and lays out panels.

**Tech Stack:** Qt5 Widgets (C++17), ROS2 Jazzy

---

## File Structure

### New files to create:
| File | Responsibility |
|------|----------------|
| `include/teaching_pendant/panels/connection_bar.hpp` | Connection status indicators |
| `src/panels/connection_bar.cpp` | Connection bar implementation |
| `include/teaching_pendant/panels/camera_panel.hpp` | Camera feed display + RGB/Depth toggle |
| `src/panels/camera_panel.cpp` | Camera panel implementation |
| `include/teaching_pendant/panels/robot_state_bar.hpp` | Robot pose/finger/TCP display |
| `src/panels/robot_state_bar.cpp` | State bar implementation |
| `include/teaching_pendant/panels/joint_control_panel.hpp` | Joint slider + edit + streaming logic |
| `src/panels/joint_control_panel.cpp` | Joint control implementation |
| `include/teaching_pendant/panels/cartesian_panel.hpp` | XYZ/RPY input + Jog + motion mode |
| `src/panels/cartesian_panel.cpp` | Cartesian control implementation |
| `include/teaching_pendant/panels/function_panel.hpp` | Speed sliders + gripper + home + E-STOP |
| `src/panels/function_panel.cpp` | Function panel implementation |

### Files to modify:
| File | Change |
|------|--------|
| `include/teaching_pendant/main_window.hpp` | Strip to thin orchestrator |
| `src/main_window.cpp` | Strip to thin orchestrator |
| `CMakeLists.txt` | Add new source files, update include paths |

### Files unchanged:
| File | Reason |
|------|--------|
| `src/main.cpp` | No changes needed |
| `src/pendant_node.cpp` | No changes needed |
| `include/teaching_pendant/pendant_node.hpp` | No changes needed |
| `package.xml` | No changes needed |

---

## Signal/Slot Design for Cross-Panel Communication

```
FunctionPanel --[estopActivated()]--> MainWindow --[broadcast]--> all panels
                                                  --> JointControlPanel::onEstopChanged(bool)
                                                  --> CartesianPanel::onEstopChanged(bool)

MainWindow --[connectionChanged(bool robot, bool camera)]--> ConnectionBar
MainWindow --[imageReceived(cv::Mat)]--> CameraPanel
MainWindow --[stateUpdated(joints, pose, finger, tcp)]--> RobotStateBar
                                                        --> JointControlPanel::onStateUpdated(...)
```

The `estop_active_` flag is replaced by a signal from `FunctionPanel` relayed through `MainWindow`. Each panel stores its own `estop_active_` bool updated via slot.

---

## Task 1: Create Panel Directory Structure

**Files:**
- Create: `include/teaching_pendant/panels/` (directory)
- Create: `src/panels/` (directory)

- [ ] **Step 1: Create directories**

```bash
mkdir -p src/teaching_pendant/include/teaching_pendant/panels
mkdir -p src/teaching_pendant/src/panels
```

- [ ] **Step 2: Commit**

```bash
git add -A src/teaching_pendant/include/teaching_pendant/panels src/teaching_pendant/src/panels
git commit -m "chore(pendant): create panels directory structure for refactor"
```

---

## Task 2: ConnectionBar Panel

**Files:**
- Create: `include/teaching_pendant/panels/connection_bar.hpp`
- Create: `src/panels/connection_bar.cpp`

**Responsibilities extracted from MainWindow:**
- `createConnectionBar()` UI (lines 83-104)
- Robot/camera status label updates (lines 61-64, 421-431)
- Connection callback setup (lines 28-36)

**Public API:**
```cpp
class ConnectionBar : public QWidget {
  Q_OBJECT
public:
  explicit ConnectionBar(std::shared_ptr<PendantNode> node, QWidget* parent = nullptr);
public slots:
  void onConnectionChanged(bool robot, bool camera);
};
```

`MainWindow` calls `onConnectionChanged` from the PendantNode connection callback.

- [ ] **Step 1: Write header**

```cpp
// include/teaching_pendant/panels/connection_bar.hpp
#pragma once

#include <QWidget>
#include <QLabel>
#include <memory>

namespace teaching_pendant {

class PendantNode;

class ConnectionBar : public QWidget {
  Q_OBJECT
public:
  explicit ConnectionBar(QWidget* parent = nullptr);

public slots:
  /// Called when robot topic or services change state
  void onRobotConnectionChanged(bool connected, bool services_ready);
  /// Called when camera connection changes
  void onCameraConnectionChanged(bool connected);

private:
  QLabel* label_robot_status_;
  QLabel* label_camera_status_;
};

}  // namespace teaching_pendant
```

- [ ] **Step 2: Write implementation**

```cpp
// src/panels/connection_bar.cpp
#include "teaching_pendant/panels/connection_bar.hpp"

#include <QHBoxLayout>
#include <QFrame>

namespace teaching_pendant {

ConnectionBar::ConnectionBar(QWidget* parent) : QWidget(parent) {
  auto* bar = new QFrame(this);
  bar->setFrameShape(QFrame::StyledPanel);
  auto* layout = new QHBoxLayout(bar);
  layout->setContentsMargins(8, 4, 8, 4);

  layout->addWidget(new QLabel("Robot:"));
  label_robot_status_ = new QLabel("Disconnected");
  label_robot_status_->setMinimumWidth(100);
  label_robot_status_->setStyleSheet("color: red; font-weight: bold;");
  layout->addWidget(label_robot_status_);

  layout->addSpacing(20);

  layout->addWidget(new QLabel("Camera:"));
  label_camera_status_ = new QLabel("Disconnected");
  label_camera_status_->setMinimumWidth(100);
  label_camera_status_->setStyleSheet("color: red; font-weight: bold;");
  layout->addWidget(label_camera_status_);

  layout->addStretch();

  auto* outer = new QHBoxLayout(this);
  outer->setContentsMargins(0, 0, 0, 0);
  outer->addWidget(bar);
}

void ConnectionBar::onRobotConnectionChanged(bool connected, bool services_ready) {
  if (connected && services_ready) {
    label_robot_status_->setText("Connected");
    label_robot_status_->setStyleSheet("color: green; font-weight: bold;");
  } else if (connected || services_ready) {
    label_robot_status_->setText("Connecting...");
    label_robot_status_->setStyleSheet("color: orange; font-weight: bold;");
  } else {
    label_robot_status_->setText("Disconnected");
    label_robot_status_->setStyleSheet("color: red; font-weight: bold;");
  }
}

void ConnectionBar::onCameraConnectionChanged(bool connected) {
  label_camera_status_->setText(connected ? "Connected" : "Disconnected");
  label_camera_status_->setStyleSheet(
      connected ? "color: green; font-weight: bold;"
                : "color: red; font-weight: bold;");
}

}  // namespace teaching_pendant
```

---

## Task 3: CameraPanel

**Files:**
- Create: `include/teaching_pendant/panels/camera_panel.hpp`
- Create: `src/panels/camera_panel.cpp`

**Responsibilities extracted from MainWindow:**
- `createCameraPanel()` UI (lines 106-128)
- Image display callback (lines 39-48)
- RGB/Depth toggle (lines 359-362)
- E-STOP visual state (lines 382-386)

**Public API:**
```cpp
class CameraPanel : public QWidget {
  Q_OBJECT
public:
  explicit CameraPanel(QWidget* parent = nullptr);
public slots:
  void onImageReceived(const QImage& image);
  void onEstopChanged(bool active);
private slots:
  void onToggleRgbDepth();
private:
  QLabel* image_label_;
  bool show_depth_ = false;
};
```

Note: `cv::Mat` → `QImage` conversion moves into `MainWindow` relay or stays in callback. The panel receives `QImage` to stay OpenCV-free.

- [ ] **Step 1: Write header**

```cpp
// include/teaching_pendant/panels/camera_panel.hpp
#pragma once

#include <QWidget>
#include <QLabel>
#include <QPushButton>
#include <QImage>

namespace teaching_pendant {

class CameraPanel : public QWidget {
  Q_OBJECT
public:
  explicit CameraPanel(QWidget* parent = nullptr);
public slots:
  void onImageReceived(const QImage& image);
  void onEstopChanged(bool active);
private slots:
  void onToggleRgbDepth();
private:
  QLabel* image_label_;
  QPushButton* btn_toggle_;
  bool show_depth_ = false;
};

}  // namespace teaching_pendant
```

- [ ] **Step 2: Write implementation**

```cpp
// src/panels/camera_panel.cpp
#include "teaching_pendant/panels/camera_panel.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QPixmap>
#include <QSizePolicy>

namespace teaching_pendant {

CameraPanel::CameraPanel(QWidget* parent) : QWidget(parent) {
  auto* group = new QGroupBox("Camera", this);
  auto* layout = new QVBoxLayout(group);

  image_label_ = new QLabel(this);
  image_label_->setAlignment(Qt::AlignCenter);
  image_label_->setMinimumSize(640, 480);
  image_label_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  image_label_->setStyleSheet("background-color: #2b2b2b; border: 1px solid #555;");
  image_label_->setText("No camera feed");

  layout->addWidget(image_label_, 1);

  auto* btn_layout = new QHBoxLayout();
  btn_toggle_ = new QPushButton("Switch to Depth", this);
  btn_toggle_->setMaximumWidth(160);
  connect(btn_toggle_, &QPushButton::clicked, this, &CameraPanel::onToggleRgbDepth);
  btn_layout->addWidget(btn_toggle_);
  btn_layout->addStretch();
  layout->addLayout(btn_layout);

  auto* outer = new QVBoxLayout(this);
  outer->setContentsMargins(0, 0, 0, 0);
  outer->addWidget(group);
}

void CameraPanel::onImageReceived(const QImage& image) {
  image_label_->setPixmap(QPixmap::fromImage(image));
}

void CameraPanel::onEstopChanged(bool active) {
  if (active) {
    image_label_->setText("E-STOP ACTIVATED");
    image_label_->setStyleSheet(
        "background-color: #cc0000; color: white; font-size: 24px; "
        "font-weight: bold; border: 2px solid #ff0000;");
  } else {
    image_label_->setText("No camera feed");
    image_label_->setStyleSheet(
        "background-color: #2b2b2b; border: 1px solid #555;");
  }
}

void CameraPanel::onToggleRgbDepth() {
  show_depth_ = !show_depth_;
  // TODO: implement depth display
}

}  // namespace teaching_pendant
```

---

## Task 4: RobotStateBar Panel

**Files:**
- Create: `include/teaching_pendant/panels/robot_state_bar.hpp`
- Create: `src/panels/robot_state_bar.cpp`

**Responsibilities extracted from MainWindow:**
- `createStateBar()` UI (lines 130-159)
- Pose/finger/TCP label updates (lines 449-453)

**Public API:**
```cpp
class RobotStateBar : public QWidget {
  Q_OBJECT
public:
  explicit RobotStateBar(QWidget* parent = nullptr);
public slots:
  void onStateUpdated(const std::array<double, 6>& pose,
                      double finger_width,
                      const std::string& tcp_name);
private:
  QLabel* label_pose_[6];
  QLabel* label_finger_;
  QLabel* label_tcp_;
};
```

- [ ] **Step 1: Write header**

```cpp
// include/teaching_pendant/panels/robot_state_bar.hpp
#pragma once

#include <QWidget>
#include <QLabel>
#include <array>
#include <string>

namespace teaching_pendant {

class RobotStateBar : public QWidget {
  Q_OBJECT
public:
  explicit RobotStateBar(QWidget* parent = nullptr);
public slots:
  void onStateUpdated(const std::array<double, 6>& pose,
                      double finger_width,
                      const std::string& tcp_name);
private:
  QLabel* label_pose_[6];
  QLabel* label_finger_;
  QLabel* label_tcp_;
};

}  // namespace teaching_pendant
```

- [ ] **Step 2: Write implementation**

```cpp
// src/panels/robot_state_bar.cpp
#include "teaching_pendant/panels/robot_state_bar.hpp"

#include <QGridLayout>
#include <QGroupBox>
#include <QFont>

namespace teaching_pendant {

RobotStateBar::RobotStateBar(QWidget* parent) : QWidget(parent) {
  auto* group = new QGroupBox("Robot State", this);
  auto* layout = new QGridLayout(group);
  layout->setSpacing(4);

  const char* pose_names[] = {"X:", "Y:", "Z:", "R:", "P:", "Yw:"};
  QFont mono("Monospace");
  for (int i = 0; i < 6; ++i) {
    layout->addWidget(new QLabel(pose_names[i]), 0, i * 2);
    label_pose_[i] = new QLabel("0.0000");
    label_pose_[i]->setMinimumWidth(80);
    label_pose_[i]->setFont(mono);
    layout->addWidget(label_pose_[i], 0, i * 2 + 1);
  }

  layout->addWidget(new QLabel("Finger:"), 0, 12);
  label_finger_ = new QLabel("0.0000");
  label_finger_->setFont(mono);
  label_finger_->setMinimumWidth(60);
  layout->addWidget(label_finger_, 0, 13);

  layout->addWidget(new QLabel("TCP:"), 0, 14);
  label_tcp_ = new QLabel("hand");
  layout->addWidget(label_tcp_, 0, 15);

  layout->setColumnStretch(16, 1);

  auto* outer = new QVBoxLayout(this);
  outer->setContentsMargins(0, 0, 0, 0);
  outer->addWidget(group);
}

void RobotStateBar::onStateUpdated(const std::array<double, 6>& pose,
                                   double finger_width,
                                   const std::string& tcp_name) {
  for (int i = 0; i < 6; ++i) {
    label_pose_[i]->setText(QString::number(pose[i], 'f', 4));
  }
  label_finger_->setText(QString::number(finger_width, 'f', 4));
  label_tcp_->setText(QString::fromStdString(tcp_name));
}

}  // namespace teaching_pendant
```

---

## Task 5: JointControlPanel

**Files:**
- Create: `include/teaching_pendant/panels/joint_control_panel.hpp`
- Create: `src/panels/joint_control_panel.cpp`

**Responsibilities extracted from MainWindow:**
- Joint slider UI creation (lines 166-205)
- All joint slider logic: `syncSliderToState`, `onJointSliderPressed/Released`, `onJointEditFinished` (lines 467-516)
- Joint follow timer + stream logic (lines 518-533)
- Unit conversion helpers (lines 537-551)
- Joint stream start trigger on first state (lines 456-460)
- `last_streamed_joints_`, `slider_is_controlled_`, `lock_timer_` state

**Public API:**
```cpp
class JointControlPanel : public QWidget {
  Q_OBJECT
public:
  explicit JointControlPanel(std::shared_ptr<PendantNode> node,
                             QWidget* parent = nullptr);
public slots:
  void onStateUpdated(const std::vector<double>& joint_angles);
  void onEstopChanged(bool active);
signals:
  /// Emitted when joint stream should start (after first successful state query)
  void jointStreamReady(const std::array<double, 7>& initial_joints);
};
```

- [ ] **Step 1: Write header**

```cpp
// include/teaching_pendant/panels/joint_control_panel.hpp
#pragma once

#include <QWidget>
#include <QSlider>
#include <QLineEdit>
#include <QLabel>
#include <QTimer>
#include <array>
#include <atomic>
#include <memory>

namespace teaching_pendant {

class PendantNode;

class JointControlPanel : public QWidget {
  Q_OBJECT
public:
  explicit JointControlPanel(std::shared_ptr<PendantNode> node,
                             QWidget* parent = nullptr);

public slots:
  /// Called with fresh joint angles from robot state
  void onStateUpdated(const std::vector<double>& joint_angles);
  /// Called when E-STOP state changes
  void onEstopChanged(bool active);

signals:
  /// Emitted once after first successful state query to start joint streaming
  void jointStreamReady(const std::array<double, 7>& initial_joints);

private:
  void setupJointControls();
  void syncSliderToState(int joint, double rad);
  void onJointSliderPressed(int joint);
  void onJointSliderReleased(int joint);
  void onJointEditFinished(int joint);
  void onJointFollowTick();

  static double radToDeg(double rad);
  static double degToRad(double deg);
  static int degToSlider(double deg);
  static double sliderToDeg(int val);

  std::shared_ptr<PendantNode> node_;

  QSlider* slider_joint_[7];
  QLineEdit* edit_joint_[7];
  QTimer* joint_follow_timer_;
  QTimer* lock_timer_[7];
  bool slider_is_controlled_[7] = {};
  std::array<double, 7> last_streamed_joints_{};
  bool joint_stream_started_ = false;

  std::atomic<bool> estop_active_{false};
};

}  // namespace teaching_pendant
```

- [ ] **Step 2: Write implementation**

```cpp
// src/panels/joint_control_panel.cpp
#include "teaching_pendant/panels/joint_control_panel.hpp"
#include "teaching_pendant/pendant_node.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QFont>
#include <cmath>

namespace {

constexpr double kJointLowerDeg[7] = {
    -166.0, -101.0, -166.0, -176.0, -166.0, -1.0, -166.0};
constexpr double kJointUpperDeg[7] = {
    166.0, 101.0, 166.0, -4.0, 166.0, 215.0, 166.0};

}  // namespace

namespace teaching_pendant {

JointControlPanel::JointControlPanel(std::shared_ptr<PendantNode> node,
                                     QWidget* parent)
    : QWidget(parent), node_(std::move(node)) {
  auto* group = new QGroupBox("Joint Control", this);
  auto* layout = new QGridLayout(group);
  layout->setColumnStretch(1, 1);

  for (int i = 0; i < 7; ++i) {
    layout->addWidget(new QLabel(QString("J%1:").arg(i + 1)), i, 0);

    slider_joint_[i] = new QSlider(Qt::Horizontal, this);
    slider_joint_[i]->setRange(
        degToSlider(kJointLowerDeg[i]),
        degToSlider(kJointUpperDeg[i]));
    slider_joint_[i]->setValue(0);
    slider_joint_[i]->setTracking(true);
    layout->addWidget(slider_joint_[i], i, 1);

    edit_joint_[i] = new QLineEdit("0.0°", this);
    edit_joint_[i]->setFixedWidth(70);
    edit_joint_[i]->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    QFont mono("Monospace");
    edit_joint_[i]->setFont(mono);
    layout->addWidget(edit_joint_[i], i, 2);

    lock_timer_[i] = new QTimer(this);
    lock_timer_[i]->setSingleShot(true);
    lock_timer_[i]->setInterval(2000);

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

  auto* outer = new QVBoxLayout(this);
  outer->setContentsMargins(0, 0, 0, 0);
  outer->addWidget(group);

  // Joint follow timer (50Hz) — started after first state refresh
  joint_follow_timer_ = new QTimer(this);
  connect(joint_follow_timer_, &QTimer::timeout, this,
          &JointControlPanel::onJointFollowTick);
}

void JointControlPanel::onStateUpdated(const std::vector<double>& joint_angles) {
  for (int i = 0; i < 7 && i < static_cast<int>(joint_angles.size()); ++i) {
    if (!slider_is_controlled_[i]) {
      syncSliderToState(i, joint_angles[i]);
      last_streamed_joints_[i] = joint_angles[i];
    }
  }

  // Start joint stream on first successful state query
  if (!joint_stream_started_ && !joint_angles.empty()) {
    joint_stream_started_ = true;
    emit jointStreamReady(last_streamed_joints_);
    joint_follow_timer_->start(20);  // 50Hz
  }
}

void JointControlPanel::onEstopChanged(bool active) {
  estop_active_ = active;
}

void JointControlPanel::syncSliderToState(int joint, double rad) {
  if (slider_is_controlled_[joint]) return;
  double deg = radToDeg(rad);
  slider_joint_[joint]->blockSignals(true);
  slider_joint_[joint]->setValue(degToSlider(deg));
  slider_joint_[joint]->blockSignals(false);
  edit_joint_[joint]->setText(
      QString::number(deg, 'f', 1) + QString::fromUtf8("\u00B0"));
}

void JointControlPanel::onJointSliderPressed(int joint) {
  slider_is_controlled_[joint] = true;
  lock_timer_[joint]->stop();
}

void JointControlPanel::onJointSliderReleased(int joint) {
  lock_timer_[joint]->start();
}

void JointControlPanel::onJointEditFinished(int joint) {
  if (estop_active_.load()) return;

  QString text = edit_joint_[joint]->text();
  text.remove(QString::fromUtf8("\u00B0"));
  bool ok = false;
  double deg = text.toDouble(&ok);
  if (!ok) {
    deg = sliderToDeg(slider_joint_[joint]->value());
    edit_joint_[joint]->setText(
        QString::number(deg, 'f', 1) + QString::fromUtf8("\u00B0"));
    return;
  }

  deg = std::max(kJointLowerDeg[joint], std::min(kJointUpperDeg[joint], deg));

  slider_joint_[joint]->blockSignals(true);
  slider_joint_[joint]->setValue(degToSlider(deg));
  slider_joint_[joint]->blockSignals(false);

  edit_joint_[joint]->setText(
      QString::number(deg, 'f', 1) + QString::fromUtf8("\u00B0"));

  std::array<double, 7> target{};
  for (int i = 0; i < 7; ++i) {
    target[i] = degToRad(sliderToDeg(slider_joint_[i]->value()));
  }
  node_->update_joint_target(target);
  last_streamed_joints_ = target;

  slider_is_controlled_[joint] = true;
  lock_timer_[joint]->start();
}

void JointControlPanel::onJointFollowTick() {
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

double JointControlPanel::radToDeg(double rad) {
  return rad * 180.0 / M_PI;
}

double JointControlPanel::degToRad(double deg) {
  return deg * M_PI / 180.0;
}

int JointControlPanel::degToSlider(double deg) {
  return static_cast<int>(std::round(deg * 100.0));
}

double JointControlPanel::sliderToDeg(int val) {
  return val / 100.0;
}

}  // namespace teaching_pendant
```

---

## Task 6: CartesianPanel

**Files:**
- Create: `include/teaching_pendant/panels/cartesian_panel.hpp`
- Create: `src/panels/cartesian_panel.cpp`

**Responsibilities extracted from MainWindow:**
- Cartesian UI (lines 207-279): mode combo, XYZ/RPY spin boxes, Move button, Jog buttons
- `onMovePose()` (lines 388-396)
- Jog press/release (lines 408-416)
- `createJogButton()` helper (lines 342-355)

**Public API:**
```cpp
class CartesianPanel : public QWidget {
  Q_OBJECT
public:
  explicit CartesianPanel(std::shared_ptr<PendantNode> node,
                          QWidget* parent = nullptr);
public slots:
  void onEstopChanged(bool active);
};
```

- [ ] **Step 1: Write header**

```cpp
// include/teaching_pendant/panels/cartesian_panel.hpp
#pragma once

#include <QWidget>
#include <QDoubleSpinBox>
#include <QComboBox>
#include <QPushButton>
#include <atomic>
#include <memory>

namespace teaching_pendant {

class PendantNode;

class CartesianPanel : public QWidget {
  Q_OBJECT
public:
  explicit CartesianPanel(std::shared_ptr<PendantNode> node,
                          QWidget* parent = nullptr);

public slots:
  void onEstopChanged(bool active);

private slots:
  void onMovePose();
  void onJogPress(int axis);
  void onJogRelease();

private:
  QPushButton* createJogButton(const QString& text, int axis);

  std::shared_ptr<PendantNode> node_;

  QDoubleSpinBox* spin_xyz_[3];
  QDoubleSpinBox* spin_rpy_[3];
  QComboBox* combo_motion_mode_;

  std::atomic<bool> estop_active_{false};
};

}  // namespace teaching_pendant
```

- [ ] **Step 2: Write implementation**

```cpp
// src/panels/cartesian_panel.cpp
#include "teaching_pendant/panels/cartesian_panel.hpp"
#include "teaching_pendant/pendant_node.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QLabel>
#include <array>

namespace teaching_pendant {

CartesianPanel::CartesianPanel(std::shared_ptr<PendantNode> node,
                               QWidget* parent)
    : QWidget(parent), node_(std::move(node)) {
  auto* group = new QGroupBox("Cartesian Control", this);
  auto* layout = new QVBoxLayout(group);

  // Motion mode
  auto* mode_layout = new QHBoxLayout();
  mode_layout->addWidget(new QLabel("Mode:"));
  combo_motion_mode_ = new QComboBox(this);
  combo_motion_mode_->addItem("moveJ");
  combo_motion_mode_->addItem("moveL");
  combo_motion_mode_->setCurrentIndex(0);
  mode_layout->addWidget(combo_motion_mode_);
  mode_layout->addStretch();
  layout->addLayout(mode_layout);

  // XYZ input
  auto* xyz_grid = new QGridLayout();
  const char* xyz_names[] = {"X:", "Y:", "Z:"};
  for (int i = 0; i < 3; ++i) {
    xyz_grid->addWidget(new QLabel(xyz_names[i]), i, 0);
    spin_xyz_[i] = new QDoubleSpinBox(this);
    spin_xyz_[i]->setRange(-2.0, 2.0);
    spin_xyz_[i]->setDecimals(4);
    spin_xyz_[i]->setSingleStep(0.01);
    spin_xyz_[i]->setMinimumWidth(80);
    xyz_grid->addWidget(spin_xyz_[i], i, 1);
  }
  layout->addLayout(xyz_grid);

  // RPY input
  auto* rpy_grid = new QGridLayout();
  const char* rpy_names[] = {"R:", "P:", "Yw:"};
  for (int i = 0; i < 3; ++i) {
    rpy_grid->addWidget(new QLabel(rpy_names[i]), i, 0);
    spin_rpy_[i] = new QDoubleSpinBox(this);
    spin_rpy_[i]->setRange(-3.14159, 3.14159);
    spin_rpy_[i]->setDecimals(3);
    spin_rpy_[i]->setSingleStep(0.01);
    spin_rpy_[i]->setMinimumWidth(80);
    rpy_grid->addWidget(spin_rpy_[i], i, 1);
  }
  layout->addLayout(rpy_grid);

  // Move button
  auto* btn_move_pose = new QPushButton("Move to Pose", this);
  btn_move_pose->setStyleSheet("padding: 6px; font-weight: bold;");
  connect(btn_move_pose, &QPushButton::clicked, this,
          &CartesianPanel::onMovePose);
  layout->addWidget(btn_move_pose);

  // Jog buttons
  auto* jog_group = new QGroupBox("Jog", this);
  auto* jog_layout = new QGridLayout(jog_group);
  jog_layout->setSpacing(2);

  struct JogBtnDef {
    const char* text;
    int axis;
  };
  JogBtnDef jog_btns[] = {
      {"+X", 0}, {"-X", 1}, {"+Y", 2}, {"-Y", 3},
      {"+Z", 4}, {"-Z", 5}, {"+R", 6}, {"-R", 7},
      {"+P", 8}, {"-P", 9}, {"+Yw", 10}, {"-Yw", 11},
  };

  for (int i = 0; i < 12; ++i) {
    auto* btn = createJogButton(jog_btns[i].text, jog_btns[i].axis);
    int row = i / 4;
    int col = i % 4;
    jog_layout->addWidget(btn, row, col);
  }

  layout->addWidget(jog_group);

  auto* outer = new QVBoxLayout(this);
  outer->setContentsMargins(0, 0, 0, 0);
  outer->addWidget(group);
}

void CartesianPanel::onEstopChanged(bool active) {
  estop_active_ = active;
}

void CartesianPanel::onMovePose() {
  if (estop_active_.load()) return;
  std::array<double, 3> xyz{
      spin_xyz_[0]->value(), spin_xyz_[1]->value(), spin_xyz_[2]->value()};
  std::array<double, 3> rpy{
      spin_rpy_[0]->value(), spin_rpy_[1]->value(), spin_rpy_[2]->value()};
  uint8_t mode = static_cast<uint8_t>(combo_motion_mode_->currentIndex());
  node_->async_move_pose(xyz, rpy, mode);
}

void CartesianPanel::onJogPress(int axis) {
  if (estop_active_.load()) return;
  uint8_t mode = static_cast<uint8_t>(combo_motion_mode_->currentIndex());
  node_->start_jog(axis, 1.0, mode);
}

void CartesianPanel::onJogRelease() {
  node_->stop_jog();
}

QPushButton* CartesianPanel::createJogButton(const QString& text, int axis) {
  auto* btn = new QPushButton(text, this);
  btn->setFixedSize(50, 35);
  btn->setAutoRepeat(false);

  connect(btn, &QPushButton::pressed, this,
          [this, axis]() { onJogPress(axis); });
  connect(btn, &QPushButton::released, this,
          [this]() { onJogRelease(); });

  return btn;
}

}  // namespace teaching_pendant
```

---

## Task 7: FunctionPanel

**Files:**
- Create: `include/teaching_pendant/panels/function_panel.hpp`
- Create: `src/panels/function_panel.cpp`

**Responsibilities extracted from MainWindow:**
- Speed slider UI + callbacks (lines 281-307, 398-406)
- Gripper buttons (lines 311-319, 364-372)
- Go Home button (lines 323-326, 374-377)
- E-STOP button + state (lines 328-333, 379-386)

**Public API:**
```cpp
class FunctionPanel : public QWidget {
  Q_OBJECT
public:
  explicit FunctionPanel(std::shared_ptr<PendantNode> node,
                         QWidget* parent = nullptr);
signals:
  /// Emitted when E-STOP is activated or deactivated
  void estopChanged(bool active);
};
```

- [ ] **Step 1: Write header**

```cpp
// include/teaching_pendant/panels/function_panel.hpp
#pragma once

#include <QWidget>
#include <QSlider>
#include <QLabel>
#include <QPushButton>
#include <atomic>
#include <memory>

namespace teaching_pendant {

class PendantNode;

class FunctionPanel : public QWidget {
  Q_OBJECT
public:
  explicit FunctionPanel(std::shared_ptr<PendantNode> node,
                         QWidget* parent = nullptr);

signals:
  void estopChanged(bool active);

private slots:
  void onSetSpeedJ(int value);
  void onSetSpeedL(int value);
  void onOpenGripper();
  void onCloseGripper();
  void onGoHome();
  void onEmergencyStop();

private:
  std::shared_ptr<PendantNode> node_;

  QSlider* slider_speed_j_;
  QSlider* slider_speed_l_;
  QLabel* label_speed_j_;
  QLabel* label_speed_l_;

  std::atomic<bool> estop_active_{false};
};

}  // namespace teaching_pendant
```

- [ ] **Step 2: Write implementation**

```cpp
// src/panels/function_panel.cpp
#include "teaching_pendant/panels/function_panel.hpp"
#include "teaching_pendant/pendant_node.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QLabel>

namespace teaching_pendant {

FunctionPanel::FunctionPanel(std::shared_ptr<PendantNode> node,
                             QWidget* parent)
    : QWidget(parent), node_(std::move(node)) {
  auto* group = new QGroupBox("Functions", this);
  auto* layout = new QVBoxLayout(group);

  layout->addWidget(new QLabel("moveJ Speed:"));
  auto* speed_j_layout = new QHBoxLayout();
  slider_speed_j_ = new QSlider(Qt::Horizontal, this);
  slider_speed_j_->setRange(1, 100);
  slider_speed_j_->setValue(50);
  label_speed_j_ = new QLabel("50%", this);
  label_speed_j_->setMinimumWidth(40);
  connect(slider_speed_j_, &QSlider::valueChanged, this,
          &FunctionPanel::onSetSpeedJ);
  speed_j_layout->addWidget(slider_speed_j_);
  speed_j_layout->addWidget(label_speed_j_);
  layout->addLayout(speed_j_layout);

  layout->addWidget(new QLabel("moveL Speed:"));
  auto* speed_l_layout = new QHBoxLayout();
  slider_speed_l_ = new QSlider(Qt::Horizontal, this);
  slider_speed_l_->setRange(1, 100);
  slider_speed_l_->setValue(50);
  label_speed_l_ = new QLabel("50%", this);
  label_speed_l_->setMinimumWidth(40);
  connect(slider_speed_l_, &QSlider::valueChanged, this,
          &FunctionPanel::onSetSpeedL);
  speed_l_layout->addWidget(slider_speed_l_);
  speed_l_layout->addWidget(label_speed_l_);
  layout->addLayout(speed_l_layout);

  layout->addSpacing(10);

  auto* btn_open = new QPushButton("Open Gripper", this);
  btn_open->setStyleSheet("padding: 6px;");
  connect(btn_open, &QPushButton::clicked, this,
          &FunctionPanel::onOpenGripper);
  layout->addWidget(btn_open);

  auto* btn_close = new QPushButton("Close Gripper", this);
  btn_close->setStyleSheet("padding: 6px;");
  connect(btn_close, &QPushButton::clicked, this,
          &FunctionPanel::onCloseGripper);
  layout->addWidget(btn_close);

  layout->addSpacing(10);

  auto* btn_home = new QPushButton("Go Home", this);
  btn_home->setStyleSheet("padding: 6px; font-weight: bold;");
  connect(btn_home, &QPushButton::clicked, this,
          &FunctionPanel::onGoHome);
  layout->addWidget(btn_home);

  auto* btn_estop = new QPushButton("E-STOP", this);
  btn_estop->setStyleSheet(
      "padding: 10px; font-weight: bold; font-size: 14px; "
      "background-color: #cc0000; color: white; border-radius: 4px;");
  connect(btn_estop, &QPushButton::clicked, this,
          &FunctionPanel::onEmergencyStop);
  layout->addWidget(btn_estop);

  layout->addStretch();

  auto* outer = new QVBoxLayout(this);
  outer->setContentsMargins(0, 0, 0, 0);
  outer->addWidget(group);
}

void FunctionPanel::onSetSpeedJ(int value) {
  label_speed_j_->setText(QString("%1%").arg(value));
  node_->async_set_speed(0, static_cast<double>(value));
}

void FunctionPanel::onSetSpeedL(int value) {
  label_speed_l_->setText(QString("%1%").arg(value));
  node_->async_set_speed(1, static_cast<double>(value));
}

void FunctionPanel::onOpenGripper() {
  if (estop_active_.load()) return;
  node_->async_open_gripper();
}

void FunctionPanel::onCloseGripper() {
  if (estop_active_.load()) return;
  node_->async_close_gripper();
}

void FunctionPanel::onGoHome() {
  if (estop_active_.load()) return;
  node_->async_go_home();
}

void FunctionPanel::onEmergencyStop() {
  estop_active_ = true;
  node_->emergency_stop();
  emit estopChanged(true);
}

}  // namespace teaching_pendant
```

---

## Task 8: Refactor MainWindow to Thin Orchestrator

**Files:**
- Modify: `include/teaching_pendant/main_window.hpp`
- Modify: `src/main_window.cpp`

MainWindow becomes a thin orchestrator:
1. Creates all 6 panel widgets
2. Sets up PendantNode callbacks to relay data to panels
3. Wires E-STOP signal from FunctionPanel to other panels
4. Runs state refresh timer and dispatches data to panels

**New MainWindow responsibilities:**
- Own PendantNode reference
- Own state refresh QTimer (5Hz)
- Relay connection callbacks → ConnectionBar
- Relay image callbacks → CameraPanel (with cv::Mat→QImage conversion)
- Relay state query results → RobotStateBar + JointControlPanel
- Relay E-STOP signal → all panels

- [ ] **Step 1: Write new header**

```cpp
// include/teaching_pendant/main_window.hpp
#pragma once

#include <QMainWindow>
#include <QTimer>
#include <memory>

namespace teaching_pendant {

class PendantNode;
class ConnectionBar;
class CameraPanel;
class RobotStateBar;
class JointControlPanel;
class CartesianPanel;
class FunctionPanel;

class MainWindow : public QMainWindow {
  Q_OBJECT
public:
  explicit MainWindow(std::shared_ptr<PendantNode> node,
                      QWidget* parent = nullptr);

private slots:
  void onRefreshState();

private:
  void setupUi();

  std::shared_ptr<PendantNode> node_;

  // Panels
  ConnectionBar* connection_bar_;
  CameraPanel* camera_panel_;
  RobotStateBar* state_bar_;
  JointControlPanel* joint_panel_;
  CartesianPanel* cartesian_panel_;
  FunctionPanel* function_panel_;

  QTimer* refresh_timer_;
};

}  // namespace teaching_pendant
```

- [ ] **Step 2: Write new implementation**

```cpp
// src/main_window.cpp
#include "teaching_pendant/main_window.hpp"
#include "teaching_pendant/pendant_node.hpp"

#include "teaching_pendant/panels/connection_bar.hpp"
#include "teaching_pendant/panels/camera_panel.hpp"
#include "teaching_pendant/panels/robot_state_bar.hpp"
#include "teaching_pendant/panels/joint_control_panel.hpp"
#include "teaching_pendant/panels/cartesian_panel.hpp"
#include "teaching_pendant/panels/function_panel.hpp"

#include <QVBoxLayout>
#include <QImage>
#include <QPixmap>

namespace teaching_pendant {

MainWindow::MainWindow(std::shared_ptr<PendantNode> node, QWidget* parent)
    : QMainWindow(parent), node_(std::move(node)) {
  setWindowTitle("Robot Teaching Pendant");
  resize(1100, 800);

  setupUi();

  // --- PendantNode callbacks ---

  // Connection state → ConnectionBar
  node_->set_connection_callback(
      [this](bool /*robot*/, bool camera) {
        QMetaObject::invokeMethod(this, [this, camera]() {
          connection_bar_->onCameraConnectionChanged(camera);
        });
      });

  // Image → CameraPanel (cv::Mat → QImage conversion here)
  node_->set_image_callback(
      [this](const cv::Mat& rgb) {
        cv::Mat copy = rgb.clone();
        QMetaObject::invokeMethod(this, [this, copy]() {
          QImage qimg(copy.data, copy.cols, copy.rows,
                      static_cast<int>(copy.step), QImage::Format_RGB888);
          camera_panel_->onImageReceived(qimg);
        });
      });

  // --- E-STOP signal relay ---
  connect(function_panel_, &FunctionPanel::estopChanged,
          joint_panel_, &JointControlPanel::onEstopChanged);
  connect(function_panel_, &FunctionPanel::estopChanged,
          cartesian_panel_, &CartesianPanel::onEstopChanged);
  connect(function_panel_, &FunctionPanel::estopChanged,
          camera_panel_, &CameraPanel::onEstopChanged);

  // --- Joint stream start ---
  connect(joint_panel_, &JointControlPanel::jointStreamReady,
          node_.get(), [this](const std::array<double, 7>& initial) {
            node_->start_joint_stream(initial);
          });

  // --- State refresh timer (5Hz) ---
  refresh_timer_ = new QTimer(this);
  connect(refresh_timer_, &QTimer::timeout, this, &MainWindow::onRefreshState);
  refresh_timer_->start(200);
}

void MainWindow::setupUi() {
  auto* central = new QWidget(this);
  auto* main_layout = new QVBoxLayout(central);
  main_layout->setSpacing(6);
  main_layout->setContentsMargins(8, 8, 8, 8);

  connection_bar_ = new ConnectionBar(this);
  main_layout->addWidget(connection_bar_);

  camera_panel_ = new CameraPanel(this);
  main_layout->addWidget(camera_panel_, 3);

  state_bar_ = new RobotStateBar(this);
  main_layout->addWidget(state_bar_);

  // Control panel: Joint | Cartesian | Function
  auto* control = new QWidget(this);
  auto* h_layout = new QHBoxLayout(control);
  h_layout->setSpacing(8);

  joint_panel_ = new JointControlPanel(node_, this);
  cartesian_panel_ = new CartesianPanel(node_, this);
  function_panel_ = new FunctionPanel(node_, this);

  h_layout->addWidget(joint_panel_, 2);
  h_layout->addWidget(cartesian_panel_, 2);
  h_layout->addWidget(function_panel_, 1);

  main_layout->addWidget(control, 2);
  setCentralWidget(central);
}

void MainWindow::onRefreshState() {
  // Update connection status
  bool robot_topic = node_->is_robot_connected();
  bool services = node_->are_services_ready();
  connection_bar_->onRobotConnectionChanged(robot_topic, services);

  if (!services) return;

  node_->async_get_state(
      [this](bool success,
             const std::vector<double>& joints,
             const std::array<double, 6>& pose,
             double finger,
             const std::string& tcp) {
        if (!success) return;
        QMetaObject::invokeMethod(this, [this, joints, pose, finger, tcp]() {
          state_bar_->onStateUpdated(pose, finger, tcp);
          joint_panel_->onStateUpdated(joints);
        });
      });
}

}  // namespace teaching_pendant
```

---

## Task 9: Update CMakeLists.txt

**Files:**
- Modify: `CMakeLists.txt`

Add all new source files and include paths. Ensure AUTOMOC scans the panels/ subdirectory.

- [ ] **Step 1: Update CMakeLists.txt**

Replace the `add_executable` block and ensure all new files are listed:

```cmake
# In CMakeLists.txt, replace the add_executable section with:

add_executable(teaching_pendant
  src/main.cpp
  src/pendant_node.cpp
  src/main_window.cpp
  # Panels
  src/panels/connection_bar.cpp
  src/panels/camera_panel.cpp
  src/panels/robot_state_bar.cpp
  src/panels/joint_control_panel.cpp
  src/panels/cartesian_panel.cpp
  src/panels/function_panel.cpp
  # Headers (for AUTOMOC)
  include/teaching_pendant/main_window.hpp
  include/teaching_pendant/panels/connection_bar.hpp
  include/teaching_pendant/panels/camera_panel.hpp
  include/teaching_pendant/panels/robot_state_bar.hpp
  include/teaching_pendant/panels/joint_control_panel.hpp
  include/teaching_pendant/panels/cartesian_panel.hpp
  include/teaching_pendant/panels/function_panel.hpp
)
```

---

## Task 10: Build Verification

- [ ] **Step 1: Build**

```bash
cd /home/cll/workspace/isaac_ros_project
source install/setup.zsh
colcon build --base-paths src --packages-select teaching_pendant
```

Expected: Clean build with no errors.

- [ ] **Step 2: Fix any compilation errors**

If AUTOMOC issues occur, verify:
1. All headers with `Q_OBJECT` are listed in `add_executable`
2. `AUTOMOC_PATH_PREFIX ON` is set
3. Include paths are correct

- [ ] **Step 3: Commit**

```bash
git add -A src/teaching_pendant/
git commit -m "refactor(pendant): split MainWindow into 6 independent Panel widgets

- ConnectionBar: robot/camera status display
- CameraPanel: camera feed + RGB/Depth toggle
- RobotStateBar: pose/finger/TCP display
- JointControlPanel: joint slider + edit + streaming
- CartesianPanel: XYZ/RPY + Jog + motion mode
- FunctionPanel: speed/gripper/home/E-STOP
- MainWindow reduced to thin orchestrator
- Cross-panel E-STOP via Qt signals"
```
