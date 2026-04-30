#include "robot_hmi/panels/joint_control_panel.hpp"
#include "robot_hmi/pendant_node.hpp"

#include <QVBoxLayout>
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

namespace robot_hmi {

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

    edit_joint_[i] = new QLineEdit("0.0\u00B0", this);
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

  joint_follow_timer_ = new QTimer(this);
  connect(joint_follow_timer_, &QTimer::timeout, this,
          &JointControlPanel::onJointFollowTick);
}

void JointControlPanel::onStateUpdated(const std::vector<double>& joint_angles) {
  // Sync slider DISPLAY to actual state (visual feedback only)
  // Do NOT update command_target_ — the robot must hold the last commanded position
  for (int i = 0; i < 7 && i < static_cast<int>(joint_angles.size()); ++i) {
    if (!slider_is_controlled_[i]) {
      syncSliderToState(i, joint_angles[i]);
    }
  }

  // Start joint stream on first state update
  if (!joint_stream_started_ && !joint_angles.empty()) {
    joint_stream_started_ = true;
    // Initialize command_target_ from actual state
    for (int i = 0; i < 7; ++i) {
      command_target_[i] = joint_angles[i];
    }
    emit jointStreamReady(command_target_);
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

  // Update command target for this joint
  command_target_[joint] = degToRad(deg);
  node_->update_joint_target(command_target_);

  slider_is_controlled_[joint] = true;
  lock_timer_[joint]->start();
}

void JointControlPanel::onJointFollowTick() {
  if (estop_active_.load()) return;

  // After jog stops, resync command_target_ to actual state (via slider display)
  if (resync_after_jog_) {
    resync_after_jog_ = false;
    for (int i = 0; i < 7; ++i) {
      command_target_[i] = degToRad(sliderToDeg(slider_joint_[i]->value()));
    }
    // Don't publish — just sync internal state.
    // The controller holds position on its own after jog stops.
    return;
  }

  // For controlled joints: read target from slider (user input)
  // For non-controlled joints: sync to actual position via slider display
  // so command_target_ is always up-to-date as a starting point for future
  // interactions (e.g. after a script trajectory completes).
  bool any_controlled = false;
  for (int i = 0; i < 7; ++i) {
    if (slider_is_controlled_[i]) {
      any_controlled = true;
      double deg = sliderToDeg(slider_joint_[i]->value());
      command_target_[i] = degToRad(deg);
      edit_joint_[i]->setText(
          QString::number(deg, 'f', 1) + QString::fromUtf8("\u00B0"));
    } else {
      command_target_[i] = degToRad(sliderToDeg(slider_joint_[i]->value()));
    }
  }

  // Only send to controller when user is actively controlling sliders.
  // The controller holds position on its own — continuous streaming is
  // unnecessary and would permanently claim motion_owner_ as kPendant.
  if (any_controlled) {
    node_->update_joint_target(command_target_);
  }
}

void JointControlPanel::notifyJogStopped() {
  resync_after_jog_ = true;
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

}  // namespace robot_hmi
