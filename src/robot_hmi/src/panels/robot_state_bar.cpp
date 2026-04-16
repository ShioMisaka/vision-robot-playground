#include "robot_hmi/panels/robot_state_bar.hpp"

#include <QGridLayout>
#include <QVBoxLayout>
#include <QGroupBox>
#include <QFont>

namespace robot_hmi {

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

}  // namespace robot_hmi
