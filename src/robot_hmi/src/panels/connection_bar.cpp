#include "robot_hmi/panels/connection_bar.hpp"

#include <QHBoxLayout>
#include <QFrame>

namespace robot_hmi {

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

}  // namespace robot_hmi
