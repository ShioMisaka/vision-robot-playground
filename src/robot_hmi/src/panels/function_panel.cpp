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
