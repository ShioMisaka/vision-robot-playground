#include "robot_hmi/panels/function_panel.hpp"
#include "robot_hmi/pendant_node.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QLabel>

namespace robot_hmi {

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

  btn_estop_ = new QPushButton("E-STOP", this);
  btn_estop_->setStyleSheet(
      "padding: 10px; font-weight: bold; font-size: 14px; "
      "background-color: #cc0000; color: white; border-radius: 4px;");
  connect(btn_estop_, &QPushButton::clicked, this,
          &FunctionPanel::onEmergencyStop);
  layout->addWidget(btn_estop_);

  layout->addStretch();

  auto* outer = new QVBoxLayout(this);
  outer->setContentsMargins(0, 0, 0, 0);
  outer->addWidget(group);

  // 定时检查故障状态，更新 E-STOP 按钮外观
  fault_check_timer_ = new QTimer(this);
  connect(fault_check_timer_, &QTimer::timeout,
          this, &FunctionPanel::onCheckFaultState);
  fault_check_timer_->start(200);  // 5Hz
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
  if (node_->is_robot_fault()) {
    // 故障状态：请求清除故障，但不立即释放 UI 锁定。
    // onCheckFaultState 定时器会在故障实际清除后自动解锁。
    node_->clear_fault();
  } else {
    // 正常状态：急停
    estop_active_ = true;
    node_->emergency_stop();
    update_estop_button(true);
    emit estopChanged(true);
  }
}

void FunctionPanel::onCheckFaultState() {
  bool fault = node_->is_robot_fault();
  update_estop_button(fault);

  // 故障清除后自动解除 UI 锁定
  if (!fault && estop_active_.load()) {
    estop_active_ = false;
    emit estopChanged(false);
  }
}

void FunctionPanel::update_estop_button(bool fault) {
  if (fault) {
    btn_estop_->setText("CLEAR FAULT");
    btn_estop_->setStyleSheet(
        "padding: 10px; font-weight: bold; font-size: 14px; "
        "background-color: #cc8800; color: white; border-radius: 4px;");
  } else {
    btn_estop_->setText("E-STOP");
    btn_estop_->setStyleSheet(
        "padding: 10px; font-weight: bold; font-size: 14px; "
        "background-color: #cc0000; color: white; border-radius: 4px;");
  }
}

}  // namespace robot_hmi
