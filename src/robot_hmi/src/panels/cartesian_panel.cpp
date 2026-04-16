#include "robot_hmi/panels/cartesian_panel.hpp"
#include "robot_hmi/pendant_node.hpp"
#include <robot_msgs/msg/jog_command.hpp>

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QLabel>
#include <array>

namespace robot_hmi {

CartesianPanel::CartesianPanel(std::shared_ptr<PendantNode> node,
                               QWidget* parent)
    : QWidget(parent), node_(std::move(node)) {
  auto* group = new QGroupBox("Cartesian Control", this);
  auto* layout = new QVBoxLayout(group);

  // Motion mode + Frame selector
  auto* mode_layout = new QHBoxLayout();
  mode_layout->addWidget(new QLabel("Mode:"));
  combo_motion_mode_ = new QComboBox(this);
  combo_motion_mode_->addItem("moveJ");
  combo_motion_mode_->addItem("moveL");
  combo_motion_mode_->setCurrentIndex(0);
  mode_layout->addWidget(combo_motion_mode_);

  mode_layout->addSpacing(12);
  mode_layout->addWidget(new QLabel("Frame:"));
  combo_frame_ = new QComboBox(this);
  combo_frame_->addItem("Base");
  combo_frame_->addItem("TCP");
  combo_frame_->setCurrentIndex(0);
  mode_layout->addWidget(combo_frame_);
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

  // Jog buttons — two columns: XYZ (left) | RPY (right), +/- side by side
  auto* jog_group = new QGroupBox("Jog", this);
  auto* jog_layout = new QGridLayout(jog_group);
  jog_layout->setSpacing(2);

  // XYZ column (left)
  jog_layout->addWidget(new QLabel("XYZ"), 0, 0, 1, 2, Qt::AlignCenter);
  struct JogBtnDef {
    const char* text;
    int axis;
  };
  JogBtnDef xyz_btns[] = {{"+X", 0}, {"-X", 1}, {"+Y", 2}, {"-Y", 3}, {"+Z", 4}, {"-Z", 5}};
  for (int i = 0; i < 6; ++i) {
    int row = 1 + i / 2;
    int col = i % 2;
    jog_layout->addWidget(createJogButton(xyz_btns[i].text, xyz_btns[i].axis), row, col);
  }

  // RPY column (right), offset by 2 columns + spacing
  jog_layout->setColumnMinimumWidth(2, 8);  // spacer
  jog_layout->addWidget(new QLabel("RPY"), 0, 3, 1, 2, Qt::AlignCenter);
  JogBtnDef rpy_btns[] = {{"+R", 6}, {"-R", 7}, {"+P", 8}, {"-P", 9}, {"+Yw", 10}, {"-Yw", 11}};
  for (int i = 0; i < 6; ++i) {
    int row = 1 + i / 2;
    int col = 3 + i % 2;
    jog_layout->addWidget(createJogButton(rpy_btns[i].text, rpy_btns[i].axis), row, col);
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
  // combo: index 0="Base", 1="TCP"
  // JogCommand: TCP_FRAME=0, BASE_FRAME=1
  // So Base→1, TCP→0
  uint8_t frame = (combo_frame_->currentIndex() == 0)
                      ? robot_msgs::msg::JogCommand::BASE_FRAME
                      : robot_msgs::msg::JogCommand::TCP_FRAME;
  node_->start_jog(axis, 0, frame);
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

}  // namespace robot_hmi
