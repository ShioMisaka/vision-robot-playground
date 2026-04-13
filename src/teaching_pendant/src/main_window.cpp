#include "teaching_pendant/main_window.hpp"

#include <QPixmap>
#include <QSizePolicy>
#include <QFont>
#include <cmath>

namespace {

// Panda joint limits in degrees (from panda_profile.hpp, converted)
constexpr double kJointLowerDeg[7] = {
    -166.0, -101.0, -166.0, -176.0, -166.0, -1.0, -166.0};
constexpr double kJointUpperDeg[7] = {
    166.0, 101.0, 166.0, -4.0, 166.0, 215.0, 166.0};

}  // namespace

namespace teaching_pendant {

MainWindow::MainWindow(std::shared_ptr<PendantNode> node, QWidget* parent)
    : QMainWindow(parent), node_(std::move(node)) {
  setWindowTitle("Robot Teaching Pendant");
  resize(1100, 800);

  setupUi();

  // 设置连接状态回调（相机状态由回调驱动，机器人状态由 onRefreshState 综合判断）
  node_->set_connection_callback(
      [this](bool /*robot*/, bool camera) {
        QMetaObject::invokeMethod(this, [this, camera]() {
          label_camera_status_->setText(camera ? "Connected" : "Disconnected");
          label_camera_status_->setStyleSheet(
              camera ? "color: green; font-weight: bold;"
                     : "color: red; font-weight: bold;");
        });
      });

  // 设置图像回调（cv::Mat RGB 已在 ROS2 线程中转换好，Qt 线程只做 QImage 包装）
  node_->set_image_callback(
      [this](const cv::Mat& rgb) {
        // clone 避免数据竞争，轻量操作
        cv::Mat copy = rgb.clone();
        QMetaObject::invokeMethod(this, [this, copy]() {
          QImage qimg(copy.data, copy.cols, copy.rows,
                      static_cast<int>(copy.step), QImage::Format_RGB888);
          image_label_->setPixmap(QPixmap::fromImage(qimg));
        });
      });

  // 状态刷新定时器（异步查询，不阻塞 GUI）
  refresh_timer_ = new QTimer(this);
  connect(refresh_timer_, &QTimer::timeout, this, &MainWindow::onRefreshState);
  refresh_timer_->start(200);  // 5Hz 异步查询

  // 关节跟随定时器（50Hz）—— 等第一次状态刷新后再启动
  joint_follow_timer_ = new QTimer(this);
  connect(joint_follow_timer_, &QTimer::timeout, this, &MainWindow::onJointFollowTick);
  // 不在此启动，等 onRefreshState 首次成功后启动

  // 初始状态
  label_robot_status_->setText("Disconnected");
  label_robot_status_->setStyleSheet("color: red; font-weight: bold;");
  label_camera_status_->setText("Disconnected");
  label_camera_status_->setStyleSheet("color: red; font-weight: bold;");
}

// ===== UI Setup =====

void MainWindow::setupUi() {
  auto* central = new QWidget(this);
  auto* main_layout = new QVBoxLayout(central);
  main_layout->setSpacing(6);
  main_layout->setContentsMargins(8, 8, 8, 8);

  main_layout->addWidget(createConnectionBar());
  main_layout->addWidget(createCameraPanel(), 3);
  main_layout->addWidget(createStateBar());
  main_layout->addWidget(createControlPanel(), 2);

  setCentralWidget(central);
}

QWidget* MainWindow::createConnectionBar() {
  auto* bar = new QFrame(this);
  bar->setFrameShape(QFrame::StyledPanel);
  auto* layout = new QHBoxLayout(bar);
  layout->setContentsMargins(8, 4, 8, 4);

  layout->addWidget(new QLabel("Robot:"));
  label_robot_status_ = new QLabel("Disconnected");
  label_robot_status_->setMinimumWidth(100);
  layout->addWidget(label_robot_status_);

  layout->addSpacing(20);

  layout->addWidget(new QLabel("Camera:"));
  label_camera_status_ = new QLabel("Disconnected");
  label_camera_status_->setMinimumWidth(100);
  layout->addWidget(label_camera_status_);

  layout->addStretch();

  return bar;
}

QWidget* MainWindow::createCameraPanel() {
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
  auto* btn_toggle = new QPushButton("Switch to Depth", this);
  btn_toggle->setMaximumWidth(160);
  connect(btn_toggle, &QPushButton::clicked, this, &MainWindow::onToggleRgbDepth);
  btn_layout->addWidget(btn_toggle);
  btn_layout->addStretch();
  layout->addLayout(btn_layout);

  return group;
}

QWidget* MainWindow::createStateBar() {
  auto* group = new QGroupBox("Robot State", this);
  auto* layout = new QGridLayout(group);
  layout->setSpacing(4);

  const char* pose_names[] = {"X:", "Y:", "Z:", "R:", "P:", "Yw:"};
  for (int i = 0; i < 6; ++i) {
    layout->addWidget(new QLabel(pose_names[i]), 0, i * 2);
    label_pose_[i] = new QLabel("0.0000");
    label_pose_[i]->setMinimumWidth(80);
    QFont mono;
    mono.setFamily("Monospace");
    label_pose_[i]->setFont(mono);
    layout->addWidget(label_pose_[i], 0, i * 2 + 1);
  }

  layout->addWidget(new QLabel("Finger:"), 0, 12);
  label_finger_ = new QLabel("0.0000");
  label_finger_->setFont(QFont("Monospace"));
  label_finger_->setMinimumWidth(60);
  layout->addWidget(label_finger_, 0, 13);

  layout->addWidget(new QLabel("TCP:"), 0, 14);
  label_tcp_ = new QLabel("hand");
  layout->addWidget(label_tcp_, 0, 15);

  layout->setColumnStretch(16, 1);

  return group;
}

QWidget* MainWindow::createControlPanel() {
  auto* panel = new QWidget(this);
  auto* h_layout = new QHBoxLayout(panel);
  h_layout->setSpacing(8);

  // === 左侧：关节控制（Slider + 角度输入）===
  auto* joint_group = new QGroupBox("Joint Control", this);
  auto* joint_layout = new QGridLayout(joint_group);
  joint_layout->setColumnStretch(1, 1);  // slider stretches

  for (int i = 0; i < 7; ++i) {
    joint_layout->addWidget(new QLabel(QString("J%1:").arg(i + 1)), i, 0);

    slider_joint_[i] = new QSlider(Qt::Horizontal, this);
    slider_joint_[i]->setRange(
        degToSlider(kJointLowerDeg[i]),
        degToSlider(kJointUpperDeg[i]));
    slider_joint_[i]->setValue(0);
    slider_joint_[i]->setTracking(true);
    joint_layout->addWidget(slider_joint_[i], i, 1);

    edit_joint_[i] = new QLineEdit("0.0°", this);
    edit_joint_[i]->setFixedWidth(70);
    edit_joint_[i]->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    QFont mono("Monospace");
    edit_joint_[i]->setFont(mono);
    joint_layout->addWidget(edit_joint_[i], i, 2);

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

  h_layout->addWidget(joint_group, 2);

  // === 中间：笛卡尔控制 + Jog ===
  auto* cart_group = new QGroupBox("Cartesian Control", this);
  auto* cart_layout = new QVBoxLayout(cart_group);

  // 运动模式
  auto* mode_layout = new QHBoxLayout();
  mode_layout->addWidget(new QLabel("Mode:"));
  combo_motion_mode_ = new QComboBox(this);
  combo_motion_mode_->addItem("moveJ");
  combo_motion_mode_->addItem("moveL");
  combo_motion_mode_->setCurrentIndex(0);
  mode_layout->addWidget(combo_motion_mode_);
  mode_layout->addStretch();
  cart_layout->addLayout(mode_layout);

  // XYZ 输入
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
  cart_layout->addLayout(xyz_grid);

  // RPY 输入
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
  cart_layout->addLayout(rpy_grid);

  // 执行按钮
  auto* btn_move_pose = new QPushButton("Move to Pose", this);
  btn_move_pose->setStyleSheet("padding: 6px; font-weight: bold;");
  connect(btn_move_pose, &QPushButton::clicked, this, &MainWindow::onMovePose);
  cart_layout->addWidget(btn_move_pose);

  // Jog 按钮
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

  cart_layout->addWidget(jog_group);
  h_layout->addWidget(cart_group, 2);

  // === 右侧：速度 + 功能按钮 ===
  auto* func_group = new QGroupBox("Functions", this);
  auto* func_layout = new QVBoxLayout(func_group);

  func_layout->addWidget(new QLabel("moveJ Speed:"));
  auto* speed_j_layout = new QHBoxLayout();
  slider_speed_j_ = new QSlider(Qt::Horizontal, this);
  slider_speed_j_->setRange(1, 100);
  slider_speed_j_->setValue(50);
  label_speed_j_ = new QLabel("50%", this);
  label_speed_j_->setMinimumWidth(40);
  connect(slider_speed_j_, &QSlider::valueChanged, this, &MainWindow::onSetSpeedJ);
  speed_j_layout->addWidget(slider_speed_j_);
  speed_j_layout->addWidget(label_speed_j_);
  func_layout->addLayout(speed_j_layout);

  func_layout->addWidget(new QLabel("moveL Speed:"));
  auto* speed_l_layout = new QHBoxLayout();
  slider_speed_l_ = new QSlider(Qt::Horizontal, this);
  slider_speed_l_->setRange(1, 100);
  slider_speed_l_->setValue(50);
  label_speed_l_ = new QLabel("50%", this);
  label_speed_l_->setMinimumWidth(40);
  connect(slider_speed_l_, &QSlider::valueChanged, this, &MainWindow::onSetSpeedL);
  speed_l_layout->addWidget(slider_speed_l_);
  speed_l_layout->addWidget(label_speed_l_);
  func_layout->addLayout(speed_l_layout);

  func_layout->addSpacing(10);

  auto* btn_open = new QPushButton("Open Gripper", this);
  btn_open->setStyleSheet("padding: 6px;");
  connect(btn_open, &QPushButton::clicked, this, &MainWindow::onOpenGripper);
  func_layout->addWidget(btn_open);

  auto* btn_close = new QPushButton("Close Gripper", this);
  btn_close->setStyleSheet("padding: 6px;");
  connect(btn_close, &QPushButton::clicked, this, &MainWindow::onCloseGripper);
  func_layout->addWidget(btn_close);

  func_layout->addSpacing(10);

  auto* btn_home = new QPushButton("Go Home", this);
  btn_home->setStyleSheet("padding: 6px; font-weight: bold;");
  connect(btn_home, &QPushButton::clicked, this, &MainWindow::onGoHome);
  func_layout->addWidget(btn_home);

  auto* btn_estop = new QPushButton("E-STOP", this);
  btn_estop->setStyleSheet(
      "padding: 10px; font-weight: bold; font-size: 14px; "
      "background-color: #cc0000; color: white; border-radius: 4px;");
  connect(btn_estop, &QPushButton::clicked, this, &MainWindow::onEmergencyStop);
  func_layout->addWidget(btn_estop);

  func_layout->addStretch();

  h_layout->addWidget(func_group, 1);

  return panel;
}

QPushButton* MainWindow::createJogButton(const QString& text, int axis) {
  auto* btn = new QPushButton(text, this);
  btn->setFixedSize(50, 35);
  btn->setAutoRepeat(false);

  connect(btn, &QPushButton::pressed, this, [this, axis]() {
    onJogPress(axis);
  });
  connect(btn, &QPushButton::released, this, [this]() {
    onJogRelease();
  });

  return btn;
}

// ===== Slots（全部异步，不阻塞 GUI） =====

void MainWindow::onToggleRgbDepth() {
  show_depth_ = !show_depth_;
  // TODO: 切换到深度图显示时，需要从 ROS2 节点获取最近一帧 depth
}

void MainWindow::onOpenGripper() {
  if (estop_active_.load()) return;
  node_->async_open_gripper();
}

void MainWindow::onCloseGripper() {
  if (estop_active_.load()) return;
  node_->async_close_gripper();
}

void MainWindow::onGoHome() {
  if (estop_active_.load()) return;
  node_->async_go_home();
}

void MainWindow::onEmergencyStop() {
  estop_active_ = true;
  node_->emergency_stop();
  image_label_->setText("E-STOP ACTIVATED");
  image_label_->setStyleSheet(
      "background-color: #cc0000; color: white; font-size: 24px; "
      "font-weight: bold; border: 2px solid #ff0000;");
}

void MainWindow::onMovePose() {
  if (estop_active_.load()) return;
  std::array<double, 3> xyz{
      spin_xyz_[0]->value(), spin_xyz_[1]->value(), spin_xyz_[2]->value()};
  std::array<double, 3> rpy{
      spin_rpy_[0]->value(), spin_rpy_[1]->value(), spin_rpy_[2]->value()};
  uint8_t mode = static_cast<uint8_t>(combo_motion_mode_->currentIndex());
  node_->async_move_pose(xyz, rpy, mode);
}

void MainWindow::onSetSpeedJ(int value) {
  label_speed_j_->setText(QString("%1%").arg(value));
  node_->async_set_speed(0, static_cast<double>(value));
}

void MainWindow::onSetSpeedL(int value) {
  label_speed_l_->setText(QString("%1%").arg(value));
  node_->async_set_speed(1, static_cast<double>(value));
}

void MainWindow::onJogPress(int axis) {
  if (estop_active_.load()) return;
  uint8_t mode = static_cast<uint8_t>(combo_motion_mode_->currentIndex());
  node_->start_jog(axis, 1.0, mode);
}

void MainWindow::onJogRelease() {
  node_->stop_jog();
}

void MainWindow::onRefreshState() {
  // Connection status updates even during e-stop
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

  if (!services) return;

  node_->async_get_state(
      [this](bool success,
             const std::vector<double>& joints,
             const std::array<double, 6>& pose,
             double finger,
             const std::string& tcp) {
        if (!success) return;
        QMetaObject::invokeMethod(this, [this, joints, pose, finger, tcp]() {
          for (int i = 0; i < 7 && i < (int)joints.size(); ++i) {
            if (!slider_is_controlled_[i]) {
              syncSliderToState(i, joints[i]);
              last_streamed_joints_[i] = joints[i];
            }
          }
          for (int i = 0; i < 6; ++i) {
            label_pose_[i]->setText(QString::number(pose[i], 'f', 4));
          }
          label_finger_->setText(QString::number(finger, 'f', 4));
          label_tcp_->setText(QString::fromStdString(tcp));

          // 首次状态刷新成功后启动关节流控
          if (!joint_stream_started_) {
            joint_stream_started_ = true;
            node_->start_joint_stream(last_streamed_joints_);
            joint_follow_timer_->start(20);
          }
        });
      });
}

// ===== Joint slider slot implementations =====

void MainWindow::syncSliderToState(int joint, double rad) {
  if (slider_is_controlled_[joint]) return;
  double deg = radToDeg(rad);
  slider_joint_[joint]->blockSignals(true);
  slider_joint_[joint]->setValue(degToSlider(deg));
  slider_joint_[joint]->blockSignals(false);
  edit_joint_[joint]->setText(QString::number(deg, 'f', 1) + QString::fromUtf8("\u00B0"));
}

void MainWindow::onJointSliderPressed(int joint) {
  slider_is_controlled_[joint] = true;
  lock_timer_[joint]->stop();
}

void MainWindow::onJointSliderReleased(int joint) {
  lock_timer_[joint]->start();
}

void MainWindow::onJointEditFinished(int joint) {
  if (estop_active_.load()) return;

  QString text = edit_joint_[joint]->text();
  text.remove(QString::fromUtf8("\u00B0"));
  bool ok = false;
  double deg = text.toDouble(&ok);
  if (!ok) {
    // Restore to current slider position
    deg = sliderToDeg(slider_joint_[joint]->value());
    edit_joint_[joint]->setText(QString::number(deg, 'f', 1) + QString::fromUtf8("\u00B0"));
    return;
  }

  deg = std::max(kJointLowerDeg[joint], std::min(kJointUpperDeg[joint], deg));

  slider_joint_[joint]->blockSignals(true);
  slider_joint_[joint]->setValue(degToSlider(deg));
  slider_joint_[joint]->blockSignals(false);

  edit_joint_[joint]->setText(QString::number(deg, 'f', 1) + QString::fromUtf8("\u00B0"));

  std::array<double, 7> target{};
  for (int i = 0; i < 7; ++i) {
    target[i] = degToRad(sliderToDeg(slider_joint_[i]->value()));
  }
  node_->update_joint_target(target);
  last_streamed_joints_ = target;

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

// ===== Unit conversion helpers =====

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

}  // namespace teaching_pendant
