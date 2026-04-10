#pragma once

#include <QMainWindow>
#include <QLabel>
#include <QPushButton>
#include <QSlider>
#include <QDoubleSpinBox>
#include <QLineEdit>
#include <QComboBox>
#include <QGroupBox>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QTimer>
#include <QFrame>
#include <mutex>
#include <atomic>

#include <QImage>
#include <array>
#include <vector>
#include <optional>

#include "teaching_pendant/pendant_node.hpp"

namespace teaching_pendant {

/// 示教器主窗口
class MainWindow : public QMainWindow {
  Q_OBJECT

public:
  explicit MainWindow(std::shared_ptr<PendantNode> node, QWidget* parent = nullptr);

private slots:
  void onToggleRgbDepth();
  void onOpenGripper();
  void onCloseGripper();
  void onGoHome();
  void onEmergencyStop();
  void onMovePose();
  void onSetSpeedJ(int value);
  void onSetSpeedL(int value);

  // Jog slots
  void onJogPress(int axis);
  void onJogRelease();

  // Timer: 异步查询状态
  void onRefreshState();

private:
  void setupUi();

  // Widget helpers
  QWidget* createConnectionBar();
  QWidget* createCameraPanel();
  QWidget* createStateBar();
  QWidget* createControlPanel();

  // Jog button helper
  QPushButton* createJogButton(const QString& text, int axis);

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

  // ROS2 node
  std::shared_ptr<PendantNode> node_;

  // Camera state
  bool show_depth_ = false;
  std::mutex image_mutex_;
  QLabel* image_label_;

  // State display
  QLabel* label_pose_[6];  // X, Y, Z, R, P, Y
  QLabel* label_joints_[7];
  QLabel* label_finger_;
  QLabel* label_tcp_;

  // Connection indicators
  QLabel* label_robot_status_;
  QLabel* label_camera_status_;

  // Joint slider controls
  QSlider* slider_joint_[7];
  QLineEdit* edit_joint_[7];
  QTimer* joint_follow_timer_;
  QTimer* lock_timer_[7];  // per-joint interaction lock timer
  bool slider_is_controlled_[7] = {};
  std::array<double, 7> last_streamed_joints_{};

  // Cartesian control inputs
  QDoubleSpinBox* spin_xyz_[3];
  QDoubleSpinBox* spin_rpy_[3];

  // Speed sliders
  QSlider* slider_speed_j_;
  QSlider* slider_speed_l_;
  QLabel* label_speed_j_;
  QLabel* label_speed_l_;

  // Motion mode
  QComboBox* combo_motion_mode_;

  // Refresh timer
  QTimer* refresh_timer_;

  // E-stop state
  std::atomic<bool> estop_active_{false};
};

}  // namespace teaching_pendant
