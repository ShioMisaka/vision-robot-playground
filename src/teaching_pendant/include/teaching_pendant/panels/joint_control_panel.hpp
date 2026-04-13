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
  void onStateUpdated(const std::vector<double>& joint_angles);
  void onEstopChanged(bool active);

signals:
  void jointStreamReady(const std::array<double, 7>& initial_joints);

private:
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
