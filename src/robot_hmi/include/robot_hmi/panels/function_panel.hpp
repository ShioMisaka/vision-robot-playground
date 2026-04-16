#pragma once

#include <QWidget>
#include <QSlider>
#include <QLabel>
#include <QPushButton>
#include <QTimer>
#include <atomic>
#include <memory>

namespace robot_hmi {

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
  void onCheckFaultState();

private:
  void update_estop_button(bool fault);

  std::shared_ptr<PendantNode> node_;

  QSlider* slider_speed_j_;
  QSlider* slider_speed_l_;
  QLabel* label_speed_j_;
  QLabel* label_speed_l_;
  QPushButton* btn_estop_;

  std::atomic<bool> estop_active_{false};
  QTimer* fault_check_timer_;
};

}  // namespace robot_hmi
