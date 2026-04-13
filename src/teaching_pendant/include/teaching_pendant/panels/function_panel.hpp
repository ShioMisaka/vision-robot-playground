#pragma once

#include <QWidget>
#include <QSlider>
#include <QLabel>
#include <QPushButton>
#include <atomic>
#include <memory>

namespace teaching_pendant {

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

private:
  std::shared_ptr<PendantNode> node_;

  QSlider* slider_speed_j_;
  QSlider* slider_speed_l_;
  QLabel* label_speed_j_;
  QLabel* label_speed_l_;

  std::atomic<bool> estop_active_{false};
};

}  // namespace teaching_pendant
