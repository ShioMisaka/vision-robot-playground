#pragma once

#include <QWidget>
#include <QDoubleSpinBox>
#include <QComboBox>
#include <QPushButton>
#include <QEvent>
#include <atomic>
#include <memory>

namespace robot_hmi {

class PendantNode;

class CartesianPanel : public QWidget {
  Q_OBJECT
public:
  explicit CartesianPanel(std::shared_ptr<PendantNode> node,
                          QWidget* parent = nullptr);

public slots:
  void onEstopChanged(bool active);

protected:
  bool eventFilter(QObject* watched, QEvent* event) override;

private slots:
  void onMovePose();
  void onReadCurrentPose();
  void onJogPress(int axis);
  void onJogRelease();

private:
  QPushButton* createJogButton(const QString& text, int axis);

  std::shared_ptr<PendantNode> node_;

  QDoubleSpinBox* spin_xyz_[3];
  QDoubleSpinBox* spin_rpy_[3];
  QComboBox* combo_motion_mode_;
  QComboBox* combo_frame_;

  std::atomic<bool> estop_active_{false};
};

}  // namespace robot_hmi
