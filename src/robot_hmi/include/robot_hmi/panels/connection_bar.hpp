#pragma once

#include <QWidget>
#include <QLabel>

namespace robot_hmi {

class ConnectionBar : public QWidget {
  Q_OBJECT
public:
  explicit ConnectionBar(QWidget* parent = nullptr);

public slots:
  void onRobotConnectionChanged(bool connected, bool services_ready);
  void onCameraConnectionChanged(bool connected);

private:
  QLabel* label_robot_status_;
  QLabel* label_camera_status_;
};

}  // namespace robot_hmi
