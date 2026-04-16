#pragma once

#include <QWidget>
#include <QLabel>

namespace teaching_pendant {

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

}  // namespace teaching_pendant
