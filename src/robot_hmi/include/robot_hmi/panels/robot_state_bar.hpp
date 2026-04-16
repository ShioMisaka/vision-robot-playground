#pragma once

#include <QWidget>
#include <QLabel>
#include <array>
#include <string>

namespace robot_hmi {

class RobotStateBar : public QWidget {
  Q_OBJECT
public:
  explicit RobotStateBar(QWidget* parent = nullptr);

public slots:
  void onStateUpdated(const std::array<double, 6>& pose,
                      double finger_width,
                      const std::string& tcp_name);

private:
  QLabel* label_pose_[6];
  QLabel* label_finger_;
  QLabel* label_tcp_;
};

}  // namespace robot_hmi
