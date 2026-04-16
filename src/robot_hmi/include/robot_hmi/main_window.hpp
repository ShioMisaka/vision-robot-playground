#pragma once

#include <QMainWindow>
#include <QTimer>
#include <memory>

namespace robot_hmi {

class PendantNode;
class ConnectionBar;
class CameraPanel;
class RobotStateBar;
class JointControlPanel;
class CartesianPanel;
class FunctionPanel;

class MainWindow : public QMainWindow {
  Q_OBJECT
public:
  explicit MainWindow(std::shared_ptr<PendantNode> node,
                      QWidget* parent = nullptr);

private slots:
  void onRefreshState();

private:
  void setupUi();

  std::shared_ptr<PendantNode> node_;

  ConnectionBar* connection_bar_;
  CameraPanel* camera_panel_;
  RobotStateBar* state_bar_;
  JointControlPanel* joint_panel_;
  CartesianPanel* cartesian_panel_;
  FunctionPanel* function_panel_;

  QTimer* refresh_timer_;
};

}  // namespace robot_hmi
