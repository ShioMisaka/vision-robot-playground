#include "robot_hmi/main_window.hpp"
#include "robot_hmi/pendant_node.hpp"

#include "robot_hmi/panels/connection_bar.hpp"
#include "robot_hmi/panels/camera_panel.hpp"
#include "robot_hmi/panels/robot_state_bar.hpp"
#include "robot_hmi/panels/joint_control_panel.hpp"
#include "robot_hmi/panels/cartesian_panel.hpp"
#include "robot_hmi/panels/function_panel.hpp"

#include <QVBoxLayout>
#include <QImage>
#include <QPixmap>
#include <opencv2/core.hpp>

namespace robot_hmi {

MainWindow::MainWindow(std::shared_ptr<PendantNode> node, QWidget* parent)
    : QMainWindow(parent), node_(std::move(node)) {
  setWindowTitle("Robot Teaching Pendant");
  resize(1100, 800);

  setupUi();

  // Connection state -> ConnectionBar
  node_->set_connection_callback(
      [this](bool /*robot*/, bool camera) {
        QMetaObject::invokeMethod(this, [this, camera]() {
          connection_bar_->onCameraConnectionChanged(camera);
        });
      });

  // Image -> CameraPanel (cv::Mat -> QImage conversion)
  node_->set_image_callback(
      [this](const cv::Mat& rgb, const cv::Mat& depth_visual,
             const cv::Mat& depth_raw) {
        cv::Mat rgb_copy = rgb.clone();
        cv::Mat depth_copy = depth_visual.clone();
        cv::Mat raw_copy = depth_raw.clone();
        QMetaObject::invokeMethod(this, [this, rgb_copy, depth_copy, raw_copy]() {
          QImage rgb_img = QImage(rgb_copy.data, rgb_copy.cols, rgb_copy.rows,
                      static_cast<int>(rgb_copy.step), QImage::Format_RGB888).copy();
          QImage depth_img = QImage(depth_copy.data, depth_copy.cols, depth_copy.rows,
                      static_cast<int>(depth_copy.step), QImage::Format_RGB888).copy();
          camera_panel_->onImageReceived(rgb_img, depth_img, raw_copy);
        });
      });

  // E-STOP signal relay
  connect(function_panel_, &FunctionPanel::estopChanged,
          joint_panel_, &JointControlPanel::onEstopChanged);
  connect(function_panel_, &FunctionPanel::estopChanged,
          cartesian_panel_, &CartesianPanel::onEstopChanged);
  connect(function_panel_, &FunctionPanel::estopChanged,
          camera_panel_, &CameraPanel::onEstopChanged);

  // Joint stream start
  connect(joint_panel_, &JointControlPanel::jointStreamReady,
          this, [this](const std::array<double, 7>& initial) {
            node_->start_joint_stream(initial);
          });

  // Jog stopped -> resync joint panel's command_target_
  node_->set_jog_stopped_callback(
      [this]() {
        QMetaObject::invokeMethod(this, [this]() {
          joint_panel_->notifyJogStopped();
        });
      });

  // State refresh timer (5Hz)
  refresh_timer_ = new QTimer(this);
  connect(refresh_timer_, &QTimer::timeout, this, &MainWindow::onRefreshState);
  refresh_timer_->start(200);
}

void MainWindow::setupUi() {
  auto* central = new QWidget(this);
  auto* main_layout = new QVBoxLayout(central);
  main_layout->setSpacing(6);
  main_layout->setContentsMargins(8, 8, 8, 8);

  connection_bar_ = new ConnectionBar(this);
  main_layout->addWidget(connection_bar_);

  camera_panel_ = new CameraPanel(this);
  main_layout->addWidget(camera_panel_, 3);

  state_bar_ = new RobotStateBar(this);
  main_layout->addWidget(state_bar_);

  // Control panel: Joint | Cartesian | Function
  auto* control = new QWidget(this);
  auto* h_layout = new QHBoxLayout(control);
  h_layout->setSpacing(8);

  joint_panel_ = new JointControlPanel(node_, this);
  cartesian_panel_ = new CartesianPanel(node_, this);
  function_panel_ = new FunctionPanel(node_, this);

  h_layout->addWidget(joint_panel_, 2);
  h_layout->addWidget(cartesian_panel_, 2);
  h_layout->addWidget(function_panel_, 1);

  main_layout->addWidget(control, 2);
  setCentralWidget(central);
}

void MainWindow::onRefreshState() {
  bool robot_topic = node_->is_robot_connected();
  bool services = node_->are_services_ready();
  connection_bar_->onRobotConnectionChanged(robot_topic, services);

  if (!services) return;

  node_->async_get_state(
      [this](bool success,
             const std::vector<double>& joints,
             const std::array<double, 6>& pose,
             double finger,
             const std::string& tcp) {
        if (!success) return;
        QMetaObject::invokeMethod(this, [this, joints, pose, finger, tcp]() {
          state_bar_->onStateUpdated(pose, finger, tcp);
          joint_panel_->onStateUpdated(joints);
        });
      });
}

}  // namespace robot_hmi
