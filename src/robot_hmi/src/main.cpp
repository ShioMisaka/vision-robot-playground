#include <QApplication>
#include <csignal>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "robot_hmi/pendant_node.hpp"
#include "robot_hmi/main_window.hpp"

int main(int argc, char* argv[]) {
  // 初始化 ROS2
  rclcpp::init(argc, argv);

  // 创建示教器节点（连接到独立运行的 robot_controller_node）
  auto pendant = robot_hmi::PendantNode::create(
      "robot_controller_node", {});

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(pendant);

  std::thread ros_thread([&executor]() { executor.spin(); });

  // 初始化 Qt（在 ROS2 init 之后）
  QApplication app(argc, argv);

  // SIGINT/SIGTERM 处理：协调 Qt + ROS2 有序关闭
  std::signal(SIGINT, [](int) { QApplication::quit(); });
  std::signal(SIGTERM, [](int) { QApplication::quit(); });

  // 创建主窗口
  robot_hmi::MainWindow window(pendant);
  window.show();

  // Qt 事件循环
  int ret = app.exec();

  // 清理：先销毁节点（内部停止后台线程并 join），再关闭 ROS2
  executor.cancel();
  ros_thread.join();
  pendant.reset();
  rclcpp::shutdown();

  return ret;
}
