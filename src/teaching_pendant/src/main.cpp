#include <QApplication>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "robot_control_cpp/nodes/robot_controller_node.hpp"
#include "robot_control_cpp/profiles/panda_profile.hpp"
#include "robot_control_cpp/nodes/topic_config.hpp"

#include "teaching_pendant/pendant_node.hpp"
#include "teaching_pendant/main_window.hpp"

int main(int argc, char* argv[]) {
  // 初始化 ROS2
  rclcpp::init(argc, argv);

  // 创建机器人控制节点（内嵌，提供所有 Service Server）
  // 与 Python 脚本架构一致：每个客户端自带 RobotControllerNode
  auto robot = robot_control::RobotControllerNode::create(
      robot_control::profiles::panda(),
      robot_control::profiles::panda_gripper(),
      robot_control::TopicConfig());

  // 创建示教器节点（Service Client）
  auto pendant = teaching_pendant::PendantNode::create();

  // 两个节点共享同一个 MultiThreadedExecutor
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(robot);
  executor.add_node(pendant);

  std::thread ros_thread([&executor]() { executor.spin(); });

  // 初始化 Qt（在 ROS2 init 之后）
  QApplication app(argc, argv);

  // 创建主窗口
  teaching_pendant::MainWindow window(pendant);
  window.show();

  // Qt 事件循环
  int ret = app.exec();

  // 清理
  executor.cancel();
  ros_thread.join();
  rclcpp::shutdown();

  return ret;
}
