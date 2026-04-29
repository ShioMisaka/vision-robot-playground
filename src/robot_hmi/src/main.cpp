#include <QApplication>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "robot_controller/nodes/robot_controller_node.hpp"
#include "robot_controller/profiles/panda_profile.hpp"
#include "robot_controller/motion/topic_config.hpp"

#include "robot_hmi/pendant_node.hpp"
#include "robot_hmi/main_window.hpp"

int main(int argc, char* argv[]) {
  // 初始化 ROS2
  rclcpp::init(argc, argv);

  // 创建机器人参数（复用同一 profile）
  auto profile = robot_control::profiles::panda();
  auto gripper = robot_control::profiles::panda_gripper();

  // 创建机器人控制节点（内嵌，提供所有 Service Server）
  auto robot = robot_control::RobotControllerNode::create(
      profile, gripper, robot_control::TopicConfig());

  // 创建示教器节点（Jog IK 由 RobotControllerNode 处理）
  auto pendant = robot_hmi::PendantNode::create(
      "robot_controller_node", profile.joint_names);

  // 让 PendantNode 能在脚本接管时暂停内嵌控制器的 100Hz 循环
  pendant->set_controller_node(robot);

  // 两个节点共享同一个 MultiThreadedExecutor
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(robot);
  executor.add_node(pendant);

  std::thread ros_thread([&executor]() { executor.spin(); });

  // 初始化 Qt（在 ROS2 init 之后）
  QApplication app(argc, argv);

  // 创建主窗口
  robot_hmi::MainWindow window(pendant);
  window.show();

  // Qt 事件循环
  int ret = app.exec();

  // 清理
  executor.cancel();
  ros_thread.join();
  rclcpp::shutdown();

  return ret;
}
