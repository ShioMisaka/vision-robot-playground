#include <rclcpp/rclcpp.hpp>

#include "robot_controller/nodes/robot_controller_node.hpp"
#include "robot_controller/profiles/panda_profile.hpp"
#include "robot_controller/motion/topic_config.hpp"

#include <robot_logger/logger.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto profile = robot_control::profiles::panda();
  auto gripper = robot_control::profiles::panda_gripper();

  auto node = robot_control::RobotControllerNode::create(
      profile, gripper, robot_control::TopicConfig());

  LOG_INFO("robot_controller_node (standalone) started");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
