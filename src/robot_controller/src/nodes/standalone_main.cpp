#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "robot_controller/nodes/robot_controller_node.hpp"
#include "robot_controller/kinematics/profile_loader.hpp"
#include "robot_controller/nodes/topic_config.hpp"

#include <robot_logger/logger.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  const auto desc_dir =
      ament_index_cpp::get_package_share_directory("robot_description");
  const auto config = robot_control::ProfileLoader::load(
      desc_dir + "/config/panda_profile.yaml", desc_dir);

  auto node = robot_control::RobotControllerNode::create(
      config.robot, config.gripper, robot_control::TopicConfig());

  LOG_INFO("robot_controller_node (standalone) started");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
