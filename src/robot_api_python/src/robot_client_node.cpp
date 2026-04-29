#include "robot_api_python/robot_client_node.hpp"

#include <chrono>
#include <robot_logger/logger.hpp>

namespace robot_api_python {

std::shared_ptr<RobotClientNode> RobotClientNode::create(
    const std::string& service_prefix) {
  auto node = std::shared_ptr<RobotClientNode>(
      new RobotClientNode(service_prefix));
  return node;
}

RobotClientNode::RobotClientNode(const std::string& service_prefix)
    : Node("robot_client_node") {
  controller_ = std::make_shared<ServiceRobotController>(
      shared_from_this(), service_prefix);
  LOG_INFO("RobotClientNode created (prefix: {})", service_prefix);
}

bool RobotClientNode::wait_for_services(double timeout) {
  auto start = std::chrono::steady_clock::now();
  auto deadline = start + std::chrono::duration<double>(timeout);

  // Wait for critical services
  auto ctrl = controller_;  // avoid accessing member in lambda
  auto cli_move = ctrl->cli_move_joint_;
  auto cli_pose = ctrl->cli_move_pose_;
  auto cli_gripper = ctrl->cli_gripper_;
  auto cli_state = ctrl->cli_state_;

  while (std::chrono::steady_clock::now() < deadline) {
    if (cli_move->service_is_ready() && cli_pose->service_is_ready() &&
        cli_gripper->service_is_ready() && cli_state->service_is_ready()) {
      LOG_INFO("All robot services ready");
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (!rclcpp::ok()) return false;
  }

  LOG_ERROR("Timeout waiting for robot services ({:.1f}s)", timeout);
  return false;
}

}  // namespace robot_api_python
