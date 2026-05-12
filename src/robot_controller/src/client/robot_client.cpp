#include "robot_controller/client/robot_client.hpp"

#include <chrono>
#include <thread>

#include <robot_logger/logger.hpp>

namespace robot_control {

using namespace std::chrono_literals;

std::shared_ptr<RobotClient> RobotClient::create(
    const std::string& node_name,
    const std::string& target_prefix) {
  auto node = std::shared_ptr<RobotClient>(
      new RobotClient(node_name, target_prefix));
  node->init();
  return node;
}

RobotClient::RobotClient(const std::string& node_name,
                         const std::string& target_prefix)
    : Node(node_name), target_prefix_(target_prefix) {
  // init() will be called by create() after shared_from_this() is safe
}

void RobotClient::init() {
  controller_ = std::make_shared<ActionRobotController>(
      shared_from_this(), target_prefix_);
  LOG_INFO("RobotClient created (prefix: {})", target_prefix_);
}

bool RobotClient::wait_for_services(double timeout) {
  return controller_->wait_for_actions(timeout) &&
         controller_->wait_for_services(timeout);
}

std::string RobotClient::session_id() const {
  return controller_->session_id();
}

}  // namespace robot_control
