#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "robot_api/service_robot_controller.hpp"

namespace robot_api {

/// 轻量 ROS2 客户端节点，连接到独立 robot_controller_node
/// 持有 ServiceRobotController，通过 ROS2 Service 控制机器人
class RobotClient : public rclcpp::Node {
 public:
  static std::shared_ptr<RobotClient> create(
      const std::string& service_prefix = "robot_controller_node");

  /// 获取运动控制器（通过 Service 调用实现）
  std::shared_ptr<ServiceRobotController> get_controller() {
    return controller_;
  }

  /// 等待所有必需的 Service 就绪
  bool wait_for_services(double timeout = 10.0);

  /// 获取节点 logger
  using rclcpp::Node::get_logger;

 private:
  explicit RobotClient(const std::string& service_prefix);
  void init();

  std::string service_prefix_;
  std::shared_ptr<ServiceRobotController> controller_;
};

}  // namespace robot_api
