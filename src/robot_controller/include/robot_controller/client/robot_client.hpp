#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "robot_controller/client/action_robot_controller.hpp"

namespace robot_control {

/// 轻量 ROS2 客户端节点，连接到独立 robot_controller_node。
/// 持有 ActionRobotController，通过 ROS2 Action + Service 控制机器人。
/// 替代旧的 robot_api::RobotClient（位于 robot_api_cpp 包）。
class RobotClient : public rclcpp::Node {
 public:
  /// @brief 工厂创建客户端节点
  /// @param node_name 节点名称（默认 "robot_client"）
  /// @param target_prefix 目标控制器节点的服务/Action 前缀（默认 "robot_controller_node"）
  /// @return 共享指针，创建失败返回 nullptr
  static std::shared_ptr<RobotClient> create(
      const std::string& node_name = "robot_client",
      const std::string& target_prefix = "robot_controller_node");

  /// 获取运动控制器（通过 Action/Service 调用实现）
  std::shared_ptr<ActionRobotController> get_controller() {
    return controller_;
  }

  /// 等待所有必需的 Action 和 Service 就绪
  /// @param timeout 超时时间（秒），默认 10s
  /// @return true 表示全部就绪
  bool wait_for_services(double timeout = 10.0);

  /// 获取当前 session_id
  std::string session_id() const;

 private:
  explicit RobotClient(const std::string& node_name,
                       const std::string& target_prefix);
  void init();

  std::string target_prefix_;
  std::shared_ptr<ActionRobotController> controller_;
};

}  // namespace robot_control
