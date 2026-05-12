#pragma once

#include <mutex>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "robot_msgs/action/grasp_task.hpp"
#include "robot_controller/client/robot_client.hpp"
#include "robot_tasks/grasp_task_manager.hpp"

namespace robot_tasks {

/// GraspTask Action Server 节点
/// 将 GraspTaskManager 封装为 ROS2 Action Server，支持 Goal/Cancel/Feedback。
/// 内部创建 RobotClient（连接外部 robot_controller_node）和 VisionProcessorNode
/// （订阅相机话题），协调完成两阶段视觉引导抓取。
///
/// 使用方式：
///   auto node = GraspTaskNode::create();
///   auto executor = rclcpp::executors::MultiThreadedExecutor();
///   executor.add_node(node);
///   for (auto& sub : node->get_sub_nodes()) { executor.add_node(sub); }
///   executor.spin();
class GraspTaskNode : public rclcpp::Node {
 public:
  /// @brief 工厂方法：创建 GraspTask Action Server 节点
  /// @param node_name 节点名称（默认 "grasp_task_node"）
  /// @param target_prefix 目标控制器节点的服务/Action 前缀（默认 "robot_controller_node"）
  /// @return 共享指针，创建失败返回 nullptr
  static std::shared_ptr<GraspTaskNode> create(
      const std::string& node_name = "grasp_task_node",
      const std::string& target_prefix = "robot_controller_node");

  /// 获取需要加入 executor 一起 spin 的子节点
  /// （RobotClient、VisionProcessorNode 等，它们是独立的 rclcpp::Node）
  std::vector<rclcpp::Node::SharedPtr> get_sub_nodes() const;

 private:
  GraspTaskNode(const std::string& node_name,
                const std::string& target_prefix);
  void init();

  // ===== Action Server 回调 =====
  using GraspTaskAction = robot_msgs::action::GraspTask;

  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const GraspTaskAction::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<GraspTaskAction>>
          goal_handle);

  void handle_accepted(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<GraspTaskAction>>
          goal_handle);

  void execute(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<GraspTaskAction>>
          goal_handle);

  /// 将 GraspState 枚举转为 Action 常量 uint8
  static uint8_t state_to_uint8(GraspState state);

  /// 将 GraspState 枚举转为可读字符串
  static const char* state_to_string(GraspState state);

  // ===== Action Server =====
  rclcpp_action::Server<GraspTaskAction>::SharedPtr action_server_;

  // ===== 依赖 =====
  std::shared_ptr<robot_control::RobotClient> robot_client_;
  std::shared_ptr<robot_vision::IVisionProcessor> vision_;
  rclcpp::Node::SharedPtr vision_node_;  ///< VisionProcessorNode（也是 rclcpp::Node）
  std::string target_prefix_;

  // ===== 当前任务跟踪（支持取消）=====
  std::shared_ptr<rclcpp_action::ServerGoalHandle<GraspTaskAction>>
      current_goal_;
  std::unique_ptr<GraspTaskManager> current_task_;
  std::mutex task_mutex_;
};

}  // namespace robot_tasks
