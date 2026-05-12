#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <robot_msgs/action/move_j.hpp>
#include <robot_msgs/action/move_l.hpp>
#include <robot_msgs/action/go_home.hpp>

namespace robot_control {

class RobotControllerNode;

/// @brief Action Server 处理器（MoveJ / MoveL / GoHome）
/// @details 作为 RobotControllerNode 的辅助类，管理 3 个 Action Server。
///          每个 Action 提供 handle_goal / handle_cancel / handle_accepted 回调。
///          handle_accepted 在独立线程中执行运动并周期发布 Feedback。
class ActionHandlers {
public:
  using MoveJAction = robot_msgs::action::MoveJ;
  using MoveLAction = robot_msgs::action::MoveL;
  using GoHomeAction = robot_msgs::action::GoHome;

  explicit ActionHandlers(RobotControllerNode* node);
  ~ActionHandlers();

  // === MoveJ callbacks ===
  rclcpp_action::GoalResponse handle_movej_goal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const MoveJAction::Goal> goal);
  rclcpp_action::CancelResponse handle_movej_cancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveJAction>> gh);
  void handle_movej_accepted(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveJAction>> gh);

  // === MoveL callbacks ===
  rclcpp_action::GoalResponse handle_movel_goal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const MoveLAction::Goal> goal);
  rclcpp_action::CancelResponse handle_movel_cancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveLAction>> gh);
  void handle_movel_accepted(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveLAction>> gh);

  // === GoHome callbacks ===
  rclcpp_action::GoalResponse handle_gohome_goal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const GoHomeAction::Goal> goal);
  rclcpp_action::CancelResponse handle_gohome_cancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<GoHomeAction>> gh);
  void handle_gohome_accepted(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<GoHomeAction>> gh);

  /// @brief 通知当前活跃的 Action 执行线程需要取消（由 RobotCmd STOP 触发）
  void request_cancel();

private:
  /// @brief 通用验证逻辑：session 有效 + 状态 IDLE
  bool validate_goal(const std::string& session_id) const;

  /// @brief MoveJ 执行体
  void execute_movej(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveJAction>> gh);

  /// @brief MoveL 执行体
  void execute_movel(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveLAction>> gh);

  /// @brief GoHome 执行体
  void execute_gohome(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<GoHomeAction>> gh);

  /// @brief 等待运动完成并周期性发布 Feedback（通用）
  /// @tparam FeedbackT 反馈消息类型
  /// @tparam GoalHandleT GoalHandle 类型
  /// @param gh GoalHandle
  /// @param get_feedback 构造 Feedback 的回调
  template <typename FeedbackT, typename GoalHandleT>
  void wait_and_publish_feedback(
      std::shared_ptr<GoalHandleT> gh,
      std::function<FeedbackT()>&& get_feedback);

  RobotControllerNode* node_;

  /// @brief 取消标志（原子变量，由 request_cancel 或 handle_cancel 设置）
  std::atomic<bool> cancel_requested_{false};

  /// @brief 当前活跃的执行线程（同一时刻只有一个 Action 可执行）
  std::thread exec_thread_;
  std::mutex exec_mutex_;
};

}  // namespace robot_control
