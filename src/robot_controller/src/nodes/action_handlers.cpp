#include "robot_controller/nodes/action_handlers.hpp"
#include "robot_controller/nodes/robot_controller_node.hpp"
#include "robot_controller/nodes/robot_state.hpp"
#include "robot_controller/motion/control_constants.hpp"

#include <robot_logger/logger.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <vector>

namespace robot_control {

// ============================================================================
// Construction / Destruction
// ============================================================================

ActionHandlers::ActionHandlers(RobotControllerNode* node) : node_(node) {}

ActionHandlers::~ActionHandlers() {
  cancel_requested_ = true;
  if (exec_thread_.joinable()) {
    exec_thread_.join();
  }
}

// ============================================================================
// Validation helper
// ============================================================================

bool ActionHandlers::validate_goal(const std::string& session_id) const {
  if (!node_->lease_manager().is_valid_session(session_id)) {
    LOG_WARN("Action goal rejected: invalid session_id='{}'", session_id);
    return false;
  }
  auto state = node_->state_machine().state();
  if (state != RobotState::kIdle) {
    LOG_WARN("Action goal rejected: robot not IDLE (state={})",
             RobotStateMachine::state_name(state));
    return false;
  }
  return true;
}

// ============================================================================
// Cancel from external source (RobotCmd STOP)
// ============================================================================

void ActionHandlers::request_cancel() { cancel_requested_ = true; }

// ============================================================================
// MoveJ
// ============================================================================

rclcpp_action::GoalResponse ActionHandlers::handle_movej_goal(
    const rclcpp_action::GoalUUID& /*uuid*/,
    std::shared_ptr<const MoveJAction::Goal> goal) {
  if (!validate_goal(goal->session_id)) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ActionHandlers::handle_movej_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveJAction>> /*gh*/) {
  cancel_requested_ = true;
  LOG_INFO("MoveJ cancel requested");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ActionHandlers::handle_movej_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveJAction>> gh) {
  // 等待前一个执行线程结束
  if (exec_thread_.joinable()) {
    exec_thread_.join();
  }
  cancel_requested_ = false;
  exec_thread_ = std::thread(&ActionHandlers::execute_movej, this, gh);
}

void ActionHandlers::execute_movej(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveJAction>> gh) {
  auto goal = gh->get_goal();
  auto controller = node_->get_controller();
  bool success = false;
  std::string message;

  try {
    const uint8_t JOINT_SPACE = 0;
    const uint8_t CARTESIAN = 1;

    if (goal->mode == JOINT_SPACE) {
      // 关节空间 MoveJ
      std::vector<double> angles(goal->joint_angles.begin(),
                                  goal->joint_angles.end());
      controller->moveJ(angles, false);
    } else if (goal->mode == CARTESIAN) {
      // 笛卡尔空间 MoveJ（IK + 关节空间轨迹）
      std::array<double, 3> xyz = {
          goal->position.x, goal->position.y, goal->position.z};
      std::optional<std::array<double, 3>> rpy;
      if (!std::isnan(goal->orientation.x) ||
          !std::isnan(goal->orientation.y) ||
          !std::isnan(goal->orientation.z)) {
        rpy = std::array<double, 3>{
            goal->orientation.x, goal->orientation.y, goal->orientation.z};
      }
      controller->moveJ(xyz, rpy, goal->finger_width, false);
    } else {
      auto result = std::make_shared<MoveJAction::Result>();
      result->success = false;
      result->message = "Invalid mode: " + std::to_string(goal->mode);
      gh->abort(result);
      return;
    }

    // 运动已提交（非阻塞），等待完成并发布 Feedback
    auto feedback_fn = [this]() -> MoveJAction::Feedback {
      MoveJAction::Feedback fb;
      auto& gen = node_->get_bridge()->setpoint_generator();
      if (gen.is_active()) {
        auto sp = gen.tick(std::chrono::steady_clock::now());
        fb.progress = sp.progress;
        fb.estimated_time_remaining = sp.time_remaining;
      } else {
        fb.progress = 1.0;
        fb.estimated_time_remaining = 0.0;
      }
      auto joints = node_->get_controller()->get_joint_angles();
      std::copy_n(joints.begin(),
                   std::min(joints.size(), size_t{7}),
                   fb.current_joint_angles.begin());
      return fb;
    };

    wait_and_publish_feedback<MoveJAction::Feedback>(gh, std::move(feedback_fn));

    if (cancel_requested_) {
      auto result = std::make_shared<MoveJAction::Result>();
      result->success = false;
      result->message = "Canceled";
      auto joints = controller->get_joint_angles();
      std::copy_n(joints.begin(),
                   std::min(joints.size(), size_t{7}),
                   result->final_joint_angles.begin());
      auto pose = controller->get_end_effector_pose();
      std::copy_n(pose.begin(), 6, result->final_tcp_pose.begin());
      gh->canceled(result);
      LOG_INFO("MoveJ action canceled");
      return;
    }

    success = true;
    message = "MoveJ completed";
  } catch (const std::exception& e) {
    success = false;
    message = std::string("MoveJ failed: ") + e.what();
    LOG_ERROR("MoveJ action failed: {}", e.what());
  }

  auto result = std::make_shared<MoveJAction::Result>();
  result->success = success;
  result->message = message;
  auto joints = controller->get_joint_angles();
  std::copy_n(joints.begin(),
               std::min(joints.size(), size_t{7}),
               result->final_joint_angles.begin());
  auto pose = controller->get_end_effector_pose();
  std::copy_n(pose.begin(), 6, result->final_tcp_pose.begin());

  if (success) {
    gh->succeed(result);
    LOG_INFO("MoveJ action succeeded");
  } else {
    gh->abort(result);
  }
}

// ============================================================================
// MoveL
// ============================================================================

rclcpp_action::GoalResponse ActionHandlers::handle_movel_goal(
    const rclcpp_action::GoalUUID& /*uuid*/,
    std::shared_ptr<const MoveLAction::Goal> goal) {
  if (!validate_goal(goal->session_id)) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ActionHandlers::handle_movel_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveLAction>> /*gh*/) {
  cancel_requested_ = true;
  LOG_INFO("MoveL cancel requested");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ActionHandlers::handle_movel_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveLAction>> gh) {
  if (exec_thread_.joinable()) {
    exec_thread_.join();
  }
  cancel_requested_ = false;
  exec_thread_ = std::thread(&ActionHandlers::execute_movel, this, gh);
}

void ActionHandlers::execute_movel(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveLAction>> gh) {
  auto goal = gh->get_goal();
  auto controller = node_->get_controller();
  bool success = false;
  std::string message;

  try {
    std::array<double, 3> xyz = {
        goal->position.x, goal->position.y, goal->position.z};
    std::optional<std::array<double, 3>> rpy;
    if (!std::isnan(goal->orientation.x) ||
        !std::isnan(goal->orientation.y) ||
        !std::isnan(goal->orientation.z)) {
      rpy = std::array<double, 3>{
          goal->orientation.x, goal->orientation.y, goal->orientation.z};
    }

    controller->moveL(xyz, rpy, goal->finger_width, false);

    // 等待完成并发布 Feedback
    auto feedback_fn = [this]() -> MoveLAction::Feedback {
      MoveLAction::Feedback fb;
      auto& gen = node_->get_bridge()->setpoint_generator();
      if (gen.is_active()) {
        auto sp = gen.tick(std::chrono::steady_clock::now());
        fb.progress = sp.progress;
        fb.estimated_time_remaining = sp.time_remaining;
      } else {
        fb.progress = 1.0;
        fb.estimated_time_remaining = 0.0;
      }
      auto joints = node_->get_controller()->get_joint_angles();
      std::copy_n(joints.begin(),
                   std::min(joints.size(), size_t{7}),
                   fb.current_joint_angles.begin());
      return fb;
    };

    wait_and_publish_feedback<MoveLAction::Feedback>(gh, std::move(feedback_fn));

    if (cancel_requested_) {
      auto result = std::make_shared<MoveLAction::Result>();
      result->success = false;
      result->message = "Canceled";
      auto joints = controller->get_joint_angles();
      std::copy_n(joints.begin(),
                   std::min(joints.size(), size_t{7}),
                   result->final_joint_angles.begin());
      auto pose = controller->get_end_effector_pose();
      std::copy_n(pose.begin(), 6, result->final_tcp_pose.begin());
      gh->canceled(result);
      LOG_INFO("MoveL action canceled");
      return;
    }

    success = true;
    message = "MoveL completed";
  } catch (const std::exception& e) {
    success = false;
    message = std::string("MoveL failed: ") + e.what();
    LOG_ERROR("MoveL action failed: {}", e.what());
  }

  auto result = std::make_shared<MoveLAction::Result>();
  result->success = success;
  result->message = message;
  auto joints = controller->get_joint_angles();
  std::copy_n(joints.begin(),
               std::min(joints.size(), size_t{7}),
               result->final_joint_angles.begin());
  auto pose = controller->get_end_effector_pose();
  std::copy_n(pose.begin(), 6, result->final_tcp_pose.begin());

  if (success) {
    gh->succeed(result);
    LOG_INFO("MoveL action succeeded");
  } else {
    gh->abort(result);
  }
}

// ============================================================================
// GoHome
// ============================================================================

rclcpp_action::GoalResponse ActionHandlers::handle_gohome_goal(
    const rclcpp_action::GoalUUID& /*uuid*/,
    std::shared_ptr<const GoHomeAction::Goal> goal) {
  if (!validate_goal(goal->session_id)) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ActionHandlers::handle_gohome_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<GoHomeAction>> /*gh*/) {
  cancel_requested_ = true;
  LOG_INFO("GoHome cancel requested");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ActionHandlers::handle_gohome_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<GoHomeAction>> gh) {
  if (exec_thread_.joinable()) {
    exec_thread_.join();
  }
  cancel_requested_ = false;
  exec_thread_ = std::thread(&ActionHandlers::execute_gohome, this, gh);
}

void ActionHandlers::execute_gohome(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<GoHomeAction>> gh) {
  auto goal = gh->get_goal();
  auto controller = node_->get_controller();
  bool success = false;
  std::string message;

  try {
    controller->go_home(false);

    // 等待完成并发布 Feedback
    auto feedback_fn = [this]() -> GoHomeAction::Feedback {
      GoHomeAction::Feedback fb;
      auto& gen = node_->get_bridge()->setpoint_generator();
      if (gen.is_active()) {
        auto sp = gen.tick(std::chrono::steady_clock::now());
        fb.progress = sp.progress;
        fb.estimated_time_remaining = sp.time_remaining;
      } else {
        fb.progress = 1.0;
        fb.estimated_time_remaining = 0.0;
      }
      auto joints = node_->get_controller()->get_joint_angles();
      std::copy_n(joints.begin(),
                   std::min(joints.size(), size_t{7}),
                   fb.current_joint_angles.begin());
      return fb;
    };

    wait_and_publish_feedback<GoHomeAction::Feedback>(gh, std::move(feedback_fn));

    if (cancel_requested_) {
      auto result = std::make_shared<GoHomeAction::Result>();
      result->success = false;
      result->message = "Canceled";
      auto joints = controller->get_joint_angles();
      std::copy_n(joints.begin(),
                   std::min(joints.size(), size_t{7}),
                   result->final_joint_angles.begin());
      auto pose = controller->get_end_effector_pose();
      std::copy_n(pose.begin(), 6, result->final_tcp_pose.begin());
      gh->canceled(result);
      LOG_INFO("GoHome action canceled");
      return;
    }

    success = true;
    message = "GoHome completed";
  } catch (const std::exception& e) {
    success = false;
    message = std::string("GoHome failed: ") + e.what();
    LOG_ERROR("GoHome action failed: {}", e.what());
  }

  auto result = std::make_shared<GoHomeAction::Result>();
  result->success = success;
  result->message = message;
  auto joints = controller->get_joint_angles();
  std::copy_n(joints.begin(),
               std::min(joints.size(), size_t{7}),
               result->final_joint_angles.begin());
  auto pose = controller->get_end_effector_pose();
  std::copy_n(pose.begin(), 6, result->final_tcp_pose.begin());

  if (success) {
    gh->succeed(result);
    LOG_INFO("GoHome action succeeded");
  } else {
    gh->abort(result);
  }
}

// ============================================================================
// Template: wait_and_publish_feedback
// ============================================================================

template <typename FeedbackT, typename GoalHandleT>
void ActionHandlers::wait_and_publish_feedback(
    std::shared_ptr<GoalHandleT> gh,
    std::function<FeedbackT()>&& get_feedback) {
  auto& gen = node_->get_bridge()->setpoint_generator();
  auto start = std::chrono::steady_clock::now();
  constexpr auto feedback_interval = std::chrono::milliseconds(100);  // 10Hz
  auto last_feedback_time = std::chrono::steady_clock::now();

  while (rclcpp::ok() && !cancel_requested_) {
    // 检查轨迹是否完成（由 100Hz 控制循环驱动）
    if (!gen.is_active()) {
      break;
    }

    // 检查客户端是否请求取消
    if (gh->is_canceling()) {
      cancel_requested_ = true;
      // 触发平滑减速（与 STOP 命令相同）
      gen.cancel();
      break;
    }

    // 周期发布 Feedback
    auto now = std::chrono::steady_clock::now();
    if (now - last_feedback_time >= feedback_interval) {
      gh->publish_feedback(std::make_shared<FeedbackT>(get_feedback()));
      last_feedback_time = now;
    }

    // 超时检测
    double elapsed = std::chrono::duration<double>(now - start).count();
    if (elapsed > ControlConstants::kTrajectoryTimeout) {
      LOG_WARN("Action execution timed out after {:.1f}s", elapsed);
      cancel_requested_ = true;
      gen.cancel();
      break;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  // 等待轨迹播放完毕（减速停止阶段）
  if (cancel_requested_ || !gen.is_active()) {
    auto wait_start = std::chrono::steady_clock::now();
    while (gen.is_active()) {
      auto elapsed = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - wait_start).count();
      if (elapsed > 2.0) {
        gen.cancel();
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }
}

}  // namespace robot_control
