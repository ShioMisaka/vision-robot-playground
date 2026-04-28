#include "robot_controller/nodes/robot_controller_node.hpp"
#include "robot_controller/kinematics/ik_solver.hpp"
#include "robot_controller/motion/topic_config.hpp"
#include "robot_controller/nodes/robot_state.hpp"
#include "robot_controller/nodes/robot_state_model.hpp"
#include "robot_controller/motion/control_constants.hpp"
#include "robot_controller/motion/jog_controller.hpp"

#include <robot_msgs/srv/set_tcp.hpp>
#include <robot_msgs/srv/set_speed_ratio.hpp>
#include <robot_msgs/srv/robot_cmd.hpp>
#include <robot_msgs/msg/robot_status.hpp>
#include <robot_msgs/msg/jog_command.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <string>
#include <vector>

namespace robot_control {

// ===== RobotControllerNode =====

std::shared_ptr<RobotControllerNode> RobotControllerNode::create(
    const RobotProfile& profile,
    const GripperProfile& gripper,
    const TopicConfig& topics) {
  auto node = std::shared_ptr<RobotControllerNode>(
      new RobotControllerNode(profile, gripper, topics));
  node->init();
  return node;
}

RobotControllerNode::RobotControllerNode(const RobotProfile& profile,
                                         const GripperProfile& gripper,
                                         const TopicConfig& topics)
    : Node("robot_controller_node"),
      state_model_(profile.dof),
      profile_(profile),
      gripper_(gripper),
      topics_(topics) {
  state_cbg_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
  pub_cbg_ = create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);

  ik_ = std::make_shared<IKSolver>(profile);
}

void RobotControllerNode::init() {
  bridge_ = std::make_shared<RosMotionBridge>(
      shared_from_this(), topics_, ik_, profile_, gripper_);

  controller_ = std::make_shared<RobotMotionController>(
      ik_, profile_, gripper_, bridge_);

  bridge_->set_on_trajectory_started([this]() {
    auto s = state_machine_.state();
    if (s == RobotState::kTeaching) {
      // 从 jog 模式强制切回轨迹模式（可能由残留 jog 消息触发）
      RCLCPP_WARN(this->get_logger(),
                  "Trajectory started while in TEACHING state, forcing to IDLE");
      if (jog_controller_ && jog_controller_->is_active()) {
        auto actual_j = state_model_.get_actual_joints();
        std::array<double, 7> fb{};
        for (size_t i = 0; i < 7 && i < actual_j.size(); ++i) fb[i] = actual_j[i];
        jog_controller_->emergency_stop(fb);
      }
      jog_settling_ = false;
      state_machine_.transition_to(RobotState::kIdle);
      s = RobotState::kIdle;
    }
    if (s == RobotState::kIdle) {
      state_machine_.transition_to(RobotState::kMoving);
    }
  });

  // === 100Hz 控制循环 ===
  control_loop_timer_ = create_wall_timer(
      std::chrono::microseconds(
          static_cast<int64_t>(1e6 / ControlConstants::kControlLoopHz)),
      [this]() { control_loop_tick(); }, pub_cbg_);

  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = state_cbg_;

  joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      topics_.joint_state, 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        bridge_->update_joint_state(msg);
        {
          std::lock_guard<std::mutex> lock(ready_mutex_);
          ready_ = true;
        }
        ready_cv_.notify_all();
      },
      sub_opts);

  // Service servers
  srv_ik_ = create_service<robot_msgs::srv::SolveIK>(
      "~/solve_ik",
      [this](const std::shared_ptr<robot_msgs::srv::SolveIK::Request> req,
             std::shared_ptr<robot_msgs::srv::SolveIK::Response> res) {
        handle_solve_ik(req, res);
      });

  srv_move_joint_ = create_service<robot_msgs::srv::MoveJoint>(
      "~/move_joint",
      [this](const std::shared_ptr<robot_msgs::srv::MoveJoint::Request> req,
             std::shared_ptr<robot_msgs::srv::MoveJoint::Response> res) {
        handle_move_joint(req, res);
      });

  srv_move_pose_ = create_service<robot_msgs::srv::MovePose>(
      "~/move_pose",
      [this](const std::shared_ptr<robot_msgs::srv::MovePose::Request> req,
             std::shared_ptr<robot_msgs::srv::MovePose::Response> res) {
        handle_move_pose(req, res);
      });

  srv_move_linear_ = create_service<robot_msgs::srv::MoveLinear>(
      "~/move_linear",
      [this](const std::shared_ptr<robot_msgs::srv::MoveLinear::Request> req,
             std::shared_ptr<robot_msgs::srv::MoveLinear::Response> res) {
        handle_move_linear(req, res);
      });

  srv_gripper_ = create_service<robot_msgs::srv::ControlGripper>(
      "~/control_gripper",
      [this](const std::shared_ptr<robot_msgs::srv::ControlGripper::Request> req,
             std::shared_ptr<robot_msgs::srv::ControlGripper::Response> res) {
        handle_control_gripper(req, res);
      });

  srv_home_ = create_service<robot_msgs::srv::GoHome>(
      "~/go_home",
      [this](const std::shared_ptr<robot_msgs::srv::GoHome::Request> req,
             std::shared_ptr<robot_msgs::srv::GoHome::Response> res) {
        handle_go_home(req, res);
      });

  srv_speed_ = create_service<robot_msgs::srv::SetSpeed>(
      "~/set_speed",
      [this](const std::shared_ptr<robot_msgs::srv::SetSpeed::Request> req,
             std::shared_ptr<robot_msgs::srv::SetSpeed::Response> res) {
        handle_set_speed(req, res);
      });

  srv_state_ = create_service<robot_msgs::srv::GetRobotState>(
      "~/get_state",
      [this](const std::shared_ptr<robot_msgs::srv::GetRobotState::Request> req,
             std::shared_ptr<robot_msgs::srv::GetRobotState::Response> res) {
        handle_get_state(req, res);
      });

  RCLCPP_INFO(this->get_logger(), "RobotControllerNode started (with services)");

  // === Pendant service servers ===
  pendant_set_tcp_srv_ = create_service<robot_msgs::srv::SetTCP>(
      "~/set_tcp",
      [this](const std::shared_ptr<robot_msgs::srv::SetTCP::Request> req,
             std::shared_ptr<robot_msgs::srv::SetTCP::Response> res) {
        handle_pendant_set_tcp(req, res);
      });

  set_speed_ratio_srv_ = create_service<robot_msgs::srv::SetSpeedRatio>(
      "~/set_speed_ratio",
      [this](const std::shared_ptr<robot_msgs::srv::SetSpeedRatio::Request> req,
             std::shared_ptr<robot_msgs::srv::SetSpeedRatio::Response> res) {
        handle_set_speed_ratio(req, res);
      });

  robot_cmd_srv_ = create_service<robot_msgs::srv::RobotCmd>(
      "~/robot_cmd",
      [this](const std::shared_ptr<robot_msgs::srv::RobotCmd::Request> req,
             std::shared_ptr<robot_msgs::srv::RobotCmd::Response> res) {
        handle_robot_cmd(req, res);
      });

  // === External joint target subscription (from pendant joint stream) ===
  rclcpp::SubscriptionOptions ext_sub_opts;
  ext_sub_opts.callback_group = state_cbg_;
  external_joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "~/joint_target", 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(external_target_mutex_);
        external_joint_target_.assign(
            msg->position.begin(), msg->position.end());
        external_target_time_ = this->now();
      }, ext_sub_opts);

  // === Jog subscription ===
  rclcpp::SubscriptionOptions jog_opts;
  jog_opts.callback_group = state_cbg_;
  jog_sub_ = create_subscription<robot_msgs::msg::JogCommand>(
      "~/jog_command", rclcpp::SensorDataQoS(),
      [this](const robot_msgs::msg::JogCommand::SharedPtr msg) {
        handle_jog_command(msg);
      }, jog_opts);

  // === Jog watchdog (200ms) ===
  jog_watchdog_timer_ = create_wall_timer(
      std::chrono::milliseconds(200),
      [this]() { jog_watchdog_callback(); }, pub_cbg_);

  // === Jog controller (pure C++ math) ===
  JogConfig jog_cfg;
  jog_cfg.dof = profile_.dof;
  jog_cfg.dt = ControlConstants::kControlLoopDt;
  jog_controller_ = std::make_unique<JogController>(ik_, jog_cfg);

  // === Status publisher (10Hz) ===
  status_pub_ = create_publisher<robot_msgs::msg::RobotStatus>(
      "~/status", 10);
  status_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() { publish_status(); }, pub_cbg_);

  RCLCPP_INFO(this->get_logger(),
              "RobotControllerNode: pendant interface ready (jog, status)");
}

bool RobotControllerNode::wait_for_ready(double timeout) {
  std::unique_lock<std::mutex> lock(ready_mutex_);
  return ready_cv_.wait_for(
      lock, std::chrono::duration<double>(timeout),
      [this] { return ready_; });
}

// ===== Jog + Watchdog =====

void RobotControllerNode::handle_jog_command(
    const robot_msgs::msg::JogCommand::SharedPtr msg) {
  auto state = state_machine_.state();
  if (state != RobotState::kIdle && state != RobotState::kTeaching) {
    return;
  }

  RCLCPP_WARN(this->get_logger(),
              "[DIAG] Jog command received in state %s (vel=[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f])",
              RobotStateMachine::state_name(state),
              msg->velocity[0], msg->velocity[1], msg->velocity[2],
              msg->velocity[3], msg->velocity[4], msg->velocity[5]);

  // 检测零速度（停止指令）
  bool all_zero = true;
  for (int i = 0; i < 6; ++i) {
    if (std::abs(msg->velocity[i]) > 1e-10) {
      all_zero = false;
      break;
    }
  }

  if (all_zero) {
    // 按钮释放: 进入减速
    if (jog_controller_->is_active()) {
      jog_controller_->stop();
    }
  } else {
    // 按钮按下（或方向改变）: 仅在速度/坐标系变化时重启 Jog
    std::array<double, 6> vel;
    std::copy(msg->velocity.begin(), msg->velocity.end(), vel.begin());

    bool need_restart = !jog_controller_->is_active() ||
                        jog_controller_->is_stopping();
    // 检查速度是否变化
    if (!need_restart) {
      auto current_target = jog_controller_->get_target_velocity();
      for (int i = 0; i < 6; ++i) {
        if (std::abs(vel[i] - current_target[i]) > 1e-6) {
          need_restart = true;
          break;
        }
      }
    }

    if (need_restart) {
      if (jog_controller_->is_stopping()) {
        jog_controller_->reset();
        jog_settling_ = false;
      }
      jog_controller_->start_raw(vel, msg->frame);
    }
  }

  if (state == RobotState::kIdle) {
    state_machine_.transition_to(RobotState::kTeaching);
  }

  last_jog_time_ = this->now();
}

void RobotControllerNode::jog_watchdog_callback() {
  if (state_machine_.state() != RobotState::kTeaching) return;

  // Jog 正在减速（stop 已调用）或 settling 中，允许自然完成，不触发看门狗
  if (jog_settling_ ||
      (jog_controller_ && jog_controller_->is_stopping())) {
    return;
  }

  auto elapsed = (this->now() - last_jog_time_).seconds();
  if (elapsed > 0.2) {
    RCLCPP_WARN(this->get_logger(), "Jog watchdog: no command for %.2fs, stopping", elapsed);
    auto actual = state_model_.get_actual_joints();
    std::array<double, 7> fb{};
    for (int i = 0; i < 7 && i < static_cast<int>(actual.size()); ++i) {
      fb[i] = actual[i];
    }
    jog_controller_->emergency_stop(fb);
    jog_settling_ = false;
    state_machine_.transition_to(RobotState::kIdle);
  }
}

// ===== Emergency Stop =====

void RobotControllerNode::emergency_stop() {
  bridge_->setpoint_generator().cancel();
  jog_settling_ = false;

  if (jog_controller_ && jog_controller_->is_active()) {
    auto actual = state_model_.get_actual_joints();
    std::array<double, 7> fb{};
    for (int i = 0; i < 7 && i < static_cast<int>(actual.size()); ++i) {
      fb[i] = actual[i];
    }
    jog_controller_->emergency_stop(fb);
  }

  waiting_settle_ = false;

  state_model_.align_target_to_actual();
  state_machine_.force_state(RobotState::kFault);
  state_machine_.set_error(100, "EMERGENCY_STOP activated");
  RCLCPP_ERROR(this->get_logger(), "EMERGENCY_STOP activated");
}

// ===== Status Publisher =====

void RobotControllerNode::publish_status() {
  auto msg = std::make_unique<robot_msgs::msg::RobotStatus>();
  msg->state = static_cast<uint8_t>(state_machine_.state());
  msg->speed_ratio = global_speed_ratio_;
  msg->error_code = state_machine_.error_code();
  msg->error_message = state_machine_.error_message();

  auto angles = controller_->get_joint_angles();
  std::copy_n(angles.begin(), 7, msg->joint_angles.begin());

  auto pose = controller_->get_end_effector_pose();
  msg->tcp_pose = {pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]};

  msg->finger_width = controller_->get_finger_width();
  msg->tcp_name = controller_->get_current_tcp();
  {
    std::lock_guard<std::mutex> lock(ready_mutex_);
    msg->is_connected = ready_;
  }

  status_pub_->publish(std::move(msg));
}

// ===== 100Hz Control Loop =====

void RobotControllerNode::control_loop_tick() {
  if (shutdown_.load()) return;

  // === 1. READ ===
  auto actual = bridge_->get_current_arm();
  double actual_finger = bridge_->get_current_finger();
  state_model_.update_actual(actual, actual_finger);

  // === 2. PLAN ===
  auto state = state_machine_.state();
  auto target = state_model_.get_target_joints();
  double target_finger = state_model_.get_target_gripper();

  switch (state) {
    case RobotState::kMoving: {
      auto& gen = bridge_->setpoint_generator();
      if (gen.is_active()) {
        auto sp = gen.tick(std::chrono::steady_clock::now());
        target = sp.joint_positions;
        target_finger = sp.finger_width;

        if (sp.done) {
          state_machine_.transition_to(RobotState::kIdle);
          // Notify Python/blocking waiters (prevent deadlock)
          bridge_->notify_trajectory_complete();
        }
      }
      break;
    }
    case RobotState::kTeaching: {
      if (jog_controller_ && jog_controller_->is_active()) {
        std::array<double, 7> fb{};
        auto actual_j = state_model_.get_actual_joints();
        for (size_t i = 0; i < 7 && i < actual_j.size(); ++i) fb[i] = actual_j[i];
        double jog_finger = controller_->is_grasping()
                                ? gripper_.min_width
                                : bridge_->get_current_finger();
        jog_controller_->set_finger_width(jog_finger);
        jog_controller_->tick(fb, [](const std::vector<double>&, double) {});
        auto jog_target = jog_controller_->get_commanded_joints();
        target = std::vector<double>(jog_target.begin(), jog_target.end());
        target_finger = jog_finger;

        if (!jog_controller_->is_active()) {
          jog_settling_ = true;
          jog_settle_start_time_ = std::chrono::steady_clock::now();
        }
      }

      // Jog settling: 保持发布最后目标，等机器人追上后再转 kIdle
      if (jog_settling_) {
        bool on_target = state_model_.is_on_target(
            ControlConstants::kArrivalTolerance);
        auto settle_elapsed = std::chrono::steady_clock::now() -
                              jog_settle_start_time_;
        bool timed_out = std::chrono::duration<double>(settle_elapsed).count() >
                         0.5;

        if (on_target || timed_out) {
          target = actual;
          target_finger = actual_finger;
          jog_settling_ = false;
          state_machine_.transition_to(RobotState::kIdle);
        }
      }
      break;
    }
    case RobotState::kStopping: {
      auto& gen = bridge_->setpoint_generator();
      bool settled = !gen.is_active() &&
                     state_model_.is_on_target(ControlConstants::kArrivalTolerance);

      bool timed_out = false;
      auto stop_elapsed = std::chrono::steady_clock::now() - stopping_start_time_;
      if (std::chrono::duration<double>(stop_elapsed).count() >
          ControlConstants::kTrajectoryTimeout) {
        timed_out = true;
      }

      if (settled || timed_out) {
        state_machine_.transition_to(RobotState::kIdle);
        if (timed_out) {
          RCLCPP_WARN(this->get_logger(), "STOP timeout -> IDLE");
        } else {
          RCLCPP_INFO(this->get_logger(), "STOP complete -> IDLE");
        }
      }
      break;
    }
    case RobotState::kIdle:
    case RobotState::kFault:
      {
        std::lock_guard<std::mutex> lock(external_target_mutex_);
        auto elapsed = (this->now() - external_target_time_).seconds();
        if (!external_joint_target_.empty() && elapsed < 0.2) {
          target = external_joint_target_;
        } else {
          target = actual;
        }
      }
      target_finger = controller_->is_grasping()
                          ? gripper_.min_width
                          : bridge_->get_current_finger();
      break;
  }

  state_model_.update_target(target, target_finger);

  // === 3. MONITOR ===
  double error = state_model_.max_following_error();

  if (state == RobotState::kMoving &&
      error > ControlConstants::kFollowingErrorLimit) {
    RCLCPP_ERROR(this->get_logger(),
                 "Following error %.4f rad exceeds limit %.4f rad -- EMERGENCY STOP",
                 error, ControlConstants::kFollowingErrorLimit);
    emergency_stop();
    return;
  }

  if (state == RobotState::kTeaching &&
      error > ControlConstants::kTeachingFollowErrorLimit) {
    RCLCPP_WARN(this->get_logger(),
                "Jog following error %.4f rad exceeds limit %.4f rad -- stopping jog gracefully",
                error, ControlConstants::kTeachingFollowErrorLimit);
    if (jog_controller_ && jog_controller_->is_active()) {
      auto actual_j = state_model_.get_actual_joints();
      std::array<double, 7> fb{};
      for (size_t i = 0; i < 7 && i < actual_j.size(); ++i) fb[i] = actual_j[i];
      jog_controller_->emergency_stop(fb);
    }
    jog_settling_ = false;
    state_model_.align_target_to_actual();
    state_machine_.transition_to(RobotState::kIdle);
    return;
  }

  // === 4. WRITE ===
  if (state == RobotState::kIdle || state == RobotState::kFault) {
    // Arm: publish only when external joint target stream is active
    // (PendantNode joint dragging). Published separately from gripper to
    // avoid overwriting direct API calls like open_gripper()/close_gripper().
    bool has_external = false;
    {
      std::lock_guard<std::mutex> lock(external_target_mutex_);
      has_external = !external_joint_target_.empty() &&
                     (this->now() - external_target_time_).seconds() < 0.2;
    }
    if (has_external) {
      bridge_->publish_arm(target);
    }
    // Gripper: maintain force only when grasping (old behavior).
    // Direct API calls (open/close_gripper) work because the loop does NOT
    // publish gripper when not grasping.
    if (controller_->is_grasping()) {
      bridge_->publish_gripper(target_finger);
    }
  } else {
    bridge_->publish_command(target, target_finger);
  }
}

// ===== Destructor =====

RobotControllerNode::~RobotControllerNode() {
  shutdown_ = true;
  if (jog_controller_) {
    jog_controller_->reset();
  }
  bridge_->cancel_trajectory();
}

}  // namespace robot_control
