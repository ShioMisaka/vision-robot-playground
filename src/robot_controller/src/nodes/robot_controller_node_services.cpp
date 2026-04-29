#include "robot_controller/nodes/robot_controller_node.hpp"
#include "robot_controller/kinematics/ik_solver.hpp"
#include "robot_controller/motion/control_constants.hpp"
#include "robot_controller/nodes/robot_state.hpp"

#include <robot_msgs/srv/set_tcp.hpp>
#include <robot_msgs/srv/set_speed_ratio.hpp>
#include <robot_msgs/srv/robot_cmd.hpp>

#include <chrono>
#include <string>
#include <vector>

namespace robot_control {

// ===== Service Callbacks =====

void RobotControllerNode::handle_solve_ik(
    const std::shared_ptr<robot_msgs::srv::SolveIK::Request> req,
    std::shared_ptr<robot_msgs::srv::SolveIK::Response> res) {
  if (req->xyz.size() != 3) {
    res->success = false;
    res->message = "xyz must have exactly 3 elements";
    return;
  }

  std::array<double, 3> xyz{req->xyz[0], req->xyz[1], req->xyz[2]};
  std::optional<std::array<double, 3>> rpy;

  if (req->rpy.size() == 3) {
    rpy = std::array<double, 3>{req->rpy[0], req->rpy[1], req->rpy[2]};
  }

  auto result = ik_->solve(xyz, rpy);
  if (result) {
    res->success = true;
    res->joint_angles = *result;
    res->message = "IK solved";
  } else {
    res->success = false;
    res->message = "IK solution not found";
  }
}

void RobotControllerNode::handle_move_joint(
    const std::shared_ptr<robot_msgs::srv::MoveJoint::Request> req,
    std::shared_ptr<robot_msgs::srv::MoveJoint::Response> res) {
  try {
    controller_->moveJ(req->joint_angles, req->block);
    res->success = true;
    res->message = "moveJ completed";
  } catch (const std::exception& e) {
    res->success = false;
    res->message = std::string("moveJ failed: ") + e.what();
  }
}

void RobotControllerNode::handle_move_pose(
    const std::shared_ptr<robot_msgs::srv::MovePose::Request> req,
    std::shared_ptr<robot_msgs::srv::MovePose::Response> res) {
  if (req->xyz.size() != 3) {
    res->success = false;
    res->message = "xyz must have exactly 3 elements";
    return;
  }

  std::array<double, 3> xyz{req->xyz[0], req->xyz[1], req->xyz[2]};
  std::optional<std::array<double, 3>> rpy;

  if (req->rpy.size() == 3) {
    rpy = std::array<double, 3>{req->rpy[0], req->rpy[1], req->rpy[2]};
  }

  try {
    if (req->mode == 1) {
      controller_->moveL(xyz, rpy, req->finger);
    } else {
      controller_->moveJ(xyz, rpy, req->finger);
    }
    res->success = true;
    res->message = (req->mode == 1) ? "moveL completed" : "moveJ completed";
  } catch (const std::exception& e) {
    res->success = false;
    res->message = std::string("move_pose failed: ") + e.what();
  }
}

void RobotControllerNode::handle_move_linear(
    const std::shared_ptr<robot_msgs::srv::MoveLinear::Request> req,
    std::shared_ptr<robot_msgs::srv::MoveLinear::Response> res) {
  if (req->delta.size() != 3) {
    res->success = false;
    res->message = "delta must have exactly 3 elements";
    return;
  }

  std::array<double, 3> delta{req->delta[0], req->delta[1], req->delta[2]};

  try {
    controller_->move_linear(delta, req->frame, req->finger);
    res->success = true;
    res->message = "move_linear completed";
  } catch (const std::exception& e) {
    res->success = false;
    res->message = std::string("move_linear failed: ") + e.what();
  }
}

void RobotControllerNode::handle_control_gripper(
    const std::shared_ptr<robot_msgs::srv::ControlGripper::Request> req,
    std::shared_ptr<robot_msgs::srv::ControlGripper::Response> res) {
  try {
    switch (req->command) {
      case 0:
        controller_->open_gripper();
        res->message = "gripper opened";
        break;
      case 1:
        controller_->close_gripper();
        res->message = "gripper closed";
        break;
      case 2:
        controller_->set_gripper(req->width);
        res->message = "gripper set to " + std::to_string(req->width);
        break;
      default:
        res->success = false;
        res->message = "invalid command: use 0=open, 1=close, 2=set_width";
        return;
    }
    res->success = true;
  } catch (const std::exception& e) {
    res->success = false;
    res->message = std::string("gripper failed: ") + e.what();
  }
}

void RobotControllerNode::handle_go_home(
    const std::shared_ptr<robot_msgs::srv::GoHome::Request> /*req*/,
    std::shared_ptr<robot_msgs::srv::GoHome::Response> res) {
  try {
    controller_->go_home();
    res->success = true;
    res->message = "go_home completed";
  } catch (const std::exception& e) {
    res->success = false;
    res->message = std::string("go_home failed: ") + e.what();
  }
}

void RobotControllerNode::handle_set_speed(
    const std::shared_ptr<robot_msgs::srv::SetSpeed::Request> req,
    std::shared_ptr<robot_msgs::srv::SetSpeed::Response> res) {
  auto mode = (req->mode == 1) ? MotionMode::kMoveL : MotionMode::kMoveJ;
  controller_->set_speed(mode, req->percent);
  res->success = true;
  res->message = "speed set";
}

void RobotControllerNode::handle_get_state(
    const std::shared_ptr<robot_msgs::srv::GetRobotState::Request> /*req*/,
    std::shared_ptr<robot_msgs::srv::GetRobotState::Response> res) {
  try {
    res->joint_angles = controller_->get_joint_angles();
    auto pose = controller_->get_end_effector_pose();
    res->tcp_pose = {pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]};
    res->finger_width = controller_->get_finger_width();
    res->tcp_name = controller_->get_current_tcp();
    res->success = true;
    res->message = "ok";
  } catch (const std::exception& e) {
    res->success = false;
    res->message = std::string("get_state failed: ") + e.what();
  }
}

// ===== Pendant Service Callbacks =====

void RobotControllerNode::handle_pendant_set_tcp(
    const std::shared_ptr<robot_msgs::srv::SetTCP::Request> req,
    std::shared_ptr<robot_msgs::srv::SetTCP::Response> res) {
  try {
    controller_->set_tcp(req->name);
    res->success = true;
    res->message = "TCP set to " + req->name;
  } catch (const std::exception& e) {
    res->success = false;
    res->message = std::string("SetTCP failed: ") + e.what();
  }
}

void RobotControllerNode::handle_set_speed_ratio(
    const std::shared_ptr<robot_msgs::srv::SetSpeedRatio::Request> req,
    std::shared_ptr<robot_msgs::srv::SetSpeedRatio::Response> res) {
  if (req->ratio < 0.0 || req->ratio > 1.0) {
    res->success = false;
    res->message = "ratio must be in [0.0, 1.0]";
    return;
  }
  global_speed_ratio_ = req->ratio;
  res->success = true;
  res->message = "global speed ratio set to " + std::to_string(req->ratio);
}

void RobotControllerNode::handle_robot_cmd(
    const std::shared_ptr<robot_msgs::srv::RobotCmd::Request> req,
    std::shared_ptr<robot_msgs::srv::RobotCmd::Response> res) {
  switch (req->command) {
    case robot_msgs::srv::RobotCmd::Request::STOP:
      if (state_machine_.state() == RobotState::kMoving ||
          state_machine_.state() == RobotState::kTeaching) {
        bridge_->setpoint_generator().cancel();
        if (jog_controller_ && jog_controller_->is_active()) {
          jog_controller_->stop();
        }
        state_machine_.transition_to(RobotState::kStopping);
        {
          stopping_start_time_ = std::chrono::steady_clock::now();
        }
        motion_owner_.store(MotionOwner::kNone);
        res->success = true;
        res->message = "STOP executed";
      } else if (state_machine_.state() == RobotState::kStopping) {
        res->success = true;
        res->message = "STOP: already stopping";
      } else {
        res->success = true;
        res->message = "STOP: no motion to stop";
      }
      break;

    case robot_msgs::srv::RobotCmd::Request::EMERGENCY_STOP:
      emergency_stop();
      res->success = true;
      res->message = "EMERGENCY_STOP executed";
      break;

    case robot_msgs::srv::RobotCmd::Request::CLEAR_FAULT:
      if (state_machine_.state() == RobotState::kFault) {
        state_machine_.clear_error();
        state_model_.align_target_to_actual();
        state_machine_.transition_to(RobotState::kIdle);
        motion_owner_.store(MotionOwner::kNone);
        res->success = true;
        res->message = "FAULT cleared";
      } else {
        res->success = false;
        res->message = "CLEAR_FAULT: robot not in FAULT state";
      }
      break;

    default:
      res->success = false;
      res->message = "Unknown command: " + std::to_string(req->command);
      break;
  }
}

}  // namespace robot_control
