#include "robot_controller/motion/robot_motion_controller.hpp"
#include "robot_controller/kinematics/ik_solver.hpp"
#include "robot_controller/motion/control_constants.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <iostream>

namespace robot_control {

RobotMotionController::RobotMotionController(
    std::shared_ptr<IKSolver> ik, const RobotProfile& profile,
    const GripperProfile& gripper,
    std::shared_ptr<MotionIOBridge> bridge)
    : ik_(std::move(ik)),
      profile_(profile),
      gripper_(gripper),
      bridge_(std::move(bridge)),
      current_tcp_(profile.default_tcp),
      current_tcp_config_(profile.tcp_frames.at(profile.default_tcp)) {}

void RobotMotionController::interpolate_to(
    const std::vector<double>& target, double finger, int steps,
    double step_time, bool block) {
  auto current = bridge_->get_current_arm();
  int dof = static_cast<int>(current.size());

  for (int i = 1; i <= steps; ++i) {
    double t = static_cast<double>(i) / steps;
    std::vector<double> interp(dof);
    for (int j = 0; j < dof; ++j) {
      interp[j] = current[j] + t * (target[j] - current[j]);
    }
    // 抓取状态下始终发送目标夹爪宽度以维持夹持力，
    // 非抓取时中间步保持当前夹爪避免误动
    double step_finger;
    if (grasping_) {
      step_finger = finger;
    } else {
      step_finger = (i == steps) ? finger : bridge_->get_current_finger();
    }
    bridge_->publish_command(interp, step_finger);
    std::this_thread::sleep_for(
        std::chrono::duration<double>(step_time));
  }

  if (block) {
    bridge_->wait_for_motion(
        target, finger,
        ControlConstants::kJointTolerance,
        ControlConstants::kFingerTolerance,
        ControlConstants::kMotionTimeout,
        ControlConstants::kPollInterval,
        ControlConstants::kSettleTime,
        !grasping_);
  }
}

void RobotMotionController::set_arm(const std::vector<double>& angles,
                                    bool block) {
  if (static_cast<int>(angles.size()) != profile_.dof) {
    throw std::invalid_argument(
        "set_arm: expected " + std::to_string(profile_.dof) +
        " joint angles, got " + std::to_string(angles.size()));
  }
  // 抓取状态下发送 min_width 以持续施加夹持力，
  // 非抓取时保持当前夹爪位置避免误动
  double finger = grasping_ ? gripper_.min_width
                            : bridge_->get_current_finger();
  bridge_->publish_command(angles, finger);
  if (block) {
    bridge_->wait_for_motion(
        angles, finger,
        ControlConstants::kJointTolerance,
        ControlConstants::kFingerTolerance,
        ControlConstants::kMotionTimeout,
        ControlConstants::kPollInterval,
        ControlConstants::kSettleTime,
        !grasping_);
  }
}

void RobotMotionController::set_gripper(double width, bool block) {
  grasping_ = false;
  auto arm = bridge_->get_current_arm();
  bridge_->publish_command(arm, width);
  if (block) {
    bridge_->wait_for_motion(
        arm, width,
        ControlConstants::kJointTolerance,
        ControlConstants::kFingerTolerance,
        ControlConstants::kMotionTimeout,
        ControlConstants::kPollInterval,
        ControlConstants::kSettleTime,
        true);
  }
}

void RobotMotionController::open_gripper(bool block) {
  set_gripper(gripper_.max_width, block);
}

void RobotMotionController::close_gripper(bool block) {
  auto arm = bridge_->get_current_arm();
  bridge_->publish_command(arm, gripper_.min_width);
  if (block) {
    bridge_->wait_for_finger_settle(
        ControlConstants::kFingerStableCount,
        ControlConstants::kFingerStableTol,
        0.05, 5.0);
  }
  grasping_ = true;
}

void RobotMotionController::move_to_pose(
    const std::array<double, 3>& xyz,
    const std::optional<std::array<double, 3>>& rpy,
    double finger, int steps, double step_time, bool block) {
  auto tcp_offset = tcp_transform_matrix();
  std::vector<double> angles;

  if (rpy.has_value()) {
    // 完整位姿约束：T_hand = T_tcp_target @ inv(T_tcp_in_hand)
    Eigen::Matrix4d target = Eigen::Matrix4d::Identity();
    Eigen::AngleAxisd roll_angle((*rpy)[0], Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch_angle((*rpy)[1], Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw_angle((*rpy)[2], Eigen::Vector3d::UnitZ());
    target.block<3, 3>(0, 0) =
        (yaw_angle * pitch_angle * roll_angle).toRotationMatrix();
    target(0, 3) = xyz[0];
    target(1, 3) = xyz[1];
    target(2, 3) = xyz[2];

    Eigen::Matrix4d hand_target = target * tcp_offset.inverse();
    Eigen::Vector3d hand_xyz = hand_target.block<3, 1>(0, 3);
    Eigen::Matrix3d hand_rot = hand_target.block<3, 3>(0, 0);
    Eigen::Vector3d hand_rpy = hand_rot.eulerAngles(0, 1, 2);

    std::array<double, 3> h_xyz = {hand_xyz.x(), hand_xyz.y(), hand_xyz.z()};
    std::array<double, 3> h_rpy = {hand_rpy.x(), hand_rpy.y(), hand_rpy.z()};
    auto result = ik_->solve(h_xyz, h_rpy);
    if (!result) {
      throw std::runtime_error("IK solver failed for target pose");
    }
    angles = *result;
  } else {
    // 仅位置约束：用当前 hand 姿态近似偏移
    auto current = bridge_->get_current_arm();
    Eigen::Matrix4d hand_matrix = ik_->forward_matrix(current);
    Eigen::Matrix3d R_hand = hand_matrix.block<3, 3>(0, 0);
    Eigen::Vector3d offset_xyz(current_tcp_config_.offset_xyz[0],
                               current_tcp_config_.offset_xyz[1],
                               current_tcp_config_.offset_xyz[2]);
    Eigen::Vector3d hand_xyz =
        Eigen::Vector3d(xyz[0], xyz[1], xyz[2]) - R_hand * offset_xyz;

    std::array<double, 3> h_xyz = {hand_xyz.x(), hand_xyz.y(), hand_xyz.z()};
    auto result = ik_->solve(h_xyz, std::nullopt);
    if (!result) {
      throw std::runtime_error("IK solver failed for target pose");
    }
    angles = *result;
  }

  // 实际 finger：-1 表示保持当前（抓取时保持目标值而非反馈值）
  double actual_finger;
  if (finger < 0) {
    actual_finger = grasping_ ? gripper_.min_width
                              : bridge_->get_current_finger();
  } else {
    actual_finger = finger;
  }

  if (steps > 0) {
    interpolate_to(angles, actual_finger, steps, step_time, block);
  } else {
    bridge_->publish_command(angles, actual_finger);
    if (block) {
      bridge_->wait_for_motion(
          angles, actual_finger,
          ControlConstants::kJointTolerance,
          ControlConstants::kFingerTolerance,
          ControlConstants::kMotionTimeout,
          ControlConstants::kPollInterval,
          ControlConstants::kSettleTime,
          !grasping_);
    }
  }
}

void RobotMotionController::move_linear(const std::array<double, 3>& delta,
                                        const std::string& frame,
                                        double finger, bool block) {
  auto pose = get_end_effector_pose();
  Eigen::Vector3d pos(pose[0], pose[1], pose[2]);
  Eigen::Vector3d rpy(pose[3], pose[4], pose[5]);

  Eigen::Vector3d target_pos = pos;

  if (frame == "end_effector") {
    Eigen::Matrix3d rot =
        (Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX()) *
         Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()))
            .toRotationMatrix();
    Eigen::Vector3d d(delta[0], delta[1], delta[2]);
    target_pos = pos + rot * d;
  } else {
    target_pos = pos + Eigen::Vector3d(delta[0], delta[1], delta[2]);
  }

  std::array<double, 3> target_xyz = {target_pos.x(), target_pos.y(),
                                      target_pos.z()};
  std::array<double, 3> target_rpy = {pose[3], pose[4], pose[5]};

  move_to_pose(target_xyz, target_rpy, finger, 0, 0.08, block);
}

void RobotMotionController::rotate_joint(int index, double delta_angle,
                                         bool block) {
  if (index < 0 || index >= profile_.dof) {
    throw std::invalid_argument(
        "rotate_joint: index must be 0~" + std::to_string(profile_.dof - 1) +
        ", got " + std::to_string(index));
  }
  auto angles = bridge_->get_current_arm();
  angles[index] += delta_angle;
  set_arm(angles, block);
}

void RobotMotionController::go_home(bool block) {
  if (static_cast<int>(profile_.home_joints.size()) < profile_.dof + 2) {
    throw std::runtime_error("go_home: home_joints must have dof+2 values");
  }
  std::vector<double> arm(profile_.home_joints.begin(),
                          profile_.home_joints.begin() + profile_.dof);
  // 抓取状态下持续施加夹持力，非抓取时使用 home 默认夹爪宽度
  double finger = grasping_ ? gripper_.min_width
                            : profile_.home_joints[profile_.dof];
  bridge_->publish_command(arm, finger);
  if (block) {
    bridge_->wait_for_motion(
        arm, finger,
        ControlConstants::kJointTolerance,
        ControlConstants::kFingerTolerance,
        ControlConstants::kMotionTimeout,
        ControlConstants::kPollInterval,
        ControlConstants::kSettleTime,
        !grasping_);
  }
}

std::vector<double> RobotMotionController::get_joint_angles() const {
  return bridge_->get_current_arm();
}

std::array<double, 6> RobotMotionController::get_end_effector_pose() const {
  auto angles = bridge_->get_current_arm();
  Eigen::Matrix4d hand_matrix = ik_->forward_matrix(angles);
  Eigen::Matrix4d tcp_matrix = hand_matrix * tcp_transform_matrix();

  Eigen::Vector3d pos = tcp_matrix.block<3, 1>(0, 3);
  Eigen::Vector3d rpy =
      tcp_matrix.block<3, 3>(0, 0).eulerAngles(0, 1, 2);

  return {pos.x(), pos.y(), pos.z(), rpy.x(), rpy.y(), rpy.z()};
}

double RobotMotionController::get_finger_width() const {
  return bridge_->get_current_finger();
}

void RobotMotionController::set_tcp(const std::string& name) {
  auto it = profile_.tcp_frames.find(name);
  if (it == profile_.tcp_frames.end()) {
    throw std::invalid_argument(
        "Unknown TCP: " + name);
  }
  current_tcp_ = name;
  current_tcp_config_ = it->second;
  bridge_->set_tcp_name(name);
}

std::string RobotMotionController::get_current_tcp() const {
  return current_tcp_;
}

std::optional<std::array<double, 6>> RobotMotionController::lookup_transform(
    const std::string& target_frame, const std::string& source_frame,
    double timeout) {
  return bridge_->lookup_transform(target_frame, source_frame, timeout);
}

Eigen::Matrix4d RobotMotionController::tcp_transform_matrix() const {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  Eigen::AngleAxisd roll_angle(current_tcp_config_.offset_rpy[0],
                               Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitch_angle(current_tcp_config_.offset_rpy[1],
                                Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yaw_angle(current_tcp_config_.offset_rpy[2],
                              Eigen::Vector3d::UnitZ());
  T.block<3, 3>(0, 0) =
      (yaw_angle * pitch_angle * roll_angle).toRotationMatrix();
  T(0, 3) = current_tcp_config_.offset_xyz[0];
  T(1, 3) = current_tcp_config_.offset_xyz[1];
  T(2, 3) = current_tcp_config_.offset_xyz[2];
  return T;
}

void RobotMotionController::set_speed(MotionMode mode, double percent) {
  if (percent <= 0) {
    throw std::invalid_argument("set_speed: percent must be > 0, got " +
                                std::to_string(percent));
  }
  double clamped = std::min(percent, 100.0);
  switch (mode) {
    case MotionMode::kMoveJ: movej_speed_ = clamped; break;
    case MotionMode::kMoveL: movel_speed_ = clamped; break;
    default: break;
  }
}

double RobotMotionController::get_speed(MotionMode mode) const {
  switch (mode) {
    case MotionMode::kMoveJ: return movej_speed_;
    case MotionMode::kMoveL: return movel_speed_;
    default: return 50.0;
  }
}

void RobotMotionController::moveJ_internal(
    const std::vector<double>& target_angles, double finger, bool block) {
  auto current = bridge_->get_current_arm();

  // 检查是否有实际运动量
  double max_delta = 0.0;
  for (int i = 0; i < profile_.dof; ++i) {
    max_delta = std::max(max_delta, std::abs(target_angles[i] - current[i]));
  }
  if (max_delta < 1e-6) {
    return;
  }

  // 使用 S 曲线轨迹规划器，按速度百分比缩放运动极限
  double speed_factor = movej_speed_ / 100.0;
  std::vector<MotionLimits> configs;
  configs.reserve(profile_.dof);
  for (int i = 0; i < profile_.dof; ++i) {
    configs.push_back({
        profile_.joint_limits.max_vel * speed_factor,
        profile_.joint_limits.max_acc * speed_factor,
        profile_.joint_limits.max_jerk * speed_factor});
  }

  auto trajectory = TrajectoryPlanner::plan_joint(
      current, target_angles, configs,
      ControlConstants::kTrajectoryDt);

  // 基于绝对时间调度发送轨迹点，避免 sleep_for 累积误差
  double dt = ControlConstants::kTrajectoryDt;
  auto start = std::chrono::steady_clock::now();
  for (size_t i = 1; i < trajectory.size(); ++i) {
    double t = static_cast<double>(i) * dt;
    auto target_time = start + std::chrono::duration<double>(t);
    std::this_thread::sleep_until(target_time);
    bridge_->publish_command(trajectory[i], finger);
  }

  if (block) {
    bridge_->wait_for_motion(
        target_angles, finger,
        ControlConstants::kJointTolerance,
        ControlConstants::kFingerTolerance,
        ControlConstants::kMotionTimeout,
        ControlConstants::kPollInterval,
        ControlConstants::kSettleTime,
        !grasping_);
  }
}

void RobotMotionController::moveJ(
    const std::vector<double>& target_angles, bool block) {
  if (static_cast<int>(target_angles.size()) != profile_.dof) {
    throw std::invalid_argument(
        "moveJ: expected " + std::to_string(profile_.dof) +
        " joint angles, got " + std::to_string(target_angles.size()));
  }

  double finger = grasping_ ? gripper_.min_width
                            : bridge_->get_current_finger();
  moveJ_internal(target_angles, finger, block);
}

void RobotMotionController::moveJ(
    const std::array<double, 3>& xyz,
    const std::optional<std::array<double, 3>>& rpy,
    double finger, bool block) {
  auto tcp_offset = tcp_transform_matrix();
  std::vector<double> target_angles;

  if (rpy.has_value()) {
    Eigen::Matrix4d target = Eigen::Matrix4d::Identity();
    Eigen::AngleAxisd roll_angle((*rpy)[0], Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch_angle((*rpy)[1], Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw_angle((*rpy)[2], Eigen::Vector3d::UnitZ());
    target.block<3, 3>(0, 0) =
        (yaw_angle * pitch_angle * roll_angle).toRotationMatrix();
    target(0, 3) = xyz[0];
    target(1, 3) = xyz[1];
    target(2, 3) = xyz[2];

    Eigen::Matrix4d hand_target = target * tcp_offset.inverse();
    Eigen::Vector3d hand_xyz = hand_target.block<3, 1>(0, 3);
    Eigen::Vector3d hand_rpy = hand_target.block<3, 3>(0, 0).eulerAngles(0, 1, 2);

    std::array<double, 3> h_xyz = {hand_xyz.x(), hand_xyz.y(), hand_xyz.z()};
    std::array<double, 3> h_rpy = {hand_rpy.x(), hand_rpy.y(), hand_rpy.z()};
    auto result = ik_->solve(h_xyz, h_rpy);
    if (!result) {
      throw std::runtime_error("moveJ(pose): IK solver failed for target pose");
    }
    target_angles = *result;
  } else {
    auto current = bridge_->get_current_arm();
    Eigen::Matrix4d hand_matrix = ik_->forward_matrix(current);
    Eigen::Matrix3d R_hand = hand_matrix.block<3, 3>(0, 0);
    Eigen::Vector3d offset_xyz(current_tcp_config_.offset_xyz[0],
                               current_tcp_config_.offset_xyz[1],
                               current_tcp_config_.offset_xyz[2]);
    Eigen::Vector3d hand_xyz =
        Eigen::Vector3d(xyz[0], xyz[1], xyz[2]) - R_hand * offset_xyz;

    std::array<double, 3> h_xyz = {hand_xyz.x(), hand_xyz.y(), hand_xyz.z()};
    auto result = ik_->solve(h_xyz, std::nullopt);
    if (!result) {
      throw std::runtime_error("moveJ(pose): IK solver failed for target pose");
    }
    target_angles = *result;
  }

  // 求解 finger
  double actual_finger;
  if (finger < 0) {
    actual_finger = grasping_ ? gripper_.min_width
                              : bridge_->get_current_finger();
  } else {
    actual_finger = finger;
  }

  // 用关节空间 S 曲线 moveJ 执行
  moveJ_internal(target_angles, actual_finger, block);
}

void RobotMotionController::moveL(
    const std::array<double, 3>& xyz,
    const std::optional<std::array<double, 3>>& rpy,
    double finger, bool block) {
  auto current_pose = get_end_effector_pose();
  Eigen::Vector3d start_pos(current_pose[0], current_pose[1], current_pose[2]);
  Eigen::Vector3d end_pos(xyz[0], xyz[1], xyz[2]);

  double total_dist = (end_pos - start_pos).norm();
  if (total_dist < 1e-6) {
    return;
  }

  // 用 S 曲线规划器规划笛卡尔距离曲线
  double speed_factor = movel_speed_ / 100.0;
  MotionLimits cart_cfg{
      profile_.cartesian_limits.max_vel * speed_factor,
      profile_.cartesian_limits.max_acc * speed_factor,
      profile_.cartesian_limits.max_jerk * speed_factor};

  auto cart_traj = SCurvePlanner::plan(
      0.0, total_dist, cart_cfg, ControlConstants::kTrajectoryDt);

  // 姿态插值
  Eigen::Vector3d start_rpy(current_pose[3], current_pose[4], current_pose[5]);
  Eigen::Quaterniond start_quat =
      (Eigen::AngleAxisd(start_rpy.x(), Eigen::Vector3d::UnitX()) *
       Eigen::AngleAxisd(start_rpy.y(), Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(start_rpy.z(), Eigen::Vector3d::UnitZ()));

  Eigen::Quaterniond end_quat = start_quat;
  if (rpy.has_value()) {
    end_quat =
        (Eigen::AngleAxisd((*rpy)[0], Eigen::Vector3d::UnitX()) *
         Eigen::AngleAxisd((*rpy)[1], Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd((*rpy)[2], Eigen::Vector3d::UnitZ()));
  }

  double actual_finger;
  if (finger < 0) {
    actual_finger = grasping_ ? gripper_.min_width
                              : bridge_->get_current_finger();
  } else {
    actual_finger = finger;
  }

  auto tcp_offset = tcp_transform_matrix();

  // 预计算所有 IK 解
  std::vector<std::vector<double>> joint_traj;
  joint_traj.reserve(cart_traj.size());

  for (const auto& pt : cart_traj) {
    double alpha = (total_dist > 1e-12) ? pt.pos / total_dist : 1.0;
    alpha = std::clamp(alpha, 0.0, 1.0);

    Eigen::Vector3d interp_pos = start_pos + alpha * (end_pos - start_pos);
    Eigen::Quaterniond interp_quat = start_quat.slerp(alpha, end_quat);

    Eigen::Matrix4d target = Eigen::Matrix4d::Identity();
    target.block<3, 3>(0, 0) = interp_quat.toRotationMatrix();
    target(0, 3) = interp_pos.x();
    target(1, 3) = interp_pos.y();
    target(2, 3) = interp_pos.z();

    Eigen::Matrix4d hand_target = target * tcp_offset.inverse();
    Eigen::Vector3d hand_xyz = hand_target.block<3, 1>(0, 3);
    Eigen::Vector3d hand_rpy = hand_target.block<3, 3>(0, 0).eulerAngles(0, 1, 2);

    std::array<double, 3> h_xyz = {hand_xyz.x(), hand_xyz.y(), hand_xyz.z()};
    std::array<double, 3> h_rpy = {hand_rpy.x(), hand_rpy.y(), hand_rpy.z()};

    auto ik_result = ik_->solve(h_xyz, h_rpy);
    if (!ik_result) {
      throw std::runtime_error("moveL: IK failed at trajectory point");
    }

    joint_traj.push_back(*ik_result);
  }

  // 基于绝对时间调度发送轨迹点
  double dt = ControlConstants::kTrajectoryDt;
  auto start = std::chrono::steady_clock::now();
  for (size_t i = 1; i < joint_traj.size(); ++i) {
    double t = static_cast<double>(i) * dt;
    auto target_time = start + std::chrono::duration<double>(t);
    std::this_thread::sleep_until(target_time);
    bridge_->publish_command(joint_traj[i], actual_finger);
  }

  if (block && !joint_traj.empty()) {
    const auto& final_angles = joint_traj.back();
    bridge_->publish_command(final_angles, actual_finger);
    bridge_->wait_for_motion(
        final_angles, actual_finger,
        ControlConstants::kJointTolerance,
        ControlConstants::kFingerTolerance,
        ControlConstants::kMotionTimeout,
        ControlConstants::kPollInterval,
        ControlConstants::kSettleTime,
        !grasping_);
  }
}

}  // namespace robot_control
