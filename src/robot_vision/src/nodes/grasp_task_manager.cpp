#include "robot_vision/nodes/grasp_task_manager.hpp"

#include <Eigen/Geometry>
#include <rclcpp/logging.hpp>

#include <chrono>
#include <cmath>
#include <thread>
#include <vector>

namespace robot_control {

GraspTaskManager::GraspTaskManager(
    std::shared_ptr<IRobotController> robot,
    std::shared_ptr<IVisionProcessor> vision,
    const std::string& base_frame,
    const std::string& camera_frame,
    double approach_height, double grasp_height_offset,
    const std::array<double, 3>& grasp_rpy)
    : robot_(std::move(robot)),
      vision_(std::move(vision)),
      base_frame_(base_frame),
      camera_frame_(camera_frame),
      approach_height_(approach_height),
      grasp_height_offset_(grasp_height_offset),
      grasp_rpy_(grasp_rpy) {}

bool GraspTaskManager::run(double timeout) {
  auto start = std::chrono::steady_clock::now();
  state_ = GraspState::kDetecting;

  try {
    while (state_ != GraspState::kDone && state_ != GraspState::kError) {
      auto elapsed = std::chrono::duration<double>(
                         std::chrono::steady_clock::now() - start)
                         .count();
      if (elapsed > timeout) {
        state_ = GraspState::kError;
        return false;
      }

      switch (state_) {
        case GraspState::kDetecting:
          if (!step_detect()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            continue;
          }
          break;
        case GraspState::kApproaching:
          step_approach();
          break;
        case GraspState::kDescending:
          step_descend();
          break;
        case GraspState::kGrasping:
          step_grasp();
          break;
        case GraspState::kLifting:
          step_lift();
          break;
        default:
          state_ = GraspState::kError;
          break;
      }
    }
  } catch (const std::exception& e) {
    state_ = GraspState::kError;
    return false;
  }

  return state_ == GraspState::kDone;
}

bool GraspTaskManager::step_detect() {
  auto result = vision_->get_latest_result();
  if (!result.has_value() || !result->detected) {
    return false;
  }

  // 必须成功转换到基座坐标系，否则拒绝抓取（避免碰撞风险）
  auto base_xyz = transform_to_base(result->xyz);
  if (!base_xyz.has_value()) {
    RCLCPP_WARN(rclcpp::get_logger("grasp_task_manager"),
                "TF lookup failed, cannot determine grasp target in base frame");
    return false;
  }

  target_xyz_ = *base_xyz;
  state_ = GraspState::kApproaching;
  return true;
}

void GraspTaskManager::step_approach() {
  if (!target_xyz_.has_value()) {
    state_ = GraspState::kError;
    return;
  }
  auto target = *target_xyz_;
  target[2] += approach_height_;

  robot_->move_to_pose(target, grasp_rpy_, -1.0, 0, 0.08, true);

  state_ = GraspState::kDescending;
}

void GraspTaskManager::step_descend() {
  if (!target_xyz_.has_value()) {
    state_ = GraspState::kError;
    return;
  }
  auto target = *target_xyz_;
  target[2] += grasp_height_offset_;

  robot_->move_to_pose(target, grasp_rpy_, -1.0, 0, 0.08, true);

  state_ = GraspState::kGrasping;
}

void GraspTaskManager::step_grasp() {
  robot_->close_gripper(true);
  state_ = GraspState::kLifting;
}

void GraspTaskManager::step_lift() {
  double lift_height = approach_height_ + 0.1;
  robot_->move_linear({0.0, 0.0, lift_height}, "base", -1.0, true);

  state_ = GraspState::kDone;
}

std::optional<std::array<double, 3>> GraspTaskManager::transform_to_base(
    const Eigen::Vector3d& camera_xyz) {
  auto tf = robot_->lookup_transform(
      base_frame_, camera_frame_, 1.0);
  if (!tf.has_value()) {
    return std::nullopt;
  }

  // tf = {x, y, z, roll, pitch, yaw}
  Eigen::AngleAxisd roll((*tf)[3], Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitch((*tf)[4], Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yaw((*tf)[5], Eigen::Vector3d::UnitZ());
  Eigen::Matrix3d R = (yaw * pitch * roll).toRotationMatrix();

  Eigen::Vector3d t((*tf)[0], (*tf)[1], (*tf)[2]);
  Eigen::Vector3d base_point = R * camera_xyz + t;

  return std::array<double, 3>{base_point.x(), base_point.y(),
                                base_point.z()};
}

}  // namespace robot_control
