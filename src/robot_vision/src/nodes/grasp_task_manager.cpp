#include "robot_vision/nodes/grasp_task_manager.hpp"
#include "robot_controller/kinematics/robot_profile.hpp"

#include <Eigen/Geometry>
#include <rclcpp/logging.hpp>

#include <chrono>
#include <cmath>
#include <thread>
#include <vector>

namespace robot_vision {

using robot_control::rpy_to_rotation;

namespace {
const char* state_name(GraspState s) {
  switch (s) {
    case GraspState::kIdle:        return "IDLE";
    case GraspState::kDetecting:   return "DETECTING";
    case GraspState::kApproaching: return "APPROACHING";
    case GraspState::kReDetecting: return "RE_DETECTING";
    case GraspState::kDescending:  return "DESCENDING";
    case GraspState::kGrasping:    return "GRASPING";
    case GraspState::kLifting:     return "LIFTING";
    case GraspState::kDone:        return "DONE";
    case GraspState::kError:       return "ERROR";
  }
  return "UNKNOWN";
}
}  // namespace

GraspTaskManager::GraspTaskManager(
    std::shared_ptr<IRobotController> robot,
    std::shared_ptr<IVisionProcessor> vision,
    const std::string& base_frame,
    const std::string& camera_frame,
    double approach_height, double grasp_height_offset,
    const std::array<double, 3>& grasp_rpy,
    int redetect_samples, double redetect_interval,
    double max_reach,
    double approach_step_size, double approach_tolerance,
    int max_approach_steps, int max_consecutive_failures,
    const std::string& hand_frame,
    const std::array<double, 3>& camera_offset,
    const std::array<double, 3>& camera_rpy)
    : robot_(std::move(robot)),
      vision_(std::move(vision)),
      base_frame_(base_frame),
      camera_frame_(camera_frame),
      approach_height_(approach_height),
      grasp_height_offset_(grasp_height_offset),
      grasp_rpy_(grasp_rpy),
      max_reach_(max_reach),
      approach_step_size_(approach_step_size),
      approach_tolerance_(approach_tolerance),
      max_approach_steps_(max_approach_steps),
      max_consecutive_failures_(max_consecutive_failures),
      redetect_samples_(redetect_samples),
      redetect_interval_(redetect_interval),
      hand_frame_(hand_frame),
      camera_offset_(camera_offset),
      camera_rpy_(camera_rpy) {}

bool GraspTaskManager::run(double timeout) {
  auto start = std::chrono::steady_clock::now();
  state_ = GraspState::kDetecting;

  try {
    while (state_ != GraspState::kDone && state_ != GraspState::kError) {
      // 检查中止请求（信号处理等）
      if (abort_requested_.load(std::memory_order_acquire)) {
        RCLCPP_WARN(rclcpp::get_logger("grasp_task_manager"),
                    "Grasp aborted by external request");
        state_ = GraspState::kError;
        return false;
      }

      auto elapsed = std::chrono::duration<double>(
                         std::chrono::steady_clock::now() - start)
                         .count();
      if (elapsed > timeout) {
        RCLCPP_ERROR(rclcpp::get_logger("grasp_task_manager"),
                     "Grasp timeout (%.1fs)", timeout);
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
        case GraspState::kReDetecting:
          if (!step_redetect()) {
            state_ = GraspState::kError;
          }
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
    RCLCPP_ERROR(rclcpp::get_logger("grasp_task_manager"),
                 "Exception in state %s: %s",
                 state_name(state_.load()), e.what());
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
  auto base_xyz = transform_to_base(result->xyz, /*log_details=*/true);
  if (!base_xyz.has_value()) {
    RCLCPP_WARN(rclcpp::get_logger("grasp_task_manager"),
                "TF lookup failed, cannot determine grasp target in base frame");
    return false;
  }

  target_xyz_ = *base_xyz;
  RCLCPP_INFO(rclcpp::get_logger("grasp_task_manager"),
              "Detected target at [%.4f, %.4f, %.4f] -> APPROACHING",
              (*base_xyz)[0], (*base_xyz)[1], (*base_xyz)[2]);
  state_ = GraspState::kApproaching;
  return true;
}

void GraspTaskManager::step_approach() {
  if (!target_xyz_.has_value()) {
    state_ = GraspState::kError;
    return;
  }

  auto logger = rclcpp::get_logger("grasp_task_manager");

  // 计算接近点（目标正上方 approach_height）
  auto target = *target_xyz_;
  double approach_z = target[2] + approach_height_;

  // 可达性检查
  double dist = std::sqrt(target[0] * target[0] + target[1] * target[1] +
                          approach_z * approach_z);
  if (dist > max_reach_) {
    RCLCPP_ERROR(logger,
                 "Approach target is %.3fm from base, exceeds max reach %.2fm",
                 dist, max_reach_);
    state_ = GraspState::kError;
    return;
  }

  // 获取当前 TCP 位置用于日志
  auto pose = robot_->get_end_effector_pose();
  Eigen::Vector3d current_pos(pose[0], pose[1], pose[2]);
  std::array<double, 3> approach_xyz = {target[0], target[1], approach_z};
  Eigen::Vector3d approach_point(target[0], target[1], approach_z);

  RCLCPP_INFO(logger,
              "Approaching: current [%.4f, %.4f, %.4f] -> approach [%.4f, %.4f, %.4f], "
              "distance=%.3fm (moveL, keep orientation)",
              current_pos.x(), current_pos.y(), current_pos.z(),
              approach_point.x(), approach_point.y(), approach_point.z(),
              (approach_point - current_pos).norm());

  // moveL + nullopt：只改位置，不改朝向。
  // 保持相机朝向不变，避免物体脱离视野。
  // 跟随误差限制已放宽到 0.25 rad 以适应 Isaac Sim 物理延迟。
  try {
    robot_->moveL(approach_xyz, std::nullopt, -1.0, true);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger, "Approach moveL failed: %s", e.what());
    state_ = GraspState::kError;
    return;
  }

  // 验证是否实际到达接近点（检测 FAULT/超时等异常）
  auto pose_after = robot_->get_end_effector_pose();
  Eigen::Vector3d actual_pos(pose_after[0], pose_after[1], pose_after[2]);
  double pos_error = (actual_pos - approach_point).norm();
  RCLCPP_INFO(logger,
              "Approach done: actual [%.4f, %.4f, %.4f], position error %.4fm",
              actual_pos.x(), actual_pos.y(), actual_pos.z(), pos_error);

  constexpr double kApproachPosTolerance = 0.05;  // 5cm
  if (pos_error > kApproachPosTolerance) {
    RCLCPP_ERROR(logger,
                 "Approach position error %.4fm exceeds tolerance %.2fm "
                 "(robot may be in FAULT state or motion timed out)",
                 pos_error, kApproachPosTolerance);
    state_ = GraspState::kError;
    return;
  }

  // 跳过重检测（kReDetecting），直接用初始检测目标下降。
  // 原因：当前相机内参可能与 Isaac Sim 实际分辨率不匹配，
  // 近距离重检测的 3D 投影误差会被放大导致坐标漂移。
  // 初始检测精度已足够（approach 误差 < 1cm）。
  state_ = GraspState::kDescending;
}

bool GraspTaskManager::step_redetect() {
  constexpr int kMaxRetries = 3;
  for (int retry = 0; retry < kMaxRetries; ++retry) {
    if (abort_requested_.load(std::memory_order_acquire)) {
      return false;
    }

    auto result = vision_->average_detections(
        redetect_samples_, redetect_interval_);

    if (!result.has_value() || !result->detected) {
      RCLCPP_WARN(rclcpp::get_logger("grasp_task_manager"),
                  "Re-detection attempt %d/%d failed: no valid detection",
                  retry + 1, kMaxRetries);
      continue;
    }

    auto base_xyz = transform_to_base(result->xyz);
    if (!base_xyz.has_value()) {
      RCLCPP_WARN(rclcpp::get_logger("grasp_task_manager"),
                  "Re-detection attempt %d/%d failed: TF lookup error",
                  retry + 1, kMaxRetries);
      continue;
    }

    if (target_xyz_.has_value()) {
      RCLCPP_INFO(rclcpp::get_logger("grasp_task_manager"),
                  "Target updated: [%.4f, %.4f, %.4f] -> [%.4f, %.4f, %.4f]",
                  (*target_xyz_)[0], (*target_xyz_)[1], (*target_xyz_)[2],
                  (*base_xyz)[0], (*base_xyz)[1], (*base_xyz)[2]);
    }

    target_xyz_ = *base_xyz;
    state_ = GraspState::kDescending;
    return true;
  }

  RCLCPP_ERROR(rclcpp::get_logger("grasp_task_manager"),
               "Re-detection failed after %d attempts", kMaxRetries);
  return false;
}

void GraspTaskManager::step_descend() {
  if (!target_xyz_.has_value()) {
    state_ = GraspState::kError;
    return;
  }

  auto logger = rclcpp::get_logger("grasp_task_manager");
  auto target = *target_xyz_;
  target[2] += grasp_height_offset_;

  auto pose = robot_->get_end_effector_pose();
  RCLCPP_INFO(logger,
              "Descending: current [%.4f, %.4f, %.4f] -> target [%.4f, %.4f, %.4f] "
              "(grasp height offset +%.3fm, keeping current orientation)",
              pose[0], pose[1], pose[2],
              target[0], target[1], target[2],
              grasp_height_offset_);

  // 使用 moveL 下降（直线避免侧向碰撞），但保持当前朝向（std::nullopt）。
  // 不使用 grasp_rpy_ 的原因：观察位朝向 RPY≈[-π,0,0] 与 grasp_rpy_=[π,0,π]
  // 虽然都是朝下，但相差 180° Z 旋转，moveL 的 SLERP 插值会在下降过程中
  // 产生不必要的旋转，可能在某些中间姿态导致 IK 无解。
  // 两者夹爪都朝下，对抓取无影响。
  try {
    robot_->moveL(target, std::nullopt, -1.0, true);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger, "Descend moveL failed: %s", e.what());
    state_ = GraspState::kError;
    return;
  }

  // 验证是否到达抓取高度（检测 FAULT/超时）
  {
    Eigen::Vector3d descend_target(target[0], target[1], target[2]);
    auto pose_after = robot_->get_end_effector_pose();
    Eigen::Vector3d actual(pose_after[0], pose_after[1], pose_after[2]);
    double pos_error = (actual - descend_target).norm();
    RCLCPP_INFO(logger,
                "Descend done: actual [%.4f, %.4f, %.4f], position error %.4fm",
                actual.x(), actual.y(), actual.z(), pos_error);
    if (pos_error > 0.05) {
      RCLCPP_ERROR(logger,
                   "Descend position error %.4fm > 0.05m, aborting grasp",
                   pos_error);
      state_ = GraspState::kError;
      return;
    }
  }

  state_ = GraspState::kGrasping;
}

void GraspTaskManager::step_grasp() {
  auto logger = rclcpp::get_logger("grasp_task_manager");
  RCLCPP_INFO(logger, "Closing gripper...");
  robot_->close_gripper(true);
  state_ = GraspState::kLifting;
}

void GraspTaskManager::step_lift() {
  auto logger = rclcpp::get_logger("grasp_task_manager");
  double lift_height = approach_height_ + 0.1;
  RCLCPP_INFO(logger, "Lifting %.3fm...", lift_height);

  try {
    robot_->move_linear({0.0, 0.0, lift_height}, "base", -1.0, true);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger, "Lift failed: %s", e.what());
    state_ = GraspState::kError;
    return;
  }

  state_ = GraspState::kDone;
}

std::optional<std::array<double, 3>> GraspTaskManager::transform_to_base(
    const Eigen::Vector3d& camera_xyz, bool log_details) {
  // 直接从 hand 位姿 + 已知相机外参计算，不依赖 TF 发布的相机坐标变换。
  // 这样可以避免 Isaac Sim 使用旧 URDF 发布错误相机 TF 的问题。
  //
  // 变换链: base ← hand ← camera_link ← camera_color_optical_frame
  //   hand ← camera_link: 使用构造时传入的 camera_offset_ / camera_rpy_
  //   camera_link ← optical: 标准光学坐标系旋转 rpy=(-π/2, 0, -π/2)

  // 1. 查询 base → hand（来自 TF 关节状态链，不涉及相机 URDF）
  auto tf_hand = robot_->lookup_transform(
      base_frame_, hand_frame_, 1.0);
  if (!tf_hand.has_value()) {
    return std::nullopt;
  }

  if (log_details) {
    RCLCPP_INFO(rclcpp::get_logger("grasp_task_manager"),
                "[TRANSFORM] Step 1 - camera_xyz=[%.4f, %.4f, %.4f]",
                camera_xyz.x(), camera_xyz.y(), camera_xyz.z());
    RCLCPP_INFO(rclcpp::get_logger("grasp_task_manager"),
                "[TRANSFORM] Step 2 - hand t=[%.4f, %.4f, %.4f] rpy=[%.4f, %.4f, %.4f]",
                (*tf_hand)[0], (*tf_hand)[1], (*tf_hand)[2],
                (*tf_hand)[3], (*tf_hand)[4], (*tf_hand)[5]);
    RCLCPP_INFO(rclcpp::get_logger("grasp_task_manager"),
                "[TRANSFORM] Step 3 - camera extrinsics: offset=[%.4f, %.4f, %.4f] "
                "rpy=[%.4f, %.4f, %.4f]",
                camera_offset_[0], camera_offset_[1], camera_offset_[2],
                camera_rpy_[0], camera_rpy_[1], camera_rpy_[2]);
  }

  // 2. base ← hand
  Eigen::Matrix3d R_bh = rpy_to_rotation(
      (*tf_hand)[3], (*tf_hand)[4], (*tf_hand)[5]);
  Eigen::Vector3d t_bh((*tf_hand)[0], (*tf_hand)[1], (*tf_hand)[2]);

  // 3. hand ← camera_link（已知相机外参）
  Eigen::Matrix3d R_hc = rpy_to_rotation(
      camera_rpy_[0], camera_rpy_[1], camera_rpy_[2]);
  Eigen::Vector3d t_hc(
      camera_offset_[0], camera_offset_[1], camera_offset_[2]);

  // 4. camera_link ← optical（标准光学坐标系: X右, Y下, Z前）
  constexpr double kHalfPi = 1.57079632679;
  Eigen::Matrix3d R_co = rpy_to_rotation(-kHalfPi, 0.0, -kHalfPi);

  // 5. 合成: base ← optical
  Eigen::Matrix3d R = R_bh * R_hc * R_co;
  Eigen::Vector3d t = R_bh * t_hc + t_bh;

  // 6. 变换相机坐标点到基座坐标系
  Eigen::Vector3d base_point = R * camera_xyz + t;

  if (log_details) {
    // 打印合成旋转矩阵 R 的列向量（每列代表光学坐标轴在基座系中的方向）
    RCLCPP_INFO(rclcpp::get_logger("grasp_task_manager"),
                "[TRANSFORM] Step 4 - R columns (optical axes in base): "
                "X=[%.3f,%.3f,%.3f] Y=[%.3f,%.3f,%.3f] Z=[%.3f,%.3f,%.3f]",
                R(0,0), R(1,0), R(2,0),
                R(0,1), R(1,1), R(2,1),
                R(0,2), R(1,2), R(2,2));
    RCLCPP_INFO(rclcpp::get_logger("grasp_task_manager"),
                "[TRANSFORM] Step 5 - optical origin in base (t)=[%.4f, %.4f, %.4f]",
                t.x(), t.y(), t.z());
    RCLCPP_INFO(rclcpp::get_logger("grasp_task_manager"),
                "[TRANSFORM] Step 6 - RESULT: base_point=[%.4f, %.4f, %.4f]",
                base_point.x(), base_point.y(), base_point.z());
  }

  return std::array<double, 3>{base_point.x(), base_point.y(),
                                base_point.z()};
}

}  // namespace robot_vision
