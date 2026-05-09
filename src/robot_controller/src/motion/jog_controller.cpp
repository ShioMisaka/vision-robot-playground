#include "robot_controller/motion/jog_controller.hpp"

#include <algorithm>
#include <cmath>

#include <Eigen/Geometry>

#include "robot_controller/kinematics/ik_solver.hpp"

namespace robot_control {

JogController::JogController(std::shared_ptr<IKSolver> ik,
                             const JogConfig& config)
    : ik_(std::move(ik)), config_(config) {}

JogController::~JogController() = default;

void JogController::init_position(
    const std::array<double, 7>& feedback_joints) {
  jog_q_current_ = feedback_joints;
}

void JogController::start(int axis, uint8_t frame) {
  // 重置 S-curve 状态
  jog_active_ = true;
  jog_stopping_ = false;
  jog_v_ = 0.0;
  jog_a_ = 0.0;
  frame_ = frame;
  jog_pose_initialized_ = false;
  cartesian_offset_ = {};

  // 构建速度向量: axis → velocity[0..5]
  // axis 0=+X,1=-X,2=+Y,3=-Y,4=+Z,5=-Z,6=+R,7=-R,8=+P,9=-P,10=+Yw,11=-Yw
  target_velocity_ = {};
  if (axis < 6) {
    int idx = axis / 2;
    double sign = (axis % 2 == 0) ? 1.0 : -1.0;
    target_velocity_[idx] = config_.linear_speed * sign;
  } else {
    int rpy_idx = (axis - 6) / 2;
    double sign = ((axis - 6) % 2 == 0) ? 1.0 : -1.0;
    target_velocity_[3 + rpy_idx] = config_.angular_speed * sign;
  }
}

void JogController::start_raw(const std::array<double, 6>& velocity,
                               uint8_t frame) {
  jog_active_ = true;
  jog_stopping_ = false;
  jog_v_ = 0.0;
  jog_a_ = 0.0;
  frame_ = frame;
  target_velocity_ = velocity;
  jog_pose_initialized_ = false;
  cartesian_offset_ = {};
}

void JogController::stop() {
  if (!jog_active_) return;
  // 进入减速阶段 — tick() 继续运行直到 v=0
  jog_stopping_ = true;
}

void JogController::emergency_stop(
    const std::array<double, 7>& feedback_joints) {
  jog_active_ = false;
  jog_stopping_ = false;
  jog_v_ = 0.0;
  jog_a_ = 0.0;
  jog_q_current_ = feedback_joints;
  target_velocity_ = {};
  jog_pose_initialized_ = false;
  cartesian_offset_ = {};
}

void JogController::reset() {
  jog_active_ = false;
  jog_stopping_ = false;
  jog_v_ = 0.0;
  jog_a_ = 0.0;
  target_velocity_ = {};
  jog_pose_initialized_ = false;
  cartesian_offset_ = {};
}

bool JogController::tick(
    const std::array<double, 7>& feedback_joints,
    std::function<void(const std::vector<double>&, double)> publish_fn) {
  if (!jog_active_ || !ik_) return false;

  const double dt = config_.dt;
  const double a_max = config_.a_max;
  const double j_max = config_.j_max;

  // 首次 tick 或位置未初始化时从反馈初始化
  if (jog_v_ == 0.0 && jog_a_ == 0.0 && !jog_stopping_) {
    init_position(feedback_joints);
  }

  // === 1. S-curve 速度 ramp（jerk-limited acceleration control）===
  if (jog_stopping_) {
    // --- 减速至 v=0 ---
    if (jog_a_ < 0 && jog_v_ <= jog_a_ * jog_a_ / (2.0 * j_max) + 1e-8) {
      double a_target = -std::sqrt(2.0 * j_max * std::max(0.0, jog_v_));
      double da = a_target - jog_a_;
      da = std::max(-j_max * dt, std::min(j_max * dt, da));
      jog_a_ += da;
    } else {
      double da = -a_max - jog_a_;
      da = std::max(-j_max * dt, std::min(j_max * dt, da));
      jog_a_ += da;
    }

    jog_v_ += jog_a_ * dt;
    if (jog_v_ <= 0.0) {
      jog_v_ = 0.0;
      jog_a_ = 0.0;
      jog_active_ = false;
      return false;  // 减速完成
    }
  } else {
    // --- 加速至 v=1.0 ---
    double dv_remaining = 1.0 - jog_v_;
    if (jog_a_ > 0 && dv_remaining <= jog_a_ * jog_a_ / (2.0 * j_max) + 1e-8) {
      double a_target = std::sqrt(2.0 * j_max * std::max(0.0, dv_remaining));
      double da = a_target - jog_a_;
      da = std::max(-j_max * dt, std::min(j_max * dt, da));
      jog_a_ += da;
    } else {
      double da = a_max - jog_a_;
      da = std::max(-j_max * dt, std::min(j_max * dt, da));
      jog_a_ += da;
    }

    jog_v_ += jog_a_ * dt;
    if (jog_v_ >= 1.0) {
      jog_v_ = 1.0;
      jog_a_ = 0.0;
    }
  }

  // === 2. 按速度比例缩放笛卡尔速度 ===
  std::array<double, 6> vel{};
  for (int i = 0; i < 6; ++i) {
    vel[i] = target_velocity_[i] * jog_v_;
  }

  // === 3. 计算当前 FK（用于 TCP→Base 坐标变换）===
  std::vector<double> q_vec(jog_q_current_.begin(), jog_q_current_.end());
  Eigen::Matrix4d T_current = ik_->forward_matrix(q_vec);
  Eigen::Matrix3d current_R = T_current.block<3, 3>(0, 0);

  // === 4. 记录初始位姿（仅首次 tick）===
  if (!jog_pose_initialized_) {
    initial_position_ = T_current.block<3, 1>(0, 3);
    initial_rotation_ = current_R;
    jog_pose_initialized_ = true;
  }

  // === 5. 坐标变换（TCP → Base）===
  if (frame_ == kTcpFrame) {
    Eigen::Vector3d v_tcp(vel[0], vel[1], vel[2]);
    Eigen::Vector3d v_base = current_R * v_tcp;
    vel[0] = v_base.x(); vel[1] = v_base.y(); vel[2] = v_base.z();

    Eigen::Vector3d w_tcp(vel[3], vel[4], vel[5]);
    Eigen::Vector3d w_base = current_R * w_tcp;
    vel[3] = w_base.x(); vel[4] = w_base.y(); vel[5] = w_base.z();
  }

  // === 6. 笛卡尔增量累积（精确，无 Jacobian 近似）===
  std::array<double, 6> delta{};
  for (int i = 0; i < 6; ++i) {
    delta[i] = vel[i] * dt;
    cartesian_offset_[i] += delta[i];
  }

  // === 7. 构造目标笛卡尔位姿 ===
  Eigen::Vector3d target_pos = initial_position_ +
      Eigen::Vector3d(cartesian_offset_[0], cartesian_offset_[1], cartesian_offset_[2]);

  // 旋转：initial_rotation * exp(w_offset)
  Eigen::Vector3d w_offset(cartesian_offset_[3], cartesian_offset_[4], cartesian_offset_[5]);
  Eigen::Matrix3d target_R = initial_rotation_;
  if (w_offset.norm() > 1e-10) {
    target_R = initial_rotation_ *
        Eigen::AngleAxisd(w_offset.norm(), w_offset.normalized()).toRotationMatrix();
  }

  // 提取 RPY
  Eigen::Vector3d target_rpy = target_R.eulerAngles(0, 1, 2);

  // === 8. 解析 IK（每步独立，无积分误差）===
  std::array<double, 3> tgt_xyz = {target_pos.x(), target_pos.y(), target_pos.z()};
  std::array<double, 3> tgt_rpy = {target_rpy.x(), target_rpy.y(), target_rpy.z()};

  auto ik_result = ik_->solve_from(tgt_xyz, tgt_rpy, q_vec);

  if (ik_result) {
    // === 9. 关节速度限制与同步缩放 ===
    double max_ratio = 0.0;
    for (int i = 0; i < config_.dof; ++i) {
      double dq = std::abs((*ik_result)[i] - jog_q_current_[i]);
      double limit = (i < 7) ? config_.joint_vel_limits[i] : 2.175;
      double ratio = dq / (limit * dt);
      if (ratio > max_ratio) max_ratio = ratio;
    }

    if (max_ratio > 1.0) {
      // 缩放关节增量以满足速度限制，同时回退笛卡尔偏移
      double scale = 1.0 / max_ratio;
      for (int i = 0; i < config_.dof; ++i) {
        (*ik_result)[i] = jog_q_current_[i] +
            ((*ik_result)[i] - jog_q_current_[i]) * scale;
      }
      // 回退笛卡尔偏移到与实际关节运动一致
      for (int i = 0; i < 6; ++i) {
        cartesian_offset_[i] -= delta[i] * (1.0 - scale);
      }
    }

    for (int i = 0; i < config_.dof; ++i) {
      jog_q_current_[i] = (*ik_result)[i];
    }

    // 发布关节指令
    if (publish_fn) {
      std::vector<double> arm(jog_q_current_.begin(), jog_q_current_.end());
      publish_fn(arm, finger_width_);
    }
  }

  return true;
}

}  // namespace robot_control
