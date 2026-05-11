#include "robot_controller/kinematics/ik_solver.hpp"
#include "robot_controller/kinematics/robot_profile.hpp"

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

#include <Eigen/SVD>

#include <stdexcept>
#include <mutex>
#include <string>
#include <vector>

namespace robot_control {

/// 阻尼最小二乘法（DLS）IK 速度求解器
/// 在奇异位形附近自动增加阻尼，避免关节速度爆炸
class DampedIkSolverVel {
public:
  DampedIkSolverVel(const KDL::Chain& chain,
                    double lambda_min = 0.01, double lambda_max = 0.5)
      : jac_solver_(chain),
        lambda_min_(lambda_min),
        lambda_max_(lambda_max) {}

  /// @brief 计算关节速度 dq = J^T (J J^T + λ²I)^{-1} Δx
  /// @param q 当前关节角度
  /// @param delta_twist 期望的笛卡尔速度（位姿误差）
  /// @param qdot 输出关节速度
  /// @return 操作度指标（0 = 奇异，1 = 远离奇异）
  double CartToJnt(const KDL::JntArray& q,
                   const KDL::Twist& delta_twist,
                   KDL::JntArray& qdot) {
    KDL::Jacobian J(q.rows());
    jac_solver_.JntToJac(q, J);

    int nj = q.rows();
    int nc = 6;  // 6D twist

    // 构建 Eigen 矩阵
    Eigen::MatrixXd Je = Eigen::MatrixXd::Zero(nc, nj);
    for (int i = 0; i < nc; ++i) {
      for (int j = 0; j < nj; ++j) {
        Je(i, j) = J(i, j);
      }
    }

    Eigen::VectorXd dx(nc);
    dx(0) = delta_twist.vel.x();
    dx(1) = delta_twist.vel.y();
    dx(2) = delta_twist.vel.z();
    dx(3) = delta_twist.rot.x();
    dx(4) = delta_twist.rot.y();
    dx(5) = delta_twist.rot.z();

    // SVD 分解求操作度
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(Je,
        Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::VectorXd singular_values = svd.singularValues();

    // 操作度 w = σ_min / σ_max
    double sigma_max = singular_values(0);
    double sigma_min = singular_values(nj < 6 ? nj - 1 : 5);
    double manipulability = (sigma_max > 1e-10)
        ? sigma_min / sigma_max : 0.0;

    // 自适应阻尼：操作度越低，阻尼越大
    // 在奇异附近 (manipulability < threshold) 线性增加 λ
    double threshold = 0.1;
    double lambda;
    if (manipulability >= threshold) {
      lambda = lambda_min_;
    } else {
      lambda = lambda_min_ +
          (lambda_max_ - lambda_min_) * (1.0 - manipulability / threshold);
    }

    // DLS: dq = J^T (J J^T + λ²I)^{-1} dx
    Eigen::MatrixXd JJT = Je * Je.transpose();
    JJT.diagonal().array() += lambda * lambda;
    Eigen::VectorXd dq = Je.transpose() * JJT.ldlt().solve(dx);

    for (int j = 0; j < nj; ++j) {
      qdot(j) = dq(j);
    }

    return manipulability;
  }

private:
  KDL::ChainJntToJacSolver jac_solver_;
  double lambda_min_;
  double lambda_max_;
};

struct IKSolver::Impl {
  RobotProfile profile;
  std::unique_ptr<KDL::Chain> chain;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver;
  std::unique_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver;
  KDL::JntArray q_min;
  KDL::JntArray q_max;
  std::unique_ptr<KDL::ChainIkSolverPos_NR_JL> ik_solver;

  // 缓存上一次 IK 结果作为下次初始猜测
  mutable std::mutex last_result_mutex_;
  mutable std::optional<std::vector<double>> last_result;
};

IKSolver::IKSolver(const RobotProfile& profile) : impl_(std::make_unique<Impl>()) {
  impl_->profile = profile;

  urdf::Model model;
  if (!model.initFile(profile.urdf_path)) {
    throw std::runtime_error(
        "IKSolver: failed to load URDF: " + profile.urdf_path);
  }

  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(model, tree)) {
    throw std::runtime_error(
        "IKSolver: failed to parse URDF to KDL tree");
  }

  impl_->chain = std::make_unique<KDL::Chain>();
  if (!tree.getChain(profile.base_frame, profile.hand_frame,
                     *impl_->chain)) {
    throw std::runtime_error(
        "IKSolver: failed to extract chain from " + profile.base_frame +
        " to " + profile.hand_frame);
  }

  impl_->fk_solver =
      std::make_unique<KDL::ChainFkSolverPos_recursive>(*impl_->chain);

  // 使用较大的 eps 截断小奇异值，提高奇异位形附近的稳定性
  impl_->ik_vel_solver =
      std::make_unique<KDL::ChainIkSolverVel_pinv>(*impl_->chain, 1e-3);

  impl_->q_min = KDL::JntArray(profile.dof);
  impl_->q_max = KDL::JntArray(profile.dof);
  for (int i = 0; i < profile.dof; ++i) {
    impl_->q_min(i) = profile.joint_limits_lower[i];
    impl_->q_max(i) = profile.joint_limits_upper[i];
  }

  impl_->ik_solver = std::make_unique<KDL::ChainIkSolverPos_NR_JL>(
      *impl_->chain, impl_->q_min, impl_->q_max,
      *impl_->fk_solver, *impl_->ik_vel_solver);
}

IKSolver::~IKSolver() = default;

int IKSolver::get_dof() const {
  return impl_->profile.dof;
}

std::optional<std::vector<double>> IKSolver::solve(
    const std::array<double, 3>& xyz,
    const std::optional<std::array<double, 3>>& rpy) const {
  int dof = impl_->profile.dof;

  // 构造初始猜测
  std::vector<double> init_guess;
  {
    std::lock_guard<std::mutex> lock(impl_->last_result_mutex_);
    if (impl_->last_result.has_value()) {
      init_guess = *impl_->last_result;
    } else {
      init_guess = impl_->profile.ik_default_guess;
    }
  }

  KDL::JntArray q_init(dof);
  for (int i = 0; i < dof; ++i) {
    q_init(i) = init_guess[i];
  }

  KDL::Frame target_frame;
  if (rpy.has_value()) {
    target_frame.M = KDL::Rotation::RPY((*rpy)[0], (*rpy)[1], (*rpy)[2]);
  }
  target_frame.p = KDL::Vector(xyz[0], xyz[1], xyz[2]);

  KDL::JntArray q_out(dof);
  int ret = impl_->ik_solver->CartToJnt(q_init, target_frame, q_out);

  if (ret < 0) {
    return std::nullopt;
  }

  std::vector<double> angles(dof);
  for (int i = 0; i < dof; ++i) {
    angles[i] = q_out(i);
  }

  {
    std::lock_guard<std::mutex> lock(impl_->last_result_mutex_);
    impl_->last_result = angles;
  }
  return angles;
}

std::array<double, 6> IKSolver::forward(
    const std::vector<double>& joint_angles) const {
  int dof = impl_->profile.dof;
  KDL::JntArray q(dof);
  for (int i = 0; i < dof; ++i) {
    q(i) = joint_angles[i];
  }

  KDL::Frame frame;
  impl_->fk_solver->JntToCart(q, frame);

  double roll, pitch, yaw;
  frame.M.GetRPY(roll, pitch, yaw);

  return {frame.p.x(), frame.p.y(), frame.p.z(), roll, pitch, yaw};
}

Eigen::Matrix4d IKSolver::forward_matrix(
    const std::vector<double>& joint_angles) const {
  int dof = impl_->profile.dof;
  KDL::JntArray q(dof);
  for (int i = 0; i < dof; ++i) {
    q(i) = joint_angles[i];
  }

  KDL::Frame frame;
  impl_->fk_solver->JntToCart(q, frame);

  Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
  m(0, 0) = frame.M(0, 0);
  m(0, 1) = frame.M(0, 1);
  m(0, 2) = frame.M(0, 2);
  m(1, 0) = frame.M(1, 0);
  m(1, 1) = frame.M(1, 1);
  m(1, 2) = frame.M(1, 2);
  m(2, 0) = frame.M(2, 0);
  m(2, 1) = frame.M(2, 1);
  m(2, 2) = frame.M(2, 2);
  m(0, 3) = frame.p.x();
  m(1, 3) = frame.p.y();
  m(2, 3) = frame.p.z();

  return m;
}

std::optional<std::vector<double>> IKSolver::velocity_ik(
    const std::vector<double>& current_joints,
    const std::array<double, 6>& cartesian_delta) const {
  int dof = impl_->profile.dof;
  KDL::JntArray q(dof);
  for (int i = 0; i < dof; ++i) {
    q(i) = current_joints[i];
  }

  KDL::Twist twist(
      KDL::Vector(cartesian_delta[0], cartesian_delta[1], cartesian_delta[2]),
      KDL::Vector(cartesian_delta[3], cartesian_delta[4], cartesian_delta[5]));

  KDL::JntArray dq(dof);
  int ret = impl_->ik_vel_solver->CartToJnt(q, twist, dq);

  if (ret < 0) return std::nullopt;

  std::vector<double> result(dof);
  for (int i = 0; i < dof; ++i) {
    result[i] = current_joints[i] + dq(i);
  }
  return result;
}

void IKSolver::set_seed(const std::vector<double>& seed) const {
  std::lock_guard<std::mutex> lock(impl_->last_result_mutex_);
  impl_->last_result = seed;
}

std::optional<std::vector<double>> IKSolver::solve_from(
    const std::array<double, 3>& xyz,
    const std::optional<std::array<double, 3>>& rpy,
    const std::vector<double>& initial_guess) const {
  int dof = impl_->profile.dof;
  KDL::JntArray q_init(dof);
  for (int i = 0; i < dof; ++i) {
    q_init(i) = initial_guess[i];
  }

  KDL::Frame target_frame;
  if (rpy.has_value()) {
    target_frame.M = KDL::Rotation::RPY((*rpy)[0], (*rpy)[1], (*rpy)[2]);
  }
  target_frame.p = KDL::Vector(xyz[0], xyz[1], xyz[2]);

  KDL::JntArray q_out(dof);
  int ret = impl_->ik_solver->CartToJnt(q_init, target_frame, q_out);

  if (ret < 0) return std::nullopt;

  std::vector<double> angles(dof);
  for (int i = 0; i < dof; ++i) {
    angles[i] = q_out(i);
  }

  // Do NOT update last_result — jog uses actual position as guess
  return angles;
}

std::optional<std::vector<double>> IKSolver::solve_from_frame(
    const Eigen::Vector3d& position,
    const Eigen::Matrix3d& rotation,
    const std::vector<double>& initial_guess) const {
  int dof = impl_->profile.dof;
  KDL::JntArray q_init(dof);
  for (int i = 0; i < dof; ++i) {
    q_init(i) = initial_guess[i];
  }

  KDL::Frame target_frame;
  // 直接从 Eigen 矩阵构造 KDL 旋转，避免 Euler angle 往返转换
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      target_frame.M(i, j) = rotation(i, j);
    }
  }
  target_frame.p = KDL::Vector(position.x(), position.y(), position.z());

  KDL::JntArray q_out(dof);
  int ret = impl_->ik_solver->CartToJnt(q_init, target_frame, q_out);

  if (ret < 0) return std::nullopt;

  std::vector<double> angles(dof);
  for (int i = 0; i < dof; ++i) {
    angles[i] = q_out(i);
  }

  return angles;
}

}  // namespace robot_control
