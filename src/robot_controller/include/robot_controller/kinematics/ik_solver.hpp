#pragma once

#include <array>
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include <Eigen/Core>

namespace robot_control {

struct RobotProfile;

/// IK/FK 运动学求解器，基于 KDL（Orocos），不依赖 rclcpp
class IKSolver {
public:
  /// @brief 构造求解器并加载 URDF 模型
  /// @param profile 机器人参数描述
  explicit IKSolver(const RobotProfile& profile);

  ~IKSolver();

  // 禁止拷贝
  IKSolver(const IKSolver&) = delete;
  IKSolver& operator=(const IKSolver&) = delete;

  /// @brief 求解逆运动学
  /// @param xyz 目标位置 [x, y, z]（米）
  /// @param rpy 目标姿态 [roll, pitch, yaw]（弧度），nullopt 则仅约束位置
  /// @return 7 个关节角度，求解失败返回 nullopt
  std::optional<std::vector<double>> solve(
      const std::array<double, 3>& xyz,
      const std::optional<std::array<double, 3>>& rpy = std::nullopt) const;

  /// @brief 求解正运动学
  /// @param joint_angles 7 个关节角度
  /// @return 位姿 [x, y, z, roll, pitch, yaw]
  std::array<double, 6> forward(
      const std::vector<double>& joint_angles) const;

  /// @brief 正运动学，返回 4x4 齐次变换矩阵
  /// @param joint_angles 7 个关节角度
  /// @return 4x4 Eigen 齐次矩阵
  Eigen::Matrix4d forward_matrix(
      const std::vector<double>& joint_angles) const;

  /// @brief 获取主动关节数
  int get_dof() const;

  /// @brief 基于雅可比伪逆的速度 IK（单步，非迭代）
  /// 直接将笛卡尔速度转换为关节增量：dq = J^+ * twist
  /// @param current_joints 当前关节角度
  /// @param cartesian_delta 6D 笛卡尔增量 [dx,dy,dz,dr,dp,dyaw]（base frame）
  /// @return 新关节角度，失败返回 nullopt
  std::optional<std::vector<double>> velocity_ik(
      const std::vector<double>& current_joints,
      const std::array<double, 6>& cartesian_delta) const;

  /// @brief 设置下次 solve() 的初始猜测（更新 last_result 缓存）
  /// @param seed 初始关节角度猜测（通常为当前关节角度）
  void set_seed(const std::vector<double>& seed) const;

  /// @brief 从指定初始猜测求解 IK（不修改 last_result 缓存）
  /// 适用于 Jog 等需要从当前实际关节位置开始求解的场景
  /// @param xyz 目标位置 [x, y, z]（米）
  /// @param rpy 目标姿态 [roll, pitch, yaw]（弧度）
  /// @param initial_guess 初始关节角度猜测
  /// @return 关节角度，求解失败返回 nullopt
  std::optional<std::vector<double>> solve_from(
      const std::array<double, 3>& xyz,
      const std::optional<std::array<double, 3>>& rpy,
      const std::vector<double>& initial_guess) const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace robot_control
