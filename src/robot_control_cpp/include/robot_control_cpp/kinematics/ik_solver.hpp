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

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace robot_control
