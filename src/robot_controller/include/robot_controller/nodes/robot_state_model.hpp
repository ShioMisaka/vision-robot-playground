#pragma once

#include <shared_mutex>
#include <mutex>
#include <vector>
#include <cmath>
#include <algorithm>

namespace robot_control {

/// @brief 线程安全的机器人状态数据模型（The Model）
/// @details 使用 std::shared_mutex 实现读写锁，适用于读多写少场景。
///          100Hz 控制循环频繁读取，joint_states 回调写入实际值。
class RobotStateModel {
public:
  /// @brief 构造函数
  /// @param dof 机械臂自由度
  /// @param default_gripper 默认夹爪开度（米）
  explicit RobotStateModel(int dof, double default_gripper = 0.04)
      : target_joints_(dof, 0.0),
        actual_joints_(dof, 0.0),
        target_gripper_(default_gripper),
        actual_gripper_(default_gripper) {}

  // --- Actual (written by joint_states callback) ---

  /// @brief 更新实际关节和夹爪状态（由订阅回调调用）
  /// @param joints 实际关节角度（弧度）
  /// @param gripper 实际夹爪开度（米）
  void update_actual(const std::vector<double>& joints, double gripper) {
    std::unique_lock lock(mutex_);
    actual_joints_ = joints;
    actual_gripper_ = gripper;
  }

  /// @brief 获取实际关节角度
  /// @return 关节角度数组（弧度）
  std::vector<double> get_actual_joints() const {
    std::shared_lock lock(mutex_);
    return actual_joints_;
  }

  /// @brief 获取实际夹爪开度
  /// @return 夹爪开度（米）
  double get_actual_gripper() const {
    std::shared_lock lock(mutex_);
    return actual_gripper_;
  }

  // --- Target (written by control loop) ---

  /// @brief 更新目标关节和夹爪状态（由控制循环调用）
  /// @param joints 目标关节角度（弧度）
  /// @param gripper 目标夹爪开度（米）
  void update_target(const std::vector<double>& joints, double gripper) {
    std::unique_lock lock(mutex_);
    target_joints_ = joints;
    target_gripper_ = gripper;
  }

  /// @brief 获取目标关节角度
  /// @return 关节角度数组（弧度）
  std::vector<double> get_target_joints() const {
    std::shared_lock lock(mutex_);
    return target_joints_;
  }

  /// @brief 获取目标夹爪开度
  /// @return 夹爪开度（米）
  double get_target_gripper() const {
    std::shared_lock lock(mutex_);
    return target_gripper_;
  }

  // --- Monitoring helpers ---

  /// @brief 计算最大跟踪误差（所有关节中 target - actual 的最大绝对值）
  /// @return 最大误差（弧度）
  double max_following_error() const {
    std::shared_lock lock(mutex_);
    double max_err = 0.0;
    size_t n = std::min(target_joints_.size(), actual_joints_.size());
    for (size_t i = 0; i < n; ++i) {
      max_err = std::max(max_err,
                         std::abs(target_joints_[i] - actual_joints_[i]));
    }
    return max_err;
  }

  /// @brief 检查到位（所有轴误差 < tolerance）
  /// @param tolerance 到位阈值（弧度）
  /// @return true 所有关节在容差内
  bool is_on_target(double tolerance) const {
    std::shared_lock lock(mutex_);
    size_t n = std::min(target_joints_.size(), actual_joints_.size());
    for (size_t i = 0; i < n; ++i) {
      if (std::abs(target_joints_[i] - actual_joints_[i]) > tolerance) {
        return false;
      }
    }
    return true;
  }

  /// @brief 将 target 对齐到 actual（用于 CLEAR_FAULT 恢复、急停锁死）
  void align_target_to_actual() {
    std::unique_lock lock(mutex_);
    target_joints_ = actual_joints_;
    target_gripper_ = actual_gripper_;
  }

  /// @brief 获取关节数
  /// @return 自由度数量
  size_t dof() const {
    std::shared_lock lock(mutex_);
    return target_joints_.size();
  }

private:
  mutable std::shared_mutex mutex_;
  std::vector<double> target_joints_;
  std::vector<double> actual_joints_;
  double target_gripper_;
  double actual_gripper_;
};

}  // namespace robot_control
