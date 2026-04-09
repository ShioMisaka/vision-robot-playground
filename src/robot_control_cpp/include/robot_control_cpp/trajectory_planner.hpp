#pragma once

#include <vector>

#include "robot_control_cpp/robot_profile.hpp"

namespace robot_control {

/// 轨迹采样点
struct TrajectoryPoint {
  double t;    ///< 时间
  double pos;  ///< 位置
  double vel;  ///< 速度
  double acc;  ///< 加速度
};

/// 单轴七段式 S 曲线规划器（Jerk 连续，时间最优）
class SCurvePlanner {
public:
  /// @brief 规划从 q0 到 q1 的单轴轨迹
  /// @param q0 起始位置
  /// @param q1 目标位置
  /// @param limits 运动极限参数（max_vel, max_acc, max_jerk）
  /// @param dt 采样时间步长
  /// @return 采样点序列（包含起始点和终止点）
  static std::vector<TrajectoryPoint> plan(
      double q0, double q1, const MotionLimits& limits, double dt);
};

/// 多轴同步轨迹规划器
class TrajectoryPlanner {
public:
  /// @brief 规划多轴同步 S 曲线轨迹
  /// @param q_start 起始关节角度
  /// @param q_end 目标关节角度
  /// @param limits 每个轴的运动极限
  /// @param dt 采样时间步长
  /// @return 按时间步采样的关节位置序列（每步包含所有轴）
  static std::vector<std::vector<double>> plan_joint(
      const std::vector<double>& q_start,
      const std::vector<double>& q_end,
      const std::vector<MotionLimits>& limits,
      double dt);
};

}  // namespace robot_control
