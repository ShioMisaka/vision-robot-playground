#pragma once

#include "robot_controller/motion/motion_io_bridge.hpp"  // TrajectoryStep

#include <atomic>
#include <chrono>
#include <mutex>
#include <vector>

namespace robot_control {

/// @brief tick() 调用结果
struct SetpointResult {
  std::vector<double> joint_positions;  ///< 当前时刻的目标关节角
  double finger_width = 0.04;           ///< 夹爪宽度
  double progress = 0.0;                ///< 0.0 ~ 1.0
  double time_remaining = 0.0;          ///< 预计剩余时间（秒）
  bool done = false;                    ///< 轨迹所有点已发送完毕
};

/// @brief Tick-based 轨迹指令发生器（零 ROS 依赖，零线程）
///
/// 使用模式:
///   1. start(trajectory, finger) — 提交预计算轨迹
///   2. 每次 100Hz tick 调用 tick() 获取当前目标
///   3. tick().done == true 表示轨迹播放完成
///   4. cancel() 立即标记为完成（done=true）
class SetpointGenerator {
public:
  SetpointGenerator() = default;

  /// @brief 提交轨迹（覆盖当前轨迹）
  /// @param trajectory 预计算轨迹步骤
  /// @param finger_width 夹爪宽度
  void start(const std::vector<TrajectoryStep>& trajectory, double finger_width);

  /// @brief 取消当前轨迹（标记为 done）
  void cancel();

  /// @brief 获取当前时刻的指令目标
  /// @param now 当前时间点
  /// @return SetpointResult，done=true 表示轨迹播放完毕
  SetpointResult tick(std::chrono::steady_clock::time_point now);

  /// @brief 是否有活跃轨迹
  bool is_active() const { return active_.load(); }

  /// @brief 获取最终目标关节角（轨迹最后一个点）
  std::vector<double> final_target() const;

  /// @brief 获取夹爪宽度
  double finger_width() const;

  /// @brief 获取总时长
  double total_duration() const;

private:
  mutable std::mutex mutex_;
  std::vector<TrajectoryStep> trajectory_;
  double finger_width_ = 0.04;
  double total_duration_ = 0.0;
  std::chrono::steady_clock::time_point start_time_;
  std::atomic<bool> active_{false};
  std::atomic<bool> cancelled_{false};
};

}  // namespace robot_control
