#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

namespace robot_control {

/// @brief 轨迹执行步骤
struct TrajectoryStep {
  std::vector<double> joint_positions;
  double time_from_start;  // seconds
};

/// @brief 非阻塞轨迹执行器（零 ROS 依赖）
/// 在独立线程中执行预计算的轨迹点序列，支持进度查询和取消。
class TrajectoryExecutor {
public:
  /// @brief 构造
  /// @param publish_fn 每步调用：发送关节指令 (positions, finger)
  TrajectoryExecutor(
      std::function<void(const std::vector<double>&, double)> publish_fn);

  ~TrajectoryExecutor();

  /// @brief 启动轨迹执行（非阻塞）
  /// @param trajectory 预计算的轨迹步骤序列
  /// @param finger_width 夹爪宽度
  void start(const std::vector<TrajectoryStep>& trajectory,
             double finger_width);

  /// @brief 查询执行进度
  /// @param progress 输出：0.0-1.0
  /// @param current_angles 输出：当前关节角度
  /// @param time_remaining 输出：预计剩余时间（秒）
  /// @return true 轨迹正在执行
  bool get_progress(double& progress,
                    std::vector<double>& current_angles,
                    double& time_remaining) const;

  /// @brief 取消当前轨迹
  void cancel();

  /// @brief 是否正在执行
  bool is_active() const;

  /// @brief 阻塞等待执行完成
  /// @param timeout 超时时间（秒），0 表示无限等待
  /// @return true 正常完成，false 超时或被取消
  bool wait_for_completion(double timeout = 0.0);

private:
  void execute_loop();

  std::function<void(const std::vector<double>&, double)> publish_fn_;

  std::vector<TrajectoryStep> trajectory_;
  double finger_width_ = 0.0;
  double total_duration_ = 0.0;

  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::thread thread_;
  std::atomic<bool> active_{false};
  std::atomic<bool> cancelled_{false};
  std::atomic<bool> completed_{false};
  size_t current_step_{0};
};

}  // namespace robot_control
