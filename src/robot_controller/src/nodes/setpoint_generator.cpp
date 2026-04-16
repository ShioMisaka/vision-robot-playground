#include "robot_controller/nodes/setpoint_generator.hpp"

#include <algorithm>

namespace robot_control {

void SetpointGenerator::start(
    const std::vector<TrajectoryStep>& trajectory, double finger_width) {
  std::lock_guard<std::mutex> lock(mutex_);
  trajectory_ = trajectory;
  finger_width_ = finger_width;
  total_duration_ = trajectory.empty() ? 0.0 : trajectory.back().time_from_start;
  start_time_ = std::chrono::steady_clock::now();
  active_ = true;
  cancelled_ = false;
}

void SetpointGenerator::cancel() {
  std::lock_guard<std::mutex> lock(mutex_);
  cancelled_ = true;
  active_ = false;
}

SetpointResult SetpointGenerator::tick(
    std::chrono::steady_clock::time_point now) {
  std::lock_guard<std::mutex> lock(mutex_);

  SetpointResult result;
  result.finger_width = finger_width_;
  result.done = false;

  if (!active_ || cancelled_) {
    result.done = true;
    result.progress = cancelled_ ? 0.0 : 1.0;
    result.time_remaining = 0.0;
    if (!trajectory_.empty()) {
      result.joint_positions = trajectory_.back().joint_positions;
    }
    active_ = false;
    return result;
  }

  double elapsed =
      std::chrono::duration<double>(now - start_time_).count();

  // 线性搜索当前时间对应的轨迹点（trajectory 已按 time_from_start 排序）
  size_t idx = 0;
  while (idx < trajectory_.size() - 1 &&
         trajectory_[idx + 1].time_from_start <= elapsed) {
    ++idx;
  }

  result.joint_positions = trajectory_[idx].joint_positions;
  result.progress = (total_duration_ > 1e-12)
                        ? std::clamp(elapsed / total_duration_, 0.0, 1.0)
                        : 1.0;
  result.time_remaining = std::max(0.0, total_duration_ - elapsed);

  if (elapsed >= total_duration_) {
    result.joint_positions = trajectory_.back().joint_positions;
    result.progress = 1.0;
    result.time_remaining = 0.0;
    result.done = true;
    active_ = false;
  }

  return result;
}

std::vector<double> SetpointGenerator::final_target() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (trajectory_.empty()) return {};
  return trajectory_.back().joint_positions;
}

double SetpointGenerator::finger_width() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return finger_width_;
}

double SetpointGenerator::total_duration() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return total_duration_;
}

}  // namespace robot_control
