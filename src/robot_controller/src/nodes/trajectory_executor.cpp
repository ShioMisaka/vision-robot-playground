#include "robot_controller/nodes/trajectory_executor.hpp"

#include <algorithm>
#include <stdexcept>

namespace robot_control {

TrajectoryExecutor::TrajectoryExecutor(
    std::function<void(const std::vector<double>&, double)> publish_fn)
    : publish_fn_(std::move(publish_fn)) {}

TrajectoryExecutor::~TrajectoryExecutor() {
  cancel();
  if (thread_.joinable()) {
    thread_.join();
  }
}

void TrajectoryExecutor::start(const std::vector<TrajectoryStep>& trajectory,
                               double finger_width) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (active_) {
    throw std::runtime_error("TrajectoryExecutor: already active");
  }

  trajectory_ = trajectory;
  finger_width_ = finger_width;
  total_duration_ = trajectory.empty() ? 0.0 : trajectory.back().time_from_start;
  current_step_ = 0;
  active_ = true;
  cancelled_ = false;
  completed_ = false;

  thread_ = std::thread(&TrajectoryExecutor::execute_loop, this);
}

void TrajectoryExecutor::execute_loop() {
  auto start = std::chrono::steady_clock::now();

  for (size_t i = 0; i < trajectory_.size(); ++i) {
    if (cancelled_) break;

    double t = trajectory_[i].time_from_start;
    auto target_time = start + std::chrono::duration<double>(t);
    std::this_thread::sleep_until(target_time);

    if (cancelled_) break;

    publish_fn_(trajectory_[i].joint_positions, finger_width_);

    std::lock_guard<std::mutex> lock(mutex_);
    current_step_ = i;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!cancelled_) {
      completed_ = true;
    }
    active_ = false;
  }
  cv_.notify_all();
}

bool TrajectoryExecutor::get_progress(double& progress,
                                      std::vector<double>& current_angles,
                                      double& time_remaining) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!active_ && !completed_) return false;
  if (trajectory_.empty()) return false;

  size_t step = current_step_;
  double elapsed = (step < trajectory_.size()) ? trajectory_[step].time_from_start : total_duration_;

  progress = (total_duration_ > 1e-12) ? elapsed / total_duration_ : 1.0;
  progress = std::clamp(progress, 0.0, 1.0);

  if (step < trajectory_.size()) {
    current_angles = trajectory_[step].joint_positions;
  } else if (!trajectory_.empty()) {
    current_angles = trajectory_.back().joint_positions;
  }

  time_remaining = std::max(0.0, total_duration_ - elapsed);
  return active_;
}

void TrajectoryExecutor::cancel() {
  cancelled_ = true;
  cv_.notify_all();
}

bool TrajectoryExecutor::is_active() const {
  return active_.load();
}

bool TrajectoryExecutor::wait_for_completion(double timeout) {
  std::unique_lock<std::mutex> lock(mutex_);
  if (timeout <= 0.0) {
    cv_.wait(lock, [this] { return !active_; });
    return completed_.load();
  }
  return cv_.wait_for(lock, std::chrono::duration<double>(timeout),
                      [this] { return !active_; }) &&
         completed_.load();
}

}  // namespace robot_control
