/// @file test_trajectory_planner.cpp
/// @brief S 曲线轨迹规划器独立测试（无需 ROS2 / Isaac Sim）
///
/// 编译后直接运行：
///   ./test_trajectory_planner

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "robot_controller/kinematics/trajectory_planner.hpp"

namespace {

bool approx_eq(double a, double b, double eps = 1e-4) {
  return std::abs(a - b) < eps;
}

int test_count = 0;
int pass_count = 0;

void check(bool cond, const std::string& name) {
  ++test_count;
  if (cond) {
    ++pass_count;
    std::cout << "  [PASS] " << name << std::endl;
  } else {
    std::cout << "  [FAIL] " << name << std::endl;
  }
}

}  // namespace

int main() {
  std::cout << "=== SCurvePlanner Tests ===" << std::endl;

  robot_control::MotionLimits cfg{1.0, 2.0, 10.0};  // v=1, a=2, j=10
  double dt = 0.01;

  // ---- 1. Zero displacement ----
  std::cout << "\n--- 1. Zero displacement ---" << std::endl;
  {
    auto traj = robot_control::SCurvePlanner::plan(0.5, 0.5, cfg, dt);
    check(traj.size() == 1, "Single point for zero displacement");
    check(approx_eq(traj[0].pos, 0.5), "Position is start");
    check(approx_eq(traj[0].vel, 0.0), "Velocity is zero");
  }

  // ---- 2. Full profile (large displacement) ----
  std::cout << "\n--- 2. Full 7-phase profile ---" << std::endl;
  {
    auto traj = robot_control::SCurvePlanner::plan(0.0, 5.0, cfg, dt);
    check(!traj.empty(), "Trajectory not empty");
    check(approx_eq(traj.front().pos, 0.0, 1e-6), "Start at q0");
    check(approx_eq(traj.front().vel, 0.0), "Start vel = 0");
    check(approx_eq(traj.front().acc, 0.0), "Start acc = 0");
    check(approx_eq(traj.back().pos, 5.0, 1e-2), "End at q1");
    check(approx_eq(traj.back().vel, 0.0, 0.01), "End vel = 0");
    check(approx_eq(traj.back().acc, 0.0, 0.01), "End acc = 0");

    bool vel_ok = true;
    for (auto& p : traj) {
      if (std::abs(p.vel) > cfg.max_vel + 0.01) {
        vel_ok = false;
        break;
      }
    }
    check(vel_ok, "Max velocity not exceeded");

    bool acc_ok = true;
    for (auto& p : traj) {
      if (std::abs(p.acc) > cfg.max_acc + 0.01) {
        acc_ok = false;
        break;
      }
    }
    check(acc_ok, "Max acceleration not exceeded");
  }

  // ---- 3. Negative displacement ----
  std::cout << "\n--- 3. Negative displacement ---" << std::endl;
  {
    auto traj = robot_control::SCurvePlanner::plan(5.0, 0.0, cfg, dt);
    check(approx_eq(traj.front().pos, 5.0, 1e-6), "Start at q0");
    check(approx_eq(traj.back().pos, 0.0, 1e-2), "End at q1");
    check(approx_eq(traj.back().vel, 0.0, 0.01), "End vel = 0");
  }

  // ---- 4. No cruise phase (medium displacement) ----
  std::cout << "\n--- 4. No cruise phase ---" << std::endl;
  {
    auto traj = robot_control::SCurvePlanner::plan(0.0, 0.6, cfg, dt);
    check(!traj.empty(), "Trajectory not empty");
    check(approx_eq(traj.front().pos, 0.0, 1e-6), "Start at 0");
    check(approx_eq(traj.back().pos, 0.6, 1e-2), "End at 0.6");
    check(approx_eq(traj.back().vel, 0.0, 0.01), "End vel = 0");

    double peak_vel = 0;
    for (auto& p : traj) {
      peak_vel = std::max(peak_vel, std::abs(p.vel));
    }
    check(peak_vel < cfg.max_vel + 0.01, "Peak vel < max_vel (no cruise)");
  }

  // ---- 5. Very short distance (only jerk phases) ----
  std::cout << "\n--- 5. Very short distance ---" << std::endl;
  {
    auto traj = robot_control::SCurvePlanner::plan(0.0, 0.1, cfg, dt);
    check(!traj.empty(), "Trajectory not empty");
    check(approx_eq(traj.back().pos, 0.1, 1e-2), "End at 0.1");
    check(approx_eq(traj.back().vel, 0.0, 0.01), "End vel = 0");
    check(approx_eq(traj.back().acc, 0.0, 0.01), "End acc = 0");
  }

  // ---- 6. Jerk continuity ----
  std::cout << "\n--- 6. Jerk continuity (acc smoothness) ---" << std::endl;
  {
    auto traj = robot_control::SCurvePlanner::plan(0.0, 5.0, cfg, dt);
    bool smooth = true;
    for (size_t i = 1; i < traj.size(); ++i) {
      double delta_acc = std::abs(traj[i].acc - traj[i-1].acc);
      double max_delta = cfg.max_jerk * dt * 1.5;
      if (delta_acc > max_delta) {
        smooth = false;
        std::cout << "    Acc discontinuity at t=" << traj[i].t
                  << ": delta_a=" << delta_acc << " > " << max_delta << std::endl;
        break;
      }
    }
    check(smooth, "Acceleration changes smoothly (jerk bounded)");
  }

  // ---- 7. Multi-axis sync ----
  std::cout << "\n--- 7. Multi-axis synchronization ---" << std::endl;
  {
    std::vector<double> q_start = {0.0, 0.0, 0.0};
    std::vector<double> q_end = {1.0, 0.5, 2.0};
    std::vector<robot_control::MotionLimits> configs = {
      {1.0, 2.0, 10.0},
      {1.0, 2.0, 10.0},
      {1.0, 2.0, 10.0}
    };

    auto traj = robot_control::TrajectoryPlanner::plan_joint(q_start, q_end, configs, dt);
    check(!traj.empty(), "Multi-axis trajectory not empty");

    bool start_ok = true;
    for (size_t i = 0; i < 3; ++i) {
      if (!approx_eq(traj.front()[i], q_start[i], 0.01)) start_ok = false;
    }
    check(start_ok, "Multi-axis start at q_start");

    bool end_ok = true;
    for (size_t i = 0; i < 3; ++i) {
      if (!approx_eq(traj.back()[i], q_end[i], 0.05)) end_ok = false;
    }
    check(end_ok, "Multi-axis end at q_end");

    check(traj.size() > 10, "Trajectory has multiple steps");
  }

  // ---- 8. Different axis limits ----
  std::cout << "\n--- 8. Different axis limits ---" << std::endl;
  {
    std::vector<double> q_start = {0.0, 0.0};
    std::vector<double> q_end = {1.0, 1.0};
    std::vector<robot_control::MotionLimits> configs = {
      {2.0, 4.0, 20.0},  // Fast axis
      {0.5, 1.0, 5.0}    // Slow axis
    };

    auto traj = robot_control::TrajectoryPlanner::plan_joint(q_start, q_end, configs, dt);
    check(!traj.empty(), "Different limits trajectory not empty");

    bool end_ok = true;
    for (size_t i = 0; i < 2; ++i) {
      if (!approx_eq(traj.back()[i], q_end[i], 0.05)) end_ok = false;
    }
    check(end_ok, "Both axes reach target");
  }

  // ---- Summary ----
  std::cout << "\n=== Results: " << pass_count << "/" << test_count
            << " passed ===" << std::endl;
  return (pass_count == test_count) ? 0 : 1;
}
