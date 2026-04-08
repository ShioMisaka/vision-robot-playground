# S 曲线运动控制接口实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add moveJ/moveL motion interfaces with seven-segment S-curve trajectory planning and percentage-based speed control to the robot motion controller.

**Architecture:** New `SCurvePlanner` (single-axis) and `TrajectoryPlanner` (multi-axis sync) classes in Layer 1 (pure C++, no ROS). `RobotMotionController` gains moveJ/moveL/set_speed methods and independent speed state for each mode. Old interfaces remain untouched.

**Tech Stack:** C++17, Eigen3 (quaternion slerp for moveL), existing MotionIOBridge pattern

**Spec:** `docs/superpowers/specs/2026-04-08-scurve-motion-design.md`

---

## File Structure

| File | Responsibility |
|------|---------------|
| `src/robot_control_cpp/include/robot_control_cpp/trajectory_planner.hpp` | **New.** SCurvePlanner + TrajectoryPlanner declarations |
| `src/robot_control_cpp/src/trajectory_planner.cpp` | **New.** Seven-segment algorithm + multi-axis sync |
| `src/robot_control_cpp/include/robot_control_cpp/robot_profile.hpp` | Add `MotionLimits` struct + fields to `RobotProfile` |
| `src/robot_control_cpp/include/robot_control_cpp/panda_profile.hpp` | Fill Panda joint/cartesian limits |
| `src/robot_control_cpp/include/robot_control_cpp/config.hpp` | Add `kTrajectoryDt` constant |
| `src/robot_control_cpp/include/robot_control_cpp/i_robot_controller.hpp` | Add `MotionMode` enum + moveJ/moveL/set_speed/get_speed |
| `src/robot_control_cpp/include/robot_control_cpp/robot_motion_controller.hpp` | Add method declarations + speed members |
| `src/robot_control_cpp/src/robot_motion_controller.cpp` | Implement moveJ/moveL/set_speed/get_speed |
| `src/robot_control_cpp/CMakeLists.txt` | Add trajectory_planner.cpp to robot_control_core |
| `src/robot_control_cpp_py/src/bindings.cpp` | Bind new methods, MotionLimits, MotionMode |
| `src/robot_control_test/test/test_trajectory_planner.cpp` | **New.** Standalone tests for SCurvePlanner + TrajectoryPlanner |
| `src/robot_control_test/CMakeLists.txt` | Add test_trajectory_planner target |

---

### Task 1: MotionLimits + MotionMode + kTrajectoryDt

**Files:**
- Modify: `src/robot_control_cpp/include/robot_control_cpp/robot_profile.hpp`
- Modify: `src/robot_control_cpp/include/robot_control_cpp/i_robot_controller.hpp`
- Modify: `src/robot_control_cpp/include/robot_control_cpp/config.hpp`

- [ ] **Step 1: Add MotionLimits to robot_profile.hpp**

Add before `TcpConfig`:

```cpp
/// 运动极限参数（速度/加速度/加加速度）
struct MotionLimits {
  double max_vel = 1.0;   ///< moveJ: rad/s, moveL: m/s
  double max_acc = 1.0;   ///< moveJ: rad/s², moveL: m/s²
  double max_jerk = 1.0;  ///< moveJ: rad/s³, moveL: m/s³
};
```

Add to `RobotProfile`:
```cpp
MotionLimits joint_limits;
MotionLimits cartesian_limits;
```

- [ ] **Step 2: Add MotionMode enum to i_robot_controller.hpp**

Add before `class IRobotController`:

```cpp
/// 运动模式，用于 set_speed/get_speed
enum class MotionMode { kMoveJ, kMoveL };
```

- [ ] **Step 3: Add kTrajectoryDt to config.hpp**

Add inside `ControlConstants`:
```cpp
static constexpr double kTrajectoryDt = 0.01;  ///< 轨迹规划时间步长（100 Hz）
```

- [ ] **Step 4: Add new virtual methods to IRobotController**

```cpp
virtual void moveJ(const std::vector<double>& target_angles,
                   bool block = true) = 0;
virtual void moveL(const std::array<double, 3>& xyz,
                   const std::optional<std::array<double, 3>>& rpy = std::nullopt,
                   double finger = -1.0, bool block = true) = 0;
virtual void set_speed(MotionMode mode, double percent) = 0;
virtual double get_speed(MotionMode mode) const = 0;
```

- [ ] **Step 5: Commit**

```bash
git add src/robot_control_cpp/include/
git commit -m "feat: add MotionLimits, MotionMode, and new virtual interface methods"
```

---

### Task 2: SCurvePlanner — Seven-Segment S-Curve Algorithm

**Files:**
- Create: `src/robot_control_cpp/include/robot_control_cpp/trajectory_planner.hpp`
- Create: `src/robot_control_cpp/src/trajectory_planner.cpp`

- [ ] **Step 1: Write trajectory_planner.hpp**

```cpp
#pragma once

#include <vector>

namespace robot_control {

/// 单轴七段式 S 曲线规划参数
struct SCurveConfig {
  double max_vel;   ///< 最大速度
  double max_acc;   ///< 最大加速度
  double max_jerk;  ///< 最大加加速度
};

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
  /// @param cfg 运动极限参数
  /// @param dt 采样时间步长
  /// @return 采样点序列（包含起始点和终止点）
  static std::vector<TrajectoryPoint> plan(
      double q0, double q1, const SCurveConfig& cfg, double dt);
};

/// 多轴同步轨迹规划器
class TrajectoryPlanner {
public:
  /// @brief 规划多轴同步 S 曲线轨迹
  /// @param q_start 起始关节角度
  /// @param q_end 目标关节角度
  /// @param configs 每个轴的运动极限
  /// @param dt 采样时间步长
  /// @return 按时间步采样的关节位置序列（每步包含所有轴）
  static std::vector<std::vector<double>> plan_joint(
      const std::vector<double>& q_start,
      const std::vector<double>& q_end,
      const std::vector<SCurveConfig>& configs,
      double dt);
};

}  // namespace robot_control
```

- [ ] **Step 2: Implement SCurvePlanner::plan in trajectory_planner.cpp**

Algorithm: Classic seven-segment time-optimal trajectory planning using **closed-form analytical solutions** per phase (not numerical integration).

Each phase has known initial conditions (a0, v0, p0) and constant jerk j. The analytical solution within a phase of duration T_j is:
```
a(t) = a0 + j*t
v(t) = v0 + a0*t + 0.5*j*t²
p(t) = p0 + v0*t + 0.5*a0*t² + (1/6)*j*t³
```

```cpp
#include "robot_control_cpp/trajectory_planner.hpp"

#include <algorithm>
#include <cmath>

namespace robot_control {

namespace {

/// Evaluate position at time tau within a phase using analytical formula
/// @param p0 initial position
/// @param v0 initial velocity
/// @param a0 initial acceleration
/// @param j  jerk (constant within phase)
/// @param tau time within phase [0, T_phase]
double eval_pos(double p0, double v0, double a0, double j, double tau) {
  return p0 + v0 * tau + 0.5 * a0 * tau * tau + (1.0 / 6.0) * j * tau * tau * tau;
}

/// Evaluate velocity at time tau within a phase
double eval_vel(double v0, double a0, double j, double tau) {
  return v0 + a0 * tau + 0.5 * j * tau * tau;
}

/// Evaluate acceleration at time tau within a phase
double eval_acc(double a0, double j, double tau) {
  return a0 + j * tau;
}

}  // namespace

std::vector<TrajectoryPoint> SCurvePlanner::plan(
    double q0, double q1, const SCurveConfig& cfg, double dt) {
  double h = q1 - q0;
  if (std::abs(h) < 1e-12) {
    return {{0.0, q0, 0.0, 0.0}};
  }

  // Work with positive displacement, flip at the end if needed
  int sign = (h > 0) ? 1 : -1;
  double abs_h = std::abs(h);

  double v_max = cfg.max_vel;
  double a_max = cfg.max_acc;
  double j_max = cfg.max_jerk;

  // ============ Determine phase durations ============

  // Minimum distance for full accel+decel to reach v_max
  // Accel phases 1-3 distance + decel phases 5-7 distance (symmetric)
  double d_full = 2.0 * (v_max * a_max / j_max + v_max * v_max / (2.0 * a_max));

  double t1 = 0, t2 = 0, t3 = 0, t4 = 0, t5 = 0, t6 = 0, t7 = 0;

  if (abs_h >= d_full) {
    // ---- Case 1: Full 7-phase profile (all phases present) ----
    t1 = a_max / j_max;
    t3 = t1;
    t5 = t1;
    t7 = t1;

    // Velocity at end of phase 1: v1 = 0.5 * j_max * t1²
    double v1 = 0.5 * j_max * t1 * t1;
    // After phases 1+2+3, velocity should be v_max
    // Phase 3 contributes additional velocity: 0.5 * j_max * t3² = v1
    // So v_max = v1 + a_max * t2 + v1 = 2*v1 + a_max * t2
    t2 = (v_max - 2.0 * v1) / a_max;
    t6 = t2;

    // Compute distance covered by accel phases 1-3 analytically
    double p1 = (1.0 / 6.0) * j_max * t1 * t1 * t1;
    double v_after_1 = v1;
    double p2 = p1 + v_after_1 * t2 + 0.5 * a_max * t2 * t2;
    double v_after_2 = v_after_1 + a_max * t2;
    double p3 = p2 + v_after_2 * t3 + 0.5 * a_max * t3 * t3
              - (1.0 / 6.0) * j_max * t3 * t3 * t3;

    double d_accel = p3;
    t4 = (abs_h - 2.0 * d_accel) / v_max;
    if (t4 < 0) t4 = 0;

  } else {
    // ---- Cases 2 & 3: Max velocity not reachable (t4=0) ----
    t4 = 0;
    t1 = a_max / j_max;
    t3 = t1;
    t5 = t1;
    t7 = t1;

    // Try Case 2: assume a_max is reached (t1 = a_max/j_max), solve for v_peak
    // h = v_peak²/a_max + 2*v_peak*a_max/j_max
    // v_peak² + 2*v_peak*a_max²/j_max - h*a_max = 0
    double b_coeff = 2.0 * a_max * a_max / j_max;
    double v_peak = (-b_coeff + std::sqrt(b_coeff * b_coeff + 4.0 * abs_h * a_max)) / 2.0;

    double v1 = 0.5 * j_max * t1 * t1;  // = 0.5 * a_max² / j_max

    // Check if t2 would be non-negative (i.e., a_max actually reached)
    // v_peak must >= 2*v1 for t2 >= 0
    if (v_peak >= 2.0 * v1 - 1e-12) {
      // Case 2: a_max reached, but v_max not reached
      t2 = (v_peak - 2.0 * v1) / a_max;
      if (t2 < 0) t2 = 0;
      t6 = t2;
    } else {
      // Case 3: even a_max not reached, only jerk phases
      // t1=t3=t5=t7, t2=t4=t6=0
      // Total distance: h = 2*j_max*t1³
      t2 = 0;
      t6 = 0;
      t1 = std::cbrt(abs_h / (2.0 * j_max));
      t3 = t1;
      t5 = t1;
      t7 = t1;
    }
  }

  double T = t1 + t2 + t3 + t4 + t5 + t6 + t7;

  // ============ Sample trajectory using analytical formulas ============

  std::vector<TrajectoryPoint> points;
  points.reserve(static_cast<size_t>(T / dt) + 2);

  // Record start point
  points.push_back({0.0, q0, 0.0, 0.0});

  // Phase boundaries: cumulative times
  double t_bounds[8];
  t_bounds[0] = 0;
  t_bounds[1] = t1;
  t_bounds[2] = t1 + t2;
  t_bounds[3] = t1 + t2 + t3;
  t_bounds[4] = t1 + t2 + t3 + t4;
  t_bounds[5] = t1 + t2 + t3 + t4 + t5;
  t_bounds[6] = t1 + t2 + t3 + t4 + t5 + t6;
  t_bounds[7] = T;

  // Jerk for each phase (signed)
  double j_signed = static_cast<double>(sign) * j_max;
  double jerks[7] = {
    j_signed,        // Phase 1: +jerk (accel increasing)
    0.0,             // Phase 2: constant accel
    -j_signed,       // Phase 3: -jerk (accel decreasing to 0)
    0.0,             // Phase 4: cruise (constant vel)
    -j_signed,       // Phase 5: -jerk (decel increasing)
    0.0,             // Phase 6: constant decel
    j_signed         // Phase 7: +jerk (decel decreasing to 0)
  };

  // For each sample time, find phase and compute analytically
  int num_steps = static_cast<int>(std::ceil(T / dt));
  for (int step = 1; step <= num_steps; ++step) {
    double t = std::min(step * dt, T);

    // Find which phase we're in
    int phase = 0;
    for (int p = 0; p < 7; ++p) {
      if (t >= t_bounds[p] - 1e-12) {
        phase = p;
      }
    }

    // Walk forward from t=0 through all completed phases to get initial conditions
    // for the current phase, then evaluate analytically within the phase.
    double p0 = 0.0, v0 = 0.0, a0 = 0.0;
    double t_acc = 0.0;
    for (int p = 0; p < phase; ++p) {
      double T_p = t_bounds[p + 1] - t_bounds[p];
      // End-of-phase values become initial conditions for next phase
      double p_end = eval_pos(p0, v0, a0, jerks[p], T_p);
      double v_end = eval_vel(v0, a0, jerks[p], T_p);
      double a_end = eval_acc(a0, jerks[p], T_p);
      p0 = p_end;
      v0 = v_end;
      a0 = a_end;
      t_acc = t_bounds[p + 1];
    }

    // Time within current phase
    double tau = t - t_acc;
    double j = jerks[phase];

    double pos = static_cast<double>(sign) * eval_pos(p0, v0, a0, j, tau) + q0;
    double vel = static_cast<double>(sign) * eval_vel(v0, a0, j, tau);
    double acc = static_cast<double>(sign) * eval_acc(a0, j, tau);

    points.push_back({t, pos, vel, acc});
  }

  // Ensure final point is exactly at target
  if (!points.empty()) {
    points.back().pos = q1;
    points.back().vel = 0.0;
    points.back().acc = 0.0;
    points.back().t = T;
  }

  return points;
}

}  // namespace robot_control
```

**Key design decisions:**
- Uses **closed-form analytical evaluation** (not Euler integration) — no accumulated error
- Each sample point computes from phase boundaries, not incrementally
- Sign convention: all internal calculations use positive displacement, multiplied by `sign` at evaluation
- Phase durations computed from standard seven-segment theory with 3 cases

- [ ] **Step 3: Commit**

```bash
git add src/robot_control_cpp/include/robot_control_cpp/trajectory_planner.hpp \
        src/robot_control_cpp/src/trajectory_planner.cpp
git commit -m "feat: add SCurvePlanner with seven-segment trajectory algorithm"
```

---

### Task 3: TrajectoryPlanner — Multi-Axis Sync

**Files:**
- Modify: `src/robot_control_cpp/src/trajectory_planner.cpp`

- [ ] **Step 1: Implement TrajectoryPlanner::plan_joint**

Strategy: Plan each axis independently, find longest time T_max, then re-plan shorter axes with **only max_vel reduced** (max_acc and max_jerk unchanged, per spec) using iterative binary search to match T_max exactly.

```cpp
std::vector<std::vector<double>> TrajectoryPlanner::plan_joint(
    const std::vector<double>& q_start,
    const std::vector<double>& q_end,
    const std::vector<SCurveConfig>& configs,
    double dt) {
  size_t n = q_start.size();
  if (n != q_end.size() || n != configs.size()) {
    throw std::invalid_argument("plan_joint: dimension mismatch");
  }

  // Step 1: Plan each axis independently, find longest time
  std::vector<std::vector<TrajectoryPoint>> axis_trajectories(n);
  double T_max = 0.0;

  for (size_t i = 0; i < n; ++i) {
    axis_trajectories[i] = SCurvePlanner::plan(q_start[i], q_end[i], configs[i], dt);
    if (!axis_trajectories[i].empty()) {
      T_max = std::max(T_max, axis_trajectories[i].back().t);
    }
  }

  // Step 2: Re-plan shorter axes to match T_max by reducing only max_vel
  // Use binary search on max_vel to converge to T_max
  for (size_t i = 0; i < n; ++i) {
    double T_i = axis_trajectories[i].back().t;
    if (T_i < T_max - dt * 0.5) {
      SCurveConfig adjusted = configs[i];
      // Binary search: low=0, high=original max_vel
      double v_low = 0.0;
      double v_high = configs[i].max_vel;

      for (int iter = 0; iter < 30; ++iter) {  // ~1e-9 precision after 30 iterations
        double v_mid = 0.5 * (v_low + v_high);
        adjusted.max_vel = v_mid;
        auto trial = SCurvePlanner::plan(q_start[i], q_end[i], adjusted, dt);
        double T_trial = trial.empty() ? 0.0 : trial.back().t;
        if (T_trial < T_max) {
          v_high = v_mid;  // Too slow, need more vel
        } else {
          v_low = v_mid;   // Too fast, need less vel
        }
      }
      adjusted.max_vel = 0.5 * (v_low + v_high);
      axis_trajectories[i] = SCurvePlanner::plan(q_start[i], q_end[i], adjusted, dt);
    }
  }

  // Step 3: Resample all axes to uniform dt grid aligned to T_max
  int num_steps = static_cast<int>(std::round(T_max / dt));
  std::vector<std::vector<double>> result;
  result.reserve(num_steps + 1);

  for (int step = 0; step <= num_steps; ++step) {
    double t = std::min(step * dt, T_max);
    std::vector<double> positions(n);
    for (size_t i = 0; i < n; ++i) {
      auto& traj = axis_trajectories[i];
      // Linear scan for t (trajectories are sorted by time)
      size_t idx = 0;
      while (idx + 1 < traj.size() && traj[idx + 1].t < t - 1e-12) {
        ++idx;
      }
      if (idx + 1 >= traj.size()) {
        positions[i] = traj.back().pos;
      } else {
        double t0 = traj[idx].t;
        double t1 = traj[idx + 1].t;
        double alpha = (t1 - t0) > 1e-12 ? (t - t0) / (t1 - t0) : 0.0;
        alpha = std::clamp(alpha, 0.0, 1.0);
        positions[i] = traj[idx].pos + alpha * (traj[idx + 1].pos - traj[idx].pos);
      }
    }
    result.push_back(positions);
  }

  return result;
}
```

- [ ] **Step 2: Commit**

```bash
git add src/robot_control_cpp/src/trajectory_planner.cpp
git commit -m "feat: add TrajectoryPlanner multi-axis synchronization"
```

---

### Task 4: Test SCurvePlanner and TrajectoryPlanner

**Files:**
- Create: `src/robot_control_test/test/test_trajectory_planner.cpp`
- Modify: `src/robot_control_test/CMakeLists.txt`

- [ ] **Step 1: Write test_trajectory_planner.cpp**

Following the same hand-rolled test pattern as `test_ik_solver.cpp`:

```cpp
#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "robot_control_cpp/trajectory_planner.hpp"

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
  std::cout << "=== TrajectoryPlanner Tests ===" << std::endl;

  SCurveConfig cfg{1.0, 2.0, 10.0};  // v=1, a=2, j=10
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
    check(approx_eq(traj.back().vel, 0.0, 0.01), "End vel ≈ 0");
    check(approx_eq(traj.back().acc, 0.0, 0.01), "End acc ≈ 0");

    // Check velocity never exceeds max_vel
    bool vel_ok = true;
    for (auto& p : traj) {
      if (std::abs(p.vel) > cfg.max_vel + 0.01) {
        vel_ok = false;
        break;
      }
    }
    check(vel_ok, "Max velocity not exceeded");

    // Check acceleration never exceeds max_acc
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
    check(approx_eq(traj.back().vel, 0.0, 0.01), "End vel ≈ 0");
  }

  // ---- 4. No cruise phase (medium displacement) ----
  std::cout << "\n--- 4. No cruise phase ---" << std::endl;
  {
    // Use small displacement that exceeds d_min_a but not d_full
    // d_min_a = a_max²/j_max = 4/10 = 0.4
    // d_full = 2*(v*a/j + v²/(2a)) = 2*(1*2/10 + 1/(4)) = 2*(0.2+0.25) = 0.9
    auto traj = robot_control::SCurvePlanner::plan(0.0, 0.6, cfg, dt);
    check(!traj.empty(), "Trajectory not empty");
    check(approx_eq(traj.front().pos, 0.0, 1e-6), "Start at 0");
    check(approx_eq(traj.back().pos, 0.6, 1e-2), "End at 0.6");
    check(approx_eq(traj.back().vel, 0.0, 0.01), "End vel ≈ 0");

    // Peak velocity should be less than max_vel
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
    check(approx_eq(traj.back().vel, 0.0, 0.01), "End vel ≈ 0");
    check(approx_eq(traj.back().acc, 0.0, 0.01), "End acc ≈ 0");
  }

  // ---- 6. Jerk continuity ----
  std::cout << "\n--- 6. Jerk continuity (acc smoothness) ---" << std::endl;
  {
    auto traj = robot_control::SCurvePlanner::plan(0.0, 5.0, cfg, dt);
    // Check that acceleration changes smoothly between adjacent samples
    // The max acc change between consecutive samples should be bounded by j_max*dt
    bool smooth = true;
    for (size_t i = 1; i < traj.size(); ++i) {
      double delta_acc = std::abs(traj[i].acc - traj[i-1].acc);
      double max_delta = cfg.max_jerk * dt * 1.5;  // tolerance for discretization
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
    std::vector<robot_control::SCurveConfig> configs = {
      {1.0, 2.0, 10.0},
      {1.0, 2.0, 10.0},
      {1.0, 2.0, 10.0}
    };

    auto traj = robot_control::TrajectoryPlanner::plan_joint(q_start, q_end, configs, dt);
    check(!traj.empty(), "Multi-axis trajectory not empty");

    // First point should be at start
    bool start_ok = true;
    for (size_t i = 0; i < 3; ++i) {
      if (!approx_eq(traj.front()[i], q_start[i], 0.01)) start_ok = false;
    }
    check(start_ok, "Multi-axis start at q_start");

    // Last point should be at end
    bool end_ok = true;
    for (size_t i = 0; i < 3; ++i) {
      if (!approx_eq(traj.back()[i], q_end[i], 0.05)) end_ok = false;
    }
    check(end_ok, "Multi-axis end at q_end");

    // All axes should have the same number of points
    check(traj.size() > 10, "Trajectory has multiple steps");
  }

  // ---- 8. Config with different axis limits ----
  std::cout << "\n--- 8. Different axis limits ---" << std::endl;
  {
    std::vector<double> q_start = {0.0, 0.0};
    std::vector<double> q_end = {1.0, 1.0};
    std::vector<robot_control::SCurveConfig> configs = {
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
```

- [ ] **Step 2: Add to robot_control_test/CMakeLists.txt**

```cmake
add_executable(test_trajectory_planner
  test/test_trajectory_planner.cpp
)
target_link_libraries(test_trajectory_planner robot_control_cpp::robot_control_core)
install(TARGETS test_trajectory_planner
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
```

- [ ] **Step 3: Build and run test**

```bash
colcon build --base-paths src --packages-select robot_control_cpp robot_control_test
./install/robot_control_test/lib/robot_control_test/test_trajectory_planner
```

- [ ] **Step 4: Fix any failing tests until all pass**

- [ ] **Step 5: Commit**

```bash
git add src/robot_control_test/
git commit -m "test: add standalone tests for SCurvePlanner and TrajectoryPlanner"
```

---

### Task 5: Panda Profile Motion Limits

**Files:**
- Modify: `src/robot_control_cpp/include/robot_control_cpp/panda_profile.hpp`

- [ ] **Step 1: Add motion limits to panda() profile**

Add before `return p;` in `panda()`:

```cpp
// 关节空间运动极限（Franka Panda 典型值）
p.joint_limits = MotionLimits{1.0, 2.0, 10.0};  // rad/s, rad/s², rad/s³

// 笛卡尔空间运动极限
p.cartesian_limits = MotionLimits{0.5, 1.0, 5.0};  // m/s, m/s², m/s³
```

These are conservative defaults. Users can override per application.

- [ ] **Step 2: Commit**

```bash
git add src/robot_control_cpp/include/robot_control_cpp/panda_profile.hpp
git commit -m "feat: add Panda joint and cartesian motion limits"
```

---

### Task 6: Implement moveJ/moveL/set_speed in RobotMotionController

**Files:**
- Modify: `src/robot_control_cpp/include/robot_control_cpp/robot_motion_controller.hpp`
- Modify: `src/robot_control_cpp/src/robot_motion_controller.cpp`
- Modify: `src/robot_control_cpp/CMakeLists.txt`

- [ ] **Step 1: Add to robot_motion_controller.hpp**

New includes:
```cpp
#include "robot_control_cpp/trajectory_planner.hpp"
```

New member variables:
```cpp
double movej_speed_ = 50.0;
double movel_speed_ = 50.0;
```

New method declarations:
```cpp
void moveJ(const std::vector<double>& target_angles, bool block = true) override;
void moveL(const std::array<double, 3>& xyz,
           const std::optional<std::array<double, 3>>& rpy = std::nullopt,
           double finger = -1.0, bool block = true) override;
void set_speed(MotionMode mode, double percent) override;
double get_speed(MotionMode mode) const override;
```

- [ ] **Step 2: Add trajectory_planner.cpp to CMakeLists.txt**

In `add_library(robot_control_core SHARED ...)` add:
```cmake
src/trajectory_planner.cpp
```

- [ ] **Step 3: Implement set_speed/get_speed in robot_motion_controller.cpp**

```cpp
void RobotMotionController::set_speed(MotionMode mode, double percent) {
  if (percent <= 0) {
    throw std::invalid_argument("set_speed: percent must be > 0, got " +
                                std::to_string(percent));
  }
  double clamped = std::min(percent, 100.0);
  switch (mode) {
    case MotionMode::kMoveJ: movej_speed_ = clamped; break;
    case MotionMode::kMoveL: movel_speed_ = clamped; break;
    default: break;
  }
}

double RobotMotionController::get_speed(MotionMode mode) const {
  switch (mode) {
    case MotionMode::kMoveJ: return movej_speed_;
    case MotionMode::kMoveL: return movel_speed_;
    default: return 50.0;
  }
}
```

- [ ] **Step 4: Implement moveJ**

```cpp
void RobotMotionController::moveJ(
    const std::vector<double>& target_angles, bool block) {
  if (static_cast<int>(target_angles.size()) != profile_.dof) {
    throw std::invalid_argument(
        "moveJ: expected " + std::to_string(profile_.dof) +
        " joint angles, got " + std::to_string(target_angles.size()));
  }

  auto q_start = bridge_->get_current_arm();
  double ratio = movej_speed_ / 100.0;

  // Build per-axis configs from profile limits scaled by speed %
  std::vector<SCurveConfig> configs(profile_.dof);
  for (int i = 0; i < profile_.dof; ++i) {
    configs[i] = {
      profile_.joint_limits.max_vel * ratio,
      profile_.joint_limits.max_acc * ratio,
      profile_.joint_limits.max_jerk * ratio
    };
  }

  auto trajectory = TrajectoryPlanner::plan_joint(
      q_start, target_angles, configs, ControlConstants::kTrajectoryDt);

  double finger = grasping_ ? gripper_.min_width
                            : bridge_->get_current_finger();

  // Skip first point (it's the current position)
  for (size_t i = 1; i < trajectory.size(); ++i) {
    bridge_->publish_command(trajectory[i], finger);
    std::this_thread::sleep_for(
        std::chrono::duration<double>(ControlConstants::kTrajectoryDt));
  }

  if (block) {
    bridge_->wait_for_motion(
        target_angles, finger,
        ControlConstants::kJointTolerance,
        ControlConstants::kFingerTolerance,
        ControlConstants::kMotionTimeout,
        ControlConstants::kPollInterval,
        ControlConstants::kSettleTime,
        !grasping_);
  }
}
```

- [ ] **Step 5: Implement moveL**

moveL applies S-curve profiling to the **Cartesian alpha parameter** (0→1), not to the joint space directly. This ensures the Cartesian path remains linear while still having smooth acceleration/deceleration.

```cpp
void RobotMotionController::moveL(
    const std::array<double, 3>& xyz,
    const std::optional<std::array<double, 3>>& rpy,
    double finger, bool block) {
  auto current_pose = get_end_effector_pose();
  Eigen::Vector3d start_pos(current_pose[0], current_pose[1], current_pose[2]);
  Eigen::Vector3d end_pos(xyz[0], xyz[1], xyz[2]);

  double total_dist = (end_pos - start_pos).norm();
  if (total_dist < 1e-6) {
    return;
  }

  double ratio = movel_speed_ / 100.0;
  double cart_vel = profile_.cartesian_limits.max_vel * ratio;
  double cart_acc = profile_.cartesian_limits.max_acc * ratio;
  double cart_jerk = profile_.cartesian_limits.max_jerk * ratio;

  // Plan S-curve on the normalized path parameter alpha ∈ [0, 1]
  // alpha is treated as "position" with velocity in [0, 1/s], etc.
  // Scale: alpha maps linearly to distance, so alpha_vel = cart_vel / total_dist
  SCurveConfig alpha_cfg;
  alpha_cfg.max_vel = cart_vel / total_dist;
  alpha_cfg.max_acc = cart_acc / total_dist;
  alpha_cfg.max_jerk = cart_jerk / total_dist;

  auto alpha_traj = SCurvePlanner::plan(0.0, 1.0, alpha_cfg, ControlConstants::kTrajectoryDt);

  // Current orientation as quaternion
  Eigen::Vector3d start_rpy(current_pose[3], current_pose[4], current_pose[5]);
  Eigen::Quaterniond start_quat =
      (Eigen::AngleAxisd(start_rpy.x(), Eigen::Vector3d::UnitX()) *
       Eigen::AngleAxisd(start_rpy.y(), Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(start_rpy.z(), Eigen::Vector3d::UnitZ()));

  // Target orientation
  Eigen::Quaterniond end_quat = start_quat;
  if (rpy.has_value()) {
    end_quat =
        (Eigen::AngleAxisd((*rpy)[0], Eigen::Vector3d::UnitX()) *
         Eigen::AngleAxisd((*rpy)[1], Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd((*rpy)[2], Eigen::Vector3d::UnitZ()));
  }

  // Actual finger target
  double actual_finger;
  if (finger < 0) {
    actual_finger = grasping_ ? gripper_.min_width
                              : bridge_->get_current_finger();
  } else {
    actual_finger = finger;
  }

  auto tcp_offset = tcp_transform_matrix();
  std::vector<double> final_angles;

  // For each S-curve sample, compute Cartesian pose then solve IK
  for (size_t i = 1; i < alpha_traj.size(); ++i) {
    double alpha = alpha_traj[i].pos;

    // Position: linear interpolation (guarantees straight line)
    Eigen::Vector3d interp_pos = start_pos + alpha * (end_pos - start_pos);

    // Orientation: slerp
    Eigen::Quaterniond interp_quat = start_quat.slerp(alpha, end_quat);

    // Convert to hand target (accounting for TCP offset)
    Eigen::Matrix4d target = Eigen::Matrix4d::Identity();
    target.block<3, 3>(0, 0) = interp_quat.toRotationMatrix();
    target(0, 3) = interp_pos.x();
    target(1, 3) = interp_pos.y();
    target(2, 3) = interp_pos.z();

    Eigen::Matrix4d hand_target = target * tcp_offset.inverse();
    Eigen::Vector3d hand_xyz = hand_target.block<3, 1>(0, 3);
    Eigen::Vector3d hand_rpy = hand_target.block<3, 3>(0, 0).eulerAngles(0, 1, 2);

    std::array<double, 3> h_xyz = {hand_xyz.x(), hand_xyz.y(), hand_xyz.z()};
    std::array<double, 3> h_rpy = {hand_rpy.x(), hand_rpy.y(), hand_rpy.z()};

    auto ik_result = ik_->solve(h_xyz, h_rpy);
    if (!ik_result) {
      throw std::runtime_error(
          "moveL: IK failed at path interpolation point " +
          std::to_string(i) + "/" + std::to_string(alpha_traj.size()) +
          ", alpha=" + std::to_string(alpha));
    }

    final_angles = *ik_result;
    bridge_->publish_command(final_angles, actual_finger);
    std::this_thread::sleep_for(
        std::chrono::duration<double>(ControlConstants::kTrajectoryDt));
  }

  if (block && !final_angles.empty()) {
    bridge_->wait_for_motion(
        final_angles, actual_finger,
        ControlConstants::kJointTolerance,
        ControlConstants::kFingerTolerance,
        ControlConstants::kMotionTimeout,
        ControlConstants::kPollInterval,
        ControlConstants::kSettleTime,
        !grasping_);
  }
}
```

- [ ] **Step 6: Build and verify compilation**

```bash
colcon build --base-paths src --packages-select robot_control_cpp
```

- [ ] **Step 7: Commit**

```bash
git add src/robot_control_cpp/
git commit -m "feat: implement moveJ/moveL with S-curve trajectory and speed control"
```

---

### Task 7: Python Bindings

**Files:**
- Modify: `src/robot_control_cpp_py/src/bindings.cpp`

- [ ] **Step 1: Add bindings for MotionMode, MotionLimits, and new methods**

After the `GraspState` enum binding, add:

```cpp
// MotionMode 枚举
py::enum_<MotionMode>(m, "MotionMode")
    .value("MOVE_J", MotionMode::kMoveJ)
    .value("MOVE_L", MotionMode::kMoveL)
    .export_values();
```

After `GripperProfile` binding, add:

```cpp
// MotionLimits
py::class_<MotionLimits>(m, "MotionLimits")
    .def(py::init<>())
    .def_readwrite("max_vel", &MotionLimits::max_vel)
    .def_readwrite("max_acc", &MotionLimits::max_acc)
    .def_readwrite("max_jerk", &MotionLimits::max_jerk);
```

Add to `RobotProfile` binding:
```cpp
.def_readwrite("joint_limits", &RobotProfile::joint_limits)
.def_readwrite("cartesian_limits", &RobotProfile::cartesian_limits)
```

Add to `RobotMotionController` binding:
```cpp
.def("moveJ", &RobotMotionController::moveJ,
     py::arg("target_angles"), py::arg("block") = true,
     py::call_guard<py::gil_scoped_release>())
.def("moveL", &RobotMotionController::moveL,
     py::arg("xyz"), py::arg("rpy") = py::none(),
     py::arg("finger") = -1.0, py::arg("block") = true,
     py::call_guard<py::gil_scoped_release>())
.def("set_speed", &RobotMotionController::set_speed,
     py::arg("mode"), py::arg("percent"))
.def("get_speed", &RobotMotionController::get_speed,
     py::arg("mode"))
```

Add `kTrajectoryDt` to ControlConstants exports:
```cpp
m.attr("TRAJECTORY_DT") = ControlConstants::kTrajectoryDt;
```

- [ ] **Step 2: Build bindings**

```bash
colcon build --base-paths src --packages-up-to robot_control_cpp_py
```

- [ ] **Step 3: Commit**

```bash
git add src/robot_control_cpp_py/
git commit -m "feat: add Python bindings for moveJ/moveL and speed control"
```

---

### Task 8: Integration Verification

**Files:** None new

- [ ] **Step 1: Full build**

```bash
colcon build --base-paths src --packages-up-to robot_control_cpp_py robot_control_test
```

- [ ] **Step 2: Run standalone tests**

```bash
./install/robot_control_test/lib/robot_control_test/test_trajectory_planner
./install/robot_control_test/lib/robot_control_test/test_ik_solver
```

- [ ] **Step 3: Verify Python import**

```bash
python3 -c "
import robot_control_cpp_py as rc
# Check new types exist
print('MotionMode:', rc.MotionMode.MOVE_J, rc.MotionMode.MOVE_L)
print('MotionLimits:', rc.MotionLimits())
p = rc.profiles.panda()
print('joint_limits:', p.joint_limits.max_vel, p.joint_limits.max_acc, p.joint_limits.max_jerk)
print('cartesian_limits:', p.cartesian_limits.max_vel, p.cartesian_limits.max_acc, p.cartesian_limits.max_jerk)
print('TRAJECTORY_DT:', rc.TRAJECTORY_DT)
print('OK')
"
```

- [ ] **Step 4: Final commit if any fixes needed**

```bash
git add -A
git commit -m "fix: address integration issues from verification"
```
