#include "robot_control_cpp/trajectory_planner.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace robot_control {

namespace {

/// @brief 解析求解阶段内位置
double eval_pos(double p0, double v0, double a0, double j, double tau) {
  return p0 + v0 * tau + 0.5 * a0 * tau * tau
       + (1.0 / 6.0) * j * tau * tau * tau;
}

/// @brief 解析求解阶段内速度
double eval_vel(double v0, double a0, double j, double tau) {
  return v0 + a0 * tau + 0.5 * j * tau * tau;
}

/// @brief 解析求解阶段内加速度
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

  // 所有内部计算使用正位移，sign 仅在输出时乘
  int sign = (h > 0) ? 1 : -1;
  double abs_h = std::abs(h);

  double v_max = cfg.max_vel;
  double a_max = cfg.max_acc;
  double j_max = cfg.max_jerk;

  // ============ 计算各段时间 ============

  // 达到 v_max 所需的最小距离（完整加减速对称距离）
  double d_full = 2.0 * (v_max * a_max / j_max + v_max * v_max / (2.0 * a_max));

  double t1 = 0, t2 = 0, t3 = 0, t4 = 0, t5 = 0, t6 = 0, t7 = 0;

  if (abs_h >= d_full) {
    // ---- 情况 1：完整七段 ----
    t1 = a_max / j_max;
    t3 = t1;
    t5 = t1;
    t7 = t1;

    double v1 = 0.5 * j_max * t1 * t1;
    // v_max = 2*v1 + a_max * t2
    t2 = (v_max - 2.0 * v1) / a_max;
    t6 = t2;

    // 解析计算加速阶段（1-3）的距离
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
    // ---- 情况 2 & 3：最大速度不可达（t4=0）----
    t4 = 0;
    t1 = a_max / j_max;
    t3 = t1;
    t5 = t1;
    t7 = t1;

    // 解二次方程求 v_peak
    // v_peak^2 + 2*v_peak*a_max^2/j_max - h*a_max = 0
    double b_coeff = 2.0 * a_max * a_max / j_max;
    double v_peak = (-b_coeff + std::sqrt(b_coeff * b_coeff + 4.0 * abs_h * a_max)) / 2.0;

    double v1 = 0.5 * j_max * t1 * t1;

    // 检查 t2 是否非负（即 a_max 确实达到）
    if (v_peak >= 2.0 * v1 - 1e-12) {
      // 情况 2：a_max 达到，但 v_max 未达到
      t2 = (v_peak - 2.0 * v1) / a_max;
      if (t2 < 0) t2 = 0;
      t6 = t2;
    } else {
      // 情况 3：连 a_max 也未达到，仅有 jerk 阶段
      // 总距离：h = 2 * j_max * t1^3
      t2 = 0;
      t6 = 0;
      t1 = std::cbrt(abs_h / (2.0 * j_max));
      t3 = t1;
      t5 = t1;
      t7 = t1;
    }
  }

  double T = t1 + t2 + t3 + t4 + t5 + t6 + t7;

  // ============ 使用解析公式采样轨迹 ============

  std::vector<TrajectoryPoint> points;
  points.reserve(static_cast<size_t>(T / dt) + 2);

  // 记录起点
  points.push_back({0.0, q0, 0.0, 0.0});

  // 阶段边界：累积时间
  double t_bounds[8];
  t_bounds[0] = 0;
  t_bounds[1] = t1;
  t_bounds[2] = t1 + t2;
  t_bounds[3] = t1 + t2 + t3;
  t_bounds[4] = t1 + t2 + t3 + t4;
  t_bounds[5] = t1 + t2 + t3 + t4 + t5;
  t_bounds[6] = t1 + t2 + t3 + t4 + t5 + t6;
  t_bounds[7] = T;

  // 正位移 S 曲线各阶段的 jerk
  double jerks[7] = {
    j_max,    // 阶段 1: +jerk
    0.0,      // 阶段 2: 恒定加速
    -j_max,   // 阶段 3: -jerk
    0.0,      // 阶段 4: 匀速巡航
    -j_max,   // 阶段 5: -jerk（减速）
    0.0,      // 阶段 6: 恒定减速
    j_max     // 阶段 7: +jerk（减速到零）
  };

  int num_steps = static_cast<int>(std::ceil(T / dt));
  for (int step = 1; step <= num_steps; ++step) {
    double t = std::min(static_cast<double>(step) * dt, T);

    // 查找当前阶段
    int phase = 0;
    for (int p = 0; p < 7; ++p) {
      if (t >= t_bounds[p] - 1e-12) {
        phase = p;
      }
    }

    // 从 t=0 正向遍历已完成阶段，获取当前阶段的初始条件
    double p0 = 0.0, v0 = 0.0, a0 = 0.0;
    double t_acc = 0.0;
    for (int p = 0; p < phase; ++p) {
      double T_p = t_bounds[p + 1] - t_bounds[p];
      double p_end = eval_pos(p0, v0, a0, jerks[p], T_p);
      double v_end = eval_vel(v0, a0, jerks[p], T_p);
      double a_end = eval_acc(a0, jerks[p], T_p);
      p0 = p_end;
      v0 = v_end;
      a0 = a_end;
      t_acc = t_bounds[p + 1];
    }

    double tau = t - t_acc;
    double j = jerks[phase];

    // 内部解析求值后乘以 sign 得到实际输出
    double pos = static_cast<double>(sign) * eval_pos(p0, v0, a0, j, tau) + q0;
    double vel = static_cast<double>(sign) * eval_vel(v0, a0, j, tau);
    double acc = static_cast<double>(sign) * eval_acc(a0, j, tau);

    points.push_back({t, pos, vel, acc});
  }

  // 强制终止点精确到达目标
  if (!points.empty()) {
    points.back().pos = q1;
    points.back().vel = 0.0;
    points.back().acc = 0.0;
    points.back().t = T;
  }

  return points;
}

std::vector<std::vector<double>> TrajectoryPlanner::plan_joint(
    const std::vector<double>& q_start,
    const std::vector<double>& q_end,
    const std::vector<SCurveConfig>& configs,
    double dt) {
  size_t n = q_start.size();
  if (n != q_end.size() || n != configs.size()) {
    throw std::invalid_argument("plan_joint: dimension mismatch");
  }

  // 步骤 1: 各轴独立规划，找到最长运动时间 T_max
  std::vector<std::vector<TrajectoryPoint>> axis_trajectories(n);
  double T_max = 0.0;

  for (size_t i = 0; i < n; ++i) {
    axis_trajectories[i] = SCurvePlanner::plan(q_start[i], q_end[i], configs[i], dt);
    if (!axis_trajectories[i].empty()) {
      T_max = std::max(T_max, axis_trajectories[i].back().t);
    }
  }

  // 步骤 2: 对较短轴仅降低 max_vel（保持 max_acc、max_jerk 不变）
  // 使用二分搜索使轨迹时间逼近 T_max
  for (size_t i = 0; i < n; ++i) {
    double T_i = axis_trajectories[i].back().t;
    if (T_i < T_max - dt * 0.5) {
      SCurveConfig adjusted = configs[i];
      double v_low = 0.0;
      double v_high = configs[i].max_vel;

      for (int iter = 0; iter < 30; ++iter) {
        double v_mid = 0.5 * (v_low + v_high);
        adjusted.max_vel = v_mid;
        auto trial = SCurvePlanner::plan(q_start[i], q_end[i], adjusted, dt);
        double T_trial = trial.empty() ? 0.0 : trial.back().t;
        if (T_trial < T_max) {
          v_high = v_mid;
        } else {
          v_low = v_mid;
        }
      }
      adjusted.max_vel = 0.5 * (v_low + v_high);
      axis_trajectories[i] = SCurvePlanner::plan(q_start[i], q_end[i], adjusted, dt);
    }
  }

  // 步骤 3: 重采样所有轴到 T_max 对齐的均匀 dt 网格
  int num_steps = static_cast<int>(std::round(T_max / dt));
  std::vector<std::vector<double>> result;
  result.reserve(static_cast<size_t>(num_steps) + 1);

  for (int step = 0; step <= num_steps; ++step) {
    double t = std::min(static_cast<double>(step) * dt, T_max);
    std::vector<double> positions(n);
    for (size_t i = 0; i < n; ++i) {
      auto& traj = axis_trajectories[i];
      size_t idx = 0;
      while (idx + 1 < traj.size() && traj[idx + 1].t < t - 1e-12) {
        ++idx;
      }
      if (idx + 1 >= traj.size()) {
        positions[i] = traj.back().pos;
      } else {
        double t0 = traj[idx].t;
        double t1_time = traj[idx + 1].t;
        double alpha = (t1_time - t0) > 1e-12
                        ? (t - t0) / (t1_time - t0) : 0.0;
        alpha = std::clamp(alpha, 0.0, 1.0);
        positions[i] = traj[idx].pos + alpha * (traj[idx + 1].pos - traj[idx].pos);
      }
    }
    result.push_back(positions);
  }

  return result;
}

}  // namespace robot_control
