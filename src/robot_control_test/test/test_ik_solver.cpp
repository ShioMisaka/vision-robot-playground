/// @file test_ik_solver.cpp
/// @brief IK 求解器独立测试（无需 ROS2 / Isaac Sim）
///
/// 编译后直接运行：
///   ./test_ik_solver
///
/// 验证内容：
///   1. URDF 加载 + KDL 链构建
///   2. 正运动学（FK）计算
///   3. 逆运动学（IK）求解 + FK 验证闭环一致性

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "robot_control_cpp/ik_solver.hpp"
#include "robot_control_cpp/panda_profile.hpp"

namespace {

/// 比较两个 double，容差 eps
bool approx_eq(double a, double b, double eps = 1e-3) {
  return std::abs(a - b) < eps;
}

/// 比较两个 std::vector<double>
bool approx_vec_eq(const std::vector<double>& a,
                   const std::vector<double>& b, double eps = 1e-3) {
  if (a.size() != b.size()) return false;
  for (size_t i = 0; i < a.size(); ++i) {
    if (!approx_eq(a[i], b[i], eps)) return false;
  }
  return true;
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
  std::cout << "=== IK Solver 独立测试 ===" << std::endl;

  // ---- 1. 构造求解器 ----
  std::cout << "\n--- 1. 构造求解器 ---" << std::endl;
  auto profile = robot_control::profiles::panda();
  std::unique_ptr<robot_control::IKSolver> solver;

  try {
    solver = std::make_unique<robot_control::IKSolver>(profile);
    check(true, "IKSolver 构造成功");
  } catch (const std::exception& e) {
    check(false, std::string("IKSolver 构造失败: ") + e.what());
    return 1;
  }

  check(solver->get_dof() == 7, "DOF == 7");

  // ---- 2. 正运动学（全零关节角 → 默认位姿）----
  std::cout << "\n--- 2. 正运动学（全零） ---" << std::endl;
  std::vector<double> zero_joints(7, 0.0);
  auto fk_zero = solver->forward(zero_joints);

  std::cout << "  FK(zero): pos=(" << fk_zero[0] << ", " << fk_zero[1]
            << ", " << fk_zero[2] << ")  rpy=(" << fk_zero[3] << ", "
            << fk_zero[4] << ", " << fk_zero[5] << ")" << std::endl;

  // Panda 全零位 z ≈ 0.926m（臂完全伸直向上）
  check(fk_zero[2] > 0.2 && fk_zero[2] < 1.0,
        "FK(zero) z 在合理范围 (0.2~1.0m)");

  // ---- 3. IK → FK 闭环一致性（有姿态约束）----
  std::cout << "\n--- 3. IK→FK 闭环（全姿态约束） ---" << std::endl;

  // 用 FK 得到一个已知位姿
  std::vector<double> test_joints = {0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785};
  auto fk_known = solver->forward(test_joints);
  std::cout << "  FK(known): pos=(" << fk_known[0] << ", " << fk_known[1]
            << ", " << fk_known[2] << ")  rpy=(" << fk_known[3] << ", "
            << fk_known[4] << ", " << fk_known[5] << ")" << std::endl;

  // 用该位姿做 IK
  std::array<double, 3> target_xyz = {fk_known[0], fk_known[1], fk_known[2]};
  std::array<double, 3> target_rpy = {fk_known[3], fk_known[4], fk_known[5]};
  auto ik_result = solver->solve(target_xyz, target_rpy);

  check(ik_result.has_value(), "IK 求解成功");

  if (ik_result.has_value()) {
    // 用 IK 结果做 FK，验证位姿一致
    auto fk_verify = solver->forward(*ik_result);

    bool pos_ok = approx_eq(fk_verify[0], fk_known[0]) &&
                  approx_eq(fk_verify[1], fk_known[1]) &&
                  approx_eq(fk_verify[2], fk_known[2]);
    check(pos_ok, "IK→FK 位置闭环误差 < 1mm");

    bool rpy_ok = approx_eq(fk_verify[3], fk_known[3]) &&
                  approx_eq(fk_verify[4], fk_known[4]) &&
                  approx_eq(fk_verify[5], fk_known[5]);
    check(rpy_ok, "IK→FK 姿态闭环误差 < 1°");

    // 验证关节角差异不大
    bool joints_close = true;
    for (int i = 0; i < 7; ++i) {
      if (std::abs((*ik_result)[i] - test_joints[i]) > 0.3) {
        joints_close = false;
        break;
      }
    }
    check(joints_close, "IK 结果与原始关节角偏差合理（< 0.3rad）");
  }

  // ---- 4. 仅位置约束 IK ----
  std::cout << "\n--- 4. 仅位置约束 IK ---" << std::endl;
  auto ik_pos_only = solver->solve(target_xyz, std::nullopt);
  check(ik_pos_only.has_value(), "仅位置约束 IK 求解成功");

  if (ik_pos_only.has_value()) {
    auto fk_pos = solver->forward(*ik_pos_only);
    bool pos_ok = approx_eq(fk_pos[0], fk_known[0]) &&
                  approx_eq(fk_pos[1], fk_known[1]) &&
                  approx_eq(fk_pos[2], fk_known[2]);
    check(pos_ok, "仅位置 IK→FK 位置闭环误差 < 1mm");
  }

  // ---- 5. forward_matrix 返回 4x4 矩阵 ----
  std::cout << "\n--- 5. forward_matrix ---" << std::endl;
  auto mat = solver->forward_matrix(test_joints);
  check(mat.rows() == 4 && mat.cols() == 4, "矩阵尺寸 4x4");
  check(approx_eq(mat(0, 3), fk_known[0]) &&
        approx_eq(mat(1, 3), fk_known[1]) &&
        approx_eq(mat(2, 3), fk_known[2]),
        "forward_matrix 位置与 forward 一致");

  // ---- 6. 异常输入 ----
  std::cout << "\n--- 6. 异常输入 ---" << std::endl;
  // 不可达位姿应返回 nullopt
  auto ik_unreachable = solver->solve({100.0, 0.0, 0.0});
  check(!ik_unreachable.has_value(), "不可达位姿返回 nullopt");

  // ---- 汇总 ----
  std::cout << "\n=== 结果: " << pass_count << "/" << test_count
            << " 通过 ===" << std::endl;

  return (pass_count == test_count) ? 0 : 1;
}
