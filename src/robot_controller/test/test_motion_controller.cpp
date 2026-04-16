/// @file test_motion_controller.cpp
/// @brief RobotMotionController 单元测试（无需 ROS2 / Isaac Sim）
///
/// 使用 MockMotionBridge 模拟底层 IO，验证 moveJ/moveL/set_speed/get_speed
///
/// 编译后直接运行：
///   ./test_motion_controller
///
/// 测试内容：
///   1. set_speed / get_speed 基本功能与边界
///   2. moveJ 轨迹命令数量与首末端值
///   3. moveJ 速度百分比影响轨迹时长
///   4. moveJ 抓取状态下夹爪维持
///   5. moveJ 参数校验（DOF 不匹配）
///   6. moveL 笛卡尔直线运动
///   7. moveL 速度百分比影响轨迹时长

#include <cmath>
#include <iostream>
#include <mutex>
#include <string>
#include <vector>
#include <array>
#include <optional>

#include "robot_controller/motion/robot_motion_controller.hpp"
#include "robot_controller/kinematics/ik_solver.hpp"
#include "robot_controller/profiles/panda_profile.hpp"

namespace {

bool approx_eq(double a, double b, double eps = 1e-3) {
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

// ===== MockMotionBridge =====

struct Command {
  std::vector<double> arm;
  double finger;
};

class MockMotionBridge : public robot_control::MotionIOBridge {
public:
  explicit MockMotionBridge(const std::vector<double>& initial_arm,
                            double initial_finger = 0.04)
      : arm_(initial_arm), finger_(initial_finger) {}

  void publish_command(const std::vector<double>& arm,
                       double finger) override {
    std::lock_guard<std::mutex> lock(mutex_);
    commands_.push_back({arm, finger});
    arm_ = arm;
    finger_ = finger;
  }

  std::vector<double> get_current_arm() const override {
    std::lock_guard<std::mutex> lock(mutex_);
    return arm_;
  }

  double get_current_finger() const override {
    std::lock_guard<std::mutex> lock(mutex_);
    return finger_;
  }

  bool wait_for_motion(const std::vector<double>& /*target_arm*/,
                       double /*finger*/, double /*joint_tol*/,
                       double /*finger_tol*/, double /*timeout*/,
                       double /*poll_interval*/, double /*settle_time*/,
                       bool /*check_finger*/) override {
    return true;
  }

  bool wait_for_finger_settle(int /*stable_count*/, double /*tol*/,
                              double /*poll_interval*/,
                              double /*timeout*/) override {
    return true;
  }

  std::optional<std::array<double, 6>> lookup_transform(
      const std::string& /*target_frame*/,
      const std::string& /*source_frame*/,
      double /*timeout*/) override {
    return std::nullopt;
  }

  void set_tcp_name(const std::string& /*name*/) override {}

  // 测试辅助方法
  const std::vector<Command>& commands() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return commands_;
  }

  void clear_commands() {
    std::lock_guard<std::mutex> lock(mutex_);
    commands_.clear();
  }

  size_t command_count() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return commands_.size();
  }

 private:
  mutable std::mutex mutex_;
  std::vector<double> arm_;
  double finger_;
  std::vector<Command> commands_;
};

// 创建控制器和桥接的工厂函数
struct TestFixture {
  std::shared_ptr<robot_control::IKSolver> ik;
  robot_control::RobotProfile profile;
  robot_control::GripperProfile gripper;
  std::shared_ptr<MockMotionBridge> bridge;
  std::shared_ptr<robot_control::RobotMotionController> controller;

  explicit TestFixture(const std::vector<double>& initial_arm)
      : profile(robot_control::profiles::panda()),
        gripper(robot_control::profiles::panda_gripper()),
        bridge(std::make_shared<MockMotionBridge>(initial_arm)) {
    ik = std::make_shared<robot_control::IKSolver>(profile);
    controller = std::make_shared<robot_control::RobotMotionController>(
        ik, profile, gripper, bridge);
  }
};

}  // namespace

int main() {
  std::cout << "=== RobotMotionController 单元测试 ===" << std::endl;

  // ---- 1. set_speed / get_speed ----
  std::cout << "\n--- 1. set_speed / get_speed ---" << std::endl;
  {
    auto profile = robot_control::profiles::panda();
    auto gripper = robot_control::profiles::panda_gripper();
    auto bridge = std::make_shared<MockMotionBridge>(
        profile.home_joints);
    auto ik = std::make_shared<robot_control::IKSolver>(profile);
    auto ctrl = std::make_shared<robot_control::RobotMotionController>(
        ik, profile, gripper, bridge);

    // 默认速度
    check(approx_eq(ctrl->get_speed(robot_control::MotionMode::kMoveJ), 50.0),
          "默认 moveJ 速度 = 50%");
    check(approx_eq(ctrl->get_speed(robot_control::MotionMode::kMoveL), 50.0),
          "默认 moveL 速度 = 50%");

    // 设置速度
    ctrl->set_speed(robot_control::MotionMode::kMoveJ, 80.0);
    check(approx_eq(ctrl->get_speed(robot_control::MotionMode::kMoveJ), 80.0),
          "set_speed(moveJ, 80) → get_speed = 80%");

    ctrl->set_speed(robot_control::MotionMode::kMoveL, 30.0);
    check(approx_eq(ctrl->get_speed(robot_control::MotionMode::kMoveL), 30.0),
          "set_speed(moveL, 30) → get_speed = 30%");

    // moveJ/moveL 速度独立
    check(approx_eq(ctrl->get_speed(robot_control::MotionMode::kMoveJ), 80.0),
          "moveJ 速度不受 moveL 影响");

    // >100 钳位到 100
    ctrl->set_speed(robot_control::MotionMode::kMoveJ, 150.0);
    check(approx_eq(ctrl->get_speed(robot_control::MotionMode::kMoveJ), 100.0),
          "set_speed(moveJ, 150) → 钳位到 100%");

    // =0 抛异常
    bool threw = false;
    try {
      ctrl->set_speed(robot_control::MotionMode::kMoveJ, 0.0);
    } catch (const std::invalid_argument&) {
      threw = true;
    }
    check(threw, "set_speed(moveJ, 0) 抛出 invalid_argument");

    // <0 抛异常
    threw = false;
    try {
      ctrl->set_speed(robot_control::MotionMode::kMoveL, -10.0);
    } catch (const std::invalid_argument&) {
      threw = true;
    }
    check(threw, "set_speed(moveL, -10) 抛出 invalid_argument");
  }

  // ---- 2. moveJ 轨迹基本验证 ----
  std::cout << "\n--- 2. moveJ 轨迹基本验证 ---" << std::endl;
  {
    auto profile = robot_control::profiles::panda();
    auto home = std::vector<double>(
        profile.home_joints.begin(),
        profile.home_joints.begin() + profile.dof);
    TestFixture tf(home);

    // 目标：关节1 旋转 0.2 rad
    auto target = home;
    target[0] += 0.2;

    tf.bridge->clear_commands();
    tf.controller->moveJ(target, true);

    auto& cmds = tf.bridge->commands();
    check(!cmds.empty(), "moveJ 产生了命令序列");

    // 第一个命令应接近起始位置（跳过第 0 步 = 当前位置）
    bool first_close = true;
    for (int i = 0; i < 7; ++i) {
      if (std::abs(cmds[0].arm[i] - home[i]) > 0.05) {
        first_close = false;
        break;
      }
    }
    check(first_close, "第一个命令接近起始位姿");

    // 最后一个命令应接近目标
    bool last_close = true;
    for (int i = 0; i < 7; ++i) {
      if (std::abs(cmds.back().arm[i] - target[i]) > 0.01) {
        last_close = false;
        break;
      }
    }
    check(last_close, "最后一个命令接近目标位姿");

    std::cout << "  moveJ 命令数量: " << cmds.size() << std::endl;
  }

  // ---- 3. moveJ 速度影响轨迹时长 ----
  std::cout << "\n--- 3. moveJ 速度影响轨迹时长 ---" << std::endl;
  {
    auto profile = robot_control::profiles::panda();
    auto home = std::vector<double>(
        profile.home_joints.begin(),
        profile.home_joints.begin() + profile.dof);
    auto target = home;
    target[0] += 0.3;

    // 100% 速度
    TestFixture tf_fast(home);
    tf_fast.controller->set_speed(robot_control::MotionMode::kMoveJ, 100.0);
    tf_fast.bridge->clear_commands();
    tf_fast.controller->moveJ(target, true);
    size_t fast_count = tf_fast.bridge->command_count();

    // 25% 速度
    TestFixture tf_slow(home);
    tf_slow.controller->set_speed(robot_control::MotionMode::kMoveJ, 25.0);
    tf_slow.bridge->clear_commands();
    tf_slow.controller->moveJ(target, true);
    size_t slow_count = tf_slow.bridge->command_count();

    std::cout << "  100% 速度命令数: " << fast_count
              << ", 25% 速度命令数: " << slow_count << std::endl;
    check(slow_count > fast_count,
          "低速(25%)产生更多命令点（轨迹更长）");

    // 慢速应至少是快速的 2 倍（理论上 ~4 倍，但考虑浮点取 2 倍）
    check(slow_count >= fast_count * 2,
          "低速命令数 >= 快速的 2 倍");
  }

  // ---- 4. moveJ 抓取状态下夹爪维持 min_width ----
  std::cout << "\n--- 4. moveJ 抓取状态下夹爪维持 ---" << std::endl;
  {
    auto profile = robot_control::profiles::panda();
    auto home = std::vector<double>(
        profile.home_joints.begin(),
        profile.home_joints.begin() + profile.dof);
    TestFixture tf(home);

    // 先闭合夹爪（设置 grasping_ = true）
    tf.controller->close_gripper(true);
    check(tf.bridge->get_current_finger() <= tf.gripper.min_width + 0.001,
          "close_gripper 后 finger 接近 min_width");

    // moveJ 时应保持 min_width
    auto target = home;
    target[1] += 0.1;
    tf.bridge->clear_commands();
    tf.controller->moveJ(target, true);

    auto& cmds = tf.bridge->commands();
    bool all_min = true;
    for (const auto& cmd : cmds) {
      if (std::abs(cmd.finger - tf.gripper.min_width) > 0.001) {
        all_min = false;
        break;
      }
    }
    check(all_min, "抓取状态下 moveJ 所有命令 finger = min_width");
  }

  // ---- 5. moveJ 参数校验 ----
  std::cout << "\n--- 5. moveJ 参数校验 ---" << std::endl;
  {
    auto profile = robot_control::profiles::panda();
    auto home = std::vector<double>(
        profile.home_joints.begin(),
        profile.home_joints.begin() + profile.dof);
    TestFixture tf(home);

    // DOF 不匹配
    bool threw = false;
    try {
      tf.controller->moveJ({0.0, 0.0, 0.0}, true);
    } catch (const std::invalid_argument&) {
      threw = true;
    }
    check(threw, "moveJ(3 DOF) 抛出 invalid_argument");
  }

  // ---- 6. moveL 笛卡尔直线运动 ----
  std::cout << "\n--- 6. moveL 笛卡尔直线运动 ---" << std::endl;
  {
    auto profile = robot_control::profiles::panda();
    auto home = std::vector<double>(
        profile.home_joints.begin(),
        profile.home_joints.begin() + profile.dof);
    TestFixture tf(home);

    // 获取当前末端位姿
    auto pose = tf.controller->get_end_effector_pose();
    std::cout << "  当前 TCP: pos=(" << pose[0] << ", " << pose[1]
              << ", " << pose[2] << ")" << std::endl;

    // moveL: Z 方向抬升 0.05m
    std::array<double, 3> target_xyz = {pose[0], pose[1], pose[2] + 0.05};
    std::array<double, 3> target_rpy = {pose[3], pose[4], pose[5]};

    tf.bridge->clear_commands();
    tf.controller->moveL(target_xyz, target_rpy, 0.04, true);

    auto& cmds = tf.bridge->commands();
    check(!cmds.empty(), "moveL 产生了命令序列");

    // 最后一个命令经 FK 应接近目标位姿
    auto final_angles = cmds.back().arm;
    std::array<double, 3> tgt_xyz = {target_xyz[0], target_xyz[1], target_xyz[2]};
    std::array<double, 3> tgt_rpy = {target_rpy[0], target_rpy[1], target_rpy[2]};
    auto ik_result = tf.ik->solve(tgt_xyz, tgt_rpy);
    check(ik_result.has_value(), "moveL 目标位姿 IK 可解");

    if (ik_result.has_value()) {
      bool final_close = true;
      for (int i = 0; i < 7; ++i) {
        if (std::abs(final_angles[i] - (*ik_result)[i]) > 0.05) {
          final_close = false;
          break;
        }
      }
      check(final_close, "moveL 末端命令接近目标 IK 解");
    }

    std::cout << "  moveL 命令数量: " << cmds.size() << std::endl;
  }

  // ---- 7. moveL 速度影响轨迹时长 ----
  std::cout << "\n--- 7. moveL 速度影响轨迹时长 ---" << std::endl;
  {
    auto profile = robot_control::profiles::panda();
    auto home = std::vector<double>(
        profile.home_joints.begin(),
        profile.home_joints.begin() + profile.dof);

    // 预先获取位姿用于两个 fixture
    TestFixture tf_tmp(home);
    auto pose = tf_tmp.controller->get_end_effector_pose();

    TestFixture tf1(home);
    pose = tf1.controller->get_end_effector_pose();
    std::array<double, 3> target_xyz = {pose[0], pose[1], pose[2] + 0.08};
    std::array<double, 3> target_rpy = {pose[3], pose[4], pose[5]};

    // 100% 速度
    tf1.controller->set_speed(robot_control::MotionMode::kMoveL, 100.0);
    tf1.bridge->clear_commands();
    tf1.controller->moveL(target_xyz, target_rpy, 0.04, true);
    size_t fast_count = tf1.bridge->command_count();

    // 25% 速度
    TestFixture tf2(home);
    tf2.controller->set_speed(robot_control::MotionMode::kMoveL, 25.0);
    tf2.bridge->clear_commands();
    tf2.controller->moveL(target_xyz, target_rpy, 0.04, true);
    size_t slow_count = tf2.bridge->command_count();

    std::cout << "  100% 速度命令数: " << fast_count
              << ", 25% 速度命令数: " << slow_count << std::endl;
    check(slow_count > fast_count,
          "低速 moveL 产生更多命令点");
    check(slow_count >= static_cast<size_t>(fast_count * 1.3),
          "低速 moveL 命令数 >= 快速的 1.3 倍");
  }

  // ---- 汇总 ----
  std::cout << "\n=== 结果: " << pass_count << "/" << test_count
            << " 通过 ===" << std::endl;

  return (pass_count == test_count) ? 0 : 1;
}
