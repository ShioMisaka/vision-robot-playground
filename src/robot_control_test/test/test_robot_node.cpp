/// @file test_robot_node.cpp
/// @brief C++ 节点与 Isaac Sim 通信集成测试
///
/// 前提：Isaac Sim 已启动并发布 /joint_states 话题
///
/// 编译后运行：
///   source install/setup.bash
///   ./test_robot_node
///
/// 测试流程：
///   1. 创建 RobotControllerNode，等待关节状态就绪
///   2. 读取当前关节角度
///   3. 张开/闭合夹爪
///   4. 回 home 位
///   5. IK 位姿控制测试（小幅移动）
///   6. 读取末端执行器位姿

#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include "robot_control_cpp/nodes/robot_controller_node.hpp"
#include "robot_control_cpp/nodes/vision_processor_node.hpp"
#include "robot_control_cpp/vision/color_detector.hpp"
#include "robot_control_cpp/panda_profile.hpp"
#include "robot_control_cpp/motion/control_constants.hpp"
#include "robot_control_cpp/nodes/topic_config.hpp"

namespace {

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

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  std::cout << "=== C++ Robot Node 集成测试 ===" << std::endl;
  std::cout << "前提：Isaac Sim 已启动并发布 /joint_states\n" << std::endl;

  // ---- 构造节点 ----
  auto profile = robot_control::profiles::panda();
  auto gripper = robot_control::profiles::panda_gripper();
  robot_control::TopicConfig topics;

  auto robot_node = robot_control::RobotControllerNode::create(
      profile, gripper, topics);

  // 创建 executor 并在后台 spin
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(robot_node);
  auto spin_thread = std::thread([executor]() { executor->spin(); });

  // ---- 1. 等待就绪 ----
  std::cout << "--- 1. 等待关节状态就绪 ---" << std::endl;
  bool ready = robot_node->wait_for_ready(
      robot_control::ControlConstants::kReadyTimeout);
  check(ready, "关节状态就绪（5s 内收到 /joint_states）");

  if (!ready) {
    std::cerr << "未收到关节状态，请确认 Isaac Sim 已启动。测试终止。"
              << std::endl;
    executor->cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 1;
  }

  auto controller = robot_node->get_controller();

  // ---- 2. 读取关节角度 ----
  std::cout << "\n--- 2. 读取关节角度 ---" << std::endl;
  auto angles = controller->get_joint_angles();
  check(static_cast<int>(angles.size()) == 7, "关节角度数量 == 7");

  std::cout << "  当前关节角: ";
  for (size_t i = 0; i < angles.size(); ++i) {
    std::cout << angles[i];
    if (i < angles.size() - 1) std::cout << ", ";
  }
  std::cout << std::endl;

  double finger = controller->get_finger_width();
  std::cout << "  夹爪宽度: " << finger << " m" << std::endl;
  check(finger >= 0.0 && finger <= 0.08, "夹爪宽度在合理范围");

  // ---- 3. 夹爪控制 ----
  std::cout << "\n--- 3. 夹爪控制 ---" << std::endl;
  try {
    controller->open_gripper(true);
    check(true, "open_gripper 成功");
  } catch (const std::exception& e) {
    std::cout << "  [FAIL] open_gripper 异常: " << e.what() << std::endl;
    check(false, "open_gripper");
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  try {
    controller->close_gripper(true);
    check(true, "close_gripper 成功");
  } catch (const std::exception& e) {
    std::cout << "  [FAIL] close_gripper 异常: " << e.what() << std::endl;
    check(false, "close_gripper");
  }

  // ---- 4. 回 home ----
  std::cout << "\n--- 4. 回 home ---" << std::endl;
  try {
    controller->open_gripper(true);
    controller->go_home(true);
    check(true, "go_home 成功");
  } catch (const std::exception& e) {
    std::cout << "  [FAIL] go_home 异常: " << e.what() << std::endl;
    check(false, "go_home");
  }

  // 验证关节角已到达 home
  auto home_angles = controller->get_joint_angles();
  bool home_ok = true;
  for (int i = 0; i < 7; ++i) {
    if (std::abs(home_angles[i] - profile.home_joints[i]) >
        robot_control::ControlConstants::kJointTolerance) {
      home_ok = false;
      break;
    }
  }
  check(home_ok, "关节角已到达 home 位（容差 0.05rad）");

  // ---- 5. IK 位姿控制（小幅移动）----
  std::cout << "\n--- 5. IK 位姿控制 ---" << std::endl;
  try {
    // 获取当前 TCP 位姿
    auto pose = controller->get_end_effector_pose();
    std::cout << "  当前 TCP: pos=(" << pose[0] << ", " << pose[1]
              << ", " << pose[2] << ")  rpy=(" << pose[3] << ", "
              << pose[4] << ", " << pose[5] << ")" << std::endl;

    // 在当前位姿基础上做小幅度 Z 方向抬升
    std::array<double, 3> target_xyz = {pose[0], pose[1], pose[2] + 0.05};
    std::array<double, 3> target_rpy = {pose[3], pose[4], pose[5]};

    controller->move_to_pose(target_xyz, target_rpy, 0.04, 10, 0.08, true);
    check(true, "move_to_pose (Z+0.05m) 成功");

    // 验证位置变化
    auto pose_after = controller->get_end_effector_pose();
    double dz = pose_after[2] - pose[2];
    std::cout << "  移动后 TCP Z: " << pose_after[2]
              << "  delta_Z=" << dz << "m" << std::endl;
    check(std::abs(dz - 0.05) < 0.02, "实际 Z 增量约 0.05m（容差 2cm）");

    // 回到原位
    controller->move_to_pose({pose[0], pose[1], pose[2]}, target_rpy,
                              0.04, 10, 0.08, true);
    std::cout << "  已回到原位" << std::endl;

  } catch (const std::exception& e) {
    std::cout << "  [FAIL] IK 运动异常: " << e.what() << std::endl;
    check(false, "move_to_pose");
  }

  // ---- 6. 单关节旋转 ----
  std::cout << "\n--- 6. 单关节旋转 ---" << std::endl;
  try {
    auto before = controller->get_joint_angles();
    controller->rotate_joint(0, 0.1, true);
    auto after = controller->get_joint_angles();
    double delta = after[0] - before[0];
    check(std::abs(delta - 0.1) < 0.05,
          "rotate_joint(0, +0.1) 关节1 增量正确");
    // 回退
    controller->rotate_joint(0, -0.1, true);
  } catch (const std::exception& e) {
    std::cout << "  [FAIL] rotate_joint 异常: " << e.what() << std::endl;
    check(false, "rotate_joint");
  }

  // ---- 7. TCP 切换 ----
  std::cout << "\n--- 7. TCP 切换 ---" << std::endl;
  try {
    controller->set_tcp("grasptarget");
    check(controller->get_current_tcp() == "grasptarget",
          "set_tcp('grasptarget') 成功");

    auto tcp_pose = controller->get_end_effector_pose();
    std::cout << "  grasptarget TCP: pos=(" << tcp_pose[0] << ", "
              << tcp_pose[1] << ", " << tcp_pose[2] << ")" << std::endl;

    // TCP offset 0.1m in Z，所以 grasptarget Z 应高于 hand Z
    // 验证：用 hand TCP 获取 Z，grasptarget Z 应 ≈ hand_Z + 0.1
    controller->set_tcp("hand");
    auto hand_pose = controller->get_end_effector_pose();
    controller->set_tcp("grasptarget");
    auto gt_pose = controller->get_end_effector_pose();
    double tcp_offset_z = gt_pose[2] - hand_pose[2];
    std::cout << "  TCP offset Z = " << tcp_offset_z << "m (预期 ~0.1m)"
              << std::endl;
    check(std::abs(tcp_offset_z - 0.1) < 0.02,
          "TCP offset Z ≈ 0.1m");

    controller->set_tcp("hand");
  } catch (const std::exception& e) {
    std::cout << "  [FAIL] TCP 异常: " << e.what() << std::endl;
    check(false, "TCP 切换");
  }

  // ---- 8. TF 查询 ----
  std::cout << "\n--- 8. TF 查询 ---" << std::endl;
  try {
    auto tf = controller->lookup_transform("panda_link0", "panda_hand", 1.0);
    if (tf.has_value()) {
      std::cout << "  panda_link0→panda_hand: ("
                << (*tf)[0] << ", " << (*tf)[1] << ", " << (*tf)[2]
                << ")" << std::endl;
      check(true, "TF 查询成功");
    } else {
      check(false, "TF 查询返回 nullopt");
    }
  } catch (const std::exception& e) {
    std::cout << "  [FAIL] TF 异常: " << e.what() << std::endl;
    check(false, "TF 查询");
  }

  // ---- 9. set_speed / get_speed ----
  std::cout << "\n--- 9. set_speed / get_speed ---" << std::endl;
  try {
    controller->set_speed(robot_control::MotionMode::kMoveJ, 80.0);
    double sj = controller->get_speed(robot_control::MotionMode::kMoveJ);
    check(std::abs(sj - 80.0) < 0.01, "set/get moveJ speed = 80%");

    controller->set_speed(robot_control::MotionMode::kMoveL, 30.0);
    double sl = controller->get_speed(robot_control::MotionMode::kMoveL);
    check(std::abs(sl - 30.0) < 0.01, "set/get moveL speed = 30%");

    // 独立性
    double sj2 = controller->get_speed(robot_control::MotionMode::kMoveJ);
    check(std::abs(sj2 - 80.0) < 0.01, "moveJ 速度不受 moveL 影响");

    // 恢复默认
    controller->set_speed(robot_control::MotionMode::kMoveJ, 50.0);
    controller->set_speed(robot_control::MotionMode::kMoveL, 50.0);
  } catch (const std::exception& e) {
    std::cout << "  [FAIL] speed 异常: " << e.what() << std::endl;
    check(false, "set_speed / get_speed");
  }

  // ---- 10. moveJ 关节空间运动 ----
  std::cout << "\n--- 10. moveJ 关节空间运动 ---" << std::endl;
  try {
    controller->open_gripper(true);
    controller->go_home(true);

    auto before = controller->get_joint_angles();
    auto target = before;
    target[0] += 0.2;

    controller->set_speed(robot_control::MotionMode::kMoveJ, 50.0);
    controller->moveJ(target, true);
    check(true, "moveJ 执行成功");

    auto after = controller->get_joint_angles();
    double delta = after[0] - before[0];
    std::cout << "  moveJ 关节1 增量: " << delta << " rad (预期 0.2)" << std::endl;
    check(std::abs(delta - 0.2) < 0.05,
          "moveJ 关节1 增量 ≈ 0.2 rad（容差 0.05）");

    // 回 home
    controller->go_home(true);
  } catch (const std::exception& e) {
    std::cout << "  [FAIL] moveJ 异常: " << e.what() << std::endl;
    check(false, "moveJ");
  }

  // ---- 11. moveL 笛卡尔空间运动 ----
  std::cout << "\n--- 11. moveL 笛卡尔空间运动 ---" << std::endl;
  try {
    controller->go_home(true);
    auto pose = controller->get_end_effector_pose();
    std::cout << "  当前 TCP: pos=(" << pose[0] << ", " << pose[1]
              << ", " << pose[2] << ")" << std::endl;

    // Z 方向抬升 5cm
    std::array<double, 3> target_xyz = {pose[0], pose[1], pose[2] + 0.05};
    std::array<double, 3> target_rpy = {pose[3], pose[4], pose[5]};

    controller->set_speed(robot_control::MotionMode::kMoveL, 50.0);
    controller->moveL(target_xyz, target_rpy, 0.04, true);
    check(true, "moveL 执行成功");

    auto pose_after = controller->get_end_effector_pose();
    double dz = pose_after[2] - pose[2];
    std::cout << "  moveL 后 TCP Z: " << pose_after[2]
              << "  delta_Z=" << dz << "m" << std::endl;
    check(std::abs(dz - 0.05) < 0.02,
          "moveL Z 增量 ≈ 0.05m（容差 2cm）");

    // 回原位
    std::array<double, 3> back_xyz = {pose[0], pose[1], pose[2]};
    controller->moveL(back_xyz, target_rpy, 0.04, true);
    check(true, "moveL 回原位成功");

    controller->go_home(true);
  } catch (const std::exception& e) {
    std::cout << "  [FAIL] moveL 异常: " << e.what() << std::endl;
    check(false, "moveL");
  }

  // ---- 汇总 ----
  std::cout << "\n=== 集成测试结果: " << pass_count << "/" << test_count
            << " 通过 ===" << std::endl;

  // 清理
  executor->cancel();
  spin_thread.join();
  rclcpp::shutdown();

  return (pass_count == test_count) ? 0 : 1;
}
