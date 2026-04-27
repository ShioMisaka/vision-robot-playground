/// @file test_camera_tf.cpp
/// @brief 相机 TF 链验证集成测试
///
/// 前提：Isaac Sim 已启动并发布 /joint_states 话题
///
/// 编译后运行：
///   source install/setup.bash
///   ros2 run robot_control_test test_camera_tf
///
/// 测试流程：
///   1. 创建 RobotControllerNode，等待关节状态就绪
///   2. 验证 TF 查询：hand → camera_link → camera_color_optical_frame
///   3. 验证完整链：panda_link0 → camera_color_optical_frame
///   4. 运动一致性：机械臂运动后 hand → camera 偏移不变
///   5. 坐标转换：transform_to_base 将相机坐标转到 base 坐标系

#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include "robot_controller/nodes/robot_controller_node.hpp"
#include "robot_vision/nodes/vision_processor_node.hpp"
#include "robot_vision/vision/color_detector.hpp"
#include "robot_vision/nodes/grasp_task_manager.hpp"
#include "robot_controller/profiles/panda_profile.hpp"
#include "robot_controller/motion/control_constants.hpp"
#include "robot_controller/motion/topic_config.hpp"

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

void print_tf(const std::string& label,
              const std::optional<std::array<double, 6>>& tf) {
  if (!tf.has_value()) {
    std::cout << "  " << label << ": <查询失败>" << std::endl;
    return;
  }
  auto deg = [](double r) { return r * 180.0 / M_PI; };
  std::cout << "  " << label << ": xyz=["
            << (*tf)[0] << ", " << (*tf)[1] << ", " << (*tf)[2] << "]  rpy=["
            << deg((*tf)[3]) << ", " << deg((*tf)[4]) << ", " << deg((*tf)[5])
            << "] deg" << std::endl;
}

}  // namespace

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  std::cout << "=== 相机 TF 链验证测试 ===" << std::endl;
  std::cout << "前提：Isaac Sim 已启动并发布 /joint_states\n" << std::endl;

  // ---- 配置 ----
  auto profile = robot_control::profiles::panda();
  auto gripper = robot_control::profiles::panda_gripper();
  robot_control::TopicConfig topics;

  // 相机外参（与 URDF 一致）
  topics.camera_extrinsics.xyz = {0.015, 0.0, 0.03};
  topics.camera_extrinsics.rpy = {0.0, M_PI / 2.0, M_PI};

  auto robot_node = robot_control::RobotControllerNode::create(
      profile, gripper, topics);

  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(robot_node);
  auto spin_thread = std::thread([executor]() { executor->spin(); });

  // ---- 等待就绪 ----
  std::cout << "--- 等待关节状态就绪 ---" << std::endl;
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

  auto ctrl = robot_node->get_controller();
  std::cout << std::endl;

  // ---- 测试 1: TF 查询（局部链） ----
  std::cout << "--- 1. TF 查询（局部链） ---" << std::endl;

  auto tf_hc = ctrl->lookup_transform("panda_hand", "camera_link", 2.0);
  print_tf("hand → camera_link", tf_hc);
  check(tf_hc.has_value(), "hand → camera_link 查询成功");

  auto tf_ho = ctrl->lookup_transform(
      "panda_hand", "camera_color_optical_frame", 2.0);
  print_tf("hand → camera_optical", tf_ho);
  check(tf_ho.has_value(), "hand → camera_color_optical_frame 查询成功");
  std::cout << std::endl;

  // ---- 测试 2: TF 查询（完整链） ----
  std::cout << "--- 2. TF 查询（完整链） ---" << std::endl;

  auto tf_full = ctrl->lookup_transform(
      "panda_link0", "camera_color_optical_frame", 2.0);
  print_tf("base → camera_optical", tf_full);
  check(tf_full.has_value(), "panda_link0 → camera_color_optical_frame 查询成功");
  std::cout << std::endl;

  // ---- 测试 3: 运动一致性 ----
  std::cout << "--- 3. 运动 TF 一致性 ---" << std::endl;

  auto before = ctrl->lookup_transform("panda_hand", "camera_link", 2.0);
  check(before.has_value(), "运动前 hand → camera_link 可查");

  if (before.has_value()) {
    // 小幅旋转 joint2（远离奇异位）
    double delta = 0.1;
    ctrl->rotate_joint(1, delta, true);

    auto after = ctrl->lookup_transform("panda_hand", "camera_link", 2.0);

    // 恢复
    ctrl->rotate_joint(1, -delta, true);

    if (after.has_value()) {
      bool consistent = true;
      for (int i = 0; i < 6; ++i) {
        if (std::abs((*before)[i] - (*after)[i]) > 1e-4) {
          consistent = false;
          break;
        }
      }
      check(consistent, "运动前后 hand → camera_link 偏移一致");
    } else {
      check(false, "运动后 hand → camera_link 可查");
    }
  }
  std::cout << std::endl;

  // ---- 测试 4: transform_to_base 坐标转换 ----
  std::cout << "--- 4. transform_to_base 坐标转换 ---" << std::endl;

  auto tf_cam = ctrl->lookup_transform(
      "panda_link0", "camera_color_optical_frame", 2.0);
  check(tf_cam.has_value(), "坐标转换：base → camera TF 可查");

  if (tf_cam.has_value()) {
    // 构造 GraspTaskManager
    auto detector = std::make_shared<robot_vision::ColorDetector>(
        std::array<int, 3>{0, 0, 0}, std::array<int, 3>{0, 0, 0});
    auto vision = robot_vision::VisionProcessorNode::create(
        detector, robot_control::TopicConfig());
    auto manager = std::make_shared<robot_vision::GraspTaskManager>(
        ctrl, vision, "panda_link0", "camera_color_optical_frame",
        0.15, 0.02,
        std::array<double, 3>{M_PI, 0.0, M_PI});

    // 光学坐标系 Z 朝前（朝下），[0, 0, 0.1] = 相机正下方 0.1m
    Eigen::Vector3d camera_point(0.0, 0.0, 0.1);
    auto base_point = manager->transform_to_base(camera_point);
    check(base_point.has_value(), "transform_to_base 返回有效结果");

    if (base_point.has_value()) {
      // 手动计算期望值验证
      double roll = (*tf_cam)[3], pitch = (*tf_cam)[4], yaw = (*tf_cam)[5];
      double cr = std::cos(roll), sr = std::sin(roll);
      double cp = std::cos(pitch), sp = std::sin(pitch);
      double cy = std::cos(yaw), sy = std::sin(yaw);

      // R = Rz(yaw) * Ry(pitch) * Rx(roll)，取第三列乘以 0.1
      double ex = (cy * sp * cr + sy * sr) * 0.1 + (*tf_cam)[0];
      double ey = (sy * sp * cr - cy * sr) * 0.1 + (*tf_cam)[1];
      double ez = cp * cr * 0.1 + (*tf_cam)[2];

      std::cout << "  相机 base 位置:    [" << (*tf_cam)[0] << ", "
                << (*tf_cam)[1] << ", " << (*tf_cam)[2] << "]" << std::endl;
      std::cout << "  期望 base 点:      [" << ex << ", " << ey << ", "
                << ez << "]" << std::endl;
      std::cout << "  transform_to_base: [" << (*base_point)[0] << ", "
                << (*base_point)[1] << ", " << (*base_point)[2] << "]"
                << std::endl;

      double err = std::sqrt(std::pow((*base_point)[0] - ex, 2) +
                             std::pow((*base_point)[1] - ey, 2) +
                             std::pow((*base_point)[2] - ez, 2));
      std::cout << "  误差: " << err * 1000.0 << " mm" << std::endl;
      check(err < 1e-3, "坐标转换误差 < 1mm");
    }
  }
  std::cout << std::endl;

  // ---- 汇总 ----
  std::cout << "=== 测试结果: " << pass_count << "/" << test_count
            << " 通过 ===" << std::endl;

  executor->cancel();
  spin_thread.join();
  rclcpp::shutdown();

  return (pass_count == test_count) ? 0 : 1;
}
