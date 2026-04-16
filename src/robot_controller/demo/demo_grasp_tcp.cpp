/// @file demo_grasp_tcp.cpp
/// @brief C++ 版 test_grasp_tcp.py：使用 grasptarget TCP 抓取指定位置
///
/// 编译后运行（需要 Isaac Sim 已启动）：
///   source /opt/ros/jazzy/setup.bash
///   source install/setup.bash
///   ros2 run robot_controller demo_grasp_tcp
///
/// 流程：
///   1. 切换到 grasptarget TCP（指尖坐标系）
///   2. 张开夹爪
///   3. 移动到目标上方（夹爪朝下）
///   4. 下降到目标位置
///   5. 闭合夹爪
///   6. 提起

#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include "robot_controller/nodes/robot_controller_node.hpp"
#include "robot_controller/profiles/panda_profile.hpp"
#include "robot_controller/motion/control_constants.hpp"
#include "robot_controller/nodes/topic_config.hpp"

// ---- 目标参数 ----
constexpr double kTargetX = 0.52699;
constexpr double kTargetY = 0.0;
constexpr double kTargetZ = 0.04026;
constexpr double kApproachHeight = 0.10;   // 目标上方 10cm
constexpr double kLiftHeight = 0.25;       // 提起高度 15cm

// 夹爪朝下姿态
const std::array<double, 3> kGripperDownRpy = {
    0.0, -M_PI, -M_PI};

// ---- 运动参数 ----
constexpr double kMoveJSpeed = 60.0;  // 关节运动速度百分比
constexpr double kMoveLSpeed = 40.0;  // 笛卡尔运动速度百分比

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  // 构造机器人控制节点
  auto profile = robot_control::profiles::panda();
  auto gripper = robot_control::profiles::panda_gripper();
  robot_control::TopicConfig topics;

  auto robot_node = robot_control::RobotControllerNode::create(
      profile, gripper, topics);

  // MultiThreadedExecutor 后台 spin
  auto executor =
      std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(robot_node);
  auto spin_thread = std::thread([executor]() { executor->spin(); });

  RCLCPP_INFO(robot_node->get_logger(),
              "等待与 Isaac Sim 建立连接...");

  bool ready = robot_node->wait_for_ready(
      robot_control::ControlConstants::kReadyTimeout);
  if (!ready) {
    RCLCPP_ERROR(robot_node->get_logger(),
                 "等待关节状态超时，请确认 Isaac Sim 已启动");
    executor->cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 1;
  }

  auto ctrl = robot_node->get_controller();

  // 设置运动速度
  ctrl->set_speed(robot_control::MotionMode::kMoveJ, kMoveJSpeed);
  ctrl->set_speed(robot_control::MotionMode::kMoveL, kMoveLSpeed);

  // 切换到指尖坐标系
  ctrl->set_tcp("grasptarget");

  // 张开夹爪
  RCLCPP_INFO(robot_node->get_logger(), "张开夹爪");
  ctrl->open_gripper(true);

  // moveJ：移动到目标上方（关节空间快速定位）
  RCLCPP_INFO(robot_node->get_logger(), "--- moveJ 到目标上方 ---");
  std::array<double, 3> approach_pos = {
      kTargetX, kTargetY, kTargetZ + kApproachHeight};
  ctrl->moveJ(approach_pos, kGripperDownRpy, gripper.max_width, true);

  // moveL：直线下降到目标位置
  RCLCPP_INFO(robot_node->get_logger(), "--- moveL 下降到目标 ---");
  std::array<double, 3> target_pos = {kTargetX, kTargetY, kTargetZ};
  ctrl->moveL(target_pos, kGripperDownRpy, gripper.max_width, true);

  // 闭合夹爪
  RCLCPP_INFO(robot_node->get_logger(), "--- 闭合夹爪 ---");
  ctrl->close_gripper(true);

  // moveL：提起（保持夹爪闭合）
  RCLCPP_INFO(robot_node->get_logger(), "--- moveL 提起 ---");
  std::array<double, 3> lift_pos = {
      kTargetX, kTargetY, kTargetZ + kLiftHeight};
  ctrl->moveL(lift_pos, kGripperDownRpy, gripper.min_width, true);

  RCLCPP_INFO(robot_node->get_logger(), "--- 改变姿态 ---");
  auto angles = ctrl->get_joint_angles();
  angles[1] -= 0.3;
  ctrl->moveJ(angles, true);

  // moveJ：关节旋转展示
  RCLCPP_INFO(robot_node->get_logger(), "--- moveJ 转90° ---");
  angles = ctrl->get_joint_angles();
  angles[0] += 1.5708;
  ctrl->moveJ(angles, true);

  RCLCPP_INFO(robot_node->get_logger(), "--- moveJ 抬起大臂 ---");
  angles = ctrl->get_joint_angles();
  angles[3] += 1.5708;
  ctrl->moveJ(angles, true);

  RCLCPP_INFO(robot_node->get_logger(), "完成!");

  // 清理
  executor->cancel();
  spin_thread.join();
  rclcpp::shutdown();
  return 0;
}
