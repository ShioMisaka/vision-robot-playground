#pragma once

#include <map>
#include <string>
#include <vector>
#include "robot_control_cpp/robot_profile.hpp"
#include "robot_control_cpp/color_detector.hpp"

namespace robot_control::profiles {

/// @brief 创建 Franka Panda 机器人参数
inline RobotProfile panda() {
  RobotProfile p;
  p.name = "panda";
  p.urdf_path = "urdf/panda.urdf";
  p.dof = 7;

  p.joint_names = {
      "panda_joint1", "panda_joint2", "panda_joint3",
      "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"};

  p.all_joint_names = {
      "panda_joint1", "panda_joint2", "panda_joint3",
      "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7",
      "panda_finger_joint1", "panda_finger_joint2"};

  p.joint_limits_lower = {
      -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973};
  p.joint_limits_upper = {
      2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973};

  // 9 个值：7 关节 + 2 夹爪
  p.home_joints = {0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.0, 0.4, 0.4};

  // IK 默认初始猜测
  p.ik_default_guess = {0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.0};

  p.base_frame = "panda_link0";
  p.hand_frame = "panda_hand";

  // TCP 配置
  p.tcp_frames = {
      {"hand", TcpConfig{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}},
      {"grasptarget", TcpConfig{{0.0, 0.0, 0.1}, {0.0, 0.0, 0.0}}},
  };
  p.default_tcp = "hand";

  return p;
}

/// @brief 创建 Panda 二指夹爪参数
inline GripperProfile panda_gripper() {
  return {"parallel", 0.0, 0.04, 1};
}

/// @brief 创建 Panda 红色检测器
inline ColorDetector panda_red_detector() {
  return ColorDetector({0, 100, 100}, {10, 255, 255});
}

}  // namespace robot_control::profiles
