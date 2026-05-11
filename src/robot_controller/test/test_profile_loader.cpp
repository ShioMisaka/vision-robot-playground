#include <cmath>
#include <fstream>
#include <iostream>
#include <string>

#include "robot_controller/kinematics/profile_loader.hpp"

/// 写入临时 YAML 文件用于测试
static std::string write_test_yaml() {
  const std::string path = "/tmp/test_robot_profile.yaml";
  std::ofstream f(path);
  f << R"yaml(
robot:
  name: test_robot
  urdf_path: test.urdf
  dof: 7
  joint_names: [j1, j2, j3, j4, j5, j6, j7]
  all_joint_names: [j1, j2, j3, j4, j5, j6, j7, f1, f2]
  joint_limits_lower: [-1.0, -2.0, -1.0, -3.0, -1.0, -0.5, -1.0]
  joint_limits_upper: [1.0, 2.0, 1.0, -0.1, 1.0, 3.0, 1.0]
  home_joints: [0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.0, 0.04, 0.04]
  ik_default_guess: [0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.0]
  base_frame: base_link
  hand_frame: hand
  default_tcp: hand
  tcp_frames:
    hand:
      offset_xyz: [0.0, 0.0, 0.0]
      offset_rpy: [0.0, 0.0, 0.0]
    grasptarget:
      offset_xyz: [0.0, 0.0, 0.105]
      offset_rpy: [0.0, 0.0, 0.0]
  joint_limits:
    max_vel: 1.0
    max_acc: 2.0
    max_jerk: 10.0
  cartesian_limits:
    max_vel: 0.5
    max_acc: 1.0
    max_jerk: 5.0

gripper:
  type: parallel
  min_width: 0.0
  max_width: 0.04
  dof: 1
)yaml";
  f.close();
  return path;
}

int main() {
  const std::string yaml_path = write_test_yaml();
  auto config = robot_control::ProfileLoader::load(yaml_path, "/fake/base");

  const auto& p = config.robot;
  int errors = 0;

  // 基本信息
  if (p.name != "test_robot") { std::cerr << "FAIL: name\n"; ++errors; }
  if (p.dof != 7) { std::cerr << "FAIL: dof\n"; ++errors; }

  // URDF 路径解析（相对路径应被 base_dir 拼接）
  if (p.urdf_path != "/fake/base/test.urdf") {
    std::cerr << "FAIL: urdf_path = " << p.urdf_path << "\n"; ++errors;
  }

  // 关节名称
  if (p.joint_names.size() != 7) { std::cerr << "FAIL: joint_names size\n"; ++errors; }
  if (p.all_joint_names.size() != 9) { std::cerr << "FAIL: all_joint_names size\n"; ++errors; }

  // 关节限位
  if (std::abs(p.joint_limits_lower[0] - (-1.0)) > 1e-6) { std::cerr << "FAIL: lower limit\n"; ++errors; }
  if (std::abs(p.joint_limits_upper[0] - 1.0) > 1e-6) { std::cerr << "FAIL: upper limit\n"; ++errors; }

  // Home 位
  if (p.home_joints.size() != 9) { std::cerr << "FAIL: home_joints size\n"; ++errors; }
  if (std::abs(p.home_joints[3] - (-1.57)) > 1e-6) { std::cerr << "FAIL: home_joints[3]\n"; ++errors; }

  // IK 默认猜测
  if (p.ik_default_guess.size() != 7) { std::cerr << "FAIL: ik_default_guess size\n"; ++errors; }

  // 坐标系
  if (p.base_frame != "base_link") { std::cerr << "FAIL: base_frame\n"; ++errors; }
  if (p.hand_frame != "hand") { std::cerr << "FAIL: hand_frame\n"; ++errors; }

  // TCP
  if (p.default_tcp != "hand") { std::cerr << "FAIL: default_tcp\n"; ++errors; }
  if (p.tcp_frames.size() != 2) { std::cerr << "FAIL: tcp_frames size\n"; ++errors; }
  if (std::abs(p.tcp_frames.at("grasptarget").offset_xyz[2] - 0.105) > 1e-6) {
    std::cerr << "FAIL: grasptarget offset_xyz\n"; ++errors;
  }

  // 运动极限
  if (std::abs(p.joint_limits.max_vel - 1.0) > 1e-6) { std::cerr << "FAIL: joint max_vel\n"; ++errors; }
  if (std::abs(p.cartesian_limits.max_jerk - 5.0) > 1e-6) { std::cerr << "FAIL: cartesian max_jerk\n"; ++errors; }

  // 夹爪
  const auto& g = config.gripper;
  if (g.type != "parallel") { std::cerr << "FAIL: gripper type\n"; ++errors; }
  if (std::abs(g.max_width - 0.04) > 1e-6) { std::cerr << "FAIL: gripper max_width\n"; ++errors; }
  if (g.dof != 1) { std::cerr << "FAIL: gripper dof\n"; ++errors; }

  if (errors == 0) {
    std::cout << "ALL TESTS PASSED\n";
  } else {
    std::cout << errors << " TESTS FAILED\n";
  }
  return errors;
}
