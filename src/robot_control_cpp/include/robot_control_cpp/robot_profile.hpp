#pragma once

#include <array>
#include <map>
#include <string>
#include <vector>

namespace robot_control {

/// 机器人 TCP（工具中心点）配置
struct TcpConfig {
  std::array<double, 3> offset_xyz = {0.0, 0.0, 0.0};
  std::array<double, 3> offset_rpy = {0.0, 0.0, 0.0};
};

/// 机器人物理参数描述，新增机器人只需定义新的 profile 实例
struct RobotProfile {
  std::string name;
  std::string urdf_path;
  int dof = 7;
  std::vector<std::string> joint_names;
  std::vector<std::string> all_joint_names;  // 包含夹爪关节
  std::vector<double> joint_limits_lower;
  std::vector<double> joint_limits_upper;
  std::vector<double> home_joints;  // 包含夹爪（dof + 2）
  std::vector<double> ik_default_guess;
  std::string base_frame;
  std::string hand_frame;
  std::map<std::string, TcpConfig> tcp_frames;
  std::string default_tcp = "hand";
};

/// 夹爪参数描述
struct GripperProfile {
  std::string type = "parallel";
  double min_width = 0.0;
  double max_width = 0.04;
  int dof = 1;
};

}  // namespace robot_control
