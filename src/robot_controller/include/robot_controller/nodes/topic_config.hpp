#pragma once

#include <string>

namespace robot_control {

/// ROS2 话题名称配置（仅 nodes 层使用）
struct TopicConfig {
  std::string joint_command = "/joint_command";
  std::string joint_state = "/joint_states";
};

}  // namespace robot_control
