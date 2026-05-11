#pragma once

#include <string>
#include "robot_controller/kinematics/robot_profile.hpp"

namespace robot_control {

/// 机器人完整配置（从 YAML 文件加载）
struct RobotConfig {
  RobotProfile robot;
  GripperProfile gripper;
};

/// 从 YAML 文件加载机器人配置
/// YAML 格式参考 robot_description/config/panda_profile.yaml
class ProfileLoader {
 public:
  /// @brief 从 YAML 文件加载机器人配置
  /// @param yaml_path YAML 文件绝对路径
  /// @param urdf_base_dir URDF 基础目录（用于解析相对 urdf_path），为空则相对路径不解析，直接使用 YAML 中的原始值
  /// @return 机器人配置
  static RobotConfig load(const std::string& yaml_path,
                          const std::string& urdf_base_dir = "");
};

}  // namespace robot_control
