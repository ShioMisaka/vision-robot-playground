#include "robot_controller/kinematics/profile_loader.hpp"

#include <filesystem>
#include <stdexcept>
#include <string>

#include <yaml-cpp/yaml.h>

namespace robot_control {

namespace fs = std::filesystem;

RobotConfig ProfileLoader::load(const std::string& yaml_path,
                                 const std::string& urdf_base_dir) {
  YAML::Node doc;
  try {
    doc = YAML::LoadFile(yaml_path);
  } catch (const YAML::Exception& e) {
    throw std::runtime_error("Failed to load profile YAML: " + yaml_path +
                             " - " + e.what());
  }

  // --- Fix 1: YAML structure validation ---

  const auto& robot_node = doc["robot"];
  if (!robot_node || !robot_node.IsMap()) {
    throw std::runtime_error("Profile YAML missing 'robot' section: " + yaml_path);
  }

  const auto& gripper_node = doc["gripper"];
  if (!gripper_node || !gripper_node.IsMap()) {
    throw std::runtime_error("Profile YAML missing 'gripper' section: " + yaml_path);
  }

  RobotConfig config;

  // 基本信息
  config.robot.name = robot_node["name"].as<std::string>();
  config.robot.dof = robot_node["dof"].as<int>();

  // URDF 路径：如果是相对路径，拼接 base_dir
  const auto urdf_rel = robot_node["urdf_path"].as<std::string>();
  if (!urdf_base_dir.empty() && !fs::path(urdf_rel).is_absolute()) {
    config.robot.urdf_path =
        (fs::path(urdf_base_dir) / urdf_rel).generic_string();
  } else {
    config.robot.urdf_path = urdf_rel;
  }

  // 关节名称
  config.robot.joint_names =
      robot_node["joint_names"].as<std::vector<std::string>>();
  config.robot.all_joint_names =
      robot_node["all_joint_names"].as<std::vector<std::string>>();

  // 关节限位
  config.robot.joint_limits_lower =
      robot_node["joint_limits_lower"].as<std::vector<double>>();
  config.robot.joint_limits_upper =
      robot_node["joint_limits_upper"].as<std::vector<double>>();

  // Home + IK guess
  config.robot.home_joints =
      robot_node["home_joints"].as<std::vector<double>>();
  config.robot.ik_default_guess =
      robot_node["ik_default_guess"].as<std::vector<double>>();

  // 坐标系
  config.robot.base_frame = robot_node["base_frame"].as<std::string>();
  config.robot.hand_frame = robot_node["hand_frame"].as<std::string>();
  config.robot.default_tcp = robot_node["default_tcp"].as<std::string>();

  // TCP frames
  const auto& tcp_node = robot_node["tcp_frames"];
  if (!tcp_node || !tcp_node.IsMap()) {
    throw std::runtime_error("Profile YAML missing 'robot.tcp_frames' section: " + yaml_path);
  }
  for (const auto& entry : tcp_node) {
    const std::string name = entry.first.as<std::string>();
    const auto& frame = entry.second;
    TcpConfig cfg;
    auto xyz = frame["offset_xyz"].as<std::vector<double>>();
    auto rpy = frame["offset_rpy"].as<std::vector<double>>();
    // Fix 4: TCP offset vector size check
    if (xyz.size() < 3) {
      throw std::runtime_error("TCP frame '" + name +
          "' offset_xyz must have at least 3 elements (got " +
          std::to_string(xyz.size()) + "): " + yaml_path);
    }
    if (rpy.size() < 3) {
      throw std::runtime_error("TCP frame '" + name +
          "' offset_rpy must have at least 3 elements (got " +
          std::to_string(rpy.size()) + "): " + yaml_path);
    }
    std::copy_n(xyz.begin(), 3, cfg.offset_xyz.begin());
    std::copy_n(rpy.begin(), 3, cfg.offset_rpy.begin());
    config.robot.tcp_frames[name] = cfg;
  }

  // 运动极限
  const auto& jl = robot_node["joint_limits"];
  if (!jl || !jl.IsMap()) {
    throw std::runtime_error("Profile YAML missing 'robot.joint_limits' section: " + yaml_path);
  }
  config.robot.joint_limits = {
      jl["max_vel"].as<double>(),
      jl["max_acc"].as<double>(),
      jl["max_jerk"].as<double>(),
  };
  const auto& cl = robot_node["cartesian_limits"];
  if (!cl || !cl.IsMap()) {
    throw std::runtime_error("Profile YAML missing 'robot.cartesian_limits' section: " + yaml_path);
  }
  config.robot.cartesian_limits = {
      cl["max_vel"].as<double>(),
      cl["max_acc"].as<double>(),
      cl["max_jerk"].as<double>(),
  };

  // 夹爪
  config.gripper.type = gripper_node["type"].as<std::string>();
  config.gripper.min_width = gripper_node["min_width"].as<double>();
  config.gripper.max_width = gripper_node["max_width"].as<double>();
  config.gripper.dof = gripper_node["dof"].as<int>();

  // --- Fix 2: dof consistency check ---
  const int dof = config.robot.dof;
  if (static_cast<int>(config.robot.joint_names.size()) != dof) {
    throw std::runtime_error("joint_names size (" +
        std::to_string(config.robot.joint_names.size()) +
        ") does not match dof (" + std::to_string(dof) + "): " + yaml_path);
  }
  if (static_cast<int>(config.robot.home_joints.size()) < dof) {
    throw std::runtime_error("home_joints size (" +
        std::to_string(config.robot.home_joints.size()) +
        ") is less than dof (" + std::to_string(dof) + "): " + yaml_path);
  }

  return config;
}

}  // namespace robot_control
