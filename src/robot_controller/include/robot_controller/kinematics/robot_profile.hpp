#pragma once

#include <array>
#include <map>
#include <string>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace robot_control {

/// 运动极限参数（速度/加速度/加加速度）
struct MotionLimits {
  double max_vel = 1.0;   ///< moveJ: rad/s, moveL: m/s
  double max_acc = 1.0;   ///< moveJ: rad/s², moveL: m/s²
  double max_jerk = 1.0;  ///< moveJ: rad/s³, moveL: m/s³
};

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
  MotionLimits joint_limits;
  MotionLimits cartesian_limits;
};

/// 夹爪参数描述
struct GripperProfile {
  std::string type = "parallel";
  double min_width = 0.0;
  double max_width = 0.04;
  int dof = 1;
};

/// 根据 TcpConfig 构建 4x4 齐次变换矩阵
inline Eigen::Matrix4d tcp_to_matrix4d(const TcpConfig& cfg) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  Eigen::AngleAxisd roll_a(cfg.offset_rpy[0], Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitch_a(cfg.offset_rpy[1], Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yaw_a(cfg.offset_rpy[2], Eigen::Vector3d::UnitZ());
  T.block<3, 3>(0, 0) = (yaw_a * pitch_a * roll_a).toRotationMatrix();
  T(0, 3) = cfg.offset_xyz[0];
  T(1, 3) = cfg.offset_xyz[1];
  T(2, 3) = cfg.offset_xyz[2];
  return T;
}

/// 运动模式，用于 set_speed/get_speed
enum class MotionMode { kMoveJ, kMoveL };

/// 从 RPY 角构建 3x3 旋转矩阵（ZYX 内旋顺序）
inline Eigen::Matrix3d rpy_to_rotation(double roll, double pitch, double yaw) {
  return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
}

}  // namespace robot_control
