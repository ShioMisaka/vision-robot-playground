#pragma once

#include <array>
#include <string>

namespace robot_control {

/// 相机外参（相对于 hand_frame 的固定偏移）
struct CameraExtrinsics {
  /// camera_link 相对于 hand 的位置偏移（米）
  std::array<double, 3> xyz = {0.015, 0.0, 0.03};
  /// camera_link 相对于 hand 的旋转（ZYX 惯性欧拉角，弧度）
  std::array<double, 3> rpy = {0.0, 1.57079632679, 3.14159265359};
};

/// ROS 2 话题配置，支持按机器人实例自定义
struct TopicConfig {
  std::string joint_command = "/joint_command";
  std::string joint_state = "/joint_states";
  std::string camera_left = "/camera/image_raw/left";
  std::string camera_depth = "/camera/image_raw/depth";
  std::string camera_frame = "camera_color_optical_frame";
  CameraExtrinsics camera_extrinsics;
};

/// 通用控制常量
struct ControlConstants {
  /// 关节运动到位判定阈值（弧度）
  static constexpr double kJointTolerance = 0.05;
  /// 夹爪运动到位判定阈值（米）
  static constexpr double kFingerTolerance = 0.002;
  /// 运动等待超时（秒）
  static constexpr double kMotionTimeout = 10.0;
  /// 运动轮询间隔（秒）
  static constexpr double kPollInterval = 0.02;
  /// 到位后稳定等待时间（秒）
  static constexpr double kSettleTime = 0.2;
  /// 默认插值步数
  static constexpr int kDefaultSteps = 10;
  /// 默认插值步间隔（秒）
  static constexpr double kDefaultStepTime = 0.08;
  /// 图像同步队列大小
  static constexpr int kImageSyncQueueSize = 10;
  /// 图像同步时间容差（秒）
  static constexpr double kImageSyncSlop = 0.1;
  /// 夹爪稳定检测次数
  static constexpr int kFingerStableCount = 5;
  /// 夹爪稳定容差（米）
  static constexpr double kFingerStableTol = 0.001;
  /// 就绪等待超时（秒）
  static constexpr double kReadyTimeout = 5.0;
  /// 轨迹规划时间步长（50 Hz）
  static constexpr double kTrajectoryDt = 0.02;
};

}  // namespace robot_control
