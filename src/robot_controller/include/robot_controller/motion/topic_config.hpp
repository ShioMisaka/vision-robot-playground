#pragma once

#include <array>
#include <string>

#include "robot_description/camera_config.hpp"

namespace robot_control {

/// 相机外参配置，值来自 robot_description::CameraExtrinsics（单一事实来源）
struct CameraExtrinsics {
  std::array<double, 3> xyz = {
      robot_description::CameraExtrinsics::kOffsetX,
      robot_description::CameraExtrinsics::kOffsetY,
      robot_description::CameraExtrinsics::kOffsetZ};
  std::array<double, 3> rpy = {
      robot_description::CameraExtrinsics::kRoll,
      robot_description::CameraExtrinsics::kPitch,
      robot_description::CameraExtrinsics::kYaw};
};

struct TopicConfig {
  std::string joint_command = "/joint_command";
  std::string joint_state = "/joint_states";
  std::string camera_left = "/camera/image_raw/left";
  std::string camera_depth = "/camera/image_raw/depth";
  std::string camera_frame = "camera_color_optical_frame";
  CameraExtrinsics camera_extrinsics;
};

}  // namespace robot_control
