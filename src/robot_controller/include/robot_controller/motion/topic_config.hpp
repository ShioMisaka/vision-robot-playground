#pragma once

#include <array>
#include <string>

namespace robot_control {

struct CameraExtrinsics {
  std::array<double, 3> xyz = {0.015, 0.0, 0.03};
  std::array<double, 3> rpy = {0.0, -1.57079632679, 0.0};
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
