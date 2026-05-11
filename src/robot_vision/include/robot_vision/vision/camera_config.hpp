#pragma once

#include <array>
#include <string>

namespace robot_vision {

/// Runtime camera intrinsics configuration (loaded from YAML)
struct CameraIntrinsicsConfig {
  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;
};

/// Runtime camera extrinsics configuration (mount -> camera_link, loaded from YAML)
struct CameraExtrinsicsConfig {
  std::array<double, 3> offset_xyz = {};
  std::array<double, 3> rpy = {};
};

/// Optical frame rotation config (camera_link -> camera_color_optical_frame)
struct CameraOpticalFrameConfig {
  double pitch = 3.14159265359;
};

/// Complete camera configuration (loaded from YAML)
struct CameraConfig {
  std::string name;
  std::string mount_frame;    // e.g., "panda_hand"
  std::string camera_frame;   // e.g., "camera_link"
  std::string optical_frame;  // e.g., "camera_color_optical_frame"
  CameraIntrinsicsConfig intrinsics;
  CameraExtrinsicsConfig extrinsics;
  CameraOpticalFrameConfig optical_frame_rotation;
};

}  // namespace robot_vision
