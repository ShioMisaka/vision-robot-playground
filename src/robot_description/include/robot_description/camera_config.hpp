#pragma once

/// @file camera_config.hpp
/// @brief ZED_X_Mini 相机内参 + 外参（统一配置，单一事实来源）
///
/// 内参 Isaac Sim 参数:
///   focal_length    = 2.208
///   h_aperture      = 5.76
///   v_aperture      = 3.24
///   image_resolution = 1280 x 720
///
/// 计算公式:
///   fx = focal_length * width  / h_aperture = 2.208 * 1280 / 5.76
///   fy = focal_length * height / v_aperture = 2.208 * 720  / 3.24
///   cx = width  / 2
///   cy = height / 2
///
/// 外参来源: URDF panda.urdf 中 panda_hand_camera_joint
///   <origin rpy="3.14159265359 0 -1.57079632679" xyz="0.025 -0.015 0.015"/>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace robot_description {

struct CameraIntrinsics {
  static constexpr double kFx = 490.6666666666667;
  static constexpr double kFy = 490.6666666666667;
  static constexpr double kCx = 640.0;
  static constexpr double kCy = 360.0;
};

/// 相机外参（panda_hand → camera_link）
/// 来源: URDF panda_hand_camera_joint
///   xyz = "0.025 -0.015 0.015"  rpy = "3.14159265359 0 -1.57079632679"
struct CameraExtrinsics {
  static constexpr double kOffsetX = 0.025;
  static constexpr double kOffsetY = -0.015;
  static constexpr double kOffsetZ = 0.015;
  static constexpr double kRoll  = 3.14159265359;   // π
  static constexpr double kPitch = 0.0;
  static constexpr double kYaw   = -1.57079632679;  // -π/2
};

/// 光学坐标系旋转（camera_link → camera_color_optical_frame）
/// USD 相机 (X-right, Y-up, Z-back) → ROS 光学 (X-right, Y-down, Z-front): Ry(π)
struct CameraOpticalFrame {
  static constexpr double kPitch = 3.14159265359;  // π, 即 Ry(π)
};

}  // namespace robot_description
