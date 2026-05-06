#pragma once

/// @file camera_config.hpp
/// @brief ZED_X_Mini 相机内参（统一配置）
///
/// Isaac Sim 参数:
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

namespace robot_description {

struct CameraIntrinsics {
  static constexpr double kFx = 490.6666666666667;
  static constexpr double kFy = 490.6666666666667;
  static constexpr double kCx = 640.0;
  static constexpr double kCy = 360.0;
};

}  // namespace robot_description
