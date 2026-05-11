#pragma once

#include <string>

#include "robot_vision/vision/camera_config.hpp"

namespace robot_vision {

/// Load camera configuration from a YAML file
/// Follows the ProfileLoader pattern from robot_controller
class CameraConfigLoader {
 public:
  /// Load camera configuration from YAML file
  /// @param yaml_path Absolute path to camera YAML file
  /// @return CameraConfig with all parameters loaded
  /// @throws std::runtime_error on invalid format or missing fields
  static CameraConfig load(const std::string& yaml_path);
};

}  // namespace robot_vision
