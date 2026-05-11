#include "robot_vision/vision/camera_config_loader.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>

#include <yaml-cpp/yaml.h>

namespace robot_vision {

CameraConfig CameraConfigLoader::load(const std::string& yaml_path) {
  YAML::Node doc;
  try {
    doc = YAML::LoadFile(yaml_path);
  } catch (const YAML::Exception& e) {
    throw std::runtime_error("Failed to load camera config YAML: " + yaml_path +
                             " - " + e.what());
  }

  // Validate top-level "camera" node
  const auto& camera = doc["camera"];
  if (!camera || !camera.IsMap()) {
    throw std::runtime_error(
        "Camera config YAML missing top-level 'camera' section: " + yaml_path);
  }

  CameraConfig config;

  // Basic info
  config.name = camera["name"].as<std::string>();
  config.mount_frame = camera["mount_frame"].as<std::string>();
  config.camera_frame = camera["camera_frame"].as<std::string>();
  config.optical_frame = camera["optical_frame"].as<std::string>();

  // Intrinsics
  const auto& intrinsics = camera["intrinsics"];
  if (!intrinsics || !intrinsics.IsMap()) {
    throw std::runtime_error(
        "Camera config YAML missing 'camera.intrinsics' section: " + yaml_path);
  }
  config.intrinsics.fx = intrinsics["fx"].as<double>();
  config.intrinsics.fy = intrinsics["fy"].as<double>();
  config.intrinsics.cx = intrinsics["cx"].as<double>();
  config.intrinsics.cy = intrinsics["cy"].as<double>();

  // Extrinsics
  const auto& extrinsics = camera["extrinsics"];
  if (!extrinsics || !extrinsics.IsMap()) {
    throw std::runtime_error(
        "Camera config YAML missing 'camera.extrinsics' section: " + yaml_path);
  }

  auto offset_xyz = extrinsics["offset_xyz"].as<std::vector<double>>();
  if (offset_xyz.size() < 3) {
    throw std::runtime_error(
        "Camera config 'extrinsics.offset_xyz' must have at least 3 elements "
        "(got " +
        std::to_string(offset_xyz.size()) + "): " + yaml_path);
  }
  std::copy_n(offset_xyz.begin(), 3, config.extrinsics.offset_xyz.begin());

  auto rpy = extrinsics["rpy"].as<std::vector<double>>();
  if (rpy.size() < 3) {
    throw std::runtime_error(
        "Camera config 'extrinsics.rpy' must have at least 3 elements "
        "(got " +
        std::to_string(rpy.size()) + "): " + yaml_path);
  }
  std::copy_n(rpy.begin(), 3, config.extrinsics.rpy.begin());

  // Optical frame rotation
  const auto& optical = camera["optical_frame_rotation"];
  if (!optical || !optical.IsMap()) {
    throw std::runtime_error(
        "Camera config YAML missing 'camera.optical_frame_rotation' section: " +
        yaml_path);
  }
  config.optical_frame_rotation.pitch = optical["pitch"].as<double>();

  return config;
}

}  // namespace robot_vision
