#pragma once

#include <Eigen/Core>
#include <array>
#include <optional>

namespace robot_vision {

/// 单次视觉检测结果
struct DetectionResult {
  bool detected = false;
  Eigen::Vector3d xyz = Eigen::Vector3d::Zero();
  Eigen::Vector2i uv = Eigen::Vector2i::Zero();
  double confidence = 0.0;
};

/// 视觉处理抽象接口，不依赖任何 ROS 2 组件
class IVisionProcessor {
public:
  virtual ~IVisionProcessor() = default;

  /// 获取最新检测结果
  virtual std::optional<DetectionResult> get_latest_result() const = 0;

  /// 阻塞等待直到检测到目标或超时
  virtual std::optional<DetectionResult> wait_for_detection(
      double timeout = 10.0) = 0;
};

}  // namespace robot_vision
