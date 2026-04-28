#pragma once

#include <Eigen/Core>
#include <array>
#include <optional>
#include <string>

namespace robot_vision {

/// 单次视觉检测结果
struct DetectionResult {
  bool detected = false;
  Eigen::Vector3d xyz = Eigen::Vector3d::Zero();
  Eigen::Vector2i uv = Eigen::Vector2i::Zero();
  double confidence = 0.0;
  std::string label;  ///< 物体类别标签（空=未指定，预留 YOLO 等扩展）
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

  /// 采集 N 个样本，平均 xyz/uv，降低深度噪声
  /// @param sample_count 采集样本数
  /// @param sample_interval 样本间隔（秒）
  /// @param timeout 总超时（秒）
  /// @return 平均后的检测结果，有效样本不足返回 nullopt
  virtual std::optional<DetectionResult> average_detections(
      int sample_count = 5,
      double sample_interval = 0.1,
      double timeout = 5.0) = 0;
};

}  // namespace robot_vision
