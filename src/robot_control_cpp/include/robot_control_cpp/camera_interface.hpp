#pragma once

#include <Eigen/Core>
#include <opencv2/core.hpp>

#include "robot_control_cpp/i_vision_processor.hpp"

namespace robot_control {

/// 图像处理抽象基类，子类实现具体检测逻辑
class CameraInterface {
public:
  virtual ~CameraInterface() = default;

  /// @brief 处理一帧 RGB + 深度图像，返回检测结果
  /// @param rgb BGR 格式图像
  /// @param depth 深度图（uint16 mm 或 float32 m）
  /// @return 检测结果
  virtual DetectionResult process_image(const cv::Mat& rgb,
                                        const cv::Mat& depth) const = 0;

  /// @brief 像素坐标 + 深度反投影到相机坐标系 3D 点
  /// @param u, v 像素坐标
  /// @param depth 深度值（米）
  /// @param fx, fy, cx, cy 相机内参
  /// @return 相机坐标系下的 3D 位置
  static Eigen::Vector3d pixel_to_3d(int u, int v, double depth,
                                      double fx, double fy,
                                      double cx, double cy);
};

}  // namespace robot_control
