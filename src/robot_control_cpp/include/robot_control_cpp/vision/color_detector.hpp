#pragma once

#include <array>
#include <Eigen/Core>
#include <opencv2/core.hpp>

#include "robot_control_cpp/vision/camera_interface.hpp"

namespace robot_control {

/// 基于 HSV 颜色范围的目标检测器
class ColorDetector : public CameraInterface {
public:
  /// @brief 构造颜色检测器
  /// @param lower_hsv HSV 下界 [h, s, v]
  /// @param upper_hsv HSV 上界 [h, s, v]
  ColorDetector(const std::array<int, 3>& lower_hsv,
                const std::array<int, 3>& upper_hsv);

  /// @brief 检测颜色区域质心
  /// @param bgr_image BGR 图像
  /// @return 像素坐标 (u, v)，未检测到返回 nullopt
  std::optional<Eigen::Vector2i> detect(const cv::Mat& bgr_image) const;

  /// @brief 在图像上绘制十字准星
  void draw_target(cv::Mat& image, int cx, int cy,
                   const std::string& label) const;

  DetectionResult process_image(const cv::Mat& rgb,
                                const cv::Mat& depth) const override;

  /// @brief 设置相机内参（用于 3D 投影）
  void set_camera_intrinsics(double fx, double fy, double cx, double cy);

private:
  cv::Scalar lower_hsv_;
  cv::Scalar upper_hsv_;
  double fx_ = 614.0;
  double fy_ = 614.0;
  double cx_ = 320.0;
  double cy_ = 240.0;
};

}  // namespace robot_control
