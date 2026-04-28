#pragma once

#include <array>
#include <Eigen/Core>
#include <opencv2/core.hpp>

#include "robot_vision/vision/camera_interface.hpp"

namespace robot_vision {

/// 基于 HSV 颜色范围的目标检测器
class ColorDetector : public CameraInterface {
public:
  /// @brief 构造颜色检测器
  /// @param lower_hsv HSV 下界 [h, s, v]
  /// @param upper_hsv HSV 上界 [h, s, v]
  /// @param fx 焦距 x（像素）
  /// @param fy 焦距 y（像素）
  /// @param cx 光心 x（像素）
  /// @param cy 光心 y（像素）
  ColorDetector(const std::array<int, 3>& lower_hsv,
                const std::array<int, 3>& upper_hsv,
                double fx = 614.0, double fy = 614.0,
                double cx = 320.0, double cy = 240.0);

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

}  // namespace robot_vision
