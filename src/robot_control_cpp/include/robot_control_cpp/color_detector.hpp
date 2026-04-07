#pragma once

#include <array>
#include <opencv2/core.hpp>
#include <Eigen/Core>
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
