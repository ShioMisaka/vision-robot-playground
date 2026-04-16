#include "robot_vision/vision/color_detector.hpp"

#include <opencv2/imgproc.hpp>

#include <cmath>
#include <vector>

namespace robot_control {

// ---- CameraInterface ----

Eigen::Vector3d CameraInterface::pixel_to_3d(int u, int v, double depth,
                                            double fx, double fy,
                                            double cx, double cy) {
  double x = (u - cx) * depth / fx;
  double y = (v - cy) * depth / fy;
  return Eigen::Vector3d(x, y, depth);
}

// ---- ColorDetector ----

ColorDetector::ColorDetector(const std::array<int, 3>& lower_hsv,
                             const std::array<int, 3>& upper_hsv)
    : lower_hsv_(static_cast<double>(lower_hsv[0]),
                 static_cast<double>(lower_hsv[1]),
                 static_cast<double>(lower_hsv[2])),
      upper_hsv_(static_cast<double>(upper_hsv[0]),
                 static_cast<double>(upper_hsv[1]),
                 static_cast<double>(upper_hsv[2])) {}

std::optional<Eigen::Vector2i> ColorDetector::detect(
    const cv::Mat& bgr_image) const {
  cv::Mat hsv;
  cv::cvtColor(bgr_image, hsv, cv::COLOR_BGR2HSV);

  cv::Mat mask;
  cv::inRange(hsv, lower_hsv_, upper_hsv_, mask);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  if (contours.empty()) {
    return std::nullopt;
  }

  // 找最大轮廓
  int max_idx = 0;
  double max_area = 0.0;
  for (size_t i = 0; i < contours.size(); ++i) {
    double area = cv::contourArea(contours[i]);
    if (area > max_area) {
      max_area = area;
      max_idx = static_cast<int>(i);
    }
  }

  cv::Moments m = cv::moments(contours[max_idx]);
  if (m.m00 < 1e-6) {
    return std::nullopt;
  }

  int cx = static_cast<int>(m.m10 / m.m00);
  int cy = static_cast<int>(m.m01 / m.m00);
  return Eigen::Vector2i(cx, cy);
}

void ColorDetector::draw_target(cv::Mat& image, int cx, int cy,
                                const std::string& label) const {
  cv::circle(image, cv::Point(cx, cy), 15, cv::Scalar(0, 255, 0), 2);
  cv::line(image, cv::Point(cx - 20, cy), cv::Point(cx + 20, cy),
           cv::Scalar(0, 255, 0), 2);
  cv::line(image, cv::Point(cx, cy - 20), cv::Point(cx, cy + 20),
           cv::Scalar(0, 255, 0), 2);
  cv::putText(image, label + ": (" + std::to_string(cx) + ", " +
                  std::to_string(cy) + ")",
              cv::Point(cx - 60, cy - 30), cv::FONT_HERSHEY_SIMPLEX, 0.6,
              cv::Scalar(0, 255, 0), 2);
}

DetectionResult ColorDetector::process_image(const cv::Mat& rgb,
                                             const cv::Mat& depth) const {
  DetectionResult result;
  auto uv = detect(rgb);
  if (!uv.has_value()) {
    return result;
  }

  result.detected = true;
  result.uv = *uv;

  // 获取深度值
  double depth_value = 0.0;
  if (depth.type() == CV_16U) {
    depth_value =
        static_cast<double>(depth.at<uint16_t>(uv->y(), uv->x())) / 1000.0;
  } else if (depth.type() == CV_32F) {
    depth_value = static_cast<double>(depth.at<float>(uv->y(), uv->x()));
  }

  if (depth_value > 0.0) {
    result.xyz = pixel_to_3d(uv->x(), uv->y(), depth_value,
                             fx_, fy_, cx_, cy_);
  }

  return result;
}

void ColorDetector::set_camera_intrinsics(double fx, double fy, double cx,
                                          double cy) {
  fx_ = fx;
  fy_ = fy;
  cx_ = cx;
  cy_ = cy;
}

}  // namespace robot_control
