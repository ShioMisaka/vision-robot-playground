#pragma once

#include <QWidget>
#include <QLabel>
#include <QPushButton>
#include <QImage>

#include <opencv2/core.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <optional>
#include <mutex>
#include <vector>
#include <algorithm>

namespace robot_hmi {

class CameraPanel : public QWidget {
  Q_OBJECT
public:
  explicit CameraPanel(QWidget* parent = nullptr);

public slots:
  void onImageReceived(const QImage& rgb, const QImage& depth,
                       const cv::Mat& depth_raw);
  void onEstopChanged(bool active);
  void onTransformUpdated(
      const std::optional<geometry_msgs::msg::TransformStamped>& tf);

private slots:
  void onToggleRgbDepth();

private:
  /// 获取像素点邻域的深度中值（5x5），减少单像素噪声
  float getMedianDepth(int img_x, int img_y) const;

  /// 构建悬浮提示文本（可扩展：后续加入三维坐标等）
  QString buildTooltipText(int img_x, int img_y) const;

  /// 鼠标位置 → 原始图像像素坐标，返回 false 表示不在画面内
  bool mapToImageCoords(const QPoint& label_pos, int& img_x, int& img_y) const;

  bool eventFilter(QObject* watched, QEvent* event) override;

  void displayCurrentImage();

  QLabel* image_label_;
  QPushButton* btn_toggle_;
  bool show_depth_ = false;
  QImage rgb_image_;
  QImage depth_image_;
  cv::Mat depth_raw_;

  /// 缓存的相机→基坐标系变换（由 MainWindow 5Hz 更新）
  mutable std::mutex tf_mutex_;
  std::optional<geometry_msgs::msg::TransformStamped> camera_to_base_;
};

}  // namespace robot_hmi
