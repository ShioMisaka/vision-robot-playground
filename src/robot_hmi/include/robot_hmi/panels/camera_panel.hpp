#pragma once

#include <QWidget>
#include <QLabel>
#include <QPushButton>
#include <QImage>

#include <opencv2/core.hpp>

namespace robot_hmi {

class CameraPanel : public QWidget {
  Q_OBJECT
public:
  explicit CameraPanel(QWidget* parent = nullptr);

public slots:
  void onImageReceived(const QImage& rgb, const QImage& depth,
                       const cv::Mat& depth_raw);
  void onEstopChanged(bool active);

private slots:
  void onToggleRgbDepth();

private:
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
};

}  // namespace robot_hmi
