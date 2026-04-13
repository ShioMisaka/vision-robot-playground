#pragma once

#include <QWidget>
#include <QLabel>
#include <QPushButton>
#include <QImage>

namespace teaching_pendant {

class CameraPanel : public QWidget {
  Q_OBJECT
public:
  explicit CameraPanel(QWidget* parent = nullptr);

public slots:
  void onImageReceived(const QImage& image);
  void onEstopChanged(bool active);

private slots:
  void onToggleRgbDepth();

private:
  QLabel* image_label_;
  QPushButton* btn_toggle_;
  bool show_depth_ = false;
};

}  // namespace teaching_pendant
