#include "teaching_pendant/panels/camera_panel.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QPixmap>
#include <QSizePolicy>

namespace teaching_pendant {

CameraPanel::CameraPanel(QWidget* parent) : QWidget(parent) {
  auto* group = new QGroupBox("Camera", this);
  auto* layout = new QVBoxLayout(group);

  image_label_ = new QLabel(this);
  image_label_->setAlignment(Qt::AlignCenter);
  image_label_->setMinimumSize(640, 480);
  image_label_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  image_label_->setStyleSheet("background-color: #2b2b2b; border: 1px solid #555;");
  image_label_->setText("No camera feed");

  layout->addWidget(image_label_, 1);

  auto* btn_layout = new QHBoxLayout();
  btn_toggle_ = new QPushButton("Switch to Depth", this);
  btn_toggle_->setMaximumWidth(160);
  connect(btn_toggle_, &QPushButton::clicked, this, &CameraPanel::onToggleRgbDepth);
  btn_layout->addWidget(btn_toggle_);
  btn_layout->addStretch();
  layout->addLayout(btn_layout);

  auto* outer = new QVBoxLayout(this);
  outer->setContentsMargins(0, 0, 0, 0);
  outer->addWidget(group);
}

void CameraPanel::onImageReceived(const QImage& image) {
  image_label_->setPixmap(QPixmap::fromImage(image));
}

void CameraPanel::onEstopChanged(bool active) {
  if (active) {
    image_label_->setText("E-STOP ACTIVATED");
    image_label_->setStyleSheet(
        "background-color: #cc0000; color: white; font-size: 24px; "
        "font-weight: bold; border: 2px solid #ff0000;");
  } else {
    image_label_->setText("No camera feed");
    image_label_->setStyleSheet(
        "background-color: #2b2b2b; border: 1px solid #555;");
  }
}

void CameraPanel::onToggleRgbDepth() {
  show_depth_ = !show_depth_;
  // TODO: implement depth display
}

}  // namespace teaching_pendant
