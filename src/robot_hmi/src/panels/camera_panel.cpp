#include "robot_hmi/panels/camera_panel.hpp"

#include <robot_logger/logger.hpp>
#include "robot_description/camera_config.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QPixmap>
#include <QSizePolicy>
#include <QToolTip>
#include <QHelpEvent>

#include <cmath>

namespace robot_hmi {

CameraPanel::CameraPanel(QWidget* parent) : QWidget(parent) {
  auto* group = new QGroupBox("Camera", this);
  auto* layout = new QVBoxLayout(group);

  image_label_ = new QLabel(this);
  image_label_->setAlignment(Qt::AlignCenter);
  image_label_->setMinimumSize(640, 480);
  image_label_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  image_label_->setStyleSheet("background-color: #2b2b2b; border: 1px solid #555;");
  image_label_->setText("No camera feed");
  image_label_->setMouseTracking(true);
  image_label_->installEventFilter(this);

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

void CameraPanel::onImageReceived(const QImage& rgb, const QImage& depth,
                                  const cv::Mat& depth_raw) {
  rgb_image_ = rgb;
  depth_image_ = depth;
  depth_raw_ = depth_raw;
  if (!estop_active_) {
    displayCurrentImage();
  }
}

void CameraPanel::displayCurrentImage() {
  const QImage& img = show_depth_ ? depth_image_ : rgb_image_;
  if (img.isNull()) return;
  image_label_->setPixmap(QPixmap::fromImage(img));
}

void CameraPanel::onEstopChanged(bool active) {
  estop_active_ = active;
  if (active) {
    image_label_->setPixmap(QPixmap());
    image_label_->setText("E-STOP ACTIVATED");
    image_label_->setStyleSheet(
        "background-color: #cc0000; color: white; font-size: 24px; "
        "font-weight: bold; border: 2px solid #ff0000;");
  } else {
    image_label_->setText("No camera feed");
    image_label_->setStyleSheet(
        "background-color: #2b2b2b; border: 1px solid #555;");
    displayCurrentImage();
  }
}

void CameraPanel::onToggleRgbDepth() {
  show_depth_ = !show_depth_;
  btn_toggle_->setText(show_depth_ ? "Switch to RGB" : "Switch to Depth");
  displayCurrentImage();
}

void CameraPanel::onTransformUpdated(
    const std::optional<geometry_msgs::msg::TransformStamped>& tf) {
  std::lock_guard<std::mutex> lock(tf_mutex_);
  camera_to_base_ = tf;
}

// ===== 鼠标悬停深度提示 =====

bool CameraPanel::eventFilter(QObject* watched, QEvent* event) {
  if (watched != image_label_) {
    return QWidget::eventFilter(watched, event);
  }

  if (event->type() == QEvent::MouseMove) {
    auto* me = static_cast<QMouseEvent*>(event);
    int img_x = 0, img_y = 0;
    if (mapToImageCoords(me->pos(), img_x, img_y)) {
      QString text = buildTooltipText(img_x, img_y);
      QToolTip::showText(image_label_->mapToGlobal(me->pos()), text,
                         image_label_);
    } else {
      QToolTip::hideText();
    }
    return true;
  }

  if (event->type() == QEvent::MouseButtonPress) {
    auto* me = static_cast<QMouseEvent*>(event);
    int img_x = 0, img_y = 0;
    if (mapToImageCoords(me->pos(), img_x, img_y)) {
      printClickDiag(img_x, img_y);
    }
    return true;
  }

  if (event->type() == QEvent::Leave) {
    QToolTip::hideText();
    return true;
  }

  return QWidget::eventFilter(watched, event);
}

bool CameraPanel::mapToImageCoords(const QPoint& label_pos, int& img_x,
                                   int& img_y) const {
  const QPixmap pm = image_label_->pixmap(Qt::ReturnByValue);
  if (pm.isNull() || depth_raw_.empty()) return false;

  // QLabel 居中显示 pixmap，计算 pixmap 在 label 中的偏移
  int ox = (image_label_->width() - pm.width()) / 2;
  int oy = (image_label_->height() - pm.height()) / 2;

  int px = label_pos.x() - ox;
  int py = label_pos.y() - oy;
  if (px < 0 || px >= pm.width() || py < 0 || py >= pm.height()) return false;

  // pixmap → 原始图像坐标（四舍五入避免截断导致的 ±1 像素抖动）
  float sx = static_cast<float>(depth_raw_.cols) / pm.width();
  float sy = static_cast<float>(depth_raw_.rows) / pm.height();
  img_x = std::min(static_cast<int>(px * sx + 0.5f), depth_raw_.cols - 1);
  img_y = std::min(static_cast<int>(py * sy + 0.5f), depth_raw_.rows - 1);
  return true;
}

void CameraPanel::printClickDiag(int img_x, int img_y) const {
  if (depth_raw_.empty()) return;

  constexpr double fx = robot_description::CameraIntrinsics::kFx;
  constexpr double fy = robot_description::CameraIntrinsics::kFy;
  constexpr double cx = robot_description::CameraIntrinsics::kCx;
  constexpr double cy = robot_description::CameraIntrinsics::kCy;

  float depth = getMedianDepth(img_x, img_y);
  if (depth <= 0) {
    LOG_WARN("[CLICK] 深度无效");
    return;
  }

  double x3d = (img_x - cx) * depth / fx;
  double y3d = (img_y - cy) * depth / fy;
  double z3d = depth;

  LOG_INFO("======== [CLICK DIAG] ========");
  LOG_INFO("  图像分辨率: {}x{}", depth_raw_.cols, depth_raw_.rows);
  LOG_INFO("  像素: ({}, {})", img_x, img_y);
  LOG_INFO("  深度: {:.4f} m", depth);
  LOG_INFO("  3D(Cam): ({:.5f}, {:.5f}, {:.5f})", x3d, y3d, z3d);
  LOG_INFO("  内参: fx={:.2f} fy={:.2f} cx={:.1f} cy={:.1f}", fx, fy, cx, cy);

  auto base = transformToBase(x3d, y3d, z3d);
  if (base.has_value()) {
    LOG_INFO("  3D(Base): ({:.5f}, {:.5f}, {:.5f})",
             (*base)[0], (*base)[1], (*base)[2]);
  } else {
    LOG_WARN("  变换: N/A (camera_to_base 未就绪)");
  }
  LOG_INFO("================================");
}

float CameraPanel::getMedianDepth(int img_x, int img_y) const {
  std::vector<float> values;
  values.reserve(25);
  // 5x5 邻域中值滤波，减少单像素深度噪声
  for (int dy = -2; dy <= 2; ++dy) {
    for (int dx = -2; dx <= 2; ++dx) {
      int ny = img_y + dy;
      int nx = img_x + dx;
      if (ny >= 0 && ny < depth_raw_.rows && nx >= 0 && nx < depth_raw_.cols) {
        float d = depth_raw_.at<float>(ny, nx);
        if (d > 0) values.push_back(d);
      }
    }
  }
  if (values.empty()) return 0;
  std::nth_element(values.begin(), values.begin() + values.size() / 2,
                   values.end());
  return values[values.size() / 2];
}

QString CameraPanel::buildTooltipText(int img_x, int img_y) const {
  // 相机内参（统一配置，来自 robot_description::CameraIntrinsics）
  constexpr double fx = robot_description::CameraIntrinsics::kFx;
  constexpr double fy = robot_description::CameraIntrinsics::kFy;
  constexpr double cx = robot_description::CameraIntrinsics::kCx;
  constexpr double cy = robot_description::CameraIntrinsics::kCy;

  // ---- 深度值（5x5 中值滤波减少噪声） ----
  float depth = getMedianDepth(img_x, img_y);
  QString depth_str =
      (depth > 0) ? QString("%1 m").arg(depth, 0, 'f', 3) : QStringLiteral("无效");

  // ---- 像素坐标 + 图像尺寸诊断 ----
  QString size_str;
  if (!depth_raw_.empty()) {
    size_str = QString("(%1,%2) [%3x%4]")
                   .arg(img_x).arg(img_y)
                   .arg(depth_raw_.cols).arg(depth_raw_.rows);
  } else {
    size_str = QString("(%1, %2)").arg(img_x).arg(img_y);
  }

  // ---- 相机坐标系 3D 坐标 ----
  QString xyz_str;
  QString base_str;
  if (depth > 0) {
    double x3d = (img_x - cx) * depth / fx;
    double y3d = (img_y - cy) * depth / fy;
    double z3d = depth;
    xyz_str = QString("(%1, %2, %3) m")
                  .arg(x3d, 0, 'f', 3)
                  .arg(y3d, 0, 'f', 3)
                  .arg(z3d, 0, 'f', 3);

    auto base = transformToBase(x3d, y3d, z3d);
    if (base.has_value()) {
      base_str = QString("(%1, %2, %3) m")
                     .arg((*base)[0], 0, 'f', 3)
                     .arg((*base)[1], 0, 'f', 3)
                     .arg((*base)[2], 0, 'f', 3);
    } else {
      base_str = QStringLiteral("N/A");
    }
  } else {
    xyz_str = QStringLiteral("无效");
    base_str = QStringLiteral("N/A");
  }

  return QString("<p style='white-space:pre;margin:2px;"
                  "background:#ffffcc;padding:3px;border-radius:2px'>"
                  "<b>像素:</b> %1<br>"
                  "<b>深度:</b> %2<br>"
                  "<b>3D(Cam):</b> %3<br>"
                  "<b>3D(Base):</b> %4"
                  "</p>")
      .arg(size_str, depth_str, xyz_str, base_str);
}

std::optional<std::array<double, 3>> CameraPanel::transformToBase(
    double x3d, double y3d, double z3d) const {
  std::optional<geometry_msgs::msg::TransformStamped> tf;
  {
    std::lock_guard<std::mutex> lock(tf_mutex_);
    tf = camera_to_base_;
  }
  if (!tf.has_value()) return std::nullopt;

  // 四元数 → 旋转矩阵，变换相机系 3D 点到 base 系
  double qx = tf->transform.rotation.x;
  double qy = tf->transform.rotation.y;
  double qz = tf->transform.rotation.z;
  double qw = tf->transform.rotation.w;
  double r00 = 1 - 2*(qy*qy + qz*qz), r01 = 2*(qx*qy - qz*qw), r02 = 2*(qx*qz + qy*qw);
  double r10 = 2*(qx*qy + qz*qw),     r11 = 1 - 2*(qx*qx + qz*qz), r12 = 2*(qy*qz - qx*qw);
  double r20 = 2*(qx*qz - qy*qw),     r21 = 2*(qy*qz + qx*qw),     r22 = 1 - 2*(qx*qx + qy*qy);
  double tx = tf->transform.translation.x;
  double ty = tf->transform.translation.y;
  double tz = tf->transform.translation.z;

  return std::array<double, 3>{
      r00*x3d + r01*y3d + r02*z3d + tx,
      r10*x3d + r11*y3d + r12*z3d + ty,
      r20*x3d + r21*y3d + r22*z3d + tz};
}

}  // namespace robot_hmi
