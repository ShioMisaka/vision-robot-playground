#include "robot_vision/nodes/vision_processor_node.hpp"

#include <robot_logger/logger.hpp>

#include <chrono>
#include <thread>

namespace robot_vision {

std::shared_ptr<VisionProcessorNode> VisionProcessorNode::create(
    std::shared_ptr<CameraInterface> processor, const VisionTopicConfig& config) {
  auto node = std::shared_ptr<VisionProcessorNode>(
      new VisionProcessorNode(std::move(processor), config));
  node->init(config);
  return node;
}

VisionProcessorNode::VisionProcessorNode(
    std::shared_ptr<CameraInterface> processor, const VisionTopicConfig& /*config*/)
    : Node("vision_processor_node"), processor_(std::move(processor)) {
  // init() will be called by create() after shared_from_this() is safe
}

void VisionProcessorNode::init(const VisionTopicConfig& config) {
  auto qos_profile = rclcpp::SensorDataQoS();

  left_sub_ =
      std::make_unique<message_filters::Subscriber<sensor_msgs::msg::Image>>(
          shared_from_this(), config.camera_left,
          qos_profile.get_rmw_qos_profile());

  depth_sub_ =
      std::make_unique<message_filters::Subscriber<sensor_msgs::msg::Image>>(
          shared_from_this(), config.camera_depth,
          qos_profile.get_rmw_qos_profile());

  sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(config.sync_queue_size), *left_sub_,
      *depth_sub_);
  sync_->setMaxIntervalDuration(rclcpp::Duration(
      static_cast<int64_t>(config.sync_max_slop * 1e9), 0));
  sync_->registerCallback(&VisionProcessorNode::on_synced_image, this);

  LOG_INFO("VisionProcessorNode started");
}

void VisionProcessorNode::on_synced_image(
    const sensor_msgs::msg::Image::ConstSharedPtr& left,
    const sensor_msgs::msg::Image::ConstSharedPtr& depth) {
  // 诊断：首帧打印图像尺寸信息
  static bool first_frame_logged = false;
  if (!first_frame_logged) {
    LOG_INFO("[DIAG] Image: rgb={}x{} encoding={}, depth={}x{} encoding={}",
             left->width, left->height, left->encoding,
             depth->width, depth->height, depth->encoding);
    first_frame_logged = true;
  }

  cv_bridge::CvImagePtr cv_left;
  cv_bridge::CvImagePtr cv_depth;

  try {
    cv_left = cv_bridge::toCvCopy(left, "bgr8");
    cv_depth = cv_bridge::toCvCopy(depth);
  } catch (const cv_bridge::Exception& e) {
    LOG_ERROR("CV bridge error: {}", e.what());
    return;
  }

  auto result = processor_->process_image(cv_left->image, cv_depth->image);

  {
    std::lock_guard<std::mutex> lock(result_mutex_);
    latest_result_ = result;
    has_result_ = true;
  }
  result_cv_.notify_all();
}

std::optional<DetectionResult> VisionProcessorNode::get_latest_result() const {
  std::lock_guard<std::mutex> lock(result_mutex_);
  return latest_result_;
}

std::optional<DetectionResult> VisionProcessorNode::wait_for_detection(
    double timeout) {
  std::unique_lock<std::mutex> lock(result_mutex_);

  // 立即检查已有结果
  if (has_result_ && latest_result_.has_value() &&
      latest_result_->detected) {
    return latest_result_;
  }

  auto deadline = std::chrono::steady_clock::now() +
                  std::chrono::duration<double>(timeout);

  while (result_cv_.wait_until(lock, deadline) !=
         std::cv_status::timeout) {
    if (has_result_ && latest_result_.has_value() &&
        latest_result_->detected) {
      return latest_result_;
    }
  }

  LOG_WARN("Detection timeout ({:.1f}s)", timeout);
  return std::nullopt;
}

std::optional<DetectionResult> VisionProcessorNode::average_detections(
    int sample_count, double sample_interval, double timeout) {
  auto deadline = std::chrono::steady_clock::now() +
                  std::chrono::duration<double>(timeout);

  Eigen::Vector3d sum_xyz = Eigen::Vector3d::Zero();
  Eigen::Vector2d sum_uv = Eigen::Vector2d::Zero();
  double sum_conf = 0.0;
  int valid_count = 0;

  for (int i = 0; i < sample_count; ++i) {
    if (std::chrono::steady_clock::now() >= deadline) break;

    if (i > 0) {
      std::this_thread::sleep_for(
          std::chrono::duration<double>(sample_interval));
    }

    auto result = get_latest_result();
    if (result.has_value() && result->detected) {
      sum_xyz += result->xyz;
      sum_uv += result->uv.cast<double>();
      sum_conf += result->confidence;
      ++valid_count;
    }
  }

  if (valid_count == 0) {
    LOG_WARN("average_detections: 0/{} valid samples", sample_count);
    return std::nullopt;
  }

  DetectionResult avg;
  avg.detected = true;
  avg.xyz = sum_xyz / valid_count;
  avg.uv = (sum_uv / valid_count).cast<int>();
  avg.confidence = sum_conf / valid_count;

  LOG_INFO("average_detections: {}/{} valid, xyz=[{:.4f}, {:.4f}, {:.4f}]",
           valid_count, sample_count, avg.xyz.x(), avg.xyz.y(),
           avg.xyz.z());
  return avg;
}

}  // namespace robot_vision
