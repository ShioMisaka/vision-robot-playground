#include "robot_control_cpp/nodes/vision_processor_node.hpp"
#include "robot_control_cpp/nodes/topic_config.hpp"
#include "robot_control_cpp/motion/control_constants.hpp"

#include <chrono>

namespace robot_control {

std::shared_ptr<VisionProcessorNode> VisionProcessorNode::create(
    std::shared_ptr<CameraInterface> processor, const TopicConfig& topics) {
  auto node = std::shared_ptr<VisionProcessorNode>(
      new VisionProcessorNode(std::move(processor), topics));
  node->init(topics);
  return node;
}

VisionProcessorNode::VisionProcessorNode(
    std::shared_ptr<CameraInterface> processor, const TopicConfig& /*topics*/)
    : Node("vision_processor_node"), processor_(std::move(processor)) {
  // init() will be called by create() after shared_from_this() is safe
}

void VisionProcessorNode::init(const TopicConfig& topics) {
  auto qos_profile = rclcpp::SensorDataQoS();

  left_sub_ =
      std::make_unique<message_filters::Subscriber<sensor_msgs::msg::Image>>(
          shared_from_this(), topics.camera_left,
          qos_profile.get_rmw_qos_profile());

  depth_sub_ =
      std::make_unique<message_filters::Subscriber<sensor_msgs::msg::Image>>(
          shared_from_this(), topics.camera_depth,
          qos_profile.get_rmw_qos_profile());

  sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(ControlConstants::kImageSyncQueueSize), *left_sub_,
      *depth_sub_);
  sync_->setMaxIntervalDuration(rclcpp::Duration(
      static_cast<int64_t>(ControlConstants::kImageSyncSlop * 1e9), 0));
  sync_->registerCallback(&VisionProcessorNode::on_synced_image, this);

  RCLCPP_INFO(this->get_logger(), "VisionProcessorNode started");
}

void VisionProcessorNode::on_synced_image(
    const sensor_msgs::msg::Image::ConstSharedPtr& left,
    const sensor_msgs::msg::Image::ConstSharedPtr& depth) {
  cv_bridge::CvImagePtr cv_left;
  cv_bridge::CvImagePtr cv_depth;

  try {
    cv_left = cv_bridge::toCvCopy(left, "bgr8");
    cv_depth = cv_bridge::toCvCopy(depth, "passthrough");
  } catch (const cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "CV bridge error: %s", e.what());
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
  auto deadline = std::chrono::steady_clock::now() +
                  std::chrono::duration<double>(timeout);

  while (result_cv_.wait_until(lock, deadline) !=
         std::cv_status::timeout) {
    if (has_result_ && latest_result_.has_value() &&
        latest_result_->detected) {
      return latest_result_;
    }
  }

  RCLCPP_WARN(this->get_logger(), "Detection timeout (%.1fs)", timeout);
  return std::nullopt;
}

}  // namespace robot_control
