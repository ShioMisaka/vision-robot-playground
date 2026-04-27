#pragma once

#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.hpp>

#include "robot_vision/vision/camera_interface.hpp"
#include "robot_vision/vision/i_vision_processor.hpp"
#include "robot_controller/motion/topic_config.hpp"

namespace robot_vision {

using robot_control::TopicConfig;

/// ROS 2 视觉处理节点
/// 通过 message_filters 同步订阅 RGB + 深度图像
class VisionProcessorNode : public rclcpp::Node, public IVisionProcessor {
public:
  /// @brief 工厂方法：创建视觉处理节点
  static std::shared_ptr<VisionProcessorNode> create(
      std::shared_ptr<CameraInterface> processor,
      const TopicConfig& topics);

  // IVisionProcessor 接口
  std::optional<DetectionResult> get_latest_result() const override;
  std::optional<DetectionResult> wait_for_detection(
      double timeout = 10.0) override;

private:
  VisionProcessorNode(std::shared_ptr<CameraInterface> processor,
                      const TopicConfig& topics);
  void init(const TopicConfig& topics);

  void on_synced_image(
      const sensor_msgs::msg::Image::ConstSharedPtr& left,
      const sensor_msgs::msg::Image::ConstSharedPtr& depth);

  std::shared_ptr<CameraInterface> processor_;

  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>>
      left_sub_;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>>
      depth_sub_;
  std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  mutable std::mutex result_mutex_;
  std::optional<DetectionResult> latest_result_;
  bool has_result_ = false;
  mutable std::condition_variable result_cv_;
};

}  // namespace robot_vision
