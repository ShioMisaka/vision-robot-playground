/// @file demo_camera.cpp
/// @brief C++ 版 test_camera.py：显示左目 RGB 与深度图
///
/// 编译后运行（需要 Isaac Sim 已启动）：
///   source /opt/ros/jazzy/setup.bash
///   source install/setup.bash
///   ros2 run robot_control_test demo_camera
///
/// 功能：
///   1. 订阅左目 RGB 和深度图话题（message_filters 同步）
///   2. RGB 原色显示，深度图归一化后伪彩色显示
///   3. 按 q 退出

#include <chrono>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#include "robot_controller/motion/control_constants.hpp"
#include "robot_controller/motion/topic_config.hpp"

using namespace std::chrono_literals;

/// 相机图像显示节点
class CameraDisplayNode : public rclcpp::Node {
public:
  static std::shared_ptr<CameraDisplayNode> create(const robot_control::TopicConfig& topics) {
    auto node = std::shared_ptr<CameraDisplayNode>(
        new CameraDisplayNode(topics));
    node->init(topics);
    return node;
  }

  /// 获取最新帧（线程安全）
  std::optional<std::pair<cv::Mat, cv::Mat>> get_frames() const {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    if (left_frame_.empty() || depth_frame_.empty()) {
      return std::nullopt;
    }
    return std::make_pair(left_frame_.clone(), depth_frame_.clone());
  }

private:
  CameraDisplayNode(const robot_control::TopicConfig& topics)
      : Node("camera_display_node") {
    (void)topics;
  }

  void init(const robot_control::TopicConfig& topics) {
    auto qos = rclcpp::SensorDataQoS();

    left_sub_ = std::make_unique<message_filters::Subscriber<sensor_msgs::msg::Image>>(
        shared_from_this(), topics.camera_left, qos.get_rmw_qos_profile());
    depth_sub_ = std::make_unique<message_filters::Subscriber<sensor_msgs::msg::Image>>(
        shared_from_this(), topics.camera_depth, qos.get_rmw_qos_profile());

    sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(robot_control::ControlConstants::kImageSyncQueueSize),
        *left_sub_, *depth_sub_);
    sync_->setMaxIntervalDuration(rclcpp::Duration(
        static_cast<int64_t>(robot_control::ControlConstants::kImageSyncSlop * 1e9), 0));
    sync_->registerCallback(&CameraDisplayNode::on_images, this);

    RCLCPP_INFO(this->get_logger(),
                "CameraDisplayNode 已启动，订阅：[%s, %s]",
                topics.camera_left.c_str(), topics.camera_depth.c_str());
  }

  void on_images(const sensor_msgs::msg::Image::ConstSharedPtr& left_msg,
                 const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg) {
    cv_bridge::CvImagePtr cv_left;
    cv_bridge::CvImagePtr cv_depth;

    try {
      cv_left = cv_bridge::toCvCopy(left_msg, "bgr8");
      cv_depth = cv_bridge::toCvCopy(depth_msg);
    } catch (const cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge 转换失败: %s", e.what());
      return;
    }

    // 归一化深度图到 0-255 灰度
    cv::Mat depth_vis;
    if (cv_depth->image.depth() == CV_16U) {
      cv_depth->image.convertTo(depth_vis, CV_8U, 255.0 / 5000.0, 0);
    } else if (cv_depth->image.depth() == CV_32F) {
      cv_depth->image.convertTo(depth_vis, CV_8U, 255.0 / 5.0, 0);
    } else {
      cv::normalize(cv_depth->image, depth_vis, 0, 255, cv::NORM_MINMAX, CV_8U);
    }

    // 伪彩色
    cv::Mat depth_color;
    cv::applyColorMap(depth_vis, depth_color, cv::COLORMAP_JET);

    std::lock_guard<std::mutex> lock(frame_mutex_);
    left_frame_ = cv_left->image;
    depth_frame_ = depth_color;

    RCLCPP_INFO(this->get_logger(),
                "收到图像: 左目 %dx%d encoding=%s, 深度 %dx%d encoding=%s",
                left_msg->width, left_msg->height, left_msg->encoding.c_str(),
                depth_msg->width, depth_msg->height, depth_msg->encoding.c_str());
  }

  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> left_sub_;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> depth_sub_;
  std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  mutable std::mutex frame_mutex_;
  cv::Mat left_frame_;
  cv::Mat depth_frame_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  robot_control::TopicConfig topics;
  auto display_node = CameraDisplayNode::create(topics);

  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(display_node);
  auto spin_thread = std::thread([executor]() { executor->spin(); });

  RCLCPP_INFO(display_node->get_logger(), "等待相机图像...按 q 退出");

  try {
    while (rclcpp::ok()) {
      auto frames = display_node->get_frames();
      if (frames) {
        auto& [left, depth] = *frames;

        // 统一宽度，上下布局
        int width = std::max(left.cols, depth.cols);
        cv::Mat left_resized, depth_resized;
        if (left.cols != width) {
          cv::resize(left, left_resized, {width, left.rows});
        } else {
          left_resized = left;
        }
        if (depth.cols != width) {
          cv::resize(depth, depth_resized, {width, depth.rows});
        } else {
          depth_resized = depth;
        }

        cv::Mat combined;
        cv::vconcat(left_resized, depth_resized, combined);
        cv::imshow("Left RGB / Depth (Jet)", combined);
      } else {
        // 还没收到图像，短暂等待
        std::this_thread::sleep_for(100ms);
        continue;
      }

      int key = cv::waitKey(1) & 0xFF;
      if (key == 'q') {
        RCLCPP_INFO(display_node->get_logger(), "用户退出");
        break;
      }
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(display_node->get_logger(), "异常: %s", e.what());
  }

  cv::destroyAllWindows();
  executor->cancel();
  spin_thread.join();
  rclcpp::shutdown();
  return 0;
}
