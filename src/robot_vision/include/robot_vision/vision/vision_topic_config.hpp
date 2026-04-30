#pragma once

#include <string>

namespace robot_vision {

/// 视觉节点的相机话题配置
/// 独立于 robot_controller::TopicConfig，仅包含视觉所需的字段
struct VisionTopicConfig {
  std::string camera_left = "/camera/image_raw/left";
  std::string camera_depth = "/camera/image_raw/depth";

  /// 图像同步参数
  int sync_queue_size = 10;
  double sync_max_slop = 0.1;  // 秒
};

}  // namespace robot_vision
