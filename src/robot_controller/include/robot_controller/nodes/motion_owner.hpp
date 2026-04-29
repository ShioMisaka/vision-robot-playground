#pragma once

#include <cstdint>

namespace robot_control {

/// 当前运动控制权的持有者
enum class MotionOwner : uint8_t {
  kNone = 0,      ///< 无活跃持有者 — kIdle 保持当前位置
  kPendant = 1,   ///< 示教器 — 通过 joint_target 流 / jog / service 控制
  kScript = 2,    ///< 外部脚本 — 通过 C++ API / Python 控制
};

/// 轨迹提交来源（决定所有权归属）
enum class MotionSource : uint8_t {
  kService = 0,   ///< 来自 ROS2 Service 回调（示教器发起）
  kApi = 1,       ///< 来自 C++ API 直接调用（脚本发起）
};

}  // namespace robot_control
