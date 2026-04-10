#pragma once

#include <cstdint>
#include <mutex>
#include <string>
#include <vector>
#include <array>

namespace robot_control {

/// @brief 机器人运行状态
enum class RobotState : uint8_t {
  kIdle = 0,
  kMoving = 1,
  kTeaching = 2,
  kStopping = 3,
  kFault = 4,
};

/// @brief 状态机管理器（零 ROS 依赖，线程安全）
class RobotStateMachine {
public:
  RobotStateMachine() = default;

  /// @brief 获取当前状态
  RobotState state() const;

  /// @brief 尝试状态转换
  /// @return true 转换成功，false 转换被拒绝
  bool transition_to(RobotState target);

  /// @brief 强制设置状态（仅用于 EMERGENCY_STOP）
  void force_state(RobotState target);

  /// @brief 获取状态名（用于日志）
  static const char* state_name(RobotState s);

  /// @brief 获取错误码
  int32_t error_code() const;

  /// @brief 获取错误信息
  std::string error_message() const;

  /// @brief 设置错误
  void set_error(int32_t code, const std::string& message);

  /// @brief 清除错误
  void clear_error();

private:
  mutable std::mutex mutex_;
  RobotState state_{RobotState::kIdle};
  int32_t error_code_{0};
  std::string error_message_;

  static bool is_valid_transition(RobotState from, RobotState to);
};

}  // namespace robot_control
