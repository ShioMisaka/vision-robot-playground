#include "robot_controller/nodes/robot_state.hpp"
#include <mutex>
#include <shared_mutex>
#include <stdexcept>

namespace robot_control {

const char* RobotStateMachine::state_name(RobotState s) {
  switch (s) {
    case RobotState::kIdle:     return "IDLE";
    case RobotState::kMoving:   return "MOVING";
    case RobotState::kTeaching: return "TEACHING";
    case RobotState::kStopping: return "STOPPING";
    case RobotState::kFault:    return "FAULT";
    default:                    return "UNKNOWN";
  }
}

bool RobotStateMachine::is_valid_transition(RobotState from, RobotState to) {
  if (to == RobotState::kFault) return true;  // EMERGENCY_STOP from any state
  switch (from) {
    case RobotState::kIdle:
      return to == RobotState::kMoving || to == RobotState::kTeaching;
    case RobotState::kMoving:
      return to == RobotState::kIdle || to == RobotState::kStopping;
    case RobotState::kTeaching:
      return to == RobotState::kIdle || to == RobotState::kStopping;
    case RobotState::kStopping:
      return to == RobotState::kIdle || to == RobotState::kFault;
    case RobotState::kFault:
      return to == RobotState::kIdle;
    default:
      return false;
  }
}

RobotState RobotStateMachine::state() const {
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return state_;
}

bool RobotStateMachine::transition_to(RobotState target) {
  std::unique_lock<std::shared_mutex> lock(mutex_);
  if (!is_valid_transition(state_, target)) {
    return false;
  }
  state_ = target;
  return true;
}

void RobotStateMachine::force_state(RobotState target) {
  std::unique_lock<std::shared_mutex> lock(mutex_);
  state_ = target;
}

int32_t RobotStateMachine::error_code() const {
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return error_code_;
}

std::string RobotStateMachine::error_message() const {
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return error_message_;
}

void RobotStateMachine::set_error(int32_t code, const std::string& message) {
  std::unique_lock<std::shared_mutex> lock(mutex_);
  error_code_ = code;
  error_message_ = message;
}

void RobotStateMachine::clear_error() {
  std::unique_lock<std::shared_mutex> lock(mutex_);
  error_code_ = 0;
  error_message_.clear();
}

}  // namespace robot_control
