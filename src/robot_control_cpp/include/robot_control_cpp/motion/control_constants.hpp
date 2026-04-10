#pragma once

namespace robot_control {

struct ControlConstants {
  static constexpr double kJointTolerance = 0.05;
  static constexpr double kFingerTolerance = 0.002;
  static constexpr double kMotionTimeout = 10.0;
  static constexpr double kPollInterval = 0.02;
  static constexpr double kSettleTime = 0.2;
  static constexpr int kDefaultSteps = 10;
  static constexpr double kDefaultStepTime = 0.08;
  static constexpr int kImageSyncQueueSize = 10;
  static constexpr double kImageSyncSlop = 0.1;
  static constexpr int kFingerStableCount = 5;
  static constexpr double kFingerStableTol = 0.001;
  static constexpr double kReadyTimeout = 5.0;
  static constexpr double kTrajectoryDt = 0.02;
};

}  // namespace robot_control
