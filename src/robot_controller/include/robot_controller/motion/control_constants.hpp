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
  static constexpr double kFollowingErrorLimit = 0.1;   ///< 轨迹执行跟踪误差阈值（rad）
  static constexpr double kTeachingFollowErrorLimit = 0.3;  ///< Jog 模式跟踪误差阈值（rad），比轨迹宽松
  static constexpr double kArrivalTolerance = 0.01;      ///< 到位判定阈值（rad）
  static constexpr double kArrivalSettleTime = 0.2;      ///< 到位稳定等待时间（秒）
  static constexpr double kTrajectoryTimeout = 15.0;     ///< 轨迹执行超时（秒）
  static constexpr double kControlLoopHz = 100.0;        ///< 控制循环频率（Hz）
  static constexpr double kControlLoopDt = 1.0 / 100.0;  ///< 控制循环周期（秒）
};

}  // namespace robot_control
