#pragma once

#include <array>
#include <chrono>
#include <memory>
#include <optional>
#include <string>

#include "robot_control_cpp/i_robot_controller.hpp"
#include "robot_control_cpp/i_vision_processor.hpp"

namespace robot_control {

/// 抓取任务状态
enum class GraspState {
  kIdle,
  kDetecting,
  kApproaching,
  kDescending,
  kGrasping,
  kLifting,
  kDone,
  kError
};

/// 视觉引导抓取任务管理器
/// 在主线程中运行状态机，协调视觉与运动控制
class GraspTaskManager {
public:
  /// @brief 构造抓取任务管理器
  /// @param robot 运动控制接口
  /// @param vision 视觉处理接口
  /// @param approach_height 接近时目标上方偏移高度（米）
  /// @param grasp_height_offset 抓取时高度偏移（米）
  /// @param grasp_rpy 抓取姿态 [roll, pitch, yaw]（弧度）
  GraspTaskManager(std::shared_ptr<IRobotController> robot,
                   std::shared_ptr<IVisionProcessor> vision,
                   double approach_height = 0.15,
                   double grasp_height_offset = 0.02,
                   const std::array<double, 3>& grasp_rpy = {
                       3.14159265, 0.0, 3.14159265});

  /// @brief 运行完整抓取流程（阻塞）
  /// @param timeout 整体超时（秒）
  /// @return true=成功
  bool run(double timeout = 30.0);

  /// @brief 获取当前状态
  GraspState get_state() const { return state_; }

private:
  bool step_detect();
  void step_approach();
  void step_descend();
  void step_grasp();
  void step_lift();

  /// 将相机坐标系 3D 点转换到基座坐标系
  std::optional<std::array<double, 3>> transform_to_base(
      const Eigen::Vector3d& camera_xyz);

  std::shared_ptr<IRobotController> robot_;
  std::shared_ptr<IVisionProcessor> vision_;

  double approach_height_;
  double grasp_height_offset_;
  std::array<double, 3> grasp_rpy_;

  GraspState state_ = GraspState::kIdle;
  std::optional<std::array<double, 3>> target_xyz_;
};

}  // namespace robot_control
