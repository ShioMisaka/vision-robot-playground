#pragma once

#include <array>
#include <chrono>
#include <memory>
#include <optional>
#include <string>

#include "robot_controller/motion/i_robot_controller.hpp"
#include "robot_vision/vision/i_vision_processor.hpp"

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
  /// @param base_frame 基座坐标系名称
  /// @param camera_frame 相机光学坐标系名称
  /// @param approach_height 接近时目标上方偏移高度（米）
  /// @param grasp_height_offset 抓取时高度偏移（米）
  /// @param grasp_rpy 抓取姿态 [roll, pitch, yaw]（弧度）
  GraspTaskManager(std::shared_ptr<IRobotController> robot,
                   std::shared_ptr<IVisionProcessor> vision,
                   const std::string& base_frame = "panda_link0",
                   const std::string& camera_frame = "camera_color_optical_frame",
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

  /// @brief 将相机坐标系 3D 点转换到基座坐标系
  /// @param camera_xyz 相机光学坐标系下的 3D 点
  /// @return 基座坐标系下的 3D 点，失败返回 nullopt
  std::optional<std::array<double, 3>> transform_to_base(
      const Eigen::Vector3d& camera_xyz);

private:
  bool step_detect();
  void step_approach();
  void step_descend();
  void step_grasp();
  void step_lift();

  std::shared_ptr<IRobotController> robot_;
  std::shared_ptr<IVisionProcessor> vision_;

  std::string base_frame_;
  std::string camera_frame_;

  double approach_height_;
  double grasp_height_offset_;
  std::array<double, 3> grasp_rpy_;

  GraspState state_ = GraspState::kIdle;
  std::optional<std::array<double, 3>> target_xyz_;
};

}  // namespace robot_control
