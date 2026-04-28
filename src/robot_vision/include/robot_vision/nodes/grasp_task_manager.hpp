#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <optional>
#include <string>

#include "robot_controller/motion/i_robot_controller.hpp"
#include "robot_vision/vision/i_vision_processor.hpp"

namespace robot_vision {

using robot_control::IRobotController;

/// 抓取任务状态
enum class GraspState {
  kIdle,
  kDetecting,
  kApproaching,
  kReDetecting,
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
  /// @param redetect_samples 近距离重检测采样次数
  /// @param redetect_interval 重检测采样间隔（秒）
  /// @param max_reach 机器人最大工作半径（米），超出则拒绝抓取
  /// @param approach_step_size 动态逼近每步距离（米）
  /// @param approach_tolerance 到达判定容差（米）
  /// @param max_approach_steps 最大逼近步数
  /// @param max_consecutive_failures 连续检测失败上限
  /// @param hand_frame 末端手爪坐标系名称
  /// @param camera_offset 相机安装偏移 [x,y,z]（相对 hand 坐标系，米）
  /// @param camera_rpy 相机安装姿态 [roll,pitch,yaw]（相对 hand 坐标系，弧度）
  GraspTaskManager(std::shared_ptr<IRobotController> robot,
                   std::shared_ptr<IVisionProcessor> vision,
                   const std::string& base_frame = "panda_link0",
                   const std::string& camera_frame = "camera_color_optical_frame",
                   double approach_height = 0.15,
                   double grasp_height_offset = 0.02,
                   const std::array<double, 3>& grasp_rpy = {
                       3.14159265, 0.0, 3.14159265},
                   int redetect_samples = 5,
                   double redetect_interval = 0.1,
                   double max_reach = 0.85,
                   double approach_step_size = 0.025,
                   double approach_tolerance = 0.01,
                   int max_approach_steps = 50,
                   int max_consecutive_failures = 3,
                   const std::string& hand_frame = "panda_hand",
                   const std::array<double, 3>& camera_offset = {
                       0.015, 0.0, 0.03},
                   const std::array<double, 3>& camera_rpy = {
                       0.0, -1.57079632679, 0.0});

  /// @brief 运行完整抓取流程（阻塞）
  /// @param timeout 整体超时（秒）
  /// @return true=成功
  bool run(double timeout = 30.0);

  /// @brief 获取当前状态
  GraspState get_state() const { return state_.load(std::memory_order_relaxed); }

  /// @brief 请求中止抓取（线程安全，供信号处理等调用）
  void request_abort() { abort_requested_.store(true, std::memory_order_release); }

  /// @brief 是否已请求中止
  bool is_abort_requested() const { return abort_requested_.load(std::memory_order_acquire); }

  /// @brief 将相机坐标系 3D 点转换到基座坐标系
  /// @param camera_xyz 相机光学坐标系下的 3D 点
  /// @param log_details 是否记录详细坐标信息
  /// @return 基座坐标系下的 3D 点，失败返回 nullopt
  std::optional<std::array<double, 3>> transform_to_base(
      const Eigen::Vector3d& camera_xyz, bool log_details = false);

private:
  bool step_detect();
  void step_approach();
  bool step_redetect();
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
  double max_reach_;

  double approach_step_size_;
  double approach_tolerance_;
  int max_approach_steps_;
  int max_consecutive_failures_;

  int redetect_samples_;
  double redetect_interval_;

  std::string hand_frame_;
  std::array<double, 3> camera_offset_;
  std::array<double, 3> camera_rpy_;

  std::atomic<GraspState> state_ = GraspState::kIdle;
  std::atomic<bool> abort_requested_{false};
  std::optional<std::array<double, 3>> target_xyz_;
};

}  // namespace robot_vision
