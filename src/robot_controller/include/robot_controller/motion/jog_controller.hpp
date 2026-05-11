#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <vector>

#include <Eigen/Core>

namespace robot_control {

class IKSolver;

/// @brief Jog 速度限制与 S-curve 形状配置
struct JogConfig {
  double linear_speed = 0.05;       ///< m/s for XYZ（50 mm/s）
  double angular_speed = 0.2;       ///< rad/s for RPY（~11 deg/s）
  double a_max = 5.0;               ///< velocity scale 最大加速度（1/s²）
  double j_max = 50.0;              ///< velocity scale 最大 jerk（1/s³）
  double dt = 0.02;                 ///< tick 周期（秒，50Hz）
  int dof = 7;                      ///< 关节数

  /// 每关节速度限制（rad/s）
  std::array<double, 7> joint_vel_limits = {
      2.175, 2.175, 2.175, 2.175, 2.610, 2.610, 2.610};
};

/// @brief 坐标系常量（与 JogCommand.msg 一致）
constexpr uint8_t kTcpFrame = 0;
constexpr uint8_t kBaseFrame = 1;

/// @brief 纯 C++ Jog 控制器（无 ROS 依赖）
///
/// 封装 S-curve 速度规划、Jacobian 速度 IK、
/// TCP→Base 坐标变换、关节速度限制与同步缩放。
///
/// 使用模式:
///   1. start(axis, frame) 或 start_raw(velocity, frame)
///   2. 以 config_.dt 为周期调用 tick()
///   3. tick() 返回 false 时 Jog 完成
///   4. stop() 进入减速；emergency_stop() 立即停止
class JogController {
public:
  /// @brief 构造 Jog 控制器
  /// @param ik IK 求解器（共享指针，须比本对象长寿）
  /// @param config Jog 配置参数
  explicit JogController(std::shared_ptr<IKSolver> ik,
                         const JogConfig& config = JogConfig());

  ~JogController();

  // 禁止拷贝
  JogController(const JogController&) = delete;
  JogController& operator=(const JogController&) = delete;

  /// @brief 从 UI axis 索引启动 Jog
  /// @param axis 0=+X,1=-X,2=+Y,3=-Y,4=+Z,5=-Z,
  ///             6=+R,7=-R,8=+P,9=-P,10=+Yw,11=-Yw
  /// @param frame kTcpFrame(0) 或 kBaseFrame(1)
  void start(int axis, uint8_t frame);

  /// @brief 从原始速度向量启动 Jog（直接使用 JogCommand 数据）
  /// @param velocity 6D 笛卡尔速度 [vx,vy,vz,vr,vp,vyaw]
  /// @param frame kTcpFrame(0) 或 kBaseFrame(1)
  void start_raw(const std::array<double, 6>& velocity, uint8_t frame);

  /// @brief 请求平滑停止（S-curve 减速）
  void stop();

  /// @brief 急停（立即停止，位置归零到反馈值）
  /// @param feedback_joints 当前实际关节角度
  void emergency_stop(const std::array<double, 7>& feedback_joints);

  /// @brief 重置所有状态
  void reset();

  /// @brief 执行一个 tick 的 Jog 计算
  /// @param feedback_joints 当前实际关节角度（用于初始化位置）
  /// @return true 仍在运行（加速/匀速/减速中），false 减速完成
  bool tick(const std::array<double, 7>& feedback_joints);

  /// @brief Jog 是否处于活动状态
  bool is_active() const { return jog_active_; }

  /// @brief 是否处于减速阶段
  bool is_stopping() const { return jog_stopping_; }

  /// @brief 获取内部指令关节位置
  std::array<double, 7> get_commanded_joints() const { return jog_q_current_; }

  /// @brief 获取当前速度比例（0..1）
  double get_velocity_scale() const { return jog_v_; }

  /// @brief 获取当前目标笛卡尔速度
  const std::array<double, 6>& get_target_velocity() const { return target_velocity_; }

private:
  void init_position(const std::array<double, 7>& feedback_joints);

  /// @brief S-curve 速度 ramp（jerk-limited 加减速控制）
  /// @return true 继续（加速/匀速/减速中），false 减速完成
  bool update_velocity_ramp();

  /// @brief 关节速度限制与同步缩放，超限时回退笛卡尔偏移
  /// @param ik_result IK 解算结果（会被就地修改）
  /// @param delta 本 tick 的笛卡尔增量（用于回退偏移）
  void enforce_joint_limits(std::vector<double>& ik_result,
                            const std::array<double, 6>& delta);

  std::shared_ptr<IKSolver> ik_;
  JogConfig config_;

  // 目标速度指令
  std::array<double, 6> target_velocity_{};
  uint8_t frame_ = kBaseFrame;

  // S-curve ramp 状态（归一化 0..1 速度比例）
  double jog_v_ = 0.0;          ///< 当前速度比例（0..1）
  double jog_a_ = 0.0;          ///< 当前加速度（1/s²）
  bool jog_stopping_ = false;   ///< 减速阶段
  bool jog_active_ = false;     ///< Jog 活动状态

  // 内部位置追踪（避免反馈延迟）
  std::array<double, 7> jog_q_current_{};

  // 笛卡尔偏移累积（位置式 IK：精确累积，无积分误差）
  std::array<double, 6> cartesian_offset_{};  ///< [dx, dy, dz, dRx, dRy, dRz] base frame

  // 初始位姿参考（用于计算目标笛卡尔位姿）
  Eigen::Vector3d initial_position_{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d initial_rotation_{Eigen::Matrix3d::Identity()};
  bool jog_pose_initialized_ = false;

};

}  // namespace robot_control
