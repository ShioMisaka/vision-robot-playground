#pragma once

#include <memory>
#include <string>
#include <vector>
#include <array>
#include <optional>
#include <Eigen/Core>

#include "robot_controller/motion/i_robot_controller.hpp"
#include "robot_controller/motion/motion_io_bridge.hpp"
#include "robot_controller/kinematics/robot_profile.hpp"
#include "robot_controller/kinematics/trajectory_planner.hpp"

namespace robot_control {

class IKSolver;

/// 通用机器人运动控制器，不依赖 rclcpp
/// 通过 MotionIOBridge 与底层通信解耦
class RobotMotionController : public IRobotController {
public:
  /// @brief 构造运动控制器
  /// @param ik IK 求解器
  /// @param profile 机器人参数
  /// @param gripper 夹爪参数
  /// @param bridge IO 桥接
  RobotMotionController(std::shared_ptr<IKSolver> ik,
                        const RobotProfile& profile,
                        const GripperProfile& gripper,
                        std::shared_ptr<MotionIOBridge> bridge);

  // IRobotController 接口实现
  void set_arm(const std::vector<double>& angles,
               bool block = true) override;
  void set_gripper(double width, bool block = true) override;
  void open_gripper(bool block = true) override;
  void close_gripper(bool block = true) override;
  void move_to_pose(const std::array<double, 3>& xyz,
                    const std::optional<std::array<double, 3>>& rpy,
                    double finger = -1.0,
                    int steps = 0,
                    double step_time = 0.08,
                    bool block = true) override;
  void move_linear(const std::array<double, 3>& delta,
                   const std::string& frame = "base",
                   double finger = -1.0,
                   bool block = true) override;
  void rotate_joint(int index, double delta_angle,
                    bool block = true) override;
  void go_home(bool block = true) override;
  std::vector<double> get_joint_angles() const override;
  std::array<double, 6> get_end_effector_pose() const override;
  double get_finger_width() const override;
  void set_tcp(const std::string& name) override;
  std::string get_current_tcp() const override;
  std::optional<std::array<double, 6>> lookup_transform(
      const std::string& target_frame,
      const std::string& source_frame,
      double timeout = 1.0) override;

  // 新增 S 曲线运动接口
  void moveJ(const std::vector<double>& target_angles,
             bool block = true) override;
  void moveJ(const std::array<double, 3>& xyz,
             const std::optional<std::array<double, 3>>& rpy = std::nullopt,
             double finger = -1.0, bool block = true) override;
  void moveL(const std::array<double, 3>& xyz,
             const std::optional<std::array<double, 3>>& rpy = std::nullopt,
             double finger = -1.0, bool block = true) override;
  void set_speed(MotionMode mode, double percent) override;
  double get_speed(MotionMode mode) const override;

private:
  /// 关节空间线性插值运动
  void interpolate_to(const std::vector<double>& target, double finger,
                      int steps, double step_time, bool block);

  /// 计算当前 TCP 的 4x4 齐次变换矩阵
  Eigen::Matrix4d tcp_transform_matrix() const;

  /// 关节空间 S 曲线 moveJ 执行（含 finger，由 pose 版 moveJ 内部调用）
  void moveJ_internal(const std::vector<double>& target_angles,
                      double finger, bool block);

  std::shared_ptr<IKSolver> ik_;
  RobotProfile profile_;
  GripperProfile gripper_;
  std::shared_ptr<MotionIOBridge> bridge_;
  std::string current_tcp_;
  TcpConfig current_tcp_config_;
  bool grasping_ = false;  // 夹取物体标志
  double movej_speed_ = 50.0;
  double movel_speed_ = 50.0;
};

}  // namespace robot_control
