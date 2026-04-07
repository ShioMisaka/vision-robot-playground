#pragma once

#include <memory>
#include <string>
#include <vector>
#include <array>
#include <optional>
#include <Eigen/Core>

#include "robot_control_cpp/i_robot_controller.hpp"
#include "robot_control_cpp/robot_profile.hpp"

namespace robot_control {

class IKSolver;

/// IO 桥接接口 — 将运动控制逻辑与底层通信实现解耦
/// ROS 2 节点实现此接口以提供实际的发布/订阅能力
class MotionIOBridge {
public:
  virtual ~MotionIOBridge() = default;

  /// 发布关节指令（arm + gripper）
  virtual void publish_command(const std::vector<double>& arm,
                               double finger) = 0;

  /// 获取当前关节角度反馈
  virtual std::vector<double> get_current_arm() const = 0;

  /// 获取当前夹爪宽度反馈
  virtual double get_current_finger() const = 0;

  /// 阻塞等待运动到位
  virtual bool wait_for_motion(const std::vector<double>& target_arm,
                               double finger,
                               double joint_tol, double finger_tol,
                               double timeout, double poll_interval,
                               double settle_time,
                               bool check_finger) = 0;

  /// 阻塞等待夹爪稳定（用于抓取物体时）
  virtual bool wait_for_finger_settle(int stable_count, double tol,
                                      double poll_interval,
                                      double timeout) = 0;

  /// 查询 TF 变换
  virtual std::optional<std::array<double, 6>> lookup_transform(
      const std::string& target_frame,
      const std::string& source_frame,
      double timeout) = 0;

  /// 设置当前 TCP 名称
  virtual void set_tcp_name(const std::string& name) = 0;
};

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

private:
  /// 关节空间线性插值运动
  void interpolate_to(const std::vector<double>& target, double finger,
                      int steps, double step_time, bool block);

  /// 计算当前 TCP 的 4x4 齐次变换矩阵
  Eigen::Matrix4d tcp_transform_matrix() const;

  std::shared_ptr<IKSolver> ik_;
  RobotProfile profile_;
  GripperProfile gripper_;
  std::shared_ptr<MotionIOBridge> bridge_;
  std::string current_tcp_;
  TcpConfig current_tcp_config_;
  bool grasping_ = false;  // 夹取物体标志
};

}  // namespace robot_control
