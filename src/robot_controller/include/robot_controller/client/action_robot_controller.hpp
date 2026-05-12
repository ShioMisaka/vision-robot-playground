#pragma once

#include <array>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <robot_msgs/action/move_j.hpp>
#include <robot_msgs/action/move_l.hpp>
#include <robot_msgs/action/go_home.hpp>
#include <robot_msgs/srv/control_gripper.hpp>
#include <robot_msgs/srv/get_robot_state.hpp>
#include <robot_msgs/srv/set_speed.hpp>
#include <robot_msgs/srv/set_tcp.hpp>
#include <robot_msgs/srv/acquire_control.hpp>
#include <robot_msgs/srv/release_control.hpp>
#include <robot_msgs/srv/renew_lease.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "robot_controller/motion/i_robot_controller.hpp"

namespace robot_control {

/// IRobotController 的 ROS2 Action/Service 客户端实现
/// 通过调用独立 robot_controller_node 的 Action 和 Service 接口控制机器人。
/// 运动方法使用 Action Client（MoveJ/MoveL/GoHome），配置和查询使用 Service Client。
/// 支持租约管理：acquire_control 后自动续租，release_control 停止续租。
class ActionRobotController : public IRobotController {
 public:
  /// @brief 构造控制器
  /// @param node 共享的 ROS2 节点
  /// @param prefix 服务/Action 前缀（如 "robot_controller_node"）
  ActionRobotController(rclcpp::Node::SharedPtr node,
                        const std::string& prefix);

  ~ActionRobotController() override;

  // 禁止拷贝
  ActionRobotController(const ActionRobotController&) = delete;
  ActionRobotController& operator=(const ActionRobotController&) = delete;

  // ===== IRobotController 运动接口 =====

  void set_arm(const std::vector<double>& angles,
               bool block = true) override;
  void set_gripper(double width, bool block = true) override;
  void open_gripper(bool block = true) override;
  void close_gripper(bool block = true) override;
  void move_to_pose(const std::array<double, 3>& xyz,
                    const std::optional<std::array<double, 3>>& rpy,
                    double finger = -1.0, int steps = 0,
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

  // ===== IRobotController 租约接口 =====

  bool acquire_control(const std::string& client_name,
                       double lease_duration = 0) override;
  void release_control() override;
  bool renew_lease() override;
  std::string session_id() const override;

  // ===== IRobotController 进度回调 =====

  void set_progress_callback(ProgressCallback cb) override;

  // ===== Action/Service 客户端访问（供 RobotClient 使用）=====

  bool wait_for_actions(double timeout = 10.0);
  bool wait_for_services(double timeout = 10.0);

 private:
  /// 刷新状态缓存（关节角、末端位姿、夹爪宽度、TCP 名称）
  void refresh_state_cache() const;

  /// 调用 ControlGripper 服务
  void call_gripper(uint8_t command, double width = 0.0);

  /// 发送 MoveJ Action goal 并可选等待结果
  void send_movej_goal(robot_msgs::action::MoveJ::Goal goal, bool block);

  /// 发送 MoveL Action goal 并可选等待结果
  void send_movel_goal(robot_msgs::action::MoveL::Goal goal, bool block);

  /// 发送 GoHome Action goal 并可选等待结果
  void send_gohome_goal(robot_msgs::action::GoHome::Goal goal, bool block);

  /// 租约续约定时器回调
  void lease_renewal_callback();

  rclcpp::Node::SharedPtr node_;
  std::string prefix_;

  // ===== Action Clients =====
  rclcpp_action::Client<robot_msgs::action::MoveJ>::SharedPtr cli_movej_;
  rclcpp_action::Client<robot_msgs::action::MoveL>::SharedPtr cli_movel_;
  rclcpp_action::Client<robot_msgs::action::GoHome>::SharedPtr cli_gohome_;

  // ===== Service Clients =====
  rclcpp::Client<robot_msgs::srv::ControlGripper>::SharedPtr cli_gripper_;
  rclcpp::Client<robot_msgs::srv::SetSpeed>::SharedPtr cli_speed_;
  rclcpp::Client<robot_msgs::srv::GetRobotState>::SharedPtr cli_state_;
  rclcpp::Client<robot_msgs::srv::SetTCP>::SharedPtr cli_set_tcp_;
  rclcpp::Client<robot_msgs::srv::AcquireControl>::SharedPtr cli_acquire_;
  rclcpp::Client<robot_msgs::srv::ReleaseControl>::SharedPtr cli_release_;
  rclcpp::Client<robot_msgs::srv::RenewLease>::SharedPtr cli_renew_;

  // ===== TF =====
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ===== 租约状态 =====
  mutable std::mutex session_mutex_;
  std::string session_id_;
  rclcpp::TimerBase::SharedPtr renewal_timer_;

  // ===== 状态缓存 =====
  mutable std::mutex cache_mutex_;
  mutable std::vector<double> cached_joints_;
  mutable std::array<double, 6> cached_pose_{};
  mutable double cached_finger_ = 0.04;
  mutable std::string cached_tcp_;
  mutable double cached_speed_j_ = 50.0;
  mutable double cached_speed_l_ = 50.0;

  // ===== 进度回调 =====
  std::mutex progress_mutex_;
  ProgressCallback progress_callback_;
};

}  // namespace robot_control
