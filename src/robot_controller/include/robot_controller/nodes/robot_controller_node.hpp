#pragma once

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <robot_msgs/srv/solve_ik.hpp>
#include <robot_msgs/srv/move_joint.hpp>
#include <robot_msgs/srv/move_pose.hpp>
#include <robot_msgs/srv/move_linear.hpp>
#include <robot_msgs/srv/control_gripper.hpp>
#include <robot_msgs/srv/go_home.hpp>
#include <robot_msgs/srv/set_speed.hpp>
#include <robot_msgs/srv/get_robot_state.hpp>

#include <robot_msgs/srv/set_tcp.hpp>
#include <robot_msgs/srv/set_speed_ratio.hpp>
#include <robot_msgs/srv/robot_cmd.hpp>
#include <robot_msgs/msg/robot_status.hpp>
#include <robot_msgs/msg/jog_command.hpp>

#include "robot_controller/motion/robot_motion_controller.hpp"
#include "robot_controller/kinematics/robot_profile.hpp"
#include "robot_controller/motion/topic_config.hpp"
#include "robot_controller/nodes/robot_state.hpp"
#include "robot_controller/nodes/robot_state_model.hpp"
#include "robot_controller/motion/jog_controller.hpp"
#include "robot_controller/nodes/setpoint_generator.hpp"
#include "robot_controller/nodes/motion_owner.hpp"

namespace robot_control {

class IKSolver;

/// MotionIOBridge 的 ROS 2 实现
class RosMotionBridge : public MotionIOBridge {
public:
  using TrajectoryStartedCallback = std::function<void(MotionSource)>;

  RosMotionBridge(rclcpp::Node::SharedPtr node,
                  const TopicConfig& topics,
                  std::shared_ptr<IKSolver> ik,
                  const RobotProfile& profile,
                  const GripperProfile& gripper);

  void publish_command(const std::vector<double>& arm,
                       double finger) override;

  /// 仅发布臂关节指令（不包含夹爪，用于 kIdle 时发布外部关节目标）
  void publish_arm(const std::vector<double>& arm) override;

  /// 仅发布夹爪关节指令（kIdle 时维持夹持力，不覆盖 arm）
  void publish_gripper(double finger) override;
  std::vector<double> get_current_arm() const override;
  double get_current_finger() const override;
  bool wait_for_motion(const std::vector<double>& target_arm,
                       double finger,
                       double joint_tol, double finger_tol,
                       double timeout, double poll_interval,
                       double settle_time,
                       bool check_finger) override;
  bool wait_for_finger_settle(int stable_count, double tol,
                              double poll_interval,
                              double timeout) override;
  std::optional<std::array<double, 6>> lookup_transform(
      const std::string& target_frame,
      const std::string& source_frame,
      double timeout) override;
  void set_tcp_name(const std::string& name) override;

  void submit_trajectory(
      const std::vector<TrajectoryStep>& steps, double finger,
      MotionSource source = MotionSource::kApi) override;
  bool wait_trajectory_completion(double timeout) override;
  void cancel_trajectory() override;

  /// 通知阻塞等待者轨迹完成（由控制循环调用）
  void notify_trajectory_complete();

  /// 获取 SetpointGenerator（供控制循环使用）
  SetpointGenerator& setpoint_generator() { return setpoint_gen_; }

  /// 由节点订阅回调调用，更新关节状态并发布 TF
  void update_joint_state(
      const sensor_msgs::msg::JointState::SharedPtr msg);

  /// 获取 TF buffer（供外部查询）
  std::shared_ptr<tf2_ros::Buffer> get_tf_buffer() { return tf_buffer_; }

  /// 设置轨迹启动回调（由 RobotControllerNode 注册，用于切换状态机）
  void set_on_trajectory_started(TrajectoryStartedCallback cb) {
    on_trajectory_started_ = std::move(cb);
  }

private:
  void publish_ee_tf(const sensor_msgs::msg::JointState::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  TopicConfig topics_;
  RobotProfile profile_;
  GripperProfile gripper_;
  std::shared_ptr<IKSolver> ik_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr cmd_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  mutable std::mutex state_mutex_;
  std::vector<double> current_arm_;
  double current_finger_ = 0.04;

  std::string current_tcp_name_;
  TcpConfig current_tcp_config_;

  SetpointGenerator setpoint_gen_;
  std::mutex trajectory_mutex_;
  std::condition_variable trajectory_cv_;
  TrajectoryStartedCallback on_trajectory_started_;

  /// 发布 hand → camera_link → camera_color_optical_frame 静态 TF
  void publish_camera_tf();
};

/// ROS 2 机器人控制节点
/// 使用 create() 工厂方法构造（内部需要 shared_from_this）
class RobotControllerNode : public rclcpp::Node {
public:
  /// @brief 工厂方法：创建控制节点
  /// @param profile 机器人参数
  /// @param gripper 夹爪参数
  /// @param topics 话题配置
  static std::shared_ptr<RobotControllerNode> create(
      const RobotProfile& profile,
      const GripperProfile& gripper,
      const TopicConfig& topics);

  /// @brief 阻塞等待首次关节状态接收
  bool wait_for_ready(double timeout = 5.0);

  /// @brief 获取底层运动控制器
  std::shared_ptr<RobotMotionController> get_controller() {
    return controller_;
  }

  /// @brief 获取 TF buffer
  std::shared_ptr<tf2_ros::Buffer> get_tf_buffer() {
    return bridge_->get_tf_buffer();
  }

  /// @brief 获取底层 IO bridge
  std::shared_ptr<RosMotionBridge> get_bridge() { return bridge_; }

  /// @brief 获取 IK 求解器（供同进程节点直接调用，绕过 DDS）
  std::shared_ptr<IKSolver> get_ik_solver() { return ik_; }

  /// @brief 获取状态机（只读）
  const RobotStateMachine& state_machine() const { return state_machine_; }

  /// @brief 声明运动控制权
  void claim_ownership(MotionOwner owner) { motion_owner_.store(owner); }
  /// @brief 释放运动控制权
  void release_ownership() { motion_owner_.store(MotionOwner::kNone); }
  /// @brief 获取当前运动控制权持有者
  MotionOwner get_motion_owner() const { return motion_owner_.load(); }

  ~RobotControllerNode();

private:
  RobotControllerNode(const RobotProfile& profile,
                      const GripperProfile& gripper,
                      const TopicConfig& topics);

  /// shared_from_this() 安全后调用
  void init();

  // Service callbacks
  void handle_solve_ik(
      const std::shared_ptr<robot_msgs::srv::SolveIK::Request> req,
      std::shared_ptr<robot_msgs::srv::SolveIK::Response> res);
  void handle_move_joint(
      const std::shared_ptr<robot_msgs::srv::MoveJoint::Request> req,
      std::shared_ptr<robot_msgs::srv::MoveJoint::Response> res);
  void handle_move_pose(
      const std::shared_ptr<robot_msgs::srv::MovePose::Request> req,
      std::shared_ptr<robot_msgs::srv::MovePose::Response> res);
  void handle_move_linear(
      const std::shared_ptr<robot_msgs::srv::MoveLinear::Request> req,
      std::shared_ptr<robot_msgs::srv::MoveLinear::Response> res);
  void handle_control_gripper(
      const std::shared_ptr<robot_msgs::srv::ControlGripper::Request> req,
      std::shared_ptr<robot_msgs::srv::ControlGripper::Response> res);
  void handle_go_home(
      const std::shared_ptr<robot_msgs::srv::GoHome::Request> req,
      std::shared_ptr<robot_msgs::srv::GoHome::Response> res);
  void handle_set_speed(
      const std::shared_ptr<robot_msgs::srv::SetSpeed::Request> req,
      std::shared_ptr<robot_msgs::srv::SetSpeed::Response> res);
  void handle_get_state(
      const std::shared_ptr<robot_msgs::srv::GetRobotState::Request> req,
      std::shared_ptr<robot_msgs::srv::GetRobotState::Response> res);

  // === Pendant service callbacks ===
  void handle_pendant_set_tcp(
      const std::shared_ptr<robot_msgs::srv::SetTCP::Request> req,
      std::shared_ptr<robot_msgs::srv::SetTCP::Response> res);
  void handle_set_speed_ratio(
      const std::shared_ptr<robot_msgs::srv::SetSpeedRatio::Request> req,
      std::shared_ptr<robot_msgs::srv::SetSpeedRatio::Response> res);
  void handle_robot_cmd(
      const std::shared_ptr<robot_msgs::srv::RobotCmd::Request> req,
      std::shared_ptr<robot_msgs::srv::RobotCmd::Response> res);

  // === Jog + watchdog ===
  void handle_jog_command(const robot_msgs::msg::JogCommand::SharedPtr msg);
  void jog_watchdog_callback();

  // === Status publisher ===
  void publish_status();

  // === Emergency stop helper ===
  void emergency_stop();

  // === 100Hz 控制循环 ===
  rclcpp::TimerBase::SharedPtr control_loop_timer_;
  RobotStateModel state_model_;

  std::chrono::steady_clock::time_point stopping_start_time_;
  bool waiting_settle_ = false;

  // === 控制循环方法 ===
  void control_loop_tick();

  std::atomic<bool> shutdown_{false};
  std::atomic<MotionOwner> motion_owner_{MotionOwner::kNone};

  std::shared_ptr<RosMotionBridge> bridge_;
  std::shared_ptr<RobotMotionController> controller_;
  std::shared_ptr<IKSolver> ik_;

  RobotProfile profile_;
  GripperProfile gripper_;
  TopicConfig topics_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::CallbackGroup::SharedPtr state_cbg_;
  rclcpp::CallbackGroup::SharedPtr pub_cbg_;

  rclcpp::Service<robot_msgs::srv::SolveIK>::SharedPtr srv_ik_;
  rclcpp::Service<robot_msgs::srv::MoveJoint>::SharedPtr srv_move_joint_;
  rclcpp::Service<robot_msgs::srv::MovePose>::SharedPtr srv_move_pose_;
  rclcpp::Service<robot_msgs::srv::MoveLinear>::SharedPtr srv_move_linear_;
  rclcpp::Service<robot_msgs::srv::ControlGripper>::SharedPtr srv_gripper_;
  rclcpp::Service<robot_msgs::srv::GoHome>::SharedPtr srv_home_;
  rclcpp::Service<robot_msgs::srv::SetSpeed>::SharedPtr srv_speed_;
  rclcpp::Service<robot_msgs::srv::GetRobotState>::SharedPtr srv_state_;

  // === State machine ===
  RobotStateMachine state_machine_;

  // === Global speed ratio (atomic for thread-safe access) ===
  std::atomic<double> global_speed_ratio_{1.0};

  // === Pendant service servers ===
  rclcpp::Service<robot_msgs::srv::SetTCP>::SharedPtr pendant_set_tcp_srv_;
  rclcpp::Service<robot_msgs::srv::SetSpeedRatio>::SharedPtr set_speed_ratio_srv_;
  rclcpp::Service<robot_msgs::srv::RobotCmd>::SharedPtr robot_cmd_srv_;

  // === Jog + watchdog ===
  rclcpp::Subscription<robot_msgs::msg::JogCommand>::SharedPtr jog_sub_;
  rclcpp::TimerBase::SharedPtr jog_watchdog_timer_;
  rclcpp::Time last_jog_time_;

  // === Jog controller (pure C++ math) ===
  std::unique_ptr<JogController> jog_controller_;

  // === Jog settling ===
  bool jog_settling_ = false;
  std::chrono::steady_clock::time_point jog_settle_start_time_;

  // === Status publisher ===
  rclcpp::Publisher<robot_msgs::msg::RobotStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  std::mutex ready_mutex_;
  std::condition_variable ready_cv_;
  bool ready_ = false;

  // === External joint target stream (from pendant) ===
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr external_joint_sub_;
  std::mutex external_target_mutex_;
  std::vector<double> external_joint_target_;
  rclcpp::Time external_target_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace robot_control
