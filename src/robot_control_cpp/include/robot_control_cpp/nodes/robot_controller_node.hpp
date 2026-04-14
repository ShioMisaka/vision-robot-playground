#pragma once

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <robot_control_msgs/srv/solve_ik.hpp>
#include <robot_control_msgs/srv/move_joint.hpp>
#include <robot_control_msgs/srv/move_pose.hpp>
#include <robot_control_msgs/srv/move_linear.hpp>
#include <robot_control_msgs/srv/control_gripper.hpp>
#include <robot_control_msgs/srv/go_home.hpp>
#include <robot_control_msgs/srv/set_speed.hpp>
#include <robot_control_msgs/srv/get_robot_state.hpp>

#include <rclcpp_action/rclcpp_action.hpp>

#include <arm_control_interfaces/action/move_j.hpp>
#include <arm_control_interfaces/action/move_l.hpp>
#include <arm_control_interfaces/srv/set_tcp.hpp>
#include <arm_control_interfaces/srv/set_speed_ratio.hpp>
#include <arm_control_interfaces/srv/robot_cmd.hpp>
#include <arm_control_interfaces/msg/robot_status.hpp>
#include <arm_control_interfaces/msg/jog_command.hpp>

#include "robot_control_cpp/motion/robot_motion_controller.hpp"
#include "robot_control_cpp/kinematics/robot_profile.hpp"
#include "robot_control_cpp/nodes/topic_config.hpp"
#include "robot_control_cpp/nodes/robot_state.hpp"
#include "robot_control_cpp/nodes/trajectory_executor.hpp"

namespace robot_control {

class IKSolver;

/// MotionIOBridge 的 ROS 2 实现
class RosMotionBridge : public MotionIOBridge {
public:
  RosMotionBridge(rclcpp::Node::SharedPtr node,
                  const TopicConfig& topics,
                  std::shared_ptr<IKSolver> ik,
                  const RobotProfile& profile,
                  const GripperProfile& gripper);

  void publish_command(const std::vector<double>& arm,
                       double finger) override;
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

  /// 由节点订阅回调调用，更新关节状态并发布 TF
  void update_joint_state(
      const sensor_msgs::msg::JointState::SharedPtr msg);

  /// 获取 TF buffer（供外部查询）
  std::shared_ptr<tf2_ros::Buffer> get_tf_buffer() { return tf_buffer_; }

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

  /// @brief 获取状态机（只读）
  const RobotStateMachine& state_machine() const { return state_machine_; }

  ~RobotControllerNode();

private:
  RobotControllerNode(const RobotProfile& profile,
                      const GripperProfile& gripper,
                      const TopicConfig& topics);

  /// shared_from_this() 安全后调用
  void init();

  // Service callbacks
  void handle_solve_ik(
      const std::shared_ptr<robot_control_msgs::srv::SolveIK::Request> req,
      std::shared_ptr<robot_control_msgs::srv::SolveIK::Response> res);
  void handle_move_joint(
      const std::shared_ptr<robot_control_msgs::srv::MoveJoint::Request> req,
      std::shared_ptr<robot_control_msgs::srv::MoveJoint::Response> res);
  void handle_move_pose(
      const std::shared_ptr<robot_control_msgs::srv::MovePose::Request> req,
      std::shared_ptr<robot_control_msgs::srv::MovePose::Response> res);
  void handle_move_linear(
      const std::shared_ptr<robot_control_msgs::srv::MoveLinear::Request> req,
      std::shared_ptr<robot_control_msgs::srv::MoveLinear::Response> res);
  void handle_control_gripper(
      const std::shared_ptr<robot_control_msgs::srv::ControlGripper::Request> req,
      std::shared_ptr<robot_control_msgs::srv::ControlGripper::Response> res);
  void handle_go_home(
      const std::shared_ptr<robot_control_msgs::srv::GoHome::Request> req,
      std::shared_ptr<robot_control_msgs::srv::GoHome::Response> res);
  void handle_set_speed(
      const std::shared_ptr<robot_control_msgs::srv::SetSpeed::Request> req,
      std::shared_ptr<robot_control_msgs::srv::SetSpeed::Response> res);
  void handle_get_state(
      const std::shared_ptr<robot_control_msgs::srv::GetRobotState::Request> req,
      std::shared_ptr<robot_control_msgs::srv::GetRobotState::Response> res);

  // === Action callbacks (MoveJ) ===
  rclcpp_action::GoalResponse handle_movej_goal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const arm_control_interfaces::action::MoveJ::Goal> goal);
  rclcpp_action::CancelResponse handle_movej_cancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<arm_control_interfaces::action::MoveJ>> goal_handle);
  void handle_movej_accepted(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<arm_control_interfaces::action::MoveJ>> goal_handle);

  // === Action callbacks (MoveL) ===
  rclcpp_action::GoalResponse handle_movel_goal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const arm_control_interfaces::action::MoveL::Goal> goal);
  rclcpp_action::CancelResponse handle_movel_cancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<arm_control_interfaces::action::MoveL>> goal_handle);
  void handle_movel_accepted(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<arm_control_interfaces::action::MoveL>> goal_handle);

  // === Pendant service callbacks ===
  void handle_pendant_set_tcp(
      const std::shared_ptr<arm_control_interfaces::srv::SetTCP::Request> req,
      std::shared_ptr<arm_control_interfaces::srv::SetTCP::Response> res);
  void handle_set_speed_ratio(
      const std::shared_ptr<arm_control_interfaces::srv::SetSpeedRatio::Request> req,
      std::shared_ptr<arm_control_interfaces::srv::SetSpeedRatio::Response> res);
  void handle_robot_cmd(
      const std::shared_ptr<arm_control_interfaces::srv::RobotCmd::Request> req,
      std::shared_ptr<arm_control_interfaces::srv::RobotCmd::Response> res);

  // === Jog + watchdog ===
  void handle_jog_command(const arm_control_interfaces::msg::JogCommand::SharedPtr msg);
  void jog_watchdog_callback();

  // === Status publisher ===
  void publish_status();

  // === Emergency stop helper ===
  void emergency_stop();

  // === Action execution threads (joined on destruction) ===
  std::thread movej_thread_;
  std::thread movel_thread_;
  std::atomic<bool> shutdown_{false};

  std::shared_ptr<RosMotionBridge> bridge_;
  std::shared_ptr<RobotMotionController> controller_;
  std::shared_ptr<IKSolver> ik_;

  RobotProfile profile_;
  GripperProfile gripper_;
  TopicConfig topics_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::CallbackGroup::SharedPtr state_cbg_;
  rclcpp::CallbackGroup::SharedPtr pub_cbg_;

  rclcpp::Service<robot_control_msgs::srv::SolveIK>::SharedPtr srv_ik_;
  rclcpp::Service<robot_control_msgs::srv::MoveJoint>::SharedPtr srv_move_joint_;
  rclcpp::Service<robot_control_msgs::srv::MovePose>::SharedPtr srv_move_pose_;
  rclcpp::Service<robot_control_msgs::srv::MoveLinear>::SharedPtr srv_move_linear_;
  rclcpp::Service<robot_control_msgs::srv::ControlGripper>::SharedPtr srv_gripper_;
  rclcpp::Service<robot_control_msgs::srv::GoHome>::SharedPtr srv_home_;
  rclcpp::Service<robot_control_msgs::srv::SetSpeed>::SharedPtr srv_speed_;
  rclcpp::Service<robot_control_msgs::srv::GetRobotState>::SharedPtr srv_state_;

  // === State machine ===
  RobotStateMachine state_machine_;

  // === Trajectory executor ===
  std::unique_ptr<TrajectoryExecutor> trajectory_executor_;

  // === Global speed ratio (atomic for thread-safe access from action threads) ===
  std::atomic<double> global_speed_ratio_{1.0};

  // === Action servers ===
  rclcpp_action::Server<arm_control_interfaces::action::MoveJ>::SharedPtr movej_action_;
  rclcpp_action::Server<arm_control_interfaces::action::MoveL>::SharedPtr movel_action_;

  // === Pendant service servers ===
  rclcpp::Service<arm_control_interfaces::srv::SetTCP>::SharedPtr pendant_set_tcp_srv_;
  rclcpp::Service<arm_control_interfaces::srv::SetSpeedRatio>::SharedPtr set_speed_ratio_srv_;
  rclcpp::Service<arm_control_interfaces::srv::RobotCmd>::SharedPtr robot_cmd_srv_;

  // === Jog + watchdog ===
  rclcpp::Subscription<arm_control_interfaces::msg::JogCommand>::SharedPtr jog_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jog_result_pub_;
  rclcpp::TimerBase::SharedPtr jog_watchdog_timer_;
  rclcpp::Time last_jog_time_;

  // === Status publisher ===
  rclcpp::Publisher<arm_control_interfaces::msg::RobotStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  std::mutex ready_mutex_;
  std::condition_variable ready_cv_;
  bool ready_ = false;
};

}  // namespace robot_control
