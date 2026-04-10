#pragma once

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

#include "robot_control_cpp/motion/robot_motion_controller.hpp"
#include "robot_control_cpp/kinematics/robot_profile.hpp"
#include "robot_control_cpp/nodes/topic_config.hpp"

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

private:
  RobotControllerNode(const RobotProfile& profile,
                      const GripperProfile& gripper,
                      const TopicConfig& topics);

  /// shared_from_this() 安全后调用
  void init();

  std::shared_ptr<RosMotionBridge> bridge_;
  std::shared_ptr<RobotMotionController> controller_;
  std::shared_ptr<IKSolver> ik_;

  RobotProfile profile_;
  GripperProfile gripper_;
  TopicConfig topics_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::CallbackGroup::SharedPtr state_cbg_;
  rclcpp::CallbackGroup::SharedPtr pub_cbg_;

  std::mutex ready_mutex_;
  std::condition_variable ready_cv_;
  bool ready_ = false;
};

}  // namespace robot_control
