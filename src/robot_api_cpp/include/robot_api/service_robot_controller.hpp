#pragma once

#include <array>
#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

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

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "robot_controller/motion/i_robot_controller.hpp"

namespace robot_api {

/// IRobotController 的 ROS2 Service 客户端实现
/// 通过调用独立 robot_controller_node 的 Service 接口控制机器人
class ServiceRobotController : public robot_control::IRobotController {
 public:
  ServiceRobotController(rclcpp::Node::SharedPtr node,
                          const std::string& service_prefix);

  // IRobotController 接口
  void set_arm(const std::vector<double>& angles, bool block = true) override;
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
             double finger = -1.0, bool block = true,
             robot_control::MotionSource source =
                 robot_control::MotionSource::kApi) override;
  void moveL(const std::array<double, 3>& xyz,
             const std::optional<std::array<double, 3>>& rpy = std::nullopt,
             double finger = -1.0, bool block = true,
             robot_control::MotionSource source =
                 robot_control::MotionSource::kApi) override;
  void set_speed(robot_control::MotionMode mode, double percent) override;
  double get_speed(robot_control::MotionMode mode) const override;

 private:
  // Allow RobotClient to check service readiness
  friend class RobotClient;
  void refresh_state_cache();
  void call_move_joint(const std::vector<double>& angles, bool block);
  void call_move_pose(const std::array<double, 3>& xyz,
                      const std::optional<std::array<double, 3>>& rpy,
                      uint8_t mode, double finger);
  void call_gripper(uint8_t command, double width = 0.0);

  rclcpp::Node::SharedPtr node_;
  std::string prefix_;

  // Service clients
  rclcpp::Client<robot_msgs::srv::MoveJoint>::SharedPtr cli_move_joint_;
  rclcpp::Client<robot_msgs::srv::MovePose>::SharedPtr cli_move_pose_;
  rclcpp::Client<robot_msgs::srv::MoveLinear>::SharedPtr cli_move_linear_;
  rclcpp::Client<robot_msgs::srv::ControlGripper>::SharedPtr cli_gripper_;
  rclcpp::Client<robot_msgs::srv::GoHome>::SharedPtr cli_home_;
  rclcpp::Client<robot_msgs::srv::SetSpeed>::SharedPtr cli_speed_;
  rclcpp::Client<robot_msgs::srv::GetRobotState>::SharedPtr cli_state_;
  rclcpp::Client<robot_msgs::srv::SetTCP>::SharedPtr cli_set_tcp_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Cached state
  mutable std::mutex cache_mutex_;
  mutable std::vector<double> cached_joints_;
  mutable std::array<double, 6> cached_pose_{};
  mutable double cached_finger_ = 0.04;
  mutable std::string cached_tcp_;
  mutable double cached_speed_j_ = 50.0;
  mutable double cached_speed_l_ = 50.0;
};

}  // namespace robot_api
