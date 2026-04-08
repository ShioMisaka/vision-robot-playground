#include "robot_control_cpp/robot_controller_node.hpp"
#include "robot_control_cpp/ik_solver.hpp"
#include "robot_control_cpp/config.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <string>
#include <thread>
#include <vector>

namespace robot_control {

// ===== RosMotionBridge =====

RosMotionBridge::RosMotionBridge(rclcpp::Node::SharedPtr node,
                                 const TopicConfig& topics,
                                 std::shared_ptr<IKSolver> ik,
                                 const RobotProfile& profile)
    : node_(std::move(node)),
      topics_(topics),
      profile_(profile),
      ik_(std::move(ik)),
      current_tcp_name_(profile.default_tcp),
      current_tcp_config_(profile.tcp_frames.at(profile.default_tcp)) {
  cmd_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
      topics_.joint_command, 10);

  tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*node_);
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // 初始化当前关节状态
  current_arm_.resize(profile_.dof, 0.0);
}

void RosMotionBridge::publish_command(const std::vector<double>& arm,
                                      double finger) {
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = node_->now();

  for (size_t i = 0; i < arm.size(); ++i) {
    msg.name.push_back(profile_.joint_names[i]);
    msg.position.push_back(arm[i]);
  }
  // 夹爪双指
  msg.name.push_back("panda_finger_joint1");
  msg.position.push_back(finger);
  msg.name.push_back("panda_finger_joint2");
  msg.position.push_back(finger);

  cmd_pub_->publish(msg);
}

std::vector<double> RosMotionBridge::get_current_arm() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return current_arm_;
}

double RosMotionBridge::get_current_finger() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return current_finger_;
}

bool RosMotionBridge::wait_for_motion(const std::vector<double>& target_arm,
                                      double finger, double joint_tol,
                                      double finger_tol, double timeout,
                                      double poll_interval,
                                      double settle_time,
                                      bool check_finger) {
  auto deadline = std::chrono::steady_clock::now() +
                  std::chrono::duration<double>(timeout);

  while (std::chrono::steady_clock::now() < deadline) {
    std::vector<double> current;
    double current_finger;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      current = current_arm_;
      current_finger = current_finger_;
    }

    bool arm_ok = true;
    for (size_t i = 0; i < target_arm.size() && i < current.size(); ++i) {
      if (std::abs(target_arm[i] - current[i]) > joint_tol) {
        arm_ok = false;
        break;
      }
    }
    bool finger_ok =
        !check_finger || std::abs(finger - current_finger) < finger_tol;

    if (arm_ok && finger_ok) {
      std::this_thread::sleep_for(
          std::chrono::duration<double>(settle_time));
      // settle 后重新检查，防止振荡导致误判到位
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current = current_arm_;
        current_finger = current_finger_;
      }
      arm_ok = true;
      for (size_t i = 0; i < target_arm.size() && i < current.size(); ++i) {
        if (std::abs(target_arm[i] - current[i]) > joint_tol) {
          arm_ok = false;
          break;
        }
      }
      finger_ok =
          !check_finger || std::abs(finger - current_finger) < finger_tol;
      if (arm_ok && finger_ok) {
        return true;
      }
    }

    std::this_thread::sleep_for(
        std::chrono::duration<double>(poll_interval));
  }

  return false;
}

bool RosMotionBridge::wait_for_finger_settle(int stable_count, double tol,
                                             double poll_interval,
                                             double timeout) {
  auto deadline = std::chrono::steady_clock::now() +
                  std::chrono::duration<double>(timeout);
  int count = 0;
  double last = get_current_finger();

  while (std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(
        std::chrono::duration<double>(poll_interval));
    double curr = get_current_finger();
    if (std::abs(curr - last) < tol) {
      ++count;
      if (count >= stable_count) {
        return true;
      }
    } else {
      count = 0;
      last = curr;
    }
  }
  return false;
}

std::optional<std::array<double, 6>> RosMotionBridge::lookup_transform(
    const std::string& target_frame, const std::string& source_frame,
    double timeout) {
  try {
    auto tf = tf_buffer_->lookupTransform(
        target_frame, source_frame,
        tf2::TimePointZero,
        tf2::durationFromSec(timeout));

    double x = tf.transform.translation.x;
    double y = tf.transform.translation.y;
    double z = tf.transform.translation.z;

    double roll, pitch, yaw;
    tf2::Quaternion q(tf.transform.rotation.x, tf.transform.rotation.y,
                      tf.transform.rotation.z, tf.transform.rotation.w);
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    return std::array<double, 6>{x, y, z, roll, pitch, yaw};
  } catch (const tf2::TransformException& e) {
    RCLCPP_WARN(node_->get_logger(), "TF lookup failed: %s", e.what());
    return std::nullopt;
  }
}

void RosMotionBridge::set_tcp_name(const std::string& name) {
  auto it = profile_.tcp_frames.find(name);
  if (it != profile_.tcp_frames.end()) {
    current_tcp_name_ = name;
    current_tcp_config_ = it->second;
  }
}

void RosMotionBridge::update_joint_state(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
  {
    std::lock_guard<std::mutex> lock(state_mutex_);

    current_arm_.clear();
    current_finger_ = 0.0;

    // 构建关节名到位置的映射
    std::map<std::string, double> name_to_pos;
    for (size_t i = 0; i < msg->name.size(); ++i) {
      name_to_pos[msg->name[i]] = msg->position[i];
    }

    for (const auto& jname : profile_.joint_names) {
      auto it = name_to_pos.find(jname);
      current_arm_.push_back(
          it != name_to_pos.end() ? it->second : 0.0);
    }

    auto it = name_to_pos.find("panda_finger_joint1");
    if (it != name_to_pos.end()) {
      current_finger_ = it->second;
    }
  }

  publish_ee_tf(msg);
}

void RosMotionBridge::publish_ee_tf(
    const sensor_msgs::msg::JointState::SharedPtr /*msg*/) {
  try {
    std::vector<double> arm;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      arm = current_arm_;
    }

    auto pose = ik_->forward(arm);
    auto stamp = node_->now();

    // 发布 base -> hand
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = stamp;
    t.header.frame_id = profile_.base_frame;
    t.child_frame_id = profile_.hand_frame;
    t.transform.translation.x = pose[0];
    t.transform.translation.y = pose[1];
    t.transform.translation.z = pose[2];

    tf2::Quaternion q;
    q.setRPY(pose[3], pose[4], pose[5]);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(t);

    // 发布 hand -> tcp（如果有非默认 TCP）
    if (current_tcp_name_ != "hand") {
      geometry_msgs::msg::TransformStamped t_tcp;
      t_tcp.header.stamp = stamp;
      t_tcp.header.frame_id = profile_.hand_frame;
      t_tcp.child_frame_id = current_tcp_name_;
      t_tcp.transform.translation.x = current_tcp_config_.offset_xyz[0];
      t_tcp.transform.translation.y = current_tcp_config_.offset_xyz[1];
      t_tcp.transform.translation.z = current_tcp_config_.offset_xyz[2];

      tf2::Quaternion q_tcp;
      q_tcp.setRPY(current_tcp_config_.offset_rpy[0],
                   current_tcp_config_.offset_rpy[1],
                   current_tcp_config_.offset_rpy[2]);
      t_tcp.transform.rotation.x = q_tcp.x();
      t_tcp.transform.rotation.y = q_tcp.y();
      t_tcp.transform.rotation.z = q_tcp.z();
      t_tcp.transform.rotation.w = q_tcp.w();

      tf_broadcaster_->sendTransform(t_tcp);
    }
  } catch (...) {
    // FK 失败不影响主流程
  }
}

// ===== RobotControllerNode =====

std::shared_ptr<RobotControllerNode> RobotControllerNode::create(
    const RobotProfile& profile,
    const GripperProfile& gripper,
    const TopicConfig& topics) {
  auto node = std::shared_ptr<RobotControllerNode>(
      new RobotControllerNode(profile, gripper, topics));
  node->init();
  return node;
}

RobotControllerNode::RobotControllerNode(const RobotProfile& profile,
                                         const GripperProfile& gripper,
                                         const TopicConfig& topics)
    : Node("robot_controller_node"),
      profile_(profile),
      gripper_(gripper),
      topics_(topics) {
  state_cbg_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
  pub_cbg_ = create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);

  ik_ = std::make_shared<IKSolver>(profile);
}

void RobotControllerNode::init() {
  bridge_ = std::make_shared<RosMotionBridge>(
      shared_from_this(), topics_, ik_, profile_);

  controller_ = std::make_shared<RobotMotionController>(
      ik_, profile_, gripper_, bridge_);

  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = state_cbg_;

  joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      topics_.joint_state, 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        bridge_->update_joint_state(msg);

        {
          std::lock_guard<std::mutex> lock(ready_mutex_);
          ready_ = true;
        }
        ready_cv_.notify_all();
      },
      sub_opts);

  RCLCPP_INFO(this->get_logger(), "RobotControllerNode started");
}

bool RobotControllerNode::wait_for_ready(double timeout) {
  std::unique_lock<std::mutex> lock(ready_mutex_);
  return ready_cv_.wait_for(
      lock, std::chrono::duration<double>(timeout),
      [this] { return ready_; });
}

}  // namespace robot_control
