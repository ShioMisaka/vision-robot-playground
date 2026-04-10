#include "robot_control_cpp/nodes/robot_controller_node.hpp"
#include "robot_control_cpp/kinematics/ik_solver.hpp"
#include "robot_control_cpp/nodes/topic_config.hpp"

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
#include <iostream>

namespace robot_control {

// ===== RosMotionBridge =====

RosMotionBridge::RosMotionBridge(rclcpp::Node::SharedPtr node,
                                 const TopicConfig& topics,
                                 std::shared_ptr<IKSolver> ik,
                                 const RobotProfile& profile,
                                 const GripperProfile& gripper)
    : node_(std::move(node)),
      topics_(topics),
      profile_(profile),
      gripper_(gripper),
      ik_(std::move(ik)),
      current_tcp_name_(profile.default_tcp),
      current_tcp_config_(profile.tcp_frames.at(profile.default_tcp)) {
  cmd_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
      topics_.joint_command, 10);

  tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*node_);
  static_tf_broadcaster_ =
      std::make_unique<tf2_ros::StaticTransformBroadcaster>(*node_);
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // 初始化当前关节状态
  current_arm_.resize(profile_.dof, 0.0);

  // 发布相机静态 TF（hand → camera_link → camera_color_optical_frame）
  publish_camera_tf();
}

void RosMotionBridge::publish_command(const std::vector<double>& arm,
                                      double finger) {
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = node_->now();

  for (size_t i = 0; i < arm.size(); ++i) {
    msg.name.push_back(profile_.joint_names[i]);
    msg.position.push_back(arm[i]);
  }
  // 夹爪关节（使用 profile 中 all_joint_names 里 dof 之后的名称）
  for (size_t i = profile_.dof; i < profile_.all_joint_names.size(); ++i) {
    msg.name.push_back(profile_.all_joint_names[i]);
    msg.position.push_back(finger);
  }

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

  int poll_count = 0;
  while (std::chrono::steady_clock::now() < deadline) {
    std::vector<double> current;
    double current_finger;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      current = current_arm_;
      current_finger = current_finger_;
    }

    bool arm_ok = true;
    double max_error = 0.0;
    int max_error_idx = -1;
    for (size_t i = 0; i < target_arm.size() && i < current.size(); ++i) {
      double error = std::abs(target_arm[i] - current[i]);
      if (error > max_error) {
        max_error = error;
        max_error_idx = static_cast<int>(i);
      }
      if (error > joint_tol) {
        arm_ok = false;
      }
    }
    bool finger_ok =
        !check_finger || std::abs(finger - current_finger) < finger_tol;

    poll_count++;
    if (poll_count % 50 == 0) {  // 每秒输出一次（50 * 0.02 = 1秒）
      std::cout << "  wait_for_motion poll " << poll_count
                << ": max_error=" << max_error << " @ joint " << max_error_idx
                << ", arm_ok=" << arm_ok << ", finger_ok=" << finger_ok << std::endl;
    }

    if (arm_ok && finger_ok) {
      std::cout << "  wait_for_motion: settling for " << settle_time << "s..." << std::endl;
      std::this_thread::sleep_for(
          std::chrono::duration<double>(settle_time));
      // settle 后重新检查，防止振荡导致误判到位
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current = current_arm_;
        current_finger = current_finger_;
      }
      arm_ok = true;
      max_error = 0.0;
      for (size_t i = 0; i < target_arm.size() && i < current.size(); ++i) {
        double error = std::abs(target_arm[i] - current[i]);
        if (error > max_error) {
          max_error = error;
          max_error_idx = static_cast<int>(i);
        }
        if (error > joint_tol) {
          arm_ok = false;
        }
      }
      finger_ok =
          !check_finger || std::abs(finger - current_finger) < finger_tol;
      std::cout << "  wait_for_motion after settle: max_error=" << max_error
                << ", arm_ok=" << arm_ok << std::endl;
      if (arm_ok && finger_ok) {
        std::cout << "  wait_for_motion: SUCCESS" << std::endl;
        return true;
      }
    }

    std::this_thread::sleep_for(
        std::chrono::duration<double>(poll_interval));
  }

  std::cout << "  wait_for_motion: TIMEOUT after " << poll_count << " polls" << std::endl;
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

    auto it = name_to_pos.empty() ? name_to_pos.end()
        : name_to_pos.find(profile_.all_joint_names[profile_.dof]);
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

void RosMotionBridge::publish_camera_tf() {
  const auto& ext = topics_.camera_extrinsics;

  // hand → camera_link
  geometry_msgs::msg::TransformStamped t_cam;
  t_cam.header.stamp = rclcpp::Time(0);
  t_cam.header.frame_id = profile_.hand_frame;
  t_cam.child_frame_id = "camera_link";
  t_cam.transform.translation.x = ext.xyz[0];
  t_cam.transform.translation.y = ext.xyz[1];
  t_cam.transform.translation.z = ext.xyz[2];

  tf2::Quaternion q_cam;
  q_cam.setRPY(ext.rpy[0], ext.rpy[1], ext.rpy[2]);
  t_cam.transform.rotation.x = q_cam.x();
  t_cam.transform.rotation.y = q_cam.y();
  t_cam.transform.rotation.z = q_cam.z();
  t_cam.transform.rotation.w = q_cam.w();

  // camera_link → camera_color_optical_frame（标准光学坐标系旋转）
  geometry_msgs::msg::TransformStamped t_opt;
  t_opt.header.stamp = rclcpp::Time(0);
  t_opt.header.frame_id = "camera_link";
  t_opt.child_frame_id = topics_.camera_frame;

  tf2::Quaternion q_opt;
  q_opt.setRPY(-1.57079632679, 0.0, -1.57079632679);
  t_opt.transform.rotation.x = q_opt.x();
  t_opt.transform.rotation.y = q_opt.y();
  t_opt.transform.rotation.z = q_opt.z();
  t_opt.transform.rotation.w = q_opt.w();

  static_tf_broadcaster_->sendTransform(t_cam);
  static_tf_broadcaster_->sendTransform(t_opt);
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
      shared_from_this(), topics_, ik_, profile_, gripper_);

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

  // Service servers
  srv_ik_ = create_service<robot_control_msgs::srv::SolveIK>(
      "~/solve_ik",
      [this](const std::shared_ptr<robot_control_msgs::srv::SolveIK::Request> req,
             std::shared_ptr<robot_control_msgs::srv::SolveIK::Response> res) {
        handle_solve_ik(req, res);
      });

  srv_move_joint_ = create_service<robot_control_msgs::srv::MoveJoint>(
      "~/move_joint",
      [this](const std::shared_ptr<robot_control_msgs::srv::MoveJoint::Request> req,
             std::shared_ptr<robot_control_msgs::srv::MoveJoint::Response> res) {
        handle_move_joint(req, res);
      });

  srv_move_pose_ = create_service<robot_control_msgs::srv::MovePose>(
      "~/move_pose",
      [this](const std::shared_ptr<robot_control_msgs::srv::MovePose::Request> req,
             std::shared_ptr<robot_control_msgs::srv::MovePose::Response> res) {
        handle_move_pose(req, res);
      });

  srv_move_linear_ = create_service<robot_control_msgs::srv::MoveLinear>(
      "~/move_linear",
      [this](const std::shared_ptr<robot_control_msgs::srv::MoveLinear::Request> req,
             std::shared_ptr<robot_control_msgs::srv::MoveLinear::Response> res) {
        handle_move_linear(req, res);
      });

  srv_gripper_ = create_service<robot_control_msgs::srv::ControlGripper>(
      "~/control_gripper",
      [this](const std::shared_ptr<robot_control_msgs::srv::ControlGripper::Request> req,
             std::shared_ptr<robot_control_msgs::srv::ControlGripper::Response> res) {
        handle_control_gripper(req, res);
      });

  srv_home_ = create_service<robot_control_msgs::srv::GoHome>(
      "~/go_home",
      [this](const std::shared_ptr<robot_control_msgs::srv::GoHome::Request> req,
             std::shared_ptr<robot_control_msgs::srv::GoHome::Response> res) {
        handle_go_home(req, res);
      });

  srv_speed_ = create_service<robot_control_msgs::srv::SetSpeed>(
      "~/set_speed",
      [this](const std::shared_ptr<robot_control_msgs::srv::SetSpeed::Request> req,
             std::shared_ptr<robot_control_msgs::srv::SetSpeed::Response> res) {
        handle_set_speed(req, res);
      });

  srv_state_ = create_service<robot_control_msgs::srv::GetRobotState>(
      "~/get_state",
      [this](const std::shared_ptr<robot_control_msgs::srv::GetRobotState::Request> req,
             std::shared_ptr<robot_control_msgs::srv::GetRobotState::Response> res) {
        handle_get_state(req, res);
      });

  RCLCPP_INFO(this->get_logger(), "RobotControllerNode started (with services)");
}

bool RobotControllerNode::wait_for_ready(double timeout) {
  std::unique_lock<std::mutex> lock(ready_mutex_);
  return ready_cv_.wait_for(
      lock, std::chrono::duration<double>(timeout),
      [this] { return ready_; });
}

// ===== Service Callbacks =====

void RobotControllerNode::handle_solve_ik(
    const std::shared_ptr<robot_control_msgs::srv::SolveIK::Request> req,
    std::shared_ptr<robot_control_msgs::srv::SolveIK::Response> res) {
  if (req->xyz.size() != 3) {
    res->success = false;
    res->message = "xyz must have exactly 3 elements";
    return;
  }

  std::array<double, 3> xyz{req->xyz[0], req->xyz[1], req->xyz[2]};
  std::optional<std::array<double, 3>> rpy;

  if (req->rpy.size() == 3) {
    rpy = std::array<double, 3>{req->rpy[0], req->rpy[1], req->rpy[2]};
  }

  auto result = ik_->solve(xyz, rpy);
  if (result) {
    res->success = true;
    res->joint_angles = *result;
    res->message = "IK solved";
  } else {
    res->success = false;
    res->message = "IK solution not found";
  }
}

void RobotControllerNode::handle_move_joint(
    const std::shared_ptr<robot_control_msgs::srv::MoveJoint::Request> req,
    std::shared_ptr<robot_control_msgs::srv::MoveJoint::Response> res) {
  try {
    controller_->moveJ(req->joint_angles, req->block);
    res->success = true;
    res->message = "moveJ completed";
  } catch (const std::exception& e) {
    res->success = false;
    res->message = std::string("moveJ failed: ") + e.what();
  }
}

void RobotControllerNode::handle_move_pose(
    const std::shared_ptr<robot_control_msgs::srv::MovePose::Request> req,
    std::shared_ptr<robot_control_msgs::srv::MovePose::Response> res) {
  if (req->xyz.size() != 3) {
    res->success = false;
    res->message = "xyz must have exactly 3 elements";
    return;
  }

  std::array<double, 3> xyz{req->xyz[0], req->xyz[1], req->xyz[2]};
  std::optional<std::array<double, 3>> rpy;

  if (req->rpy.size() == 3) {
    rpy = std::array<double, 3>{req->rpy[0], req->rpy[1], req->rpy[2]};
  }

  try {
    if (req->mode == 1) {
      controller_->moveL(xyz, rpy, req->finger);
    } else {
      controller_->moveJ(xyz, rpy, req->finger);
    }
    res->success = true;
    res->message = (req->mode == 1) ? "moveL completed" : "moveJ completed";
  } catch (const std::exception& e) {
    res->success = false;
    res->message = std::string("move_pose failed: ") + e.what();
  }
}

void RobotControllerNode::handle_move_linear(
    const std::shared_ptr<robot_control_msgs::srv::MoveLinear::Request> req,
    std::shared_ptr<robot_control_msgs::srv::MoveLinear::Response> res) {
  if (req->delta.size() != 3) {
    res->success = false;
    res->message = "delta must have exactly 3 elements";
    return;
  }

  std::array<double, 3> delta{req->delta[0], req->delta[1], req->delta[2]};

  try {
    controller_->move_linear(delta, req->frame, req->finger);
    res->success = true;
    res->message = "move_linear completed";
  } catch (const std::exception& e) {
    res->success = false;
    res->message = std::string("move_linear failed: ") + e.what();
  }
}

void RobotControllerNode::handle_control_gripper(
    const std::shared_ptr<robot_control_msgs::srv::ControlGripper::Request> req,
    std::shared_ptr<robot_control_msgs::srv::ControlGripper::Response> res) {
  try {
    switch (req->command) {
      case 0:
        controller_->open_gripper();
        res->message = "gripper opened";
        break;
      case 1:
        controller_->close_gripper();
        res->message = "gripper closed";
        break;
      case 2:
        controller_->set_gripper(req->width);
        res->message = "gripper set to " + std::to_string(req->width);
        break;
      default:
        res->success = false;
        res->message = "invalid command: use 0=open, 1=close, 2=set_width";
        return;
    }
    res->success = true;
  } catch (const std::exception& e) {
    res->success = false;
    res->message = std::string("gripper failed: ") + e.what();
  }
}

void RobotControllerNode::handle_go_home(
    const std::shared_ptr<robot_control_msgs::srv::GoHome::Request> /*req*/,
    std::shared_ptr<robot_control_msgs::srv::GoHome::Response> res) {
  try {
    controller_->go_home();
    res->success = true;
    res->message = "go_home completed";
  } catch (const std::exception& e) {
    res->success = false;
    res->message = std::string("go_home failed: ") + e.what();
  }
}

void RobotControllerNode::handle_set_speed(
    const std::shared_ptr<robot_control_msgs::srv::SetSpeed::Request> req,
    std::shared_ptr<robot_control_msgs::srv::SetSpeed::Response> res) {
  auto mode = (req->mode == 1) ? MotionMode::kMoveL : MotionMode::kMoveJ;
  controller_->set_speed(mode, req->percent);
  res->success = true;
  res->message = "speed set";
}

void RobotControllerNode::handle_get_state(
    const std::shared_ptr<robot_control_msgs::srv::GetRobotState::Request> /*req*/,
    std::shared_ptr<robot_control_msgs::srv::GetRobotState::Response> res) {
  try {
    res->joint_angles = controller_->get_joint_angles();
    auto pose = controller_->get_end_effector_pose();
    res->tcp_pose = {pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]};
    res->finger_width = controller_->get_finger_width();
    res->tcp_name = controller_->get_current_tcp();
    res->success = true;
    res->message = "ok";
  } catch (const std::exception& e) {
    res->success = false;
    res->message = std::string("get_state failed: ") + e.what();
  }
}

}  // namespace robot_control
