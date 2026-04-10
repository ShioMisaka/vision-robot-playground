#include "robot_control_cpp/nodes/robot_controller_node.hpp"
#include "robot_control_cpp/kinematics/ik_solver.hpp"
#include "robot_control_cpp/nodes/topic_config.hpp"
#include "robot_control_cpp/nodes/robot_state.hpp"
#include "robot_control_cpp/nodes/trajectory_executor.hpp"
#include "robot_control_cpp/motion/control_constants.hpp"

#include <rclcpp_action/rclcpp_action.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <arm_control_interfaces/action/move_j.hpp>
#include <arm_control_interfaces/action/move_l.hpp>
#include <arm_control_interfaces/srv/set_tcp.hpp>
#include <arm_control_interfaces/srv/set_speed_ratio.hpp>
#include <arm_control_interfaces/srv/robot_cmd.hpp>
#include <arm_control_interfaces/msg/robot_status.hpp>
#include <arm_control_interfaces/msg/jog_command.hpp>

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

  // Trajectory executor: wraps bridge_->publish_command for non-blocking execution
  trajectory_executor_ = std::make_unique<TrajectoryExecutor>(
      [this](const std::vector<double>& arm, double finger) {
        bridge_->publish_command(arm, finger);
      });

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

  // === Action servers ===
  movej_action_ = rclcpp_action::create_server<arm_control_interfaces::action::MoveJ>(
      this, "~/movej",
      std::bind(&RobotControllerNode::handle_movej_goal, this,
                std::placeholders::_1, std::placeholders::_2),
      std::bind(&RobotControllerNode::handle_movej_cancel, this,
                std::placeholders::_1),
      std::bind(&RobotControllerNode::handle_movej_accepted, this,
                std::placeholders::_1));

  movel_action_ = rclcpp_action::create_server<arm_control_interfaces::action::MoveL>(
      this, "~/movel",
      std::bind(&RobotControllerNode::handle_movel_goal, this,
                std::placeholders::_1, std::placeholders::_2),
      std::bind(&RobotControllerNode::handle_movel_cancel, this,
                std::placeholders::_1),
      std::bind(&RobotControllerNode::handle_movel_accepted, this,
                std::placeholders::_1));

  // === Pendant service servers ===
  pendant_set_tcp_srv_ = create_service<arm_control_interfaces::srv::SetTCP>(
      "~/set_tcp",
      [this](const std::shared_ptr<arm_control_interfaces::srv::SetTCP::Request> req,
             std::shared_ptr<arm_control_interfaces::srv::SetTCP::Response> res) {
        handle_pendant_set_tcp(req, res);
      });

  set_speed_ratio_srv_ = create_service<arm_control_interfaces::srv::SetSpeedRatio>(
      "~/set_speed_ratio",
      [this](const std::shared_ptr<arm_control_interfaces::srv::SetSpeedRatio::Request> req,
             std::shared_ptr<arm_control_interfaces::srv::SetSpeedRatio::Response> res) {
        handle_set_speed_ratio(req, res);
      });

  robot_cmd_srv_ = create_service<arm_control_interfaces::srv::RobotCmd>(
      "~/robot_cmd",
      [this](const std::shared_ptr<arm_control_interfaces::srv::RobotCmd::Request> req,
             std::shared_ptr<arm_control_interfaces::srv::RobotCmd::Response> res) {
        handle_robot_cmd(req, res);
      });

  // === Jog subscription ===
  rclcpp::SubscriptionOptions jog_opts;
  jog_opts.callback_group = state_cbg_;
  jog_sub_ = create_subscription<arm_control_interfaces::msg::JogCommand>(
      "~/jog_command", rclcpp::SensorDataQoS(),
      [this](const arm_control_interfaces::msg::JogCommand::SharedPtr msg) {
        handle_jog_command(msg);
      }, jog_opts);

  // === Jog watchdog (200ms) ===
  jog_watchdog_timer_ = create_wall_timer(
      std::chrono::milliseconds(200),
      [this]() { jog_watchdog_callback(); }, pub_cbg_);

  // === Status publisher (10Hz) ===
  status_pub_ = create_publisher<arm_control_interfaces::msg::RobotStatus>(
      "~/status", 10);
  status_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() { publish_status(); }, pub_cbg_);

  RCLCPP_INFO(this->get_logger(),
              "RobotControllerNode: pendant interface ready (actions, jog, status)");
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

// ===== MoveJ Action =====

rclcpp_action::GoalResponse RobotControllerNode::handle_movej_goal(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const arm_control_interfaces::action::MoveJ::Goal> goal) {
  if (state_machine_.state() != RobotState::kIdle) {
    RCLCPP_WARN(this->get_logger(), "MoveJ rejected: robot not IDLE (state=%s)",
                RobotStateMachine::state_name(state_machine_.state()));
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (goal->speed_ratio < 0.0 || goal->speed_ratio > 1.0) {
    RCLCPP_WARN(this->get_logger(), "MoveJ rejected: speed_ratio out of [0,1]");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RobotControllerNode::handle_movej_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<arm_control_interfaces::action::MoveJ>>) {
  if (state_machine_.state() == RobotState::kMoving) {
    state_machine_.transition_to(RobotState::kStopping);
    trajectory_executor_->cancel();
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  return rclcpp_action::CancelResponse::REJECT;
}

void RobotControllerNode::handle_movej_accepted(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<arm_control_interfaces::action::MoveJ>> goal_handle) {
  if (!state_machine_.transition_to(RobotState::kMoving)) {
    auto result = std::make_shared<arm_control_interfaces::action::MoveJ::Result>();
    result->success = false;
    result->message = "MoveJ rejected: cannot transition to MOVING state";
    goal_handle->abort(result);
    return;
  }

  movej_thread_ = std::thread([this, goal_handle]() {
    auto result = std::make_shared<arm_control_interfaces::action::MoveJ::Result>();
    auto feedback = std::make_shared<arm_control_interfaces::action::MoveJ::Feedback>();

    try {
      auto goal = goal_handle->get_goal();
      double effective_speed = goal->speed_ratio * global_speed_ratio_.load();

      std::vector<TrajectoryStep> steps;
      std::vector<double> target_angles;

      if (goal->mode == arm_control_interfaces::action::MoveJ::Goal::CARTESIAN) {
        std::array<double, 3> xyz = {goal->position.x, goal->position.y, goal->position.z};
        std::array<double, 3> rpy = {goal->orientation.x, goal->orientation.y, goal->orientation.z};

        auto tcp_cfg = profile_.tcp_frames.at(controller_->get_current_tcp());
        Eigen::Matrix4d T_tcp = Eigen::Matrix4d::Identity();
        Eigen::AngleAxisd roll_a(tcp_cfg.offset_rpy[0], Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitch_a(tcp_cfg.offset_rpy[1], Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yaw_a(tcp_cfg.offset_rpy[2], Eigen::Vector3d::UnitZ());
        T_tcp.block<3,3>(0,0) = (yaw_a * pitch_a * roll_a).toRotationMatrix();
        T_tcp(0,3) = tcp_cfg.offset_xyz[0]; T_tcp(1,3) = tcp_cfg.offset_xyz[1]; T_tcp(2,3) = tcp_cfg.offset_xyz[2];

        Eigen::Matrix4d T_target = Eigen::Matrix4d::Identity();
        Eigen::AngleAxisd e_roll(rpy[0], Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd e_pitch(rpy[1], Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd e_yaw(rpy[2], Eigen::Vector3d::UnitZ());
        T_target.block<3,3>(0,0) = (e_yaw * e_pitch * e_roll).toRotationMatrix();
        T_target(0,3) = xyz[0]; T_target(1,3) = xyz[1]; T_target(2,3) = xyz[2];

        Eigen::Matrix4d hand_target = T_target * T_tcp.inverse();
        Eigen::Vector3d h_xyz = hand_target.block<3,1>(0,3);
        Eigen::Vector3d h_rpy = hand_target.block<3,3>(0,0).eulerAngles(0,1,2);

        auto ik_result = ik_->solve(
            std::array<double,3>{h_xyz.x(), h_xyz.y(), h_xyz.z()},
            std::array<double,3>{h_rpy.x(), h_rpy.y(), h_rpy.z()});
        if (!ik_result) {
          result->success = false;
          result->message = "IK solver failed";
          goal_handle->abort(result);
          state_machine_.transition_to(RobotState::kFault);
          state_machine_.set_error(1, "MoveJ: IK failed");
          return;
        }
        target_angles = *ik_result;
      } else {
        target_angles = {goal->joint_angles.begin(), goal->joint_angles.end()};
      }

      auto current = bridge_->get_current_arm();
      double speed_factor = effective_speed * (controller_->get_speed(MotionMode::kMoveJ) / 100.0);
      std::vector<MotionLimits> configs(profile_.dof);
      for (int i = 0; i < profile_.dof; ++i) {
        configs[i] = {profile_.joint_limits.max_vel * speed_factor,
                      profile_.joint_limits.max_acc * speed_factor,
                      profile_.joint_limits.max_jerk * speed_factor};
      }

      auto trajectory = TrajectoryPlanner::plan_joint(
          current, target_angles, configs, ControlConstants::kTrajectoryDt);

      double dt = ControlConstants::kTrajectoryDt;
      for (size_t i = 0; i < trajectory.size(); ++i) {
        steps.push_back({trajectory[i], static_cast<double>(i) * dt});
      }

      double finger = (goal->finger_width >= 0) ? goal->finger_width
                      : bridge_->get_current_finger();

      trajectory_executor_->start(steps, finger);

      while (trajectory_executor_->is_active()) {
        double progress;
        std::vector<double> cur_angles;
        double time_rem;
        trajectory_executor_->get_progress(progress, cur_angles, time_rem);

        feedback->progress = progress;
        std::copy_n(cur_angles.begin(), 7, feedback->current_joint_angles.begin());
        feedback->estimated_time_remaining = time_rem;
        goal_handle->publish_feedback(feedback);

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }

      if (trajectory_executor_->wait_for_completion(2.0)) {
        bridge_->wait_for_motion(
            target_angles, finger,
            ControlConstants::kJointTolerance,
            ControlConstants::kFingerTolerance,
            ControlConstants::kMotionTimeout,
            ControlConstants::kPollInterval,
            ControlConstants::kSettleTime,
            true);

        if (state_machine_.state() == RobotState::kFault) {
          result->success = false;
          result->message = "MoveJ interrupted by EMERGENCY_STOP";
          goal_handle->abort(result);
          return;
        }

        auto final_angles = controller_->get_joint_angles();
        auto final_pose = controller_->get_end_effector_pose();
        result->success = true;
        result->message = "MoveJ completed";
        std::copy_n(final_angles.begin(), 7, result->final_joint_angles.begin());
        result->final_tcp_pose = {final_pose[0], final_pose[1], final_pose[2],
                                  final_pose[3], final_pose[4], final_pose[5]};
        goal_handle->succeed(result);
        state_machine_.transition_to(RobotState::kIdle);
      } else {
        if (state_machine_.state() == RobotState::kFault) {
          result->success = false;
          result->message = "MoveJ interrupted by EMERGENCY_STOP";
          goal_handle->abort(result);
          return;
        }
        result->success = false;
        result->message = "MoveJ cancelled";
        goal_handle->canceled(result);
        state_machine_.transition_to(RobotState::kIdle);
      }

    } catch (const std::exception& e) {
      result->success = false;
      result->message = std::string("MoveJ error: ") + e.what();
      goal_handle->abort(result);
      state_machine_.transition_to(RobotState::kFault);
      state_machine_.set_error(2, result->message);
    }
  });
}

// ===== MoveL Action =====

rclcpp_action::GoalResponse RobotControllerNode::handle_movel_goal(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const arm_control_interfaces::action::MoveL::Goal> goal) {
  if (state_machine_.state() != RobotState::kIdle) {
    RCLCPP_WARN(this->get_logger(), "MoveL rejected: robot not IDLE (state=%s)",
                RobotStateMachine::state_name(state_machine_.state()));
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (goal->speed_ratio < 0.0 || goal->speed_ratio > 1.0) {
    RCLCPP_WARN(this->get_logger(), "MoveL rejected: speed_ratio out of [0,1]");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RobotControllerNode::handle_movel_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<arm_control_interfaces::action::MoveL>>) {
  if (state_machine_.state() == RobotState::kMoving) {
    state_machine_.transition_to(RobotState::kStopping);
    trajectory_executor_->cancel();
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  return rclcpp_action::CancelResponse::REJECT;
}

void RobotControllerNode::handle_movel_accepted(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<arm_control_interfaces::action::MoveL>> goal_handle) {
  if (!state_machine_.transition_to(RobotState::kMoving)) {
    auto result = std::make_shared<arm_control_interfaces::action::MoveL::Result>();
    result->success = false;
    result->message = "MoveL rejected: cannot transition to MOVING state";
    goal_handle->abort(result);
    return;
  }

  movel_thread_ = std::thread([this, goal_handle]() {
    auto result = std::make_shared<arm_control_interfaces::action::MoveL::Result>();
    auto feedback = std::make_shared<arm_control_interfaces::action::MoveL::Feedback>();

    try {
      auto goal = goal_handle->get_goal();
      double effective_speed = goal->speed_ratio * global_speed_ratio_.load();

      std::array<double, 3> xyz = {goal->position.x, goal->position.y, goal->position.z};
      std::array<double, 3> rpy = {goal->orientation.x, goal->orientation.y, goal->orientation.z};

      auto current_pose = controller_->get_end_effector_pose();
      Eigen::Vector3d start_pos(current_pose[0], current_pose[1], current_pose[2]);
      Eigen::Vector3d end_pos(xyz[0], xyz[1], xyz[2]);
      double total_dist = (end_pos - start_pos).norm();

      if (total_dist < 1e-6) {
        result->success = true;
        result->message = "MoveL: already at target";
        goal_handle->succeed(result);
        state_machine_.transition_to(RobotState::kIdle);
        return;
      }

      double speed_factor = effective_speed * (controller_->get_speed(MotionMode::kMoveL) / 100.0);
      MotionLimits cart_cfg{
          profile_.cartesian_limits.max_vel * speed_factor,
          profile_.cartesian_limits.max_acc * speed_factor,
          profile_.cartesian_limits.max_jerk * speed_factor};

      auto cart_traj = SCurvePlanner::plan(
          0.0, total_dist, cart_cfg, ControlConstants::kTrajectoryDt);

      Eigen::Vector3d start_rpy(current_pose[3], current_pose[4], current_pose[5]);
      Eigen::Quaterniond start_quat =
          (Eigen::AngleAxisd(start_rpy.x(), Eigen::Vector3d::UnitX()) *
           Eigen::AngleAxisd(start_rpy.y(), Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(start_rpy.z(), Eigen::Vector3d::UnitZ()));
      Eigen::Quaterniond end_quat =
          (Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX()) *
           Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()));

      auto tcp_cfg = profile_.tcp_frames.at(controller_->get_current_tcp());
      Eigen::Matrix4d T_tcp = Eigen::Matrix4d::Identity();
      Eigen::AngleAxisd e_roll(tcp_cfg.offset_rpy[0], Eigen::Vector3d::UnitX());
      Eigen::AngleAxisd e_pitch(tcp_cfg.offset_rpy[1], Eigen::Vector3d::UnitY());
      Eigen::AngleAxisd e_yaw(tcp_cfg.offset_rpy[2], Eigen::Vector3d::UnitZ());
      T_tcp.block<3,3>(0,0) = (e_yaw * e_pitch * e_roll).toRotationMatrix();
      T_tcp(0,3) = tcp_cfg.offset_xyz[0]; T_tcp(1,3) = tcp_cfg.offset_xyz[1]; T_tcp(2,3) = tcp_cfg.offset_xyz[2];

      std::vector<TrajectoryStep> steps;
      double finger = (goal->finger_width >= 0) ? goal->finger_width : bridge_->get_current_finger();

      for (const auto& pt : cart_traj) {
        double alpha = (total_dist > 1e-12) ? pt.pos / total_dist : 1.0;
        alpha = std::clamp(alpha, 0.0, 1.0);

        Eigen::Vector3d interp_pos = start_pos + alpha * (end_pos - start_pos);
        Eigen::Quaterniond interp_quat = start_quat.slerp(alpha, end_quat);

        Eigen::Matrix4d T_target = Eigen::Matrix4d::Identity();
        T_target.block<3,3>(0,0) = interp_quat.toRotationMatrix();
        T_target(0,3) = interp_pos.x(); T_target(1,3) = interp_pos.y(); T_target(2,3) = interp_pos.z();

        Eigen::Matrix4d hand_target = T_target * T_tcp.inverse();
        Eigen::Vector3d h_xyz = hand_target.block<3,1>(0,3);
        Eigen::Vector3d h_rpy = hand_target.block<3,3>(0,0).eulerAngles(0,1,2);

        auto ik_result = ik_->solve(
            std::array<double,3>{h_xyz.x(), h_xyz.y(), h_xyz.z()},
            std::array<double,3>{h_rpy.x(), h_rpy.y(), h_rpy.z()});
        if (!ik_result) {
          throw std::runtime_error("MoveL: IK failed at trajectory point");
        }
        steps.push_back({*ik_result, pt.t});
      }

      trajectory_executor_->start(steps, finger);

      auto next_fb_time = std::chrono::steady_clock::now();
      while (trajectory_executor_->is_active()) {
        double progress;
        std::vector<double> cur_angles;
        double time_rem;
        trajectory_executor_->get_progress(progress, cur_angles, time_rem);

        feedback->progress = progress;
        std::copy_n(cur_angles.begin(), 7, feedback->current_joint_angles.begin());
        feedback->estimated_time_remaining = time_rem;
        goal_handle->publish_feedback(feedback);

        next_fb_time += std::chrono::milliseconds(20);
        std::this_thread::sleep_until(next_fb_time);
      }

      if (trajectory_executor_->wait_for_completion(2.0)) {
        if (!steps.empty()) {
          bridge_->wait_for_motion(
              steps.back().joint_positions, finger,
              ControlConstants::kJointTolerance,
              ControlConstants::kFingerTolerance,
              ControlConstants::kMotionTimeout,
              ControlConstants::kPollInterval,
              ControlConstants::kSettleTime,
              true);
        }

        if (state_machine_.state() == RobotState::kFault) {
          result->success = false;
          result->message = "MoveL interrupted by EMERGENCY_STOP";
          goal_handle->abort(result);
          return;
        }

        auto final_angles = controller_->get_joint_angles();
        auto final_pose = controller_->get_end_effector_pose();
        result->success = true;
        result->message = "MoveL completed";
        std::copy_n(final_angles.begin(), 7, result->final_joint_angles.begin());
        result->final_tcp_pose = {final_pose[0], final_pose[1], final_pose[2],
                                  final_pose[3], final_pose[4], final_pose[5]};
        goal_handle->succeed(result);
        state_machine_.transition_to(RobotState::kIdle);
      } else {
        if (state_machine_.state() == RobotState::kFault) {
          result->success = false;
          result->message = "MoveL interrupted by EMERGENCY_STOP";
          goal_handle->abort(result);
          return;
        }
        result->success = false;
        result->message = "MoveL cancelled";
        goal_handle->canceled(result);
        state_machine_.transition_to(RobotState::kIdle);
      }

    } catch (const std::exception& e) {
      result->success = false;
      result->message = std::string("MoveL error: ") + e.what();
      goal_handle->abort(result);
      state_machine_.transition_to(RobotState::kFault);
      state_machine_.set_error(3, result->message);
    }
  });
}

// ===== Pendant Service Callbacks =====

void RobotControllerNode::handle_pendant_set_tcp(
    const std::shared_ptr<arm_control_interfaces::srv::SetTCP::Request> req,
    std::shared_ptr<arm_control_interfaces::srv::SetTCP::Response> res) {
  try {
    controller_->set_tcp(req->name);
    res->success = true;
    res->message = "TCP set to " + req->name;
  } catch (const std::exception& e) {
    res->success = false;
    res->message = std::string("SetTCP failed: ") + e.what();
  }
}

void RobotControllerNode::handle_set_speed_ratio(
    const std::shared_ptr<arm_control_interfaces::srv::SetSpeedRatio::Request> req,
    std::shared_ptr<arm_control_interfaces::srv::SetSpeedRatio::Response> res) {
  if (req->ratio < 0.0 || req->ratio > 1.0) {
    res->success = false;
    res->message = "ratio must be in [0.0, 1.0]";
    return;
  }
  global_speed_ratio_ = req->ratio;
  res->success = true;
  res->message = "global speed ratio set to " + std::to_string(req->ratio);
}

void RobotControllerNode::handle_robot_cmd(
    const std::shared_ptr<arm_control_interfaces::srv::RobotCmd::Request> req,
    std::shared_ptr<arm_control_interfaces::srv::RobotCmd::Response> res) {
  switch (req->command) {
    case arm_control_interfaces::srv::RobotCmd::Request::STOP:
      if (state_machine_.state() == RobotState::kMoving ||
          state_machine_.state() == RobotState::kTeaching) {
        trajectory_executor_->cancel();
        bridge_->publish_command(bridge_->get_current_arm(),
                                bridge_->get_current_finger());
        state_machine_.transition_to(RobotState::kStopping);
        // Do NOT transition to kIdle here — the action thread handles
        // kStopping -> kIdle after the cancelled trajectory finishes.
        // Transitioning to kIdle immediately would allow new goals while
        // the old action thread is still running, causing std::terminate.
        res->success = true;
        res->message = "STOP executed";
      } else if (state_machine_.state() == RobotState::kStopping) {
        bridge_->publish_command(bridge_->get_current_arm(),
                                bridge_->get_current_finger());
        res->success = true;
        res->message = "STOP: already stopping";
      } else {
        res->success = true;
        res->message = "STOP: no motion to stop";
      }
      break;

    case arm_control_interfaces::srv::RobotCmd::Request::EMERGENCY_STOP:
      emergency_stop();
      res->success = true;
      res->message = "EMERGENCY_STOP executed";
      break;

    case arm_control_interfaces::srv::RobotCmd::Request::CLEAR_FAULT:
      if (state_machine_.state() == RobotState::kFault) {
        state_machine_.clear_error();
        state_machine_.transition_to(RobotState::kIdle);
        res->success = true;
        res->message = "FAULT cleared";
      } else {
        res->success = false;
        res->message = "CLEAR_FAULT: robot not in FAULT state";
      }
      break;

    default:
      res->success = false;
      res->message = "Unknown command: " + std::to_string(req->command);
      break;
  }
}

// ===== Jog + Watchdog =====

void RobotControllerNode::handle_jog_command(
    const arm_control_interfaces::msg::JogCommand::SharedPtr msg) {
  auto state = state_machine_.state();
  if (state != RobotState::kIdle && state != RobotState::kTeaching) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Jog ignored: robot state=%s",
                         RobotStateMachine::state_name(state));
    return;
  }

  if (state == RobotState::kIdle) {
    state_machine_.transition_to(RobotState::kTeaching);
  }

  last_jog_time_ = this->now();

  double dt = 0.02;
  auto current = bridge_->get_current_arm();
  auto pose = controller_->get_end_effector_pose();

  std::array<double, 3> delta = {
      msg->velocity[0] * dt,
      msg->velocity[1] * dt,
      msg->velocity[2] * dt
  };
  std::array<double, 3> delta_rpy = {
      msg->velocity[3] * dt,
      msg->velocity[4] * dt,
      msg->velocity[5] * dt
  };

  Eigen::Vector3d pos(pose[0], pose[1], pose[2]);
  Eigen::Vector3d rpy(pose[3], pose[4], pose[5]);
  Eigen::Matrix3d rot = (Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX()) *
                         Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()))
                            .toRotationMatrix();
  Eigen::Vector3d new_pos = pos + rot * Eigen::Vector3d(delta[0], delta[1], delta[2]);

  Eigen::AngleAxisd d_roll(delta_rpy[0], Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd d_pitch(delta_rpy[1], Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd d_yaw(delta_rpy[2], Eigen::Vector3d::UnitZ());
  Eigen::Matrix3d d_rot = (d_yaw * d_pitch * d_roll).toRotationMatrix();
  Eigen::Matrix3d new_rot = d_rot * rot;

  auto tcp_cfg = profile_.tcp_frames.at(controller_->get_current_tcp());
  Eigen::Matrix4d T_tcp = Eigen::Matrix4d::Identity();
  Eigen::AngleAxisd e_roll(tcp_cfg.offset_rpy[0], Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd e_pitch(tcp_cfg.offset_rpy[1], Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd e_yaw(tcp_cfg.offset_rpy[2], Eigen::Vector3d::UnitZ());
  T_tcp.block<3,3>(0,0) = (e_yaw * e_pitch * e_roll).toRotationMatrix();
  T_tcp(0,3) = tcp_cfg.offset_xyz[0]; T_tcp(1,3) = tcp_cfg.offset_xyz[1]; T_tcp(2,3) = tcp_cfg.offset_xyz[2];

  Eigen::Matrix4d T_target = Eigen::Matrix4d::Identity();
  T_target.block<3,3>(0,0) = new_rot;
  T_target(0,3) = new_pos.x(); T_target(1,3) = new_pos.y(); T_target(2,3) = new_pos.z();

  Eigen::Matrix4d hand_target = T_target * T_tcp.inverse();
  Eigen::Vector3d h_xyz = hand_target.block<3,1>(0,3);
  Eigen::Vector3d h_rpy = hand_target.block<3,3>(0,0).eulerAngles(0,1,2);

  auto ik_result = ik_->solve(
      std::array<double,3>{h_xyz.x(), h_xyz.y(), h_xyz.z()},
      std::array<double,3>{h_rpy.x(), h_rpy.y(), h_rpy.z()});

  if (ik_result) {
    bridge_->publish_command(*ik_result, bridge_->get_current_finger());
  } else {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Jog: IK failed, skipping step");
  }
}

void RobotControllerNode::jog_watchdog_callback() {
  if (state_machine_.state() != RobotState::kTeaching) return;

  auto elapsed = (this->now() - last_jog_time_).seconds();
  if (elapsed > 0.2) {
    RCLCPP_WARN(this->get_logger(), "Jog watchdog: no command for %.2fs, stopping", elapsed);
    bridge_->publish_command(bridge_->get_current_arm(), bridge_->get_current_finger());
    state_machine_.transition_to(RobotState::kIdle);
  }
}

// ===== Emergency Stop =====

void RobotControllerNode::emergency_stop() {
  trajectory_executor_->cancel();
  bridge_->publish_command(bridge_->get_current_arm(), bridge_->get_current_finger());
  state_machine_.force_state(RobotState::kFault);
  state_machine_.set_error(100, "EMERGENCY_STOP activated");
  RCLCPP_ERROR(this->get_logger(), "EMERGENCY_STOP activated");
}

// ===== Status Publisher =====

void RobotControllerNode::publish_status() {
  auto msg = std::make_unique<arm_control_interfaces::msg::RobotStatus>();
  msg->state = static_cast<uint8_t>(state_machine_.state());
  msg->speed_ratio = global_speed_ratio_;
  msg->error_code = state_machine_.error_code();
  msg->error_message = state_machine_.error_message();

  auto angles = controller_->get_joint_angles();
  std::copy_n(angles.begin(), 7, msg->joint_angles.begin());

  auto pose = controller_->get_end_effector_pose();
  msg->tcp_pose = {pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]};

  msg->finger_width = controller_->get_finger_width();
  msg->tcp_name = controller_->get_current_tcp();
  {
    std::lock_guard<std::mutex> lock(ready_mutex_);
    msg->is_connected = ready_;
  }

  status_pub_->publish(std::move(msg));
}

// ===== Destructor =====

RobotControllerNode::~RobotControllerNode() {
  shutdown_ = true;
  if (trajectory_executor_) {
    trajectory_executor_->cancel();
  }
  if (movej_thread_.joinable()) movej_thread_.join();
  if (movel_thread_.joinable()) movel_thread_.join();
}

}  // namespace robot_control
