#include "robot_api_python/service_robot_controller.hpp"

#include <chrono>
#include <stdexcept>

using robot_control::MotionMode;

namespace robot_api_python {

ServiceRobotController::ServiceRobotController(
    rclcpp::Node::SharedPtr node, const std::string& service_prefix)
    : node_(node), prefix_("/" + service_prefix) {
  cli_move_joint_ = node_->create_client<robot_msgs::srv::MoveJoint>(
      prefix_ + "/move_joint");
  cli_move_pose_ = node_->create_client<robot_msgs::srv::MovePose>(
      prefix_ + "/move_pose");
  cli_move_linear_ = node_->create_client<robot_msgs::srv::MoveLinear>(
      prefix_ + "/move_linear");
  cli_gripper_ = node_->create_client<robot_msgs::srv::ControlGripper>(
      prefix_ + "/control_gripper");
  cli_home_ = node_->create_client<robot_msgs::srv::GoHome>(
      prefix_ + "/go_home");
  cli_speed_ = node_->create_client<robot_msgs::srv::SetSpeed>(
      prefix_ + "/set_speed");
  cli_state_ = node_->create_client<robot_msgs::srv::GetRobotState>(
      prefix_ + "/get_state");
  cli_set_tcp_ = node_->create_client<robot_msgs::srv::SetTCP>(
      prefix_ + "/set_tcp");

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void ServiceRobotController::refresh_state_cache() {
  auto req = std::make_shared<robot_msgs::srv::GetRobotState::Request>();
  auto future = cli_state_->async_send_request(req);
  if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
    throw std::runtime_error("get_state service call timeout");
  }
  auto res = future.get();
  if (!res->success) {
    throw std::runtime_error("get_state failed: " + res->message);
  }
  std::lock_guard<std::mutex> lock(cache_mutex_);
  cached_joints_ = res->joint_angles;
  for (int i = 0; i < 6 && i < static_cast<int>(res->tcp_pose.size()); ++i) {
    cached_pose_[i] = res->tcp_pose[i];
  }
  cached_finger_ = res->finger_width;
  cached_tcp_ = res->tcp_name;
}

void ServiceRobotController::call_move_joint(
    const std::vector<double>& angles, bool block) {
  auto req = std::make_shared<robot_msgs::srv::MoveJoint::Request>();
  req->joint_angles = angles;
  req->block = block;
  auto future = cli_move_joint_->async_send_request(req);
  if (block) {
    if (future.wait_for(std::chrono::seconds(15)) !=
        std::future_status::ready) {
      throw std::runtime_error("move_joint service call timeout");
    }
    auto res = future.get();
    if (!res->success) {
      throw std::runtime_error("move_joint failed: " + res->message);
    }
  }
}

void ServiceRobotController::call_move_pose(
    const std::array<double, 3>& xyz,
    const std::optional<std::array<double, 3>>& rpy,
    uint8_t mode, double finger) {
  auto req = std::make_shared<robot_msgs::srv::MovePose::Request>();
  req->xyz = {xyz[0], xyz[1], xyz[2]};
  if (rpy) {
    req->rpy = {(*rpy)[0], (*rpy)[1], (*rpy)[2]};
  }
  req->mode = mode;
  req->finger = finger;
  auto future = cli_move_pose_->async_send_request(req);
  if (future.wait_for(std::chrono::seconds(15)) != std::future_status::ready) {
    throw std::runtime_error("move_pose service call timeout");
  }
  auto res = future.get();
  if (!res->success) {
    throw std::runtime_error(
        std::string(mode == 1 ? "moveL" : "moveJ") + " failed: " + res->message);
  }
}

void ServiceRobotController::call_gripper(uint8_t command, double width) {
  auto req = std::make_shared<robot_msgs::srv::ControlGripper::Request>();
  req->command = command;
  req->width = width;
  auto future = cli_gripper_->async_send_request(req);
  if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
    throw std::runtime_error("control_gripper service call timeout");
  }
  auto res = future.get();
  if (!res->success) {
    throw std::runtime_error("gripper failed: " + res->message);
  }
}

// ===== IRobotController implementations =====

void ServiceRobotController::set_arm(const std::vector<double>& angles,
                                      bool block) {
  call_move_joint(angles, block);
}

void ServiceRobotController::set_gripper(double width, bool block) {
  call_gripper(2, width);
}

void ServiceRobotController::open_gripper(bool block) {
  call_gripper(0);
}

void ServiceRobotController::close_gripper(bool block) {
  call_gripper(1);
}

void ServiceRobotController::move_to_pose(
    const std::array<double, 3>& xyz,
    const std::optional<std::array<double, 3>>& rpy,
    double finger, int steps, double step_time, bool block) {
  // steps/step_time not supported via service — delegate to moveJ
  call_move_pose(xyz, rpy, 0, finger);
}

void ServiceRobotController::move_linear(
    const std::array<double, 3>& delta, const std::string& frame,
    double finger, bool block) {
  auto req = std::make_shared<robot_msgs::srv::MoveLinear::Request>();
  req->delta = {delta[0], delta[1], delta[2]};
  req->frame = frame;
  req->finger = finger;
  auto future = cli_move_linear_->async_send_request(req);
  if (block) {
    if (future.wait_for(std::chrono::seconds(15)) !=
        std::future_status::ready) {
      throw std::runtime_error("move_linear service call timeout");
    }
    auto res = future.get();
    if (!res->success) {
      throw std::runtime_error("move_linear failed: " + res->message);
    }
  }
}

void ServiceRobotController::rotate_joint(int index, double delta_angle,
                                            bool block) {
  // Not directly available via service — get current angles, modify, set
  refresh_state_cache();
  std::vector<double> angles;
  {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    angles = cached_joints_;
  }
  if (index < 0 || index >= static_cast<int>(angles.size())) {
    throw std::runtime_error("rotate_joint: index out of range");
  }
  angles[index] += delta_angle;
  call_move_joint(angles, block);
}

void ServiceRobotController::go_home(bool block) {
  auto req = std::make_shared<robot_msgs::srv::GoHome::Request>();
  auto future = cli_home_->async_send_request(req);
  if (future.wait_for(std::chrono::seconds(15)) != std::future_status::ready) {
    throw std::runtime_error("go_home service call timeout");
  }
  auto res = future.get();
  if (!res->success) {
    throw std::runtime_error("go_home failed: " + res->message);
  }
}

std::vector<double> ServiceRobotController::get_joint_angles() const {
  const_cast<ServiceRobotController*>(this)->refresh_state_cache();
  std::lock_guard<std::mutex> lock(cache_mutex_);
  return cached_joints_;
}

std::array<double, 6> ServiceRobotController::get_end_effector_pose() const {
  const_cast<ServiceRobotController*>(this)->refresh_state_cache();
  std::lock_guard<std::mutex> lock(cache_mutex_);
  return cached_pose_;
}

double ServiceRobotController::get_finger_width() const {
  const_cast<ServiceRobotController*>(this)->refresh_state_cache();
  std::lock_guard<std::mutex> lock(cache_mutex_);
  return cached_finger_;
}

void ServiceRobotController::set_tcp(const std::string& name) {
  auto req = std::make_shared<robot_msgs::srv::SetTCP::Request>();
  req->name = name;
  auto future = cli_set_tcp_->async_send_request(req);
  if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
    throw std::runtime_error("set_tcp service call timeout");
  }
  auto res = future.get();
  if (!res->success) {
    throw std::runtime_error("set_tcp failed: " + res->message);
  }
}

std::string ServiceRobotController::get_current_tcp() const {
  const_cast<ServiceRobotController*>(this)->refresh_state_cache();
  std::lock_guard<std::mutex> lock(cache_mutex_);
  return cached_tcp_;
}

std::optional<std::array<double, 6>>
ServiceRobotController::lookup_transform(
    const std::string& target_frame,
    const std::string& source_frame,
    double timeout) {
  try {
    auto transform = tf_buffer_->lookupTransform(
        target_frame, source_frame,
        tf2::TimePointZero);
    auto& t = transform.transform.translation;
    auto& r = transform.transform.rotation;
    // Quaternion to Euler (roll, pitch, yaw)
    double sinr_cosp = 2.0 * (r.w * r.x + r.y * r.z);
    double cosr_cosp = 1.0 - 2.0 * (r.x * r.x + r.y * r.y);
    double sinp = 2.0 * (r.w * r.y - r.z * r.x);
    double siny_cosp = 2.0 * (r.w * r.z + r.x * r.y);
    double cosy_cosp = 1.0 - 2.0 * (r.y * r.y + r.z * r.z);
    std::array<double, 6> result = {{
        t.x, t.y, t.z,
        std::atan2(sinr_cosp, cosr_cosp),
        std::abs(sinp) >= 1.0 ? std::copysign(M_PI / 2, sinp)
                               : std::asin(sinp),
        std::atan2(siny_cosp, cosy_cosp)
    }};
    return result;
  } catch (const tf2::TransformException& ex) {
    return std::nullopt;
  }
}

void ServiceRobotController::moveJ(const std::vector<double>& target_angles,
                                    bool block) {
  call_move_joint(target_angles, block);
}

void ServiceRobotController::moveJ(
    const std::array<double, 3>& xyz,
    const std::optional<std::array<double, 3>>& rpy,
    double finger, bool block,
    robot_control::MotionSource source) {
  call_move_pose(xyz, rpy, 0, finger);
}

void ServiceRobotController::moveL(
    const std::array<double, 3>& xyz,
    const std::optional<std::array<double, 3>>& rpy,
    double finger, bool block,
    robot_control::MotionSource source) {
  call_move_pose(xyz, rpy, 1, finger);
}

void ServiceRobotController::set_speed(MotionMode mode, double percent) {
  auto req = std::make_shared<robot_msgs::srv::SetSpeed::Request>();
  req->mode = (mode == MotionMode::kMoveL) ? 1 : 0;
  req->percent = percent;
  auto future = cli_speed_->async_send_request(req);
  future.wait_for(std::chrono::seconds(2));
  std::lock_guard<std::mutex> lock(cache_mutex_);
  if (mode == MotionMode::kMoveL) {
    cached_speed_l_ = percent;
  } else {
    cached_speed_j_ = percent;
  }
}

double ServiceRobotController::get_speed(MotionMode mode) const {
  std::lock_guard<std::mutex> lock(cache_mutex_);
  return (mode == MotionMode::kMoveL) ? cached_speed_l_ : cached_speed_j_;
}

}  // namespace robot_api_python
