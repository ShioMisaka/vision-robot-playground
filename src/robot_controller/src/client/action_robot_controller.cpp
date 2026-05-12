#include "robot_controller/client/action_robot_controller.hpp"

#include <chrono>
#include <stdexcept>
#include <thread>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <robot_logger/logger.hpp>

using namespace std::chrono_literals;

namespace robot_control {

// ======================================================================
// 构造/析构
// ======================================================================

ActionRobotController::ActionRobotController(rclcpp::Node::SharedPtr node,
                                             const std::string& prefix)
    : node_(node), prefix_("/" + prefix) {
  // ----- Action Clients -----
  cli_movej_ = rclcpp_action::create_client<robot_msgs::action::MoveJ>(
      node_, prefix_ + "/move_j");
  cli_movel_ = rclcpp_action::create_client<robot_msgs::action::MoveL>(
      node_, prefix_ + "/move_l");
  cli_gohome_ = rclcpp_action::create_client<robot_msgs::action::GoHome>(
      node_, prefix_ + "/go_home");

  // ----- Service Clients -----
  cli_gripper_ = node_->create_client<robot_msgs::srv::ControlGripper>(
      prefix_ + "/control_gripper");
  cli_speed_ = node_->create_client<robot_msgs::srv::SetSpeed>(
      prefix_ + "/set_speed");
  cli_state_ = node_->create_client<robot_msgs::srv::GetRobotState>(
      prefix_ + "/get_state");
  cli_set_tcp_ = node_->create_client<robot_msgs::srv::SetTCP>(
      prefix_ + "/set_tcp");
  cli_acquire_ = node_->create_client<robot_msgs::srv::AcquireControl>(
      prefix_ + "/acquire_control");
  cli_release_ = node_->create_client<robot_msgs::srv::ReleaseControl>(
      prefix_ + "/release_control");
  cli_renew_ = node_->create_client<robot_msgs::srv::RenewLease>(
      prefix_ + "/renew_lease");

  // ----- TF -----
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  LOG_INFO("ActionRobotController created (prefix: {})", prefix_);
}

ActionRobotController::~ActionRobotController() {
  // 停止续约定时器
  if (renewal_timer_) {
    renewal_timer_->cancel();
    renewal_timer_.reset();
  }
}

// ======================================================================
// Action/Service 就绪检查
// ======================================================================

bool ActionRobotController::wait_for_actions(double timeout) {
  auto deadline = std::chrono::steady_clock::now() +
                  std::chrono::duration<double>(timeout);
  while (std::chrono::steady_clock::now() < deadline) {
    if (cli_movej_->action_server_is_ready() &&
        cli_movel_->action_server_is_ready() &&
        cli_gohome_->action_server_is_ready()) {
      LOG_INFO("All action servers ready");
      return true;
    }
    std::this_thread::sleep_for(100ms);
    if (!rclcpp::ok()) return false;
  }
  LOG_ERROR("Timeout waiting for action servers ({:.1f}s)", timeout);
  return false;
}

bool ActionRobotController::wait_for_services(double timeout) {
  auto deadline = std::chrono::steady_clock::now() +
                  std::chrono::duration<double>(timeout);
  while (std::chrono::steady_clock::now() < deadline) {
    if (cli_gripper_->service_is_ready() &&
        cli_state_->service_is_ready() &&
        cli_speed_->service_is_ready()) {
      LOG_INFO("All critical services ready");
      return true;
    }
    std::this_thread::sleep_for(100ms);
    if (!rclcpp::ok()) return false;
  }
  LOG_ERROR("Timeout waiting for services ({:.1f}s)", timeout);
  return false;
}

// ======================================================================
// 状态缓存
// ======================================================================

void ActionRobotController::refresh_state_cache() const {
  auto req = std::make_shared<robot_msgs::srv::GetRobotState::Request>();
  auto future = cli_state_->async_send_request(req);
  if (future.wait_for(2s) != std::future_status::ready) {
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

// ======================================================================
// Gripper 辅助
// ======================================================================

void ActionRobotController::call_gripper(uint8_t command, double width) {
  auto req = std::make_shared<robot_msgs::srv::ControlGripper::Request>();
  req->command = command;
  req->width = width;
  auto future = cli_gripper_->async_send_request(req);
  if (future.wait_for(5s) != std::future_status::ready) {
    throw std::runtime_error("control_gripper service call timeout");
  }
  auto res = future.get();
  if (!res->success) {
    throw std::runtime_error("gripper failed: " + res->message);
  }
}

// ======================================================================
// Action 发送辅助
// ======================================================================

void ActionRobotController::send_movej_goal(
    robot_msgs::action::MoveJ::Goal goal, bool block) {
  // 填充 session_id
  {
    std::lock_guard<std::mutex> lock(session_mutex_);
    goal.session_id = session_id_;
  }

  if (!cli_movej_->action_server_is_ready()) {
    throw std::runtime_error("MoveJ action server not ready");
  }

  auto send_goal_options =
      rclcpp_action::Client<robot_msgs::action::MoveJ>::SendGoalOptions();

  // 注册 feedback 回调
  send_goal_options.feedback_callback =
      [this](
          std::shared_ptr<rclcpp_action::ClientGoalHandle<
              robot_msgs::action::MoveJ>> /*goal_handle*/,
          const std::shared_ptr<
              const robot_msgs::action::MoveJ::Feedback> feedback) {
        std::lock_guard<std::mutex> lock(progress_mutex_);
        if (progress_callback_) {
          progress_callback_(feedback->progress);
        }
      };

  auto goal_handle_future = cli_movej_->async_send_goal(goal, send_goal_options);
  if (goal_handle_future.wait_for(5s) != std::future_status::ready) {
    throw std::runtime_error("MoveJ send_goal timeout");
  }

  auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    throw std::runtime_error("MoveJ goal was rejected by server");
  }

  if (!block) return;

  // 同步等待结果
  auto result_future = cli_movej_->async_get_result(goal_handle);
  if (result_future.wait_for(30s) != std::future_status::ready) {
    throw std::runtime_error("MoveJ action result timeout");
  }

  auto result = result_future.get();
  if (!result.result->success) {
    throw std::runtime_error("MoveJ failed: " + result.result->message);
  }
}

void ActionRobotController::send_movel_goal(
    robot_msgs::action::MoveL::Goal goal, bool block) {
  {
    std::lock_guard<std::mutex> lock(session_mutex_);
    goal.session_id = session_id_;
  }

  if (!cli_movel_->action_server_is_ready()) {
    throw std::runtime_error("MoveL action server not ready");
  }

  auto send_goal_options =
      rclcpp_action::Client<robot_msgs::action::MoveL>::SendGoalOptions();

  send_goal_options.feedback_callback =
      [this](
          std::shared_ptr<rclcpp_action::ClientGoalHandle<
              robot_msgs::action::MoveL>> /*goal_handle*/,
          const std::shared_ptr<
              const robot_msgs::action::MoveL::Feedback> feedback) {
        std::lock_guard<std::mutex> lock(progress_mutex_);
        if (progress_callback_) {
          progress_callback_(feedback->progress);
        }
      };

  auto goal_handle_future = cli_movel_->async_send_goal(goal, send_goal_options);
  if (goal_handle_future.wait_for(5s) != std::future_status::ready) {
    throw std::runtime_error("MoveL send_goal timeout");
  }

  auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    throw std::runtime_error("MoveL goal was rejected by server");
  }

  if (!block) return;

  auto result_future = cli_movel_->async_get_result(goal_handle);
  if (result_future.wait_for(30s) != std::future_status::ready) {
    throw std::runtime_error("MoveL action result timeout");
  }

  auto result = result_future.get();
  if (!result.result->success) {
    throw std::runtime_error("MoveL failed: " + result.result->message);
  }
}

void ActionRobotController::send_gohome_goal(
    robot_msgs::action::GoHome::Goal goal, bool block) {
  {
    std::lock_guard<std::mutex> lock(session_mutex_);
    goal.session_id = session_id_;
  }

  if (!cli_gohome_->action_server_is_ready()) {
    throw std::runtime_error("GoHome action server not ready");
  }

  auto send_goal_options =
      rclcpp_action::Client<robot_msgs::action::GoHome>::SendGoalOptions();

  send_goal_options.feedback_callback =
      [this](
          std::shared_ptr<rclcpp_action::ClientGoalHandle<
              robot_msgs::action::GoHome>> /*goal_handle*/,
          const std::shared_ptr<
              const robot_msgs::action::GoHome::Feedback> feedback) {
        std::lock_guard<std::mutex> lock(progress_mutex_);
        if (progress_callback_) {
          progress_callback_(feedback->progress);
        }
      };

  auto goal_handle_future = cli_gohome_->async_send_goal(goal, send_goal_options);
  if (goal_handle_future.wait_for(5s) != std::future_status::ready) {
    throw std::runtime_error("GoHome send_goal timeout");
  }

  auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    throw std::runtime_error("GoHome goal was rejected by server");
  }

  if (!block) return;

  auto result_future = cli_gohome_->async_get_result(goal_handle);
  if (result_future.wait_for(30s) != std::future_status::ready) {
    throw std::runtime_error("GoHome action result timeout");
  }

  auto result = result_future.get();
  if (!result.result->success) {
    throw std::runtime_error("GoHome failed: " + result.result->message);
  }
}

// ======================================================================
// IRobotController 运动方法实现
// ======================================================================

void ActionRobotController::set_arm(const std::vector<double>& angles,
                                    bool block) {
  // 委托给 moveJ 关节空间模式
  moveJ(angles, block);
}

void ActionRobotController::set_gripper(double width, bool block) {
  (void)block;
  call_gripper(2, width);
}

void ActionRobotController::open_gripper(bool block) {
  (void)block;
  call_gripper(0);
}

void ActionRobotController::close_gripper(bool block) {
  (void)block;
  call_gripper(1);
}

void ActionRobotController::move_to_pose(
    const std::array<double, 3>& xyz,
    const std::optional<std::array<double, 3>>& rpy,
    double finger, int steps, double step_time, bool block) {
  // 委托给 moveJ 笛卡尔模式（IK 在服务端完成）
  (void)steps;
  (void)step_time;
  moveJ(xyz, rpy, finger, block);
}

void ActionRobotController::move_linear(const std::array<double, 3>& delta,
                                        const std::string& frame,
                                        double finger, bool block) {
  // move_linear 是相对运动（delta），MoveL Action 接受绝对目标。
  // 策略：获取当前 TCP 位姿 → 根据 frame 应用 delta → 发送 MoveL 绝对目标。
  refresh_state_cache();
  std::array<double, 6> current_pose;
  {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    current_pose = cached_pose_;
  }

  // 当前位姿
  double cx = current_pose[0], cy = current_pose[1], cz = current_pose[2];
  double roll = current_pose[3], pitch = current_pose[4], yaw = current_pose[5];

  std::array<double, 3> target_xyz;
  if (frame == "tcp") {
    // TCP 坐标系下的 delta：需要旋转到基坐标系
    // 简化：使用 RPY 构造旋转矩阵，将 delta 从 TCP 系转到 base 系
    double cr = std::cos(roll), sr = std::sin(roll);
    double cp = std::cos(pitch), sp = std::sin(pitch);
    double cy_ = std::cos(yaw), sy = std::sin(yaw);

    // R = Rz(yaw) * Ry(pitch) * Rx(roll) — ZYX Euler
    double r00 = cy_ * cp, r01 = cy_ * sp * sr - sy * cr, r02 = cy_ * sp * cr + sy * sr;
    double r10 = sy * cp,  r11 = sy * sp * sr + cy_ * cr, r12 = sy * sp * cr - cy_ * sr;
    double r20 = -sp,      r21 = cp * sr,                 r22 = cp * cr;

    target_xyz = {
        cx + r00 * delta[0] + r01 * delta[1] + r02 * delta[2],
        cy + r10 * delta[0] + r11 * delta[1] + r12 * delta[2],
        cz + r20 * delta[0] + r21 * delta[1] + r22 * delta[2]
    };
  } else {
    // Base 坐标系：直接加 delta
    target_xyz = {cx + delta[0], cy + delta[1], cz + delta[2]};
  }

  // 保持当前姿态不变，只改变位置
  std::array<double, 3> rpy = {roll, pitch, yaw};
  moveL(target_xyz, rpy, finger, block);
}

void ActionRobotController::rotate_joint(int index, double delta_angle,
                                         bool block) {
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
  moveJ(angles, block);
}

void ActionRobotController::go_home(bool block) {
  robot_msgs::action::GoHome::Goal goal;
  goal.speed_ratio = 1.0;
  send_gohome_goal(goal, block);
}

std::vector<double> ActionRobotController::get_joint_angles() const {
  refresh_state_cache();
  std::lock_guard<std::mutex> lock(cache_mutex_);
  return cached_joints_;
}

std::array<double, 6>
ActionRobotController::get_end_effector_pose() const {
  refresh_state_cache();
  std::lock_guard<std::mutex> lock(cache_mutex_);
  return cached_pose_;
}

double ActionRobotController::get_finger_width() const {
  refresh_state_cache();
  std::lock_guard<std::mutex> lock(cache_mutex_);
  return cached_finger_;
}

void ActionRobotController::set_tcp(const std::string& name) {
  auto req = std::make_shared<robot_msgs::srv::SetTCP::Request>();
  req->name = name;
  auto future = cli_set_tcp_->async_send_request(req);
  if (future.wait_for(2s) != std::future_status::ready) {
    throw std::runtime_error("set_tcp service call timeout");
  }
  auto res = future.get();
  if (!res->success) {
    throw std::runtime_error("set_tcp failed: " + res->message);
  }
}

std::string ActionRobotController::get_current_tcp() const {
  refresh_state_cache();
  std::lock_guard<std::mutex> lock(cache_mutex_);
  return cached_tcp_;
}

std::optional<std::array<double, 6>>
ActionRobotController::lookup_transform(const std::string& target_frame,
                                        const std::string& source_frame,
                                        double /*timeout*/) {
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

void ActionRobotController::moveJ(const std::vector<double>& target_angles,
                                  bool block) {
  robot_msgs::action::MoveJ::Goal goal;
  goal.mode = robot_msgs::action::MoveJ::Goal::JOINT_SPACE;
  for (size_t i = 0; i < 7 && i < target_angles.size(); ++i) {
    goal.joint_angles[i] = target_angles[i];
  }
  goal.speed_ratio = 1.0;
  send_movej_goal(goal, block);
}

void ActionRobotController::moveJ(
    const std::array<double, 3>& xyz,
    const std::optional<std::array<double, 3>>& rpy,
    double finger, bool block) {
  robot_msgs::action::MoveJ::Goal goal;
  goal.mode = robot_msgs::action::MoveJ::Goal::CARTESIAN;

  geometry_msgs::msg::Point position;
  position.x = xyz[0];
  position.y = xyz[1];
  position.z = xyz[2];
  goal.position = position;

  if (rpy) {
    geometry_msgs::msg::Vector3 orientation;
    orientation.x = (*rpy)[0];
    orientation.y = (*rpy)[1];
    orientation.z = (*rpy)[2];
    goal.orientation = orientation;
  }

  goal.speed_ratio = 1.0;
  goal.finger_width = finger;
  send_movej_goal(goal, block);
}

void ActionRobotController::moveL(
    const std::array<double, 3>& xyz,
    const std::optional<std::array<double, 3>>& rpy,
    double finger, bool block) {
  robot_msgs::action::MoveL::Goal goal;

  geometry_msgs::msg::Point position;
  position.x = xyz[0];
  position.y = xyz[1];
  position.z = xyz[2];
  goal.position = position;

  if (rpy) {
    geometry_msgs::msg::Vector3 orientation;
    orientation.x = (*rpy)[0];
    orientation.y = (*rpy)[1];
    orientation.z = (*rpy)[2];
    goal.orientation = orientation;
  }

  goal.speed_ratio = 1.0;
  goal.finger_width = finger;
  send_movel_goal(goal, block);
}

void ActionRobotController::set_speed(MotionMode mode, double percent) {
  auto req = std::make_shared<robot_msgs::srv::SetSpeed::Request>();
  req->mode = (mode == MotionMode::kMoveL) ? 1 : 0;
  req->percent = percent;
  auto future = cli_speed_->async_send_request(req);
  future.wait_for(2s);
  std::lock_guard<std::mutex> lock(cache_mutex_);
  if (mode == MotionMode::kMoveL) {
    cached_speed_l_ = percent;
  } else {
    cached_speed_j_ = percent;
  }
}

double ActionRobotController::get_speed(MotionMode mode) const {
  std::lock_guard<std::mutex> lock(cache_mutex_);
  return (mode == MotionMode::kMoveL) ? cached_speed_l_ : cached_speed_j_;
}

// ======================================================================
// 租约管理
// ======================================================================

bool ActionRobotController::acquire_control(const std::string& client_name,
                                            double lease_duration) {
  auto req = std::make_shared<robot_msgs::srv::AcquireControl::Request>();
  req->client_name = client_name;
  req->lease_duration = lease_duration;
  auto future = cli_acquire_->async_send_request(req);
  if (future.wait_for(5s) != std::future_status::ready) {
    LOG_ERROR("acquire_control service call timeout");
    return false;
  }
  auto res = future.get();
  if (!res->success) {
    LOG_ERROR("acquire_control failed: {}", res->message);
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(session_mutex_);
    session_id_ = res->session_id;
  }

  // 启动续约定时器（3 秒间隔）
  renewal_timer_ = node_->create_wall_timer(
      3s, [this]() { lease_renewal_callback(); });

  LOG_INFO("Control acquired, session_id: {}", res->session_id);
  return true;
}

void ActionRobotController::release_control() {
  // 停止续约定时器
  if (renewal_timer_) {
    renewal_timer_->cancel();
    renewal_timer_.reset();
  }

  std::string sid;
  {
    std::lock_guard<std::mutex> lock(session_mutex_);
    sid = session_id_;
  }

  if (!sid.empty()) {
    auto req = std::make_shared<robot_msgs::srv::ReleaseControl::Request>();
    req->session_id = sid;
    auto future = cli_release_->async_send_request(req);
    if (future.wait_for(5s) != std::future_status::ready) {
      LOG_ERROR("release_control service call timeout");
    } else {
      auto res = future.get();
      if (!res->success) {
        LOG_ERROR("release_control failed: {}", res->message);
      }
    }
  }

  {
    std::lock_guard<std::mutex> lock(session_mutex_);
    session_id_.clear();
  }

  LOG_INFO("Control released");
}

bool ActionRobotController::renew_lease() {
  std::string sid;
  {
    std::lock_guard<std::mutex> lock(session_mutex_);
    sid = session_id_;
  }

  if (sid.empty()) {
    return false;
  }

  auto req = std::make_shared<robot_msgs::srv::RenewLease::Request>();
  req->session_id = sid;
  auto future = cli_renew_->async_send_request(req);
  if (future.wait_for(3s) != std::future_status::ready) {
    LOG_WARN("renew_lease service call timeout");
    return false;
  }
  auto res = future.get();
  if (!res->success) {
    LOG_WARN("renew_lease failed: {}", res->message);
    return false;
  }
  return true;
}

std::string ActionRobotController::session_id() const {
  std::lock_guard<std::mutex> lock(session_mutex_);
  return session_id_;
}

void ActionRobotController::lease_renewal_callback() {
  if (!renew_lease()) {
    LOG_WARN_THROTTLE(5000, "Lease renewal failed, will retry");
  }
}

// ======================================================================
// 进度回调
// ======================================================================

void ActionRobotController::set_progress_callback(ProgressCallback cb) {
  std::lock_guard<std::mutex> lock(progress_mutex_);
  progress_callback_ = std::move(cb);
}

}  // namespace robot_control
