#include "teaching_pendant/pendant_node.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <chrono>
#include <cmath>
#include <thread>
#include <iostream>

namespace teaching_pendant {

std::shared_ptr<PendantNode> PendantNode::create(
    const std::string& robot_service_prefix,
    std::shared_ptr<robot_control::IKSolver> ik_solver) {
  auto node = std::shared_ptr<PendantNode>(
      new PendantNode(robot_service_prefix, std::move(ik_solver)));
  node->init();
  return node;
}

PendantNode::PendantNode(const std::string& robot_service_prefix,
                         std::shared_ptr<robot_control::IKSolver> ik_solver)
    : Node("teaching_pendant_node"),
      service_prefix_(robot_service_prefix),
      ik_(std::move(ik_solver)) {}

void PendantNode::init() {
  // Service clients
  cli_ik_ = create_client<robot_control_msgs::srv::SolveIK>(
      service_prefix_ + "/solve_ik");
  cli_move_joint_ = create_client<robot_control_msgs::srv::MoveJoint>(
      service_prefix_ + "/move_joint");
  cli_move_pose_ = create_client<robot_control_msgs::srv::MovePose>(
      service_prefix_ + "/move_pose");
  cli_move_linear_ = create_client<robot_control_msgs::srv::MoveLinear>(
      service_prefix_ + "/move_linear");
  cli_gripper_ = create_client<robot_control_msgs::srv::ControlGripper>(
      service_prefix_ + "/control_gripper");
  cli_home_ = create_client<robot_control_msgs::srv::GoHome>(
      service_prefix_ + "/go_home");
  cli_speed_ = create_client<robot_control_msgs::srv::SetSpeed>(
      service_prefix_ + "/set_speed");
  cli_state_ = create_client<robot_control_msgs::srv::GetRobotState>(
      service_prefix_ + "/get_state");

  // Joint state subscriber（仅用于连接检测）
  auto state_cbg = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = state_cbg;

  joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        joint_state_callback(msg);
      },
      sub_opts);

  // Camera image subscribers (synchronized)
  auto qos_profile = rclcpp::SensorDataQoS();

  rgb_sub_ =
      std::make_unique<message_filters::Subscriber<sensor_msgs::msg::Image>>(
          shared_from_this(), "/camera/image_raw/left",
          qos_profile.get_rmw_qos_profile());

  depth_sub_ =
      std::make_unique<message_filters::Subscriber<sensor_msgs::msg::Image>>(
          shared_from_this(), "/camera/image_raw/depth",
          qos_profile.get_rmw_qos_profile());

  sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(10), *rgb_sub_, *depth_sub_);
  sync_->setMaxIntervalDuration(rclcpp::Duration(0, 100000000));  // 100ms
  sync_->registerCallback(&PendantNode::image_callback, this);

  // 后台任务线程
  task_thread_ = std::thread([this]() {
    while (task_running_.load()) {
      std::function<void()> task;
      {
        std::unique_lock<std::mutex> lock(task_mutex_);
        task_cv_.wait(lock, [this]() {
          return !task_queue_.empty() || !task_running_.load();
        });
        if (!task_running_.load() && task_queue_.empty()) return;
        if (!task_queue_.empty()) {
          task = std::move(task_queue_.front());
          task_queue_.erase(task_queue_.begin());
        }
      }
      if (task) task();
    }
  });

  // 直接发布关节指令（低延迟，绕过 service 层）
  joint_cmd_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      "/joint_command", 10);

  // Jog 命令发布器（仅用于 robot_controller_node 状态机管理）
  jog_pub_ = create_publisher<arm_control_interfaces::msg::JogCommand>(
      service_prefix_ + "/jog_command", 10);

  // 服务发现定时器：周期性检查所有关键服务的 DDS 可用性
  discovery_timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      [this]() {
        bool ready = cli_state_->service_is_ready() &&
                     cli_move_joint_->service_is_ready() &&
                     cli_move_pose_->service_is_ready() &&
                     cli_move_linear_->service_is_ready() &&
                     cli_gripper_->service_is_ready() &&
                     cli_home_->service_is_ready();
        bool was_ready = services_ready_.exchange(ready);
        if (ready && !was_ready) {
          RCLCPP_INFO(get_logger(), "All robot services discovered and ready");
        }
      });

  RCLCPP_INFO(get_logger(), "TeachingPendantNode started (service prefix: %s)",
              service_prefix_.c_str());
}

PendantNode::~PendantNode() {
  stop_joint_stream();
  stop_jog();
  task_running_ = false;
  task_cv_.notify_all();
  if (task_thread_.joinable()) {
    task_thread_.join();
  }
}

// ===== Callback Setters =====

void PendantNode::set_connection_callback(ConnectionCallback cb) {
  std::lock_guard<std::mutex> lock(cb_mutex_);
  conn_cb_ = std::move(cb);
}

void PendantNode::set_image_callback(ImageCallback cb) {
  std::lock_guard<std::mutex> lock(cb_mutex_);
  image_cb_ = std::move(cb);
}

void PendantNode::set_jog_stopped_callback(JogStoppedCallback cb) {
  std::lock_guard<std::mutex> lock(cb_mutex_);
  jog_stopped_cb_ = std::move(cb);
}

// ===== 后台任务队列 =====

void PendantNode::post_task(std::function<void()> task) {
  {
    std::lock_guard<std::mutex> lock(task_mutex_);
    task_queue_.push_back(std::move(task));
  }
  task_cv_.notify_one();
}

// ===== Joint State Callback（仅连接检测） =====

void PendantNode::joint_state_callback(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
  bool was_connected = robot_connected_.load();
  robot_connected_ = true;

  // Cache latest arm joint angles for resync after jog
  {
    std::lock_guard<std::mutex> lock(latest_joints_mutex_);
    for (int i = 0; i < 7 && i < static_cast<int>(msg->position.size()); ++i) {
      latest_joints_[i] = msg->position[i];
    }
  }

  if (!was_connected) {
    std::lock_guard<std::mutex> lock(cb_mutex_);
    if (conn_cb_) {
      conn_cb_(true, camera_connected_.load());
    }
  }
}

// ===== Image Callback（含 OpenCV 处理，在 ROS2 线程执行） =====

void PendantNode::image_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg) {
  bool was_connected = camera_connected_.load();
  camera_connected_ = true;

  if (!was_connected) {
    std::lock_guard<std::mutex> lock(cb_mutex_);
    if (conn_cb_) {
      conn_cb_(robot_connected_.load(), true);
    }
  }

  // 帧率限制：~30fps
  auto now = std::chrono::steady_clock::now();
  static auto last_frame = now;
  auto elapsed = std::chrono::duration<double>(now - last_frame).count();
  if (elapsed < 0.033) return;
  last_frame = now;

  try {
    cv_bridge::CvImageConstPtr rgb_cv =
        cv_bridge::toCvShare(rgb_msg, "bgr8");
    cv_bridge::CvImageConstPtr depth_cv =
        cv_bridge::toCvShare(depth_msg);

    // 在 ROS2 线程中完成 OpenCV 处理，生成 QImage
    cv::Mat display;

    std::lock_guard<std::mutex> lock(cb_mutex_);
    if (image_cb_) {
      cv::Mat rgb;
      cv::cvtColor(rgb_cv->image, rgb, cv::COLOR_BGR2RGB);
      image_cb_(rgb);
    }
  } catch (const cv_bridge::Exception& e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Image conversion failed: %s", e.what());
  }
}

// ===== 异步控制接口 =====

void PendantNode::async_get_state(std::function<void(
    bool success,
    const std::vector<double>& joints,
    const std::array<double, 6>& pose,
    double finger_width,
    const std::string& tcp_name)> callback) {
  post_task([this, callback]() {
    if (!cli_state_->service_is_ready()) {
      if (callback) callback(false, {}, {}, 0.0, "");
      return;
    }
    auto req = std::make_shared<robot_control_msgs::srv::GetRobotState::Request>();
    auto future = cli_state_->async_send_request(req);
    if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
      if (callback) callback(false, {}, {}, 0.0, "");
      return;
    }
    auto res = future.get();
    if (!res->success) {
      if (callback) callback(false, {}, {}, 0.0, "");
      return;
    }
    std::vector<double> joints = res->joint_angles;
    std::array<double, 6> pose;
    for (int i = 0; i < 6 && i < (int)res->tcp_pose.size(); ++i) {
      pose[i] = res->tcp_pose[i];
    }
    if (callback) callback(true, joints, pose, res->finger_width, res->tcp_name);
    current_finger_ = res->finger_width;
  });
}

void PendantNode::async_move_joint(const std::vector<double>& angles,
                                   VoidCallback callback) {
  post_task([this, angles, callback]() {
    if (!cli_move_joint_->service_is_ready()) {
      if (callback) callback(false, "Service not available");
      return;
    }
    auto req = std::make_shared<robot_control_msgs::srv::MoveJoint::Request>();
    req->joint_angles = angles;
    req->block = true;
    auto future = cli_move_joint_->async_send_request(req);
    if (future.wait_for(std::chrono::seconds(15)) != std::future_status::ready) {
      if (callback) callback(false, "Service call timeout");
      return;
    }
    auto res = future.get();
    if (callback) callback(res->success, res->message);
  });
}

void PendantNode::async_move_pose(const std::array<double, 3>& xyz,
                                  const std::optional<std::array<double, 3>>& rpy,
                                  uint8_t mode, double finger,
                                  VoidCallback callback) {
  post_task([this, xyz, rpy, mode, finger, callback]() {
    if (!cli_move_pose_->service_is_ready()) {
      if (callback) callback(false, "Service not available");
      return;
    }
    auto req = std::make_shared<robot_control_msgs::srv::MovePose::Request>();
    req->xyz = {xyz[0], xyz[1], xyz[2]};
    if (rpy) {
      req->rpy = {(*rpy)[0], (*rpy)[1], (*rpy)[2]};
    }
    req->mode = mode;
    req->finger = finger;
    auto future = cli_move_pose_->async_send_request(req);
    if (future.wait_for(std::chrono::seconds(15)) != std::future_status::ready) {
      if (callback) callback(false, "Service call timeout");
      return;
    }
    auto res = future.get();
    if (callback) callback(res->success, res->message);
  });
}

void PendantNode::async_open_gripper(VoidCallback callback) {
  post_task([this, callback]() {
    if (!cli_gripper_->service_is_ready()) {
      if (callback) callback(false, "Service not available");
      return;
    }
    auto req = std::make_shared<robot_control_msgs::srv::ControlGripper::Request>();
    req->command = 0;
    auto future = cli_gripper_->async_send_request(req);
    if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
      if (callback) callback(false, "Service call timeout");
      return;
    }
    auto res = future.get();
    if (callback) callback(res->success, res->message);
  });
}

void PendantNode::async_close_gripper(VoidCallback callback) {
  post_task([this, callback]() {
    if (!cli_gripper_->service_is_ready()) {
      if (callback) callback(false, "Service not available");
      return;
    }
    auto req = std::make_shared<robot_control_msgs::srv::ControlGripper::Request>();
    req->command = 1;
    auto future = cli_gripper_->async_send_request(req);
    if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
      if (callback) callback(false, "Service call timeout");
      return;
    }
    auto res = future.get();
    if (callback) callback(res->success, res->message);
  });
}

void PendantNode::async_go_home(VoidCallback callback) {
  post_task([this, callback]() {
    if (!cli_home_->service_is_ready()) {
      if (callback) callback(false, "Service not available");
      return;
    }
    auto req = std::make_shared<robot_control_msgs::srv::GoHome::Request>();
    auto future = cli_home_->async_send_request(req);
    if (future.wait_for(std::chrono::seconds(15)) != std::future_status::ready) {
      if (callback) callback(false, "Service call timeout");
      return;
    }
    auto res = future.get();
    if (callback) callback(res->success, res->message);
  });
}

void PendantNode::async_set_speed(uint8_t mode, double percent) {
  post_task([this, mode, percent]() {
    if (!cli_speed_->service_is_ready()) return;
    auto req = std::make_shared<robot_control_msgs::srv::SetSpeed::Request>();
    req->mode = mode;
    req->percent = percent;
    auto future = cli_speed_->async_send_request(req);
    future.wait_for(std::chrono::seconds(2));
  });
}

// ===== Jog Control =====
//
// Architecture: 3-phase S-curve velocity profile + Jacobian velocity IK
//
//   Phase 1 (Jerk-up):   acceleration ramps from 0 to a_max at rate j_max
//   Phase 2 (Const Acc): acceleration holds at a_max
//   Phase 3 (Jerk-down): acceleration ramps from a_max to 0 at rate j_max
//                         → velocity reaches v_max with zero derivative
//
//   Deceleration is the mirror image: a_max → 0 → -a_max → 0
//
//   Joint synchronization: after computing dq = J⁺ * dx, check each joint
//   velocity against its physical limit and scale down the Cartesian delta
//   if any joint would exceed its limit.

void PendantNode::start_jog(int axis, uint8_t /*mode*/, uint8_t frame) {
  if (!ik_) {
    RCLCPP_WARN(get_logger(), "Jog unavailable: no IK solver");
    return;
  }

  // If currently decelerating from previous jog, snap to current and restart
  if (jog_stopping_ && jog_timer_) {
    jog_timer_.reset();
    jog_stopping_ = false;
  }

  // Mark jog as active — update_joint_target() from JointControlPanel is blocked
  jog_active_ = true;
  jog_stopping_ = false;
  jog_v_ = 0.0;   // velocity scale starts at zero
  jog_a_ = 0.0;   // acceleration starts at zero

  // Initialize internal commanded position from actual robot position
  {
    std::lock_guard<std::mutex> lock(latest_joints_mutex_);
    jog_q_current_ = latest_joints_;
  }

  // Build JogCommand velocity: axis maps to velocity[0..5]
  // axis 0=+X,1=-X,2=+Y,3=-Y,4=+Z,5=-Z,6=+R,7=-R,8=+P,9=-P,10=+Yw,11=-Yw
  jog_msg_ = arm_control_interfaces::msg::JogCommand();
  jog_msg_.frame = frame;

  const double jog_speed = 0.05;       // 50 mm/s for XYZ
  const double jog_rpy_speed = 0.2;    // ~11 deg/s for RPY

  if (axis < 6) {
    int idx = axis / 2;
    double sign = (axis % 2 == 0) ? 1.0 : -1.0;
    jog_msg_.velocity[idx] = jog_speed * sign;
  } else {
    int rpy_idx = (axis - 6) / 2;
    double sign = ((axis - 6) % 2 == 0) ? 1.0 : -1.0;
    jog_msg_.velocity[3 + rpy_idx] = jog_rpy_speed * sign;
  }

  // 50Hz timer: Jacobian velocity IK + S-curve ramp
  jog_timer_ = create_wall_timer(
      std::chrono::milliseconds(20),
      [this]() { jog_tick(); });
}

void PendantNode::stop_jog() {
  if (!jog_timer_) return;
  // Enter deceleration phase — timer keeps running until v reaches 0
  jog_stopping_ = true;
}

void PendantNode::jog_finish() {
  jog_timer_.reset();
  jog_active_ = false;
  jog_stopping_ = false;
  jog_v_ = 0.0;
  jog_a_ = 0.0;

  // Stream target stays at last jog_q_current_ — no snap to feedback.
  // JointControlPanel will take over from the next state update tick.

  {
    std::lock_guard<std::mutex> lock(cb_mutex_);
    if (jog_stopped_cb_) jog_stopped_cb_();
  }
}

void PendantNode::jog_tick() {
  constexpr double dt = 0.02;  // 50Hz

  // === S-curve ramp parameters (normalized velocity 0..1) ===
  // These control the shape of the velocity profile.
  // With a_max=5.0 and j_max=50.0:
  //   Jerk-up/down time: a_max/j_max = 0.1s each
  //   Const-accel time:  (1.0 - 2*0.1*0.1*50/2) / 5.0 ≈ 0.1s
  //   Total accel: ~0.3s, Total decel: ~0.3s
  constexpr double a_max = 5.0;    // max acceleration of velocity scale (1/s²)
  constexpr double j_max = 50.0;   // max jerk of velocity scale (1/s³)

  // === 1. S-curve velocity ramp (jerk-limited acceleration control) ===
  if (jog_stopping_) {
    // --- Deceleration to v=0 ---
    // Goal: reach v=0 with a=0 (no residual acceleration at stop)
    // The "stopping distance" (velocity to shed during final jerk-up) is:
    //   Δv_jerkup = a² / (2 * j_max)
    // When v ≤ Δv_jerkup, enter final jerk-up phase.

    if (jog_a_ < 0 && jog_v_ <= jog_a_ * jog_a_ / (2.0 * j_max) + 1e-8) {
      // Final jerk-up phase: smoothly reduce |a| to 0
      // Target: when a=0, v=0.  So: v ≈ a²/(2*j) → a = -sqrt(2*j*v)
      double a_target = -std::sqrt(2.0 * j_max * std::max(0.0, jog_v_));
      // Apply jerk limit to approach a_target
      double da = a_target - jog_a_;
      da = std::max(-j_max * dt, std::min(j_max * dt, da));
      jog_a_ += da;
    } else {
      // Increase deceleration toward -a_max
      double da = -a_max - jog_a_;
      da = std::max(-j_max * dt, std::min(j_max * dt, da));
      jog_a_ += da;
    }

    jog_v_ += jog_a_ * dt;
    if (jog_v_ <= 0.0) {
      jog_v_ = 0.0;
      jog_a_ = 0.0;
      jog_finish();
      return;
    }
  } else {
    // --- Acceleration to v=1.0 ---
    // Goal: reach v=1.0 with a=0
    // The velocity gained during final jerk-down is:
    //   Δv_jerkdn = a² / (2 * j_max)
    // When remaining Δv ≤ Δv_jerkdn, enter final jerk-down phase.

    double dv_remaining = 1.0 - jog_v_;
    if (jog_a_ > 0 && dv_remaining <= jog_a_ * jog_a_ / (2.0 * j_max) + 1e-8) {
      // Final jerk-down phase: smoothly reduce a to 0
      // Target: v + a²/(2*j) = 1.0 → a = sqrt(2*j*(1.0-v))
      double a_target = std::sqrt(2.0 * j_max * std::max(0.0, dv_remaining));
      double da = a_target - jog_a_;
      da = std::max(-j_max * dt, std::min(j_max * dt, da));
      jog_a_ += da;
    } else {
      // Increase acceleration toward a_max
      double da = a_max - jog_a_;
      da = std::max(-j_max * dt, std::min(j_max * dt, da));
      jog_a_ += da;
    }

    jog_v_ += jog_a_ * dt;
    if (jog_v_ >= 1.0) {
      jog_v_ = 1.0;
      jog_a_ = 0.0;
    }
  }

  // === 2. Scale Cartesian velocity by ramp ===
  std::array<double, 6> vel{};
  for (int i = 0; i < 6; ++i) {
    vel[i] = jog_msg_.velocity[i] * jog_v_;
  }

  // === 3. Frame transformation (TCP → Base) ===
  if (jog_msg_.frame == arm_control_interfaces::msg::JogCommand::TCP_FRAME) {
    std::vector<double> q_vec(jog_q_current_.begin(), jog_q_current_.end());
    Eigen::Matrix4d T = ik_->forward_matrix(q_vec);
    Eigen::Matrix3d R = T.block<3, 3>(0, 0);

    Eigen::Vector3d v_tcp(vel[0], vel[1], vel[2]);
    Eigen::Vector3d v_base = R * v_tcp;
    vel[0] = v_base.x(); vel[1] = v_base.y(); vel[2] = v_base.z();

    Eigen::Vector3d w_tcp(vel[3], vel[4], vel[5]);
    Eigen::Vector3d w_base = R * w_tcp;
    vel[3] = w_base.x(); vel[4] = w_base.y(); vel[5] = w_base.z();
  }

  // === 4. Cartesian delta = velocity * dt ===
  std::array<double, 6> delta{};
  for (int i = 0; i < 6; ++i) {
    delta[i] = vel[i] * dt;
  }

  // === 5. Velocity IK: dq = J⁺ * delta ===
  std::vector<double> q_vec(jog_q_current_.begin(), jog_q_current_.end());
  auto result = ik_->velocity_ik(q_vec, delta);

  // === 6. Joint velocity limit enforcement (synchronization) ===
  // If any joint velocity exceeds its physical limit, scale down the
  // entire delta proportionally so ALL joints slow down together.
  if (result) {
    // Panda joint velocity limits (rad/s)
    constexpr double joint_vel_limits[7] = {
        2.175, 2.175, 2.175, 2.175, 2.610, 2.610, 2.610};
    double max_ratio = 0.0;
    for (int i = 0; i < 7; ++i) {
      double dq = std::abs((*result)[i] - jog_q_current_[i]);
      double ratio = dq / (joint_vel_limits[i] * dt);
      if (ratio > max_ratio) max_ratio = ratio;
    }
    if (max_ratio > 1.0) {
      // Scale down to respect the most-limited joint
      double scale = 1.0 / max_ratio;
      for (int i = 0; i < 7; ++i) {
        (*result)[i] = jog_q_current_[i] + ((*result)[i] - jog_q_current_[i]) * scale;
      }
    }

    for (int i = 0; i < 7; ++i) {
      jog_q_current_[i] = (*result)[i];
    }
    set_jog_stream_target(jog_q_current_);
  }

  // === 7. Send JogCommand for state machine ===
  jog_msg_.stamp = now();
  jog_pub_->publish(jog_msg_);

  // === 8. Debug logging (throttled to 2Hz) ===
  static auto last_dbg = std::chrono::steady_clock::now();
  static int tick_count = 0;
  tick_count++;
  auto now_dbg = std::chrono::steady_clock::now();
  if ((now_dbg - last_dbg) > std::chrono::milliseconds(500)) {
    last_dbg = now_dbg;
    auto pose_cmd = ik_->forward(q_vec);
    std::cout << "[JOG] #" << tick_count
              << " v=" << jog_v_ << " a=" << jog_a_
              << (jog_stopping_ ? " DECEL" : " ACCEL")
              << "\n  cmd xyz=[" << pose_cmd[0] << "," << pose_cmd[1] << "," << pose_cmd[2] << "]"
              << " rpy=[" << pose_cmd[3] << "," << pose_cmd[4] << "," << pose_cmd[5] << "]"
              << "\n  vel=[" << vel[0] << "," << vel[1] << "," << vel[2] << ","
              << vel[3] << "," << vel[4] << "," << vel[5] << "]"
              << " IK=" << (result ? "OK" : "FAIL") << std::endl;
  }
}

void PendantNode::emergency_stop() {
  // Immediate stop — skip deceleration ramp
  if (jog_timer_) {
    jog_timer_.reset();
  }
  jog_active_ = false;
  jog_stopping_ = false;
  jog_v_ = 0.0;
  jog_a_ = 0.0;
  pause_joint_stream();
  {
    std::lock_guard<std::mutex> jlock(latest_joints_mutex_);
    std::lock_guard<std::mutex> slock(joint_stream_mutex_);
    joint_stream_target_ = latest_joints_;
    joint_stream_dirty_ = true;
  }
}

// ===== Joint Stream Control =====

void PendantNode::start_joint_stream(const std::array<double, 7>& initial) {
  if (joint_stream_running_.load()) return;
  {
    std::lock_guard<std::mutex> lock(joint_stream_mutex_);
    joint_stream_target_ = initial;
    joint_stream_dirty_ = false;
  }
  joint_stream_running_ = true;
  joint_stream_paused_ = false;
  joint_stream_thread_ = std::thread([this]() {
    const std::vector<std::string> joint_names = {
        "panda_joint1", "panda_joint2", "panda_joint3",
        "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7",
        "panda_finger_joint1", "panda_finger_joint2"};

    while (joint_stream_running_.load()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      if (joint_stream_paused_.load()) continue;

      // Always publish at 50Hz — Isaac Sim needs continuous position
      // commands to hold joints against gravity
      std::array<double, 7> target;
      {
        std::lock_guard<std::mutex> lock(joint_stream_mutex_);
        target = joint_stream_target_;
        joint_stream_dirty_ = false;
      }

      sensor_msgs::msg::JointState msg;
      msg.header.stamp = now();
      msg.name = joint_names;
      msg.position.reserve(9);
      for (int i = 0; i < 7; ++i) {
        msg.position.push_back(target[i]);
      }
      double finger = current_finger_.load();
      msg.position.push_back(finger);
      msg.position.push_back(finger);
      joint_cmd_pub_->publish(msg);
    }
  });
}

void PendantNode::update_joint_target(const std::array<double, 7>& target) {
  // During jog, block updates from JointControlPanel to avoid overwriting
  // jog IK results — the jog result subscriber uses set_jog_stream_target()
  if (jog_active_.load()) return;
  std::lock_guard<std::mutex> lock(joint_stream_mutex_);
  joint_stream_target_ = target;
  joint_stream_dirty_ = true;
}

void PendantNode::set_jog_stream_target(const std::array<double, 7>& target) {
  std::lock_guard<std::mutex> lock(joint_stream_mutex_);
  joint_stream_target_ = target;
  joint_stream_dirty_ = true;
}

void PendantNode::stop_joint_stream() {
  joint_stream_running_ = false;
  joint_stream_paused_ = false;
  if (joint_stream_thread_.joinable()) {
    joint_stream_thread_.join();
  }
}

void PendantNode::pause_joint_stream() {
  joint_stream_paused_ = true;
}

void PendantNode::resume_joint_stream() {
  joint_stream_paused_ = false;
}

}  // namespace teaching_pendant
