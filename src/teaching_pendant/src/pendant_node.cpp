#include "teaching_pendant/pendant_node.hpp"

#include <chrono>
#include <thread>
#include <iostream>

namespace teaching_pendant {

std::shared_ptr<PendantNode> PendantNode::create(
    const std::string& robot_service_prefix) {
  auto node = std::shared_ptr<PendantNode>(
      new PendantNode(robot_service_prefix));
  node->init();
  return node;
}

PendantNode::PendantNode(const std::string& robot_service_prefix)
    : Node("teaching_pendant_node"),
      service_prefix_(robot_service_prefix) {}

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
  emergency_stop();
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
    const sensor_msgs::msg::JointState::SharedPtr /*msg*/) {
  bool was_connected = robot_connected_.load();
  robot_connected_ = true;

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

void PendantNode::start_jog(int axis, double direction, uint8_t mode) {
  if (jog_running_.load()) stop_jog();

  jog_axis_ = axis;
  jog_direction_ = direction;
  jog_mode_ = mode;
  jog_running_ = true;

  jog_thread_ = std::thread([this]() { jog_loop(); });
}

void PendantNode::stop_jog() {
  jog_running_ = false;
  if (jog_thread_.joinable()) {
    jog_thread_.join();
  }
}

void PendantNode::jog_loop() {
  int axis = jog_axis_.load();
  double dir = jog_direction_.load();

  double delta = 0.005;      // 5mm per step
  double rpy_delta = 0.02;   // ~1.1 deg per step

  while (jog_running_.load()) {
    if (!cli_move_linear_->service_is_ready()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }

    std::array<double, 3> d{0, 0, 0};
    if (axis < 6) {
      int idx = axis / 2;
      double sign = (axis % 2 == 0) ? 1.0 : -1.0;
      d[idx] = delta * sign * dir;
    } else {
      int rpy_idx = (axis - 6) / 2;
      double sign = ((axis - 6) % 2 == 0) ? 1.0 : -1.0;
      d[rpy_idx] = rpy_delta * sign * dir;
    }

    auto req = std::make_shared<robot_control_msgs::srv::MoveLinear::Request>();
    req->delta = {d[0], d[1], d[2]};
    req->frame = "base";
    auto future = cli_move_linear_->async_send_request(req);
    if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                           "Jog service call timeout");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

void PendantNode::emergency_stop() {
  stop_jog();
}

}  // namespace teaching_pendant
