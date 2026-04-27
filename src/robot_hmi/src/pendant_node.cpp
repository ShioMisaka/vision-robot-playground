#include "robot_hmi/pendant_node.hpp"

#include <chrono>
#include <cmath>
#include <thread>

namespace robot_hmi {

std::shared_ptr<PendantNode> PendantNode::create(
    const std::string& robot_service_prefix,
    const std::vector<std::string>& joint_names) {
  auto node = std::shared_ptr<PendantNode>(
      new PendantNode(robot_service_prefix));
  node->joint_names_ = joint_names;
  node->init();
  return node;
}

PendantNode::PendantNode(const std::string& robot_service_prefix)
    : Node("robot_hmi_node"),
      service_prefix_(robot_service_prefix) {}

void PendantNode::set_joint_names(const std::vector<std::string>& names) {
  joint_names_ = names;
}

void PendantNode::init() {
  // Service clients
  cli_move_joint_ = create_client<robot_msgs::srv::MoveJoint>(
      service_prefix_ + "/move_joint");
  cli_move_pose_ = create_client<robot_msgs::srv::MovePose>(
      service_prefix_ + "/move_pose");
  cli_move_linear_ = create_client<robot_msgs::srv::MoveLinear>(
      service_prefix_ + "/move_linear");
  cli_gripper_ = create_client<robot_msgs::srv::ControlGripper>(
      service_prefix_ + "/control_gripper");
  cli_home_ = create_client<robot_msgs::srv::GoHome>(
      service_prefix_ + "/go_home");
  cli_speed_ = create_client<robot_msgs::srv::SetSpeed>(
      service_prefix_ + "/set_speed");
  cli_state_ = create_client<robot_msgs::srv::GetRobotState>(
      service_prefix_ + "/get_state");

  // E-STOP service client (forwards to RobotControllerNode)
  cli_robot_cmd_ = create_client<robot_msgs::srv::RobotCmd>(
      service_prefix_ + "/robot_cmd");

  // Joint state subscriber
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

  // RobotStatus subscription (detect jog completion)
  status_sub_ = create_subscription<robot_msgs::msg::RobotStatus>(
      service_prefix_ + "/status", 10,
      [this](const robot_msgs::msg::RobotStatus::SharedPtr msg) {
        on_robot_status(msg);
      });

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

  // 发布关节目标到控制器（由 100Hz 控制循环统一执行，避免双发布竞争）
  joint_cmd_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      service_prefix_ + "/joint_target", 10);

  // Jog 命令发布器（发送 JogCommand 到 RobotControllerNode）
  jog_pub_ = create_publisher<robot_msgs::msg::JogCommand>(
      service_prefix_ + "/jog_command", 10);

  // 服务发现定时器
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

// ===== Joint State Callback =====

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

  // Sync gripper finger position from joint state feedback.
  // /joint_states has 9 values: 7 arm + 2 gripper (panda_finger_joint1/2).
  // Use the first finger joint's position as finger_width, consistent with
  // the controller's get_current_finger() which reads one finger only.
  if (msg->position.size() >= 8) {
    current_finger_ = msg->position[7];
  }

  if (!was_connected) {
    std::lock_guard<std::mutex> lock(cb_mutex_);
    if (conn_cb_) {
      conn_cb_(true, camera_connected_.load());
    }
  }
}

// ===== Image Callback =====

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
  auto elapsed = std::chrono::duration<double>(now - last_frame_time_).count();
  if (elapsed < 0.033) return;
  last_frame_time_ = now;

  try {
    cv_bridge::CvImageConstPtr rgb_cv =
        cv_bridge::toCvShare(rgb_msg, "bgr8");
    cv_bridge::CvImageConstPtr depth_cv =
        cv_bridge::toCvShare(depth_msg);

    std::lock_guard<std::mutex> lock(cb_mutex_);
    if (image_cb_) {
      cv::Mat rgb;
      cv::cvtColor(rgb_cv->image, rgb, cv::COLOR_BGR2RGB);

      // 深度图归一化 + colormap 可视化
      cv::Mat depth_f;
      if (depth_cv->image.type() == CV_16UC1) {
        depth_cv->image.convertTo(depth_f, CV_32F, 1.0 / 1000.0);
      } else {
        depth_f = depth_cv->image.clone();
      }

      cv::Mat valid_mask = (depth_f > 0);
      double min_val = 0, max_val = 0;
      cv::minMaxLoc(depth_f, &min_val, &max_val, nullptr, nullptr, valid_mask);

      cv::Mat depth_color;
      if (max_val > min_val && max_val > 0) {
        cv::Mat normalized;
        cv::normalize(depth_f, normalized, 0, 255, cv::NORM_MINMAX, CV_8U,
                      valid_mask);
        cv::applyColorMap(normalized, depth_color, cv::COLORMAP_JET);
        depth_color.setTo(cv::Scalar(0, 0, 0), ~valid_mask);
      } else {
        depth_color = cv::Mat::zeros(depth_f.size(), CV_8UC3);
      }
      cv::cvtColor(depth_color, depth_color, cv::COLOR_BGR2RGB);

      image_cb_(rgb, depth_color, depth_f);
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
    auto req = std::make_shared<robot_msgs::srv::GetRobotState::Request>();
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
    auto req = std::make_shared<robot_msgs::srv::MoveJoint::Request>();
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
  // Resolve default finger (-1) to actual current position so the gripper
  // stays where it is instead of being forced open by the controller's
  // grasping_ flag (which defaults to false on startup).
  double actual_finger = (finger < 0) ? current_finger_.load() : finger;
  post_task([this, xyz, rpy, mode, actual_finger, callback]() {
    if (!cli_move_pose_->service_is_ready()) {
      if (callback) callback(false, "Service not available");
      return;
    }
    auto req = std::make_shared<robot_msgs::srv::MovePose::Request>();
    req->xyz = {xyz[0], xyz[1], xyz[2]};
    if (rpy) {
      req->rpy = {(*rpy)[0], (*rpy)[1], (*rpy)[2]};
    }
    req->mode = mode;
    req->finger = actual_finger;
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
    auto req = std::make_shared<robot_msgs::srv::ControlGripper::Request>();
    req->command = 0;
    auto future = cli_gripper_->async_send_request(req);
    if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
      if (callback) callback(false, "Service call timeout");
      return;
    }
    auto res = future.get();
    if (res->success) {
      current_finger_ = 0.04;  // max_width
    }
    if (callback) callback(res->success, res->message);
  });
}

void PendantNode::async_close_gripper(VoidCallback callback) {
  post_task([this, callback]() {
    if (!cli_gripper_->service_is_ready()) {
      if (callback) callback(false, "Service not available");
      return;
    }
    auto req = std::make_shared<robot_msgs::srv::ControlGripper::Request>();
    req->command = 1;
    auto future = cli_gripper_->async_send_request(req);
    if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
      if (callback) callback(false, "Service call timeout");
      return;
    }
    auto res = future.get();
    if (res->success) {
      current_finger_ = 0.0;  // min_width
    }
    if (callback) callback(res->success, res->message);
  });
}

void PendantNode::async_go_home(VoidCallback callback) {
  post_task([this, callback]() {
    if (!cli_home_->service_is_ready()) {
      if (callback) callback(false, "Service not available");
      return;
    }
    auto req = std::make_shared<robot_msgs::srv::GoHome::Request>();
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
    auto req = std::make_shared<robot_msgs::srv::SetSpeed::Request>();
    req->mode = mode;
    req->percent = percent;
    auto future = cli_speed_->async_send_request(req);
    future.wait_for(std::chrono::seconds(2));
  });
}

// ===== Jog Control（仅发布 JogCommand 消息） =====

robot_msgs::msg::JogCommand PendantNode::build_jog_command(
    int axis, uint8_t frame) const {
  robot_msgs::msg::JogCommand msg;
  msg.frame = frame;

  // axis 0=+X,1=-X,2=+Y,3=-Y,4=+Z,5=-Z,6=+R,7=-R,8=+P,9=-P,10=+Yw,11=-Yw
  constexpr double jog_speed = 0.05;       // 50 mm/s for XYZ
  constexpr double jog_rpy_speed = 0.2;    // ~11 deg/s for RPY

  if (axis < 6) {
    int idx = axis / 2;
    double sign = (axis % 2 == 0) ? 1.0 : -1.0;
    msg.velocity[idx] = jog_speed * sign;
  } else {
    int rpy_idx = (axis - 6) / 2;
    double sign = ((axis - 6) % 2 == 0) ? 1.0 : -1.0;
    msg.velocity[3 + rpy_idx] = jog_rpy_speed * sign;
  }

  return msg;
}

void PendantNode::start_jog(int axis, uint8_t /*mode*/, uint8_t frame) {
  jog_active_ = true;
  last_jog_frame_ = frame;
  pause_joint_stream();

  {
    std::lock_guard<std::mutex> lock(jog_mutex_);
    last_jog_msg_ = build_jog_command(axis, frame);
    last_jog_msg_.stamp = now();
    jog_pub_->publish(last_jog_msg_);

    // 启动 50Hz 重复发送（心跳 + 持续指令），看门狗需要持续刷新
    if (!jog_repeat_timer_) {
      jog_repeat_timer_ = create_wall_timer(
          std::chrono::milliseconds(20),
          [this]() {
            if (jog_active_.load()) {
              std::lock_guard<std::mutex> lock(jog_mutex_);
              last_jog_msg_.stamp = now();
              jog_pub_->publish(last_jog_msg_);
            }
          });
    }
  }
}

void PendantNode::stop_jog() {
  if (!jog_active_.load()) return;

  // 停止重复发送
  {
    std::lock_guard<std::mutex> lock(jog_mutex_);
    if (jog_repeat_timer_) {
      jog_repeat_timer_->cancel();
      jog_repeat_timer_.reset();
    }
  }

  // 发布零速度 JogCommand 通知控制器停止
  robot_msgs::msg::JogCommand msg;
  msg.frame = last_jog_frame_;
  msg.stamp = now();
  // velocity is all zeros by default
  jog_pub_->publish(msg);

  // jog_active_ 保持 true 直到 RobotStatus 显示 kIdle
}

// ===== RobotStatus Callback =====

void PendantNode::on_robot_status(
    const robot_msgs::msg::RobotStatus::SharedPtr msg) {
  // 跟踪故障状态
  robot_fault_.store(msg->state == robot_msgs::msg::RobotStatus::FAULT);

  // 故障清除后重置急停标志
  if (msg->state == robot_msgs::msg::RobotStatus::IDLE) {
    emergency_active_ = false;
  }

  if (jog_active_.load() && !emergency_active_.load() &&
      msg->state == robot_msgs::msg::RobotStatus::IDLE) {
    // 控制器从 kTeaching 转为 kIdle → Jog 减速完成
    jog_active_ = false;

    // 同步关节流目标到当前实际位置，避免恢复时机器人跳回旧位置
    {
      std::lock_guard<std::mutex> jlock(latest_joints_mutex_);
      std::lock_guard<std::mutex> slock(joint_stream_mutex_);
      joint_stream_target_ = latest_joints_;
      joint_stream_dirty_ = true;
    }

    resume_joint_stream();

    {
      std::lock_guard<std::mutex> lock(cb_mutex_);
      if (jog_stopped_cb_) jog_stopped_cb_();
    }
  }
}

// ===== Emergency Stop =====

void PendantNode::emergency_stop() {
  jog_active_ = false;
  emergency_active_ = true;
  pause_joint_stream();
  {
    std::lock_guard<std::mutex> jlock(latest_joints_mutex_);
    std::lock_guard<std::mutex> slock(joint_stream_mutex_);
    joint_stream_target_ = latest_joints_;
    joint_stream_dirty_ = true;
  }

  // 转发 E-STOP 到 RobotControllerNode
  post_task([this]() {
    if (cli_robot_cmd_->service_is_ready()) {
      auto req = std::make_shared<robot_msgs::srv::RobotCmd::Request>();
      req->command = robot_msgs::srv::RobotCmd::Request::EMERGENCY_STOP;
      auto future = cli_robot_cmd_->async_send_request(req);
      future.wait_for(std::chrono::seconds(2));
    }
  });
}

void PendantNode::clear_fault() {
  post_task([this]() {
    if (cli_robot_cmd_->service_is_ready()) {
      auto req = std::make_shared<robot_msgs::srv::RobotCmd::Request>();
      req->command = robot_msgs::srv::RobotCmd::Request::CLEAR_FAULT;
      auto future = cli_robot_cmd_->async_send_request(req);
      future.wait_for(std::chrono::seconds(2));
    }
  });
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
    const std::vector<std::string> joint_names = joint_names_.empty()
        ? std::vector<std::string>{
              "panda_joint1", "panda_joint2", "panda_joint3",
              "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"}
        : joint_names_;

    while (joint_stream_running_.load()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      if (joint_stream_paused_.load()) continue;

      std::array<double, 7> target;
      {
        std::lock_guard<std::mutex> lock(joint_stream_mutex_);
        target = joint_stream_target_;
        joint_stream_dirty_ = false;
      }

      // Publish only 7 arm joints — gripper is decoupled and controlled
      // exclusively through ControlGripper service + controller 100Hz loop.
      // Including gripper joints here would overwrite service commands.
      sensor_msgs::msg::JointState msg;
      msg.header.stamp = now();
      msg.name = joint_names;
      msg.position.reserve(7);
      for (int i = 0; i < 7; ++i) {
        msg.position.push_back(target[i]);
      }
      joint_cmd_pub_->publish(msg);
    }
  });
}

void PendantNode::update_joint_target(const std::array<double, 7>& target) {
  if (jog_active_.load()) return;
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

}  // namespace robot_hmi
