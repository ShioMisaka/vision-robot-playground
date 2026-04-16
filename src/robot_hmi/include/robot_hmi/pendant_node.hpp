#pragma once

#include <array>
#include <atomic>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <robot_msgs/srv/move_joint.hpp>
#include <robot_msgs/srv/move_pose.hpp>
#include <robot_msgs/srv/move_linear.hpp>
#include <robot_msgs/srv/control_gripper.hpp>
#include <robot_msgs/srv/go_home.hpp>
#include <robot_msgs/srv/set_speed.hpp>
#include <robot_msgs/srv/get_robot_state.hpp>

#include <robot_msgs/msg/jog_command.hpp>
#include <robot_msgs/msg/robot_status.hpp>
#include <robot_msgs/srv/robot_cmd.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace robot_hmi {

/// ROS2 节点：示教器后端，管理所有 ROS2 通信
class PendantNode : public rclcpp::Node {
public:
  /// 连接状态回调
  using ConnectionCallback = std::function<void(bool robot, bool camera)>;

  /// 图像回调（ROS2 线程中完成 BGR→RGB 转换，返回 RGB cv::Mat）
  using ImageCallback = std::function<void(const cv::Mat& rgb)>;

  static std::shared_ptr<PendantNode> create(
      const std::string& robot_service_prefix = "robot_controller_node");

  ~PendantNode();

  /// 设置连接状态变化回调
  void set_connection_callback(ConnectionCallback cb);

  /// 设置图像回调
  void set_image_callback(ImageCallback cb);

  /// 设置 jog 停止回调（RobotStatus 检测到 kIdle 时触发，用于通知 UI 同步）
  using JogStoppedCallback = std::function<void()>;
  void set_jog_stopped_callback(JogStoppedCallback cb);

  // === 异步控制接口（后台线程执行，不阻塞调用方） ===
  // 完成回调在任意线程执行，Qt 侧需用 QMetaObject::invokeMethod

  using VoidCallback = std::function<void(bool success, const std::string& msg)>;

  void async_get_state(std::function<void(
      bool success,
      const std::vector<double>& joints,
      const std::array<double, 6>& pose,
      double finger_width,
      const std::string& tcp_name)> callback);

  void async_move_joint(const std::vector<double>& angles,
                        VoidCallback callback = nullptr);

  void async_move_pose(const std::array<double, 3>& xyz,
                       const std::optional<std::array<double, 3>>& rpy,
                       uint8_t mode, double finger = -1.0,
                       VoidCallback callback = nullptr);

  void async_open_gripper(VoidCallback callback = nullptr);
  void async_close_gripper(VoidCallback callback = nullptr);

  void async_go_home(VoidCallback callback = nullptr);

  void async_set_speed(uint8_t mode, double percent);

  // === Jog 控制（仅发布 JogCommand 消息，IK 由 RobotControllerNode 处理） ===

  /// @brief 启动 Jog（发布 JogCommand 消息）
  /// @param axis 0~5: XYZ+/XYZ-/...  6~11: RPY+/RPY-...
  /// @param mode 运动模式（未使用，保留）
  /// @param frame 0=TCP, 1=Base
  void start_jog(int axis, uint8_t mode, uint8_t frame);

  /// @brief 停止 Jog（发布零速度 JogCommand）
  void stop_jog();

  // === 急停 ===

  void emergency_stop();

  // === 关节实时流控 ===

  void start_joint_stream(const std::array<double, 7>& initial);
  void update_joint_target(const std::array<double, 7>& target);
  void stop_joint_stream();
  void pause_joint_stream();
  void resume_joint_stream();

  // === 连接状态 ===

  bool is_robot_connected() const { return robot_connected_.load(); }
  bool is_camera_connected() const { return camera_connected_.load(); }
  bool are_services_ready() const { return services_ready_.load(); }

private:
  PendantNode(const std::string& robot_service_prefix);

  void init();

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void image_callback(
      const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg,
      const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg);

  /// RobotStatus 订阅回调：检测 Jog 完成（kTeaching→kIdle）
  void on_robot_status(const robot_msgs::msg::RobotStatus::SharedPtr msg);

  /// 构建 JogCommand 速度消息
  robot_msgs::msg::JogCommand build_jog_command(
      int axis, uint8_t frame) const;

  /// 将任务提交到后台线程池执行
  void post_task(std::function<void()> task);

  std::string service_prefix_;

  // Service clients
  rclcpp::Client<robot_msgs::srv::MoveJoint>::SharedPtr cli_move_joint_;
  rclcpp::Client<robot_msgs::srv::MovePose>::SharedPtr cli_move_pose_;
  rclcpp::Client<robot_msgs::srv::MoveLinear>::SharedPtr cli_move_linear_;
  rclcpp::Client<robot_msgs::srv::ControlGripper>::SharedPtr cli_gripper_;
  rclcpp::Client<robot_msgs::srv::GoHome>::SharedPtr cli_home_;
  rclcpp::Client<robot_msgs::srv::SetSpeed>::SharedPtr cli_speed_;
  rclcpp::Client<robot_msgs::srv::GetRobotState>::SharedPtr cli_state_;

  // E-STOP service client (forwards to RobotControllerNode)
  rclcpp::Client<robot_msgs::srv::RobotCmd>::SharedPtr cli_robot_cmd_;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;

  // RobotStatus subscription
  rclcpp::Subscription<robot_msgs::msg::RobotStatus>::SharedPtr status_sub_;

  // Image sync
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> rgb_sub_;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> depth_sub_;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
  std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  // Callbacks
  std::mutex cb_mutex_;
  ConnectionCallback conn_cb_;
  ImageCallback image_cb_;
  JogStoppedCallback jog_stopped_cb_;

  // Connection state
  std::atomic<bool> robot_connected_{false};
  std::atomic<bool> camera_connected_{false};

  // Service readiness
  std::atomic<bool> services_ready_{false};
  rclcpp::TimerBase::SharedPtr discovery_timer_;

  // === Jog publisher (sends JogCommand to RobotControllerNode) ===
  rclcpp::Publisher<robot_msgs::msg::JogCommand>::SharedPtr jog_pub_;
  std::atomic<bool> jog_active_{false};
  uint8_t last_jog_frame_ = 1;  // BASE_FRAME

  // === Joint stream state ===
  std::mutex joint_stream_mutex_;
  std::array<double, 7> joint_stream_target_{};
  bool joint_stream_dirty_ = false;
  std::thread joint_stream_thread_;
  std::atomic<bool> joint_stream_running_{false};
  std::atomic<bool> joint_stream_paused_{false};

  // Direct joint command publisher (bypasses service layer for low-latency streaming)
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_pub_;
  std::atomic<double> current_finger_{0.04};  // updated by async_get_state

  // Latest joint angles from /joint_states (for resync after jog)
  std::mutex latest_joints_mutex_;
  std::array<double, 7> latest_joints_{};

  // 后台任务队列
  std::mutex task_mutex_;
  std::condition_variable task_cv_;
  std::vector<std::function<void()>> task_queue_;
  std::thread task_thread_;
  std::atomic<bool> task_running_{true};
};

}  // namespace robot_hmi
