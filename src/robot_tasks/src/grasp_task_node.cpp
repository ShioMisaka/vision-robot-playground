#include <chrono>
#include <thread>
#include <cmath>

#include <robot_logger/logger.hpp>

#include "robot_tasks/grasp_task_node.hpp"
#include "robot_vision/vision/color_detector.hpp"
#include "robot_vision/vision/vision_topic_config.hpp"
#include "robot_vision/nodes/vision_processor_node.hpp"

namespace robot_tasks {

// ---- HSV 检测参数（红色物块）----
constexpr std::array<int, 3> kLowerHsv = {0, 100, 100};
constexpr std::array<int, 3> kUpperHsv = {10, 255, 255};

// ---- 相机内参（ZED_X_Mini 默认）----
constexpr double kFx = 490.6666666666667;
constexpr double kFy = 490.6666666666667;
constexpr double kCx = 640.0;
constexpr double kCy = 360.0;

// ---- 抓取默认参数 ----
constexpr double kDefaultApproachHeight = 0.15;
constexpr double kDefaultGraspHeightOffset = -0.025;
constexpr int kDefaultRedetectSamples = 5;
constexpr double kDefaultRedetectInterval = 0.1;
constexpr double kDefaultMaxReach = 0.85;
constexpr double kDefaultApproachStepSize = 0.025;
constexpr double kDefaultApproachTolerance = 0.01;
constexpr int kDefaultMaxApproachSteps = 50;
constexpr int kDefaultMaxConsecutiveFailures = 3;

// ---- 相机外参（eye-in-hand）----
constexpr std::array<double, 3> kCameraOffset = {0.025, -0.015, 0.015};
constexpr std::array<double, 3> kCameraRpy = {M_PI, 0.0, -M_PI / 2.0};
constexpr double kOpticalFramePitch = M_PI;

// ===== 工厂方法 =====
std::shared_ptr<GraspTaskNode> GraspTaskNode::create(
    const std::string& node_name,
    const std::string& target_prefix) {
  try {
    auto node = std::shared_ptr<GraspTaskNode>(
        new GraspTaskNode(node_name, target_prefix));
    node->init();
    return node;
  } catch (const std::exception& e) {
    LOG_ERROR("创建 GraspTaskNode 失败: {}", e.what());
    return nullptr;
  }
}

// ===== 构造函数 =====
GraspTaskNode::GraspTaskNode(const std::string& node_name,
                             const std::string& target_prefix)
    : rclcpp::Node(node_name), target_prefix_(target_prefix) {}

// ===== 初始化 =====
void GraspTaskNode::init() {
  // 1. 创建 RobotClient（连接外部 robot_controller_node）
  robot_client_ = robot_control::RobotClient::create(
      std::string(this->get_name()) + "_client", target_prefix_);
  if (!robot_client_) {
    throw std::runtime_error("创建 RobotClient 失败");
  }

  // 2. 创建 ColorDetector + VisionProcessorNode
  auto detector = std::make_shared<robot_vision::ColorDetector>(
      kLowerHsv, kUpperHsv, kFx, kFy, kCx, kCy);
  robot_vision::VisionTopicConfig topic_config;
  auto vision_processor_node =
      robot_vision::VisionProcessorNode::create(detector, topic_config);
  if (!vision_processor_node) {
    throw std::runtime_error("创建 VisionProcessorNode 失败");
  }
  vision_ = vision_processor_node;            // IVisionProcessor 接口
  vision_node_ = vision_processor_node;       // rclcpp::Node（供 executor spin）

  // 3. 注册 Action Server
  action_server_ = rclcpp_action::create_server<GraspTaskAction>(
      this, "~/grasp_task",
      std::bind(&GraspTaskNode::handle_goal, this,
                std::placeholders::_1, std::placeholders::_2),
      std::bind(&GraspTaskNode::handle_cancel, this, std::placeholders::_1),
      std::bind(&GraspTaskNode::handle_accepted, this, std::placeholders::_1));

  LOG_INFO("GraspTaskNode 初始化完成 (target_prefix={})", target_prefix_);
}

// ===== Goal 回调 =====
rclcpp_action::GoalResponse GraspTaskNode::handle_goal(
    const rclcpp_action::GoalUUID& /*uuid*/,
    std::shared_ptr<const GraspTaskAction::Goal> goal) {
  std::lock_guard<std::mutex> lock(task_mutex_);
  if (current_task_) {
    LOG_WARN("拒绝 GraspTask goal：已有任务正在执行");
    return rclcpp_action::GoalResponse::REJECT;
  }
  LOG_INFO("接受 GraspTask goal: session_id={}, approach_height={:.4f}, "
           "grasp_rpy=[{:.4f}, {:.4f}, {:.4f}]",
           goal->session_id, goal->approach_height,
           goal->grasp_rpy[0], goal->grasp_rpy[1], goal->grasp_rpy[2]);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// ===== Cancel 回调 =====
rclcpp_action::CancelResponse GraspTaskNode::handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<GraspTaskAction>>
        /*goal_handle*/) {
  std::lock_guard<std::mutex> lock(task_mutex_);
  if (current_task_) {
    LOG_INFO("收到取消请求，正在中止 GraspTask...");
    current_task_->request_abort();
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

// ===== Accepted 回调 =====
void GraspTaskNode::handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<GraspTaskAction>>
        goal_handle) {
  // 在新线程中执行，避免阻塞 Action Server
  std::thread{[this, goal_handle]() { this->execute(goal_handle); }}.detach();
}

// ===== 执行 =====
void GraspTaskNode::execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<GraspTaskAction>>
        goal_handle) {
  auto goal = goal_handle->get_goal();

  // 提取 goal 参数
  double approach_height = (goal->approach_height > 0.0)
                               ? goal->approach_height
                               : kDefaultApproachHeight;
  std::array<double, 3> grasp_rpy = {M_PI, 0.0, M_PI};
  if (goal->grasp_rpy.size() == 3) {
    grasp_rpy = {goal->grasp_rpy[0], goal->grasp_rpy[1], goal->grasp_rpy[2]};
  }

  // 等待 RobotClient 就绪
  if (!robot_client_->wait_for_services(10.0)) {
    auto result = std::make_shared<GraspTaskAction::Result>();
    result->success = false;
    result->message = "等待 robot_controller_node 服务超时";
    result->final_state = GraspTaskAction::Goal::ERROR;
    goal_handle->abort(result);
    LOG_ERROR("{}", result->message);
    return;
  }

  // 获取控制器并准备机器人
  auto ctrl = robot_client_->get_controller();
  ctrl->set_speed(robot_control::MotionMode::kMoveJ, 40.0);
  ctrl->set_speed(robot_control::MotionMode::kMoveL, 10.0);
  ctrl->set_tcp("grasptarget");
  ctrl->open_gripper(true);

  // 创建 GraspTaskManager
  {
    std::lock_guard<std::mutex> lock(task_mutex_);
    current_task_ = std::make_unique<GraspTaskManager>(
        ctrl, vision_,
        "panda_link0",                // base_frame
        "camera_color_optical_frame", // camera_frame
        approach_height,
        kDefaultGraspHeightOffset,
        grasp_rpy,
        kDefaultRedetectSamples,
        kDefaultRedetectInterval,
        kDefaultMaxReach,
        kDefaultApproachStepSize,
        kDefaultApproachTolerance,
        kDefaultMaxApproachSteps,
        kDefaultMaxConsecutiveFailures,
        "panda_hand",
        kCameraOffset,
        kCameraRpy,
        kOpticalFramePitch);
    current_goal_ = goal_handle;
  }

  LOG_INFO("开始执行 GraspTask (session_id={})", goal->session_id);

  // 在后台线程运行 GraspTaskManager（阻塞调用）
  bool success = false;
  std::thread task_thread([&]() { success = current_task_->run(30.0); });

  // 主线程：轮询状态 + 发布 Feedback + 检查取消
  GraspState last_state = GraspState::kIdle;
  auto feedback = std::make_shared<GraspTaskAction::Feedback>();

  while (true) {
    GraspState state = current_task_->get_state();

    // 状态变更时发布 Feedback
    if (state != last_state) {
      feedback->current_state = state_to_uint8(state);
      feedback->state_description = state_to_string(state);
      feedback->progress = static_cast<double>(static_cast<int>(state)) /
                           static_cast<double>(static_cast<int>(GraspState::kDone));
      goal_handle->publish_feedback(feedback);
      LOG_INFO("GraspTask 状态: {} ({})", state_to_string(state),
               static_cast<int>(state));
      last_state = state;
    }

    // 终态检测
    if (state == GraspState::kDone || state == GraspState::kError) {
      break;
    }

    // 检查取消请求
    if (goal_handle->is_canceling()) {
      LOG_INFO("Goal 正在取消，请求中止 GraspTask...");
      current_task_->request_abort();
      // 等待 task_thread 退出后再 break
      // （abort 后状态机会尽快进入 kError 并退出 run()）
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // 等待后台线程结束
  if (task_thread.joinable()) {
    task_thread.join();
  }

  // 构建结果
  auto result = std::make_shared<GraspTaskAction::Result>();
  GraspState final_state = current_task_->get_state();
  result->success = success;
  result->final_state = state_to_uint8(final_state);

  auto joints = ctrl->get_joint_angles();
  auto pose = ctrl->get_end_effector_pose();
  for (size_t i = 0; i < 7 && i < joints.size(); ++i) {
    result->final_joint_angles[i] = joints[i];
  }
  for (size_t i = 0; i < 6; ++i) {
    result->final_tcp_pose[i] = pose[i];
  }

  if (success) {
    result->message = "抓取成功";
    goal_handle->succeed(result);
    LOG_INFO("GraspTask 完成: 成功");
  } else if (goal_handle->is_canceling()) {
    result->message = "任务已取消";
    goal_handle->canceled(result);
    LOG_INFO("GraspTask 完成: 已取消");
  } else {
    result->message = "抓取失败: " + std::string(state_to_string(final_state));
    goal_handle->abort(result);
    LOG_WARN("GraspTask 完成: 失败 (state={})",
             state_to_string(final_state));
  }

  // 清理当前任务
  {
    std::lock_guard<std::mutex> lock(task_mutex_);
    current_task_.reset();
    current_goal_.reset();
  }
}

// ===== get_sub_nodes =====
std::vector<rclcpp::Node::SharedPtr> GraspTaskNode::get_sub_nodes() const {
  std::vector<rclcpp::Node::SharedPtr> nodes;
  if (robot_client_) {
    nodes.push_back(robot_client_);
  }
  if (vision_node_) {
    nodes.push_back(vision_node_);
  }
  return nodes;
}

// ===== 状态映射 =====
uint8_t GraspTaskNode::state_to_uint8(GraspState state) {
  return static_cast<uint8_t>(state);
}

const char* GraspTaskNode::state_to_string(GraspState state) {
  switch (state) {
    case GraspState::kIdle:        return "IDLE";
    case GraspState::kDetecting:   return "DETECTING";
    case GraspState::kApproaching: return "APPROACHING";
    case GraspState::kReDetecting: return "RE_DETECTING";
    case GraspState::kDescending:  return "DESCENDING";
    case GraspState::kGrasping:    return "GRASPING";
    case GraspState::kLifting:     return "LIFTING";
    case GraspState::kDone:        return "DONE";
    case GraspState::kError:       return "ERROR";
  }
  return "UNKNOWN";
}

}  // namespace robot_tasks
