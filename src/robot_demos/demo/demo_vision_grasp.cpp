/// @file demo_vision_grasp.cpp
/// @brief C++ 两阶段视觉引导抓取演示（客户端模式，连接外部 robot_controller_node）
///
/// 编译后运行（需要 Isaac Sim 已启动 + robot_controller_node 已运行）：
///   source /opt/ros/jazzy/setup.bash
///   source install/setup.bash
///   ros2 run robot_demos demo_vision_grasp
///
/// 按 Ctrl+C 可安全中止，不会卡死。
///
/// 流程：
///   1. 连接外部 robot_controller_node（不创建自己的控制器实例）
///   2. 张开夹爪
///   3. GraspTaskManager.run() 自动执行两阶段抓取：
///      - kDetecting: 远距离粗检测
///      - kApproaching: moveJ 快速接近目标上方
///      - kDescending: moveL 下降到精确抓取位
///      - kGrasping: 闭合夹爪
///      - kLifting: 提起物体

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <robot_logger/logger.hpp>

#include "robot_controller/client/robot_client.hpp"

#include "robot_tasks/grasp_task_manager.hpp"
#include "robot_vision/vision/color_detector.hpp"
#include "robot_vision/vision/vision_topic_config.hpp"
#include "robot_vision/nodes/vision_processor_node.hpp"

using namespace robot_control;
using namespace robot_vision;
using namespace robot_tasks;

// ---- 全局中止标志 ----
static std::atomic<bool> g_abort{false};

// ---- HSV 检测参数（红色物块）----
const std::array<int, 3> kLowerHsv = {0, 100, 100};
const std::array<int, 3> kUpperHsv = {10, 255, 255};

// ---- 相机内参 ----
constexpr double kFx = 490.6666666666667;
constexpr double kFy = 490.6666666666667;
constexpr double kCx = 640.0;
constexpr double kCy = 360.0;

// ---- 运动参数 ----
constexpr double kMoveJSpeed = 40.0;
constexpr double kMoveLSpeed = 10.0;

// ---- 抓取参数 ----
constexpr double kApproachHeight = 0.15;      // 目标上方 15cm
constexpr double kGraspHeightOffset = -0.025; // 抓取高度偏移：视觉检测到物块顶面，需下降到 5cm 物块中心
constexpr int kRedetectSamples = 5;            // 重检测采样次数
constexpr double kRedetectInterval = 0.1;      // 采样间隔 100ms

// 夹爪朝下姿态 [roll, pitch, yaw]
const std::array<double, 3> kGraspRpy = {M_PI, 0.0, M_PI};

/// 状态名转字符串（用于日志）
const char* state_name(GraspState s) {
  switch (s) {
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

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  // ---- 创建客户端节点（连接外部 robot_controller_node）----
  auto robot_client = RobotClient::create("demo_vision_grasp", "robot_controller_node");
  auto detector = std::make_shared<ColorDetector>(
      kLowerHsv, kUpperHsv,
      kFx, kFy, kCx, kCy);
  robot_vision::VisionTopicConfig config;
  auto vision_node = VisionProcessorNode::create(detector, config);

  // ---- 后台 Executor ----
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(robot_client);
  executor->add_node(vision_node);
  std::thread spin_thread([executor]() { executor->spin(); });

  // ---- 等待服务就绪 ----
  LOG_INFO("等待 robot_controller_node 服务就绪...");
  bool ready = robot_client->wait_for_services(10.0);
  if (!ready) {
    LOG_ERROR("等待控制器服务超时，请确认 robot_controller_node 已启动");
    goto cleanup;
  }

  if (!rclcpp::ok()) goto cleanup;

  {
    auto ctrl = robot_client->get_controller();

    // ---- 诊断日志：初始状态 ----
    {
      auto pose = ctrl->get_end_effector_pose();
      LOG_INFO("[DIAG] 初始 TCP: xyz=[{:.4f}, {:.4f}, {:.4f}] rpy=[{:.4f}, {:.4f}, {:.4f}]",
               pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
    }

    // ---- 设置速度 ----
    ctrl->set_speed(robot_control::MotionMode::kMoveJ, kMoveJSpeed);
    ctrl->set_speed(robot_control::MotionMode::kMoveL, kMoveLSpeed);
    LOG_INFO("[DIAG] 速度: moveJ={:.0f}%, moveL={:.0f}%",
             kMoveJSpeed, kMoveLSpeed);

    // ---- 切换到指尖坐标系 ----
    ctrl->set_tcp("grasptarget");
    LOG_INFO("[DIAG] TCP 切换为 grasptarget");

    // ---- 张开夹爪 ----
    LOG_INFO("张开夹爪");
    ctrl->open_gripper(true);

    // ---- 等待首次检测（诊断）----
    LOG_INFO("[DIAG] 等待视觉检测（10s 超时）...");
    auto first_result = vision_node->wait_for_detection(10.0);
    if (!first_result.has_value() || !first_result->detected) {
      LOG_ERROR("[DIAG] 初始检测失败，请确认红色物块在相机视野内");
      goto cleanup;
    }
    LOG_INFO("[DIAG] 首次检测: uv=[{}, {}], camera_xyz=[{:.4f}, {:.4f}, {:.4f}]",
             first_result->uv.x(), first_result->uv.y(),
             first_result->xyz.x(), first_result->xyz.y(),
             first_result->xyz.z());
    // 诊断：验证 uv 到 camera_xyz 的投影是否一致
    double verify_x = (first_result->uv.x() - kCx) *
                      first_result->xyz.z() / kFx;
    double verify_y = (first_result->uv.y() - kCy) *
                      first_result->xyz.z() / kFy;
    LOG_INFO("[DIAG] 投影验证: 由uv反算 x={:.4f} (实际{:.4f}) y={:.4f} (实际{:.4f}) "
             "depth={:.4f}",
             verify_x, first_result->xyz.x(),
             verify_y, first_result->xyz.y(),
             first_result->xyz.z());

    if (!rclcpp::ok()) goto cleanup;

    // ---- 执行两阶段抓取 ----
    LOG_INFO("开始两阶段视觉引导抓取"
             "（粗检测 → moveJ接近 → moveL下降 → 抓取）");

    GraspTaskManager task(
        ctrl, vision_node,
        "panda_link0",
        "camera_color_optical_frame",
        kApproachHeight,
        kGraspHeightOffset,
        kGraspRpy,
        kRedetectSamples,
        kRedetectInterval,
        0.85,     // max_reach
        0.025,    // approach_step_size
        0.01,     // approach_tolerance
        50,       // max_approach_steps
        3,        // max_consecutive_failures
        "panda_hand",
        {0.025, -0.015, 0.015},    // camera_offset
        {3.14159265359, 0.0, -1.57079632679},  // camera_rpy
        3.14159265359);            // optical_frame_pitch

    // 在后台线程执行抓取，主线程监控 Ctrl+C
    bool success = false;
    std::thread grasp_thread([&]() {
      success = task.run(30.0);
    });

    // 主线程：等待抓取完成或 Ctrl+C
    while (grasp_thread.joinable()) {
      if (!rclcpp::ok() || g_abort.load(std::memory_order_acquire)) {
        LOG_INFO("[SIGNAL] 收到中止信号，正在停止抓取...");
        task.request_abort();
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (grasp_thread.joinable()) {
      grasp_thread.join();
    }

    LOG_INFO("抓取结果: {}, 最终状态: {}",
             success ? "成功" : "失败",
             state_name(task.get_state()));
  }

cleanup:
  LOG_INFO("正在清理...");
  executor->cancel();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }
  rclcpp::shutdown();
  return 0;
}
