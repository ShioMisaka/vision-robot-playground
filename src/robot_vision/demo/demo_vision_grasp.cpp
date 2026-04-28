/// @file demo_vision_grasp.cpp
/// @brief C++ 版两阶段视觉引导抓取演示（含诊断日志 + Ctrl+C 处理）
///
/// 编译后运行（需要 Isaac Sim 已启动）：
///   source /opt/ros/jazzy/setup.bash
///   source install/setup.bash
///   ros2 run robot_vision demo_vision_grasp
///
/// 按 Ctrl+C 可安全中止，不会卡死。
///
/// 流程：
///   1. 张开夹爪，移动到观察位
///   2. GraspTaskManager.run() 自动执行两阶段抓取：
///      - kDetecting: 远距离粗检测
///      - kApproaching: moveJ 快速接近目标上方
///      - kReDetecting: 近距离多样本精确重检测
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

#include "robot_controller/nodes/robot_controller_node.hpp"
#include "robot_controller/profiles/panda_profile.hpp"
#include "robot_controller/motion/control_constants.hpp"
#include "robot_controller/motion/topic_config.hpp"

#include "robot_vision/vision/color_detector.hpp"
#include "robot_vision/nodes/vision_processor_node.hpp"
#include "robot_vision/nodes/grasp_task_manager.hpp"

using namespace robot_control;
using namespace robot_vision;

// ---- 全局中止标志 ----
static std::atomic<bool> g_abort{false};

// ---- HSV 检测参数（红色物块）----
const std::array<int, 3> kLowerHsv = {0, 100, 100};
const std::array<int, 3> kUpperHsv = {10, 255, 255};

// ---- 相机内参（需匹配 Isaac Sim 中 ZED 相机配置）----
// 实际图像为 1280x720，使用对应内参
const double kCameraFx = 700.0;
const double kCameraFy = 700.0;
const double kCameraCx = 640.0;
const double kCameraCy = 360.0;

// ---- 运动参数 ----
constexpr double kMoveJSpeed = 40.0;
constexpr double kMoveLSpeed = 10.0;

// ---- 抓取参数 ----
constexpr double kApproachHeight = 0.15;      // 目标上方 15cm
constexpr double kGraspHeightOffset = 0.02;   // 抓取高度偏移 2cm
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

  auto logger = rclcpp::get_logger("demo_vision_grasp");

  // ---- 创建节点 ----
  auto profile = profiles::panda();
  auto gripper = profiles::panda_gripper();
  TopicConfig topics;

  auto robot_node = RobotControllerNode::create(profile, gripper, topics);
  auto detector = std::make_shared<ColorDetector>(
      kLowerHsv, kUpperHsv, kCameraFx, kCameraFy, kCameraCx, kCameraCy);
  auto vision_node = VisionProcessorNode::create(detector, topics);

  // ---- 后台 Executor ----
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(robot_node);
  executor->add_node(vision_node);
  std::thread spin_thread([executor]() { executor->spin(); });

  // ---- 等待就绪 ----
  RCLCPP_INFO(logger, "等待与 Isaac Sim 建立连接...");

  bool ready = robot_node->wait_for_ready(ControlConstants::kReadyTimeout);
  if (!ready) {
    RCLCPP_ERROR(logger, "等待关节状态超时，请确认 Isaac Sim 已启动");
    goto cleanup;
  }

  if (!rclcpp::ok()) goto cleanup;

  {
    auto ctrl = robot_node->get_controller();

    // ---- 诊断日志：初始状态 ----
    {
      auto pose = ctrl->get_end_effector_pose();
      RCLCPP_INFO(logger,
                  "[DIAG] 初始 TCP: xyz=[%.4f, %.4f, %.4f] rpy=[%.4f, %.4f, %.4f]",
                  pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
    }

    // ---- 设置速度 ----
    ctrl->set_speed(MotionMode::kMoveJ, kMoveJSpeed);
    ctrl->set_speed(MotionMode::kMoveL, kMoveLSpeed);
    RCLCPP_INFO(logger, "[DIAG] 速度: moveJ=%.0f%%, moveL=%.0f%%",
                kMoveJSpeed, kMoveLSpeed);

    // ---- 不移动观察位 ----
    // 用户在 Isaac Sim 中已手动定位机器人，直接使用当前位置作为观察位。
    // 确保相机朝下（rpy Z 分量 ≈ -π）且物块在视野内即可。

    // ---- 张开夹爪 ----
    RCLCPP_INFO(logger, "张开夹爪");
    ctrl->open_gripper(true);

    // ---- 等待首次检测（诊断）----
    RCLCPP_INFO(logger, "[DIAG] 等待视觉检测（10s 超时）...");
    auto first_result = vision_node->wait_for_detection(10.0);
    if (!first_result.has_value() || !first_result->detected) {
      RCLCPP_ERROR(logger,
                   "[DIAG] 初始检测失败，请确认红色物块在相机视野内");
      goto cleanup;
    }
    RCLCPP_INFO(logger,
                "[DIAG] 首次检测: uv=[%d, %d], camera_xyz=[%.4f, %.4f, %.4f]",
                first_result->uv.x(), first_result->uv.y(),
                first_result->xyz.x(), first_result->xyz.y(),
                first_result->xyz.z());
    // 诊断：验证 uv 到 camera_xyz 的投影是否一致
    double verify_x = (first_result->uv.x() - kCameraCx) *
                      first_result->xyz.z() / kCameraFx;
    double verify_y = (first_result->uv.y() - kCameraCy) *
                      first_result->xyz.z() / kCameraFy;
    RCLCPP_INFO(logger,
                "[DIAG] 投影验证: 由uv反算 x=%.4f (实际%.4f) y=%.4f (实际%.4f) "
                "depth=%.4f",
                verify_x, first_result->xyz.x(),
                verify_y, first_result->xyz.y(),
                first_result->xyz.z());

    if (!rclcpp::ok()) goto cleanup;

    // ---- 执行两阶段抓取 ----
    RCLCPP_INFO(logger,
                "开始两阶段视觉引导抓取"
                "（粗检测 → moveJ接近 → 精确重检测 → moveL下降 → 抓取）");

    GraspTaskManager task(
        ctrl, vision_node,
        "panda_link0",
        "camera_color_optical_frame",
        kApproachHeight,
        kGraspHeightOffset,
        kGraspRpy,
        kRedetectSamples,
        kRedetectInterval);

    // 在后台线程执行抓取，主线程监控 Ctrl+C
    bool success = false;
    std::thread grasp_thread([&]() {
      success = task.run(30.0);
    });

    // 主线程：等待抓取完成或 Ctrl+C
    while (grasp_thread.joinable()) {
      if (!rclcpp::ok() || g_abort.load(std::memory_order_acquire)) {
        RCLCPP_INFO(logger, "[SIGNAL] 收到中止信号，正在停止抓取...");
        task.request_abort();
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (grasp_thread.joinable()) {
      grasp_thread.join();
    }

    RCLCPP_INFO(logger, "抓取结果: %s, 最终状态: %s",
                success ? "成功" : "失败",
                state_name(task.get_state()));
  }

cleanup:
  RCLCPP_INFO(logger, "正在清理...");
  executor->cancel();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }
  rclcpp::shutdown();
  return 0;
}
