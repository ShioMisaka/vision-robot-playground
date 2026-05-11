/// @file demo_vision_diagnostic.cpp
/// @brief 视觉检测坐标变换诊断工具（不运动机器人）
///
/// 功能：
///   1. 连接 robot_controller_node 获取 TF
///   2. 通过视觉检测红色物块
///   3. 打印完整的坐标变换链
///   4. 与已知实际位置比较，找出误差来源
///
/// 用法：
///   source install/setup.bash
///   ros2 run robot_demos demo_vision_diagnostic
///
/// 可通过环境变量设置已知物块位置：
///   export CUBE_X=0.40921 CUBE_Y=0.0 CUBE_Z=0.24026

#include <array>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <robot_logger/logger.hpp>

#include "robot_api/robot_client.hpp"
#include "robot_tasks/grasp_task_manager.hpp"
#include "robot_vision/vision/color_detector.hpp"
#include "robot_vision/vision/vision_topic_config.hpp"
#include "robot_vision/nodes/vision_processor_node.hpp"
#include "robot_controller/kinematics/robot_profile.hpp"

#include <Eigen/Geometry>

using namespace robot_api;
using namespace robot_vision;
using namespace robot_tasks;
using robot_control::rpy_to_rotation;

// ---- HSV 检测参数（红色物块）----
const std::array<int, 3> kLowerHsv = {0, 100, 100};
const std::array<int, 3> kUpperHsv = {10, 255, 255};

// ---- 相机内参 ----
constexpr double kFx = 490.6666666666667;
constexpr double kFy = 490.6666666666667;
constexpr double kCx = 640.0;
constexpr double kCy = 360.0;

// ---- 相机外参（与 URDF 一致）----
const std::array<double, 3> kCameraOffset = {0.025, -0.015, 0.015};
const std::array<double, 3> kCameraRpy = {3.14159265359, 0.0, -1.57079632679};

// ---- 抓取参数（与 demo_vision_grasp 保持一致）----
constexpr double kGraspHeightOffset = -0.025;  // 从顶面到物块中心

// ---- 已知物块位置（环境变量或默认值）----
constexpr double kDefaultCubeX = 0.40921;
constexpr double kDefaultCubeY = 0.0;
constexpr double kDefaultCubeZ = 0.24026;
constexpr double kCubeSize = 0.05;  // 物块边长

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  // 读取已知物块位置
  double cube_x = std::getenv("CUBE_X") ? std::atof(std::getenv("CUBE_X")) : kDefaultCubeX;
  double cube_y = std::getenv("CUBE_Y") ? std::atof(std::getenv("CUBE_Y")) : kDefaultCubeY;
  double cube_z = std::getenv("CUBE_Z") ? std::atof(std::getenv("CUBE_Z")) : kDefaultCubeZ;

  Eigen::Vector3d cube_center(cube_x, cube_y, cube_z);
  Eigen::Vector3d cube_top(cube_x, cube_y, cube_z + kCubeSize / 2.0);

  LOG_INFO("========================================");
  LOG_INFO("视觉坐标变换诊断工具");
  LOG_INFO("========================================");
  LOG_INFO("已知物块中心 (base): [{:.5f}, {:.5f}, {:.5f}]",
           cube_center.x(), cube_center.y(), cube_center.z());
  LOG_INFO("物块顶面中心 (base): [{:.5f}, {:.5f}, {:.5f}]",
           cube_top.x(), cube_top.y(), cube_top.z());

  // ---- 创建节点 ----
  auto robot_client = RobotClient::create("robot_controller_node");
  auto detector = std::make_shared<ColorDetector>(
      kLowerHsv, kUpperHsv,
      kFx, kFy, kCx, kCy);
  robot_vision::VisionTopicConfig config;
  auto vision_node = VisionProcessorNode::create(detector, config);

  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(robot_client);
  executor->add_node(vision_node);
  std::thread spin_thread([executor]() { executor->spin(); });

  // ---- 等待服务就绪 ----
  LOG_INFO("等待 robot_controller_node 服务就绪...");
  bool ready = robot_client->wait_for_services(10.0);
  if (!ready) {
    LOG_ERROR("等待控制器服务超时");
    goto cleanup;
  }

  {
    auto ctrl = robot_client->get_controller();

    // ---- 打印当前 TCP 设置 ----
    auto pose = ctrl->get_end_effector_pose();
    LOG_INFO("");
    LOG_INFO("---- 当前机器人状态 ----");
    LOG_INFO("TCP pose (xyz): [{:.5f}, {:.5f}, {:.5f}]",
             pose[0], pose[1], pose[2]);
    LOG_INFO("TCP pose (rpy): [{:.5f}, {:.5f}, {:.5f}]",
             pose[3], pose[4], pose[5]);

    // ---- 等待视觉检测 ----
    LOG_INFO("");
    LOG_INFO("---- 等待视觉检测（10s 超时）----");
    auto result = vision_node->wait_for_detection(10.0);
    if (!result.has_value() || !result->detected) {
      LOG_ERROR("视觉检测失败，请确认红色物块在相机视野内");
      goto cleanup;
    }

    LOG_INFO("");
    LOG_INFO("==== 第 1 步: 视觉检测结果（camera optical frame）====");
    LOG_INFO("  像素坐标 uv = [{}, {}]", result->uv.x(), result->uv.y());
    LOG_INFO("  camera_xyz = [{:.5f}, {:.5f}, {:.5f}]",
             result->xyz.x(), result->xyz.y(), result->xyz.z());

    // 验证 pixel_to_3d 反投影一致性
    double verify_x = (result->uv.x() - kCx) * result->xyz.z() / kFx;
    double verify_y = (result->uv.y() - kCy) * result->xyz.z() / kFy;
    LOG_INFO("  反投影验证: x={:.5f} (实际{:.5f} 误差{:.5f}) y={:.5f} (实际{:.5f} 误差{:.5f})",
             verify_x, result->xyz.x(), std::abs(verify_x - result->xyz.x()),
             verify_y, result->xyz.y(), std::abs(verify_y - result->xyz.y()));

    // ---- 手动执行坐标变换 ----
    LOG_INFO("");
    LOG_INFO("==== 第 2 步: TF 查询 (base → panda_hand) ====");

    // 多次采样 TF 取平均（减少抖动）
    constexpr int kTfSamples = 5;
    Eigen::Vector3d avg_t_bh = Eigen::Vector3d::Zero();
    Eigen::Vector3d avg_rpy_bh = Eigen::Vector3d::Zero();
    for (int i = 0; i < kTfSamples; ++i) {
      auto tf = ctrl->lookup_transform("panda_link0", "panda_hand", 1.0);
      if (!tf.has_value()) {
        LOG_ERROR("TF lookup failed (sample {})", i);
        goto cleanup;
      }
      avg_t_bh += Eigen::Vector3d((*tf)[0], (*tf)[1], (*tf)[2]);
      avg_rpy_bh += Eigen::Vector3d((*tf)[3], (*tf)[4], (*tf)[5]);
      if (i < kTfSamples - 1) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
    }
    avg_t_bh /= kTfSamples;
    avg_rpy_bh /= kTfSamples;

    LOG_INFO("  hand 平均线速度 t = [{:.5f}, {:.5f}, {:.5f}]",
             avg_t_bh.x(), avg_t_bh.y(), avg_t_bh.z());
    LOG_INFO("  hand 平均 RPY = [{:.5f}, {:.5f}, {:.5f}]",
             avg_rpy_bh.x(), avg_rpy_bh.y(), avg_rpy_bh.z());

    // 也打印 TF 发布的 camera → base 变换作为参考
    LOG_INFO("");
    LOG_INFO("  [参考] TF 发布的 camera_color_optical_frame → base:");
    auto tf_optical = ctrl->lookup_transform("panda_link0", "camera_color_optical_frame", 1.0);
    if (tf_optical.has_value()) {
      LOG_INFO("    t = [{:.5f}, {:.5f}, {:.5f}]",
               (*tf_optical)[0], (*tf_optical)[1], (*tf_optical)[2]);
      LOG_INFO("    rpy = [{:.5f}, {:.5f}, {:.5f}]",
               (*tf_optical)[3], (*tf_optical)[4], (*tf_optical)[5]);
    } else {
      LOG_WARN("    TF 中未找到 camera_color_optical_frame → panda_link0 变换");
    }

    LOG_INFO("");
    LOG_INFO("==== 第 3 步: 手动坐标变换 ====");
    LOG_INFO("  相机外参 (hand→camera_link):");
    LOG_INFO("    offset = [{:.5f}, {:.5f}, {:.5f}]",
             kCameraOffset[0], kCameraOffset[1], kCameraOffset[2]);
    LOG_INFO("    rpy = [{:.5f}, {:.5f}, {:.5f}]",
             kCameraRpy[0], kCameraRpy[1], kCameraRpy[2]);

    // base ← hand
    Eigen::Matrix3d R_bh = rpy_to_rotation(avg_rpy_bh.x(), avg_rpy_bh.y(), avg_rpy_bh.z());
    Eigen::Vector3d t_bh = avg_t_bh;

    // hand ← camera_link
    Eigen::Matrix3d R_hc = rpy_to_rotation(kCameraRpy[0], kCameraRpy[1], kCameraRpy[2]);
    Eigen::Vector3d t_hc(kCameraOffset[0], kCameraOffset[1], kCameraOffset[2]);

    // camera_link ← optical (USD 相机 → ROS 光学坐标系: Ry(π))
    Eigen::Matrix3d R_co = rpy_to_rotation(0.0, 3.14159265359, 0.0);

    // 合成: base ← optical
    Eigen::Matrix3d R = R_bh * R_hc * R_co;
    Eigen::Vector3d t = R_bh * t_hc + t_bh;

    LOG_INFO("");
    LOG_INFO("  合成旋转矩阵 R 列向量（optical 各轴在 base 系中的方向）:");
    LOG_INFO("    X_opt→base = [{:.4f}, {:.4f}, {:.4f}]", R(0,0), R(1,0), R(2,0));
    LOG_INFO("    Y_opt→base = [{:.4f}, {:.4f}, {:.4f}]", R(0,1), R(1,1), R(2,1));
    LOG_INFO("    Z_opt→base = [{:.4f}, {:.4f}, {:.4f}]", R(0,2), R(1,2), R(2,2));
    LOG_INFO("  optical 原点在 base 中 (t) = [{:.5f}, {:.5f}, {:.5f}]",
             t.x(), t.y(), t.z());

    // 变换
    Eigen::Vector3d base_point = R * result->xyz + t;

    LOG_INFO("");
    LOG_INFO("==== 第 4 步: 变换结果 ====");
    LOG_INFO("  检测点（物块顶面）在 base 中 = [{:.5f}, {:.5f}, {:.5f}]",
             base_point.x(), base_point.y(), base_point.z());

    // 使用 GraspTaskManager 的 transform_to_base 做对比
    GraspTaskManager task(
        ctrl, vision_node,
        "panda_link0",
        "camera_color_optical_frame",
        0.15,          // approach_height
        kGraspHeightOffset,
        {M_PI, 0.0, M_PI},  // grasp_rpy
        5, 0.1,        // redetect
        0.85,          // max_reach
        0.025, 0.01, 50, 3,  // approach params
        "panda_hand",
        kCameraOffset,
        kCameraRpy,
        3.14159265359);  // optical_frame_pitch

    // 通过 step_detect 使用相同的 transform_to_base
    auto mgr_base = task.transform_to_base(result->xyz, true);
    if (mgr_base.has_value()) {
      Eigen::Vector3d mgr_point((*mgr_base)[0], (*mgr_base)[1], (*mgr_base)[2]);
      LOG_INFO("  GraspTaskManager.transform_to_base 结果 = [{:.5f}, {:.5f}, {:.5f}]",
               mgr_point.x(), mgr_point.y(), mgr_point.z());
      LOG_INFO("  手动计算与 GraspTaskManager 差异 = [{:.5f}, {:.5f}, {:.5f}], norm = {:.6f}",
               (base_point - mgr_point).x(),
               (base_point - mgr_point).y(),
               (base_point - mgr_point).z(),
               (base_point - mgr_point).norm());
    }

    LOG_INFO("");
    LOG_INFO("==== 第 5 步: 与已知位置对比 ====");

    // 顶面检测点 vs 已知顶面中心
    Eigen::Vector3d top_error = base_point - cube_top;
    LOG_INFO("  检测点 vs 已知顶面中心:");
    LOG_INFO("    已知顶面 = [{:.5f}, {:.5f}, {:.5f}]",
             cube_top.x(), cube_top.y(), cube_top.z());
    LOG_INFO("    检测结果 = [{:.5f}, {:.5f}, {:.5f}]",
             base_point.x(), base_point.y(), base_point.z());
    LOG_INFO("    误差 (检测 - 实际) = [{:.5f}, {:.5f}, {:.5f}], norm = {:.5f} m",
             top_error.x(), top_error.y(), top_error.z(), top_error.norm());

    // 加上抓取补偿后 vs 已知物块中心
    Eigen::Vector3d grasp_target = base_point;
    grasp_target[2] += kGraspHeightOffset;
    Eigen::Vector3d center_error = grasp_target - cube_center;
    LOG_INFO("");
    LOG_INFO("  加抓取补偿后 vs 已知物块中心:");
    LOG_INFO("    抓取目标 = [{:.5f}, {:.5f}, {:.5f}]",
             grasp_target.x(), grasp_target.y(), grasp_target.z());
    LOG_INFO("    已知中心 = [{:.5f}, {:.5f}, {:.5f}]",
             cube_center.x(), cube_center.y(), cube_center.z());
    LOG_INFO("    误差 (目标 - 实际) = [{:.5f}, {:.5f}, {:.5f}], norm = {:.5f} m",
             center_error.x(), center_error.y(), center_error.z(), center_error.norm());

    // 分轴误差分析
    LOG_INFO("");
    LOG_INFO("==== 第 6 步: 分轴误差分析 ====");
    LOG_INFO("  X 误差: {:.5f} m ({:+.1f} mm)", center_error.x(), center_error.x() * 1000);
    LOG_INFO("  Y 误差: {:.5f} m ({:+.1f} mm)", center_error.y(), center_error.y() * 1000);
    LOG_INFO("  Z 误差: {:.5f} m ({:+.1f} mm)", center_error.z(), center_error.z() * 1000);

    // TCP 偏移影响分析
    LOG_INFO("");
    LOG_INFO("==== 第 7 步: TCP 偏移影响分析 ====");
    LOG_INFO("  当前 TCP: grasptarget = hand + [0, 0, 0.105]");
    LOG_INFO("  抓取目标已含 grasp_height_offset = {:.3f} m", kGraspHeightOffset);
    LOG_INFO("  物块高度 = {:.3f} m, 半高 = {:.3f} m", kCubeSize, kCubeSize / 2.0);
    LOG_INFO("");
    LOG_INFO("  如果使用 hand TCP (不切换 grasptarget):");
    LOG_INFO("    则 hand 会移到抓取目标位, 夹爪指尖偏下 0.105m");
    LOG_INFO("    实际指尖位置 = [{:.5f}, {:.5f}, {:.5f}]",
             grasp_target.x(), grasp_target.y(), grasp_target.z());
    LOG_INFO("");
    LOG_INFO("  如果使用 grasptarget TCP:");
    LOG_INFO("    则 grasstarget (指尖) 会移到抓取目标位");
    LOG_INFO("    hand 位置 = 指尖位置 - [0, 0, 0.105] (hand Z 方向)");
    LOG_INFO("    hand Z ≈ {:.5f}", grasp_target.z() - 0.105);

    // 重复检测多次以评估稳定性
    LOG_INFO("");
    LOG_INFO("==== 第 8 步: 多次检测稳定性 (5次) ====");
    for (int i = 0; i < 5; ++i) {
      auto r = vision_node->wait_for_detection(2.0);
      if (!r.has_value() || !r->detected) {
        LOG_WARN("  检测 {} 失败", i + 1);
        continue;
      }
      auto b = task.transform_to_base(r->xyz, false);
      if (!b.has_value()) {
        LOG_WARN("  检测 {} 变换失败", i + 1);
        continue;
      }
      LOG_INFO("  检测 {}: camera=[{:.4f},{:.4f},{:.4f}] → base=[{:.5f},{:.5f},{:.5f}]",
               i + 1,
               r->xyz.x(), r->xyz.y(), r->xyz.z(),
               (*b)[0], (*b)[1], (*b)[2]);
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    LOG_INFO("");
    LOG_INFO("========================================");
    LOG_INFO("诊断完成");
    LOG_INFO("========================================");
  }

cleanup:
  executor->cancel();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }
  rclcpp::shutdown();
  return 0;
}
