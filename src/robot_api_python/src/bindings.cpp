/// @file bindings.cpp
/// @brief pybind11 bindings for robot controller library

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "robot_description/camera_config.hpp"

#include <rclcpp/rclcpp.hpp>
#include <robot_logger/logger.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include "robot_controller/motion/control_constants.hpp"
#include "robot_controller/nodes/topic_config.hpp"
#include "robot_controller/kinematics/robot_profile.hpp"
#include "robot_controller/kinematics/ik_solver.hpp"
#include "robot_vision/vision/color_detector.hpp"
#include "robot_controller/motion/i_robot_controller.hpp"
#include "robot_vision/vision/i_vision_processor.hpp"
#include "robot_controller/motion/robot_motion_controller.hpp"
#include "robot_vision/nodes/grasp_task_manager.hpp"
#include "robot_controller/nodes/robot_controller_node.hpp"
#include "robot_vision/nodes/vision_processor_node.hpp"
#include "robot_vision/vision/vision_topic_config.hpp"
#include "robot_controller/kinematics/profile_loader.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "robot_api/service_robot_controller.hpp"
#include "robot_api/robot_client.hpp"

#include <Eigen/Core>
#include <array>
#include <cmath>
#include <optional>
#include <string>

namespace py = pybind11;
using namespace robot_control;
using namespace robot_vision;

// ============================================================
// Eigen → Python 类型转换辅助
// ============================================================

/// 将 Eigen::Vector3d 转换为 Python list[float]
static py::list vec3d_to_list(const Eigen::Vector3d& v) {
  py::list l;
  l.append(v.x());
  l.append(v.y());
  l.append(v.z());
  return l;
}

/// 将 Eigen::Vector2i 转换为 Python list[int]
static py::list vec2i_to_list(const Eigen::Vector2i& v) {
  py::list l;
  l.append(v.x());
  l.append(v.y());
  return l;
}

/// 将 Eigen::Matrix4d 转换为 Python 嵌套 list[list[float]]
static py::list mat4d_to_nested_list(const Eigen::Matrix4d& m) {
  py::list rows;
  for (int i = 0; i < 4; ++i) {
    py::list row;
    for (int j = 0; j < 4; ++j) {
      row.append(m(i, j));
    }
    rows.append(row);
  }
  return rows;
}

// ============================================================
// 模块定义
// ============================================================

PYBIND11_MODULE(_core, m) {
  m.doc() = "Python bindings for robot controller";

  // ===== ROS2 生命周期 =====
  m.def("rclcpp_init", []() {
    rclcpp::init(0, nullptr);
  }, "Initialize rclcpp (replaces rclpy.init)");
  m.def("rclcpp_shutdown", []() {
    rclcpp::shutdown(rclcpp::contexts::get_global_default_context(), "shutdown");
  }, "Shutdown rclcpp (replaces rclpy.shutdown)");

  // robot_logger Python submodule
  py::module_ logger_mod = m.def_submodule("robot_logger");
  logger_mod.def("info", [](const std::string& msg) {
      LOG_INFO("{}", msg);
  });
  logger_mod.def("info", [](const std::string& module, const std::string& msg) {
      ::robot_logger::LoggerManager::instance().get(module)->info("{}", msg);
  });
  logger_mod.def("debug", [](const std::string& msg) {
      LOG_DEBUG("{}", msg);
  });
  logger_mod.def("debug", [](const std::string& module, const std::string& msg) {
      ::robot_logger::LoggerManager::instance().get(module)->debug("{}", msg);
  });
  logger_mod.def("warn", [](const std::string& msg) {
      LOG_WARN("{}", msg);
  });
  logger_mod.def("warn", [](const std::string& module, const std::string& msg) {
      ::robot_logger::LoggerManager::instance().get(module)->warn("{}", msg);
  });
  logger_mod.def("error", [](const std::string& msg) {
      LOG_ERROR("{}", msg);
  });
  logger_mod.def("error", [](const std::string& module, const std::string& msg) {
      ::robot_logger::LoggerManager::instance().get(module)->error("{}", msg);
  });
  logger_mod.def("critical", [](const std::string& msg) {
      LOG_CRITICAL("{}", msg);
  });
  logger_mod.def("critical", [](const std::string& module, const std::string& msg) {
      ::robot_logger::LoggerManager::instance().get(module)->critical("{}", msg);
  });
  logger_mod.def("set_level", [](const std::string& level) {
      ::robot_logger::LoggerManager::instance().set_global_level(
          spdlog::level::from_str(level));
  });
  logger_mod.def("set_level", [](const std::string& module, const std::string& level) {
      ::robot_logger::LoggerManager::instance().set_level(
          module, spdlog::level::from_str(level));
  });
  logger_mod.def("flush", []() {
      ::robot_logger::LoggerManager::instance().flush();
  });

  // ===== rclcpp::Logger =====
  py::class_<rclcpp::Logger>(m, "Logger")
      .def("info", [](rclcpp::Logger& /*log*/, const std::string& msg) {
          LOG_INFO("{}", msg);
      })
      .def("warn", [](rclcpp::Logger& /*log*/, const std::string& msg) {
          LOG_WARN("{}", msg);
      })
      .def("error", [](rclcpp::Logger& /*log*/, const std::string& msg) {
          LOG_ERROR("{}", msg);
      });

  // ===== rclcpp::Node 基类 =====
  py::class_<rclcpp::Node, std::shared_ptr<rclcpp::Node>>(m, "_NodeBase");

  // ===== MultiThreadedExecutor =====
  py::class_<rclcpp::executors::MultiThreadedExecutor,
             std::shared_ptr<rclcpp::executors::MultiThreadedExecutor>>(
      m, "MultiThreadedExecutor")
      .def(py::init<>())
      .def("add_node",
           [](rclcpp::executors::MultiThreadedExecutor& exec,
              rclcpp::Node::SharedPtr node) { exec.add_node(node); },
           py::arg("node"))
      .def("spin", &rclcpp::executors::MultiThreadedExecutor::spin,
           py::call_guard<py::gil_scoped_release>(),
           "Block and process callbacks (releases GIL)")
      .def("cancel", &rclcpp::executors::MultiThreadedExecutor::cancel);

  // ===== 数据类型 =====

  // TcpConfig
  py::class_<TcpConfig>(m, "TcpConfig")
      .def(py::init<>())
      .def_readwrite("offset_xyz", &TcpConfig::offset_xyz)
      .def_readwrite("offset_rpy", &TcpConfig::offset_rpy);

  // TopicConfig
  py::class_<TopicConfig>(m, "TopicConfig")
      .def(py::init<>())
      .def_readwrite("joint_command", &TopicConfig::joint_command)
      .def_readwrite("joint_state", &TopicConfig::joint_state);

  // RobotProfile
  py::class_<RobotProfile>(m, "RobotProfile")
      .def(py::init<>())
      .def_readwrite("name", &RobotProfile::name)
      .def_readwrite("urdf_path", &RobotProfile::urdf_path)
      .def_readwrite("dof", &RobotProfile::dof)
      .def_readwrite("joint_names", &RobotProfile::joint_names)
      .def_readwrite("all_joint_names", &RobotProfile::all_joint_names)
      .def_readwrite("joint_limits_lower", &RobotProfile::joint_limits_lower)
      .def_readwrite("joint_limits_upper", &RobotProfile::joint_limits_upper)
      .def_readwrite("home_joints", &RobotProfile::home_joints)
      .def_readwrite("ik_default_guess", &RobotProfile::ik_default_guess)
      .def_readwrite("base_frame", &RobotProfile::base_frame)
      .def_readwrite("hand_frame", &RobotProfile::hand_frame)
      .def_readwrite("tcp_frames", &RobotProfile::tcp_frames)
      .def_readwrite("default_tcp", &RobotProfile::default_tcp)
      .def_readwrite("joint_limits", &RobotProfile::joint_limits)
      .def_readwrite("cartesian_limits", &RobotProfile::cartesian_limits);

  // GripperProfile
  py::class_<GripperProfile>(m, "GripperProfile")
      .def(py::init<>())
      .def_readwrite("type", &GripperProfile::type)
      .def_readwrite("min_width", &GripperProfile::min_width)
      .def_readwrite("max_width", &GripperProfile::max_width)
      .def_readwrite("dof", &GripperProfile::dof);

  // MotionLimits
  py::class_<MotionLimits>(m, "MotionLimits")
      .def(py::init<>())
      .def_readwrite("max_vel", &MotionLimits::max_vel)
      .def_readwrite("max_acc", &MotionLimits::max_acc)
      .def_readwrite("max_jerk", &MotionLimits::max_jerk);

  // DetectionResult（Eigen 类型转换为 Python 原生类型）
  py::class_<DetectionResult>(m, "DetectionResult")
      .def(py::init<>())
      .def_readwrite("detected", &DetectionResult::detected)
      .def_property_readonly("xyz",
                             [](const DetectionResult& r) {
                               return vec3d_to_list(r.xyz);
                             })
      .def_property_readonly("uv",
                             [](const DetectionResult& r) {
                               return vec2i_to_list(r.uv);
                             })
      .def_readwrite("confidence", &DetectionResult::confidence)
      .def_readwrite("label", &DetectionResult::label);

  // GraspState 枚举
  py::enum_<GraspState>(m, "GraspState")
      .value("IDLE", GraspState::kIdle)
      .value("DETECTING", GraspState::kDetecting)
      .value("APPROACHING", GraspState::kApproaching)
      .value("RE_DETECTING", GraspState::kReDetecting)
      .value("DESCENDING", GraspState::kDescending)
      .value("GRASPING", GraspState::kGrasping)
      .value("LIFTING", GraspState::kLifting)
      .value("DONE", GraspState::kDone)
      .value("ERROR", GraspState::kError)
      .export_values();

  // MotionMode 枚举
  py::enum_<MotionMode>(m, "MotionMode")
      .value("MOVE_J", MotionMode::kMoveJ)
      .value("MOVE_L", MotionMode::kMoveL)
      .export_values();

  // VisionTopicConfig
  py::class_<VisionTopicConfig>(m, "VisionTopicConfig")
      .def(py::init<>())
      .def_readwrite("camera_left", &VisionTopicConfig::camera_left)
      .def_readwrite("camera_depth", &VisionTopicConfig::camera_depth)
      .def_readwrite("sync_queue_size", &VisionTopicConfig::sync_queue_size)
      .def_readwrite("sync_max_slop", &VisionTopicConfig::sync_max_slop);

  // ControlConstants — 导出为模块级常量
  m.attr("JOINT_TOLERANCE") = ControlConstants::kJointTolerance;
  m.attr("FINGER_TOLERANCE") = ControlConstants::kFingerTolerance;
  m.attr("MOTION_TIMEOUT") = ControlConstants::kMotionTimeout;
  m.attr("POLL_INTERVAL") = ControlConstants::kPollInterval;
  m.attr("SETTLE_TIME") = ControlConstants::kSettleTime;
  m.attr("DEFAULT_STEPS") = ControlConstants::kDefaultSteps;
  m.attr("DEFAULT_STEP_TIME") = ControlConstants::kDefaultStepTime;
  m.attr("FINGER_STABLE_COUNT") = ControlConstants::kFingerStableCount;
  m.attr("FINGER_STABLE_TOL") = ControlConstants::kFingerStableTol;
  m.attr("READY_TIMEOUT") = ControlConstants::kReadyTimeout;
  m.attr("TRAJECTORY_DT") = ControlConstants::kTrajectoryDt;
  m.attr("CONTROL_LOOP_HZ") = ControlConstants::kControlLoopHz;

  // ===== Layer 1 核心类 =====

  // IKSolver
  py::class_<IKSolver, std::shared_ptr<IKSolver>>(m, "IKSolver")
      .def(py::init<const RobotProfile&>(), py::arg("profile"))
      .def("solve", &IKSolver::solve,
           py::arg("xyz"), py::arg("rpy") = py::none(),
           py::call_guard<py::gil_scoped_release>())
      .def("forward", &IKSolver::forward, py::arg("joint_angles"))
      .def("forward_matrix",
           [](const IKSolver& self,
              const std::vector<double>& joint_angles) {
             return mat4d_to_nested_list(self.forward_matrix(joint_angles));
           },
           py::arg("joint_angles"))
      .def("get_dof", &IKSolver::get_dof)
      .def("velocity_ik", &IKSolver::velocity_ik,
           py::arg("current_joints"), py::arg("cartesian_delta"),
           py::call_guard<py::gil_scoped_release>());

  // CameraInterface（不可实例化基类）
  py::class_<CameraInterface, std::shared_ptr<CameraInterface>>(
      m, "CameraInterface");

  // ColorDetector
  py::class_<ColorDetector, CameraInterface,
             std::shared_ptr<ColorDetector>>(m, "ColorDetector")
      .def(py::init<const std::array<int, 3>&, const std::array<int, 3>&>(),
           py::arg("lower_hsv"), py::arg("upper_hsv"))
      .def("set_camera_intrinsics",
           &ColorDetector::set_camera_intrinsics,
           py::arg("fx"), py::arg("fy"), py::arg("cx"), py::arg("cy"));

  // pixel_to_3d 静态方法（返回值转换为 Python list）
  m.def("pixel_to_3d",
        [](int u, int v, double depth, double fx, double fy,
           double cx, double cy) -> py::list {
          return vec3d_to_list(
              CameraInterface::pixel_to_3d(u, v, depth, fx, fy, cx, cy));
        },
        py::arg("u"), py::arg("v"), py::arg("depth"),
        py::arg("fx"), py::arg("fy"), py::arg("cx"), py::arg("cy"));

  // IRobotController（不可实例化基类）
  py::class_<IRobotController, std::shared_ptr<IRobotController>>(
      m, "IRobotController");

  // RobotMotionController
  // 注意：用户不直接构造此类，而是通过 RobotControllerNode::get_controller() 获取
  py::class_<RobotMotionController, IRobotController,
             std::shared_ptr<RobotMotionController>>(
      m, "RobotMotionController")
      .def("set_arm", &RobotMotionController::set_arm,
           py::arg("angles"), py::arg("block") = true,
           py::call_guard<py::gil_scoped_release>())
      .def("set_gripper", &RobotMotionController::set_gripper,
           py::arg("width"), py::arg("block") = true,
           py::call_guard<py::gil_scoped_release>())
      .def("open_gripper", &RobotMotionController::open_gripper,
           py::arg("block") = true,
           py::call_guard<py::gil_scoped_release>())
      .def("close_gripper", &RobotMotionController::close_gripper,
           py::arg("block") = true,
           py::call_guard<py::gil_scoped_release>())
      .def("move_to_pose", &RobotMotionController::move_to_pose,
           py::arg("xyz"), py::arg("rpy"), py::arg("finger") = -1.0,
           py::arg("steps") = 0, py::arg("step_time") = 0.08,
           py::arg("block") = true,
           py::call_guard<py::gil_scoped_release>())
      .def("move_linear", &RobotMotionController::move_linear,
           py::arg("delta"), py::arg("frame") = std::string("base"),
           py::arg("finger") = -1.0, py::arg("block") = true,
           py::call_guard<py::gil_scoped_release>())
      .def("rotate_joint", &RobotMotionController::rotate_joint,
           py::arg("index"), py::arg("delta_angle"), py::arg("block") = true,
           py::call_guard<py::gil_scoped_release>())
      .def("go_home", &RobotMotionController::go_home,
           py::arg("block") = true,
           py::call_guard<py::gil_scoped_release>())
      .def("get_joint_angles", &RobotMotionController::get_joint_angles)
      .def("get_end_effector_pose",
           &RobotMotionController::get_end_effector_pose)
      .def("get_finger_width", &RobotMotionController::get_finger_width)
      .def("set_tcp", &RobotMotionController::set_tcp, py::arg("name"))
      .def("get_current_tcp", &RobotMotionController::get_current_tcp)
      .def("lookup_transform", &RobotMotionController::lookup_transform,
           py::arg("target_frame"), py::arg("source_frame"),
           py::arg("timeout") = 1.0,
           py::call_guard<py::gil_scoped_release>())
      .def("moveJ", static_cast<void (RobotMotionController::*)(
                         const std::vector<double>&, bool)>(
                         &RobotMotionController::moveJ),
           py::arg("target_angles"), py::arg("block") = true,
           py::call_guard<py::gil_scoped_release>())
      .def("moveJ", [](RobotMotionController& self,
                         const std::array<double, 3>& xyz,
                         const std::optional<std::array<double, 3>>& rpy,
                         double finger, bool block) {
             py::gil_scoped_release release;
             self.moveJ(xyz, rpy, finger, block);
           },
           py::arg("xyz"), py::arg("rpy") = py::none(),
           py::arg("finger") = -1.0, py::arg("block") = true)
      .def("moveL", [](RobotMotionController& self,
                         const std::array<double, 3>& xyz,
                         const std::optional<std::array<double, 3>>& rpy,
                         double finger, bool block) {
             py::gil_scoped_release release;
             self.moveL(xyz, rpy, finger, block);
           },
           py::arg("xyz"), py::arg("rpy") = py::none(),
           py::arg("finger") = -1.0, py::arg("block") = true)
      .def("set_speed", &RobotMotionController::set_speed,
           py::arg("mode"), py::arg("percent"))
      .def("get_speed", &RobotMotionController::get_speed,
           py::arg("mode"));

  // IVisionProcessor（不可实例化基类）
  py::class_<IVisionProcessor, std::shared_ptr<IVisionProcessor>>(
      m, "IVisionProcessor");

  // GraspTaskManager
  py::class_<GraspTaskManager, std::shared_ptr<GraspTaskManager>>(
      m, "GraspTaskManager")
      .def(py::init<std::shared_ptr<IRobotController>,
                    std::shared_ptr<IVisionProcessor>,
                    const std::string&, const std::string&,
                    double, double,
                    const std::array<double, 3>&,
                    int, double,
                    double, double, double, int, int,
                    const std::string&,
                    const std::array<double, 3>&,
                    const std::array<double, 3>&>(),
           py::arg("robot"), py::arg("vision"),
           py::arg("base_frame") = "panda_link0",
           py::arg("camera_frame") = "camera_color_optical_frame",
           py::arg("approach_height") = 0.15,
           py::arg("grasp_height_offset") = 0.02,
           py::arg("grasp_rpy") =
               std::array<double, 3>{M_PI, 0.0, M_PI},
           py::arg("redetect_samples") = 5,
           py::arg("redetect_interval") = 0.1,
           py::arg("max_reach") = 0.85,
           py::arg("approach_step_size") = 0.05,
           py::arg("approach_tolerance") = 0.01,
           py::arg("max_approach_steps") = 100,
           py::arg("max_consecutive_failures") = 3,
           py::arg("hand_frame") = "panda_hand",
           py::arg("camera_offset") =
               std::array<double, 3>{0.0, 0.0, 0.0},
           py::arg("camera_rpy") =
               std::array<double, 3>{
                   robot_description::CameraExtrinsics::kRoll,
                   robot_description::CameraExtrinsics::kPitch,
                   robot_description::CameraExtrinsics::kYaw})
      .def("run", &GraspTaskManager::run,
           py::arg("timeout") = 30.0,
           py::call_guard<py::gil_scoped_release>())
      .def("get_state", &GraspTaskManager::get_state)
      .def("transform_to_base",
           [](GraspTaskManager& self,
              const std::vector<double>& xyz) -> std::optional<std::array<double, 3>> {
             return self.transform_to_base(Eigen::Vector3d(xyz[0], xyz[1], xyz[2]));
           },
           py::arg("camera_xyz"),
           py::call_guard<py::gil_scoped_release>());

  // ===== Layer 2 ROS2 节点 =====

  // ServiceRobotController (IRobotController via ROS2 Service calls)
  py::class_<robot_api::ServiceRobotController, IRobotController,
             std::shared_ptr<robot_api::ServiceRobotController>>(
      m, "ServiceRobotController")
      .def("set_arm", &robot_api::ServiceRobotController::set_arm,
           py::arg("angles"), py::arg("block") = true,
           py::call_guard<py::gil_scoped_release>())
      .def("set_gripper", &robot_api::ServiceRobotController::set_gripper,
           py::arg("width"), py::arg("block") = true,
           py::call_guard<py::gil_scoped_release>())
      .def("open_gripper", &robot_api::ServiceRobotController::open_gripper,
           py::arg("block") = true,
           py::call_guard<py::gil_scoped_release>())
      .def("close_gripper", &robot_api::ServiceRobotController::close_gripper,
           py::arg("block") = true,
           py::call_guard<py::gil_scoped_release>())
      .def("move_to_pose", &robot_api::ServiceRobotController::move_to_pose,
           py::arg("xyz"), py::arg("rpy"), py::arg("finger") = -1.0,
           py::arg("steps") = 0, py::arg("step_time") = 0.08,
           py::arg("block") = true,
           py::call_guard<py::gil_scoped_release>())
      .def("move_linear", &robot_api::ServiceRobotController::move_linear,
           py::arg("delta"), py::arg("frame") = std::string("base"),
           py::arg("finger") = -1.0, py::arg("block") = true,
           py::call_guard<py::gil_scoped_release>())
      .def("go_home", &robot_api::ServiceRobotController::go_home,
           py::arg("block") = true,
           py::call_guard<py::gil_scoped_release>())
      .def("get_joint_angles", &robot_api::ServiceRobotController::get_joint_angles,
           py::call_guard<py::gil_scoped_release>())
      .def("get_end_effector_pose",
           &robot_api::ServiceRobotController::get_end_effector_pose,
           py::call_guard<py::gil_scoped_release>())
      .def("get_finger_width", &robot_api::ServiceRobotController::get_finger_width,
           py::call_guard<py::gil_scoped_release>())
      .def("set_tcp", &robot_api::ServiceRobotController::set_tcp,
           py::arg("name"))
      .def("get_current_tcp", &robot_api::ServiceRobotController::get_current_tcp,
           py::call_guard<py::gil_scoped_release>())
      .def("lookup_transform",
           &robot_api::ServiceRobotController::lookup_transform,
           py::arg("target_frame"), py::arg("source_frame"),
           py::arg("timeout") = 1.0,
           py::call_guard<py::gil_scoped_release>())
      .def("moveJ", static_cast<void (robot_api::ServiceRobotController::*)(
                         const std::vector<double>&, bool)>(
                         &robot_api::ServiceRobotController::moveJ),
           py::arg("target_angles"), py::arg("block") = true,
           py::call_guard<py::gil_scoped_release>())
      .def("moveJ", [](robot_api::ServiceRobotController& self,
                          const std::array<double, 3>& xyz,
                          const std::optional<std::array<double, 3>>& rpy,
                          double finger, bool block) {
             py::gil_scoped_release release;
             self.moveJ(xyz, rpy, finger, block);
           },
           py::arg("xyz"), py::arg("rpy") = py::none(),
           py::arg("finger") = -1.0, py::arg("block") = true)
      .def("moveL", [](robot_api::ServiceRobotController& self,
                          const std::array<double, 3>& xyz,
                          const std::optional<std::array<double, 3>>& rpy,
                          double finger, bool block) {
             py::gil_scoped_release release;
             self.moveL(xyz, rpy, finger, block);
           },
           py::arg("xyz"), py::arg("rpy") = py::none(),
           py::arg("finger") = -1.0, py::arg("block") = true)
      .def("set_speed", &robot_api::ServiceRobotController::set_speed,
           py::arg("mode"), py::arg("percent"))
      .def("get_speed", &robot_api::ServiceRobotController::get_speed,
           py::arg("mode"));

  // RobotClient (lightweight service client node)
  py::class_<robot_api::RobotClient, rclcpp::Node,
             std::shared_ptr<robot_api::RobotClient>>(
      m, "RobotClient")
      .def_static("create", &robot_api::RobotClient::create,
                  py::arg("service_prefix") = std::string("robot_controller_node"))
      .def("wait_for_services", &robot_api::RobotClient::wait_for_services,
           py::arg("timeout") = 10.0,
           py::call_guard<py::gil_scoped_release>())
      .def("get_controller", &robot_api::RobotClient::get_controller)
      .def("get_logger",
           [](robot_api::RobotClient& self) -> rclcpp::Logger {
             return self.get_logger();
           });

  // RobotControllerNode (deprecated — kept for backward compatibility)
  py::class_<RobotControllerNode, rclcpp::Node,
             std::shared_ptr<RobotControllerNode>>(
      m, "RobotControllerNode")
      .def_static("create", &RobotControllerNode::create,
                  py::arg("profile"), py::arg("gripper"), py::arg("topics"))
      .def("wait_for_ready", &RobotControllerNode::wait_for_ready,
           py::arg("timeout") = 5.0,
           py::call_guard<py::gil_scoped_release>())
      .def("get_controller", &RobotControllerNode::get_controller)
      .def("get_logger",
           [](RobotControllerNode& self) -> rclcpp::Logger {
             return self.get_logger();
           });

  // VisionProcessorNode（多重继承：rclcpp::Node + IVisionProcessor）
  py::class_<VisionProcessorNode, rclcpp::Node, IVisionProcessor,
             std::shared_ptr<VisionProcessorNode>>(
      m, "VisionProcessorNode")
      .def_static("create", &VisionProcessorNode::create,
                  py::arg("processor"), py::arg("config"))
      .def("get_latest_result", &VisionProcessorNode::get_latest_result)
      .def("wait_for_detection", &VisionProcessorNode::wait_for_detection,
           py::arg("timeout") = 10.0,
           py::call_guard<py::gil_scoped_release>())
      .def("average_detections", &VisionProcessorNode::average_detections,
           py::arg("sample_count") = 5,
           py::arg("sample_interval") = 0.1,
           py::arg("timeout") = 5.0,
           py::call_guard<py::gil_scoped_release>())
      .def("get_logger",
           [](VisionProcessorNode& self) -> rclcpp::Logger {
             return self.get_logger();
           });

  // ===== Profile 加载 =====

  // RobotConfig
  py::class_<RobotConfig>(m, "RobotConfig")
      .def_readonly("robot", &RobotConfig::robot)
      .def_readonly("gripper", &RobotConfig::gripper);

  // 便捷函数：从 robot_description 包加载指定 profile
  m.def("load_profile", [](const std::string& profile_name) {
    const auto desc_dir =
        ament_index_cpp::get_package_share_directory("robot_description");
    return ProfileLoader::load(
        desc_dir + "/config/" + profile_name + "_profile.yaml", desc_dir);
  }, py::arg("profile_name") = "panda",
     "Load robot config from YAML. Defaults to 'panda'.");
}
