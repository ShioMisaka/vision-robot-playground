/// @file bindings.cpp
/// @brief pybind11 bindings for robot_control_cpp library

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include "robot_control_cpp/config.hpp"
#include "robot_control_cpp/robot_profile.hpp"
#include "robot_control_cpp/ik_solver.hpp"
#include "robot_control_cpp/color_detector.hpp"
#include "robot_control_cpp/i_robot_controller.hpp"
#include "robot_control_cpp/i_vision_processor.hpp"
#include "robot_control_cpp/robot_motion_controller.hpp"
#include "robot_control_cpp/grasp_task_manager.hpp"
#include "robot_control_cpp/robot_controller_node.hpp"
#include "robot_control_cpp/vision_processor_node.hpp"
#include "robot_control_cpp/panda_profile.hpp"

#include <Eigen/Core>
#include <array>
#include <cmath>
#include <string>

namespace py = pybind11;
using namespace robot_control;

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
  m.doc() = "Python bindings for robot_control_cpp";

  // ===== ROS2 生命周期 =====
  m.def("rclcpp_init", []() {
    rclcpp::init(0, nullptr);
  }, "Initialize rclcpp (replaces rclpy.init)");
  m.def("rclcpp_shutdown", []() {
    rclcpp::shutdown(rclcpp::contexts::get_global_default_context(), "shutdown");
  }, "Shutdown rclcpp (replaces rclpy.shutdown)");

  // ===== rclcpp::Logger =====
  py::class_<rclcpp::Logger>(m, "Logger")
      .def("info", [](rclcpp::Logger& log, const std::string& msg) {
        RCLCPP_INFO(log, "%s", msg.c_str());
      })
      .def("warn", [](rclcpp::Logger& log, const std::string& msg) {
        RCLCPP_WARN(log, "%s", msg.c_str());
      })
      .def("error", [](rclcpp::Logger& log, const std::string& msg) {
        RCLCPP_ERROR(log, "%s", msg.c_str());
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
      .def_readwrite("joint_state", &TopicConfig::joint_state)
      .def_readwrite("camera_left", &TopicConfig::camera_left)
      .def_readwrite("camera_depth", &TopicConfig::camera_depth)
      .def_readwrite("camera_frame", &TopicConfig::camera_frame);

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
      .def_readwrite("default_tcp", &RobotProfile::default_tcp);

  // GripperProfile
  py::class_<GripperProfile>(m, "GripperProfile")
      .def(py::init<>())
      .def_readwrite("type", &GripperProfile::type)
      .def_readwrite("min_width", &GripperProfile::min_width)
      .def_readwrite("max_width", &GripperProfile::max_width)
      .def_readwrite("dof", &GripperProfile::dof);

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
      .def_readwrite("confidence", &DetectionResult::confidence);

  // GraspState 枚举
  py::enum_<GraspState>(m, "GraspState")
      .value("IDLE", GraspState::kIdle)
      .value("DETECTING", GraspState::kDetecting)
      .value("APPROACHING", GraspState::kApproaching)
      .value("DESCENDING", GraspState::kDescending)
      .value("GRASPING", GraspState::kGrasping)
      .value("LIFTING", GraspState::kLifting)
      .value("DONE", GraspState::kDone)
      .value("ERROR", GraspState::kError)
      .export_values();

  // ControlConstants — 导出为模块级常量
  m.attr("JOINT_TOLERANCE") = ControlConstants::kJointTolerance;
  m.attr("FINGER_TOLERANCE") = ControlConstants::kFingerTolerance;
  m.attr("MOTION_TIMEOUT") = ControlConstants::kMotionTimeout;
  m.attr("POLL_INTERVAL") = ControlConstants::kPollInterval;
  m.attr("SETTLE_TIME") = ControlConstants::kSettleTime;
  m.attr("DEFAULT_STEPS") = ControlConstants::kDefaultSteps;
  m.attr("DEFAULT_STEP_TIME") = ControlConstants::kDefaultStepTime;
  m.attr("IMAGE_SYNC_QUEUE_SIZE") = ControlConstants::kImageSyncQueueSize;
  m.attr("IMAGE_SYNC_SLOP") = ControlConstants::kImageSyncSlop;
  m.attr("FINGER_STABLE_COUNT") = ControlConstants::kFingerStableCount;
  m.attr("FINGER_STABLE_TOL") = ControlConstants::kFingerStableTol;
  m.attr("READY_TIMEOUT") = ControlConstants::kReadyTimeout;

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
      .def("get_dof", &IKSolver::get_dof);

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
           py::call_guard<py::gil_scoped_release>());

  // IVisionProcessor（不可实例化基类）
  py::class_<IVisionProcessor, std::shared_ptr<IVisionProcessor>>(
      m, "IVisionProcessor");

  // GraspTaskManager
  py::class_<GraspTaskManager, std::shared_ptr<GraspTaskManager>>(
      m, "GraspTaskManager")
      .def(py::init<std::shared_ptr<IRobotController>,
                    std::shared_ptr<IVisionProcessor>, double, double,
                    const std::array<double, 3>&>(),
           py::arg("robot"), py::arg("vision"),
           py::arg("approach_height") = 0.15,
           py::arg("grasp_height_offset") = 0.02,
           py::arg("grasp_rpy") =
               std::array<double, 3>{M_PI, 0.0, M_PI})
      .def("run", &GraspTaskManager::run,
           py::arg("timeout") = 30.0,
           py::call_guard<py::gil_scoped_release>())
      .def("get_state", &GraspTaskManager::get_state);

  // ===== Layer 2 ROS2 节点 =====

  // RobotControllerNode
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
                  py::arg("processor"), py::arg("topics"))
      .def("get_latest_result", &VisionProcessorNode::get_latest_result)
      .def("wait_for_detection", &VisionProcessorNode::wait_for_detection,
           py::arg("timeout") = 10.0,
           py::call_guard<py::gil_scoped_release>())
      .def("get_logger",
           [](VisionProcessorNode& self) -> rclcpp::Logger {
             return self.get_logger();
           });

  // ===== Profile 函数 =====
  auto profiles = m.def_submodule("profiles");
  profiles.def("panda", &profiles::panda);
  profiles.def("panda_gripper", &profiles::panda_gripper);
  profiles.def("panda_red_detector", &profiles::panda_red_detector);
}
