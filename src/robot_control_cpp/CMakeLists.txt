cmake_minimum_required(VERSION 3.8)
project(robot_control_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ===== 依赖查找 =====
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(image_transport REQUIRED)
find_package(robot_control_msgs REQUIRED)
find_package(arm_control_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(urdfdom REQUIRED)
find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)

# ===== robot_kinematics: IK + 轨迹规划（零 ROS 依赖）=====
add_library(robot_kinematics SHARED
  src/kinematics/ik_solver.cpp
  src/kinematics/trajectory_planner.cpp
)

target_include_directories(robot_kinematics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(robot_kinematics
  orocos_kdl
  urdfdom
  urdf
  kdl_parser
  Eigen3
)

# ===== robot_vision: 视觉处理（零 ROS 依赖）=====
add_library(robot_vision SHARED
  src/vision/color_detector.cpp
)

target_include_directories(robot_vision PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(robot_vision
  OpenCV
  Eigen3
)

# ===== robot_motion: 运动控制器（依赖 kinematics）=====
add_library(robot_motion SHARED
  src/motion/robot_motion_controller.cpp
)

target_include_directories(robot_motion PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(robot_motion robot_kinematics)

# ===== robot_nodes: ROS2 节点（依赖 motion + vision）=====
add_library(robot_nodes SHARED
  src/nodes/robot_controller_node.cpp
  src/nodes/vision_processor_node.cpp
  src/nodes/grasp_task_manager.cpp
  src/nodes/robot_state.cpp
  src/nodes/trajectory_executor.cpp
)

target_include_directories(robot_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(robot_nodes robot_motion robot_vision)

ament_target_dependencies(robot_nodes
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2
  tf2_geometry_msgs
  cv_bridge
  message_filters
  image_transport
  robot_control_msgs
  rclcpp_action
  arm_control_interfaces
)

# ===== 安装 =====
install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS robot_kinematics robot_motion robot_vision robot_nodes
  EXPORT robot_control_cpp_export
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ===== 导出（供下游包 find_package 使用）=====
ament_export_targets(robot_control_cpp_export HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2
  tf2_geometry_msgs
  cv_bridge
  message_filters
  image_transport
  robot_control_msgs
  rclcpp_action
  arm_control_interfaces
  orocos_kdl
  urdfdom
  urdf
  kdl_parser
  Eigen3
  OpenCV
)

ament_package()
