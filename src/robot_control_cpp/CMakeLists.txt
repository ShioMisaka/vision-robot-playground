cmake_minimum_required(VERSION 3.8)
project(robot_control_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++17
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
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(urdfdom REQUIRED)
find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)

# ===== Layer 1: 核心库（不依赖 rclcpp）=====
add_library(robot_control_core SHARED
  src/ik_solver.cpp
  src/color_detector.cpp
  src/robot_motion_controller.cpp
  src/grasp_task_manager.cpp
)

target_include_directories(robot_control_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(robot_control_core
  orocos_kdl
  urdfdom
  urdf
  kdl_parser
  Eigen3
  OpenCV
)

# ===== Layer 2: ROS 2 节点 =====
add_library(robot_control_nodes SHARED
  src/robot_controller_node.cpp
  src/vision_processor_node.cpp
)

target_include_directories(robot_control_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(robot_control_nodes robot_control_core)

ament_target_dependencies(robot_control_nodes
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2
  tf2_geometry_msgs
  cv_bridge
  message_filters
  image_transport
)

# ===== 安装 =====
install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS robot_control_core robot_control_nodes
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
  orocos_kdl
  urdfdom
  urdf
  kdl_parser
  Eigen3
  OpenCV
)

ament_package()
