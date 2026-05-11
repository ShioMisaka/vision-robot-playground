cmake_minimum_required(VERSION 3.8)
project(robot_vision)

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
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(robot_logger REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(robot_vision_core SHARED
  src/vision/color_detector.cpp
  src/vision/camera_config_loader.cpp
)

target_include_directories(robot_vision_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(robot_vision_core
  OpenCV
  Eigen3
)

target_link_libraries(robot_vision_core Eigen3::Eigen robot_logger::robot_logger_lib yaml-cpp)

# ===== robot_vision_nodes: ROS2 视觉节点（依赖 vision_core）=====
add_library(robot_vision_nodes SHARED
  src/nodes/vision_processor_node.cpp
)

target_include_directories(robot_vision_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(robot_vision_nodes robot_vision_core robot_logger::robot_logger_lib)
target_compile_definitions(robot_vision_nodes PRIVATE ROBOT_LOGGER_MODULE_NAME="vision")

ament_target_dependencies(robot_vision_nodes
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  message_filters
  tf2_ros
)

# ===== 安装 =====
install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS robot_vision_core robot_vision_nodes
  EXPORT robot_vision_export
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ===== 导出（供下游包 find_package 使用）=====
ament_export_targets(robot_vision_export HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  message_filters
  Eigen3
  OpenCV
  yaml-cpp
  robot_logger
)

ament_package()
