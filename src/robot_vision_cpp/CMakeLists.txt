cmake_minimum_required(VERSION 3.8)
project(robot_vision_cpp)

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
find_package(image_transport REQUIRED)
find_package(robot_control_msgs REQUIRED)
find_package(arm_control_interfaces REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(robot_control_cpp REQUIRED)

# ===== robot_vision_core: 视觉处理（零 ROS 依赖）=====
add_library(robot_vision_core SHARED
  src/vision/color_detector.cpp
)

target_include_directories(robot_vision_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(robot_vision_core
  OpenCV
  Eigen3
)

# ===== robot_vision_nodes: ROS2 视觉节点 + 任务管理器（依赖 vision_core + motion）=====
add_library(robot_vision_nodes SHARED
  src/nodes/vision_processor_node.cpp
  src/nodes/grasp_task_manager.cpp
)

target_include_directories(robot_vision_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(robot_vision_nodes robot_vision_core robot_control_cpp::robot_motion)

ament_target_dependencies(robot_vision_nodes
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  message_filters
  image_transport
  robot_control_msgs
  arm_control_interfaces
)

# ===== 安装 =====
install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS robot_vision_core robot_vision_nodes
  EXPORT robot_vision_cpp_export
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ===== 导出（供下游包 find_package 使用）=====
ament_export_targets(robot_vision_cpp_export HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  message_filters
  image_transport
  robot_control_msgs
  arm_control_interfaces
  Eigen3
  OpenCV
  robot_control_cpp
)

ament_package()
