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
find_package(image_transport REQUIRED)
find_package(robot_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(robot_controller REQUIRED)
find_package(robot_logger REQUIRED)
find_package(tf2_ros REQUIRED)
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

target_link_libraries(robot_vision_core robot_logger::robot_logger_lib)

# ===== robot_vision_nodes: ROS2 视觉节点 + 任务管理器（依赖 vision_core + motion）=====
add_library(robot_vision_nodes SHARED
  src/nodes/vision_processor_node.cpp
  src/nodes/grasp_task_manager.cpp
)

target_include_directories(robot_vision_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(robot_vision_nodes robot_vision_core robot_controller::robot_motion robot_logger::robot_logger_lib)
target_compile_definitions(robot_vision_nodes PRIVATE ROBOT_LOGGER_MODULE_NAME="vision")

ament_target_dependencies(robot_vision_nodes
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  message_filters
  image_transport
  robot_msgs
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

# ===== 集成测试（需要 Isaac Sim 运行中）=====
add_executable(test_camera_tf
  test/test_camera_tf.cpp
)
target_link_libraries(test_camera_tf robot_controller::robot_nodes robot_logger::robot_logger_lib)
target_link_libraries(test_camera_tf robot_vision_nodes)
target_compile_definitions(test_camera_tf PRIVATE ROBOT_LOGGER_MODULE_NAME="vision")

install(TARGETS test_camera_tf
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# ===== 导出（供下游包 find_package 使用）=====
ament_export_targets(robot_vision_export HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  message_filters
  image_transport
  robot_msgs
  Eigen3
  OpenCV
  robot_controller
  robot_logger
)

ament_package()
