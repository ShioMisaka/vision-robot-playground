cmake_minimum_required(VERSION 3.8)
project(robot_api_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ===== 依赖 =====
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_controller REQUIRED)
find_package(robot_logger REQUIRED)
find_package(robot_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# ===== 客户端共享库 =====
add_library(robot_api_client_lib SHARED
  src/service_robot_controller.cpp
  src/robot_client.cpp
)

target_include_directories(robot_api_client_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(robot_api_client_lib PUBLIC
  robot_controller::robot_motion
  robot_logger::robot_logger_lib
)

ament_target_dependencies(robot_api_client_lib PUBLIC
  rclcpp
  robot_msgs
  tf2_ros
)

target_compile_definitions(robot_api_client_lib PRIVATE ROBOT_LOGGER_MODULE_NAME="api_cpp")

# ===== 安装 =====
install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS robot_api_client_lib
  EXPORT robot_api_cpp_export
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ===== 导出 =====
ament_export_targets(robot_api_cpp_export HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  robot_controller
  robot_msgs
  tf2_ros
  robot_logger
)

ament_package()
