cmake_minimum_required(VERSION 3.8)
project(robot_tasks)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ===== 依赖查找 =====
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_api_cpp REQUIRED)
find_package(robot_vision REQUIRED)
find_package(robot_controller REQUIRED)
find_package(robot_logger REQUIRED)
find_package(Eigen3 REQUIRED)

# ===== robot_tasks_lib: 任务编排库 =====
add_library(robot_tasks_lib SHARED
  src/grasp_task_manager.cpp
)

target_include_directories(robot_tasks_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(robot_tasks_lib
  robot_api_cpp::robot_api_client_lib
  robot_vision::robot_vision_core
  robot_logger::robot_logger_lib
  Eigen3::Eigen
)

ament_target_dependencies(robot_tasks_lib
  rclcpp
)

target_compile_definitions(robot_tasks_lib PRIVATE ROBOT_LOGGER_MODULE_NAME="tasks")

# ===== 安装 =====
install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS robot_tasks_lib
  EXPORT robot_tasks_export
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ===== 集成测试（需要 Isaac Sim 运行中）=====
find_package(ament_index_cpp REQUIRED)
add_executable(test_camera_tf
  test/test_camera_tf.cpp
)
target_link_libraries(test_camera_tf
  robot_controller::robot_nodes
  robot_vision::robot_vision_nodes
  robot_tasks_lib
  robot_logger::robot_logger_lib
)
target_compile_definitions(test_camera_tf PRIVATE ROBOT_LOGGER_MODULE_NAME="tasks")

install(TARGETS test_camera_tf
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# ===== 导出 =====
ament_export_targets(robot_tasks_export HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  robot_api_cpp
  robot_vision
  robot_logger
)

ament_package()
