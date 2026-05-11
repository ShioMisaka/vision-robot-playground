cmake_minimum_required(VERSION 3.8)
project(robot_demos)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ===== 依赖 =====
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_controller REQUIRED)
find_package(robot_vision REQUIRED)
find_package(robot_tasks REQUIRED)
find_package(robot_api_cpp REQUIRED)
find_package(robot_logger REQUIRED)
find_package(robot_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED)
find_package(robot_description REQUIRED)

# ===== demo_grasp_tcp: TCP 抓取演示（来自 robot_controller）=====
add_executable(demo_grasp_tcp
  demo/demo_grasp_tcp.cpp
)
target_link_libraries(demo_grasp_tcp robot_controller::robot_nodes robot_logger::robot_logger_lib)
target_compile_definitions(demo_grasp_tcp PRIVATE ROBOT_LOGGER_MODULE_NAME="demos")

# ===== demo_camera: 相机画面显示（来自 robot_vision）=====
add_executable(demo_camera
  demo/demo_camera.cpp
)
target_link_libraries(demo_camera robot_controller::robot_nodes robot_vision::robot_vision_nodes robot_logger::robot_logger_lib)
target_compile_definitions(demo_camera PRIVATE ROBOT_LOGGER_MODULE_NAME="demos")

# ===== demo_vision_grasp: 视觉引导抓取（来自 robot_api_python）=====
add_executable(demo_vision_grasp
  demo/demo_vision_grasp.cpp
)
target_link_libraries(demo_vision_grasp
  robot_api_cpp::robot_api_client_lib
  robot_tasks::robot_tasks_lib
  robot_vision::robot_vision_nodes
  robot_logger::robot_logger_lib
)
ament_target_dependencies(demo_vision_grasp robot_description)
target_compile_definitions(demo_vision_grasp PRIVATE ROBOT_LOGGER_MODULE_NAME="demos")

# ===== demo_vision_diagnostic: 视觉坐标变换诊断（不运动机器人）=====
add_executable(demo_vision_diagnostic
  demo/demo_vision_diagnostic.cpp
)
target_link_libraries(demo_vision_diagnostic
  robot_api_cpp::robot_api_client_lib
  robot_tasks::robot_tasks_lib
  robot_vision::robot_vision_nodes
  robot_logger::robot_logger_lib
)
ament_target_dependencies(demo_vision_diagnostic robot_description)
target_compile_definitions(demo_vision_diagnostic PRIVATE ROBOT_LOGGER_MODULE_NAME="demos")

# ===== test_robot_node: 集成测试（来自 robot_vision）=====
add_executable(test_robot_node
  test/test_robot_node.cpp
)
target_link_libraries(test_robot_node robot_controller::robot_nodes robot_vision::robot_vision_nodes robot_logger::robot_logger_lib)
target_compile_definitions(test_robot_node PRIVATE ROBOT_LOGGER_MODULE_NAME="demos")

# ===== 安装 =====
install(TARGETS demo_grasp_tcp demo_camera demo_vision_grasp demo_vision_diagnostic test_robot_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
