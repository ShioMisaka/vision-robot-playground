cmake_minimum_required(VERSION 3.8)
project(robot_control_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ===== 依赖 =====
find_package(ament_cmake REQUIRED)
find_package(robot_control_cpp REQUIRED)
find_package(robot_vision_cpp REQUIRED)
find_package(rclcpp REQUIRED)

# ===== 测试 =====
# 独立 S 曲线轨迹规划测试（无 ROS 依赖，可离线运行）
add_executable(test_trajectory_planner
  test/test_trajectory_planner.cpp
)
target_link_libraries(test_trajectory_planner robot_control_cpp::robot_kinematics)
install(TARGETS test_trajectory_planner
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 独立运动控制器测试（无 ROS 依赖，可离线运行）
add_executable(test_motion_controller
  test/test_motion_controller.cpp
)
target_link_libraries(test_motion_controller robot_control_cpp::robot_motion)
install(TARGETS test_motion_controller
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 独立 IK 测试（无 ROS 依赖，可离线运行）
add_executable(test_ik_solver
  test/test_ik_solver.cpp
)
target_link_libraries(test_ik_solver robot_control_cpp::robot_kinematics)
install(TARGETS test_ik_solver
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 集成测试（需要 Isaac Sim 运行中）
add_executable(test_robot_node
  test/test_robot_node.cpp
)
target_link_libraries(test_robot_node robot_control_cpp::robot_nodes robot_vision_cpp::robot_vision_nodes)
install(TARGETS test_robot_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 相机 TF 链验证测试（需要 Isaac Sim 运行中）
add_executable(test_camera_tf
  test/test_camera_tf.cpp
)
target_link_libraries(test_camera_tf robot_control_cpp::robot_nodes robot_vision_cpp::robot_vision_nodes)
install(TARGETS test_camera_tf
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# ===== 演示 =====
add_executable(demo_grasp_tcp
  demo/demo_grasp_tcp.cpp
)
target_link_libraries(demo_grasp_tcp robot_control_cpp::robot_nodes)
install(TARGETS demo_grasp_tcp
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 相机画面显示（RGB + 深度伪彩色）
add_executable(demo_camera
  demo/demo_camera.cpp
)
target_link_libraries(demo_camera robot_control_cpp::robot_nodes robot_vision_cpp::robot_vision_nodes)
install(TARGETS demo_camera
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
