cmake_minimum_required(VERSION 3.8)
project(robot_controller)

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
find_package(robot_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ament_index_cpp REQUIRED)
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
  ament_index_cpp
)

# ===== robot_motion: 运动控制器（依赖 kinematics）=====
add_library(robot_motion SHARED
  src/motion/robot_motion_controller.cpp
  src/motion/jog_controller.cpp
)

target_include_directories(robot_motion PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(robot_motion robot_kinematics)

# ===== robot_nodes: ROS2 节点（依赖 motion）=====
add_library(robot_nodes SHARED
  src/nodes/robot_controller_node.cpp
  src/nodes/robot_state.cpp
  src/nodes/setpoint_generator.cpp
  src/nodes/trajectory_executor.cpp
)

target_include_directories(robot_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(robot_nodes robot_motion)

ament_target_dependencies(robot_nodes
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2
  tf2_geometry_msgs
  robot_msgs
  rclcpp_action
  ament_index_cpp
)

# ===== 测试 =====
add_executable(test_trajectory_planner
  test/test_trajectory_planner.cpp
)
target_link_libraries(test_trajectory_planner robot_kinematics)

add_executable(test_motion_controller
  test/test_motion_controller.cpp
)
target_link_libraries(test_motion_controller robot_motion)

add_executable(test_ik_solver
  test/test_ik_solver.cpp
)
target_link_libraries(test_ik_solver robot_kinematics)

# ===== 演示 =====
add_executable(demo_grasp_tcp
  demo/demo_grasp_tcp.cpp
)
target_link_libraries(demo_grasp_tcp robot_nodes)

# ===== 安装 =====
install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS robot_kinematics robot_motion robot_nodes
  EXPORT robot_controller_export
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS test_trajectory_planner test_motion_controller test_ik_solver
  demo_grasp_tcp
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# ===== 导出（供下游包 find_package 使用）=====
ament_export_targets(robot_controller_export HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2
  tf2_geometry_msgs
  robot_msgs
  rclcpp_action
  ament_index_cpp
  orocos_kdl
  urdfdom
  urdf
  kdl_parser
  Eigen3
)

ament_package()
