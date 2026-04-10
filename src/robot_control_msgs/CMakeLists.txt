cmake_minimum_required(VERSION 3.8)
project(robot_control_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SolveIK.srv"
  "srv/MoveJoint.srv"
  "srv/MovePose.srv"
  "srv/MoveLinear.srv"
  "srv/ControlGripper.srv"
  "srv/GoHome.srv"
  "srv/SetSpeed.srv"
  "srv/GetRobotState.srv"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
