cmake_minimum_required(VERSION 3.8)
project(robot_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SolveIK.srv"
  "srv/ControlGripper.srv"
  "srv/SetSpeed.srv"
  "srv/GetRobotState.srv"
  "srv/SetTCP.srv"
  "srv/SetSpeedRatio.srv"
  "srv/RobotCmd.srv"
  "srv/AcquireControl.srv"
  "srv/ReleaseControl.srv"
  "srv/RenewLease.srv"
  "srv/RequestTeachingMode.srv"
  "action/MoveJ.action"
  "action/MoveL.action"
  "action/GoHome.action"
  "action/GraspTask.action"
  "msg/RobotStatus.msg"
  "msg/JogCommand.msg"
  DEPENDENCIES std_msgs builtin_interfaces geometry_msgs action_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
