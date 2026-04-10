cmake_minimum_required(VERSION 3.8)
project(teaching_pendant)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ===== 依赖查找 =====
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(robot_control_msgs REQUIRED)
find_package(robot_control_cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Core Gui)

# AUTOMOC must be set after Qt5 find_package and ament_cmake
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# ===== 可执行文件 =====
add_executable(teaching_pendant
  src/main.cpp
  src/pendant_node.cpp
  src/main_window.cpp
  include/teaching_pendant/main_window.hpp
)

target_include_directories(teaching_pendant PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Ensure AUTOMOC scans include/ directory
set_target_properties(teaching_pendant PROPERTIES
  AUTOMOC ON
  AUTOMOC_PATH_PREFIX ON
)

target_link_libraries(teaching_pendant
  robot_control_cpp::robot_nodes
  Qt5::Widgets
  Qt5::Core
  Qt5::Gui
)

ament_target_dependencies(teaching_pendant
  rclcpp
  sensor_msgs
  cv_bridge
  message_filters
  robot_control_msgs
  OpenCV
)

# ===== 安装 =====
install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS teaching_pendant
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
