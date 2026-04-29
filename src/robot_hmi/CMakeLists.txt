cmake_minimum_required(VERSION 3.8)
project(robot_hmi)

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
find_package(robot_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Core Gui)

# AUTOMOC must be set after Qt5 find_package and ament_cmake
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# ===== 可执行文件 =====
add_executable(robot_hmi
  src/main.cpp
  src/pendant_node.cpp
  src/main_window.cpp
  # Panels
  src/panels/connection_bar.cpp
  src/panels/camera_panel.cpp
  src/panels/robot_state_bar.cpp
  src/panels/joint_control_panel.cpp
  src/panels/cartesian_panel.cpp
  src/panels/function_panel.cpp
  # Headers (for AUTOMOC)
  include/robot_hmi/main_window.hpp
  include/robot_hmi/panels/connection_bar.hpp
  include/robot_hmi/panels/camera_panel.hpp
  include/robot_hmi/panels/robot_state_bar.hpp
  include/robot_hmi/panels/joint_control_panel.hpp
  include/robot_hmi/panels/cartesian_panel.hpp
  include/robot_hmi/panels/function_panel.hpp
)

target_include_directories(robot_hmi PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Ensure AUTOMOC scans include/ directory
set_target_properties(robot_hmi PROPERTIES
  AUTOMOC ON
  AUTOMOC_PATH_PREFIX ON
)

target_link_libraries(robot_hmi
  Qt5::Widgets
  Qt5::Core
  Qt5::Gui
)

ament_target_dependencies(robot_hmi
  rclcpp
  sensor_msgs
  cv_bridge
  message_filters
  robot_msgs
  OpenCV
)

# ===== 安装 =====
install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS robot_hmi
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
