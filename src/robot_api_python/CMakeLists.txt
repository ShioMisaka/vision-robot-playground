cmake_minimum_required(VERSION 3.8)
project(robot_api_python)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ===== 依赖 =====
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_api_cpp REQUIRED)
find_package(robot_controller REQUIRED)
find_package(robot_vision REQUIRED)
find_package(robot_tasks REQUIRED)
find_package(robot_msgs REQUIRED)
find_package(robot_logger REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Python3 必须在 pybind11 之前查找（包含 Development 用于编译）
find_package(Python3 REQUIRED COMPONENTS Interpreter Development NumPy)

# 使用 pybind11 旧式 CMake 模块（兼容性更好）
find_package(pybind11 REQUIRED)

# ===== pybind11 模块 =====
pybind11_add_module(_core MODULE
  src/bindings.cpp
)

target_include_directories(_core PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(_core PRIVATE
  robot_api_cpp::robot_api_client_lib
  robot_controller::robot_kinematics
  robot_controller::robot_nodes
  robot_vision::robot_vision_core
  robot_vision::robot_vision_nodes
  robot_tasks::robot_tasks_lib
)

# ===== 安装 =====
set(PY_VER "${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}")
set(PY_DIR "lib/python${PY_VER}/site-packages")

# 安装 Python 包（自动设置 PYTHONPATH 环境钩子）
ament_python_install_package(robot_api_python
  PACKAGE_DIR robot_api_python)

# 安装编译后的 pybind11 模块到同一目录
install(TARGETS _core
  LIBRARY DESTINATION "${PY_DIR}/robot_api_python")

# ===== 导出（供下游包 find_package 使用）=====
ament_export_dependencies(
  rclcpp
  robot_api_cpp
  robot_controller
  robot_msgs
  tf2_ros
  robot_logger
  robot_tasks
)

ament_package()
