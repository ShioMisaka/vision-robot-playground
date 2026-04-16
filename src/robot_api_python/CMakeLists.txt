cmake_minimum_required(VERSION 3.8)
project(robot_control_cpp_py)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ===== 依赖 =====
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_control_cpp REQUIRED)
find_package(robot_vision_cpp REQUIRED)

# Python3 必须在 pybind11 之前查找（包含 Development 用于编译）
find_package(Python3 REQUIRED COMPONENTS Interpreter Development NumPy)

# 使用 pybind11 旧式 CMake 模块（兼容性更好）
find_package(pybind11 REQUIRED)

# ===== pybind11 模块 =====
pybind11_add_module(_core MODULE
  src/bindings.cpp
)

target_link_libraries(_core PRIVATE
  robot_control_cpp::robot_kinematics
  robot_control_cpp::robot_motion
  robot_control_cpp::robot_nodes
  robot_vision_cpp::robot_vision_core
  robot_vision_cpp::robot_vision_nodes
)

# ===== 安装 =====
set(PY_VER "${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}")
set(PY_DIR "lib/python${PY_VER}/site-packages")

# 安装 Python 包（自动设置 PYTHONPATH 环境钩子）
ament_python_install_package(robot_control_cpp_py
  PACKAGE_DIR robot_control_cpp_py)

# 安装编译后的 pybind11 模块到同一目录
install(TARGETS _core
  LIBRARY DESTINATION "${PY_DIR}/robot_control_cpp_py")

ament_package()
