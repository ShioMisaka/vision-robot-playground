cmake_minimum_required(VERSION 3.8)
project(robot_api_python)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ===== 依赖 =====
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_controller REQUIRED)
find_package(robot_vision REQUIRED)
find_package(robot_msgs REQUIRED)
find_package(robot_logger REQUIRED)
find_package(tf2_ros REQUIRED)

# Python3 必须在 pybind11 之前查找（包含 Development 用于编译）
find_package(Python3 REQUIRED COMPONENTS Interpreter Development NumPy)

# 使用 pybind11 旧式 CMake 模块（兼容性更好）
find_package(pybind11 REQUIRED)

# ===== 客户端共享库（C++ Service 客户端，可供 pybind11 和 C++ 可执行文件复用）=====
add_library(robot_api_client_lib SHARED
  src/service_robot_controller.cpp
  src/robot_client_node.cpp
)

target_include_directories(robot_api_client_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(robot_api_client_lib PUBLIC
  robot_controller::robot_motion
  robot_logger::robot_logger_lib
)

ament_target_dependencies(robot_api_client_lib PUBLIC
  rclcpp
  robot_msgs
  tf2_ros
)

target_compile_definitions(robot_api_client_lib PRIVATE ROBOT_LOGGER_MODULE_NAME="api_python")

# ===== pybind11 模块 =====
pybind11_add_module(_core MODULE
  src/bindings.cpp
)

target_include_directories(_core PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(_core PRIVATE
  robot_api_client_lib
  robot_controller::robot_kinematics
  robot_controller::robot_nodes
  robot_vision::robot_vision_core
  robot_vision::robot_vision_nodes
)

# ===== 演示（C++，使用 RobotClient 连接外部控制器）=====
add_executable(demo_vision_grasp
  demo/demo_vision_grasp.cpp
)
target_link_libraries(demo_vision_grasp
  robot_api_client_lib
  robot_vision::robot_vision_core
  robot_vision::robot_vision_nodes
  robot_logger::robot_logger_lib
)
target_compile_definitions(demo_vision_grasp PRIVATE ROBOT_LOGGER_MODULE_NAME="vision")

# ===== 安装 =====
set(PY_VER "${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}")
set(PY_DIR "lib/python${PY_VER}/site-packages")

# 安装头文件
install(DIRECTORY include/
  DESTINATION include
)

# 安装客户端共享库
install(TARGETS robot_api_client_lib
  EXPORT robot_api_python_export
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 安装 Python 包（自动设置 PYTHONPATH 环境钩子）
ament_python_install_package(robot_api_python
  PACKAGE_DIR robot_api_python)

# 安装编译后的 pybind11 模块到同一目录
install(TARGETS _core
  LIBRARY DESTINATION "${PY_DIR}/robot_api_python")

# 安装演示可执行文件
install(TARGETS demo_vision_grasp
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# ===== 导出（供下游包 find_package 使用）=====
ament_export_targets(robot_api_python_export HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  robot_controller
  robot_msgs
  tf2_ros
  robot_logger
)

ament_package()
