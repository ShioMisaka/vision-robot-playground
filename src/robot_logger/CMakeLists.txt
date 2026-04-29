cmake_minimum_required(VERSION 3.16)
project(robot_logger)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(spdlog REQUIRED)

add_library(robot_logger_lib SHARED
  src/logger_manager.cpp
)
target_include_directories(robot_logger_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(robot_logger_lib PUBLIC spdlog::spdlog)
target_compile_features(robot_logger_lib PUBLIC cxx_std_17)

# Install
install(TARGETS robot_logger_lib
  EXPORT robot_logger_export
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)

# Export for downstream find_package via ament
ament_export_targets(robot_logger_export HAS_LIBRARY_TARGET)
ament_export_dependencies(spdlog)

ament_package()
