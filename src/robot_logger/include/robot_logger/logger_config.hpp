#pragma once

#include <cstddef>
#include <string>

namespace robot_logger {

struct LoggerConfig {
  std::string log_dir = "/tmp/isaac_ros_logs";
  std::size_t max_file_size = 10 * 1024 * 1024;  // 10 MB
  std::size_t max_files = 5;
  std::string level = "info";
  bool async_mode = true;
  bool enable_console = true;
  bool enable_file = true;
};

}  // namespace robot_logger
