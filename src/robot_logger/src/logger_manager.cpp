#include "robot_logger/logger.hpp"

#include <chrono>
#include <cstdio>
#include <mutex>
#include <unordered_map>

#include <spdlog/pattern_formatter.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

namespace robot_logger {

struct LoggerManager::Impl {
  std::mutex mutex;
  LoggerConfig config;
  std::shared_ptr<spdlog::sinks::stdout_color_sink_mt> console_sink;
  std::shared_ptr<spdlog::sinks::rotating_file_sink_mt> file_sink;
  std::unordered_map<std::string, std::shared_ptr<spdlog::logger>> loggers;
  bool initialized{false};
};

LoggerManager::LoggerManager() : pimpl_(std::make_unique<Impl>()) {}

LoggerManager::~LoggerManager() {
  if (pimpl_->initialized) {
    spdlog::shutdown();
  }
}

LoggerManager& LoggerManager::instance() {
  static LoggerManager mgr;
  return mgr;
}

void LoggerManager::init(const LoggerConfig& config) {
  std::lock_guard<std::mutex> lock(pimpl_->mutex);
  if (pimpl_->initialized) return;

  pimpl_->config = config;

  // Create sinks
  std::vector<spdlog::sink_ptr> sinks;

  if (config.enable_console) {
    auto console = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    console->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] [%n] %v");
    pimpl_->console_sink = console;
    sinks.push_back(console);
  }

  if (config.enable_file) {
    // Ensure directory exists
    std::string cmd = "mkdir -p " + config.log_dir;
    std::ignore = std::system(cmd.c_str());

    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);
    struct tm tm_buf;
    localtime_r(&time, &tm_buf);
    char date_str[16];
    std::strftime(date_str, sizeof(date_str), "%Y-%m-%d", &tm_buf);

    std::string filepath =
        config.log_dir + "/isaac_ros_" + std::string(date_str) + ".log";

    auto file = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
        filepath, config.max_file_size, config.max_files);
    file->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%l] [%n] %v");
    pimpl_->file_sink = file;
    sinks.push_back(file);
  }

  // Set global spdlog level
  auto level = spdlog::level::from_str(config.level);
  spdlog::set_level(level);

  // Create default logger
  auto default_logger = std::make_shared<spdlog::logger>(
      "default", sinks.begin(), sinks.end());
  default_logger->set_level(level);
  spdlog::set_default_logger(default_logger);

  pimpl_->initialized = true;
}

std::shared_ptr<spdlog::logger> LoggerManager::get(const std::string& module) {
  std::lock_guard<std::mutex> lock(pimpl_->mutex);

  auto it = pimpl_->loggers.find(module);
  if (it != pimpl_->loggers.end()) {
    return it->second;
  }

  // Create new logger with same sinks
  std::vector<spdlog::sink_ptr> sinks;
  if (pimpl_->console_sink) sinks.push_back(pimpl_->console_sink);
  if (pimpl_->file_sink) sinks.push_back(pimpl_->file_sink);

  auto logger =
      std::make_shared<spdlog::logger>(module, sinks.begin(), sinks.end());
  logger->set_level(spdlog::default_logger()->level());

  pimpl_->loggers[module] = logger;
  return logger;
}

void LoggerManager::set_level(const std::string& module,
                              spdlog::level::level_enum level) {
  std::lock_guard<std::mutex> lock(pimpl_->mutex);
  auto it = pimpl_->loggers.find(module);
  if (it != pimpl_->loggers.end()) {
    it->second->set_level(level);
  }
}

void LoggerManager::set_global_level(spdlog::level::level_enum level) {
  spdlog::set_level(level);
  std::lock_guard<std::mutex> lock(pimpl_->mutex);
  for (auto& [name, logger] : pimpl_->loggers) {
    logger->set_level(level);
  }
}

void LoggerManager::flush() {
  std::lock_guard<std::mutex> lock(pimpl_->mutex);
  for (auto& [name, logger] : pimpl_->loggers) {
    logger->flush();
  }
  spdlog::default_logger()->flush();
}

void LoggerManager::shutdown() {
  std::lock_guard<std::mutex> lock(pimpl_->mutex);
  spdlog::shutdown();
  pimpl_->loggers.clear();
  pimpl_->initialized = false;
}

}  // namespace robot_logger
