#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include <spdlog/logger.h>
#include <spdlog/spdlog.h>

#include <robot_logger/logger_config.hpp>

namespace robot_logger {

class LoggerManager {
 public:
  static LoggerManager& instance();

  void init(const LoggerConfig& config = {});

  std::shared_ptr<spdlog::logger> get(const std::string& module);

  void set_level(const std::string& module, spdlog::level::level_enum level);
  void set_global_level(spdlog::level::level_enum level);

  void flush();
  void shutdown();

 private:
  LoggerManager();
  ~LoggerManager();

  LoggerManager(const LoggerManager&) = delete;
  LoggerManager& operator=(const LoggerManager&) = delete;

  struct Impl;
  std::unique_ptr<Impl> pimpl_;
};

namespace detail {

inline void ensure_init() {
  static std::once_flag flag;
  std::call_once(flag, [] { LoggerManager::instance().init(); });
}

}  // namespace detail

}  // namespace robot_logger

// ---------------------------------------------------------------------------
// LOG macros — default module from ROBOT_LOGGER_MODULE_NAME (set by CMake)
// ---------------------------------------------------------------------------

#ifndef ROBOT_LOGGER_MODULE_NAME
#define ROBOT_LOGGER_MODULE_NAME "unknown"
#endif

#define LOG_TRACE(...)                                             \
  do {                                                             \
    ::robot_logger::detail::ensure_init();                         \
    ::robot_logger::LoggerManager::instance()                      \
        .get(ROBOT_LOGGER_MODULE_NAME)                             \
        ->trace(__VA_ARGS__);                                      \
  } while (0)

#define LOG_DEBUG(...)                                             \
  do {                                                             \
    ::robot_logger::detail::ensure_init();                         \
    ::robot_logger::LoggerManager::instance()                      \
        .get(ROBOT_LOGGER_MODULE_NAME)                             \
        ->debug(__VA_ARGS__);                                      \
  } while (0)

#define LOG_INFO(...)                                              \
  do {                                                             \
    ::robot_logger::detail::ensure_init();                         \
    ::robot_logger::LoggerManager::instance()                      \
        .get(ROBOT_LOGGER_MODULE_NAME)                             \
        ->info(__VA_ARGS__);                                       \
  } while (0)

#define LOG_WARN(...)                                              \
  do {                                                             \
    ::robot_logger::detail::ensure_init();                         \
    ::robot_logger::LoggerManager::instance()                      \
        .get(ROBOT_LOGGER_MODULE_NAME)                             \
        ->warn(__VA_ARGS__);                                       \
  } while (0)

#define LOG_ERROR(...)                                             \
  do {                                                             \
    ::robot_logger::detail::ensure_init();                         \
    ::robot_logger::LoggerManager::instance()                      \
        .get(ROBOT_LOGGER_MODULE_NAME)                             \
        ->error(__VA_ARGS__);                                      \
  } while (0)

#define LOG_CRITICAL(...)                                          \
  do {                                                             \
    ::robot_logger::detail::ensure_init();                         \
    ::robot_logger::LoggerManager::instance()                      \
        .get(ROBOT_LOGGER_MODULE_NAME)                             \
        ->critical(__VA_ARGS__);                                   \
  } while (0)

// Throttled variants (ms interval) — thread-safe via std::atomic
#define LOG_WARN_THROTTLE(ms, ...)                                     \
  do {                                                                 \
    static std::atomic<int64_t> _log_last_ms{0};                       \
    auto _log_now_ms = std::chrono::duration_cast<std::chrono::milliseconds>( \
        std::chrono::steady_clock::now().time_since_epoch()).count();   \
    auto _log_prev = _log_last_ms.load(std::memory_order_relaxed);      \
    if (_log_now_ms - _log_prev >= ms) {                               \
      _log_last_ms.store(_log_now_ms, std::memory_order_relaxed);       \
      LOG_WARN(__VA_ARGS__);                                           \
    }                                                                  \
  } while (0)

#define LOG_INFO_THROTTLE(ms, ...)                                     \
  do {                                                                 \
    static std::atomic<int64_t> _log_last_ms{0};                       \
    auto _log_now_ms = std::chrono::duration_cast<std::chrono::milliseconds>( \
        std::chrono::steady_clock::now().time_since_epoch()).count();   \
    auto _log_prev = _log_last_ms.load(std::memory_order_relaxed);      \
    if (_log_now_ms - _log_prev >= ms) {                               \
      _log_last_ms.store(_log_now_ms, std::memory_order_relaxed);       \
      LOG_INFO(__VA_ARGS__);                                           \
    }                                                                  \
  } while (0)
