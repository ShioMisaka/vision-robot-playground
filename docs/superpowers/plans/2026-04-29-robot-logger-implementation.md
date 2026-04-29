# robot_logger 统一日志系统实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 新建 robot_logger 包，基于 spdlog 提供统一日志基础设施，迁移所有包的日志调用。

**Architecture:** 全局单例 LoggerManager + LOG_* 宏接口，spdlog 异步模式，控制台彩色 + 文件轮转双 sink。每个包通过 CMake 编译宏 `ROBOT_LOGGER_MODULE_NAME` 声明模块名。

**Tech Stack:** C++17, spdlog 1.12, ament_cmake, pybind11 (Python 绑定)

**Spec:** `docs/superpowers/specs/2026-04-29-robot-logger-design.md`

---

## File Structure

### 新建文件
| 文件 | 职责 |
|------|------|
| `src/robot_logger/CMakeLists.txt` | 构建配置，导出 robot_logger_lib |
| `src/robot_logger/package.xml` | 包声明，依赖 spdlog |
| `src/robot_logger/include/robot_logger/logger.hpp` | 主头文件：LoggerManager 声明 + LOG_* 宏 |
| `src/robot_logger/include/robot_logger/logger_config.hpp` | LoggerConfig 结构体 |
| `src/robot_logger/src/logger_manager.cpp` | LoggerManager 实现 |
| `src/robot_logger/CLAUDE.md` | 包级文档 |

### 修改文件 — 构建配置
| 文件 | 变更 |
|------|------|
| `src/robot_controller/CMakeLists.txt` | 添加 `find_package(robot_logger)` + 所有目标链接 robot_logger + 添加 `ROBOT_LOGGER_MODULE_NAME` |
| `src/robot_vision/CMakeLists.txt` | 同上 |
| `src/robot_hmi/CMakeLists.txt` | 添加 `find_package(robot_logger REQUIRED)` + `target_link_libraries` + `target_compile_definitions` |
| `src/robot_api_python/CMakeLists.txt` | 同上 + 链接到 pybind11 模块 |
| `src/robot_api_python/package.xml` | 添加 `<depend>robot_logger</depend>` |

### 修改文件 — C++ 源码（替换 RCLCPP_* → LOG_*）

**重要：不提供硬编码行号替换表**。文件可能已修改，执行时必须先 grep 确认当前内容。以下为需要修改的文件列表：

| 包 | 文件 |
|------|------|
| robot_controller | `src/nodes/robot_controller_node.cpp`, `src/nodes/ros_motion_bridge.cpp`, `src/nodes/standalone_main.cpp`, `demo/demo_grasp_tcp.cpp` |
| robot_vision | `src/nodes/vision_processor_node.cpp`, `src/nodes/grasp_task_manager.cpp`, `demo/demo_vision_grasp.cpp`, `demo/demo_camera.cpp` |
| robot_hmi | `src/pendant_node.cpp` |
| robot_api_python | `src/bindings.cpp`, `src/robot_client_node.cpp` |

### 修改文件 — 文档
| 文件 | 变更 |
|------|------|
| `CLAUDE.md` (根目录) | 添加 robot_logger 到包结构、依赖图、日志规则 |
| `src/robot_api_python/robot_api_python/__init__.py` | 导出 robot_logger Python 模块 |

---

## Task 1: 创建 robot_logger 包骨架

**Files:**
- Create: `src/robot_logger/CMakeLists.txt`
- Create: `src/robot_logger/package.xml`
- Create: `src/robot_logger/include/robot_logger/logger_config.hpp`
- Create: `src/robot_logger/include/robot_logger/logger.hpp`
- Create: `src/robot_logger/src/logger_manager.cpp`

- [ ] **Step 1: 创建包目录结构**

```bash
mkdir -p src/robot_logger/include/robot_logger src/robot_logger/src
```

- [ ] **Step 2: 创建 package.xml**

创建 `src/robot_logger/package.xml`：

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>robot_logger</name>
  <version>0.1.0</version>
  <description>Unified logging system based on spdlog for the Isaac ROS project</description>
  <maintainer email="user@example.com">ShioMisaka</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <build_depend>spdlog</build_depend>
  <exec_depend>spdlog</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 3: 创建 CMakeLists.txt**

创建 `src/robot_logger/CMakeLists.txt`：

```cmake
cmake_minimum_required(VERSION 3.16)
project(robot_logger)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

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
```

- [ ] **Step 4: 创建 logger_config.hpp**

创建 `src/robot_logger/include/robot_logger/logger_config.hpp`：

```cpp
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
```

- [ ] **Step 5: 创建 logger.hpp（头文件 + 宏定义）**

创建 `src/robot_logger/include/robot_logger/logger.hpp`：

```cpp
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
```

- [ ] **Step 6: 创建 logger_manager.cpp**

创建 `src/robot_logger/src/logger_manager.cpp`：

```cpp
#include "robot_logger/logger.hpp"

#include <chrono>
#include <cstdio>
#include <mutex>
#include <unordered_map>

#include <spdlog/pattern_formatter.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sink.h>
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
```

- [ ] **Step 7: 编译 robot_logger 包**

```bash
colcon build --base-paths src --packages-select robot_logger
```

Expected: BUILD SUCCEEDED

- [ ] **Step 8: 验证幂等编译**

```bash
source install/setup.zsh
colcon build --base-paths src --packages-select robot_logger
```

Expected: BUILD SUCCEEDED

- [ ] **Step 9: 验证 find_package 可用**

```bash
source install/setup.zsh
cmake -B /tmp/test_find_pkg -S /dev/null --find-package -DNAME=robot_logger -DCOMPILER_ID=GNU -DLANGUAGE=CXX 2>&1 || echo "Testing via colcon build instead"
```

如果上面的命令不好用，直接通过 Task 2 的编译来验证。

- [ ] **Step 10: Commit**

```bash
git add src/robot_logger/
git commit -m "feat(logger): create robot_logger package with LoggerManager and LOG macros"
```

---

## Task 2: 集成 robot_logger 到 robot_controller

**Files:**
- Modify: `src/robot_controller/CMakeLists.txt`
- Modify: `src/robot_controller/src/nodes/robot_controller_node.cpp`
- Modify: `src/robot_controller/src/nodes/ros_motion_bridge.cpp`
- Modify: `src/robot_controller/src/nodes/standalone_main.cpp`
- Modify: `src/robot_controller/demo/demo_grasp_tcp.cpp`

- [ ] **Step 1: 修改 robot_controller/CMakeLists.txt**

在 `find_package` 区域添加：
```cmake
find_package(robot_logger REQUIRED)
```

为每个 target（`robot_kinematics`, `robot_motion`, `robot_nodes`, `robot_controller_node`, `demo_grasp_tcp`, `test_trajectory_planner`, `test_motion_controller`, `test_ik_solver`）：
1. 在 `target_link_libraries` 中追加 `robot_logger::robot_logger_lib`
2. 添加 `target_compile_definitions(<target> PUBLIC ROBOT_LOGGER_MODULE_NAME="controller")`

对于共享库（robot_kinematics, robot_motion, robot_nodes）使用 `PUBLIC`，可执行目标使用 `PRIVATE`。

- [ ] **Step 2: 迁移所有 C++ 文件中的 RCLCPP_* 调用**

对以下 4 个文件逐一处理：
- `src/robot_controller/src/nodes/robot_controller_node.cpp`
- `src/robot_controller/src/nodes/ros_motion_bridge.cpp`
- `src/robot_controller/src/nodes/standalone_main.cpp`
- `src/robot_controller/demo/demo_grasp_tcp.cpp`

**每个文件的处理流程：**

1. 添加 `#include <robot_logger/logger.hpp>`（在文件头部，其他 include 之后）
2. 用 Grep 搜索文件中所有 `RCLCPP_INFO`、`RCLCPP_WARN`、`RCLCPP_ERROR`、`RCLCPP_DEBUG`、`RCLCPP_WARN_THROTTLE`、`RCLCPP_INFO_THROTTLE` 调用
3. 逐个替换，遵循以下规则：

**替换规则（适用于所有文件的通用模板）：**

| 原始模式 | 替换为 |
|---------|--------|
| `RCLCPP_INFO(this->get_logger(), "msg")` | `LOG_INFO("msg")` |
| `RCLCPP_INFO(node->get_logger(), "msg")` | `LOG_INFO("msg")` |
| `RCLCPP_INFO(rclcpp::get_logger("xxx"), "msg")` | `LOG_INFO("msg")` |
| `RCLCPP_INFO(logger, "msg")` (参数传入) | `LOG_INFO("msg")` |
| `RCLCPP_WARN(...)` | `LOG_WARN(...)` |
| `RCLCPP_ERROR(...)` | `LOG_ERROR(...)` |
| `RCLCPP_DEBUG(...)` | `LOG_DEBUG(...)` |
| `RCLCPP_WARN_THROTTLE(xxx->get_logger(), *xxx->get_clock(), N, "msg")` | `LOG_WARN_THROTTLE(N, "msg")` |
| `RCLCPP_INFO_THROTTLE(xxx->get_logger(), *xxx->get_clock(), N, "msg")` | `LOG_INFO_THROTTLE(N, "msg")` |

**格式字符串转换规则：**

| printf 格式 | fmt/spdlog 格式 |
|-------------|-----------------|
| `%s` | `{}` |
| `%d` | `{}` |
| `%u` | `{}` |
| `%.1f` | `{:.1f}` |
| `%.2f` | `{:.2f}` |
| `%.3f` | `{:.3f}` |
| `%.4f` | `{:.4f}` |
| `%.6f` | `{:.6f}` |
| `%zu` | `{}` |

**额外注意：**
- `xxx.c_str()` 参数可直接改为 `xxx`（spdlog/fmt 自动处理 std::string）
- 如果 `rclcpp/rclcpp.hpp` 仅用于日志且无其他用途，可以保留（节点类仍然需要它）
- 替换后确保没有遗漏的 `RCLCPP_` 调用

- [ ] **Step 3: 编译验证**

```bash
source install/setup.zsh
colcon build --base-paths src --packages-up-to robot_controller
```

Expected: BUILD SUCCEEDED

- [ ] **Step 4: Commit**

```bash
git add -A
git commit -m "feat(controller): integrate robot_logger, replace all RCLCPP_* with LOG_* macros"
```

---

## Task 3: 集成 robot_logger 到 robot_vision

**Files:**
- Modify: `src/robot_vision/CMakeLists.txt`
- Modify: `src/robot_vision/src/nodes/vision_processor_node.cpp`
- Modify: `src/robot_vision/src/nodes/grasp_task_manager.cpp`
- Modify: `src/robot_vision/demo/demo_vision_grasp.cpp`
- Modify: `src/robot_vision/demo/demo_camera.cpp`

- [ ] **Step 1: 修改 robot_vision/CMakeLists.txt**

同 Task 2 模式：
- 添加 `find_package(robot_logger REQUIRED)`
- 所有 target（`robot_vision_core`, `robot_vision_nodes`, `test_robot_node`, `test_camera_tf`, `demo_camera`, `demo_vision_grasp`）追加 `robot_logger::robot_logger_lib`
- 添加 `target_compile_definitions(<target> PUBLIC ROBOT_LOGGER_MODULE_NAME="vision")`

- [ ] **Step 2: 迁移所有 C++ 文件中的 RCLCPP_* 调用**

对以下 4 个文件逐一处理：
- `src/robot_vision/src/nodes/vision_processor_node.cpp`
- `src/robot_vision/src/nodes/grasp_task_manager.cpp`
- `src/robot_vision/demo/demo_vision_grasp.cpp`
- `src/robot_vision/demo/demo_camera.cpp`

遵循 Task 2 Step 2 中完全相同的替换规则和格式转换规则。

**特殊注意 `grasp_task_manager.cpp`：**
此文件有两类 RCLCPP 调用方式：
1. `RCLCPP_*(rclcpp::get_logger("grasp_task_manager"), ...)` — 统一替换为 `LOG_*(...)`
2. `RCLCPP_*(logger, ...)` 其中 `logger` 是函数参数 — 统一替换为 `LOG_*(...)`，忽略 logger 参数

- [ ] **Step 3: 编译验证**

```bash
colcon build --base-paths src --packages-up-to robot_vision
```

Expected: BUILD SUCCEEDED

- [ ] **Step 4: Commit**

```bash
git add -A
git commit -m "feat(vision): integrate robot_logger, replace all RCLCPP_* with LOG_* macros"
```

---

## Task 4: 集成 robot_logger 到 robot_hmi

**Files:**
- Modify: `src/robot_hmi/CMakeLists.txt`
- Modify: `src/robot_hmi/src/pendant_node.cpp`

- [ ] **Step 1: 修改 robot_hmi/CMakeLists.txt**

在 `find_package` 区域添加：
```cmake
find_package(robot_logger REQUIRED)
```

为 `robot_hmi` 可执行目标：
```cmake
target_link_libraries(robot_hmi PRIVATE robot_logger::robot_logger_lib)
target_compile_definitions(robot_hmi PRIVATE ROBOT_LOGGER_MODULE_NAME="hmi")
```

注意：robot_hmi 当前 CMakeLists.txt 可能没有显式 `find_package(robot_controller)`。需要先读取当前 CMakeLists.txt 确认现有结构，在适当位置添加 find_package 和 target 修改。

- [ ] **Step 2: 迁移 pendant_node.cpp 中的 RCLCPP_* 调用**

遵循 Task 2 Step 2 中的通用替换规则和格式转换规则。

- [ ] **Step 3: 编译验证**

```bash
colcon build --base-paths src --packages-up-to robot_hmi
```

Expected: BUILD SUCCEEDED

- [ ] **Step 4: Commit**

```bash
git add -A
git commit -m "feat(hmi): integrate robot_logger, replace all RCLCPP_* with LOG_* macros"
```

---

## Task 5: 集成 robot_logger 到 robot_api_python（C++ + Python 绑定）

**Files:**
- Modify: `src/robot_api_python/CMakeLists.txt`
- Modify: `src/robot_api_python/package.xml`
- Modify: `src/robot_api_python/src/bindings.cpp`
- Modify: `src/robot_api_python/src/robot_client_node.cpp`
- Modify: `src/robot_api_python/robot_api_python/__init__.py`

- [ ] **Step 1: 修改 robot_api_python/package.xml**

在 `<depend>robot_vision</depend>` 后添加：

```xml
<depend>robot_logger</depend>
```

- [ ] **Step 2: 修改 robot_api_python/CMakeLists.txt**

- 添加 `find_package(robot_logger REQUIRED)`
- `_core` pybind11 module target 追加 `robot_logger::robot_logger_lib`
- 添加 `target_compile_definitions(_core PRIVATE ROBOT_LOGGER_MODULE_NAME="api_python")`

- [ ] **Step 3: 修改 bindings.cpp — 添加 robot_logger Python 子模块**

文件：`src/robot_api_python/src/bindings.cpp`

1. 添加 `#include <robot_logger/logger.hpp>`
2. 在 `PYBIND11_MODULE` 内部，找到现有 `py::class_<rclcpp::Logger>` 块，在其 **前面** 添加 robot_logger 子模块：

```cpp
// robot_logger Python 子模块
py::module_ logger_mod = m.def_submodule("robot_logger");
logger_mod.def("info", [](const std::string& msg) {
    LOG_INFO("{}", msg);
});
logger_mod.def("info", [](const std::string& module, const std::string& msg) {
    ::robot_logger::LoggerManager::instance().get(module)->info("{}", msg);
});
logger_mod.def("debug", [](const std::string& msg) {
    LOG_DEBUG("{}", msg);
});
logger_mod.def("debug", [](const std::string& module, const std::string& msg) {
    ::robot_logger::LoggerManager::instance().get(module)->debug("{}", msg);
});
logger_mod.def("warn", [](const std::string& msg) {
    LOG_WARN("{}", msg);
});
logger_mod.def("warn", [](const std::string& module, const std::string& msg) {
    ::robot_logger::LoggerManager::instance().get(module)->warn("{}", msg);
});
logger_mod.def("error", [](const std::string& msg) {
    LOG_ERROR("{}", msg);
});
logger_mod.def("error", [](const std::string& module, const std::string& msg) {
    ::robot_logger::LoggerManager::instance().get(module)->error("{}", msg);
});
logger_mod.def("critical", [](const std::string& msg) {
    LOG_CRITICAL("{}", msg);
});
logger_mod.def("critical", [](const std::string& module, const std::string& msg) {
    ::robot_logger::LoggerManager::instance().get(module)->critical("{}", msg);
});
logger_mod.def("set_level", [](const std::string& level) {
    ::robot_logger::LoggerManager::instance().set_global_level(
        spdlog::level::from_str(level));
});
logger_mod.def("set_level", [](const std::string& module, const std::string& level) {
    ::robot_logger::LoggerManager::instance().set_level(
        module, spdlog::level::from_str(level));
});
logger_mod.def("flush", []() {
    ::robot_logger::LoggerManager::instance().flush();
});
```

3. 保留旧 `py::class_<rclcpp::Logger>` 块，但将其内部 `RCLCPP_*` 调用替换为 `LOG_*`：

```cpp
py::class_<rclcpp::Logger>(m, "Logger")
    .def("info", [](rclcpp::Logger& /*log*/, const std::string& msg) {
        LOG_INFO("{}", msg);
    })
    .def("warn", [](rclcpp::Logger& /*log*/, const std::string& msg) {
        LOG_WARN("{}", msg);
    })
    .def("error", [](rclcpp::Logger& /*log*/, const std::string& msg) {
        LOG_ERROR("{}", msg);
    });
```

- [ ] **Step 4: 迁移 robot_client_node.cpp**

遵循 Task 2 Step 2 中的通用替换规则。

- [ ] **Step 5: 编译验证**

```bash
colcon build --base-paths src --packages-up-to robot_api_python
```

Expected: BUILD SUCCEEDED

- [ ] **Step 6: Commit**

```bash
git add -A
git commit -m "feat(api_python): integrate robot_logger, add Python bindings for LOG_* macros"
```

---

## Task 6: 全量编译验证

- [ ] **Step 1: 清理重建**

```bash
rm -rf build/ install/ log/
colcon build --base-paths src
```

Expected: 所有包 BUILD SUCCEEDED

- [ ] **Step 2: 验证无残留 RCLCPP_* 调用**

```bash
grep -rn "RCLCPP_" src/ --include="*.cpp" --include="*.hpp" | grep -v "test/" | grep -v "docs/"
```

Expected: 零匹配（test/ 文件允许使用 RCLCPP_*）

- [ ] **Step 3: 验证日志文件生成**

运行任意 demo 或简单测试，确认：
- 控制台日志带颜色和模块名
- `/tmp/isaac_ros_logs/` 目录下生成 `isaac_ros_YYYY-MM-DD.log` 文件

- [ ] **Step 4: Commit（如有修复）**

---

## Task 7: 文档更新

**Files:**
- Create: `src/robot_logger/CLAUDE.md`
- Modify: `CLAUDE.md` (根目录)

- [ ] **Step 1: 创建 robot_logger/CLAUDE.md**

内容包含：
1. **包概述**：基于 spdlog 的统一日志系统
2. **核心 API**：LOG_TRACE / LOG_DEBUG / LOG_INFO / LOG_WARN / LOG_ERROR / LOG_CRITICAL 宏 + LOG_*_THROTTLE
3. **LoggerManager 接口**：init(), get(), set_level(), set_global_level(), flush(), shutdown()
4. **LoggerConfig 配置**：各字段含义和默认值
5. **Python 接口**：`robot_api_python.robot_logger.info/debug/warn/error/critical/set_level/flush`
6. **使用规范**：所有日志输出必须走 LOG_* 宏，禁止 std::cout/std::cerr/printf/RCLCPP_*

- [ ] **Step 2: 更新根 CLAUDE.md**

更新以下部分：
1. **包结构表格**：添加 `robot_logger | 统一日志系统（spdlog，宏接口，文件轮转）`
2. **依赖图**：更新为 `robot_msgs → robot_logger → robot_controller → ...`
3. **编译顺序**：`robot_msgs → robot_logger + robot_description → robot_controller → robot_vision + robot_hmi → robot_api_python`
4. **全局约定**：添加「日志规范」章节：

```
### 日志规范
所有日志输出必须使用 `robot_logger` 的 `LOG_*` 宏（C++）或 `robot_logger` 模块（Python）。
禁止使用 `std::cout`、`std::cerr`、`printf`、`RCLCPP_*` 直接输出日志。
测试代码（test/）允许使用 `std::cout`。
```

5. **各包详细文档链接**：添加 `[robot_logger](src/robot_logger/CLAUDE.md)`

- [ ] **Step 3: Commit**

```bash
git add -A
git commit -m "docs: add robot_logger CLAUDE.md and update root CLAUDE.md with logging rules"
```

---

## 完成标准

- [ ] 所有包编译通过（`colcon build --base-paths src` 零错误）
- [ ] 控制台日志输出带颜色和模块名
- [ ] 日志文件自动写入 `/tmp/isaac_ros_logs/`
- [ ] Python 可通过 `robot_api_python.robot_logger.info(...)` 写日志
- [ ] 根 CLAUDE.md 和 robot_logger/CLAUDE.md 文档完整
- [ ] 无残留的 `RCLCPP_*` 调用（test/ 文件除外）
