# robot_logger

## 概述
基于 spdlog 的统一日志系统，作为全项目最底层基础设施。所有包的日志输出必须通过此包的 `LOG_*` 宏（C++）或 `robot_logger` 模块（Python）。

## 核心 API

### C++ 宏接口（`#include <robot_logger/logger.hpp>`）

```cpp
// 基本用法 — 模块名由 CMake 编译宏 ROBOT_LOGGER_MODULE_NAME 自动设置
LOG_TRACE("Detailed trace: val={:.6f}", val);
LOG_DEBUG("Joint positions: {}, {}, {}", j0, j1, j2);
LOG_INFO("Robot initialized with {} DOF", dof);
LOG_WARN("IK seed failed, retrying attempt {}", attempt);
LOG_ERROR("Trajectory timeout after {:.1f}s", elapsed);
LOG_CRITICAL("Motor driver communication lost");

// 节流版本（毫秒间隔，线程安全）
LOG_WARN_THROTTLE(5000, "Still waiting for joint states ({:.1f}s)", elapsed);
LOG_INFO_THROTTLE(2000, "Status: {}", status);

// 指定模块名（覆盖默认）
LOG_INFO("vision", "Detected {} objects", count);
```

### 格式字符串
spdlog/fmt 语法：
- `{}` 代替 `%s`, `%d`, `%u`, `%zu`
- `{:.1f}` 代替 `%.1f`，`{:.3f}` 代替 `%.3f`，以此类推
- spdlog 自动处理 `std::string`，无需 `.c_str()`

### LoggerManager（高级用法）

```cpp
#include <robot_logger/logger.hpp>

// 自定义初始化（可选，不调用则使用默认配置）
robot_logger::LoggerConfig config;
config.log_dir = "/var/log/isaac_ros";
config.level = "debug";
config.enable_file = true;
robot_logger::LoggerManager::instance().init(config);

// 运行时级别控制
robot_logger::LoggerManager::instance().set_global_level(spdlog::level::debug);
robot_logger::LoggerManager::instance().set_level("controller", spdlog::level::trace);

// 强制刷盘
robot_logger::LoggerManager::instance().flush();

// 关闭日志系统
robot_logger::LoggerManager::instance().shutdown();
```

### Python 接口

```python
from robot_api_python._core import robot_logger

# 基本用法（Python 不支持 fmt 格式参数，必须用 f-string 预格式化）
robot_logger.info(f"Grasp plan generated: {count} waypoints")
robot_logger.warn("Detection timeout")
robot_logger.error(f"Service call failed: {msg}")
robot_logger.critical("Cannot connect to robot")

# 指定模块
robot_logger.info("vision", f"Detected {count} objects")

# 运行时级别控制
robot_logger.set_level("debug")              # 全局
robot_logger.set_level("vision", "debug")    # 按模块
robot_logger.flush()                          # 强制刷盘
```

## 配置

### LoggerConfig 结构体

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| log_dir | string | `/tmp/isaac_ros_logs` | 日志文件目录 |
| max_file_size | size_t | 10485760 (10MB) | 单文件最大尺寸 |
| max_files | size_t | 5 | 轮转保留文件数 |
| level | string | `"info"` | 全局默认级别 |
| async_mode | bool | true | 异步写入（当前未实现，始终同步写入） |
| enable_console | bool | true | 终端彩色输出 |
| enable_file | bool | true | 文件持久化 |

### 日志格式

```
[2026-04-29 14:30:15.123] [info] [controller] IK solved in 2.3ms
```

格式：`[时间戳.毫秒] [级别] [模块名] 消息内容`

### 级别体系

| 级别 | 用途 | 默认输出 |
|------|------|----------|
| TRACE | 最详细调试 | 否 |
| DEBUG | 开发调试 | 否 |
| INFO | 正常运行 | **是** |
| WARN | 可恢复异常 | **是** |
| ERROR | 错误 | **是** |
| CRITICAL | 严重错误 | **是** |

## 新包集成步骤

在新包中集成 robot_logger：

1. `package.xml`: 添加 `<depend>robot_logger</depend>`
2. `CMakeLists.txt`:
   ```cmake
   find_package(robot_logger REQUIRED)
   target_link_libraries(your_target robot_logger::robot_logger_lib)
   target_compile_definitions(your_target PUBLIC ROBOT_LOGGER_MODULE_NAME="your_module")
   ```
3. 源文件: `#include <robot_logger/logger.hpp>` 然后使用 `LOG_*` 宏

## 日志规范

**所有日志输出必须使用 `LOG_*` 宏或 Python `robot_logger` 模块。**

禁止：
- `std::cout` / `std::cerr`
- `printf`
- `RCLCPP_INFO` / `RCLCPP_WARN` / `RCLCPP_ERROR` / `RCLCPP_DEBUG`
- `fmt::print`

例外：测试代码（`test/` 目录）允许使用 `std::cout`。

## 构建依赖
- spdlog >= 1.12（系统包 `libspdlog-dev`）
