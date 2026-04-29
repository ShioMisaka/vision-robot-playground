# robot_logger — 统一日志系统设计

## 背景与动机

项目当前使用 ROS2 原生 `RCLCPP_*` 宏进行日志输出，存在以下问题：
- 非节点类通过 `rclcpp::get_logger("name")` 获取 logger，风格不统一
- 无文件持久化，进程结束日志丢失
- 无运行时级别调节能力
- 无统一格式（时间戳、线程 ID、模块名不完整）
- Layer 1 纯 C++ 核心库依赖 rclcpp 仅为了日志

设计目标：新建 `robot_logger` 包作为全项目最底层基础设施，统一所有日志输出。

## 方案决策

| 决策点 | 选择 | 理由 |
|--------|------|------|
| 底层库 | spdlog | 高性能、异步支持、文件轮转内置、C++17 原生、头文件库 |
| 架构 | 全局单例 + 宏接口 | 改造量最小，206 处调用只需替换宏名和格式字符串 |
| 包归属 | 新建 robot_logger 包 | 职责独立，依赖链最底层 |
| Python 支持 | C++ + Python 统一 | pybind11 绑定 spdlog，Python 日志也走同一管道 |

## 包结构

```
src/robot_logger/
├── CMakeLists.txt
├── package.xml
├── CLAUDE.md
├── include/robot_logger/
│   ├── logger.hpp          # 主头文件（LoggerManager 单例 + 宏定义）
│   └── logger_config.hpp   # LoggerConfig 配置结构体
└── src/
    └── logger_manager.cpp   # LoggerManager 实现
```

Python 绑定代码放在 `robot_api_python` 的 `src/` 中（复用现有 pybind11 编译链）。

## 依赖链变化

```
robot_msgs
    ↓
robot_logger  ← 新增，仅依赖 spdlog（header-only）
    ↓
robot_controller
    ↓
robot_vision → robot_api_python
    ↓
robot_hmi
```

编译顺序变为：`robot_msgs → robot_logger → robot_description → robot_controller → ...`

## 核心 API

### 宏接口（`logger.hpp`）

```cpp
#include <robot_logger/logger.hpp>

// 基本用法 — 模块名从编译单元推导或手动指定
LOG_INFO("Robot initialized with {} DOF", dof);
LOG_WARN("IK seed failed, retrying with config {}", attempt);
LOG_ERROR("Trajectory timeout after {:.1f}s", elapsed);
LOG_DEBUG("Joint positions: [{}, {}, {}, {}, {}, {}, {}]",
          j[0], j[1], j[2], j[3], j[4], j[5], j[6]);
LOG_TRACE("IK iteration {}: error = {:.6f}", i, err);

// 指定模块名（覆盖默认推导）
LOG_INFO("vision", "Detected {} objects", count);
```

宏在内部调用 `LoggerManager::instance().get(module)->info(...)` 等方法。

### 配置结构体（`logger_config.hpp`）

```cpp
namespace robot_logger {

struct LoggerConfig {
    std::string log_dir = "/tmp/isaac_ros_logs";  // 日志目录
    size_t max_file_size = 10 * 1024 * 1024;       // 单文件 10MB
    size_t max_files = 5;                           // 保留 5 个轮转文件
    std::string level = "info";                     // 全局默认级别
    bool async_mode = true;                         // 异步写入模式
    bool enable_console = true;                     // 终端彩色输出
    bool enable_file = true;                        // 文件持久化
};

}  // namespace robot_logger
```

### LoggerManager 单例（`logger.hpp` / `logger_manager.cpp`）

```cpp
namespace robot_logger {

class LoggerManager {
public:
    static LoggerManager& instance();

    // 初始化（可选，不调用则首次 LOG_* 时自动以默认配置初始化）
    void init(const LoggerConfig& config = {});

    // 获取模块 logger（首次调用时自动创建）
    std::shared_ptr<spdlog::logger> get(const std::string& module);

    // 运行时级别控制
    void set_level(const std::string& module, spdlog::level::level_enum level);
    void set_global_level(spdlog::level::level_enum level);

    // 强制刷盘
    void flush();

    // 关闭（进程退出时自动调用）
    void shutdown();

private:
    LoggerManager();
    ~LoggerManager();
    // 禁止拷贝
    LoggerManager(const LoggerManager&) = delete;
    LoggerManager& operator=(const LoggerManager&) = delete;
};

}  // namespace robot_logger
```

## 日志格式

### 统一格式

```
[2026-04-29 14:30:15.123] [info] [controller] IK solved in 2.3ms
[2026-04-29 14:30:15.456] [warn] [vision] Detection confidence low: 0.32
[2026-04-29 14:30:16.789] [error] [hmi] Service call timeout: /move_j (5000ms)
```

格式模板：`[YYYY-MM-DD HH:MM:SS.mmm] [级别] [模块名] 消息内容`

- **控制台**：带 ANSI 颜色（DEBUG=青, INFO=白, WARN=黄, ERROR=红）
- **文件**：纯文本，无颜色转义码

### 文件命名与轮转

- 文件名：`isaac_ros_YYYY-MM-DD.log`
- 轮转后：`isaac_ros_YYYY-MM-DD_1.log`, `_2.log`, ...
- 默认保留 5 个文件，单文件最大 10MB
- 日志目录默认：`/tmp/isaac_ros_logs`

## 级别体系

| 级别 | 用途 | 默认是否输出 |
|------|------|-------------|
| TRACE | 最详细的调试信息，排查特定问题时开启 | 否 |
| DEBUG | 开发阶段常用调试信息 | 否 |
| INFO | 正常运行信息 | **是** |
| WARN | 可恢复的异常情况 | **是** |
| ERROR | 需要关注的错误 | **是** |
| OFF | 关闭日志 | — |

### 运行时级别控制

```cpp
// 全局调节
robot_logger::LoggerManager::instance().set_global_level(spdlog::level::debug);

// 按模块调节（排查特定模块问题时非常有用）
robot_logger::LoggerManager::instance().set_level("controller", spdlog::level::trace);
robot_logger::LoggerManager::instance().set_level("vision", spdlog::level::debug);
```

## Python 接口

绑定代码位于 `robot_api_python/src/` 中，复用现有 pybind11 编译链。

```python
from robot_api_python import logger

# 基本用法
logger.info("Grasp plan generated: {} waypoints", len(waypoints))
logger.debug("TCP pose: {}", tcp_pose)
logger.warn("Detection timeout, retrying...")
logger.error("Service call failed: {}", msg)

# 指定模块
logger.info("vision", "Detected {} objects", count)

# 运行时级别控制
logger.set_level("debug")              # 全局
logger.set_level("vision", "debug")    # 按模块
logger.flush()                          # 强制刷盘
```

Python 调用通过 pybind11 直接走 C++ spdlog 后端，日志汇集到同一输出。

## 迁移策略

### 迁移映射

| 现有调用 | 替换为 | 备注 |
|---------|--------|------|
| `RCLCPP_INFO(this->get_logger(), "msg %s", val)` | `LOG_INFO("msg {}", val)` | `%s`/`%d` → `{}` |
| `RCLCPP_INFO(rclcpp::get_logger("xxx"), "msg")` | `LOG_INFO("msg")` | 去掉 logger 参数 |
| `RCLCPP_WARN(...)` | `LOG_WARN(...)` | 同上 |
| `RCLCPP_ERROR(...)` | `LOG_ERROR(...)` | 同上 |
| `RCLCPP_DEBUG(...)` | `LOG_DEBUG(...)` | 同上 |
| `RCLCPP_INFO_ONCE(...)` | 手动加静态变量 | spdlog 无内置 ONCE，需自行处理 |

### 影响范围

- `robot_controller`: ~38 处调用
- `robot_vision`: ~52 处调用
- `robot_hmi`: ~6 处调用
- `robot_api_python`: ~6 处调用
- **总计**: ~102 处替换

### 格式字符串差异

spdlog 使用 [fmt 库](https://github.com/fmtlib/fmt) 格式语法：
- 位置参数：`{}` 代替 `%s`, `%d`, `%f`
- 命名参数：`{name}`
- 格式说明：`{:.1f}` 代替 `%.1f`
- 宽度/对齐：`{:>10}` 等

## 异步模式与性能

- 异步模式下，日志消息先写入无锁环形缓冲区，后台线程负责格式化和写文件
- 100Hz 控制循环中 `LOG_DEBUG` 调用耗时可忽略不计（纳秒级入队）
- 缓冲区满时策略：丢弃新消息（不阻塞调用线程）
- `LoggerManager::flush()` 用于关键路径后强制刷盘

## ROS2 集成（后续扩展）

初版不实现 ROS2 sink。日志统一走 spdlog。

后续如需 `/rosout` 兼容（`ros2 topic echo /rosout` 可见），可添加自定义 spdlog sink 将日志同时发布到 ROS2 topic。此功能设计为可选开关：

```cpp
LoggerConfig config;
config.enable_ros2 = true;  // 默认 false
```

## agent 开发规范

设计完成后，在 `robot_logger/CLAUDE.md` 和根 `CLAUDE.md` 中添加以下规则：

> **所有日志输出必须使用 `robot_logger` 的 `LOG_*` 宏或 Python `logger` 接口。禁止使用 `std::cout`、`std::cerr`、`printf`、`RCLCPP_*` 直接输出日志。测试代码（test/）允许使用 `std::cout`。**
