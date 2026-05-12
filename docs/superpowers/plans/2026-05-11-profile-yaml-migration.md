# Profile YAML 迁移：消除 robot_controller 中的机器人硬编码

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将 `panda_profile.hpp` 中的硬编码机器人参数迁移到 YAML 配置文件，实现运行时加载，使 `robot_controller` 不再包含特定机器人的硬编码参数。

**Architecture:** 在 `robot_description` 包中新增 YAML profile 配置文件，在 `robot_controller` 的 `robot_kinematics` target 中新增 `ProfileLoader` 类从 YAML 加载 `RobotProfile`/`GripperProfile`。生产代码通过 `ProfileLoader` 加载配置，测试代码直接构造 `RobotProfile` 或使用 `ProfileLoader`。最终删除 `panda_profile.hpp`。

**Tech Stack:** yaml-cpp 0.8（系统已安装），ament_index_cpp（路径解析），CMake find_package

---

## 文件结构变更

| 操作 | 文件 | 职责 |
|------|------|------|
| Create | `robot_description/config/panda_profile.yaml` | Panda 机器人参数（YAML） |
| Create | `robot_controller/include/robot_controller/kinematics/profile_loader.hpp` | `ProfileLoader` 类声明 + `RobotConfig` 结构 |
| Create | `robot_controller/src/kinematics/profile_loader.cpp` | YAML 解析实现 |
| Modify | `robot_description/CMakeLists.txt` | 安装 `config/` 目录 |
| Modify | `robot_controller/CMakeLists.txt` | 添加 yaml-cpp 依赖，添加 profile_loader.cpp |
| Modify | `robot_controller/src/nodes/standalone_main.cpp` | 用 `ProfileLoader` 替换 `panda_profile.hpp` |
| Modify | `robot_controller/test/test_ik_solver.cpp` | 用 `ProfileLoader` 替换 `panda_profile.hpp` |
| Modify | `robot_controller/test/test_motion_controller.cpp` | 用 `ProfileLoader` 替换 `panda_profile.hpp` |
| Modify | `robot_api_python/src/bindings.cpp` | 用 `ProfileLoader` 绑定替换 `profiles::panda()` |
| Modify | `robot_vision/test/test_camera_tf.cpp` | 用 `ProfileLoader` 替换 `panda_profile.hpp` |
| Modify | `robot_demos/demo/demo_grasp_tcp.cpp` | 用 `ProfileLoader` 替换 `panda_profile.hpp` |
| Modify | `robot_demos/test/test_robot_node.cpp` | 用 `ProfileLoader` 替换 `panda_profile.hpp` |
| Delete | `robot_controller/include/robot_controller/profiles/panda_profile.hpp` | 删除硬编码 profile |
| Modify | `robot_controller/CLAUDE.md` | 更新文档 |
| Modify | `robot_description/CLAUDE.md` | 更新文档 |
| Modify | `robot_api_python/CLAUDE.md` | 更新文档 |

---

### Task 1: 创建 YAML Profile 配置文件

**Files:**
- Create: `src/robot_description/config/panda_profile.yaml`
- Modify: `src/robot_description/CMakeLists.txt`

- [ ] **Step 1: 创建 YAML 配置文件**

将 `panda_profile.hpp` 中的所有参数转换为 YAML 格式。`urdf_path` 使用相对于 `robot_description` share 目录的路径。

```yaml
# Franka Panda 机器人参数配置
# urdf_path 相对于 robot_description share 目录

robot:
  name: panda
  urdf_path: urdf/panda.urdf
  dof: 7

  joint_names:
    - panda_joint1
    - panda_joint2
    - panda_joint3
    - panda_joint4
    - panda_joint5
    - panda_joint6
    - panda_joint7

  all_joint_names:
    - panda_joint1
    - panda_joint2
    - panda_joint3
    - panda_joint4
    - panda_joint5
    - panda_joint6
    - panda_joint7
    - panda_finger_joint1
    - panda_finger_joint2

  joint_limits_lower: [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973]
  joint_limits_upper: [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973]

  # 9 个值：7 关节 + 2 夹爪
  home_joints: [0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.0, 0.4, 0.4]

  # IK 默认初始猜测
  ik_default_guess: [0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.0]

  base_frame: panda_link0
  hand_frame: panda_hand

  default_tcp: hand

  tcp_frames:
    hand:
      offset_xyz: [0.0, 0.0, 0.0]
      offset_rpy: [0.0, 0.0, 0.0]
    grasptarget:
      offset_xyz: [0.0, 0.0, 0.105]
      offset_rpy: [0.0, 0.0, 0.0]

  # 关节空间运动极限
  joint_limits:
    max_vel: 1.0    # rad/s
    max_acc: 2.0    # rad/s²
    max_jerk: 10.0  # rad/s³

  # 笛卡尔空间运动极限
  cartesian_limits:
    max_vel: 0.5   # m/s
    max_acc: 1.0   # m/s²
    max_jerk: 5.0  # m/s³

gripper:
  type: parallel
  min_width: 0.0
  max_width: 0.04
  dof: 1
```

- [ ] **Step 2: 更新 `robot_description/CMakeLists.txt` 安装 config 目录**

在 `install(DIRECTORY urdf/ ...)` 后添加：

```cmake
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)
```

- [ ] **Step 3: 编译验证 `robot_description` 安装了 config 文件**

```bash
source install/setup.zsh
colcon build --base-paths src --packages-select robot_description
ls install/robot_description/share/robot_description/config/panda_profile.yaml
```

Expected: 文件存在

- [ ] **Step 4: Commit**

```bash
git add src/robot_description/config/panda_profile.yaml src/robot_description/CMakeLists.txt
git commit -m "feat(description): 添加 Panda 机器人 YAML profile 配置文件"
```

---

### Task 2: 实现 ProfileLoader（含测试）

**Files:**
- Create: `src/robot_controller/include/robot_controller/kinematics/profile_loader.hpp`
- Create: `src/robot_controller/src/kinematics/profile_loader.cpp`
- Create: `src/robot_controller/test/test_profile_loader.cpp`
- Modify: `src/robot_controller/CMakeLists.txt`

- [ ] **Step 1: 创建 `ProfileLoader` 头文件**

`include/robot_controller/kinematics/profile_loader.hpp`:

```cpp
#pragma once

#include <string>
#include "robot_controller/kinematics/robot_profile.hpp"

namespace robot_control {

/// 机器人完整配置（从 YAML 文件加载）
struct RobotConfig {
  RobotProfile robot;
  GripperProfile gripper;
};

/// 从 YAML 文件加载机器人配置
/// YAML 格式参考 robot_description/config/panda_profile.yaml
class ProfileLoader {
 public:
  /// @brief 从 YAML 文件加载机器人配置
  /// @param yaml_path YAML 文件绝对路径
  /// @param urdf_base_dir URDF 基础目录（用于解析相对 urdf_path），为空则使用 yaml_path 所在目录的上级
  /// @return 机器人配置
  static RobotConfig load(const std::string& yaml_path,
                          const std::string& urdf_base_dir = "");
};

}  // namespace robot_control
```

- [ ] **Step 2: 创建 `test_profile_loader.cpp` 测试文件**

先用一个临时 YAML 文件测试，不依赖 `robot_description` 安装。

`test/test_profile_loader.cpp`:

```cpp
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>

#include "robot_controller/kinematics/profile_loader.hpp"

/// 写入临时 YAML 文件用于测试
static std::string write_test_yaml() {
  const std::string path = "/tmp/test_robot_profile.yaml";
  std::ofstream f(path);
  f << R"yaml(
robot:
  name: test_robot
  urdf_path: test.urdf
  dof: 7
  joint_names: [j1, j2, j3, j4, j5, j6, j7]
  all_joint_names: [j1, j2, j3, j4, j5, j6, j7, f1, f2]
  joint_limits_lower: [-1.0, -2.0, -1.0, -3.0, -1.0, -0.5, -1.0]
  joint_limits_upper: [1.0, 2.0, 1.0, -0.1, 1.0, 3.0, 1.0]
  home_joints: [0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.0, 0.04, 0.04]
  ik_default_guess: [0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.0]
  base_frame: base_link
  hand_frame: hand
  default_tcp: hand
  tcp_frames:
    hand:
      offset_xyz: [0.0, 0.0, 0.0]
      offset_rpy: [0.0, 0.0, 0.0]
    grasptarget:
      offset_xyz: [0.0, 0.0, 0.105]
      offset_rpy: [0.0, 0.0, 0.0]
  joint_limits:
    max_vel: 1.0
    max_acc: 2.0
    max_jerk: 10.0
  cartesian_limits:
    max_vel: 0.5
    max_acc: 1.0
    max_jerk: 5.0

gripper:
  type: parallel
  min_width: 0.0
  max_width: 0.04
  dof: 1
)yaml";
  f.close();
  return path;
}

int main() {
  const std::string yaml_path = write_test_yaml();
  auto config = robot_control::ProfileLoader::load(yaml_path, "/fake/base");

  const auto& p = config.robot;
  int errors = 0;

  // 基本信息
  if (p.name != "test_robot") { std::cerr << "FAIL: name\n"; ++errors; }
  if (p.dof != 7) { std::cerr << "FAIL: dof\n"; ++errors; }

  // URDF 路径解析（相对路径应被 base_dir 拼接）
  if (p.urdf_path != "/fake/base/test.urdf") {
    std::cerr << "FAIL: urdf_path = " << p.urdf_path << "\n"; ++errors;
  }

  // 关节名称
  if (p.joint_names.size() != 7) { std::cerr << "FAIL: joint_names size\n"; ++errors; }
  if (p.all_joint_names.size() != 9) { std::cerr << "FAIL: all_joint_names size\n"; ++errors; }

  // 关节限位
  if (std::abs(p.joint_limits_lower[0] - (-1.0)) > 1e-6) { std::cerr << "FAIL: lower limit\n"; ++errors; }
  if (std::abs(p.joint_limits_upper[0] - 1.0) > 1e-6) { std::cerr << "FAIL: upper limit\n"; ++errors; }

  // Home 位
  if (p.home_joints.size() != 9) { std::cerr << "FAIL: home_joints size\n"; ++errors; }
  if (std::abs(p.home_joints[3] - (-1.57)) > 1e-6) { std::cerr << "FAIL: home_joints[3]\n"; ++errors; }

  // IK 默认猜测
  if (p.ik_default_guess.size() != 7) { std::cerr << "FAIL: ik_default_guess size\n"; ++errors; }

  // 坐标系
  if (p.base_frame != "base_link") { std::cerr << "FAIL: base_frame\n"; ++errors; }
  if (p.hand_frame != "hand") { std::cerr << "FAIL: hand_frame\n"; ++errors; }

  // TCP
  if (p.default_tcp != "hand") { std::cerr << "FAIL: default_tcp\n"; ++errors; }
  if (p.tcp_frames.size() != 2) { std::cerr << "FAIL: tcp_frames size\n"; ++errors; }
  if (std::abs(p.tcp_frames.at("grasptarget").offset_xyz[2] - 0.105) > 1e-6) {
    std::cerr << "FAIL: grasptarget offset_xyz\n"; ++errors;
  }

  // 运动极限
  if (std::abs(p.joint_limits.max_vel - 1.0) > 1e-6) { std::cerr << "FAIL: joint max_vel\n"; ++errors; }
  if (std::abs(p.cartesian_limits.max_jerk - 5.0) > 1e-6) { std::cerr << "FAIL: cartesian max_jerk\n"; ++errors; }

  // 夹爪
  const auto& g = config.gripper;
  if (g.type != "parallel") { std::cerr << "FAIL: gripper type\n"; ++errors; }
  if (std::abs(g.max_width - 0.04) > 1e-6) { std::cerr << "FAIL: gripper max_width\n"; ++errors; }
  if (g.dof != 1) { std::cerr << "FAIL: gripper dof\n"; ++errors; }

  if (errors == 0) {
    std::cout << "ALL TESTS PASSED\n";
  } else {
    std::cout << errors << " TESTS FAILED\n";
  }
  return errors;
}
```

- [ ] **Step 3: 实现 `ProfileLoader`**

`src/kinematics/profile_loader.cpp`:

```cpp
#include "robot_controller/kinematics/profile_loader.hpp"

#include <filesystem>
#include <stdexcept>

#include <robot_logger/logger.hpp>
#include <yaml-cpp/yaml.h>

namespace robot_control {

namespace fs = std::filesystem;

RobotConfig ProfileLoader::load(const std::string& yaml_path,
                                 const std::string& urdf_base_dir) {
  YAML::Node doc;
  try {
    doc = YAML::LoadFile(yaml_path);
  } catch (const YAML::Exception& e) {
    throw std::runtime_error("Failed to load profile YAML: " + yaml_path +
                             " - " + e.what());
  }

  RobotConfig config;
  const auto& robot_node = doc["robot"];

  // 基本信息
  config.robot.name = robot_node["name"].as<std::string>();
  config.robot.dof = robot_node["dof"].as<int>();

  // URDF 路径：如果是相对路径，拼接 base_dir
  const auto urdf_rel = robot_node["urdf_path"].as<std::string>();
  if (!urdf_base_dir.empty() && !fs::path(urdf_rel).is_absolute()) {
    config.robot.urdf_path =
        (fs::path(urdf_base_dir) / urdf_rel).generic_string();
  } else {
    config.robot.urdf_path = urdf_rel;
  }

  // 关节名称
  config.robot.joint_names =
      robot_node["joint_names"].as<std::vector<std::string>>();
  config.robot.all_joint_names =
      robot_node["all_joint_names"].as<std::vector<std::string>>();

  // 关节限位
  config.robot.joint_limits_lower =
      robot_node["joint_limits_lower"].as<std::vector<double>>();
  config.robot.joint_limits_upper =
      robot_node["joint_limits_upper"].as<std::vector<double>>();

  // Home + IK guess
  config.robot.home_joints =
      robot_node["home_joints"].as<std::vector<double>>();
  config.robot.ik_default_guess =
      robot_node["ik_default_guess"].as<std::vector<double>>();

  // 坐标系
  config.robot.base_frame = robot_node["base_frame"].as<std::string>();
  config.robot.hand_frame = robot_node["hand_frame"].as<std::string>();
  config.robot.default_tcp = robot_node["default_tcp"].as<std::string>();

  // TCP frames
  const auto& tcp_node = robot_node["tcp_frames"];
  for (const auto& entry : tcp_node) {
    const std::string name = entry.first.as<std::string>();
    const auto& frame = entry.second;
    TcpConfig cfg;
    auto xyz = frame["offset_xyz"].as<std::vector<double>>();
    auto rpy = frame["offset_rpy"].as<std::vector<double>>();
    std::copy_n(xyz.begin(), 3, cfg.offset_xyz.begin());
    std::copy_n(rpy.begin(), 3, cfg.offset_rpy.begin());
    config.robot.tcp_frames[name] = cfg;
  }

  // 运动极限
  const auto& jl = robot_node["joint_limits"];
  config.robot.joint_limits = {
      jl["max_vel"].as<double>(),
      jl["max_acc"].as<double>(),
      jl["max_jerk"].as<double>(),
  };
  const auto& cl = robot_node["cartesian_limits"];
  config.robot.cartesian_limits = {
      cl["max_vel"].as<double>(),
      cl["max_acc"].as<double>(),
      cl["max_jerk"].as<double>(),
  };

  // 夹爪
  const auto& gripper_node = doc["gripper"];
  config.gripper.type = gripper_node["type"].as<std::string>();
  config.gripper.min_width = gripper_node["min_width"].as<double>();
  config.gripper.max_width = gripper_node["max_width"].as<double>();
  config.gripper.dof = gripper_node["dof"].as<int>();

  return config;
}

}  // namespace robot_control
```

- [ ] **Step 4: 更新 `robot_controller/CMakeLists.txt`**

添加 `yaml-cpp` 依赖，将 `profile_loader.cpp` 加入 `robot_kinematics` target，添加测试可执行文件。

在 `find_package` 区域添加：
```cmake
find_package(yaml-cpp REQUIRED)
```

在 `robot_kinematics` target 的源文件列表中添加 `src/kinematics/profile_loader.cpp`：
```cmake
add_library(robot_kinematics SHARED
  src/kinematics/ik_solver.cpp
  src/kinematics/trajectory_planner.cpp
  src/kinematics/profile_loader.cpp
)
```

在 `robot_kinematics` 的 `ament_target_dependencies` 后添加：
```cmake
target_link_libraries(robot_kinematics yaml-cpp)
```

在 `ament_export_dependencies(...)` 块中添加 `yaml-cpp`，使下游包能传递解析 yaml-cpp：
```cmake
ament_export_dependencies(
  robot_description
  rclcpp
  # ... 已有的依赖 ...
  yaml-cpp
)
```

添加测试可执行文件（在已有测试区块之后）：
```cmake
add_executable(test_profile_loader
  test/test_profile_loader.cpp
)
target_link_libraries(test_profile_loader robot_kinematics robot_logger::robot_logger_lib)
target_compile_definitions(test_profile_loader PRIVATE ROBOT_LOGGER_MODULE_NAME="controller")
```

在 `install(TARGETS ...)` 中将 `test_profile_loader` 加入已有测试 target 安装列表。

- [ ] **Step 5: 编译并运行测试**

```bash
source install/setup.zsh
colcon build --base-paths src --packages-select robot_controller
./install/robot_controller/lib/robot_controller/test_profile_loader
```

Expected: `ALL TESTS PASSED`

- [ ] **Step 6: 运行原有 IK 测试验证无回归**

```bash
./install/robot_controller/lib/robot_controller/test_ik_solver
```

Expected: `ALL TESTS PASSED`

- [ ] **Step 7: Commit**

```bash
git add src/robot_controller/include/robot_controller/kinematics/profile_loader.hpp \
        src/robot_controller/src/kinematics/profile_loader.cpp \
        src/robot_controller/test/test_profile_loader.cpp \
        src/robot_controller/CMakeLists.txt
git commit -m "feat(controller): 添加 ProfileLoader，从 YAML 加载机器人参数"
```

---

### Task 3: 迁移 standalone_main.cpp

**Files:**
- Modify: `src/robot_controller/src/nodes/standalone_main.cpp`

- [ ] **Step 1: 替换 profile 加载方式**

将 `standalone_main.cpp` 中的 `#include "robot_controller/profiles/panda_profile.hpp"` 替换为：

```cpp
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "robot_controller/kinematics/profile_loader.hpp"
```

将 `main()` 中的：
```cpp
auto profile = robot_control::profiles::panda();
auto gripper = robot_control::profiles::panda_gripper();
```

替换为：
```cpp
const auto desc_dir =
    ament_index_cpp::get_package_share_directory("robot_description");
const auto config = robot_control::ProfileLoader::load(
    desc_dir + "/config/panda_profile.yaml", desc_dir);
```

并将 `RobotControllerNode::create` 调用改为：
```cpp
auto node = robot_control::RobotControllerNode::create(
    config.robot, config.gripper, robot_control::TopicConfig());
```

- [ ] **Step 2: 编译并验证节点可启动**

```bash
colcon build --base-paths src --packages-select robot_controller
# 验证可执行文件存在
ls install/robot_controller/lib/robot_controller/robot_controller_node
```

Expected: 文件存在（无需 Isaac Sim 做完整功能测试）

- [ ] **Step 3: Commit**

```bash
git add src/robot_controller/src/nodes/standalone_main.cpp
git commit -m "refactor(controller): standalone_main 使用 ProfileLoader 加载 YAML 配置"
```

---

### Task 4: 迁移 robot_controller 内部测试

**Files:**
- Modify: `src/robot_controller/test/test_ik_solver.cpp`
- Modify: `src/robot_controller/test/test_motion_controller.cpp`

- [ ] **Step 1: 更新 `test_ik_solver.cpp`**

替换 `#include "robot_controller/profiles/panda_profile.hpp"` 为：
```cpp
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "robot_controller/kinematics/profile_loader.hpp"
```

替换 `auto profile = robot_control::profiles::panda();` 为：
```cpp
const auto desc_dir =
    ament_index_cpp::get_package_share_directory("robot_description");
const auto config = robot_control::ProfileLoader::load(
    desc_dir + "/config/panda_profile.yaml", desc_dir);
const auto& profile = config.robot;
```

- [ ] **Step 2: 更新 `test_motion_controller.cpp`**

替换 `#include "robot_controller/profiles/panda_profile.hpp"` 为：
```cpp
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "robot_controller/kinematics/profile_loader.hpp"
```

将所有 `robot_control::profiles::panda()` 替换为从 `ProfileLoader` 加载的 profile。

**注意：不要使用 namespace-scope static 变量**（在 `main()` 执行前初始化，失败时无法给出有意义的错误信息）。改为在 `main()` 函数开头加载一次：

```cpp
int main() {
  const auto desc_dir =
      ament_index_cpp::get_package_share_directory("robot_description");
  const auto g_config = robot_control::ProfileLoader::load(
      desc_dir + "/config/panda_profile.yaml", desc_dir);

  // ... 后续测试使用 g_config.robot, g_config.gripper ...
}
```

然后使用 `g_config.robot` 和 `g_config.gripper` 替换所有 `profiles::panda()` / `profiles::panda_gripper()` 调用。`TestFixture` 的成员初始化列表中的 `profile(robot_control::profiles::panda())` 改为 `profile(g_config.robot)`。

- [ ] **Step 3: 运行全部 robot_controller 测试验证无回归**

```bash
source install/setup.zsh
colcon build --base-paths src --packages-select robot_controller
./install/robot_controller/lib/robot_controller/test_ik_solver
./install/robot_controller/lib/robot_controller/test_motion_controller
./install/robot_controller/lib/robot_controller/test_trajectory_planner
```

Expected: 全部 PASSED

- [ ] **Step 4: Commit**

```bash
git add src/robot_controller/test/test_ik_solver.cpp \
        src/robot_controller/test/test_motion_controller.cpp
git commit -m "refactor(controller): 测试使用 ProfileLoader 加载 YAML 配置"
```

---

### Task 5: 迁移 robot_api_python 绑定

**Files:**
- Modify: `src/robot_api_python/src/bindings.cpp`

- [ ] **Step 1: 替换 profile 绑定**

移除 `#include "robot_controller/profiles/panda_profile.hpp"`。

添加 `#include "robot_controller/kinematics/profile_loader.hpp"` 和 `#include <ament_index_cpp/get_package_share_directory.hpp>`。

在绑定代码中：
- 将 `profiles` 子模块的 `profiles.def("panda", ...)` 和 `profiles.def("panda_gripper", ...)` 删除，同时删除 `auto profiles = m.def_submodule("profiles")` 行
- 添加 `RobotConfig` 类绑定和 `load_profile` 顶层函数绑定：

> **Breaking API Change:** `rc.profiles.panda()` → `rc.load_profile("panda")`
> 旧 API 直接删除，不保留向后兼容包装。

```cpp
// 在 RobotConfig 绑定后添加
py::class_<RobotConfig>(m, "RobotConfig")
    .def_readonly("robot", &RobotConfig::robot)
    .def_readonly("gripper", &RobotConfig::gripper);

// 便捷函数：从 robot_description 包加载指定 profile
m.def("load_profile", [](const std::string& profile_name) {
  const auto desc_dir =
      ament_index_cpp::get_package_share_directory("robot_description");
  return ProfileLoader::load(
      desc_dir + "/config/" + profile_name + "_profile.yaml", desc_dir);
}, py::arg("profile_name") = "panda",
   "Load robot config from YAML. Defaults to 'panda'.");
```

- [ ] **Step 2: 编译 robot_api_python 验证无报错**

```bash
source install/setup.zsh
colcon build --base-paths src --packages-select robot_api_python
```

Expected: 编译成功

- [ ] **Step 3: Commit**

```bash
git add src/robot_api_python/src/bindings.cpp
git commit -m "refactor(api_python): 用 load_profile() 替换硬编码 profiles.panda()"
```

---

### Task 6: 迁移外部包测试和 Demo

**Files:**
- Modify: `src/robot_vision/test/test_camera_tf.cpp`
- Modify: `src/robot_demos/demo/demo_grasp_tcp.cpp`
- Modify: `src/robot_demos/test/test_robot_node.cpp`

这些文件的模式相同：将 `#include "robot_controller/profiles/panda_profile.hpp"` 替换为 `ProfileLoader` 加载。

> **注意：** `robot_vision` 和 `robot_demos` 的 `CMakeLists.txt` 不需要修改。
> `ProfileLoader` 的 yaml-cpp 依赖封装在 `robot_kinematics.so` 中，通过
> `robot_nodes` → `robot_motion` → `robot_kinematics` 的链接链传递可用。

- [ ] **Step 1: 更新 `test_camera_tf.cpp`**

同 Task 4 的替换模式：`ProfileLoader::load(desc_dir + "/config/panda_profile.yaml", desc_dir)`。

- [ ] **Step 2: 更新 `demo_grasp_tcp.cpp`**

同上。

- [ ] **Step 3: 更新 `test_robot_node.cpp`**

同上。

- [ ] **Step 4: 编译全部受影响包验证无报错**

```bash
source install/setup.zsh
colcon build --base-paths src --packages-up-to robot_demos
```

Expected: 编译成功

- [ ] **Step 5: Commit**

```bash
git add src/robot_vision/test/test_camera_tf.cpp \
        src/robot_demos/demo/demo_grasp_tcp.cpp \
        src/robot_demos/test/test_robot_node.cpp
git commit -m "refactor(vision,demos): 迁移至 ProfileLoader YAML 加载"
```

---

### Task 7: 删除 panda_profile.hpp 并更新文档

**Files:**
- Delete: `src/robot_controller/include/robot_controller/profiles/panda_profile.hpp`
- Modify: `src/robot_controller/CLAUDE.md`
- Modify: `src/robot_description/CLAUDE.md`
- Modify: `src/robot_api_python/CLAUDE.md`

- [ ] **Step 1: 删除 `panda_profile.hpp`**

```bash
rm src/robot_controller/include/robot_controller/profiles/panda_profile.hpp
```

同时检查 profiles/ 目录是否为空，若为空则删除整个 profiles/ 目录。

- [ ] **Step 2: 全量编译验证**

```bash
source install/setup.zsh
colcon build --base-paths src
```

Expected: 编译成功，无找不到 `panda_profile.hpp` 的错误

- [ ] **Step 3: 更新 `robot_controller/CLAUDE.md`**

修改内容：
1. 删除 `| \`include/.../profiles/panda_profile.hpp\` | profiles::panda(), profiles::panda_gripper() | Panda 机器人参数 |` 这一行
2. 在 kinematics 层表格中添加 ProfileLoader 条目：
   `| \`include/.../kinematics/profile_loader.hpp\` | ProfileLoader, RobotConfig | 从 YAML 加载 RobotProfile/GripperProfile |`
3. 修改「新增机器人」指南：从「在 profiles/ 下添加 xxx_profile.hpp」改为「在 robot_description/config/ 下添加 xxx_profile.yaml」
4. 在关键参数章节提到 profile 来自 YAML

- [ ] **Step 4: 更新 `robot_description/CLAUDE.md`**

添加 `config/` 目录说明：
- 新增 `## Profile 配置` 章节，说明 YAML profile 文件格式和用途
- 在修改指南中添加：`**新增机器人 Profile** → 在 config/ 目录下新增 xxx_profile.yaml`

- [ ] **Step 5: 更新 `robot_api_python/CLAUDE.md`**

1. 在「预定义 Profile」表格中，将 `profiles.panda()` / `profiles.panda_gripper()` 替换为 `load_profile(name="panda")` → 返回 `RobotConfig`
2. 更新使用模式中的代码示例

- [ ] **Step 6: Commit**

```bash
git add -A
git commit -m "chore: 删除 panda_profile.hpp，更新文档至 YAML profile 加载方式"
```

---

## 验证清单

- [ ] `test_profile_loader` 通过
- [ ] `test_ik_solver` 通过
- [ ] `test_motion_controller` 通过
- [ ] `test_trajectory_planner` 通过
- [ ] `colcon build --base-paths src` 全量编译成功
- [ ] `grep -r "panda_profile" src/` 无匹配（确认彻底删除）
- [ ] `grep -r "profiles::panda" src/` 无匹配
