# robot_bringup

## 职责
集中管理机器人系统的启动文件和全局参数配置。
提供组合 launch 文件，统一管理节点启动顺序和参数加载。

## 节点清单
无可执行节点。纯 launch 配置包。

## 话题 / 服务 / Action 接口
无

## 关键参数
无 YAML 配置文件。

## Launch 文件

### controller.launch.py
启动 `robot_controller_node`（独立控制器节点）。
- 节点: `robot_controller/robot_controller_node`
- 前提: Isaac Sim 已运行并发布 `/joint_states`

```bash
ros2 launch robot_bringup controller.launch.py
```

### full_system.launch.py
启动控制器 + Qt5 示教器（完整系统）。
- 节点: `robot_controller_node` + `robot_hmi`
- 前提: Isaac Sim 已运行并发布 `/joint_states`

```bash
ros2 launch robot_bringup full_system.launch.py
```

## 启动方式
```bash
# 方式一：使用 launch 文件（推荐）
source install/setup.zsh
ros2 launch robot_bringup controller.launch.py   # 仅控制器
ros2 launch robot_bringup full_system.launch.py  # 控制器 + 示教器

# 方式二：手动分别启动
ros2 run robot_controller robot_controller_node  # 控制器
ros2 run robot_hmi robot_hmi                     # 示教器
```

## 包内依赖
- **内部依赖**: robot_controller, robot_hmi
- **外部依赖**: ament_cmake（仅构建工具）

## 修改指南
- **新增 launch 文件** → 在 `launch/` 目录下创建 `.py` launch 文件
- **新增全局参数配置** → 创建 `config/` 目录，添加 `.yaml` 参数文件，并在 `CMakeLists.txt` 中添加 `install(DIRECTORY config/ ...)`

## 注意事项
- `CMakeLists.txt` 已配置 `install(DIRECTORY launch/ ...)`，新增 launch 文件后无需修改构建脚本
- launch 文件假设 Isaac Sim 已在外部启动，不负责启动仿真环境
