# robot_bringup

## 职责
集中管理机器人系统的启动文件和全局参数配置。
当前为空壳包，launch/ 目录存在但无文件。

## 节点清单
无可执行节点。

## 话题 / 服务 / Action 接口
无

## 关键参数
无配置文件。

## 启动方式
尚无 launch 文件。当前各节点需手动分别启动：
```bash
# 1. 启动 Isaac Sim（仿真环境）
# 2. 启动控制节点
ros2 run robot_controller robot_controller_node
# 3. 启动视觉节点（可选）
ros2 run robot_vision vision_processor_node
# 4. 启动示教器（可选）
ros2 run robot_hmi robot_hmi
```

## 包内依赖
- **内部依赖**: 无
- **外部依赖**: ament_cmake（仅构建工具）

## 修改指南
- **新增 launch 文件** → 在 `launch/` 目录下创建 `.py` 或 `.xml` launch 文件
- **新增全局参数配置** → 创建 `config/` 目录，添加 `.yaml` 参数文件，并在 `CMakeLists.txt` 中添加 `install(DIRECTORY config/ ...)`

## 注意事项
- `CMakeLists.txt` 已配置 `install(DIRECTORY launch/ ...)`，新增 launch 文件后无需修改构建脚本
- 未来应在此包中创建组合启动文件，统一管理节点启动顺序和参数加载
