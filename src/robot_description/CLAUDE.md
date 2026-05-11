# robot_description

## 职责
提供机器人的 URDF 模型描述文件，供运动学求解器（KDL）、仿真器（Isaac Sim）和可视化工具使用。
当前包含 Franka Panda 7-DOF 机械臂 + 二指夹爪 + ZED_X_Mini 相机的完整模型。

## 节点清单
无可执行节点。纯数据包，安装 URDF 文件到 `share/` 目录供其他包 `find_package` 引用。

## 话题 / 服务 / Action 接口
无

## 关键参数
无 YAML 配置。

## Profile 配置

YAML profile 文件定义机器人的运动学参数（关节限位、运动极限、TCP 配置等），
供 `robot_controller` 的 `ProfileLoader` 在运行时加载。

| 文件 | 说明 |
|------|------|
| `config/panda_profile.yaml` | Franka Panda 7-DOF 参数 |

### Profile 字段说明
- `robot`: 机器人参数（name, urdf_path, dof, joint_names, limits, tcp_frames 等）
- `gripper`: 夹爪参数（type, min_width, max_width, dof）

详细字段参考 `config/panda_profile.yaml` 文件内容。

## URDF 模型详情（panda.urdf）

### 链接结构
| Link | 类型 | 说明 |
|------|------|------|
| panda_link0 | 固定基座 | 机器人底座 |
| panda_link1 ~ panda_link7 | 7 个运动链 | 7-DOF 串联臂 |
| panda_link8 | 固定法兰 | 手腕末端法兰 |
| panda_hand | 固定 | 夹爪基座 |
| panda_leftfinger / panda_rightfinger | 棱柱关节 | 二指夹爪（0~0.04m） |
| panda_grasptarget | 固定 | 抓取目标参考点 |
| camera_link | 固定 | ZED_X_Mini 相机安装点（eye-in-hand） |
| camera_color_optical_frame | 固定 | 相机光学坐标系 |

### 关节
- **7 个旋转关节**（panda_joint1~7）: 含安全控制器 + 软/硬限位
- **2 个棱柱关节**: panda_finger_joint1（主动）、panda_finger_joint2（ mimic 跟随）
- **5 个固定关节**: panda_joint8, panda_hand_joint, 相机安装(panda_hand_camera_joint), 光学帧(camera_color_optical_joint), 抓取目标(panda_grasptarget_hand)

### 相机安装参数
- 安装位置（相对 panda_hand）: `xyz="0.025 -0.015 0.015"`, `rpy="3.14159265359 0 -1.57079632679"`
- 左目光心，eye-in-hand 配置，与 Isaac Sim 中查询到的 local transform 一致
- 外参统一配置于 `include/robot_description/camera_config.hpp` 中的 `CameraExtrinsics`

### 夹爪接触属性
| 参数 | 值 | 说明 |
|------|-----|------|
| stiffness | 30000.0 N/m | 接触刚度 |
| damping | 1000.0 Ns/m | 接触阻尼 |
| spinning_friction | 0.1 | 旋转摩擦 |
| lateral_friction | 1.0 | 侧向摩擦 |

### 网格文件引用
URDF 引用外部 mesh 文件（不在本包内）：
- 可视化: `package://meshes/visual/link[0-7].obj`, `hand.obj`, `finger.obj`
- 碰撞: `package://meshes/collision/link[0-7].obj`, `hand.obj`, `finger.obj`

## 启动方式
无可执行节点。其他包通过 `ament_index` 查找 URDF 路径：
```cpp
std::string urdf_path = ament_index_cpp::get_package_share_directory("robot_description") + "/urdf/panda.urdf";
```

## 包内依赖
- **内部依赖**: 无
- **外部依赖**: ament_cmake（仅构建工具）

## 修改指南
- **修改机器人几何参数** → 编辑 `urdf/panda.urdf` 中对应 link 的 inertial/visual/collision 属性
- **修改关节限位** → 编辑对应 joint 的 `<limit>` 和 `<safety_controller>` 标签
- **更换相机型号** → 修改 `camera_link` 的安装位姿和光学帧变换
- **新增机器人模型** → 在 `urdf/` 目录下新增 `.urdf` 文件，并在 `CMakeLists.txt` 的 `install(DIRECTORY)` 中确认包含
- **新增机器人 Profile** → 在 `config/` 目录下新增 `xxx_profile.yaml`，参考 `panda_profile.yaml` 格式

## 注意事项
- URDF 为 xacro 自动生成产物，如需参数化建议改用 xacro + 配置文件
- 网格文件通过 `package://meshes/` 引用，需确保对应包已安装
- panda_finger_joint2 为 mimic 关节（跟随 finger_joint1），运动学链中只需控制 1 个夹爪 DOF
- `robot_controller` 的 `IKSolver` 通过 `robot_description` 包的 URDF 构建 KDL 运动链
