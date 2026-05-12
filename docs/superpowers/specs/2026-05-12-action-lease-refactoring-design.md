# ROS2 通信层重构：Action 化 + 租约模型 + Client Library 合并

**日期**: 2026-05-12
**状态**: 已确认

## 背景

当前系统存在三个架构层面的问题：

1. **运动指令用 Service 实现耗时操作**：moveJ/moveL/GoHome 耗时 1-15 秒，用 Service 无法反馈进度、无法取消、阻塞前端。
2. **MotionOwner 只有三个值**（kNone/kPendant/kScript），无法支持多前端并发控制，缺少注册和仲裁机制。
3. **robot_api_cpp 作为独立包设计不合理**：随着机器人和相机种类增加，API 包会膨胀且职责不清。

## 目标

- 运动指令从 Service 迁移到 Action，支持进度反馈和取消
- 引入租约模型（Lease）替代 MotionOwner，支持多前端安全仲裁
- 将 robot_api_cpp 合并进 robot_controller 作为 client 子库
- IRobotController 接口保留全部现有方法签名，新增租约管理方法，移除 MotionSource 参数

## 设计决策

| 项目 | 决策 |
|------|------|
| 包结构 | robot_api_cpp 合并进 robot_controller/robot_client，删除原包 |
| MotionOwner | 租约模型：AcquireControl/ReleaseControl/RenewLease + session_id |
| Jog/流控 | 独立的 TeachingMode Service，不改 Topic 消息定义 |
| 运动指令 | Service → Action（MoveJ/MoveL/GoHome） |
| 抓取任务 | GraspTask Action Server（robot_tasks 提供） |
| 租约超时 | 立即减速停车，安全优先 |
| Action 取消 | 前端主动 Cancel → 平滑减速；E-STOP → FAULT |
| Client Library | ActionRobotController 替代 ServiceRobotController，IRobotController 保留全部方法 + 新增租约/进度回调 |

## 零、接口迁移策略

### ABI 兼容性声明

修改 `.action` 和 `.srv` 文件（添加/删除字段）会破坏 ABI 兼容性，**必须全量重编译所有依赖包**。迁移策略：

1. Phase 1 修改 robot_msgs 后，执行 `colcon build --base-paths src` 全量编译
2. 不保留旧接口的兼容别名，一步切换
3. 所有依赖包在 Phase 2-4 同步适配

### 已有 .action 文件处理

`MoveJ.action` 和 `MoveL.action` 已存在于 `src/robot_msgs/action/`，当前未被任何节点实现。修改方式：
- 直接在现有文件中添加 `string session_id` 字段
- 无需保留无 session_id 的旧版本（因为从未被实现过）

## 一、整体架构与包结构

### 包结构变更

```
重构前                              重构后
──────────────                      ────────
robot_msgs                          robot_msgs (扩展)
robot_logger                        robot_logger (不变)
robot_description                   robot_description (不变)
robot_controller                    robot_controller (扩容)
  ├── robot_kinematics                ├── robot_kinematics (不变)
  ├── robot_motion                    ├── robot_motion (不变)
  └── robot_nodes                     ├── robot_nodes (重构: Action Server)
robot_vision                          ├── robot_client (新增: 替代 robot_api_cpp)
  ├── robot_vision_core             robot_vision (不变)
  └── robot_vision_nodes            robot_tasks (扩展)
robot_api_cpp ← 删除                  ├── robot_tasks_lib (改用 robot_client)
  ├── RobotClient                     └── Action Server (新增: GraspTask)
  └── ServiceRobotController        robot_hmi (重构 PendantNode)
robot_tasks                          robot_api_python (改绑 robot_client)
robot_hmi                            robot_demos (改用 robot_client)
robot_api_python                     robot_bringup (不变)
robot_demos
robot_bringup
```

### 依赖关系

```
robot_msgs (接口定义)
    ▲
    │
robot_description + robot_logger
    ▲
    │
robot_controller
  ├── robot_kinematics (零 ROS 依赖)
  ├── robot_motion (依赖 kinematics)
  ├── robot_nodes (Action Server + Service Server, 依赖 motion)
  └── robot_client (Action/Service Client 封装, 仅依赖 robot_msgs + robot_motion)
    ▲            ▲                  ▲            ▲
    │            │                  │            │
    ▼            ▼                  ▼            ▼
robot_tasks   robot_hmi         robot_demos   robot_api_python
(controller    (controller +      (controller   (controller
  + vision)      vision)            + vision)      only)
```

核心约束：**robot_controller 和 robot_vision 互不依赖**，它们是平行后端。上层包各自按需依赖其中一个或两个。

依赖深度差异：
- robot_tasks 对 vision 是 build 依赖（链接 robot_vision_core，调用 IVisionProcessor）
- robot_hmi 对 vision 目前是 runtime 依赖（订阅 Topic，不需要链接 vision 库）

## 二、租约模型

### 租约生命周期

```
                    AcquireControl()
前端 ──────────────────────────────────→ 控制器
                    ← session_id + lease_timeout
                    │
                    │  所有 Action Goal 携带 session_id
                    │  定期 RenewLease() 续租
                    │
                    ├── ReleaseControl() ──→ 租约释放
                    ├── 租约超时 ──────────→ 自动停止运动 + 释放
                    └── 前端崩溃 ──────────→ 租约超时后释放
```

### 新增 Service 定义

**AcquireControl.srv**
```
# Request
string client_name          # 前端标识，如 "pendant", "python_script_1"
float64 lease_duration      # 请求租约时长（秒），0 = 使用默认值
---
# Response
bool success
string session_id           # 唯一 session ID（UUID 或递增 token）
float64 lease_timeout       # 实际租约超时时间
string message
```

**ReleaseControl.srv**
```
# Request
string session_id
---
# Response
bool success
string message
```

**RenewLease.srv**
```
# Request
string session_id
float64 lease_extension     # 续期时长（秒），0 = 重置为默认值
---
# Response
bool success
float64 new_timeout
string message
```

**RequestTeachingMode.srv**
```
# Request
string session_id           # 必须持有有效租约
---
# Response
bool success
string message              # 失败时说明原因：无租约/其他前端在 Teaching
```

### 控制器侧校验规则

| 操作 | 校验规则 |
|------|---------|
| Action Goal (MoveJ/MoveL/GoHome) | 必须携带有效 session_id，且 session 与活跃租约匹配 |
| Jog/流控 Topic | 必须已进入 Teaching Mode（通过 RequestTeachingMode 获取） |
| RobotCmd (STOP/ESTOP) | 无需租约，任何前端可调用（安全优先） |
| GetRobotState / SolveIK | 无需租约（只读查询） |
| SetSpeed / SetTCP / SetSpeedRatio | 需要有效租约 |
| ControlGripper | 需要有效租约 |

### Teaching Mode 生命周期

Teaching Mode 与租约绑定，生命周期由租约控制：

- **进入**：持有租约的前端调用 `RequestTeachingMode(session_id)`
- **排他性**：同一时刻只有一个前端可处于 Teaching Mode
- **自动退出**：租约释放/超时时自动退出
- **主动退出**：调用 `ReleaseControl` 释放租约（隐含退出 Teaching Mode）
- **Jog 校验**：控制器收到 Jog/JointTarget Topic 时，检查是否有前端处于 Teaching Mode。处于 MOVING 状态（Action 执行中）时自动拒绝 Jog/流控（现有状态机机制）

注意：由于 ROS2 Topic 消息不携带发送方身份，Jog/流控的 Teaching Mode 通过「允许/禁止」模式管理——进入 Teaching Mode 后，任何发布到 `~/jog_command` 和 `~/joint_target` 的消息都会被接受。实际使用中，只有当前持有 Teaching Mode 的前端才会发送这些 Topic，其他前端应遵守约定不发送。若出现争抢，操作员可立即观察到机器人抖动并停止，200ms 看门狗提供兜底保护。

### 租约超时处理

- 控制器以 10Hz 检查租约是否过期
- 过期时：取消当前 Action → 减速停车 → 状态转 IDLE → 释放租约
- Teaching Mode 随租约过期自动退出
- 租约默认时长：**10 秒**（前端通过 RenewLease 持续续租）
- 续租间隔：默认 **3 秒**，允许 3 次续租失败后才会超时

### 前端续租策略

- HMI：GUI 打开时持续续租（每 3 秒），关闭时主动释放
- Python 脚本：脚本运行期间持续续租（每 3 秒），脚本退出时自动释放（进程崩溃后 10 秒超时）
- robot_tasks：编排层持有自己的租约，任务执行期间持续续租

## 三、Action 接口设计

### 运动类 Action（robot_controller 提供）

**MoveJ.action**
```
# Goal
uint8 JOINT_SPACE = 0
uint8 CARTESIAN = 1
uint8 mode
float64[7] joint_angles          # mode=0 时使用
geometry_msgs/Point position     # mode=1 时使用
geometry_msgs/Vector3 orientation
float64 speed_ratio
float64 finger_width
string session_id                # 租约校验
---
# Result
bool success
string message
float64[7] final_joint_angles
float64[6] final_tcp_pose
---
# Feedback
float64 progress                 # 0.0 ~ 1.0
float64[7] current_joint_angles
float64 estimated_time_remaining
```

**MoveL.action**
```
# Goal
geometry_msgs/Point position
geometry_msgs/Vector3 orientation
string frame                     # "base" 或 "tcp"，与原 MoveLinear.srv 的 frame 含义一致
float64 speed_ratio
float64 finger_width
string session_id                # 租约校验
---
# Result
bool success
string message
float64[7] final_joint_angles
float64[6] final_tcp_pose
---
# Feedback
float64 progress
float64[7] current_joint_angles
float64 estimated_time_remaining
```

**GoHome.action**
```
# Goal
float64 speed_ratio
string session_id                # 租约校验
---
# Result
bool success
string message
float64[7] final_joint_angles
float64[6] final_tcp_pose
---
# Feedback
float64 progress
float64[7] current_joint_angles
float64 estimated_time_remaining
```

### Action 取消语义

| 触发原因 | 行为 |
|---------|------|
| 前端主动 Cancel | 平滑减速停车 → IDLE，租约保留（前端可继续发新指令） |
| 租约超时 | 立即减速停车 → IDLE，租约释放 |
| E-STOP | 立即停车 → FAULT（现有机制，不走 Action 取消流程） |
| Stop (RobotCmd) | 平滑减速停车 → IDLE，租约保留 |

### 任务类 Action（robot_tasks 提供）

**GraspTask.action**
```
# Goal
# --- 状态常量（与 GraspState 枚举对应） ---
uint8 IDLE = 0
uint8 DETECTING = 1
uint8 APPROACHING = 2
uint8 RE_DETECTING = 3
uint8 DESCENDING = 4
uint8 GRASPING = 5
uint8 LIFTING = 6
uint8 DONE = 7
uint8 ERROR = 8

float64 approach_height
float64[3] grasp_rpy
string session_id
---
# Result
bool success
string message
uint8 final_state              # 最终状态机状态（使用上方常量）
float64[7] final_joint_angles
float64[6] final_tcp_pose
---
# Feedback
uint8 current_state            # 当前阶段
string state_description       # "Detecting" / "Approaching" / ...
float64 progress               # 0.0 ~ 1.0
```

### Action Server 位置

| Action | 提供者 | 说明 |
|--------|--------|------|
| MoveJ | robot_controller_node | 底层运动原语 |
| MoveL | robot_controller_node | 底层运动原语 |
| GoHome | robot_controller_node | 回安全位 |
| GraspTask | robot_tasks_node（新增） | 高级抓取编排 |

robot_tasks_node 是一个新增的轻量 ROS2 节点，内部持有 robot_client 调用 robot_controller 的 Action，同时链接 robot_vision_core 获取视觉结果。它只是一个 Action Server 外壳，核心逻辑仍由 GraspTaskManager 驱动。

### robot_tasks_node 设计

**节点生命周期：**
- 由 robot_bringup 的 launch 文件启动（或独立 `ros2 run robot_tasks robot_tasks_node`）
- 工厂模式创建（`TaskNode::create()`），与其他节点风格一致

**线程模型：**
- 使用 `MultiThreadedExecutor`
- GraspTask Action 回调在 Executor 线程中执行
- GraspTaskManager 的阻塞式 `run()` 方法在 Action 回调线程中调用，通过定期检查 `is_cancel_requested()` 支持取消

**GraspTaskManager 异步适配：**
```
GraspTask Action Goal 到达
    │
    ▼
Action Server 的 execute_callback
    │
    ├── acquire_control("robot_tasks")     # 获取控制器租约
    ├── GraspTaskManager::run() 开始执行
    │   ├── 每完成一个阶段 → publish Feedback
    │   ├── 每次循环检查 → is_cancel_requested()?
    │   │   └── 是 → 取消运动, return ABORTED
    │   └── 完成 → return SUCCEEDED
    └── release_control()                  # 释放租约
```

**包结构变更：**
- `robot_tasks_lib`（共享库）继续保留，包含 GraspTaskManager 核心逻辑
- 新增 `robot_tasks_node`（可执行目标），包含 Action Server 外壳
- robot_demos 可选择直接链接 `robot_tasks_lib`（进程内调用）或通过 Action Client 远程调用

## 四、Service 接口梳理

### 保留的 Service（瞬时操作）

| Service | 变化 | 需要租约 |
|---------|------|---------|
| GetRobotState | 不变 | 否 |
| SolveIK | 不变 | 否 |
| SetSpeed | 不变 | 是 |
| SetSpeedRatio | 不变 | 是 |
| SetTCP | 不变 | 是 |
| ControlGripper | 不变 | 是 |
| ControlGripper | 不变 | 是 |
| RobotCmd | 不变 | 否（安全优先） |

### 新增的 Service（租约管理）

| Service | 说明 |
|---------|------|
| AcquireControl | 申请租约，返回 session_id |
| ReleaseControl | 释放租约 |
| RenewLease | 续租 |
| RequestTeachingMode | 申请进入 Jog/流控模式 |

### 删除的 Service（改为 Action）

| Service | 替代为 | 原因 |
|---------|--------|------|
| MoveJoint | MoveJ Action | 耗时运动，需进度/取消 |
| MovePose | MoveJ Action (mode=CARTESIAN) | 耗时运动，需进度/取消 |
| MoveLinear | MoveL Action | 耗时运动，需进度/取消 |
| GoHome | GoHome Action | 耗时运动，需进度/取消 |

### 接口数量汇总

```
重构前:  11 个 Service + 2 个 Action(已定义未实现)
重构后:  11 个 Service (7 保留 + 4 新增租约) + 4 个 Action (3 运动 + 1 抓取)
```

## 五、Client Library（robot_client）

### 定位

合并进 robot_controller，作为第四个 CMake Target，替代原 robot_api_cpp。

```
robot_controller/
├── robot_kinematics    ← 零 ROS 依赖（不变）
├── robot_motion        ← 依赖 kinematics（不变）
├── robot_nodes         ← Action/Service Server（重构）
└── robot_client        ← Action/Service Client 封装（新增）
```

### CMake Target 依赖与导出

robot_client 必须作为独立可导出的 CMake target，使 robot_hmi、robot_tasks 等可以只链接 client 而不拉入完整的 robot_nodes（避免链接控制器节点实现）：

```
robot_kinematics (共享库) ← 零 ROS 依赖
       ▲
robot_motion (共享库)     ← IRobotController 定义在此
       ▲
robot_nodes (共享库)      ← Action/Service Server（不依赖 robot_client）
       (无依赖关系)
robot_client (共享库)     ← 仅依赖 robot_motion + robot_msgs + rclcpp
                           不依赖 robot_nodes，避免循环
```

ament_cmake 导出配置：
- `robot_controller::robot_client` 作为独立可链接 target
- robot_hmi 的 package.xml 依赖 `robot_controller`，CMake 中仅 `target_link_libraries(... robot_controller::robot_client)`

### IRobotController 接口演进

现有全部纯虚方法保留方法名和语义，部分签名微调（移除 MotionSource 参数、调整默认值），新增 4 个租约方法和 1 个进度回调：

```cpp
class IRobotController {
public:
  virtual ~IRobotController() = default;

  // ===== 运动原语（内部改为 Action 调用） =====
  virtual void set_arm(const std::vector<double>& angles, bool block = true) = 0;
  virtual void moveJ(const std::vector<double>& target_angles, bool block = true) = 0;
  virtual void moveJ(const std::array<double, 3>& xyz,
                     const std::optional<std::array<double, 3>>& rpy,
                     double finger = 0, bool block = true) = 0;  // 移除 MotionSource 参数
  virtual void moveL(const std::array<double, 3>& xyz,
                     const std::optional<std::array<double, 3>>& rpy,
                     double finger = 0, bool block = true) = 0;  // 移除 MotionSource 参数
  virtual void move_to_pose(const std::array<double, 3>& xyz,
                            const std::optional<std::array<double, 3>>& rpy,
                            double finger = 0, bool block = true) = 0;
  virtual void move_linear(const std::array<double, 3>& delta,
                           const std::string& frame = "base",
                           double finger = 0, bool block = true) = 0;
  virtual void rotate_joint(int index, double delta_angle, bool block = true) = 0;
  virtual void go_home(bool block = true) = 0;

  // ===== 夹爪（仍为 Service 调用） =====
  virtual void set_gripper(double width, bool block = true) = 0;
  virtual void open_gripper(bool block = true) = 0;
  virtual void close_gripper(bool block = true) = 0;

  // ===== 配置（仍为 Service 调用） =====
  virtual void set_speed(MotionMode mode, double percent) = 0;
  virtual void set_tcp(const std::string& name) = 0;

  // ===== 状态查询（仍为 Service 调用） =====
  virtual std::vector<double> get_joint_angles() const = 0;
  virtual std::array<double, 6> get_end_effector_pose() const = 0;
  virtual double get_finger_width() const = 0;
  virtual std::string get_current_tcp() const = 0;
  virtual std::optional<std::array<double, 6>> lookup_transform(
      const std::string& target, const std::string& source) const = 0;
  virtual double get_speed(MotionMode mode) const = 0;

  // ===== 新增：租约管理 =====
  virtual bool acquire_control(const std::string& client_name, double lease_duration = 0) = 0;
  virtual void release_control() = 0;
  virtual bool renew_lease() = 0;
  virtual std::string session_id() const = 0;

  // ===== 新增：Action 进度回调 =====
  using ProgressCallback = std::function<void(double progress)>;
  virtual void set_progress_callback(ProgressCallback cb) = 0;
};
```

### MotionSource 参数迁移

现有 `moveJ()` 和 `moveL()` 的 `MotionSource` 参数在 Action 化后不再需要——所有权由租约模型统一管理。迁移映射：

| 旧调用方式 | 新调用方式 |
|-----------|-----------|
| `moveJ(..., MotionSource::kService)` | `acquire_control()` → `moveJ(...)`（无 MotionSource 参数） |
| `moveJ(..., MotionSource::kApi)` | `acquire_control()` → `moveJ(...)`（无 MotionSource 参数） |

### ActionRobotController

替代 ServiceRobotController，通过 Action Client 实现运动方法，通过 Service Client 实现配置/查询方法。

```cpp
class ActionRobotController : public robot_control::IRobotController {
public:
  static std::shared_ptr<ActionRobotController> create(
      rclcpp::Node::SharedPtr node,
      const std::string& target_node = "robot_controller_node");

private:
  // Action Clients（运动指令）
  rclcpp_action::Client<robot_msgs::action::MoveJ>::SharedPtr movej_client_;
  rclcpp_action::Client<robot_msgs::action::MoveL>::SharedPtr movel_client_;
  rclcpp_action::Client<robot_msgs::action::GoHome>::SharedPtr gohome_client_;

  // Service Clients（配置/查询）
  rclcpp::Client<robot_msgs::srv::GetRobotState>::SharedPtr state_client_;
  rclcpp::Client<robot_msgs::srv::ControlGripper>::SharedPtr gripper_client_;
  rclcpp::Client<robot_msgs::srv::SetSpeed>::SharedPtr speed_client_;
  rclcpp::Client<robot_msgs::srv::SetSpeedRatio>::SharedPtr speed_ratio_client_;
  rclcpp::Client<robot_msgs::srv::SetTCP>::SharedPtr tcp_client_;
  rclcpp::Client<robot_msgs::srv::AcquireControl>::SharedPtr acquire_client_;
  rclcpp::Client<robot_msgs::srv::ReleaseControl>::SharedPtr release_client_;
  rclcpp::Client<robot_msgs::srv::RenewLease>::SharedPtr renew_client_;

  // 租约状态
  std::string session_id_;
  std::chrono::steady_clock::time_point lease_expiry_;
  std::mutex lease_mutex_;
  std::thread lease_renewal_thread_;
};
```

### RobotClient

对外入口不变，内部持有 ActionRobotController 实例：

```cpp
class RobotClient : public rclcpp::Node {
public:
  static std::shared_ptr<RobotClient> create(
      const std::string& target_node = "robot_controller_node");
  std::shared_ptr<ActionRobotController> get_controller();
  bool wait_for_services(double timeout = 10.0);
};
```

### RobotStatus.msg 演进

现有 `motion_owner` 字段（uint8, OWNER_NONE/PENDANT/SCRIPT）替换为租约信息：

```
# RobotStatus.msg 变更
- uint8 motion_owner          # 删除
+ string active_session_id    # 新增：当前活跃租约 ID（空字符串 = 无租约）
+ string active_client_name   # 新增：当前租约持有者名称
+ bool teaching_mode_active   # 新增：是否有前端处于 Teaching Mode
```

### PendantNode 迁移方案

当前 PendantNode 直接创建 8 个 Service Client + 4 个 Topic pub/sub。迁移到 robot_client 后：

**线程模型（不变）：**
- Qt 主线程 + ROS2 Executor 线程 + Service Task 线程 + Joint Stream 线程
- RobotClient 作为 PendantNode 内部持有的 ROS2 Node（替代 PendantNode 自身的 Node）
- 或者 PendantNode 直接继承 RobotClient（共享同一个 Node）

**接口变化：**
- PendantNode 不再直接创建 Service Client，改用 `RobotClient::get_controller()`
- Action 调用通过 `ActionRobotController` 的 `moveJ()/moveL()` 方法（签名不变）
- 进度反馈通过 `set_progress_callback()` 回调到 Qt 主线程
- Jog/流控 Topic 仍由 PendantNode 直接管理（robot_client 不封装 Topic）
- 租约管理：PendantNode 启动时 `acquire_control("pendant")`，3 秒续租，关闭时 `release_control()`

**robot_client 不封装 Topic 的原因：**
- Topic（Jog、joint_target、camera image、status）是前端特有的需求
- 不同前端对 Topic 的使用方式不同（HMI 需要图像显示，Python 脚本可能不需要）
- robot_client 只封装 Action/Service（请求-响应模式），Topic 由各前端自行管理

### 对上层的影响

| 调用方 | 变化 |
|--------|------|
| robot_tasks | IRobotController 接口不变，实现从 ServiceRobotController 换成 ActionRobotController |
| robot_hmi PendantNode | 不再直接创建 Service Client，改用 RobotClient |
| robot_api_python | pybind11 绑定从 robot_api_cpp 换成 robot_controller::robot_client |
| robot_demos | 同上 |

## 六、迁移路径

### 编译顺序

```
Phase 1: robot_msgs（接口层）—— 全量重编译
  ├── 新增 4 个 Service: AcquireControl, ReleaseControl, RenewLease, RequestTeachingMode
  ├── 修改 2 个 Action: MoveJ, MoveL（在现有文件中加 session_id）
  ├── 新增 2 个 Action: GoHome, GraspTask
  └── 删除 4 个 Service: MoveJoint, MovePose, MoveLinear, GoHome
  ⚠️  此步骤后必须全量重编译所有依赖包

Phase 2: robot_controller（后端层）—— 依赖 Phase 1
  ├── robot_nodes: 新增 Action Server（MoveJ/MoveL/GoHome）+ 租约管理逻辑
  ├── robot_nodes: 删除 4 个运动 Service handler
  ├── robot_nodes: 改造 MotionOwner → 租约模型
  ├── robot_nodes: 更新 RobotStatus.msg 发布（motion_owner → session 信息）
  └── robot_client: 新增 CMake Target（ActionRobotController + RobotClient）

Phase 3: robot_tasks（编排层）—— 依赖 Phase 2
  ├── 改用 robot_controller::robot_client（替代 robot_api_cpp）
  └── 新增 robot_tasks_node（GraspTask Action Server）

Phase 4: 前端迁移（可并行）—— 依赖 Phase 2
  ├── robot_hmi: PendantNode 重构（使用 RobotClient + 租约管理）
  ├── robot_api_python: pybind11 重绑（指向 robot_controller::robot_client）
  └── robot_demos: 更新调用方式（使用 RobotClient）

Phase 5: 清理
  └── 删除 robot_api_cpp 包 + 更新 robot_bringup launch 文件
```

### Phase 依赖关系

```
Phase 1 (robot_msgs)
    │
    ▼
Phase 2 (robot_controller)
    │
    ├──→ Phase 3 (robot_tasks)  ──→ 可独立测试
    │
    └──→ Phase 4 (前端迁移, 三个包可并行)
              ├── robot_hmi
              ├── robot_api_python
              └── robot_demos
              │
              ▼
         Phase 5 (清理)
```

注意：Phase 1 完成后所有现有代码将编译失败（接口 ABI 变更），因此 Phase 2 必须紧随其后。Phase 3 和 Phase 4 都依赖 Phase 2 完成，但它们之间互相独立。

### 测试策略

| Phase | 测试 |
|-------|------|
| Phase 2 | 单元测试：租约申请/释放/超时；Action 执行/取消/进度反馈 |
| Phase 2 | 集成测试：Action Client → Action Server 端到端 |
| Phase 3 | 集成测试：GraspTask Action 端到端 |
| Phase 4 | 手动测试：HMI GUI 操作；Python demo 运行 |
