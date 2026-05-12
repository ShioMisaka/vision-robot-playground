# robot_msgs

## 职责
定义整个机器人控制系统的 ROS2 自定义接口，包括 Service、Action 和 Message 类型。
所有其他包（robot_controller、robot_hmi、robot_vision、robot_api_python）均依赖此包。

## 节点清单
无可执行节点。纯接口定义包，通过 `rosidl` 生成 C++/Python 消息代码。

## 消息定义

### RobotStatus.msg
机器人完整状态遥测，由 `robot_controller_node` 以 10Hz 发布。

| 字段 | 类型 | 说明 |
|------|------|------|
| state | uint8 | 状态机：0=IDLE, 1=MOVING, 2=TEACHING, 3=STOPPING, 4=FAULT |
| speed_ratio | float64 | 当前全局速度比（0.0-1.0） |
| error_code | int32 | 错误码（0=正常） |
| error_message | string | 错误描述 |
| joint_angles | float64[7] | 7 轴关节角度（rad） |
| tcp_pose | float64[6] | TCP 位姿 [x, y, z, roll, pitch, yaw] |
| finger_width | float64 | 夹爪开口宽度（m） |
| tcp_name | string | 当前 TCP 名称 |
| is_connected | bool | 关节反馈是否在线 |
| active_session_id | string | 当前持有 Lease 的 session_id（空字符串表示无持有者） |
| active_client_name | string | 当前持有 Lease 的客户端名称 |
| teaching_mode_active | bool | 示教模式是否激活 |

### JogCommand.msg
Jog 点动速度命令，由 `PendantNode` 发布，`RobotControllerNode` 订阅。

| 字段 | 类型 | 说明 |
|------|------|------|
| frame | uint8 | 坐标系：0=TCP_FRAME, 1=BASE_FRAME |
| velocity | float64[6] | 6 轴速度 [vx, vy, vz, wx, wy, wz] |
| stamp | builtin_interfaces/Time | 时间戳 |

## Service 定义

### 运动控制 Service

| Service | 请求字段 | 响应字段 | 说明 |
|---------|---------|---------|------|
| SolveIK | xyz: float64[], rpy: float64[] | success, joint_angles: float64[], message | IK 求解（xyz, rpy → 关节角） |
| ControlGripper | command: uint8, width: float64 | success, message | 夹爪控制（0=open, 1=close, 2=set_width） |
| SetSpeed | mode: uint8, percent: float64 | success, message | 设置速度（0=moveJ, 1=moveL） |
| GetRobotState | _(空请求)_ | success, joint_angles, tcp_pose, finger_width, tcp_name, message | 查询完整状态 |

### 示教器专用 Service

| Service | 请求字段 | 响应字段 | 说明 |
|---------|---------|---------|------|
| SetTCP | name: string | success, message | 设置当前 TCP 工具坐标系名称 |
| SetSpeedRatio | ratio: float64 | success, message | 设置全局速度比（0.0-1.0），与模式速度复合 |
| RobotCmd | command: uint8 | success, message | 机器人命令：0=STOP, 1=EMERGENCY_STOP, 2=CLEAR_FAULT |

### Lease 管理 Service

| Service | 请求字段 | 响应字段 | 说明 |
|---------|---------|---------|------|
| AcquireControl | client_name: string, lease_duration: float64 | success, session_id: string, lease_timeout: float64, message | 获取控制权 Lease（返回 session_id） |
| ReleaseControl | session_id: string | success, message | 释放控制权 |
| RenewLease | session_id: string, lease_extension: float64 | success, new_timeout: float64, message | 续约 Lease |
| RequestTeachingMode | session_id: string | success, message | 请求示教模式（需持有 Lease） |

## Action 定义

### MoveJ.action
关节空间/笛卡尔空间运动，带进度反馈。

**Goal:**
| 字段 | 类型 | 说明 |
|------|------|------|
| mode | uint8 | 0=JOINT_SPACE, 1=CARTESIAN |
| joint_angles | float64[7] | 目标关节角（mode=0） |
| position | geometry_msgs/Point | 目标位置（mode=1） |
| orientation | geometry_msgs/Vector3 | 目标朝向（mode=1） |
| speed_ratio | float64 | 速度比 |
| finger_width | float64 | 目标夹爪宽度 |
| session_id | string | Lease 授权 session_id |

**Result:** success, message, final_joint_angles[7], final_tcp_pose[6]

**Feedback:** progress (0.0-1.0), current_joint_angles[7], estimated_time_remaining

### MoveL.action
笛卡尔空间线性运动，带进度反馈。

**Goal:** position, orientation, frame, speed_ratio, finger_width, session_id

**Result / Feedback:** 同 MoveJ

### GoHome.action
回安全位运动，带进度反馈。

**Goal:** speed_ratio, session_id

**Result:** success, message, final_joint_angles[7], final_tcp_pose[6]

**Feedback:** progress (0.0-1.0), current_joint_angles[7], estimated_time_remaining

### GraspTask.action
视觉引导抓取任务，带状态反馈。

**Goal:**
| 字段 | 类型 | 说明 |
|------|------|------|
| approach_height | float64 | 接近高度（m） |
| grasp_rpy | float64[3] | 抓取姿态 [roll, pitch, yaw] |
| session_id | string | Lease 授权 session_id |

**Result:** success, message, final_state, final_joint_angles[7], final_tcp_pose[6]

**Feedback:** current_state, state_description, progress

## 关键参数
无 YAML 配置文件。所有参数通过 Service/Action 请求传递。

## 启动方式
此包无可执行节点，仅提供消息定义。编译后自动生成 C++/Python 绑定代码。

## 包内依赖
- **内部依赖**: 无
- **外部依赖**: std_msgs, builtin_interfaces, geometry_msgs, action_msgs, rosidl_default_generators

## 修改指南
- **新增 Service** → 在 `srv/` 目录创建 `.srv` 文件，并在 `CMakeLists.txt` 的 `rosidl_generate_interfaces()` 中添加
- **新增 Action** → 在 `action/` 目录创建 `.action` 文件，同样在 `CMakeLists.txt` 注册
- **新增 Message** → 在 `msg/` 目录创建 `.msg` 文件，在 `CMakeLists.txt` 注册
- **修改已有接口字段** → 直接编辑对应 `.srv` / `.action` / `.msg` 文件，**注意：修改字段会破坏 ABI 兼容性，所有依赖包需重新编译**

## 注意事项
- 此包是整个系统的接口基础层，修改后必须重新编译所有依赖包
- `rosidl_generate_interfaces()` 中必须列出所有消息文件，遗漏会导致编译失败
- `rosidl_interface_packages` 成员组确保消息生成到 `robot_msgs` 命名空间下
- 所有 Service 响应包含 `success: bool` + `message: string`，便于错误诊断
