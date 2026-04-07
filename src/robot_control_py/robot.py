"""ROS2 机器人控制节点：发送关节指令、读取状态、IK 位姿控制

架构说明：
    本节点使用 MultiThreadedExecutor 在后台线程中处理 ROS2 回调，
    业务逻辑（阻塞式调用）在主线程中执行，二者通过 threading.Event
    和线程安全的数据结构进行同步，完全避免了 spin_once 滥用。

使用方式：
    rclpy.init()
    robot = RobotController()
    executor = MultiThreadedExecutor()
    executor.add_node(robot)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    robot.wait_for_ready()  # 阻塞等待，不使用 spin_once
    robot.open_gripper()
    time.sleep(1.0)         # 普通 sleep，回调由 executor 线程处理

扩展预留：
    - Action Server: 后续可通过 control_msgs/action/FollowJointTrajectory
      实现轨迹级别的控制，替代当前的即时位置指令
    - TF2: 已初始化 TransformBroadcaster 和 Buffer，
      用于发布末端执行器和相机的坐标变换
"""

from __future__ import annotations

import math
import threading
import time
from typing import Optional

import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, Buffer, TransformListener

from scipy.spatial.transform import Rotation

from src.robot_control_py.config import (
    ALL_JOINTS,
    ACTIVE_JOINTS,
    TOPIC_JOINT_COMMAND,
    TOPIC_JOINT_STATES,
    SAFE_HOME,
    FRAME_BASE,
    FRAME_END_EFFECTOR,
    TCP_FRAMES,
    DEFAULT_TCP,
)
from src.robot_control_py.ik_solver import IKSolver


class RobotController(Node):
    """Franka Panda ROS2 控制节点

    通过发布 JointState 指令控制 Isaac Sim 中的 Franka 机械臂，
    订阅 /joint_states 获取实时关节反馈。

    注意：本节点必须在 MultiThreadedExecutor 中运行，
    否则回调将无法在阻塞式业务逻辑期间被处理。

    扩展预留：
        后续可基于 control_msgs/action/FollowJointTrajectory 实现
        Action Server，提供轨迹级别的插值运动控制，
        替代当前单次位置指令的即时跳变模式。
    """

    def __init__(self) -> None:
        super().__init__("franka_controller")

        # ---- 回调组划分 ----
        # MutuallyExclusiveCallbackGroup: 保证关节状态回调不会重入，
        # 避免在处理一条消息时被同组回调打断导致数据竞争
        self._state_cbg = MutuallyExclusiveCallbackGroup()
        # ReentrantCallbackGroup: 发布和 TF 广播可并行执行，
        # 不需要互斥保护
        self._pub_cbg = ReentrantCallbackGroup()

        # 发布关节指令（使用可重入回调组）
        self.publisher_ = self.create_publisher(
            JointState, TOPIC_JOINT_COMMAND, 10,
            callback_group=self._pub_cbg,
        )

        # 订阅机器人实时关节状态（使用互斥回调组）
        self._latest_joint_msg: JointState | None = None
        self._joint_state_lock = threading.Lock()
        self._joint_sub = self.create_subscription(
            JointState, TOPIC_JOINT_STATES, self._on_joint_states, 10,
            callback_group=self._state_cbg,
        )

        # 就绪事件：第一条 /joint_states 到达后置位，
        # 替代旧的 spin_once 轮询等待
        self._ready_event = threading.Event()

        self.ik: IKSolver = IKSolver()

        # 记住当前关节状态，保证 open/close_gripper 只改夹爪
        self._current_arm: list[float] = [0.0] * 7
        self._current_finger: float = 0.04
        self._grasping: bool = False  # 夹取物体标志，跳过夹爪到位检查

        # ---- TF2 初始化 ----
        # TransformBroadcaster: 在关节状态回调中持续发布末端执行器 TF
        self._tf_broadcaster = TransformBroadcaster(self)
        # Buffer + Listener: 用于查询外部 TF（如相机坐标系变换）
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ---- TCP 工具坐标系 ----
        self._current_tcp_name: str = DEFAULT_TCP
        tcp_cfg = TCP_FRAMES[self._current_tcp_name]
        self._tcp_offset_xyz: list[float] = list(tcp_cfg["offset_xyz"])
        self._tcp_offset_rpy: list[float] = list(tcp_cfg["offset_rpy"])

        self.get_logger().info("RobotController 已启动")

    # ---- 同步等待 ----

    def wait_for_ready(self, timeout: float = 5.0) -> None:
        """阻塞等待直到收到第一条 /joint_states 消息

        使用 threading.Event 替代 spin_once 轮询。
        需要节点在 MultiThreadedExecutor 中运行，回调才能在等待期间被处理。

        Args:
            timeout: 超时时间（秒）

        Raises:
            TimeoutError: 超时未收到关节状态
        """
        if not self._ready_event.wait(timeout=timeout):
            raise TimeoutError(f"等待 /joint_states 超时 ({timeout}s)")

        with self._joint_state_lock:
            if self._latest_joint_msg is not None:
                name_to_pos = dict(
                    zip(self._latest_joint_msg.name, self._latest_joint_msg.position)
                )
                self._current_arm = [
                    float(name_to_pos.get(j, 0.0)) for j in ACTIVE_JOINTS
                ]
                self._current_finger = float(
                    name_to_pos.get("panda_finger_joint1", 0.0)
                )

    # ---- 内部回调 ----

    def _on_joint_states(self, msg: JointState) -> None:
        """接收 /joint_states 并同步当前手臂关节角（夹爪只由指令控制）"""
        with self._joint_state_lock:
            self._latest_joint_msg = msg
            name_to_pos = dict(zip(msg.name, msg.position))
            self._current_arm = [
                float(name_to_pos.get(j, 0.0)) for j in ACTIVE_JOINTS
            ]

        # 发布末端执行器 TF（不阻塞关节状态更新）
        self._publish_ee_tf(msg)

        # 首次收到消息时触发就绪事件，唤醒 wait_for_ready
        if not self._ready_event.is_set():
            self._ready_event.set()

    def _publish_ee_tf(self, msg: JointState) -> None:
        """根据当前关节角计算并发布末端执行器和 TCP 的 TF 变换"""
        name_to_pos = dict(zip(msg.name, msg.position))
        arm_angles = [float(name_to_pos.get(j, 0.0)) for j in ACTIVE_JOINTS]

        try:
            result = self.ik.forward(arm_angles)
            stamp = self.get_clock().now().to_msg()

            # 发布 panda_link0 -> panda_hand
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = FRAME_BASE
            t.child_frame_id = FRAME_END_EFFECTOR
            t.transform.translation.x = float(result["pos"][0])
            t.transform.translation.y = float(result["pos"][1])
            t.transform.translation.z = float(result["pos"][2])

            rot = Rotation.from_euler("xyz", result["rpy"]).as_quat()
            t.transform.rotation.x = float(rot[0])
            t.transform.rotation.y = float(rot[1])
            t.transform.rotation.z = float(rot[2])
            t.transform.rotation.w = float(rot[3])

            self._tf_broadcaster.sendTransform(t)

            # 发布 panda_hand -> <tcp_name>
            if self._current_tcp_name != "hand":
                t_tcp = TransformStamped()
                t_tcp.header.stamp = stamp
                t_tcp.header.frame_id = FRAME_END_EFFECTOR
                t_tcp.child_frame_id = self._current_tcp_name
                t_tcp.transform.translation.x = self._tcp_offset_xyz[0]
                t_tcp.transform.translation.y = self._tcp_offset_xyz[1]
                t_tcp.transform.translation.z = self._tcp_offset_xyz[2]
                rot_tcp = Rotation.from_euler(
                    "xyz", self._tcp_offset_rpy
                ).as_quat()
                t_tcp.transform.rotation.x = float(rot_tcp[0])
                t_tcp.transform.rotation.y = float(rot_tcp[1])
                t_tcp.transform.rotation.z = float(rot_tcp[2])
                t_tcp.transform.rotation.w = float(rot_tcp[3])
                self._tf_broadcaster.sendTransform(t_tcp)
        except Exception:
            pass  # FK 失败不影响主流程

    # ---- TF2 查询 ----

    def lookup_transform(
        self,
        target_frame: str,
        source_frame: str,
        timeout: float = 1.0,
    ) -> Optional[TransformStamped]:
        """查询 TF 变换

        Args:
            target_frame: 目标坐标系
            source_frame: 源坐标系
            timeout: 超时时间（秒）

        Returns:
            TransformStamped 或 None（查询失败时）
        """
        try:
            return self._tf_buffer.lookup_transform(
                target_frame, source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=timeout),
            )
        except Exception:
            self.get_logger().warning(
                f"TF 查询失败: {source_frame} -> {target_frame}"
            )
            return None

    # ---- TCP 工具坐标系 ----

    def set_tcp(self, name: str) -> None:
        """切换当前工具坐标系

        Args:
            name: TCP 名称，对应 TCP_FRAMES 中的 key

        Raises:
            ValueError: 未知 TCP 名称
        """
        if name not in TCP_FRAMES:
            raise ValueError(
                f"未知 TCP: {name}，可选: {list(TCP_FRAMES.keys())}"
            )
        self._current_tcp_name = name
        self._tcp_offset_xyz = list(TCP_FRAMES[name]["offset_xyz"])
        self._tcp_offset_rpy = list(TCP_FRAMES[name]["offset_rpy"])
        self.get_logger().info(f"已切换 TCP: {name}")

    def _tcp_transform_matrix(self) -> np.ndarray:
        """计算 TCP 相对 panda_hand 的 4x4 齐次变换矩阵

        Returns:
            4x4 numpy 齐次变换矩阵
        """
        T = np.eye(4)
        T[:3, :3] = Rotation.from_euler("xyz", self._tcp_offset_rpy).as_matrix()
        T[:3, 3] = self._tcp_offset_xyz
        return T

    # ---- 状态读取 ----

    def get_joint_angles(self) -> list[float]:
        """获取机器人当前 7 个手臂关节角度（来自仿真反馈）

        Returns:
            panda_joint1~7 的当前角度（弧度）
        """
        with self._joint_state_lock:
            if self._latest_joint_msg is None:
                self.get_logger().warn("尚未收到 /joint_states 消息，返回默认值 0")
                return [0.0] * 7

            name_to_pos = dict(
                zip(self._latest_joint_msg.name, self._latest_joint_msg.position)
            )
            return [float(name_to_pos.get(j, 0.0)) for j in ACTIVE_JOINTS]

    def get_finger_width(self) -> float:
        """获取夹爪当前开合程度

        Returns:
            0.0=闭合, 0.04=全开
        """
        with self._joint_state_lock:
            if self._latest_joint_msg is None:
                return 0.0
            name_to_pos = dict(
                zip(self._latest_joint_msg.name, self._latest_joint_msg.position)
            )
            return float(name_to_pos.get("panda_finger_joint1", 0.0))

    def get_end_effector_pose(self) -> dict[str, list[float]]:
        """获取当前 TCP 位姿（通过 FK + 偏移计算）

        Returns:
            {"pos": [x,y,z], "rpy_rad": [r,p,y], "rpy_deg": [r,p,y]}
        """
        current_angles: list[float] = self.get_joint_angles()
        hand_matrix: np.ndarray = self.ik.forward(current_angles)["matrix"]
        tcp_matrix: np.ndarray = hand_matrix @ self._tcp_transform_matrix()
        tcp_pos: list[float] = tcp_matrix[:3, 3].tolist()
        tcp_rpy: list[float] = Rotation.from_matrix(
            tcp_matrix[:3, :3]
        ).as_euler("xyz").tolist()
        return {
            "pos": tcp_pos,
            "rpy_rad": tcp_rpy,
            "rpy_deg": [math.degrees(r) for r in tcp_rpy],
        }

    # ---- 底层：直接发 9 个关节值（私有）----

    def _send(self, arm_angles: list[float], finger: float) -> None:
        """发送完整关节指令（内部使用）"""
        self._current_arm = list(arm_angles)
        self._current_finger = finger
        msg = JointState()
        msg.name = ALL_JOINTS
        msg.position = self._current_arm + [finger, finger]
        self.publisher_.publish(msg)

    def _wait_for_motion(
        self,
        arm_angles: list[float],
        finger: float,
        joint_tol: float = 0.05,
        finger_tol: float = 0.002,
        timeout: float = 10.0,
        poll_interval: float = 0.02,
        check_finger: Optional[bool] = None,
        settle_time: float = 0.2,
    ) -> None:
        """阻塞等待机器人到达目标关节角

        通过持续对比 /joint_states 反馈与指令角度，
        当所有关节偏差均小于阈值时返回。

        Args:
            arm_angles: 7 个目标关节角度
            finger: 目标夹爪开合
            joint_tol: 关节角容差（弧度），默认 0.05 rad ≈ 2.9°
            finger_tol: 夹爪容差（米），默认 0.002 m
            timeout: 超时时间（秒）
            poll_interval: 轮询间隔（秒）
            check_finger: 是否检查夹爪到位，夹取物体时夹爪无法完全闭合，
                应设为 False

        Raises:
            TimeoutError: 超时未到达目标位置
        """
        # 夹取物体后夹爪无法完全闭合，默认跳过 finger 检查
        if check_finger is None:
            check_finger = not self._grasping

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._joint_state_lock:
                if self._latest_joint_msg is not None:
                    name_to_pos = dict(
                        zip(
                            self._latest_joint_msg.name,
                            self._latest_joint_msg.position,
                        )
                    )
                    feedback_arm = [
                        float(name_to_pos.get(j, float("inf")))
                        for j in ACTIVE_JOINTS
                    ]
                    feedback_finger = float(
                        name_to_pos.get("panda_finger_joint1", float("inf"))
                    )

            arm_ok = all(
                abs(t - f) < joint_tol
                for t, f in zip(arm_angles, feedback_arm)
            )
            finger_ok = not check_finger or abs(
                finger - feedback_finger
            ) < finger_tol

            if arm_ok and finger_ok:
                # 到位后再等待一段时间，让机械臂充分稳定
                time.sleep(settle_time)
                return

            time.sleep(poll_interval)

        raise TimeoutError(
            f"运动超时 ({timeout}s)："
            f"目标 arm={[round(a, 3) for a in arm_angles]}, "
            f"实际 arm={[round(a, 3) for a in feedback_arm]}"
        )

    def _interpolate_to(
        self,
        target_angles: list[float],
        finger: float,
        steps: int = 10,
        step_time: float = 0.08,
        block: bool = True,
    ) -> None:
        """在关节空间线性插值到目标角度，避免瞬间跳变引起的震荡

        Args:
            target_angles: 7 个目标关节角度
            finger: 最终夹爪开合（仅最后一步生效）
            steps: 插值步数
            step_time: 每步间隔（秒）
            block: True=等待最后一步到位后再返回
        """
        current = self.get_joint_angles()
        for i in range(1, steps + 1):
            t = i / steps
            interp = [
                c + t * (tgt - c)
                for c, tgt in zip(current, target_angles)
            ]
            self._send(interp, finger if i == steps else self._current_finger)
            time.sleep(step_time)

        if block:
            self._wait_for_motion(target_angles, finger)

    # ---- 中层：传入关节角 / 夹爪控制 ----

    def set_arm(self, angles: list[float], block: bool = True) -> None:
        """直接设置 7 个手臂关节角度

        Args:
            angles: 7 个关节角度（弧度），顺序对应 panda_joint1~7
            block: True=等待到位后再返回
        """
        if len(angles) != 7:
            raise ValueError(f"需要 7 个关节角度，收到 {len(angles)} 个")
        self._send(angles, self._current_finger)
        if block:
            self._wait_for_motion(angles, self._current_finger)

    def set_gripper(self, width: float, block: bool = True) -> None:
        """设置夹爪开合程度

        Args:
            width: 0.0=闭合, 0.04=全开
            block: True=等待到位后再返回
        """
        self._grasping = False
        self._send(self._current_arm, width)
        if block:
            self._wait_for_motion(self._current_arm, width)

    def open_gripper(self, block: bool = True) -> None:
        """张开夹爪"""
        self.set_gripper(0.04, block=block)

    def close_gripper(self, block: bool = True) -> None:
        """闭合夹爪

        夹取物体时夹爪可能无法完全闭合，阻塞模式下会等待夹爪
        稳定（位置不再变化），然后设置 grasping 标志。
        _current_finger 保持 0.0，确保 PD 控制器持续施加闭合力。
        """
        self._send(self._current_arm, 0.0)
        if block:
            self._wait_for_finger_settle()
        self._grasping = True

    def _wait_for_finger_settle(
        self,
        stable_count: int = 5,
        tol: float = 0.001,
        poll_interval: float = 0.05,
        timeout: float = 5.0,
    ) -> None:
        """等待夹爪运动停止

        持续读取夹爪反馈，当连续 stable_count 次读数变化均小于 tol 时
        认为夹爪已稳定。不修改 _current_finger，保持 finger=0.0 指令
        以维持夹紧力。

        Args:
            stable_count: 连续稳定读数次数
            tol: 稳定判定容差（米）
            poll_interval: 轮询间隔（秒）
            timeout: 超时时间（秒）
        """
        deadline = time.monotonic() + timeout
        count = 0
        last = self.get_finger_width()

        while time.monotonic() < deadline:
            time.sleep(poll_interval)
            curr = self.get_finger_width()
            if abs(curr - last) < tol:
                count += 1
                if count >= stable_count:
                    return
            else:
                count = 0
                last = curr

    # ---- 高层：IK 位姿控制 ----

    def move_to_pose(
        self,
        xyz: list[float],
        rpy: Optional[list[float]] = None,
        finger: Optional[float] = None,
        steps: int = 0,
        step_time: float = 0.08,
        block: bool = True,
    ) -> None:
        """IK 求解并移动到指定位姿（目标为当前 TCP 位姿）

        内部自动将 TCP 目标转换为 panda_hand 目标后求解 IK。

        Args:
            xyz: [x, y, z] 目标 TCP 位置（米）
            rpy: [roll, pitch, yaw] 目标 TCP 姿态（弧度），None=不约束朝向
            finger: 夹爪开合，None=保持当前状态
            steps: 插值步数，0=瞬间跳变，>0=关节空间平滑插值
            step_time: 插值每步间隔（秒）
            block: True=等待到位后再返回
        """
        tcp_offset = self._tcp_transform_matrix()

        if rpy is not None:
            # 完整位姿约束：T_hand = T_tcp_target @ inv(T_tcp_in_hand)
            target = np.eye(4)
            target[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
            target[:3, 3] = xyz
            hand_target = target @ np.linalg.inv(tcp_offset)
            hand_xyz = hand_target[:3, 3].tolist()
            hand_rpy = (
                Rotation.from_matrix(hand_target[:3, :3])
                .as_euler("xyz")
                .tolist()
            )
            angles: list[float] = self.ik.solve(hand_xyz, hand_rpy)
        else:
            # 仅位置约束：用当前 hand 姿态近似计算偏移方向
            hand_matrix = self.ik.forward(self.get_joint_angles())["matrix"]
            R_hand = hand_matrix[:3, :3]
            hand_xyz = (
                np.array(xyz) - R_hand @ np.array(self._tcp_offset_xyz)
            ).tolist()
            angles = self.ik.solve(hand_xyz)

        # 验证：用实际 TCP 位姿对比目标
        result = self.ik.forward(angles)
        actual_tcp = result["matrix"] @ tcp_offset
        actual_tcp_pos: list[float] = actual_tcp[:3, 3].tolist()
        pos_error: float = math.dist(xyz, actual_tcp_pos)

        log = (
            f"[TCP:{self._current_tcp_name}] "
            f"目标 pos=({xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f})  "
            f"实际 pos=({actual_tcp_pos[0]:.3f}, {actual_tcp_pos[1]:.3f}, "
            f"{actual_tcp_pos[2]:.3f})  "
            f"误差={pos_error * 1000:.1f}mm"
        )
        if rpy is not None:
            actual_tcp_rpy: list[float] = (
                Rotation.from_matrix(actual_tcp[:3, :3])
                .as_euler("xyz")
                .tolist()
            )
            rpy_err: float = math.dist(rpy, actual_tcp_rpy)
            log += (
                f"\n  目标 rpy=({math.degrees(rpy[0]):.1f}, "
                f"{math.degrees(rpy[1]):.1f}, "
                f"{math.degrees(rpy[2]):.1f})  "
                f"实际 rpy=({math.degrees(actual_tcp_rpy[0]):.1f}, "
                f"{math.degrees(actual_tcp_rpy[1]):.1f}, "
                f"{math.degrees(actual_tcp_rpy[2]):.1f})  "
                f"误差={math.degrees(rpy_err):.2f}deg"
            )
        self.get_logger().info(log)

        if finger is None:
            finger = self._current_finger

        if steps > 0:
            self._interpolate_to(angles, finger, steps, step_time, block=block)
        else:
            self._send(angles, finger)
            if block:
                self._wait_for_motion(angles, finger)

    def move_linear(
        self,
        delta: list[float],
        frame: str = "base",
        finger: Optional[float] = None,
        block: bool = True,
    ) -> None:
        """沿基座或末端坐标系进行相对平移

        Args:
            delta: [dx, dy, dz] 平移量（米），正值方向取决于 frame
            frame: "base"=基座坐标系, "end_effector"=末端执行器坐标系
            finger: 夹爪开合，None=保持当前状态
            block: True=等待到位后再返回
        """
        current = self.get_end_effector_pose()
        pos: list[float] = current["pos"]
        rpy: list[float] = current["rpy_rad"]

        if frame == "end_effector":
            rot = Rotation.from_euler("xyz", rpy)
            offset = rot.apply(delta)
            target = [p + o for p, o in zip(pos, offset)]
        else:
            target = [p + d for p, d in zip(pos, delta)]

        self.move_to_pose(target, rpy=rpy, finger=finger, block=block)

    def rotate_joint(self, index: int, delta_angle: float, block: bool = True) -> None:
        """旋转指定关节

        Args:
            index: 关节索引（0-6），对应 panda_joint1~7
            delta_angle: 旋转增量（弧度），正=正方向
            block: True=等待到位后再返回
        """
        if not 0 <= index <= 6:
            raise ValueError(f"关节索引范围为 0-6，收到 {index}")
        angles = self.get_joint_angles()
        angles[index] += delta_angle
        self.set_arm(angles, block=block)

    def go_home(self, block: bool = True) -> None:
        """回到安全零位"""
        self._send(SAFE_HOME[:7], SAFE_HOME[7])
        if block:
            self._wait_for_motion(SAFE_HOME[:7], SAFE_HOME[7])
