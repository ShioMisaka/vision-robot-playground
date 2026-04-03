"""ROS2 机器人控制节点：发送关节指令、读取状态、IK 位姿控制"""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from src.config import (
    ALL_JOINTS, ACTIVE_JOINTS, TOPIC_JOINT_COMMAND,
    TOPIC_JOINT_STATES, SAFE_HOME,
)
import time as _time

from src.ik_solver import IKSolver


class RobotController(Node):
    """Franka Panda ROS2 控制节点"""

    _current_arm: list[float]
    _current_finger: float

    def __init__(self):
        super().__init__("franka_controller")

        # 发布关节指令
        self.publisher_ = self.create_publisher(JointState, TOPIC_JOINT_COMMAND, 10)

        # 订阅机器人实时关节状态
        self._latest_joint_msg: JointState | None = None
        self._joint_sub = self.create_subscription(
            JointState, TOPIC_JOINT_STATES, self._on_joint_states, 10
        )

        self.ik: IKSolver = IKSolver()

        # 记住当前关节状态，保证 open/close_gripper 只改夹爪
        self._current_arm = [0.0] * 7
        self._current_finger = 0.04

        self.get_logger().info("RobotController 已启动")

    def wait_for_ready(self, timeout: float = 5.0):
        """阻塞等待直到收到第一条 /joint_states 消息"""
        start = self.get_clock().now().nanoseconds
        while self._latest_joint_msg is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            elapsed = (self.get_clock().now().nanoseconds - start) / 1e9
            if elapsed > timeout:
                raise TimeoutError(f"等待 /joint_states 超时 ({timeout}s)")
        # 同步一次状态
        name_to_pos = dict(
            zip(self._latest_joint_msg.name, self._latest_joint_msg.position)
        )
        self._current_arm = [float(name_to_pos.get(j, 0.0)) for j in ACTIVE_JOINTS]
        self._current_finger = float(name_to_pos.get("panda_finger_joint1", 0.0))

    # ---- 内部回调 ----

    def _on_joint_states(self, msg: JointState):
        """接收 /joint_states 并同步当前手臂关节角（夹爪只由指令控制）"""
        self._latest_joint_msg = msg
        name_to_pos = dict(zip(msg.name, msg.position))
        self._current_arm = [float(name_to_pos.get(j, 0.0)) for j in ACTIVE_JOINTS]

    # ---- 状态读取 ----

    def get_joint_angles(self) -> list[float]:
        """
        获取机器人当前 7 个手臂关节角度（来自仿真反馈，非指令值）

        Returns:
            panda_joint1~7 的当前角度（弧度）
        """
        if self._latest_joint_msg is None:
            self.get_logger().warn("尚未收到 /joint_states 消息，返回默认值 0")
            return [0.0] * 7

        name_to_pos = dict(
            zip(self._latest_joint_msg.name, self._latest_joint_msg.position)
        )
        return [float(name_to_pos.get(j, 0.0)) for j in ACTIVE_JOINTS]

    def get_finger_width(self) -> float:
        """
        获取夹爪当前开合程度

        Returns:
            0.0=闭合, 0.04=全开
        """
        if self._latest_joint_msg is None:
            return 0.0
        name_to_pos = dict(
            zip(self._latest_joint_msg.name, self._latest_joint_msg.position)
        )
        return float(name_to_pos.get("panda_finger_joint1", 0.0))

    def get_end_effector_pose(self) -> dict[str, list[float]]:
        """
        获取当前末端执行器的位姿（通过 FK 从实际关节角计算）

        Returns:
            {"pos": [x,y,z], "rpy_rad": [r,p,y], "rpy_deg": [r,p,y]}
        """
        current_angles: list[float] = self.get_joint_angles()
        result = self.ik.forward(current_angles)
        return {
            "pos": result["pos"],
            "rpy_rad": result["rpy"],
            "rpy_deg": [math.degrees(r) for r in result["rpy"]],
        }

    # ---- 工具 ----

    def sleep(self, seconds: float):
        """等待指定秒数，期间保持 ROS2 回调处理"""
        end = _time.time() + seconds
        while _time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    # ---- 底层：直接发 9 个关节值（私有）----

    def _send(self, arm_angles: list[float], finger: float):
        """发送完整关节指令（内部使用）"""
        self._current_arm = list(arm_angles)
        self._current_finger = finger
        msg = JointState()
        msg.name = ALL_JOINTS
        msg.position = self._current_arm + [finger, finger]
        self.publisher_.publish(msg)

    # ---- 中层：传入关节角 / 夹爪控制 ----

    def set_arm(self, angles: list[float]):
        """
        直接设置 7 个手臂关节角度

        Args:
            angles: 7 个关节角度（弧度），顺序对应 panda_joint1~7
        """
        if len(angles) != 7:
            raise ValueError(f"需要 7 个关节角度，收到 {len(angles)} 个")
        self._send(angles, self._current_finger)

    def set_gripper(self, width: float):
        """
        设置夹爪开合程度

        Args:
            width: 0.0=闭合, 0.04=全开
        """
        self._send(self._current_arm, width)

    def open_gripper(self):
        self.set_gripper(0.04)

    def close_gripper(self):
        self.set_gripper(0.0)

    # ---- 高层：IK 位姿控制 ----

    def move_to_pose(
        self,
        xyz: list[float],
        rpy: Optional[list[float]] = None,
        finger: Optional[float] = None,
    ):
        """
        IK 求解并移动到指定位姿

        Args:
            xyz: [x, y, z] 目标位置（米）
            rpy: [roll, pitch, yaw] 目标姿态（弧度），None=不约束朝向
            finger: 夹爪开合，None=保持当前状态
        """
        angles: list[float] = self.ik.solve(xyz, rpy)
        result = self.ik.forward(angles)
        pos: list[float] = result["pos"]
        actual_rpy: list[float] = result["rpy"]
        pos_error: float = math.dist(xyz, pos)

        log = (
            f"目标 pos=({xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f})  "
            f"实际 pos=({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})  "
            f"误差={pos_error*1000:.1f}mm"
        )
        if rpy is not None:
            rpy_err: float = math.dist(rpy, actual_rpy)
            log += (
                f"\n  目标 rpy=({math.degrees(rpy[0]):.1f}, "
                f"{math.degrees(rpy[1]):.1f}, {math.degrees(rpy[2]):.1f})  "
                f"实际 rpy=({math.degrees(actual_rpy[0]):.1f}, "
                f"{math.degrees(actual_rpy[1]):.1f}, "
                f"{math.degrees(actual_rpy[2]):.1f})  "
                f"误差={math.degrees(rpy_err):.2f}deg"
            )
        self.get_logger().info(log)

        if finger is None:
            finger = self._current_finger
        self._send(angles, finger)

    def go_home(self):
        """回到安全零位"""
        self._send(SAFE_HOME[:7], SAFE_HOME[7])
