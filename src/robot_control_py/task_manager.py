"""抓取任务状态机：协调视觉检测与机械臂控制

架构说明：
    GraspTaskManager 是上层的任务编排器，在主线程中运行状态机，
    协调 VisionProcessor（目标检测）和 RobotController（机械臂控制）
    完成视觉引导抓取流程。

    所有节点通过 MultiThreadedExecutor 在后台线程中 spin，
    状态机的阻塞式业务逻辑（time.sleep 等）不会影响回调处理。

状态流转：
    IDLE -> DETECTING -> APPROACHING -> DESCENDING -> GRASPING -> LIFTING -> DONE
                                                                              -> ERROR

使用方式：
    import threading, rclpy
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init()
    executor = MultiThreadedExecutor()

    robot = RobotController()
    vision = VisionProcessor()
    task_mgr = GraspTaskManager(robot, vision)

    for node in [robot, vision]:
        executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    robot.wait_for_ready()
    success = task_mgr.run()  # 阻塞运行状态机
"""

from __future__ import annotations

import enum
import math
import time
from typing import Optional

from scipy.spatial.transform import Rotation

from src.robot_control_py.config import FRAME_BASE, FRAME_CAMERA
from src.robot_control_py.robot import RobotController
from src.robot_control_py.vision_processor import VisionProcessor


class GraspState(enum.Enum):
    """抓取任务状态枚举"""
    IDLE = "idle"               # 空闲，等待启动
    DETECTING = "detecting"     # 视觉检测目标
    APPROACHING = "approaching" # 机械臂移动到目标正上方
    DESCENDING = "descending"   # 机械臂下降到抓取位置
    GRASPING = "grasping"       # 闭合夹爪
    LIFTING = "lifting"         # 提起物体
    DONE = "done"               # 完成
    ERROR = "error"             # 出错


class GraspTaskManager:
    """视觉引导抓取任务管理器

    在主线程中运行状态机，协调视觉检测与机械臂控制。
    需要与 RobotController 和 VisionProcessor 配合使用，
    所有节点必须在 MultiThreadedExecutor 中运行。
    """

    def __init__(
        self,
        robot: RobotController,
        vision: VisionProcessor,
        approach_height: float = 0.15,
        grasp_height_offset: float = 0.02,
        grasp_rpy: Optional[list[float]] = None,
    ) -> None:
        """初始化抓取任务管理器

        Args:
            robot: RobotController 实例
            vision: VisionProcessor 实例
            approach_height: 接近时末端在目标上方的偏移高度（米）
            grasp_height_offset: 抓取时末端相对于目标的高度偏移（米）
            grasp_rpy: 抓取姿态 [roll, pitch, yaw]（弧度），
                       默认夹爪朝下 [pi, 0, pi]
        """
        self._robot = robot
        self._vision = vision
        self._approach_height = approach_height
        self._grasp_height_offset = grasp_height_offset
        self._grasp_rpy = grasp_rpy if grasp_rpy is not None else [math.pi, 0.0, math.pi]
        self._state: GraspState = GraspState.IDLE
        self._target_xyz: Optional[list[float]] = None
        self._logger = robot.get_logger()

    @property
    def state(self) -> GraspState:
        """当前任务状态"""
        return self._state

    # ---- 核心状态机 ----

    def run(self, timeout: float = 30.0) -> bool:
        """运行完整的抓取流程（阻塞）

        Args:
            timeout: 整体任务超时时间（秒）

        Returns:
            True=抓取成功, False=失败或超时
        """
        start_time = time.time()
        self._state = GraspState.DETECTING

        try:
            while self._state not in (GraspState.DONE, GraspState.ERROR):
                if time.time() - start_time > timeout:
                    self._logger.error("抓取任务超时")
                    self._state = GraspState.ERROR
                    return False

                self._logger.info(f"当前状态: {self._state.value}")

                if self._state == GraspState.DETECTING:
                    if not self._step_detect():
                        time.sleep(0.5)
                        continue

                elif self._state == GraspState.APPROACHING:
                    self._step_approach()

                elif self._state == GraspState.DESCENDING:
                    self._step_descend()

                elif self._state == GraspState.GRASPING:
                    self._step_grasp()

                elif self._state == GraspState.LIFTING:
                    self._step_lift()

            return self._state == GraspState.DONE

        except Exception as e:
            self._logger.error(f"抓取任务异常: {e}")
            self._state = GraspState.ERROR
            return False

    # ---- 状态步骤 ----

    def _step_detect(self) -> bool:
        """检测目标物体，获取 3D 坐标

        从 VisionProcessor 获取最新检测结果。
        如果检测到目标，将相机坐标系下的坐标转换为基座坐标系。

        Returns:
            True=检测到目标并已记录 3D 坐标
        """
        self._logger.info("正在检测目标物体...")
        result = self._vision.get_latest_result()

        if result is None or not result.get("detected", False):
            self._logger.warn("未检测到目标，继续等待...")
            return False

        self._target_xyz = result.get("xyz")
        if self._target_xyz is None:
            return False

        self._logger.info(
            f"检测到目标（相机系）: XYZ=({self._target_xyz[0]:.3f}, "
            f"{self._target_xyz[1]:.3f}, {self._target_xyz[2]:.3f})"
        )

        # 将相机坐标系下的目标坐标转换为基座坐标系
        base_xyz = self._transform_to_base(self._target_xyz)
        if base_xyz is not None:
            self._target_xyz = base_xyz
            self._logger.info(
                f"检测到目标（基座系）: XYZ=({base_xyz[0]:.3f}, "
                f"{base_xyz[1]:.3f}, {base_xyz[2]:.3f})"
            )

        self._state = GraspState.APPROACHING
        return True

    def _step_approach(self) -> None:
        """移动到目标正上方"""
        assert self._target_xyz is not None
        target = list(self._target_xyz)
        target[2] += self._approach_height

        self._logger.info(
            f"移动到目标上方: ({target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f})"
        )
        self._robot.move_to_pose(target, rpy=self._grasp_rpy)
        time.sleep(2.0)

        self._state = GraspState.DESCENDING

    def _step_descend(self) -> None:
        """下降到抓取位置"""
        assert self._target_xyz is not None
        target = list(self._target_xyz)
        target[2] += self._grasp_height_offset

        self._logger.info(
            f"下降到抓取位置: ({target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f})"
        )
        self._robot.move_to_pose(target, rpy=self._grasp_rpy)
        time.sleep(2.0)

        self._state = GraspState.GRASPING

    def _step_grasp(self) -> None:
        """闭合夹爪抓取"""
        self._logger.info("闭合夹爪...")
        self._robot.close_gripper()
        time.sleep(1.0)
        self._state = GraspState.LIFTING

    def _step_lift(self) -> None:
        """提起物体"""
        lift_height = self._approach_height + 0.1

        self._logger.info(f"提起物体，上升 {lift_height:.3f}m")
        self._robot.move_linear([0, 0, lift_height])
        time.sleep(2.0)

        self._state = GraspState.DONE
        self._logger.info("抓取完成!")

    # ---- 坐标变换 ----

    def _transform_to_base(
        self, camera_xyz: list[float]
    ) -> Optional[list[float]]:
        """将相机坐标系下的 3D 点转换为基座坐标系

        通过 TF2 查询 camera -> base_link 变换矩阵，
        执行旋转 + 平移变换：base_point = R * camera_point + t

        Args:
            camera_xyz: [x, y, z] 在相机坐标系下的位置（米）

        Returns:
            [x, y, z] 在基座坐标系下的位置（米），TF 查询失败返回 None
        """
        tf = self._robot.lookup_transform(FRAME_BASE, FRAME_CAMERA)
        if tf is None:
            self._logger.warn(
                "无法获取相机到基座的 TF 变换，使用原始相机坐标"
            )
            return None

        q = tf.transform.rotation
        rot = Rotation.from_quat([q.x, q.y, q.z, q.w])
        t = tf.transform.translation

        point = rot.apply(camera_xyz)
        return [point[0] + t.x, point[1] + t.y, point[2] + t.z]
