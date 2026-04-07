"""演示：视觉引导抓取 — 检测红色物块并自主完成抓取

流程：
    1. 移动到观察位（俯视桌面）
    2. 检测红色物块，连续锁定 15 帧确认
    3. 视觉伺服居中：根据像素偏移调整机械臂 XY，使目标移到画面中心
    4. 居中后下探 → 闭合夹爪 → 提起
"""

import enum
import threading
import time

import cv2
import numpy as np
import math
import rclpy
from rclpy.executors import MultiThreadedExecutor

from src.robot import RobotController
from src.vision import RED_DETECTOR
from src.vision_processor import VisionProcessor


# ---- 视觉伺服参数（需根据实际相机安装方向校准）----
# 如果居中方向相反，反转对应符号
DIRECTION_X: float = -1.0   # 图像右偏 → 机器人 +X 移动方向（-1 或 1）
DIRECTION_Y: float = 1.0    # 图像下偏 → 机器人 +Y 移动方向（-1 或 1）
MOVE_STEP: float = 0.02     # 每次居中调整的步长（米）
CENTER_THRESHOLD: float = 0.03  # 居中判定阈值（归一化偏移量）
LOCK_FRAMES: int = 15       # 连续检测到目标多少帧才确认锁定
DESCEND_DISTANCE: float = 0.15  # 下探距离（米）
LIFT_DISTANCE: float = 0.25     # 提起距离（米）


class VisionState(enum.Enum):
    """视觉引导抓取状态"""
    OBSERVE = "observe"       # 搜索目标
    CENTERING = "centering"   # 视觉伺服居中
    DESCENDING = "descending" # 下探
    GRASPING = "grasping"     # 闭合夹爪
    LIFTING = "lifting"       # 提起
    DONE = "done"             # 完成


class RedBlockVisionProcessor(VisionProcessor):
    """带红色物体检测的视觉处理器

    在左目图像上运行 HSV 颜色检测，标注结果并保存画面帧，
    供主线程通过 cv2.imshow 显示。
    """

    def __init__(self) -> None:
        super().__init__()
        self._detector = RED_DETECTOR
        self._annotated_lock = threading.Lock()
        self._annotated_frame: np.ndarray | None = None

    def process_image(
        self,
        rgb_image: np.ndarray,
        depth_image: np.ndarray,
    ) -> dict:
        """在左目图像上检测红色物块并标注画面"""
        annotated = rgb_image.copy()
        target = self._detector.detect(rgb_image)

        result = {
            "detected": False,
            "xyz": None,
            "uv": None,
            "confidence": 0.0,
            "grasp_pose": None,
        }

        if target is not None:
            cx, cy = target
            self._detector.draw_target(annotated, cx, cy, "Red Block")

            # 读取目标像素处的深度值
            depth_val = float(depth_image[cy, cx])
            # uint16 格式深度图单位通常为毫米，转换为米
            if depth_image.dtype in (np.uint16, np.int32):
                depth_val = depth_val / 1000.0

            if depth_val > 0:
                result["detected"] = True
                result["depth"] = depth_val
                result["uv"] = [cx, cy]
                result["confidence"] = 1.0
            else:
                cv2.putText(
                    annotated, "Depth=0", (cx - 40, cy + 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2,
                )
        else:
            cv2.putText(
                annotated, "No target", (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2,
            )

        with self._annotated_lock:
            self._annotated_frame = annotated

        return result

    def get_annotated_frame(self) -> np.ndarray | None:
        """获取最新标注画面（线程安全）"""
        with self._annotated_lock:
            return self._annotated_frame


def main() -> None:
    rclpy.init()

    robot = RobotController()
    vision = RedBlockVisionProcessor()

    executor = MultiThreadedExecutor()
    for node in [robot, vision]:
        executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    robot.get_logger().info("等待机器人就绪...")
    robot.wait_for_ready()
    robot.open_gripper()

    # 移动到观察位（俯视桌面）
    robot.move_to_pose(
        [0.526, 0.0, 0.4],
        rpy=[0.0, math.radians(-180), math.radians(-180)],
    )

    state = VisionState.OBSERVE
    lock_count = 0

    robot.get_logger().info("开始视觉引导抓取，按 q 退出")

    try:
        while rclpy.ok() and state != VisionState.DONE:
            frame = vision.get_annotated_frame()
            if frame is not None:
                cv2.imshow("Robot Vision", frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break

            result = vision.get_latest_result()

            # ---- OBSERVE: 搜索并锁定目标 ----
            if state == VisionState.OBSERVE:
                if result and result.get("detected"):
                    lock_count += 1
                    if lock_count >= LOCK_FRAMES:
                        state = VisionState.CENTERING
                        robot.get_logger().info("目标锁定，开始视觉伺服居中")
                else:
                    lock_count = 0

            # ---- CENTERING: 视觉伺服，移动机械臂使目标居中 ----
            elif state == VisionState.CENTERING:
                if not (result and result.get("detected")):
                    robot.get_logger().warn("丢失目标，回到观察状态")
                    state = VisionState.OBSERVE
                    lock_count = 0
                    continue

                h, w = frame.shape[:2]
                cx, cy = result["uv"]

                # 归一化偏移量 [-0.5, 0.5]，正值=目标偏右/偏下
                ex = (cx - w / 2) / w
                ey = (cy - h / 2) / h

                robot.get_logger().info(
                    f"偏移: ex={ex:.3f}, ey={ey:.3f}"
                )

                if abs(ex) < CENTER_THRESHOLD and abs(ey) < CENTER_THRESHOLD:
                    robot.get_logger().info("目标已居中，准备下探")
                    state = VisionState.DESCENDING
                else:
                    # 根据像素偏移量调整机械臂位置
                    dx = DIRECTION_X * ex * MOVE_STEP * 2
                    dy = DIRECTION_Y * ey * MOVE_STEP * 2
                    robot.move_linear([dx, dy, 0])

            # ---- DESCENDING: 下探到目标 ----
            elif state == VisionState.DESCENDING:
                robot.get_logger().info(
                    f"下探 {DESCEND_DISTANCE:.2f}m..."
                )
                robot.move_linear([0, 0, -DESCEND_DISTANCE])
                state = VisionState.GRASPING

            # ---- GRASPING: 闭合夹爪 ----
            elif state == VisionState.GRASPING:
                robot.get_logger().info("闭合夹爪...")
                robot.close_gripper()
                state = VisionState.LIFTING

            # ---- LIFTING: 提起物体 ----
            elif state == VisionState.LIFTING:
                robot.get_logger().info(
                    f"提起物体 {LIFT_DISTANCE:.2f}m..."
                )
                robot.move_linear([0, 0, LIFT_DISTANCE])
                robot.get_logger().info("抓取完成!")
                state = VisionState.DONE

    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()

    robot.go_home()
    executor.shutdown()
    robot.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
