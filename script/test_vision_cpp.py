"""演示：视觉引导抓取 — C++ 核心库后端

使用 C++ VisionProcessorNode + ColorDetector 进行目标检测，
Python 侧实现视觉伺服居中逻辑。

注意：此脚本需要独立 robot_controller_node 先启动。

流程：
    1. 移动到观察位（俯视桌面）
    2. 检测红色物块，连续锁定 15 帧确认
    3. 视觉伺服居中：根据像素偏移调整机械臂 XY
    4. 居中后下探 → 闭合夹爪 → 提起

注意：此脚本使用 rclcpp（通过 pybind11），不能同时使用 rclpy。
"""

import math
import threading

from robot_api_python import (
    rclcpp_init,
    rclcpp_shutdown,
    RobotClient,
    VisionProcessorNode,
    MultiThreadedExecutor,
    VisionTopicConfig,
    ColorDetector,
)


# ---- 视觉伺服参数 ----
DIRECTION_X: float = -1.0   # 图像右偏 → 机器人 +X 移动方向
DIRECTION_Y: float = 1.0    # 图像下偏 → 机器人 +Y 移动方向
MOVE_STEP: float = 0.02     # 每次居中调整的步长（米）
CENTER_THRESHOLD: float = 0.03  # 居中判定阈值（归一化偏移量）
LOCK_FRAMES: int = 15       # 连续检测到目标多少帧才确认锁定
DESCEND_DISTANCE: float = 0.15  # 下探距离（米）
LIFT_DISTANCE: float = 0.25     # 提起距离（米）

# 相机图像尺寸（用于归一化偏移计算）
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480


def main() -> None:
    rclcpp_init()

    detector = ColorDetector([0, 100, 100], [10, 255, 255])
    config = VisionTopicConfig()

    robot = RobotClient.create()
    vision = VisionProcessorNode.create(detector, config)

    executor = MultiThreadedExecutor()
    executor.add_node(robot)
    executor.add_node(vision)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    robot.get_logger().info("等待控制器就绪...")
    robot.wait_for_services()

    ctrl = robot.get_controller()
    ctrl.open_gripper()

    # 移动到观察位（俯视桌面）
    ctrl.move_to_pose(
        [0.526, 0.0, 0.4],
        rpy=[0.0, math.radians(-180), math.radians(-180)],
    )

    state = "observe"
    lock_count = 0

    robot.get_logger().info("开始视觉引导抓取")

    try:
        while state != "done":
            result = vision.get_latest_result()

            # ---- OBSERVE: 搜索并锁定目标 ----
            if state == "observe":
                if result and result.detected:
                    lock_count += 1
                    if lock_count >= LOCK_FRAMES:
                        state = "centering"
                        robot.get_logger().info("目标锁定，开始视觉伺服居中")
                else:
                    lock_count = 0

            # ---- CENTERING: 视觉伺服，移动机械臂使目标居中 ----
            elif state == "centering":
                if not (result and result.detected):
                    robot.get_logger().warn("丢失目标，回到观察状态")
                    state = "observe"
                    lock_count = 0
                    continue

                uv = result.uv  # [u, v] 像素坐标
                cx, cy = uv[0], uv[1]

                # 归一化偏移量 [-0.5, 0.5]
                ex = (cx - IMAGE_WIDTH / 2) / IMAGE_WIDTH
                ey = (cy - IMAGE_HEIGHT / 2) / IMAGE_HEIGHT

                robot.get_logger().info(
                    f"偏移: ex={ex:.3f}, ey={ey:.3f}"
                )

                if abs(ex) < CENTER_THRESHOLD and abs(ey) < CENTER_THRESHOLD:
                    robot.get_logger().info("目标已居中，准备下探")
                    state = "descending"
                else:
                    dx = DIRECTION_X * ex * MOVE_STEP * 2
                    dy = DIRECTION_Y * ey * MOVE_STEP * 2
                    ctrl.move_linear([dx, dy, 0])

            # ---- DESCENDING: 下探到目标 ----
            elif state == "descending":
                robot.get_logger().info(
                    f"下探 {DESCEND_DISTANCE:.2f}m..."
                )
                ctrl.move_linear([0, 0, -DESCEND_DISTANCE])
                state = "grasping"

            # ---- GRASPING: 闭合夹爪 ----
            elif state == "grasping":
                robot.get_logger().info("闭合夹爪...")
                ctrl.close_gripper()
                state = "lifting"

            # ---- LIFTING: 提起物体 ----
            elif state == "lifting":
                robot.get_logger().info(
                    f"提起物体 {LIFT_DISTANCE:.2f}m..."
                )
                ctrl.move_linear([0, 0, LIFT_DISTANCE])
                robot.get_logger().info("抓取完成!")
                state = "done"

    except KeyboardInterrupt:
        pass

    ctrl.go_home()
    executor.cancel()
    spin_thread.join()
    rclcpp_shutdown()


if __name__ == "__main__":
    main()
