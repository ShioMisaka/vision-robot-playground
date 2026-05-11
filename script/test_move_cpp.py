"""演示：IK 位姿控制 — 使用 C++ 核心库后端

注意：此脚本需要独立 robot_controller_node 先启动。

使用 C++ ServiceRobotController + rclcpp 执行运动控制，
pybind11 桥接 Python 与 C++ 库。
"""

import math
import threading

from robot_api_python import (
    rclcpp_init,
    rclcpp_shutdown,
    RobotClient,
    MultiThreadedExecutor,
    load_profile,
)

# ---- 目标参数 ----
TARGET_XYZ = [0.52699, 0.0, 0.04026]
APPROACH_HEIGHT = 0.10   # 目标上方 10cm
LIFT_HEIGHT = 0.15       # 提起高度 15cm
GRIPPER_DOWN_RPY = [0.0, math.radians(-180), math.radians(-180)]


def main() -> None:
    rclcpp_init()

    gripper = load_profile().gripper
    robot = RobotClient.create()

    executor = MultiThreadedExecutor()
    executor.add_node(robot)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    robot.get_logger().info("等待控制器就绪...")
    robot.wait_for_services()

    ctrl = robot.get_controller()

    robot.get_logger().info("--- 张开夹爪 ---")
    ctrl.open_gripper()

    # 位置 + 姿态（夹爪朝下，适合抓取）
    ctrl.moveJ(
        [0.526, 0.0, 0.18],
        rpy=[0.0, math.radians(-180), math.radians(-180)],
    )

    # 闭合夹爪
    robot.get_logger().info("--- 闭合夹爪 ---")
    ctrl.close_gripper()

    robot.get_logger().info("--- 抬起 ---")
    ctrl.moveL([0.526, 0.0, 0.38], rpy=GRIPPER_DOWN_RPY, finger=gripper.max_width)

    robot.get_logger().info("--- 旋转 ---")
    ctrl.rotate_joint(0, math.radians(90))

    robot.get_logger().info("完成!")

    executor.cancel()
    spin_thread.join()
    rclcpp_shutdown()


if __name__ == "__main__":
    main()
