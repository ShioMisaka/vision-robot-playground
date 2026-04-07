"""演示：IK 位姿控制 — 移动到指定位置和姿态

使用 MultiThreadedExecutor 在后台线程处理回调，
主线程中执行阻塞式业务逻辑。
"""

import math
import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor

from src.robot import RobotController


def main() -> None:
    rclpy.init()
    robot = RobotController()

    # MultiThreadedExecutor: 允许回调在独立线程中执行，
    # 主线程的阻塞式运动指令不会卡死回调处理
    executor = MultiThreadedExecutor()
    executor.add_node(robot)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    robot.get_logger().info("等待与 Isaac Sim 建立连接...")
    robot.wait_for_ready()  # 通过 threading.Event 阻塞等待，不使用 spin_once

    robot.get_logger().info("--- 张开夹爪 ---")
    robot.open_gripper()

    # 位置 + 姿态（夹爪朝下，适合抓取）
    robot.move_to_pose(
        [0.526, 0.0, 0.18],
        rpy=[0.0, math.radians(-180), math.radians(-180)],
    )

    # 闭合夹爪
    robot.get_logger().info("--- 闭合夹爪 ---")
    robot.close_gripper()

    robot.move_linear([0, 0, 0.2])

    robot.rotate_joint(0, math.radians(90))

    robot.rotate_joint(3, math.radians(90))

    robot.get_logger().info("完成!")

    executor.shutdown()
    robot.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
