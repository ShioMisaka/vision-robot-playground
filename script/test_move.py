"""演示：IK 位姿控制 — 移动到指定位置和姿态"""

import math
import time

import rclpy
from src.robot import RobotController


def main():
    rclpy.init()
    robot = RobotController()

    robot.get_logger().info("等待 2 秒，建立与 Isaac Sim 的连接...")
    time.sleep(1.0)

    # 回安全零位
    # robot.get_logger().info("回安全零位...")
    # robot.go_home()
    # time.sleep(3.0)

    robot.get_logger().info("--- 张开夹爪 ---")
    robot.open_gripper()
    time.sleep(1.0)

    # 位置 + 姿态（夹爪朝下，适合抓取）
    robot.move_to_pose([0.526, 0.0, 0.18], rpy=[0.0, math.radians(-180), math.radians(-180)])
    time.sleep(3.0)

    # 闭合夹爪
    robot.get_logger().info("--- 闭合夹爪 ---")
    robot.close_gripper()
    time.sleep(1.0)

    robot.move_to_pose([0.526, 0.0, 0.18], rpy=[0.0, math.radians(-180), math.radians(-180)])
    time.sleep(3.0)

    robot.get_logger().info("完成！")
    robot.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
