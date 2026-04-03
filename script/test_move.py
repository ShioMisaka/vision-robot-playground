"""演示：IK 位姿控制 — 移动到指定位置和姿态"""

import math

import rclpy
from src.robot import RobotController


def main():
    rclpy.init()
    robot = RobotController()

    robot.get_logger().info("等待 2 秒，建立与 Isaac Sim 的连接...")
    robot.wait_for_ready()

    robot.get_logger().info("--- 张开夹爪 ---")
    robot.open_gripper()
    robot.sleep(1.0)

    # 位置 + 姿态（夹爪朝下，适合抓取）
    robot.move_to_pose([0.526, 0.0, 0.18], rpy=[0.0, math.radians(-180), math.radians(-180)])
    robot.sleep(2.0)

    # 闭合夹爪
    robot.get_logger().info("--- 闭合夹爪 ---")
    robot.close_gripper()
    robot.sleep(1.0)

    robot.move_linear([0, 0, 0.2])
    robot.sleep(1.0)

    robot.rotate_joint(0, math.radians(90))
    robot.sleep(1.0)

    robot.rotate_joint(3, math.radians(90))
    robot.sleep(3.0)

    robot.get_logger().info("完成！")
    robot.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
