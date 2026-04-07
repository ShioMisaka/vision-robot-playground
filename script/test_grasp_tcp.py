"""演示：使用 grasptarget TCP 抓取指定位置

流程：
    1. 切换到 grasptarget TCP（指尖坐标系）
    2. 张开夹爪
    3. 移动到目标上方（夹爪朝下）
    4. 下降到目标位置
    5. 闭合夹爪
    6. 提起
"""

import math
import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor

from src.robot import RobotController

# ---- 目标参数 ----
TARGET_XYZ = [0.52699, 0.0, 0.0]
APPROACH_HEIGHT = 0.10   # 目标上方 10cm
LIFT_HEIGHT = 0.15       # 提起高度 15cm
GRIPPER_DOWN_RPY = [0.0, math.radians(-180), math.radians(-180)]

# ---- 运动参数 ----
INTERP_STEPS = 15        # 插值步数（越大越平滑）
STEP_TIME = 0.06         # 每步间隔（秒）


def main() -> None:
    rclpy.init()
    robot = RobotController()

    executor = MultiThreadedExecutor()
    executor.add_node(robot)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    robot.get_logger().info("等待与 Isaac Sim 建立连接...")
    robot.wait_for_ready()

    # 切换到指尖坐标系
    robot.set_tcp("grasptarget")

    # 张开夹爪
    robot.open_gripper()

    # 移动到目标上方
    robot.get_logger().info("--- 移动到目标上方 ---")
    approach = [TARGET_XYZ[0], TARGET_XYZ[1], TARGET_XYZ[2] + APPROACH_HEIGHT]
    robot.move_to_pose(approach, rpy=GRIPPER_DOWN_RPY, steps=INTERP_STEPS, step_time=STEP_TIME)

    # 下降到目标位置
    robot.get_logger().info("--- 下降到目标 ---")
    robot.move_to_pose(TARGET_XYZ, rpy=GRIPPER_DOWN_RPY, steps=INTERP_STEPS, step_time=STEP_TIME)

    # 闭合夹爪
    robot.get_logger().info("--- 闭合夹爪 ---")
    robot.close_gripper()

    # 提起
    robot.get_logger().info("--- 提起 ---")
    lift = [TARGET_XYZ[0], TARGET_XYZ[1], TARGET_XYZ[2] + LIFT_HEIGHT]
    robot.move_to_pose(lift, rpy=GRIPPER_DOWN_RPY, steps=INTERP_STEPS, step_time=STEP_TIME)

    robot.get_logger().info("完成!")
    executor.shutdown()
    robot.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
