"""演示：使用 grasptarget TCP 抓取指定位置 — C++ 核心库后端

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

from robot_api_python import (
    rclcpp_init,
    rclcpp_shutdown,
    RobotControllerNode,
    MultiThreadedExecutor,
    TopicConfig,
    profiles,
)

# ---- 目标参数 ----
TARGET_XYZ = [0.52699, 0.0, 0.04026]
APPROACH_HEIGHT = 0.10   # 目标上方 10cm
LIFT_HEIGHT = 0.15       # 提起高度 15cm
GRIPPER_DOWN_RPY = [0.0, math.radians(-180), math.radians(-180)]

# ---- 运动参数 ----


def main() -> None:
    rclcpp_init()

    profile = profiles.panda()
    gripper = profiles.panda_gripper()
    robot = RobotControllerNode.create(profile, gripper, TopicConfig())

    executor = MultiThreadedExecutor()
    executor.add_node(robot)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    robot.get_logger().info("等待与 Isaac Sim 建立连接...")
    robot.wait_for_ready()

    ctrl = robot.get_controller()

    # 切换到指尖坐标系
    ctrl.set_tcp("grasptarget")

    # 张开夹爪
    ctrl.open_gripper()

    # 移动到目标上方
    robot.get_logger().info("--- 移动到目标上方 ---")
    approach = [TARGET_XYZ[0], TARGET_XYZ[1], TARGET_XYZ[2] + APPROACH_HEIGHT]
    ctrl.moveJ(approach, rpy=GRIPPER_DOWN_RPY, finger=gripper.max_width)

    # 下降到目标位置
    robot.get_logger().info("--- 下降到目标 ---")
    ctrl.moveL(TARGET_XYZ, rpy=GRIPPER_DOWN_RPY, finger=gripper.max_width)

    # 闭合夹爪
    robot.get_logger().info("--- 闭合夹爪 ---")
    ctrl.close_gripper()

    # 提起
    robot.get_logger().info("--- 提起 ---")
    lift = [TARGET_XYZ[0], TARGET_XYZ[1], TARGET_XYZ[2] + LIFT_HEIGHT]
    ctrl.moveL(lift, rpy=GRIPPER_DOWN_RPY, finger=gripper.min_width)

    robot.get_logger().info("完成!")

    executor.cancel()
    spin_thread.join()
    rclcpp_shutdown()


if __name__ == "__main__":
    main()
