"""演示：IK 位姿控制 — 使用 C++ 核心库后端

使用 C++ RobotMotionController + rclcpp 执行运动控制，
pybind11 桥接 Python 与 C++ 库。
"""

import math
import threading

from robot_control_cpp_py import (
    rclcpp_init,
    rclcpp_shutdown,
    RobotControllerNode,
    MultiThreadedExecutor,
    TopicConfig,
    profiles,
)


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

    robot.get_logger().info("--- 张开夹爪 ---")
    ctrl.open_gripper()

    # 位置 + 姿态（夹爪朝下，适合抓取）
    ctrl.move_to_pose(
        [0.526, 0.0, 0.18],
        rpy=[0.0, math.radians(-180), math.radians(-180)],
    )

    # 闭合夹爪
    robot.get_logger().info("--- 闭合夹爪 ---")
    ctrl.close_gripper()

    ctrl.move_linear([0, 0, 0.2])

    ctrl.rotate_joint(0, math.radians(90))

    ctrl.rotate_joint(3, math.radians(90))

    robot.get_logger().info("完成!")

    executor.cancel()
    spin_thread.join()
    rclcpp_shutdown()


if __name__ == "__main__":
    main()
