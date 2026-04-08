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

# ---- 目标参数 ----
TARGET_XYZ = [0.52699, 0.0, 0.04026]
APPROACH_HEIGHT = 0.10   # 目标上方 10cm
LIFT_HEIGHT = 0.15       # 提起高度 15cm
GRIPPER_DOWN_RPY = [0.0, math.radians(-180), math.radians(-180)]

# ---- 运动参数 ----
INTERP_STEPS = 15        # 插值步数（越大越平滑）
STEP_TIME = 0.06         # 每步间隔（秒）


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

    robot.get_logger().info("--- 抬起 ---")
    ctrl.move_linear([0, 0, 0.2], rpy=GRIPPER_DOWN_RPY, finger=gripper.max_width,
                      steps=INTERP_STEPS, step_time=STEP_TIME)

    robot.get_logger().info("--- 旋转 ---")
    ctrl.rotate_joint(0, math.radians(90), 15, 0.6)

    # ctrl.rotate_joint(3, math.radians(90))

    robot.get_logger().info("完成!")

    executor.cancel()
    spin_thread.join()
    rclcpp_shutdown()


if __name__ == "__main__":
    main()
