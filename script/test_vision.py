"""演示：视觉引导抓取 — 检测红色目标并自动抓取"""

import time

import cv2
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from src.robot import RobotController
from src.vision import RED_DETECTOR


class VisionGrasp:
    """视觉引导抓取状态机"""

    def __init__(self):
        self.robot = RobotController()
        self.bridge = CvBridge()
        self.subscription = self.robot.create_subscription(
            Image, "/camera/image_raw", self.on_image, 10
        )

        # 状态机：0=搜索目标, 1=抓取中, 2=完成
        self.state = 0
        self.lock_frames = 0

    def on_image(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        target = RED_DETECTOR.detect(cv_image)

        if target:
            cx, cy = target
            RED_DETECTOR.draw_target(cv_image, cx, cy, "Red Block")

            if self.state == 0:
                self.lock_frames += 1
                cv2.putText(
                    cv_image, f"Locking... {self.lock_frames}/15",
                    (cx - 60, cy - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2
                )
                if self.lock_frames > 15:
                    self.robot.get_logger().info("目标锁定，开始抓取！")
                    self.execute_grasp()
        else:
            self.lock_frames = 0

        cv2.imshow("Robot Eyes", cv_image)
        cv2.waitKey(1)

    def execute_grasp(self):
        self.state = 1

        self.robot.get_logger().info("步骤 1/3: 下探...")
        self.robot.send_joints([0.0, 0.3, 0.0, -2.2, 0.0, 2.4, 0.78, 0.04, 0.04])
        time.sleep(2.5)

        self.robot.get_logger().info("步骤 2/3: 闭合夹爪...")
        self.robot.send_joints([0.0, 0.3, 0.0, -2.2, 0.0, 2.4, 0.78, 0.0, 0.0])
        time.sleep(1.0)

        self.robot.get_logger().info("步骤 3/3: 举起...")
        self.robot.send_joints([0.0, -0.5, 0.0, -1.5, 0.0, 1.5, 0.78, 0.0, 0.0])
        time.sleep(2.0)

        self.robot.get_logger().info("抓取完成！")
        self.state = 2


def main():
    rclpy.init()
    node = VisionGrasp()

    node.robot.get_logger().info("等待 2 秒，建立连接...")
    time.sleep(2.0)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    cv2.destroyAllWindows()
    node.robot.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
