"""测试：显示 ZEN_X_Mini 左目 RGB 与深度图，验证相机数据正常获取

用法：
    python3 script/test_camera.py

功能：
    1. 订阅左目 RGB 和深度图话题
    2. 深度图自动归一化到 0-255 灰度显示
    3. 按 q 退出
"""

from __future__ import annotations

import threading

import cv2
import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber

from src.robot_control_py.config import TOPIC_CAMERA_LEFT, TOPIC_CAMERA_DEPTH


class CameraDisplayNode(Node):
    """订阅左目 + 深度图像并在窗口中显示"""

    def __init__(self) -> None:
        super().__init__("camera_display")

        self._bridge = CvBridge()
        self._frame_lock = threading.Lock()
        self._left_frame: np.ndarray | None = None
        self._depth_frame: np.ndarray | None = None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        left_sub = Subscriber(self, Image, TOPIC_CAMERA_LEFT, qos_profile=qos)
        depth_sub = Subscriber(self, Image, TOPIC_CAMERA_DEPTH, qos_profile=qos)

        self._sync = ApproximateTimeSynchronizer(
            [left_sub, depth_sub], queue_size=10, slop=0.1,
        )
        self._sync.registerCallback(self._on_images)

        self.get_logger().info(
            f"CameraDisplay 已启动，订阅话题：{TOPIC_CAMERA_LEFT}, {TOPIC_CAMERA_DEPTH}"
        )

    def _on_images(self, left_msg: Image, depth_msg: Image) -> None:
        """同步回调：转换并缓存最新帧"""
        try:
            left = self._bridge.imgmsg_to_cv2(left_msg, "bgr8")
            depth = self._bridge.imgmsg_to_cv2(depth_msg, "passthrough")
        except Exception as e:
            self.get_logger().error(f"图像转换失败: {e}")
            return

        # 归一化深度图到 0-255 灰度，方便可视化
        if depth.dtype == np.uint16:
            depth_vis = np.clip(depth, 0, 5000).astype(np.float32)
            depth_vis = (depth_vis / 5000.0 * 255).astype(np.uint8)
        elif depth.dtype == np.float32:
            depth_vis = np.clip(depth, 0, 5.0)
            depth_vis = (depth_vis / 5.0 * 255).astype(np.uint8)
        else:
            depth_vis = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX)
            depth_vis = depth_vis.astype(np.uint8)

        # 着色为伪彩色以便观察
        depth_color = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)

        with self._frame_lock:
            self._left_frame = left
            self._depth_frame = depth_color

        self.get_logger().info(
            f"收到图像: 左目 {left_msg.width}x{left_msg.height} "
            f"encoding={left_msg.encoding}, "
            f"深度 {depth_msg.width}x{depth_msg.height} "
            f"encoding={depth_msg.encoding} dtype={depth.dtype}"
        )

    def get_frames(self) -> tuple[np.ndarray | None, np.ndarray | None]:
        """获取最新的左目和深度帧（线程安全）"""
        with self._frame_lock:
            return self._left_frame, self._depth_frame


def main() -> None:
    rclpy.init()

    node = CameraDisplayNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    node.get_logger().info("等待相机图像...按 q 退出")

    try:
        while rclpy.ok():
            left, depth = node.get_frames()

            if left is not None and depth is not None:
                # 水平拼接显示
                combined = cv2.hconcat([left, depth])
                cv2.imshow("Left RGB | Depth (Jet)", combined)
            else:
                cv2.waitKey(100)
                continue

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break

    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()

    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
