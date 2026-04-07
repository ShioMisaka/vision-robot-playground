"""ROS2 视觉处理节点：左目 + 深度图像同步与目标检测

架构说明：
    使用 message_filters.ApproximateTimeSynchronizer 同步订阅
    ZEN_X_Mini 的左目 RGB 图像和深度图像，确保时间戳对齐后再执行检测逻辑。

    同步回调中提供 process_image() 桩代码（Stub），
    供后续接入 YOLO 或 GraspNet 等 6D 姿态估计网络。
    子类可重写 process_image() 接入自定义检测器。
"""

from __future__ import annotations

import time
import threading
from typing import Optional

import numpy as np
import numpy.typing as npt
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber

from src.robot_control_py.config import TOPIC_CAMERA_LEFT, TOPIC_CAMERA_DEPTH


class VisionProcessor(Node):
    """左目 + 深度视觉处理节点

    订阅 ZEN_X_Mini 左目 RGB 和深度图像，使用 ApproximateTimeSynchronizer
    确保时间戳对齐，并在同步回调中执行目标检测。

    使用方式：
        processor = VisionProcessor()
        executor = MultiThreadedExecutor()
        executor.add_node(processor)
        # executor.spin() 在后台线程中运行
        # 主线程通过 get_latest_result() 获取检测结果
    """

    def __init__(self) -> None:
        super().__init__("vision_processor")

        self._bridge = CvBridge()

        # 最新检测结果（线程安全，供外部读取）
        self._result_lock = threading.Lock()
        self._latest_result: Optional[dict] = None

        # ---- QoS 配置 ----
        # 相机驱动通常使用 BestEffort 可靠性策略以降低延迟，
        # 订阅端也匹配 BestEffort，避免因 RELIABLE 策略导致消息被丢弃
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ---- 使用 message_filters 进行时间戳同步订阅 ----
        # ApproximateTimeSynchronizer 根据时间戳将左目和深度图像
        # 对齐到同一时刻，容忍微小的硬件同步误差
        self._left_sub: Subscriber = Subscriber(
            self, Image, TOPIC_CAMERA_LEFT, qos_profile=qos,
        )
        self._depth_sub: Subscriber = Subscriber(
            self, Image, TOPIC_CAMERA_DEPTH, qos_profile=qos,
        )

        # queue_size=10: 内部缓冲队列长度
        # slop=0.1: 允许 100ms 的时间戳误差，适应相机的同步精度
        self._sync: ApproximateTimeSynchronizer = ApproximateTimeSynchronizer(
            [self._left_sub, self._depth_sub],
            queue_size=10,
            slop=0.1,
        )
        self._sync.registerCallback(self._on_synced_image)

        self.get_logger().info("VisionProcessor 已启动")

    # ---- 同步回调 ----

    def _on_synced_image(
        self,
        left_msg: Image,
        depth_msg: Image,
    ) -> None:
        """左目 + 深度图像同步回调

        两条消息的时间戳已由 ApproximateTimeSynchronizer 对齐。

        Args:
            left_msg: 左目 RGB 图像消息
            depth_msg: 深度图像消息（16UC1 或 32FC1）
        """
        try:
            left_image: npt.NDArray[np.uint8] = self._bridge.imgmsg_to_cv2(
                left_msg, "bgr8"
            )
            depth_image: npt.NDArray = self._bridge.imgmsg_to_cv2(
                depth_msg, "passthrough"
            )
        except Exception as e:
            self.get_logger().error(f"图像转换失败: {e}")
            return

        result: dict = self.process_image(left_image, depth_image)

        with self._result_lock:
            self._latest_result = result

    # ---- 目标检测（桩代码，子类可重写）----

    def process_image(
        self,
        rgb_image: npt.NDArray[np.uint8],
        depth_image: npt.NDArray,
    ) -> dict:
        """图像处理：目标检测

        桩代码实现，返回空结果。
        重写此方法以接入 YOLO / GraspNet / 6D 姿态估计网络。

        接入示例：
            class YOLOVisionProcessor(VisionProcessor):
                def __init__(self):
                    super().__init__()
                    self._model = YOLO("yolov8n.pt")

                def process_image(self, rgb, depth):
                    results = self._model(rgb)
                    # 在 rgb 上做 2D 检测，用 depth 取对应像素深度值
                    # 结合像素坐标和深度计算 3D 坐标
                    return {"detected": True, "xyz": [x, y, z], ...}

        Args:
            rgb_image: 左目 BGR 图像 (H, W, 3), dtype=uint8
            depth_image: 深度图像 (H, W)，dtype 取决于相机驱动
                         常见为 uint16（毫米）或 float32（米）

        Returns:
            检测结果字典，格式：
            {
                "detected": bool,          # 是否检测到目标
                "xyz": [x, y, z] | None,   # 相机坐标系下的 3D 位置（米）
                "uv": [u, v] | None,        # 图像像素坐标
                "confidence": float,        # 检测置信度 [0, 1]
                "grasp_pose": dict | None,  # 抓取位姿（供后续扩展）
            }
        """
        # --- 桩代码：此处接入你的检测网络 ---
        # 示例：YOLO 目标检测
        #   results = self._yolo_model(rgb_image)
        # 示例：GraspNet 6D 抓取位姿估计
        #   grasp_poses = self._graspnet_model(rgb_image, depth_image)

        return {
            "detected": False,
            "xyz": None,
            "uv": None,
            "confidence": 0.0,
            "grasp_pose": None,
        }

    # ---- 结果获取 ----

    def get_latest_result(self) -> Optional[dict]:
        """获取最新的检测结果（线程安全）

        Returns:
            检测结果字典，尚未收到图像时返回 None
        """
        with self._result_lock:
            return self._latest_result

    def wait_for_detection(self, timeout: float = 10.0) -> Optional[dict]:
        """阻塞等待直到检测到目标或超时

        Args:
            timeout: 超时时间（秒）

        Returns:
            检测结果字典，超时返回 None
        """
        deadline = self.get_clock().now().nanoseconds + int(timeout * 1e9)
        while self.get_clock().now().nanoseconds < deadline:
            result = self.get_latest_result()
            if result is not None and result.get("detected", False):
                return result
            time.sleep(0.1)
        self.get_logger().warn(f"等待检测超时 ({timeout}s)")
        return None

    # ---- 工具方法 ----

    @staticmethod
    def pixel_to_3d(
        u: int,
        v: int,
        depth: float,
        fx: float,
        fy: float,
        cx: float,
        cy: float,
    ) -> list[float]:
        """将像素坐标 + 深度值转换为相机坐标系下的 3D 点

        针孔相机模型反投影：
            X = (u - cx) * Z / fx
            Y = (v - cy) * Z / fy
            Z = depth

        Args:
            u, v: 像素坐标
            depth: 深度值（米）
            fx, fy, cx, cy: 相机内参

        Returns:
            [x, y, z] 在相机坐标系下的 3D 位置（米）
        """
        x = (u - cx) * depth / fx
        y = (v - cy) * depth / fy
        z = depth
        return [x, y, z]
