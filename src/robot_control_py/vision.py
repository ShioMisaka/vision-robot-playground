"""视觉处理：颜色检测、目标定位"""

from __future__ import annotations

from typing import Optional

import cv2
import numpy as np
import numpy.typing as npt


class ColorDetector:
    """基于 HSV 颜色范围的目标检测器"""

    def __init__(self, lower_hsv: list[int], upper_hsv: list[int]):
        self.lower: np.ndarray = np.array(lower_hsv, dtype=np.uint8)
        self.upper: np.ndarray = np.array(upper_hsv, dtype=np.uint8)

    def detect(self, cv_image: npt.NDArray[np.uint8]) -> Optional[tuple[int, int]]:
        """
        在图像中检测目标

        Args:
            cv_image: BGR 格式的 OpenCV 图像

        Returns:
            (cx, cy) 目标中心坐标，未检测到返回 None
        """
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower, self.upper)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return None

        largest = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest)
        if M["m00"] == 0:
            return None
        return int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])

    def draw_target(
        self,
        cv_image: npt.NDArray[np.uint8],
        cx: int,
        cy: int,
        label: str = "Target",
    ):
        """
        在图像上绘制十字准星和坐标标签

        Args:
            cv_image: BGR 格式的 OpenCV 图像（原地修改）
            cx: 目标中心 x 坐标
            cy: 目标中心 y 坐标
            label: 标签文字
        """
        cv2.circle(cv_image, (cx, cy), 15, (0, 255, 0), 2)
        cv2.line(cv_image, (cx - 20, cy), (cx + 20, cy), (0, 255, 0), 2)
        cv2.line(cv_image, (cx, cy - 20), (cx, cy + 20), (0, 255, 0), 2)
        cv2.putText(
            cv_image, f"{label}: ({cx}, {cy})",
            (cx - 60, cy - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2,
        )


# 预设检测器
RED_DETECTOR: ColorDetector = ColorDetector(
    lower_hsv=[0, 100, 100], upper_hsv=[10, 255, 255]
)
