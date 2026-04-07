"""IK/FK 求解器：从 URDF 加载机器人模型，计算逆运动学和正运动学"""

from __future__ import annotations

from typing import Optional

import ikpy.chain
import numpy as np
from scipy.spatial.transform import Rotation

from src.robot_control_py.config import URDF_PATH, BASE_LINK, ACTIVE_JOINTS, IK_DEFAULT_GUESS


class IKSolver:
    """Franka Panda 逆运动学求解器"""

    def __init__(self, urdf_path: str = URDF_PATH):
        active_names = set(ACTIVE_JOINTS)
        temp = ikpy.chain.Chain.from_urdf_file(urdf_path, base_elements=[BASE_LINK])
        mask = [link.name in active_names for link in temp.links]
        self.chain: ikpy.chain.Chain = ikpy.chain.Chain.from_urdf_file(
            urdf_path, base_elements=[BASE_LINK], active_links_mask=mask
        )
        self.mask: list[bool] = mask
        self._last_result: list[float] | None = None

    def _make_initial(self) -> list[float]:
        """生成默认初始猜测"""
        angles: list[float] = [0.0] * len(self.chain.links)
        for i, link in enumerate(self.chain.links):
            if link.name in IK_DEFAULT_GUESS:
                angles[i] = IK_DEFAULT_GUESS[link.name]
        return angles

    def _extract(self, full_result: list[float]) -> list[float]:
        """从完整 IK 结果中提取 7 个 active 关节角度"""
        return [full_result[i] for i, active in enumerate(self.mask) if active]

    def solve(
        self,
        target_xyz: list[float] | np.ndarray,
        target_rpy: Optional[list[float] | np.ndarray] = None,
    ) -> list[float]:
        """
        求解逆运动学

        Args:
            target_xyz: [x, y, z] 目标位置（米）
            target_rpy: [roll, pitch, yaw] 目标姿态（弧度），None 则只约束位置

        Returns:
            7 个关节角度（弧度）
        """
        initial: list[float] = (
            self._last_result if self._last_result is not None else self._make_initial()
        )

        if target_rpy is not None:
            target_matrix = np.eye(4)
            target_matrix[:3, 3] = target_xyz
            target_matrix[:3, :3] = Rotation.from_euler("xyz", target_rpy).as_matrix()
            result = self.chain.inverse_kinematics_frame(
                target=target_matrix, initial_position=initial, orientation_mode="all"
            )
        else:
            result = self.chain.inverse_kinematics(
                target_position=target_xyz, initial_position=initial
            )

        self._last_result = result
        return self._extract(result)

    def forward(self, joint_angles: list[float]) -> dict[str, object]:
        """
        正运动学：从关节角度算末端位姿

        Args:
            joint_angles: 7 个关节角度（弧度）

        Returns:
            {"pos": [x,y,z], "rpy": [r,p,y], "matrix": 4x4 ndarray}
        """
        full: list[float] = [0.0] * len(self.chain.links)
        j: int = 0
        for i, active in enumerate(self.mask):
            if active:
                full[i] = joint_angles[j]
                j += 1
        matrix: np.ndarray = self.chain.forward_kinematics(full)
        pos: list[float] = matrix[:3, 3].tolist()
        rpy: list[float] = Rotation.from_matrix(matrix[:3, :3]).as_euler("xyz").tolist()
        return {"pos": pos, "rpy": rpy, "matrix": matrix}
