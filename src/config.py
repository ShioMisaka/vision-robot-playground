"""项目常量：路径、关节名、话题名、TF Frame、预设姿态"""

from typing import Final

# ---- 文件路径 ----
URDF_PATH: Final[str] = "urdf/panda.urdf"
BASE_LINK: Final[str] = "panda_link0"

# ---- 关节名 ----
ACTIVE_JOINTS: Final[list[str]] = [f"panda_joint{i}" for i in range(1, 8)]
ALL_JOINTS: Final[list[str]] = ACTIVE_JOINTS + [
    "panda_finger_joint1", "panda_finger_joint2"
]

# ---- ROS2 话题 ----
TOPIC_JOINT_COMMAND: Final[str] = "/joint_command"
TOPIC_JOINT_STATES: Final[str] = "/joint_states"

# 相机话题（ZEN_X_Mini 左目 + 深度）
TOPIC_CAMERA_LEFT: Final[str] = "/camera/image_raw/left"
TOPIC_CAMERA_DEPTH: Final[str] = "/camera/image_raw/depth"

# ---- TF Frame 名称 ----
FRAME_BASE: Final[str] = "panda_link0"
FRAME_END_EFFECTOR: Final[str] = "panda_hand"
FRAME_CAMERA: Final[str] = "camera_color_optical_frame"

# ---- 预设姿态（9 个值：7 关节 + 2 夹爪）----
SAFE_HOME: Final[list[float]] = [0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.0, 0.4, 0.4]

# ---- IK 默认初始猜测（弯肘翻腕，避免奇异点）----
IK_DEFAULT_GUESS: Final[dict[str, float]] = {
    "panda_joint4": -1.57,
    "panda_joint6": 1.57,
}

# ---- 夹爪参数 ----
GRIPPER_MAX_WIDTH: Final[float] = 0.04
GRIPPER_MIN_WIDTH: Final[float] = 0.0
