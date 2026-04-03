"""项目常量：路径、关节名、话题名、预设姿态"""

# ---- 文件路径 ----
URDF_PATH = "urdf/panda.urdf"
BASE_LINK = "panda_link0"

# ---- 关节名 ----
ACTIVE_JOINTS = [f"panda_joint{i}" for i in range(1, 8)]
ALL_JOINTS = ACTIVE_JOINTS + ["panda_finger_joint1", "panda_finger_joint2"]

# ---- ROS2 话题 ----
TOPIC_JOINT_COMMAND = "/joint_command"
TOPIC_JOINT_STATES = "/joint_states"
TOPIC_CAMERA_IMAGE = "/camera/image_raw"

# ---- 预设姿态（9个值：7关节 + 2夹爪）----
SAFE_HOME = [0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.0, 0.4, 0.4]

# ---- IK 默认初始猜测（弯肘翻腕，避免奇异点）----
IK_DEFAULT_GUESS = {
    "panda_joint4": -1.57,
    "panda_joint6": 1.57,
}
