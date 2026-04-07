"""Python bindings for the robot_control_cpp C++ library.

ALL ROS2 communication uses rclcpp internally.
Do NOT call rclpy.init() -- use rclcpp_init() instead.
"""

from robot_control_cpp_py._core import (
    # ROS2 lifecycle
    rclcpp_init,
    rclcpp_shutdown,
    Logger,
    MultiThreadedExecutor,
    # Data types
    TopicConfig,
    RobotProfile,
    GripperProfile,
    TcpConfig,
    DetectionResult,
    GraspState,
    # Layer 1: Core
    IKSolver,
    CameraInterface,
    ColorDetector,
    pixel_to_3d,
    IRobotController,
    RobotMotionController,
    IVisionProcessor,
    GraspTaskManager,
    # Layer 2: ROS2 nodes
    RobotControllerNode,
    VisionProcessorNode,
    # Profiles
    profiles,
    # Constants
    JOINT_TOLERANCE,
    FINGER_TOLERANCE,
    MOTION_TIMEOUT,
    POLL_INTERVAL,
    SETTLE_TIME,
    DEFAULT_STEPS,
    DEFAULT_STEP_TIME,
    IMAGE_SYNC_QUEUE_SIZE,
    IMAGE_SYNC_SLOP,
    FINGER_STABLE_COUNT,
    FINGER_STABLE_TOL,
    READY_TIMEOUT,
)

__all__ = [
    "rclcpp_init",
    "rclcpp_shutdown",
    "Logger",
    "MultiThreadedExecutor",
    "TopicConfig",
    "RobotProfile",
    "GripperProfile",
    "TcpConfig",
    "DetectionResult",
    "GraspState",
    "IKSolver",
    "CameraInterface",
    "ColorDetector",
    "pixel_to_3d",
    "IRobotController",
    "RobotMotionController",
    "IVisionProcessor",
    "GraspTaskManager",
    "RobotControllerNode",
    "VisionProcessorNode",
    "profiles",
]
