"""测试：相机 TF 链验证 — C++ 核心库后端

验证内容：
    1. TF 查询：panda_link0 → panda_hand → camera_link → camera_color_optical_frame
    2. TF 一致性：机械臂运动后 hand → camera_link 偏移保持不变
    3. 坐标转换：transform_to_base 将相机坐标系下的点转到 base 坐标系

前置条件：
    - Isaac Sim 已启动并发布 /joint_states
    - Isaac Sim 中 ZED_X_Mini 已取消 RigidBody 属性

用法：
    source install/setup.zsh
    python3 script/test_camera_tf.py
"""

import math
import threading

from robot_control_cpp_py import (
    rclcpp_init,
    rclcpp_shutdown,
    RobotControllerNode,
    MultiThreadedExecutor,
    TopicConfig,
    CameraExtrinsics,
    GraspTaskManager,
    ColorDetector,
    VisionProcessorNode,
    profiles,
)


def test_tf_lookup(ctrl, logger) -> bool:
    """测试 1：TF 查询是否正常"""
    logger.info("=== 测试 1: TF 查询 ===")
    passed = True

    # 查 hand → camera_link
    result = ctrl.lookup_transform("panda_hand", "camera_link", timeout=2.0)
    if result is None:
        logger.error("  [FAIL] panda_hand → camera_link 查询失败")
        passed = False
    else:
        logger.info(
            f"  hand → camera_link: "
            f"xyz=[{result[0]:.4f}, {result[1]:.4f}, {result[2]:.4f}]  "
            f"rpy=[{math.degrees(result[3]):.1f}, {math.degrees(result[4]):.1f}, {math.degrees(result[5]):.1f}]"
        )

    # 查 hand → camera_color_optical_frame
    result = ctrl.lookup_transform("panda_hand", "camera_color_optical_frame", timeout=2.0)
    if result is None:
        logger.error("  [FAIL] panda_hand → camera_color_optical_frame 查询失败")
        passed = False
    else:
        logger.info(
            f"  hand → camera_optical: "
            f"xyz=[{result[0]:.4f}, {result[1]:.4f}, {result[2]:.4f}]  "
            f"rpy=[{math.degrees(result[3]):.1f}, {math.degrees(result[4]):.1f}, {math.degrees(result[5]):.1f}]"
        )

    # 查 panda_link0 → camera_color_optical_frame（完整链）
    result = ctrl.lookup_transform("panda_link0", "camera_color_optical_frame", timeout=2.0)
    if result is None:
        logger.error("  [FAIL] panda_link0 → camera_color_optical_frame 查询失败")
        passed = False
    else:
        logger.info(
            f"  base → camera_optical: "
            f"xyz=[{result[0]:.4f}, {result[1]:.4f}, {result[2]:.4f}]  "
            f"rpy=[{math.degrees(result[3]):.1f}, {math.degrees(result[4]):.1f}, {math.degrees(result[5]):.1f}]"
        )

    if passed:
        logger.info("  [PASS]")
    return passed


def test_tf_consistency(ctrl, logger) -> bool:
    """测试 2：机械臂运动后相机 TF 偏移是否保持一致（固定连杆）"""
    logger.info("=== 测试 2: 运动 TF 一致性 ===")

    # 记录初始 hand → camera_link
    before = ctrl.lookup_transform("panda_hand", "camera_link", timeout=2.0)
    if before is None:
        logger.error("  [FAIL] 初始 TF 查询失败")
        return False

    # 获取当前关节角，小幅度旋转 joint1（避免奇异位）
    joints = ctrl.get_joint_angles()
    delta = math.radians(10)
    ctrl.rotate_joint(1, delta)

    # 记录运动后的 hand → camera_link
    after = ctrl.lookup_transform("panda_hand", "camera_link", timeout=2.0)

    # 恢复
    ctrl.rotate_joint(1, -delta)

    if after is None:
        logger.error("  [FAIL] 运动 TF 查询失败")
        return False

    # 比较：固定连杆的偏移不应改变
    tol = 1e-4
    xyz_ok = all(abs(before[i] - after[i]) < tol for i in range(3))
    rpy_ok = all(abs(before[i] - after[i]) < tol for i in range(3, 6))

    if xyz_ok and rpy_ok:
        logger.info("  hand → camera_link 运动前后一致")
        logger.info("  [PASS]")
        return True
    else:
        logger.error(f"  [FAIL] 偏移不一致! before={before}, after={after}")
        return False


def test_transform_to_base(ctrl, logger) -> bool:
    """测试 3：验证 transform_to_base 能正确转换坐标"""
    logger.info("=== 测试 3: transform_to_base 坐标转换 ===")

    # 获取 base → camera TF
    tf = ctrl.lookup_transform("panda_link0", "camera_color_optical_frame", timeout=2.0)
    if tf is None:
        logger.error("  [FAIL] 无法获取 base → camera TF")
        return False

    # 构造一个在相机坐标系下已知方向的点
    # 光学坐标系 Z 轴朝前（朝下），[0, 0, 0.1] = 相机正下方 0.1m
    camera_point = [0.0, 0.0, 0.1]

    # 直接用旋转矩阵计算期望结果
    roll_a = tf[3]
    pitch_a = tf[4]
    yaw_a = tf[5]
    # R = Rz(yaw) * Ry(pitch) * Rx(roll)
    cr, sr = math.cos(roll_a), math.sin(roll_a)
    cp, sp = math.cos(pitch_a), math.sin(pitch_a)
    cy, sy = math.cos(yaw_a), math.sin(yaw_a)
    R = [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]
    # base_point = R * camera_point + t
    expected = [
        R[0][2] * 0.1 + tf[0],
        R[1][2] * 0.1 + tf[1],
        R[2][2] * 0.1 + tf[2],
    ]

    # 通过 GraspTaskManager 验证
    detector = ColorDetector([0, 0, 0], [0, 0, 0])
    vision = VisionProcessorNode.create(detector, TopicConfig())
    manager = GraspTaskManager(
        ctrl, vision,
        approach_height=0.15,
        grasp_height_offset=0.02,
        grasp_rpy=[math.pi, 0.0, math.pi],
    )
    actual = manager.transform_to_base(camera_point)
    if actual is None:
        logger.error("  [FAIL] transform_to_base 返回 None")
        return False

    logger.info(f"  相机 base 位置:    [{tf[0]:.4f}, {tf[1]:.4f}, {tf[2]:.4f}]")
    logger.info(f"  期望 base 点:      [{expected[0]:.4f}, {expected[1]:.4f}, {expected[2]:.4f}]")
    logger.info(f"  transform_to_base: [{actual[0]:.4f}, {actual[1]:.4f}, {actual[2]:.4f}]")

    tol = 1e-3
    match = all(abs(actual[i] - expected[i]) < tol for i in range(3))
    if match:
        logger.info("  [PASS]")
        return True
    else:
        logger.error("  [FAIL] 结果不一致")
        return False


def main() -> None:
    rclcpp_init()

    profile = profiles.panda()
    gripper = profiles.panda_gripper()
    topics = TopicConfig()

    # 配置相机外参（与 URDF 一致）
    topics.camera_extrinsics = CameraExtrinsics()
    topics.camera_extrinsics.xyz = [0.015, 0.0, 0.03]
    topics.camera_extrinsics.rpy = [0.0, math.pi / 2, math.pi]

    robot = RobotControllerNode.create(profile, gripper, topics)

    executor = MultiThreadedExecutor()
    executor.add_node(robot)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    logger = robot.get_logger()
    logger.info("等待与 Isaac Sim 建立连接...")
    robot.wait_for_ready()
    logger.info("连接成功!")

    ctrl = robot.get_controller()

    # 运行测试（不调用 go_home，避免超时和奇异点）
    results = {}
    results["TF 查询"] = test_tf_lookup(ctrl, logger)
    results["TF 一致性"] = test_tf_consistency(ctrl, logger)
    results["坐标转换"] = test_transform_to_base(ctrl, logger)

    # 汇总
    logger.info("=" * 40)
    passed = sum(1 for v in results.values() if v)
    total = len(results)
    for name, ok in results.items():
        status = "PASS" if ok else "FAIL"
        logger.info(f"  {name}: {status}")
    logger.info(f"结果: {passed}/{total} 通过")

    executor.cancel()
    spin_thread.join()
    rclcpp_shutdown()


if __name__ == "__main__":
    main()
