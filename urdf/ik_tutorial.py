"""
============================================================
  URDF + 逆运动学 (IK) 完整教程
  Franka Panda 机械臂：从 A 位置移动到 B 位置
============================================================

前置知识：
  URDF = 用 XML 描述机器人的物理结构（连杆 link + 关节 joint）
  FK   = 正运动学：已知关节角度 → 算出末端位置
  IK   = 逆运动学：已知目标位置 → 算出需要的关节角度

依赖安装：pip install ikpy numpy
"""

import ikpy.chain
import numpy as np
import math


def load_robot(urdf_path: str) -> ikpy.chain.Chain:
    """
    第一步：从 URDF 文件加载机器人运动链

    ikpy 会解析 URDF 中的 <link> 和 <joint> 标签，
    构建出一棵"关节树"，我们指定从 panda_link0 开始。
    """
    # 先加载一次，看完整的链结构
    full_chain = ikpy.chain.Chain.from_urdf_file(
        urdf_path, base_elements=["panda_link0"]
    )
    print("=" * 60)
    print("【机器人运动链结构】")
    print(f"{'索引':<4} {'名称':<30} {'类型':<10} {'是否可控'}")
    print("-" * 60)
    for i, link in enumerate(full_chain.links):
        is_revolute = link.joint_type == "revolute"
        type_cn = {"revolute": "旋转关节", "fixed": "固定关节", "prismatic": "平移关节"}.get(link.joint_type, link.joint_type)
        print(f"{i:<4} {link.name:<30} {type_cn:<10} {'Yes' if is_revolute else 'No'}")
    print("-" * 60)

    # 生成 active_links_mask：
    #   True  = 这个关节参与 IK 求解（就是我们能控制的）
    #   False = 固定关节，不参与计算
    active_joint_names = {f"panda_joint{i}" for i in range(1, 8)}
    mask = [
        link.name in active_joint_names
        for link in full_chain.links
    ]

    # 带掩码重新加载（这样 IK 只会调整 7 个手臂关节）
    chain = ikpy.chain.Chain.from_urdf_file(
        urdf_path,
        base_elements=["panda_link0"],
        active_links_mask=mask
    )
    return chain


def forward_kinematics_demo(chain: ikpy.chain.Chain):
    """
    第二步：正运动学 (FK) 演示

    FK 回答：如果所有关节都是 0，末端在哪？
    """
    print("\n" + "=" * 60)
    print("【正运动学 FK 演示】")
    print("问题：所有关节角度 = 0 时，末端执行器在哪里？")

    # 全零角度（ikpy 的数组长度 = 链中所有 link 数量，但只有 active 的才有效）
    zero_angles = [0.0] * len(chain.links)

    # fk_matrix 是一个 4x4 齐次变换矩阵
    fk_matrix = chain.forward_kinematics(zero_angles)

    # 末端位置 = 矩阵的最后一列的前三个元素
    end_pos = fk_matrix[:3, 3]

    print(f"末端位置: X={end_pos[0]:.3f}m, Y={end_pos[1]:.3f}m, Z={end_pos[2]:.3f}m")
    print(f"（约 {end_pos[2]:.3f}m = {end_pos[2]*100:.1f}cm，就是 Panda 伸直后手指尖的高度）")
    return fk_matrix


def inverse_kinematics_demo(chain: ikpy.chain.Chain, target: list):
    """
    第三步：逆运动学 (IK) 演示

    IK 回答：要让末端到达 target 位置，每个关节要转到多少度？

    原理（数值迭代法）：
      1. 从一个初始关节角度开始
      2. 用 FK 计算当前末端位置
      3. 计算误差 = 目标 - 当前位置
      4. 用雅可比矩阵计算角度调整量
      5. 更新角度，重复直到误差足够小
    """
    print(f"\n{'='*60}")
    print(f"【逆运动学 IK 演示】")
    print(f"目标位置: X={target[0]}m, Y={target[1]}m, Z={target[2]}m")

    # ---- 初始猜测 ----
    # IK 是迭代求解，初始值很关键！
    # 全0 = 手臂完全伸直 = 奇异构型，IK 算不出来
    # 所以我们给一个"弯着肘"的初始姿态
    initial_guess = [0.0] * len(chain.links)
    for i, link in enumerate(chain.links):
        if link.name == "panda_joint4":
            initial_guess[i] = -1.5   # 肘部弯曲
        elif link.name == "panda_joint6":
            initial_guess[i] = 1.5    # 手腕翻转

    # ---- 调用 IK 求解器 ----
    ik_result = chain.inverse_kinematics(
        target_position=target,
        initial_position=initial_guess
    )

    # ---- 提取 7 个可控关节的角度 ----
    mask = chain.active_links_mask
    joint_angles_rad = [ik_result[i] for i, active in enumerate(mask) if active]
    joint_angles_deg = [math.degrees(rad) for rad in joint_angles_rad]

    joint_names = [f"panda_joint{i}" for i in range(1, 8)]
    print(f"\n{'关节名':<18} {'弧度 (rad)':<14} {'角度 (deg)':<14}")
    print("-" * 46)
    for name, rad, deg in zip(joint_names, joint_angles_rad, joint_angles_deg):
        print(f"{name:<18} {rad:<14.4f} {deg:<14.2f}")

    # ---- 用 FK 验证：用算出的角度，末端真的到目标了吗？ ----
    verify_matrix = chain.forward_kinematics(ik_result)
    actual_pos = verify_matrix[:3, 3]
    error = np.linalg.norm(np.array(target) - actual_pos)

    print(f"\n验证结果:")
    print(f"  实际末端位置: ({actual_pos[0]:.4f}, {actual_pos[1]:.4f}, {actual_pos[2]:.4f})")
    print(f"  误差: {error:.6f} m ({error*1000:.2f} mm)")

    return ik_result


def ab_movement_demo(chain: ikpy.chain.Chain, pos_a: list, pos_b: list):
    """
    第四步：从 A 移动到 B 的完整演示

    实际应用场景：
      视觉识别到物体在 A 点 → IK 算出角度 → 机械臂移到 A
      抓取后要放到 B 点 → IK 算出角度 → 机械臂移到 B
    """
    print(f"\n{'='*60}")
    print(f"【A → B 移动演示】")
    print(f"  A 点（抓取位置）: ({pos_a[0]}, {pos_a[1]}, {pos_a[2]})")
    print(f"  B 点（放置位置）: ({pos_b[0]}, {pos_b[1]}, {pos_b[2]})")

    # 初始猜测
    initial_guess = [0.0] * len(chain.links)
    for i, link in enumerate(chain.links):
        if link.name == "panda_joint4":
            initial_guess[i] = -1.5
        elif link.name == "panda_joint6":
            initial_guess[i] = 1.5

    # 求 A 点的 IK
    ik_a = chain.inverse_kinematics(target_position=pos_a, initial_position=initial_guess)
    mask = chain.active_links_mask
    angles_a = [ik_a[i] for i, active in enumerate(mask) if active]
    angles_a_deg = [round(math.degrees(rad), 2) for rad in angles_a]

    # 求 B 点的 IK（用 A 点结果作为初始猜测，更稳定）
    ik_b = chain.inverse_kinematics(target_position=pos_b, initial_position=ik_a)
    angles_b = [ik_b[i] for i, active in enumerate(mask) if active]
    angles_b_deg = [round(math.degrees(rad), 2) for rad in angles_b]

    # 计算每个关节需要变化多少
    joint_names = [f"joint{i}" for i in range(1, 8)]
    delta_deg = [round(b - a, 2) for a, b in zip(angles_a_deg, angles_b_deg)]

    print(f"\n{'关节':<10} {'A点角度(deg)':<14} {'B点角度(deg)':<14} {'变化量(deg)':<14}")
    print("-" * 52)
    for name, a, b, d in zip(joint_names, angles_a_deg, angles_b_deg, delta_deg):
        print(f"{name:<10} {a:<14} {b:<14} {d:>+14}")

    total_change = sum(abs(d) for d in delta_deg)
    print(f"\n总变化量（所有关节绝对变化之和）: {total_change:.2f} deg")

    # ---- 输出可以直接复制到 robot_brain.py 使用的格式 ----
    print(f"\n{'='*60}")
    print("可以直接在 robot_brain.py 中使用的关节角度：")
    print(f"  pose_a = {angles_a} + [0.04, 0.04]  # 抓取位置 + 夹爪张开")
    print(f"  pose_b = {angles_b} + [0.04, 0.04]  # 放置位置 + 夹爪张开")
    print(f"  pose_b_close = {angles_b} + [0.0, 0.0]  # 放置位置 + 夹爪闭合")

    return angles_a, angles_b


def main():
    urdf_path = "urdf/panda.urdf"

    print("╔══════════════════════════════════════════════════════════╗")
    print("║   Franka Panda 逆运动学 (IK) 教程                       ║")
    print("║   学会如何从 URDF 读取机器人结构，并用 IK 求解关节角度     ║")
    print("╚══════════════════════════════════════════════════════════╝")

    # 第一步：加载机器人
    chain = load_robot(urdf_path)

    # 第二步：FK 演示 —— 了解机器人"伸直"时多高
    forward_kinematics_demo(chain)

    # 第三步：IK 演示 —— 给一个目标位置，算出关节角度
    inverse_kinematics_demo(chain, target=[0.4, 0.1, 0.2])

    # 第四步：A→B 移动演示 —— 这是抓取任务的核心
    ab_movement_demo(
        chain,
        pos_a=[0.4, 0.2, 0.25],   # A: 桌面上的物体位置
        pos_b=[0.3, -0.3, 0.35],   # B: 放置位置
    )


if __name__ == "__main__":
    main()
