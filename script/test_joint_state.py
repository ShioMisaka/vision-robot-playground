"""测试：读取 Isaac Sim 发布的关节角信息"""

import rclpy
from sensor_msgs.msg import JointState


def main():
    rclpy.init()
    node = rclpy.create_node("test_joint_state")

    received = False

    def on_joint_states(msg: JointState):
        nonlocal received
        received = True
        print("关节名:", list(msg.name))
        print("关节角 (rad):", [round(v, 4) for v in msg.position])
        print("速度:", [round(v, 4) for v in msg.velocity])
        print("力矩:", [round(v, 4) for v in msg.effort])

    node.create_subscription(JointState, "/joint_states", on_joint_states, 10)

    print("等待 /joint_states 消息...")
    try:
        while not received:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass

    if not received:
        print("超时：未收到任何消息，请检查 Isaac Sim 是否在发布 /joint_states")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
