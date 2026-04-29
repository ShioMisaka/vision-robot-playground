"""启动独立 robot_controller_node。

前提：Isaac Sim 已启动并发布 /joint_states。
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_controller',
            executable='robot_controller_node',
            name='robot_controller_node',
            output='screen',
        ),
    ])
