"""启动 robot_controller_node + robot_hmi 示教器。

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
        Node(
            package='robot_hmi',
            executable='robot_hmi',
            name='robot_hmi',
            output='screen',
        ),
    ])
