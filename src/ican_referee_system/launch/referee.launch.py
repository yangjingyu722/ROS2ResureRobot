from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="ican_referee_system",
                executable="referee_node",
                name="referee_node",
                output="screen",
                parameters=[{"use_sim_time": True}],
            )
        ]
    )
