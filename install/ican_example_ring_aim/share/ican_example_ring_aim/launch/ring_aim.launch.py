from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("publish_debug_image", default_value="true"),
            Node(
                package="ican_example_ring_aim",
                executable="ring_aim_node",
                name="ring_aim_node",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "publish_debug_image": LaunchConfiguration("publish_debug_image"),
                    }
                ],
            ),
        ]
    )
