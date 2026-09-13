import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    example_share = get_package_share_directory("ican_example_navigation")
    nav2_share = get_package_share_directory("nav2_bringup")

    default_map = os.path.join(example_share, "maps", "land_shoot_map.yaml")
    tuned_params = os.path.join(example_share, "config", "nav2_params.yaml")
    default_rviz = os.path.join(example_share, "rviz", "example_nav.rviz")
    nav2_launch = os.path.join(nav2_share, "launch", "bringup_launch.py")

    rviz = LaunchConfiguration("rviz")
    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    publish_initial_pose = LaunchConfiguration("publish_initial_pose")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="Start RViz (disabled by default to protect navigation timing on VMs).",
            ),
            DeclareLaunchArgument("map", default_value=default_map),
            DeclareLaunchArgument(
                "params_file",
                default_value=tuned_params,
                description="Tuned example Nav2 parameters for the iCAN practice map.",
            ),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz),
            DeclareLaunchArgument("publish_initial_pose", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch),
                launch_arguments={
                    "use_sim_time": "true",
                    "slam": "False",
                    "map": map_yaml,
                    "params_file": params_file,
                    "autostart": "true",
                    "use_composition": "False",
                }.items(),
            ),
            TimerAction(
                period=2.0,
                actions=[
                    Node(
                        package="ican_example_navigation",
                        executable="initial_pose_publisher",
                        output="screen",
                        condition=IfCondition(publish_initial_pose),
                        parameters=[{"use_sim_time": True}],
                    )
                ],
            ),
            TimerAction(
                period=3.0,
                actions=[
                    Node(
                        package="rviz2",
                        executable="rviz2",
                        name="rviz2_nav",
                        output="screen",
                        arguments=["-d", LaunchConfiguration("rviz_config")],
                        condition=IfCondition(rviz),
                        parameters=[{"use_sim_time": True}],
                    )
                ],
            ),
        ]
    )
