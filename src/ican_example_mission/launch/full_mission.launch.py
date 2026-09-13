import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def include_launch(package_name, relative_path):
    share = get_package_share_directory(package_name)
    source = PythonLaunchDescriptionSource(os.path.join(share, relative_path))
    return IncludeLaunchDescription(source)


def ask_target_id(label, default_value, valid_ids):
    prompt = f"{label} {sorted(valid_ids)} [默认 {default_value}]: "
    while True:
        try:
            value = input(prompt).strip()
        except EOFError:
            return default_value
        if not value:
            return default_value
        if value.isdigit() and int(value) in valid_ids:
            return int(value)
        print(f"目标 id 必须是以下值之一：{sorted(valid_ids)}")


def as_bool(value):
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def launch_setup(context, *args, **kwargs):
    target_2_id = int(LaunchConfiguration("target_2_id").perform(context))
    target_3_id = int(LaunchConfiguration("target_3_id").perform(context))
    target_4_id = int(LaunchConfiguration("target_4_id").perform(context))

    if as_bool(LaunchConfiguration("prompt_target_config").perform(context)):
        print("\n请输入本轮指定靶编号。")
        print("1 号点固定为环形靶；机器人将依次完成四个点并返回指挥中心。")
        target_2_id = ask_target_id("2 号旋转靶目标 id", target_2_id, set(range(1, 6)))
        target_3_id = ask_target_id("3 号移动靶目标 id", target_3_id, set(range(6, 9)))
        target_4_id = ask_target_id("4 号移动靶目标 id", target_4_id, set(range(6, 9)))
        print("配置完成，完整任务即将启动。")

    parameters = {
        "use_sim_time": True,
        "start_delay_sec": LaunchConfiguration("start_delay_sec"),
        "nav_timeout_sec": LaunchConfiguration("nav_timeout_sec"),
        "nav_goal_retries": LaunchConfiguration("nav_goal_retries"),
        "initial_nav_goal_retries": LaunchConfiguration("initial_nav_goal_retries"),
        "nav_retry_delay_sec": LaunchConfiguration("nav_retry_delay_sec"),
        "nav_failure_settle_sec": LaunchConfiguration("nav_failure_settle_sec"),
        "nav_position_fallback_tolerance_m": LaunchConfiguration(
            "nav_position_fallback_tolerance_m"
        ),
        "task_3_right_offset_m": LaunchConfiguration("task_3_right_offset_m"),
        "task_4_right_offset_m": LaunchConfiguration("task_4_right_offset_m"),
        "aim_timeout_sec": LaunchConfiguration("aim_timeout_sec"),
        "ring_aim_mode": LaunchConfiguration("ring_aim_mode"),
        "wheel_stable_cycles": LaunchConfiguration("wheel_stable_cycles"),
        "shots_per_target": LaunchConfiguration("shots_per_target"),
        "fire_without_aim": LaunchConfiguration("fire_without_aim"),
        "target_2_id": target_2_id,
        "target_3_id": target_3_id,
        "target_4_id": target_4_id,
        "publish_referee_targets": LaunchConfiguration("publish_referee_targets"),
        "referee_target_plan_topic": LaunchConfiguration("referee_target_plan_topic"),
        "referee_current_target_topic": LaunchConfiguration("referee_current_target_topic"),
    }
    return [
        TimerAction(
            period=0.0,
            actions=[include_launch("ican_referee_system", "launch/referee.launch.py")],
        ),
        TimerAction(
            period=0.0,
            actions=[include_launch("ican_example_ring_aim", "launch/ring_aim.launch.py")],
        ),
        TimerAction(
            period=1.0,
            actions=[
                Node(
                    package="ican_example_mission",
                    executable="full_mission_node",
                    name="full_mission_node",
                    output="screen",
                    parameters=[parameters],
                )
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("start_delay_sec", default_value="2.0"),
            DeclareLaunchArgument("nav_timeout_sec", default_value="90.0"),
            DeclareLaunchArgument("nav_goal_retries", default_value="2"),
            DeclareLaunchArgument("initial_nav_goal_retries", default_value="3"),
            DeclareLaunchArgument("nav_retry_delay_sec", default_value="3.0"),
            DeclareLaunchArgument("nav_failure_settle_sec", default_value="5.0"),
            DeclareLaunchArgument(
                "nav_position_fallback_tolerance_m", default_value="0.12"
            ),
            DeclareLaunchArgument("task_3_right_offset_m", default_value="0.02"),
            DeclareLaunchArgument("task_4_right_offset_m", default_value="0.04"),
            DeclareLaunchArgument("aim_timeout_sec", default_value="15.0"),
            DeclareLaunchArgument("ring_aim_mode", default_value="model"),
            DeclareLaunchArgument("wheel_stable_cycles", default_value="2"),
            DeclareLaunchArgument("shots_per_target", default_value="1"),
            DeclareLaunchArgument("fire_without_aim", default_value="true"),
            DeclareLaunchArgument("target_2_id", default_value="1"),
            DeclareLaunchArgument("target_3_id", default_value="6"),
            DeclareLaunchArgument("target_4_id", default_value="8"),
            DeclareLaunchArgument("prompt_target_config", default_value="true"),
            DeclareLaunchArgument("publish_referee_targets", default_value="true"),
            DeclareLaunchArgument(
                "referee_target_plan_topic", default_value="/referee/target_plan"
            ),
            DeclareLaunchArgument(
                "referee_current_target_topic", default_value="/referee/current_target"
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
