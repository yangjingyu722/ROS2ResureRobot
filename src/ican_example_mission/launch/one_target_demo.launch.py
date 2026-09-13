from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def include_launch(package_name, relative_path, launch_arguments=None):
    share = get_package_share_directory(package_name)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, relative_path)),
        launch_arguments=(launch_arguments or {}).items(),
    )


def ask_target_id(label, default_value):
    prompt = f"{label} [默认 {default_value}]: "
    while True:
        try:
            value = input(prompt).strip()
        except EOFError:
            return default_value
        if not value:
            return default_value
        if value.isdigit():
            return int(value)
        print(f"目标 id 必须是非负整数：{value}")


def as_bool(value):
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def launch_setup(context, *args, **kwargs):
    start_delay_sec = LaunchConfiguration("start_delay_sec")
    publish_referee_targets = LaunchConfiguration("publish_referee_targets")
    referee_target_plan_topic = LaunchConfiguration("referee_target_plan_topic")
    referee_current_target_topic = LaunchConfiguration("referee_current_target_topic")

    target_2_id = int(LaunchConfiguration("target_2_id").perform(context))
    target_3_id = int(LaunchConfiguration("target_3_id").perform(context))
    target_4_id = int(LaunchConfiguration("target_4_id").perform(context))

    if as_bool(LaunchConfiguration("prompt_target_config").perform(context)):
        print()
        print("请输入靶子配置。当前示例先执行 1 号点位，2/3/4 号配置会写入裁判目标计划，方便后续扩展。")
        print("1 号固定环形靶使用 ring 瞄准。")
        target_2_id = ask_target_id("2 号旋转靶目标 id", target_2_id)
        target_3_id = ask_target_id("3 号平移靶目标 id", target_3_id)
        target_4_id = ask_target_id("4 号平移靶目标 id", target_4_id)
        print("配置完成，示例流程即将启动。")

    return [
        TimerAction(
            period=0.0,
            actions=[include_launch("ican_example_ring_aim", "launch/ring_aim.launch.py")],
        ),
        TimerAction(
            period=1.0,
            actions=[
                Node(
                    package="ican_example_mission",
                    executable="one_target_mission_node",
                    name="one_target_mission_node",
                    output="screen",
                    parameters=[
                        {
                            "use_sim_time": True,
                            "start_delay_sec": start_delay_sec,
                            "target_2_id": target_2_id,
                            "target_3_id": target_3_id,
                            "target_4_id": target_4_id,
                            "publish_referee_targets": publish_referee_targets,
                            "referee_target_plan_topic": referee_target_plan_topic,
                            "referee_current_target_topic": referee_current_target_topic,
                        }
                    ],
                )
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("start_delay_sec", default_value="2.0"),
            DeclareLaunchArgument("target_2_id", default_value="1"),
            DeclareLaunchArgument("target_3_id", default_value="6"),
            DeclareLaunchArgument("target_4_id", default_value="8"),
            DeclareLaunchArgument("prompt_target_config", default_value="true"),
            DeclareLaunchArgument("publish_referee_targets", default_value="true"),
            DeclareLaunchArgument("referee_target_plan_topic", default_value="/referee/target_plan"),
            DeclareLaunchArgument("referee_current_target_topic", default_value="/referee/current_target"),
            OpaqueFunction(function=launch_setup),
        ]
    )
