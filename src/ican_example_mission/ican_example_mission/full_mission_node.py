import json
import math
import time

from action_msgs.msg import GoalStatus
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import (
    PointStamped,
    PoseStamped,
    PoseWithCovarianceStamped,
    Quaternion,
    Twist,
)
from nav2_msgs.action import NavigateToPose
import rclpy

from ican_example_mission.instance_guard import InstanceGuard
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage


def quaternion_from_yaw(yaw):
    return Quaternion(z=math.sin(yaw * 0.5), w=math.cos(yaw * 0.5))


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def rotate_vector(q, vector):
    """Rotate a three-dimensional vector by a geometry_msgs Quaternion."""
    x, y, z = vector
    qx, qy, qz, qw = q.x, q.y, q.z, q.w
    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)
    return (
        x + qw * tx + (qy * tz - qz * ty),
        y + qw * ty + (qz * tx - qx * tz),
        z + qw * tz + (qx * ty - qy * tx),
    )


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


class FullMissionNode(Node):
    """Run all four shooting tasks and return to the command center."""

    WHEEL_LEAF_OFFSETS = {
        1: (0.0, 0.090000),
        2: (0.085595, 0.027812),
        3: (0.052901, -0.072812),
        4: (-0.052901, -0.072812),
        5: (-0.085595, 0.027812),
    }

    def __init__(self):
        super().__init__("full_mission_node")
        self.declare_parameter("start_delay_sec", 2.0)
        self.declare_parameter("navigate_action", "/navigate_to_pose")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("fire_service", "/shooter/shoot")
        self.declare_parameter("ring_aim_topic", "/ring_target/center_offset")
        self.declare_parameter("model_states_topic", "/gazebo/model_states")
        self.declare_parameter("robot_model_name", "abot_model")
        # 比赛规则：机器人静止超过 30 秒裁判可终止。导航超时按该红线收紧，
        # 卡死时尽快取消重试而不是干等 90 秒。
        self.declare_parameter("nav_timeout_sec", 40.0)
        self.declare_parameter("nav_goal_retries", 2)
        self.declare_parameter("initial_nav_goal_retries", 3)
        self.declare_parameter("nav_retry_delay_sec", 3.0)
        self.declare_parameter("nav_failure_settle_sec", 5.0)
        # 导航"无进展"看门狗：导航中距目标点的距离长时间没有缩短就提前
        # 取消重试，不再干等 40s 超时（比赛 30s 静止红线内必须有所反应）。
        self.declare_parameter("nav_stuck_timeout_sec", 12.0)
        self.declare_parameter("nav_stuck_min_progress_m", 0.03)
        # Nav2 放弃/超时但机器人已接近任务点中心时按到达处理。任务点放宽到
        # 0.30m：射击是按实际位姿转角的，偏 0.3m 不影响命中，却能把"终点被
        # 暂时挡住导致反复重试"的情况直接救回来。返回点用裁判的到位半径
        # 0.16m，避免机器人没真正回到指挥中心却上报 returned。
        self.declare_parameter("nav_position_fallback_tolerance_m", 0.30)
        self.declare_parameter("return_position_fallback_tolerance_m", 0.16)
        self.declare_parameter("task_3_right_offset_m", 0.02)
        self.declare_parameter("task_4_right_offset_m", 0.04)
        self.declare_parameter("aim_timeout_sec", 15.0)
        self.declare_parameter("wheel_aim_timeout_sec", 30.0)
        self.declare_parameter("aim_detection_max_age_sec", 0.8)
        self.declare_parameter("shot_settle_sec", 1.0)
        self.declare_parameter("ring_center_tolerance_px", 20.0)
        self.declare_parameter("ring_aim_gain", 0.0025)
        self.declare_parameter("model_center_tolerance_rad", 0.012)
        # 移动靶计分区半宽 0.05m（含弹径余量），对应 1.045m 距离约 0.048rad。
        # 对齐容差放宽到 0.02rad（约 2.1cm 落点误差），避免漂移的瞄准点让
        # 收敛判定反复失败、拖到超时盲射。
        self.declare_parameter("moving_model_tolerance_rad", 0.02)
        self.declare_parameter("model_aim_gain", 1.8)
        self.declare_parameter("max_angular_speed", 0.35)
        # 末段爬行补偿：误差×增益得到的角速度过小时底盘可能无响应，误差会
        # 卡在容差外导致对齐超时。低于该最小角速度的转向指令按最小值发出。
        self.declare_parameter("min_angular_speed_rad", 0.08)
        self.declare_parameter("wheel_height_tolerance_m", 0.015)
        self.declare_parameter("required_stable_cycles", 4)
        self.declare_parameter("ring_aim_mode", "model")
        self.declare_parameter("wheel_stable_cycles", 2)
        self.declare_parameter("fire_without_aim", True)
        self.declare_parameter("fire_response_timeout_sec", 8.0)
        # 开火前确认窗口：角度持续保持在对齐容差内这么久才真正调用射击服务。
        # 刚收敛时机器人还在减速、模型位姿快照可能滞后，立即开火会把旋转残余
        # 带进弹道（实测同一颗子弹，裁判射线与实际落点相差 5.8cm）。
        self.declare_parameter("pre_fire_settle_sec", 0.4)
        # 模型位姿停更保护：/gazebo/model_states 是 BEST_EFFORT，VM 卡顿时可能
        # 冻结。数据过期期间机器人按最后一条角速度指令继续转，瞄准等于盲转；
        # 此时应停车等待数据恢复，且不得开火。
        self.declare_parameter("model_states_max_age_sec", 0.3)
        # 超时盲射的底线：偏差超过该值（约 4.7cm 落点误差，仍在计分半宽内）
        # 才允许盲射，偏差再大必然脱靶，直接跳过省下约 12s 补射流程。
        self.declare_parameter("blind_fire_max_error_rad", 0.045)
        # 裁判的命中回执比射击服务返回晚约 0.1s。开火后先留一个确认窗口，
        # 裁判确认命中则跳过补射（避免首发命中还多打一枪、浪费时间）；窗口
        # 结束仍未确认才补射。
        self.declare_parameter("hit_verify_timeout_sec", 1.5)
        self.declare_parameter("target_2_id", 1)
        self.declare_parameter("target_3_id", 6)
        self.declare_parameter("target_4_id", 8)
        self.declare_parameter("publish_referee_targets", True)
        self.declare_parameter("referee_target_plan_topic", "/referee/target_plan")
        self.declare_parameter("referee_current_target_topic", "/referee/current_target")
        self.declare_parameter("referee_mission_event_topic", "/referee/mission_event")
        # map->odom 由 AMCL 发布；实测中 AMCL 可能长时间停更（冻结数分钟），
        # 导致 Nav2 控制器拿不到新鲜 TF、目标反复超时。这里按"最近一次收到且
        # 时间戳在前进的 map->odom"判定 TF 是否新鲜，导航前先等 TF，导航中
        # 停更则取消目标，等恢复后续跑，而不是干等 90s 超时烧掉重试次数。
        self.declare_parameter("tf_stale_max_age_sec", 2.0)
        self.declare_parameter("tf_stall_confirm_sec", 4.0)
        # 单次等待 TF 恢复的上限。比赛规则机器人静止超 30s 可被判终止，
        # 等待上限必须卡在这条红线以内；等不到就取消重试，靠重试循环续命。
        self.declare_parameter("tf_recover_timeout_sec", 25.0)
        self.declare_parameter("tf_max_resends", 4)
        # TF 停摆时用 Gazebo 真值向 /initialpose 重发当前位姿踢活 AMCL，
        # 避免干等 AMCL 自行恢复（实测可停摆数分钟）。仅仿真环境有意义。
        self.declare_parameter("recover_amcl_on_stall", True)
        self.declare_parameter("amcl_kick_topic", "/initialpose")
        self.declare_parameter("amcl_kick_delay_sec", 5.0)
        self.declare_parameter("amcl_kick_interval_sec", 6.0)
        # 叶片穿越枪口高度窗口的判定迟滞：离开该带才允许翻转通过点方向，
        # 避免在窗口边缘来回切换瞄准目标。
        self.declare_parameter("wheel_pass_hysteresis_m", 0.04)
        # 每个目标默认补射一次：裁判确认命中后跳过补射，未命中（或裁判
        # 无回音）才重新瞄准补射，把一次脱靶的 10 分损失救回来。
        self.declare_parameter("shots_per_target", 2)
        self.declare_parameter("referee_reset_service", "/referee/reset")
        self.declare_parameter("reset_referee_on_start", True)
        self.declare_parameter("referee_score_detail_topic", "/referee/score_detail")
        # 移动靶命中点预测：按靶标速度 × 弹丸飞行时间前置瞄准。
        self.declare_parameter("moving_target_lead", True)
        # 与射手节点 bullet_speed 默认值(30)保持一致，前置量才准确。
        self.declare_parameter("bullet_speed_mps", 30.0)

        self.target_ids = {
            2: int(self.get_parameter("target_2_id").value),
            3: int(self.get_parameter("target_3_id").value),
            4: int(self.get_parameter("target_4_id").value),
        }
        self._validate_target_ids()

        task_3_right_offset = float(self.get_parameter("task_3_right_offset_m").value)
        task_4_right_offset = float(self.get_parameter("task_4_right_offset_m").value)
        # Poses are the centers of the marked task areas in land_shoot.world. Points 3 and
        # 4 receive a robot-relative right correction to compensate the observed left bias.
        self.waypoints = [
            {"number": 1, "label": "1 号任务点", "x": 0.0, "y": -1.35, "yaw": 0.0,
             "aim": "ring", "model": "target_fixed"},
            {"number": 2, "label": "2 号任务点", "x": 0.0, "y": -2.70, "yaw": 0.0,
             "aim": "wheel", "model": "target_wheel"},
            {"number": 3, "label": "3 号任务点", "x": 1.125 - task_3_right_offset,
             "y": -1.925, "yaw": -math.pi / 2.0,
             "aim": "moving", "model": "target_moving_3"},
            {"number": 4, "label": "4 号任务点", "x": 1.125 + task_4_right_offset,
             "y": -0.775, "yaw": math.pi / 2.0,
             "aim": "moving", "model": "target_moving_4"},
            {"number": 0, "label": "指挥中心", "x": 0.0, "y": 0.0, "yaw": 0.0,
             "aim": None, "model": None},
        ]

        self.nav_client = ActionClient(
            self, NavigateToPose, str(self.get_parameter("navigate_action").value)
        )
        self.fire_client = self.create_client(
            Trigger, str(self.get_parameter("fire_service").value)
        )
        self.cmd_pub = self.create_publisher(
            Twist, str(self.get_parameter("cmd_vel_topic").value), 10
        )

        plan_qos = QoSProfile(depth=1)
        plan_qos.reliability = ReliabilityPolicy.RELIABLE
        plan_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        event_qos = QoSProfile(depth=10)
        event_qos.reliability = ReliabilityPolicy.RELIABLE
        self.referee_plan_pub = self.create_publisher(
            String, str(self.get_parameter("referee_target_plan_topic").value), plan_qos
        )
        self.referee_current_pub = self.create_publisher(
            String, str(self.get_parameter("referee_current_target_topic").value), event_qos
        )
        self.referee_mission_event_pub = self.create_publisher(
            String, str(self.get_parameter("referee_mission_event_topic").value), event_qos
        )
        self.create_subscription(
            PointStamped,
            str(self.get_parameter("ring_aim_topic").value),
            self.on_ring_offset,
            10,
        )
        self.create_subscription(
            ModelStates,
            str(self.get_parameter("model_states_topic").value),
            self.on_model_states,
            qos_profile_sensor_data,
        )
        self.map_odom_last_wall = None
        self.map_odom_last_stamp = None
        self.create_subscription(TFMessage, "/tf", self.on_tf, qos_profile_sensor_data)

        # 裁判计分明细（latched）：用于确认当前目标是否已命中，命中则跳过补射。
        self.score_detail = {}
        detail_qos = QoSProfile(depth=1)
        detail_qos.reliability = ReliabilityPolicy.RELIABLE
        detail_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            String,
            str(self.get_parameter("referee_score_detail_topic").value),
            self.on_score_detail,
            detail_qos,
        )
        self.referee_reset_client = self.create_client(
            Trigger, str(self.get_parameter("referee_reset_service").value)
        )
        self.referee_reset_done = not bool(
            self.get_parameter("reset_referee_on_start").value
        )
        self.last_reset_attempt_monotonic = 0.0
        # AMCL 踢活：TF 停摆超时后用真值重发初始位姿。
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            str(self.get_parameter("amcl_kick_topic").value),
            10,
        )
        self.last_amcl_kick_monotonic = None

        self.start_delay = Duration(seconds=float(self.get_parameter("start_delay_sec").value))
        self.nav_timeout = Duration(seconds=float(self.get_parameter("nav_timeout_sec").value))
        self.nav_goal_retries = int(self.get_parameter("nav_goal_retries").value)
        self.initial_nav_goal_retries = int(
            self.get_parameter("initial_nav_goal_retries").value
        )
        self.nav_retry_delay_sec = float(self.get_parameter("nav_retry_delay_sec").value)
        self.nav_failure_settle_sec = float(
            self.get_parameter("nav_failure_settle_sec").value
        )
        self.nav_stuck_timeout_sec = float(
            self.get_parameter("nav_stuck_timeout_sec").value
        )
        self.nav_stuck_min_progress_m = float(
            self.get_parameter("nav_stuck_min_progress_m").value
        )
        self.nav_position_fallback_tolerance = float(
            self.get_parameter("nav_position_fallback_tolerance_m").value
        )
        self.return_position_fallback_tolerance = float(
            self.get_parameter("return_position_fallback_tolerance_m").value
        )
        self.aim_timeout = Duration(seconds=float(self.get_parameter("aim_timeout_sec").value))
        # 旋转靶：目标叶片只有在特定相位才转到枪口高度(0.26m)，随机持续旋转下约
        # 每半个周期(实测 ~14s)才经过一次高度窗口。15s 的通用瞄准超时常等不到，
        # 导致盲射脱靶。这里给旋转靶单独用更长的超时，覆盖整个旋转周期。
        self.wheel_aim_timeout = Duration(
            seconds=float(self.get_parameter("wheel_aim_timeout_sec").value)
        )
        self.detection_max_age = Duration(
            seconds=float(self.get_parameter("aim_detection_max_age_sec").value)
        )
        self.shot_settle = Duration(seconds=float(self.get_parameter("shot_settle_sec").value))
        self.ring_tolerance = float(self.get_parameter("ring_center_tolerance_px").value)
        self.ring_gain = float(self.get_parameter("ring_aim_gain").value)
        self.model_tolerance = float(self.get_parameter("model_center_tolerance_rad").value)
        self.moving_model_tolerance = float(
            self.get_parameter("moving_model_tolerance_rad").value
        )
        self.model_gain = float(self.get_parameter("model_aim_gain").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.min_angular_speed = float(self.get_parameter("min_angular_speed_rad").value)
        self.wheel_height_tolerance = float(self.get_parameter("wheel_height_tolerance_m").value)
        self.required_stable_cycles = int(self.get_parameter("required_stable_cycles").value)
        self.pre_fire_settle_sec = float(self.get_parameter("pre_fire_settle_sec").value)
        self.model_states_max_age_sec = float(
            self.get_parameter("model_states_max_age_sec").value
        )
        self.blind_fire_max_error_rad = float(
            self.get_parameter("blind_fire_max_error_rad").value
        )
        self.ring_aim_mode = str(self.get_parameter("ring_aim_mode").value).strip().lower()
        if self.ring_aim_mode not in ("model", "vision"):
            self.ring_aim_mode = "model"
        self.wheel_stable_cycles = int(self.get_parameter("wheel_stable_cycles").value)
        self.shots_per_target = max(1, int(self.get_parameter("shots_per_target").value))
        self.fire_without_aim = bool(self.get_parameter("fire_without_aim").value)
        self.fire_response_timeout_sec = float(
            self.get_parameter("fire_response_timeout_sec").value
        )
        self.hit_verify_timeout_sec = float(
            self.get_parameter("hit_verify_timeout_sec").value
        )
        self.publish_referee_targets = bool(self.get_parameter("publish_referee_targets").value)
        self.robot_model_name = str(self.get_parameter("robot_model_name").value)
        self.tf_stale_max_age_sec = float(self.get_parameter("tf_stale_max_age_sec").value)
        self.tf_stall_confirm_sec = float(self.get_parameter("tf_stall_confirm_sec").value)
        self.tf_recover_timeout_sec = float(self.get_parameter("tf_recover_timeout_sec").value)
        self.tf_max_resends = int(self.get_parameter("tf_max_resends").value)
        self.wheel_pass_hysteresis_m = float(
            self.get_parameter("wheel_pass_hysteresis_m").value
        )
        self.recover_amcl_on_stall = bool(self.get_parameter("recover_amcl_on_stall").value)
        self.amcl_kick_delay_sec = float(self.get_parameter("amcl_kick_delay_sec").value)
        self.amcl_kick_interval_sec = float(self.get_parameter("amcl_kick_interval_sec").value)
        self.moving_target_lead = bool(self.get_parameter("moving_target_lead").value)
        self.bullet_speed_mps = float(self.get_parameter("bullet_speed_mps").value)

        self.state = "waiting"
        self.waypoint_index = 0
        self.boot_time = self.get_clock().now()
        self.nav_start_time = None
        self.nav_goal_handle = None
        self.nav_result_future = None
        self.nav_attempt = 0
        self.nav_retry_start_monotonic = None
        self.nav_failure_time_monotonic = None
        self.tf_resend_count = 0
        self.tf_stale_since_monotonic = None
        self.tf_wait_start_monotonic = None
        self.wheel_pass_side = 1.0
        self.aim_start_time = None
        self.aim_start_monotonic = None
        self.shot_done_time = None
        self.shot_done_monotonic = None
        self.fire_start_monotonic = None
        self.stable_cycles = 0
        self.stable_since_monotonic = None
        self.last_aim_error = None
        self.last_height_ready = None
        self.shots_fired = 0
        self.latest_ring_offset = None
        self.latest_ring_time = None
        self.model_poses = {}
        self.model_states_wall = None
        # 导航无进展看门狗状态
        self.nav_best_dist_monotonic = None
        self.nav_best_dist = None
        # 移动靶速度估计（用于命中点前置）：model_name -> (vx, vy, monotonic)
        self.model_velocity = {}
        self.model_prev_pose = {}
        self.last_wait_log_time = None
        self.timer = self.create_timer(0.05, self.tick)
        # 单实例保护：探测窗口结束前不开火、不动裁判、不发导航目标，
        # 避免与残留的旧实例互相干扰。
        self.instance_guard = InstanceGuard(
            self, "full_mission", lambda: self.state
        )
        self.instance_probe_done = False

        self.publish_target_plan()
        self.get_logger().info(
            "完整任务已准备：1 号环靶 -> 2 号旋转靶 -> 3/4 号移动靶 -> 返回指挥中心。"
        )

    def _validate_target_ids(self):
        if self.target_ids[2] not in range(1, 6):
            raise ValueError("target_2_id 必须在 1~5 之间")
        for task in (3, 4):
            if self.target_ids[task] not in range(6, 9):
                raise ValueError(f"target_{task}_id 必须在 6~8 之间")

    @property
    def current_waypoint(self):
        return self.waypoints[self.waypoint_index]

    def publish_target_plan(self):
        if not self.publish_referee_targets:
            return
        msg = String()
        msg.data = (
            "task_1=target_fixed,id=0;"
            f"task_2=target_wheel,id={self.target_ids[2]};"
            f"task_3=target_moving_3,id={self.target_ids[3]};"
            f"task_4=target_moving_4,id={self.target_ids[4]}"
        )
        self.referee_plan_pub.publish(msg)
        self.get_logger().info(f"已发布目标计划：{msg.data}")

    def publish_current_target(self):
        if not self.publish_referee_targets:
            return
        value = f"shoot_{self.current_waypoint['number']}"
        self.referee_current_pub.publish(String(data=value))
        self.get_logger().info(f"已通知裁判当前点位：{value}")

    def on_ring_offset(self, msg):
        self.latest_ring_offset = msg
        self.latest_ring_time = self.get_clock().now()

    def on_model_states(self, msg):
        self.model_poses = dict(zip(msg.name, msg.pose))
        self.model_states_wall = time.monotonic()
        now = time.monotonic()
        for name in ("target_moving_3", "target_moving_4"):
            pose = self.model_poses.get(name)
            if pose is None:
                continue
            prev = self.model_prev_pose.get(name)
            self.model_prev_pose[name] = (pose.position.x, pose.position.y, now)
            if prev is None:
                continue
            dt = now - prev[2]
            if dt < 0.01:
                continue
            vx = (pose.position.x - prev[0]) / dt
            vy = (pose.position.y - prev[1]) / dt
            last = self.model_velocity.get(name)
            if last is not None and now - last[2] < 0.5:
                # 一阶低通平滑，抑制 /gazebo/model_states 采样抖动。
                vx = 0.5 * last[0] + 0.5 * vx
                vy = 0.5 * last[1] + 0.5 * vy
            self.model_velocity[name] = (vx, vy, now)

    def on_score_detail(self, msg):
        try:
            detail = json.loads(msg.data)
            shooting = detail.get("shooting")
            if isinstance(shooting, dict):
                self.score_detail = shooting
        except (ValueError, TypeError):
            pass

    def hit_confirmed(self, task_number):
        """裁判计分明细里是否已有该任务点的命中记录。"""
        return str(task_number) in self.score_detail

    def try_reset_referee(self):
        """开局清零裁判计分，避免上一轮残留的命中记录阻止本轮射击。"""
        if self.referee_reset_done or not self.referee_reset_client.service_is_ready():
            return
        if time.monotonic() - self.last_reset_attempt_monotonic < 1.0:
            return
        self.last_reset_attempt_monotonic = time.monotonic()
        future = self.referee_reset_client.call_async(Trigger.Request())
        future.add_done_callback(self.on_referee_reset_done)

    def on_referee_reset_done(self, future):
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().warn(f"调用裁判重置服务失败：{exc}")
            return
        if result.success:
            self.referee_reset_done = True
            self.score_detail = {}
            self.get_logger().info("已清零裁判计分，本轮任务重新开始计分。")
        else:
            self.get_logger().warn(f"裁判重置失败：{result.message}")

    def on_tf(self, msg):
        for transform in msg.transforms:
            if transform.header.frame_id == "map" and transform.child_frame_id == "odom":
                stamp = transform.header.stamp.sec + transform.header.stamp.nanosec / 1e9
                # 只在时间戳前进时刷新：AMCL 停滞后仍可能重复发布旧时间戳，
                # 那种"活着但冻结"的 TF 不能算新鲜。
                if stamp != self.map_odom_last_stamp:
                    self.map_odom_last_stamp = stamp
                    self.map_odom_last_wall = time.monotonic()

    def map_tf_is_fresh(self):
        if self.map_odom_last_wall is None:
            return False
        return time.monotonic() - self.map_odom_last_wall <= self.tf_stale_max_age_sec

    def tick(self):
        now = self.get_clock().now()
        if self.state == "waiting":
            if not self.instance_probe_done:
                verdict = self.instance_guard.probe_once()
                if verdict is None:
                    return
                self.instance_probe_done = True
                if verdict:
                    self.get_logger().error(
                        "检测到另一个任务实例仍在运行："
                        f"{self.instance_guard.describe_conflict()}。两个实例会互相抢"
                        " /cmd_vel、互相取消导航目标并各自开火（重复子弹），本实例退出。"
                        "请先结束旧任务节点（或运行 scripts/cleanup_stack.sh）后重新启动。"
                    )
                    raise SystemExit(2)
            self.try_reset_referee()
            if now - self.boot_time >= self.start_delay:
                self.start_navigation()
            return
        if self.state == "navigating":
            self.update_navigation(now)
            return
        if self.state == "nav_retry_wait":
            elapsed = time.monotonic() - self.nav_retry_start_monotonic
            if elapsed >= self.nav_retry_delay_sec:
                self.start_navigation(is_retry=True)
            return
        if self.state == "nav_tf_wait":
            self.update_tf_wait()
            return
        if self.state == "nav_failure_settle":
            elapsed = time.monotonic() - self.nav_failure_time_monotonic
            if elapsed >= self.nav_failure_settle_sec:
                # 先切回 waiting 再推进：start_navigation 若因 action server
                # 未就绪而提前返回，下个 tick 才不会重复推进跳过点位。
                self.state = "waiting"
                self.waypoint_index += 1
                self.nav_attempt = 0
                self.start_navigation()
            return
        if self.state == "aiming":
            self.update_aiming(now)
            return
        if self.state == "shooting":
            elapsed = time.monotonic() - self.fire_start_monotonic
            if elapsed >= self.fire_response_timeout_sec:
                self.get_logger().error("射击服务响应超时，继续前往下一任务点。")
                self.complete_shot_step()
            return
        if self.state == "shot_settle":
            sim_ready = now - self.shot_done_time >= self.shot_settle
            wall_elapsed = time.monotonic() - self.shot_done_monotonic
            wall_ready = wall_elapsed >= self.shot_settle.nanoseconds / 1e9
            if sim_ready or wall_ready:
                self.state = "waiting"
                self.waypoint_index += 1
                self.nav_attempt = 0
                self.start_navigation()
            return
        if self.state == "hit_verify":
            self.update_hit_verify()
            return

    def start_navigation(self, is_retry=False):
        if not is_retry:
            self.nav_attempt = 0
            self.tf_resend_count = 0
        if not self.nav_client.wait_for_server(timeout_sec=0.1):
            self.log_waiting("等待 Nav2 /navigate_to_pose action server...")
            return
        if not self.map_tf_is_fresh():
            self.log_waiting("map->odom TF 不新鲜，暂缓发送导航目标...")
            self.tf_wait_start_monotonic = time.monotonic()
            self.state = "nav_tf_wait"
            return
        self.nav_attempt += 1
        self.send_nav_goal()

    def send_nav_goal(self):
        waypoint = self.current_waypoint
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = waypoint["x"]
        goal.pose.pose.position.y = waypoint["y"]
        goal.pose.pose.orientation = quaternion_from_yaw(waypoint["yaw"])
        self.get_logger().info(
            f"发送导航目标：{waypoint['label']}（第 {self.nav_attempt} 次尝试）"
            f"({waypoint['x']:.3f}, {waypoint['y']:.3f}, {waypoint['yaw']:.3f})"
        )
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.on_nav_goal_response)
        self.nav_start_time = self.get_clock().now()
        self.nav_goal_handle = None
        self.nav_result_future = None
        self.tf_stale_since_monotonic = None
        # 无进展看门狗基线：从发送目标时刻起算。
        self.nav_best_dist = None
        self.nav_best_dist_monotonic = time.monotonic()
        self.state = "navigating"

    def update_tf_wait(self):
        if self.map_tf_is_fresh():
            self.tf_wait_start_monotonic = None
            self.get_logger().info("map->odom TF 已恢复，继续发送导航目标。")
            if self.nav_attempt == 0:
                # 首次发送被 TF 停摆推迟，恢复后作为第 1 次尝试计。
                self.start_navigation(is_retry=True)
            elif self.tf_resend_count >= self.tf_max_resends:
                self.get_logger().warn("TF 恢复后的重发次数已达上限，按导航失败处理。")
                self.retry_or_finish_navigation()
            else:
                self.tf_resend_count += 1
                # 继续被停摆打断的当前尝试，不消耗导航重试预算。
                self.send_nav_goal()
            return
        self.log_waiting("等待 map->odom TF 恢复...")
        self.kick_amcl_if_stalled()
        if (
            self.tf_wait_start_monotonic is not None
            and time.monotonic() - self.tf_wait_start_monotonic > self.tf_recover_timeout_sec
        ):
            self.tf_wait_start_monotonic = None
            self.get_logger().error(
                f"map->odom TF 超过 {self.tf_recover_timeout_sec:.0f}s 未恢复，按导航失败处理。"
            )
            self.retry_or_finish_navigation()

    def kick_amcl_if_stalled(self):
        """TF 停摆超时后，用 Gazebo 真值重发 /initialpose 踢活 AMCL。

        实测 AMCL 停摆可长达数分钟且不会自行恢复；比赛规则不允许机器人
        长时间静止。重发初始位姿会让 AMCL 在真值附近重撒粒子并恢复发布
        map->odom，是仿真环境下最直接的止血手段。
        """
        if not self.recover_amcl_on_stall or self.tf_wait_start_monotonic is None:
            return
        stalled_for = time.monotonic() - self.tf_wait_start_monotonic
        if stalled_for < self.amcl_kick_delay_sec:
            return
        now = time.monotonic()
        if (
            self.last_amcl_kick_monotonic is not None
            and now - self.last_amcl_kick_monotonic < self.amcl_kick_interval_sec
        ):
            return
        robot_pose = self.model_poses.get(self.robot_model_name)
        if robot_pose is None:
            return
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose = robot_pose
        msg.pose.covariance[0] = 0.05
        msg.pose.covariance[7] = 0.05
        msg.pose.covariance[35] = 0.05
        self.initial_pose_pub.publish(msg)
        self.last_amcl_kick_monotonic = now
        self.get_logger().warn(
            f"map->odom TF 停摆 {stalled_for:.0f}s，已按真值重发 /initialpose 踢活 AMCL。"
        )

    def on_nav_goal_response(self, future):
        try:
            self.nav_goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f"发送导航目标失败：{exc}")
            self.retry_or_finish_navigation()
            return
        if not self.nav_goal_handle.accepted:
            self.get_logger().warn(f"Nav2 拒绝了 {self.current_waypoint['label']} 导航目标。")
            self.retry_or_finish_navigation()
            return
        self.nav_result_future = self.nav_goal_handle.get_result_async()

    def update_navigation(self, now):
        if self.nav_result_future is not None and self.nav_result_future.done():
            result = self.nav_result_future.result()
            succeeded = result.status == GoalStatus.STATUS_SUCCEEDED
            if succeeded:
                self.get_logger().info(f"已到达 {self.current_waypoint['label']}。")
                self.finish_navigation(True)
                return
            if self.robot_is_inside_current_task_area():
                self.get_logger().warn(
                    f"Nav2 状态码={result.status}，但机器人已进入"
                    f" {self.current_waypoint['label']} 区域，按到达处理。"
                )
                self.finish_navigation(True)
                return
            # 目标被 Nav2 中止时必须走重试预算再放弃。直接跳过会让一次导航
            # 故障静默放大成"全程未动却报完成"。
            self.get_logger().warn(
                f"{self.current_waypoint['label']} 导航结束，状态码={result.status}。"
            )
            self.retry_or_finish_navigation()
            return
        if not self.map_tf_is_fresh():
            if self.tf_stale_since_monotonic is None:
                self.tf_stale_since_monotonic = time.monotonic()
            elif time.monotonic() - self.tf_stale_since_monotonic >= self.tf_stall_confirm_sec:
                stale_for = time.monotonic() - self.tf_stale_since_monotonic
                self.get_logger().warn(
                    f"前往 {self.current_waypoint['label']} 期间 map->odom TF 停更 "
                    f"{stale_for:.0f}s，取消当前目标并等待恢复。"
                )
                if self.nav_goal_handle is not None:
                    self.nav_goal_handle.cancel_goal_async()
                self.stop_robot()
                self.tf_stale_since_monotonic = None
                self.tf_wait_start_monotonic = time.monotonic()
                self.state = "nav_tf_wait"
            return
        self.tf_stale_since_monotonic = None
        self.update_nav_stuck_watchdog()
        if self.nav_start_time is not None and now - self.nav_start_time > self.nav_timeout:
            if self.robot_is_inside_current_task_area():
                self.get_logger().warn(
                    f"前往 {self.current_waypoint['label']} 的 Nav2 action 虽超时，"
                    "但机器人已进入任务区域，按到达处理。"
                )
                if self.nav_goal_handle is not None:
                    self.nav_goal_handle.cancel_goal_async()
                self.finish_navigation(True)
                return
            self.get_logger().warn(f"前往 {self.current_waypoint['label']} 导航超时。")
            if self.nav_goal_handle is not None:
                self.nav_goal_handle.cancel_goal_async()
            self.retry_or_finish_navigation()

    def update_nav_stuck_watchdog(self):
        """导航中距目标点的距离长时间不缩短就提前取消，避免干等 40s 超时。"""
        robot_pose = self.model_poses.get(self.robot_model_name)
        if robot_pose is None or self.nav_best_dist_monotonic is None:
            return
        waypoint = self.current_waypoint
        distance = math.hypot(
            robot_pose.position.x - waypoint["x"],
            robot_pose.position.y - waypoint["y"],
        )
        now_monotonic = time.monotonic()
        if self.nav_best_dist is None or distance <= self.nav_best_dist - self.nav_stuck_min_progress_m:
            self.nav_best_dist = distance
            self.nav_best_dist_monotonic = now_monotonic
            return
        stuck_for = now_monotonic - self.nav_best_dist_monotonic
        if stuck_for < self.nav_stuck_timeout_sec:
            return
        self.get_logger().warn(
            f"前往 {self.current_waypoint['label']} 已 {stuck_for:.0f}s 无进展"
            f"（距目标 {distance:.2f}m），提前取消本次目标。"
        )
        if self.nav_goal_handle is not None:
            self.nav_goal_handle.cancel_goal_async()
        self.stop_robot()
        if self.robot_is_inside_current_task_area():
            self.finish_navigation(True)
            return
        self.retry_or_finish_navigation()

    def robot_is_inside_current_task_area(self):
        robot_pose = self.model_poses.get(self.robot_model_name)
        if robot_pose is None:
            return False
        waypoint = self.current_waypoint
        # 返回点用裁判的到位半径判定，避免没真正回到指挥中心就上报 returned。
        tolerance = (
            self.return_position_fallback_tolerance
            if waypoint["aim"] is None
            else self.nav_position_fallback_tolerance
        )
        distance = math.hypot(
            robot_pose.position.x - waypoint["x"],
            robot_pose.position.y - waypoint["y"],
        )
        return distance <= tolerance

    def retry_or_finish_navigation(self):
        self.stop_robot()
        retry_limit = (
            self.initial_nav_goal_retries if self.waypoint_index == 0 else self.nav_goal_retries
        )
        if self.nav_attempt <= retry_limit:
            self.get_logger().warn(
                f"{self.nav_retry_delay_sec:.1f} 秒后重试 {self.current_waypoint['label']}，"
                f"最多重试 {retry_limit} 次。"
            )
            self.nav_retry_start_monotonic = time.monotonic()
            self.state = "nav_retry_wait"
            return
        self.get_logger().error(
            f"{self.current_waypoint['label']} 已重试 {retry_limit} 次，停止重试。"
        )
        self.finish_navigation(False)

    def finish_navigation(self, succeeded):
        self.stop_robot()
        if self.current_waypoint["aim"] is None:
            self.state = "done"
            if succeeded:
                self.referee_mission_event_pub.publish(String(data="returned"))
                self.get_logger().info("完整任务完成，机器人已返回指挥中心。")
            else:
                self.get_logger().error("射击流程已完成，但机器人未能成功返回指挥中心。")
            return
        if not succeeded:
            skipped_label = self.current_waypoint["label"]
            self.get_logger().error(
                f"未到达 {skipped_label}，跳过该点射击并前往下一任务点。"
            )
            self.nav_failure_time_monotonic = time.monotonic()
            self.state = "nav_failure_settle"
            return
        self.start_aiming()

    def start_aiming(self, reset_shots=True):
        self.publish_current_target()
        self.aim_start_time = self.get_clock().now()
        self.aim_start_monotonic = time.monotonic()
        self.stable_cycles = 0
        self.stable_since_monotonic = None
        self.last_aim_error = None
        self.last_height_ready = None
        if reset_shots:
            self.shots_fired = 0
        self.state = "aiming"
        target_id = self.target_ids.get(self.current_waypoint["number"])
        suffix = "" if target_id is None else f"，目标 id={target_id}"
        self.get_logger().info(f"开始瞄准 {self.current_waypoint['label']}{suffix}。")

    def model_states_fresh(self):
        if self.model_states_wall is None:
            return False
        return time.monotonic() - self.model_states_wall <= self.model_states_max_age_sec

    def update_aiming(self, now):
        if self.hit_confirmed(self.current_waypoint["number"]):
            # 裁判已确认命中（补射期间命中回执到达），立即停射继续任务。
            self.stop_robot()
            self.complete_shot_step()
            return
        aim_type = self.current_waypoint["aim"]
        vision_ring = aim_type == "ring" and self.ring_aim_mode == "vision"
        if vision_ring:
            ready = self.update_ring_aim(now)
        else:
            ready = self.update_model_aim()
        if ready:
            if aim_type == "wheel" or vision_ring:
                # 旋转靶的高度窗口和视觉环靶沿用原来的连续周期判定。
                self.stable_cycles += 1
                required_cycles = (
                    self.wheel_stable_cycles if aim_type == "wheel" else self.required_stable_cycles
                )
                if self.stable_cycles >= required_cycles:
                    self.stop_robot()
                    self.fire()
                    return
            else:
                # 环靶(model)/移动靶：角度需持续保持对齐一小段时间才开火。
                # 刚收敛时机器人还在减速、位姿快照滞后，立即开火会把旋转残余
                # 带进弹道（实测同一颗子弹，裁判射线与实际落点相差 5.8cm）。
                now_monotonic = time.monotonic()
                if self.stable_since_monotonic is None:
                    self.stable_since_monotonic = now_monotonic
                elif now_monotonic - self.stable_since_monotonic >= self.pre_fire_settle_sec:
                    self.stop_robot()
                    self.fire()
                    return
        else:
            self.stable_cycles = 0
            self.stable_since_monotonic = None
        timeout = self.wheel_aim_timeout if aim_type == "wheel" else self.aim_timeout
        sim_timeout = now - self.aim_start_time > timeout
        wall_elapsed = time.monotonic() - self.aim_start_monotonic
        wall_timeout = wall_elapsed > timeout.nanoseconds / 1e9
        if sim_timeout or wall_timeout:
            self.stop_robot()
            if self.fire_without_aim:
                # 旋转靶：叶片不在枪口高度窗口时弹道必然从叶片上方/下方穿过，
                # 盲射必脱靶，消耗一次尝试预算等下一个窗口。
                height_blocked = (
                    self.current_waypoint["aim"] == "wheel"
                    and self.last_height_ready is False
                )
                if height_blocked or (
                    not vision_ring
                    and (
                        self.last_aim_error is None
                        or abs(self.last_aim_error) > self.blind_fire_max_error_rad
                    )
                ):
                    reason = (
                        "叶片未在枪口高度窗口"
                        if height_blocked
                        else (
                            "偏差未知"
                            if self.last_aim_error is None
                            else f"偏差 {self.last_aim_error:+.3f}rad 超过盲射阈值"
                        )
                    )
                    self.get_logger().error(
                        f"{self.current_waypoint['label']} 瞄准超时且{reason}，"
                        "本尝试不射击，重新瞄准。"
                    )
                    self.shots_fired += 1
                    self.complete_shot_step()
                    return
                self.get_logger().warn("瞄准超时，偏差在盲射阈值内，继续射击。")
                self.fire()
            else:
                self.get_logger().error(
                    f"{self.current_waypoint['label']} 瞄准超时，本点跳过射击并继续任务。"
                )
                self.complete_shot_step()

    def update_ring_aim(self, now):
        if self.latest_ring_offset is None or self.latest_ring_time is None:
            return False
        if now - self.latest_ring_time > self.detection_max_age:
            return False
        error = float(self.latest_ring_offset.point.x)
        if abs(error) <= self.ring_tolerance:
            self.stop_robot()
            return True
        self.publish_angular(-error * self.ring_gain)
        return False

    def update_model_aim(self):
        robot_pose = self.model_poses.get(self.robot_model_name)
        target_pose = self.model_poses.get(self.current_waypoint["model"])
        if robot_pose is None or target_pose is None:
            self.log_waiting("等待 Gazebo 机器人和靶子模型状态...")
            return False
        if not self.model_states_fresh():
            # 模型位姿停更（BEST_EFFORT + VM 卡顿）：此时继续按旧数据转向是
            # 盲转，停车等数据恢复，期间不开火也不计对齐时间。
            self.stop_robot()
            self.last_aim_error = None
            self.log_waiting("Gazebo 模型位姿停更，暂停瞄准等待数据恢复...")
            return False

        target_point, height_ready = self.desired_model_point(target_pose)
        dx = target_point[0] - robot_pose.position.x
        dy = target_point[1] - robot_pose.position.y
        desired_yaw = math.atan2(dy, dx)
        error = normalize_angle(desired_yaw - yaw_from_quaternion(robot_pose.orientation))
        self.last_aim_error = error
        tolerance = (
            self.moving_model_tolerance
            if self.current_waypoint["aim"] == "moving"
            else self.model_tolerance
        )
        if abs(error) <= tolerance:
            self.stop_robot()
            self.last_height_ready = height_ready
            return height_ready
        angular = error * self.model_gain
        if abs(angular) < self.min_angular_speed:
            # 末段爬行补偿：过小的角速度指令底盘可能不响应，误差会卡死在
            # 容差外直到超时，按最小角速度逼近。
            angular = math.copysign(self.min_angular_speed, angular)
        self.last_height_ready = height_ready
        self.publish_angular(angular)
        return False

    def desired_model_point(self, pose):
        aim_type = self.current_waypoint["aim"]
        local_offset = (0.0, 0.0, 0.0)
        height_ready = True
        if aim_type == "wheel":
            leaf_id = self.target_ids[2]
            local_y, local_z = self.WHEEL_LEAF_OFFSETS[leaf_id]
            local_offset = (0.0, local_y, local_z)
        elif aim_type == "moving":
            # Viewed from the target's front: 6, 7 and 8 occupy left, center and right thirds.
            target_id = self.target_ids[self.current_waypoint["number"]]
            local_y = {6: -0.060, 7: 0.0, 8: 0.060}[target_id]
            local_offset = (0.0, local_y, 0.0)

        rotated = rotate_vector(pose.orientation, local_offset)
        point = (
            pose.position.x + rotated[0],
            pose.position.y + rotated[1],
            pose.position.z + rotated[2],
        )
        if aim_type == "wheel":
            height_ready = abs(point[2] - 0.26) <= self.wheel_height_tolerance
            # 叶片绕水平轴旋转，只有经过枪口高度(0.26m)的窗口才可能被水平弹道
            # 命中；而穿越窗口时叶片横向偏移恰好达到最大值(±半径)且在窗口内
            # 近似恒定。若直接瞄准叶片瞬时位置，需求航向随旋转正弦摆动 ±3.3°，
            # 窗口开启瞬间偏差最大，实测永远对不齐（对准容差仅 0.69°）。改为
            # 瞄准叶片"即将穿越枪口高度"的通过点：方向由当前叶片在轮心上方/
            # 下方决定，旋转方向不变时为固定值，机器人可先对准再静候窗口。
            radius = math.hypot(local_y, local_z)
            if abs(rotated[2]) > self.wheel_pass_hysteresis_m:
                self.wheel_pass_side = -1.0 if rotated[2] > 0.0 else 1.0
            point = (
                point[0],
                pose.position.y + self.wheel_pass_side * radius,
                point[2],
            )
        elif aim_type == "moving" and self.moving_target_lead:
            # 弹丸飞行期间靶标继续平移（实测 ~0.1m，判定区半宽仅 0.05m），
            # 按靶标速度 × 飞行时间把瞄准点前置到预计命中位置。
            velocity = self.model_velocity.get(self.current_waypoint["model"])
            if velocity is not None and time.monotonic() - velocity[2] <= 0.5:
                robot_pose = self.model_poses.get(self.robot_model_name)
                if robot_pose is not None:
                    distance = math.hypot(
                        point[0] - robot_pose.position.x,
                        point[1] - robot_pose.position.y,
                    )
                    t_flight = distance / max(self.bullet_speed_mps, 0.1)
                    point = (
                        point[0] + velocity[0] * t_flight,
                        point[1] + velocity[1] * t_flight,
                        point[2],
                    )
        return point, height_ready

    def publish_angular(self, angular_speed):
        cmd = Twist()
        cmd.angular.z = clamp(angular_speed, -self.max_angular_speed, self.max_angular_speed)
        self.cmd_pub.publish(cmd)

    def fire(self):
        if self.state == "shooting":
            return
        self.shots_fired += 1
        self.state = "shooting"
        self.fire_start_monotonic = time.monotonic()
        if not self.fire_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("未找到 /shooter/shoot 服务，本点跳过。")
            self.complete_shot_step()
            return
        future = self.fire_client.call_async(Trigger.Request())
        future.add_done_callback(self.on_fire_done)
        self.get_logger().info(f"已在 {self.current_waypoint['label']} 调用射击服务。")

    def on_fire_done(self, future):
        # A late response can arrive after the wall-clock watchdog has advanced the mission.
        if self.state != "shooting":
            return
        try:
            result = future.result()
            log = self.get_logger().info if result.success else self.get_logger().error
            log(f"射击结果：success={result.success}, message='{result.message}'")
        except Exception as exc:
            self.get_logger().error(f"射击服务调用失败：{exc}")
        self.complete_shot_step()

    def complete_shot_step(self):
        self.stop_robot()
        if (
            self.shots_fired < self.shots_per_target
            and not self.hit_confirmed(self.current_waypoint["number"])
        ):
            # 命中回执比射击服务返回晚 ~0.1s：先进入确认窗口，确认命中则跳过
            # 补射，未命中（或无回音）窗口结束后才补射。
            self.shot_done_time = self.get_clock().now()
            self.shot_done_monotonic = time.monotonic()
            self.state = "hit_verify"
            return
        self.shot_done_time = self.get_clock().now()
        self.shot_done_monotonic = time.monotonic()
        self.state = "shot_settle"

    def update_hit_verify(self):
        if self.hit_confirmed(self.current_waypoint["number"]):
            self.get_logger().info(
                f"裁判确认 {self.current_waypoint['label']} 已命中，跳过补射。"
            )
            self.shot_done_time = self.get_clock().now()
            self.shot_done_monotonic = time.monotonic()
            self.state = "shot_settle"
            return
        if time.monotonic() - self.shot_done_monotonic >= self.hit_verify_timeout_sec:
            self.get_logger().info(
                f"{self.hit_verify_timeout_sec:.0f}s 内未收到命中确认，"
                f"在 {self.current_waypoint['label']} 补射"
                f"（第 {self.shots_fired}/{self.shots_per_target} 发），重新瞄准。"
            )
            self.start_aiming(reset_shots=False)

    def log_waiting(self, message):
        now = self.get_clock().now()
        elapsed = None if self.last_wait_log_time is None else now - self.last_wait_log_time
        if elapsed is None or elapsed.nanoseconds > 2_000_000_000:
            self.get_logger().info(message)
            self.last_wait_log_time = now

    def stop_robot(self):
        self.cmd_pub.publish(Twist())


def main():
    rclpy.init()
    node = FullMissionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
