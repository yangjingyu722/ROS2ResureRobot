import math
import time

from action_msgs.msg import GoalStatus
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion, Twist
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import Trigger


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
        self.declare_parameter("nav_timeout_sec", 90.0)
        self.declare_parameter("nav_goal_retries", 2)
        self.declare_parameter("initial_nav_goal_retries", 3)
        self.declare_parameter("nav_retry_delay_sec", 3.0)
        self.declare_parameter("nav_failure_settle_sec", 5.0)
        self.declare_parameter("nav_position_fallback_tolerance_m", 0.12)
        self.declare_parameter("task_3_right_offset_m", 0.02)
        self.declare_parameter("task_4_right_offset_m", 0.04)
        self.declare_parameter("aim_timeout_sec", 15.0)
        self.declare_parameter("wheel_aim_timeout_sec", 24.0)
        self.declare_parameter("aim_detection_max_age_sec", 0.8)
        self.declare_parameter("shot_settle_sec", 1.0)
        self.declare_parameter("ring_center_tolerance_px", 20.0)
        self.declare_parameter("ring_aim_gain", 0.0025)
        self.declare_parameter("model_center_tolerance_rad", 0.012)
        self.declare_parameter("model_aim_gain", 1.8)
        self.declare_parameter("max_angular_speed", 0.35)
        self.declare_parameter("wheel_height_tolerance_m", 0.025)
        self.declare_parameter("required_stable_cycles", 4)
        self.declare_parameter("ring_aim_mode", "model")
        self.declare_parameter("wheel_stable_cycles", 2)
        self.declare_parameter("shots_per_target", 1)
        self.declare_parameter("fire_without_aim", True)
        self.declare_parameter("fire_response_timeout_sec", 8.0)
        self.declare_parameter("target_2_id", 1)
        self.declare_parameter("target_3_id", 6)
        self.declare_parameter("target_4_id", 8)
        self.declare_parameter("publish_referee_targets", True)
        self.declare_parameter("referee_target_plan_topic", "/referee/target_plan")
        self.declare_parameter("referee_current_target_topic", "/referee/current_target")
        self.declare_parameter("referee_mission_event_topic", "/referee/mission_event")

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
        self.nav_position_fallback_tolerance = float(
            self.get_parameter("nav_position_fallback_tolerance_m").value
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
        self.model_gain = float(self.get_parameter("model_aim_gain").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.wheel_height_tolerance = float(self.get_parameter("wheel_height_tolerance_m").value)
        self.required_stable_cycles = int(self.get_parameter("required_stable_cycles").value)
        self.ring_aim_mode = str(self.get_parameter("ring_aim_mode").value).strip().lower()
        if self.ring_aim_mode not in ("model", "vision"):
            self.ring_aim_mode = "model"
        self.wheel_stable_cycles = int(self.get_parameter("wheel_stable_cycles").value)
        self.shots_per_target = max(1, int(self.get_parameter("shots_per_target").value))
        self.fire_without_aim = bool(self.get_parameter("fire_without_aim").value)
        self.fire_response_timeout_sec = float(
            self.get_parameter("fire_response_timeout_sec").value
        )
        self.publish_referee_targets = bool(self.get_parameter("publish_referee_targets").value)
        self.robot_model_name = str(self.get_parameter("robot_model_name").value)

        self.state = "waiting"
        self.waypoint_index = 0
        self.boot_time = self.get_clock().now()
        self.nav_start_time = None
        self.nav_goal_handle = None
        self.nav_result_future = None
        self.nav_attempt = 0
        self.nav_retry_start_monotonic = None
        self.nav_failure_time_monotonic = None
        self.aim_start_time = None
        self.aim_start_monotonic = None
        self.shot_done_time = None
        self.shot_done_monotonic = None
        self.fire_start_monotonic = None
        self.stable_cycles = 0
        self.shots_fired = 0
        self.latest_ring_offset = None
        self.latest_ring_time = None
        self.model_poses = {}
        self.last_wait_log_time = None
        self.timer = self.create_timer(0.05, self.tick)

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

    def tick(self):
        now = self.get_clock().now()
        if self.state == "waiting":
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
        if self.state == "nav_failure_settle":
            elapsed = time.monotonic() - self.nav_failure_time_monotonic
            if elapsed >= self.nav_failure_settle_sec:
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
                self.waypoint_index += 1
                self.nav_attempt = 0
                self.start_navigation()

    def start_navigation(self, is_retry=False):
        if not self.nav_client.wait_for_server(timeout_sec=0.1):
            self.log_waiting("等待 Nav2 /navigate_to_pose action server...")
            self.state = "waiting_for_nav"
            # Keep using the timer without resetting the mission start delay.
            self.state = "waiting"
            return

        waypoint = self.current_waypoint
        if not is_retry:
            self.nav_attempt = 0
        self.nav_attempt += 1
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
        self.state = "navigating"

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
            if not succeeded and self.robot_is_inside_current_task_area():
                succeeded = True
                self.get_logger().warn(
                    f"Nav2 状态码={result.status}，但机器人已进入"
                    f" {self.current_waypoint['label']} 区域，按到达处理。"
                )
            if succeeded:
                self.get_logger().info(f"已到达 {self.current_waypoint['label']}。")
            else:
                self.get_logger().warn(
                    f"{self.current_waypoint['label']} 导航结束，状态码={result.status}。"
                )
            self.finish_navigation(succeeded)
            return
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

    def robot_is_inside_current_task_area(self):
        robot_pose = self.model_poses.get(self.robot_model_name)
        if robot_pose is None:
            return False
        waypoint = self.current_waypoint
        distance = math.hypot(
            robot_pose.position.x - waypoint["x"],
            robot_pose.position.y - waypoint["y"],
        )
        return distance <= self.nav_position_fallback_tolerance

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
        if reset_shots:
            self.shots_fired = 0
        self.state = "aiming"
        target_id = self.target_ids.get(self.current_waypoint["number"])
        suffix = "" if target_id is None else f"，目标 id={target_id}"
        self.get_logger().info(f"开始瞄准 {self.current_waypoint['label']}{suffix}。")

    def update_aiming(self, now):
        if self.current_waypoint["aim"] == "ring" and self.ring_aim_mode == "vision":
            ready = self.update_ring_aim(now)
        else:
            ready = self.update_model_aim()
        if ready:
            self.stable_cycles += 1
            required_cycles = (
                self.wheel_stable_cycles
                if self.current_waypoint["aim"] == "wheel"
                else self.required_stable_cycles
            )
            if self.stable_cycles >= required_cycles:
                self.stop_robot()
                self.fire()
                return
        else:
            self.stable_cycles = 0
        timeout = (
            self.wheel_aim_timeout
            if self.current_waypoint["aim"] == "wheel"
            else self.aim_timeout
        )
        sim_timeout = now - self.aim_start_time > timeout
        wall_elapsed = time.monotonic() - self.aim_start_monotonic
        wall_timeout = wall_elapsed > timeout.nanoseconds / 1e9
        if sim_timeout or wall_timeout:
            self.stop_robot()
            if self.fire_without_aim:
                self.get_logger().warn("瞄准超时，按 fire_without_aim 配置继续射击。")
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

        target_point, height_ready = self.desired_model_point(target_pose)
        dx = target_point[0] - robot_pose.position.x
        dy = target_point[1] - robot_pose.position.y
        desired_yaw = math.atan2(dy, dx)
        error = normalize_angle(desired_yaw - yaw_from_quaternion(robot_pose.orientation))
        if abs(error) <= self.model_tolerance:
            self.stop_robot()
            return height_ready
        self.publish_angular(error * self.model_gain)
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
        if self.shots_fired < self.shots_per_target:
            self.get_logger().info(
                f"在 {self.current_waypoint['label']} 补射"
                f"（第 {self.shots_fired}/{self.shots_per_target} 发），重新瞄准。"
            )
            self.start_aiming(reset_shots=False)
            return
        self.shot_done_time = self.get_clock().now()
        self.shot_done_monotonic = time.monotonic()
        self.state = "shot_settle"

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
