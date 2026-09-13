import json
import math
import time

from gazebo_msgs.msg import ModelStates
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Int32, String
from std_srvs.srv import Trigger


WHEEL_LEAF_OFFSETS = {
    1: (0.0, 0.090000),
    2: (0.085595, 0.027812),
    3: (0.052901, -0.072812),
    4: (-0.052901, -0.072812),
    5: (-0.085595, 0.027812),
}
MOVING_REGION_CENTERS = {6: -0.060, 7: 0.0, 8: 0.060}
TASK_TARGET_MODELS = {
    1: "target_fixed",
    2: "target_wheel",
    3: "target_moving_3",
    4: "target_moving_4",
}


def inverse_rotate(q, vector):
    """Rotate a world vector into the local frame of quaternion q."""
    x, y, z = vector
    qx, qy, qz, qw = -q.x, -q.y, -q.z, q.w
    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)
    return (
        x + qw * tx + (qy * tz - qz * ty),
        y + qw * ty + (qz * tx - qx * tz),
        z + qw * tz + (qx * ty - qy * tx),
    )


def rotate(q, vector):
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


def local_point(pose, point):
    return inverse_rotate(
        pose.orientation,
        (
            point[0] - pose.position.x,
            point[1] - pose.position.y,
            point[2] - pose.position.z,
        ),
    )


def ray_plane_impact(origin, direction, target_pose):
    """Return the local impact of a world ray on target local x=0."""
    local_origin = local_point(target_pose, origin)
    local_direction = inverse_rotate(target_pose.orientation, direction)
    if abs(local_direction[0]) < 1e-8:
        return None
    distance = -local_origin[0] / local_direction[0]
    if distance < 0.0:
        return None
    return (
        0.0,
        local_origin[1] + distance * local_direction[1],
        local_origin[2] + distance * local_direction[2],
    )


def segment_plane_impact(previous, current, target_pose):
    """Return local (x, y, z) where a bullet segment crosses target local x=0."""
    p0 = local_point(target_pose, previous)
    p1 = local_point(target_pose, current)
    dx = p1[0] - p0[0]
    if abs(dx) < 1e-8:
        if abs(p1[0]) <= 0.035:
            return p1
        return None
    ratio = -p0[0] / dx
    if ratio < 0.0 or ratio > 1.0:
        return None
    return (
        0.0,
        p0[1] + ratio * (p1[1] - p0[1]),
        p0[2] + ratio * (p1[2] - p0[2]),
    )


class RefereeNode(Node):
    def __init__(self):
        super().__init__("referee_node")
        self.declare_parameter("robot_model_name", "abot_model")
        self.declare_parameter("arrival_radius_m", 0.16)
        self.declare_parameter("verify_arrival", False)
        # 默认开启射线快判：子弹速度约 20m/s、自由飞行段下落很小(~2cm/1.3m)。
        # 慢速/负载高的环境下 /gazebo/model_states 采样稀疏，对高速子弹逐帧插值
        # 会把"穿靶后又下落到地面"的样本线性插到靶面上，产生偏低的假 impact_z，
        # 让移动靶误判为脱靶。射线快判用开火时刻机器人位姿，高度准确、抗稀疏采样。
        self.declare_parameter("use_ray_quick_check", True)
        self.declare_parameter("bullet_radius_m", 0.02)
        self.declare_parameter("shot_timeout_sec", 6.5)

        # 分值与环靶几何，全部参数化，total/maximum 由配置计算得出。
        self.declare_parameter("arrival_score", 10)
        self.declare_parameter("hit_score", 10)
        self.declare_parameter("return_score", 10)
        self.declare_parameter("ring_score_center", 10)
        self.declare_parameter("ring_score_min", 6)
        self.declare_parameter("ring_radius_m", 0.135)
        self.declare_parameter("ring_band_width_m", 0.027)
        self.declare_parameter("ring_outer_margin_m", 0.0)
        self.declare_parameter("output_max_events", 100)
        self.declare_parameter(
            "task_points",
            json.dumps({1: [0.0, -1.35], 2: [0.0, -2.70],
                        3: [1.105, -1.925], 4: [1.165, -0.775]}),
        )

        self.robot_model = str(self.get_parameter("robot_model_name").value)
        self.arrival_radius = float(self.get_parameter("arrival_radius_m").value)
        self.verify_arrival = bool(self.get_parameter("verify_arrival").value)
        self.use_ray_quick_check = bool(self.get_parameter("use_ray_quick_check").value)
        self.bullet_radius = float(self.get_parameter("bullet_radius_m").value)
        self.shot_timeout = float(self.get_parameter("shot_timeout_sec").value)

        self.arrival_score = float(self.get_parameter("arrival_score").value)
        self.hit_score = float(self.get_parameter("hit_score").value)
        self.return_score = float(self.get_parameter("return_score").value)
        self.ring_score_center = int(self.get_parameter("ring_score_center").value)
        self.ring_score_min = int(self.get_parameter("ring_score_min").value)
        self.ring_radius = float(self.get_parameter("ring_radius_m").value)
        self.ring_band_width = float(self.get_parameter("ring_band_width_m").value)
        self.ring_outer_margin = float(self.get_parameter("ring_outer_margin_m").value)
        self.max_events = int(self.get_parameter("output_max_events").value)
        self.task_points = {
            int(k): tuple(float(v) for v in val)
            for k, val in json.loads(str(self.get_parameter("task_points").value)).items()
        }

        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        events = QoSProfile(depth=20)
        events.reliability = ReliabilityPolicy.RELIABLE
        self.score_pub = self.create_publisher(Int32, "/referee/score", latched)
        self.detail_pub = self.create_publisher(String, "/referee/score_detail", latched)
        self.event_pub = self.create_publisher(String, "/referee/events", events)

        self.create_subscription(
            ModelStates, "/gazebo/model_states", self.on_model_states, qos_profile_sensor_data
        )
        self.create_subscription(String, "/shooter/shot", self.on_shot, events)
        self.create_subscription(String, "/referee/target_plan", self.on_target_plan, latched)
        self.create_subscription(String, "/referee/current_target", self.on_current_target, events)
        self.create_subscription(String, "/referee/mission_event", self.on_mission_event, events)
        self.create_service(Trigger, "/referee/reset", self.on_reset)

        self.target_ids = {2: 1, 3: 6, 4: 8}
        self.model_poses = {}
        self.active_task = None
        self.pending_bullets = {}
        self.reset_score(publish=False)
        self.timer = self.create_timer(0.25, self.tick)
        self.get_logger().info(
            "裁判计分已启动：/referee/score、/referee/score_detail、/referee/events"
        )
        self.publish_score()

    def reset_score(self, publish=True):
        self.arrival_awarded = set()
        self.hit_awarded = set()
        self.return_awarded = False
        self.points = {"arrival": {}, "shooting": {}, "return": 0}
        self.pending_bullets = {}
        self.active_task = None
        self.timeline = []
        if publish:
            self.event("计分已重置")
            self.publish_score()

    def on_reset(self, _request, response):
        self.reset_score()
        response.success = True
        response.message = "score reset"
        return response

    def on_target_plan(self, msg):
        for item in msg.data.split(";"):
            fields = dict(
                part.split("=", 1) for part in item.split(",") if "=" in part
            )
            task_text = item.split("=", 1)[0].strip()
            if task_text.startswith("task_") and "id" in fields:
                try:
                    self.target_ids[int(task_text[5:])] = int(fields["id"])
                except ValueError:
                    pass
        self.publish_score()

    def on_current_target(self, msg):
        if not msg.data.startswith("shoot_"):
            return
        try:
            task = int(msg.data[6:])
        except ValueError:
            return
        if task not in TASK_TARGET_MODELS:
            return
        self.active_task = task
        if task in self.arrival_awarded:
            return
        if self.verify_arrival and not self.robot_at_task(task):
            self.event(
                f"收到 {task} 号点射击通知，但机器人未进入该点位范围，"
                f"本点不计分（verify_arrival=True）"
            )
            return
        self.arrival_awarded.add(task)
        self.points["arrival"][str(task)] = self.arrival_score
        self.event(f"到达 {task} 号任务点：+{self.arrival_score:g} 分")
        self.publish_score()

    def robot_at_task(self, task):
        point = self.task_points.get(task)
        if point is None:
            return False
        robot = self.model_poses.get(self.robot_model)
        if robot is None:
            return False
        return math.hypot(robot.position.x - point[0], robot.position.y - point[1]) <= self.arrival_radius

    def on_mission_event(self, msg):
        if msg.data.strip().lower() == "returned":
            self.award_return("任务节点确认成功返回指挥中心")

    def on_shot(self, msg):
        if self.active_task not in TASK_TARGET_MODELS:
            self.event(f"收到子弹 {msg.data}，但尚未设置有效射击点，不计分")
            return
        task = self.active_task

        # 主用射线快判：机器人发射前已 stop_robot() 停稳，开火时刻位姿可靠，
        # 射线在正确的 0.26m 高度穿靶，跨度小、不受稀疏采样影响。
        # 快判不可用时（位姿缺失/射线未穿靶）再退回子弹模型的逐帧线段追踪。
        if self.use_ray_quick_check:
            robot_pose = self.model_poses.get(self.robot_model)
            target_pose = self.model_poses.get(TASK_TARGET_MODELS[task])
            if robot_pose is not None and target_pose is not None:
                direction = rotate(robot_pose.orientation, (1.0, 0.0, 0.0))
                # Match shooter_node: 0.25 m spawn offset + 0.02 m bullet radius,
                # with the configured horizontal firing height of 0.26 m.
                origin = (
                    robot_pose.position.x + direction[0] * 0.27,
                    robot_pose.position.y + direction[1] * 0.27,
                    0.26,
                )
                impact = ray_plane_impact(origin, direction, target_pose)
                if impact is not None:
                    score = self.score_impact(task, impact)
                    if score > 0:
                        self.award_hit(task, score, impact)
                        return
                    self.event(
                        f"{task} 号点弹道经过靶面但未命中指定区域 "
                        f"(impact_y={impact[1]:.3f}, impact_z={impact[2]:.3f})"
                    )

        self.pending_bullets[msg.data] = {
            "task": task,
            "previous": None,
            "started": time.monotonic(),
            "miss_logged": False,
        }
        self.event(f"检测 {self.active_task} 号点子弹：{msg.data}")

    def on_model_states(self, msg):
        self.model_poses = dict(zip(msg.name, msg.pose))
        for name, shot in list(self.pending_bullets.items()):
            pose = self.model_poses.get(name)
            target_pose = self.model_poses.get(TASK_TARGET_MODELS[shot["task"]])
            if pose is None or target_pose is None:
                continue
            current = (pose.position.x, pose.position.y, pose.position.z)
            previous = shot["previous"]
            if previous is not None:
                impact = segment_plane_impact(previous, current, target_pose)
                if impact is not None:
                    score = self.score_impact(shot["task"], impact)
                    if score > 0:
                        self.award_hit(shot["task"], score, impact)
                        self.pending_bullets.pop(name, None)
                        continue
                    if not shot["miss_logged"]:
                        shot["miss_logged"] = True
                        _, y, z = impact
                        self.event(
                            f"{shot['task']} 号点：子弹穿过靶面但未进计分区 "
                            f"(impact_y={y:.3f}, impact_z={z:.3f}) "
                            f"需要≈{self.describe_region(shot['task'])}"
                        )
            shot["previous"] = current

    def miss_geometry(self, name, shot):
        """Snapshot of bullet last known position vs target, to diagnose 'no crossing' misses."""
        bullet = self.model_poses.get(name)
        target = self.model_poses.get(TASK_TARGET_MODELS[shot["task"]])
        if bullet is None or target is None:
            return ""
        local = local_point(target, (bullet.position.x, bullet.position.y, bullet.position.z))
        return (
            f"（子弹最后位置 x={bullet.position.x:.2f},y={bullet.position.y:.2f},z={bullet.position.z:.2f}；"
            f"靶 x={target.position.x:.2f},y={target.position.y:.2f},z={target.position.z:.2f}；"
            f"弹相对靶面局部 x={local[0]:.2f},y={local[1]:.2f},z={local[2]:.2f}）"
        )

    def describe_region(self, task):
        margin = self.bullet_radius
        if task == 1:
            return f"环形靶内半径 0~{self.ring_radius:g}m 分 {self.ring_score_center}~{self.ring_score_min} 环"
        if task == 2:
            center = WHEEL_LEAF_OFFSETS.get(self.target_ids.get(2))
            if center is None:
                return "未知旋转叶片"
            return f"叶片中心 y={center[0]:.3f}, z={center[1]:.3f} 各±{0.025 + margin:.3f}"
        center_y = MOVING_REGION_CENTERS.get(self.target_ids.get(task))
        if center_y is None:
            return "未知移动区域"
        return f"移动区域 y={center_y:g}±{0.030 + margin:.3f}, z=0±{0.035 + margin:.3f}"

    def score_impact(self, task, impact):
        _, y, z = impact
        margin = self.bullet_radius
        if task == 1:
            radius = math.hypot(y, z)
            # 统一口径：按弹心落点判环，外圈命中阈值 = ring_radius(+可选余量)，
            # 内部各环边界不再额外加余量。
            if radius > self.ring_radius + self.ring_outer_margin:
                return 0
            band = int(radius / self.ring_band_width)
            return max(self.ring_score_min, self.ring_score_center - band)
        if task == 2:
            target_id = self.target_ids.get(2)
            center = WHEEL_LEAF_OFFSETS.get(target_id)
            if center is None:
                return 0
            return self.hit_score if abs(y - center[0]) <= 0.025 + margin and abs(z - center[1]) <= 0.025 + margin else 0
        target_id = self.target_ids.get(task)
        center_y = MOVING_REGION_CENTERS.get(target_id)
        if center_y is None:
            return 0
        in_region = abs(y - center_y) <= 0.030 + margin and abs(z) <= 0.035 + margin
        return self.hit_score if in_region else 0

    def award_hit(self, task, score, impact):
        if task in self.hit_awarded:
            return
        self.hit_awarded.add(task)
        self.points["shooting"][str(task)] = score
        self.event(
            f"命中 {task} 号任务目标：+{score:g} 分 "
            f"(impact_y={impact[1]:.3f}, impact_z={impact[2]:.3f})"
        )
        self.publish_score()

    def tick(self):
        now = time.monotonic()
        for name, shot in list(self.pending_bullets.items()):
            if now - shot["started"] > self.shot_timeout:
                self.pending_bullets.pop(name, None)
                self.event(f"{shot['task']} 号点未检测到有效命中：{name}{self.miss_geometry(name, shot)}")

        # Position is a backup in case the explicit mission event is lost. Reaching
        # task 4 proves that a return leg is in progress; an earlier missed task
        # must not suppress the independent return score.
        if self.return_awarded or 4 not in self.arrival_awarded:
            return
        robot = self.model_poses.get(self.robot_model)
        if robot is None:
            return
        if math.hypot(robot.position.x, robot.position.y) <= self.arrival_radius:
            self.award_return("检测到机器人返回指挥中心")

    def award_return(self, reason):
        if self.return_awarded:
            return
        self.return_awarded = True
        self.points["return"] = self.return_score
        self.event(f"{reason}：+{self.return_score:g} 分")
        self.publish_score()

    def total(self):
        return (
            sum(float(v) for v in self.points["arrival"].values())
            + sum(float(v) for v in self.points["shooting"].values())
            + float(self.points["return"])
        )

    def maximum(self):
        n_tasks = len(TASK_TARGET_MODELS)
        arrival_max = n_tasks * self.arrival_score
        shoot_max = self.ring_score_center + (n_tasks - 1) * self.hit_score
        return arrival_max + shoot_max + self.return_score

    def record_event(self, text):
        self.timeline.append({"t": time.strftime("%H:%M:%S"), "e": text})
        if self.max_events > 0 and len(self.timeline) > self.max_events:
            self.timeline = self.timeline[-self.max_events:]

    def publish_score(self):
        total = self.total()
        maximum = self.maximum()
        self.score_pub.publish(Int32(data=int(round(total))))
        detail = {
            "total": total,
            "maximum": maximum,
            "arrival": self.points["arrival"],
            "shooting": self.points["shooting"],
            "return": self.points["return"],
            "target_ids": self.target_ids,
            "timeline": self.timeline,
        }
        self.detail_pub.publish(String(data=json.dumps(detail, ensure_ascii=False, sort_keys=True)))
        self.get_logger().info(f"当前总分：{total:g}/{maximum:g}")

    def event(self, text):
        self.record_event(text)
        self.event_pub.publish(String(data=text))
        self.get_logger().info(text)


def main():
    rclpy.init()
    node = RefereeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()