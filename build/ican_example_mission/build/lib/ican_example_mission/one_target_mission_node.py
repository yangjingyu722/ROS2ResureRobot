import math

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion, Twist
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger


def quaternion_from_yaw(yaw):
    return Quaternion(z=math.sin(yaw * 0.5), w=math.cos(yaw * 0.5))


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


class OneTargetMissionNode(Node):
    def __init__(self):
        super().__init__("one_target_mission_node")
        self.declare_parameter("start_delay_sec", 8.0)
        self.declare_parameter("navigate_action", "/navigate_to_pose")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("fire_service", "/shooter/shoot")
        self.declare_parameter("ring_aim_topic", "/ring_target/center_offset")
        self.declare_parameter("target_x", 0.0)
        self.declare_parameter("target_y", -1.35)
        self.declare_parameter("target_yaw", 0.0)
        self.declare_parameter("nav_timeout_sec", 90.0)
        self.declare_parameter("enable_aiming", True)
        self.declare_parameter("aim_timeout_sec", 10.0)
        self.declare_parameter("aim_detection_max_age_sec", 0.8)
        self.declare_parameter("center_tolerance_px", 25.0)
        self.declare_parameter("aim_gain", 0.0025)
        self.declare_parameter("max_angular_speed", 0.35)
        self.declare_parameter("fire_without_aim", True)
        self.declare_parameter("target_2_id", 1)
        self.declare_parameter("target_3_id", 6)
        self.declare_parameter("target_4_id", 8)
        self.declare_parameter("publish_referee_targets", True)
        self.declare_parameter("referee_target_plan_topic", "/referee/target_plan")
        self.declare_parameter("referee_current_target_topic", "/referee/current_target")

        self.nav_client = ActionClient(self, NavigateToPose, str(self.get_parameter("navigate_action").value))
        self.fire_client = self.create_client(Trigger, str(self.get_parameter("fire_service").value))
        self.cmd_pub = self.create_publisher(Twist, str(self.get_parameter("cmd_vel_topic").value), 10)
        referee_plan_qos = QoSProfile(depth=1)
        referee_plan_qos.reliability = ReliabilityPolicy.RELIABLE
        referee_plan_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        referee_event_qos = QoSProfile(depth=10)
        referee_event_qos.reliability = ReliabilityPolicy.RELIABLE
        self.referee_target_plan_pub = self.create_publisher(
            String, str(self.get_parameter("referee_target_plan_topic").value), referee_plan_qos
        )
        self.referee_current_target_pub = self.create_publisher(
            String, str(self.get_parameter("referee_current_target_topic").value), referee_event_qos
        )
        self.create_subscription(
            PointStamped,
            str(self.get_parameter("ring_aim_topic").value),
            self.on_ring_offset,
            10,
        )

        self.state = "waiting"
        self.boot_time = self.get_clock().now()
        self.start_delay = Duration(seconds=float(self.get_parameter("start_delay_sec").value))
        self.nav_timeout = Duration(seconds=float(self.get_parameter("nav_timeout_sec").value))
        self.aim_timeout = Duration(seconds=float(self.get_parameter("aim_timeout_sec").value))
        self.aim_detection_max_age = Duration(seconds=float(self.get_parameter("aim_detection_max_age_sec").value))
        self.enable_aiming = bool(self.get_parameter("enable_aiming").value)
        self.fire_without_aim = bool(self.get_parameter("fire_without_aim").value)
        self.center_tolerance_px = float(self.get_parameter("center_tolerance_px").value)
        self.aim_gain = float(self.get_parameter("aim_gain").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.publish_referee_targets = bool(self.get_parameter("publish_referee_targets").value)
        self.target_2_id = int(self.get_parameter("target_2_id").value)
        self.target_3_id = int(self.get_parameter("target_3_id").value)
        self.target_4_id = int(self.get_parameter("target_4_id").value)
        self.latest_ring_offset = None
        self.latest_ring_time = None
        self.nav_goal_handle = None
        self.nav_result_future = None
        self.nav_start_time = None
        self.aim_start_time = None
        self.timer = self.create_timer(0.05, self.tick)

        self.get_logger().info(
            "示例流程准备完成：先导航到 1 号任务点 "
            f"({float(self.get_parameter('target_x').value):.3f}, "
            f"{float(self.get_parameter('target_y').value):.3f}, "
            f"{float(self.get_parameter('target_yaw').value):.3f})，然后瞄准环形靶并射击。"
        )
        self.get_logger().info(
            "预留靶子配置："
            f"target_2_id={self.target_2_id}, "
            f"target_3_id={self.target_3_id}, "
            f"target_4_id={self.target_4_id}"
        )
        self.publish_referee_target_plan()

    def publish_referee_target_plan(self):
        if not self.publish_referee_targets:
            return
        msg = String()
        msg.data = (
            "task_1=target_fixed,id=0;"
            f"task_2=target_wheel,id={self.target_2_id};"
            f"task_3=target_moving_3,id={self.target_3_id};"
            f"task_4=target_moving_4,id={self.target_4_id}"
        )
        self.referee_target_plan_pub.publish(msg)
        self.get_logger().info(f"已向裁判系统发布目标计划：{msg.data}")

    def publish_referee_current_target(self):
        if not self.publish_referee_targets:
            return
        msg = String()
        msg.data = "shoot_1"
        self.referee_current_target_pub.publish(msg)
        self.get_logger().info("已向裁判系统发布当前射击点位：shoot_1")

    def on_ring_offset(self, msg):
        self.latest_ring_offset = msg
        self.latest_ring_time = self.get_clock().now()

    def tick(self):
        now = self.get_clock().now()
        if self.state == "waiting":
            if now - self.boot_time >= self.start_delay:
                self.start_navigation()
            return

        if self.state == "navigating":
            if self.nav_result_future is not None and self.nav_result_future.done():
                result = self.nav_result_future.result()
                status = result.status
                if status == GoalStatus.STATUS_SUCCEEDED:
                    self.get_logger().info("已到达 1 号任务点，准备瞄准。")
                else:
                    self.get_logger().warn(f"导航未成功结束，状态码={status}，仍继续演示射击流程。")
                self.stop_robot()
                self.start_aiming()
                return
            if self.nav_start_time is not None and now - self.nav_start_time > self.nav_timeout:
                self.get_logger().warn("导航超时，取消目标并继续演示射击流程。")
                if self.nav_goal_handle is not None:
                    self.nav_goal_handle.cancel_goal_async()
                self.stop_robot()
                self.start_aiming()
            return

        if self.state == "aiming":
            self.update_aiming(now)

    def start_navigation(self):
        if not self.nav_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().info("等待 Nav2 /navigate_to_pose action server...")
            return

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(self.get_parameter("target_x").value)
        goal.pose.pose.position.y = float(self.get_parameter("target_y").value)
        goal.pose.pose.orientation = quaternion_from_yaw(float(self.get_parameter("target_yaw").value))

        self.get_logger().info("发送导航目标：1 号任务点。")
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.on_nav_goal_response)
        self.nav_start_time = self.get_clock().now()
        self.state = "navigating"

    def on_nav_goal_response(self, future):
        self.nav_goal_handle = future.result()
        if not self.nav_goal_handle.accepted:
            self.get_logger().error("Nav2 拒绝了导航目标，继续演示射击流程。")
            self.start_aiming()
            return
        self.get_logger().info("Nav2 已接受导航目标。")
        self.nav_result_future = self.nav_goal_handle.get_result_async()

    def start_aiming(self):
        self.aim_start_time = self.get_clock().now()
        self.publish_referee_current_target()
        if not self.enable_aiming:
            self.fire()
            return
        self.get_logger().info("开始根据 /ring_target/center_offset 调整车头。")
        self.state = "aiming"

    def update_aiming(self, now):
        if self.latest_ring_offset is None or self.latest_ring_time is None:
            if now - self.aim_start_time > self.aim_timeout:
                self.get_logger().warn("瞄准超时且未检测到环形靶。")
                self.finish_aim_or_fire()
            return

        if now - self.latest_ring_time > self.aim_detection_max_age:
            if now - self.aim_start_time > self.aim_timeout:
                self.get_logger().warn("瞄准超时，最近一次环形靶检测已过期。")
                self.finish_aim_or_fire()
            return

        error_x = self.latest_ring_offset.point.x
        confidence = self.latest_ring_offset.point.z
        self.get_logger().info(f"环形靶偏移：x={error_x:.1f}px confidence={confidence:.2f}")

        if abs(error_x) <= self.center_tolerance_px:
            self.get_logger().info("环形靶已接近画面中心，准备射击。")
            self.stop_robot()
            self.fire()
            return

        cmd = Twist()
        cmd.angular.z = clamp(-error_x * self.aim_gain, -self.max_angular_speed, self.max_angular_speed)
        self.cmd_pub.publish(cmd)

        if now - self.aim_start_time > self.aim_timeout:
            self.get_logger().warn("瞄准超时。")
            self.finish_aim_or_fire()

    def finish_aim_or_fire(self):
        self.stop_robot()
        if self.fire_without_aim:
            self.fire()
        else:
            self.get_logger().warn("fire_without_aim=false，未射击。")
            self.state = "done"

    def fire(self):
        self.state = "shooting"
        if not self.fire_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("未找到 /shooter/shoot 服务。")
            self.state = "done"
            return
        future = self.fire_client.call_async(Trigger.Request())
        future.add_done_callback(self.on_fire_done)
        self.get_logger().info("已调用射击服务。")

    def on_fire_done(self, future):
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f"射击服务调用失败：{exc}")
            self.state = "done"
            return
        self.get_logger().info(f"射击完成：success={result.success}, message='{result.message}'")
        self.state = "done"

    def stop_robot(self):
        try:
            self.cmd_pub.publish(Twist())
        except Exception:
            pass


def main():
    rclpy.init()
    node = OneTargetMissionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        if rclpy.ok():
            node.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
