from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def yaw_to_quaternion(yaw):
    import math

    return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)


class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__("initial_pose_publisher")
        self.declare_parameter("x", 0.0)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("yaw", 0.0)
        self.declare_parameter("repeat_count", 10)
        self.declare_parameter("period_sec", 0.5)

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", qos)
        self.count = 0
        self.repeat_count = int(self.get_parameter("repeat_count").value)
        self.timer = self.create_timer(float(self.get_parameter("period_sec").value), self.publish_pose)
        self.get_logger().info("Publishing initial pose for AMCL.")

    def publish_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = float(self.get_parameter("x").value)
        msg.pose.pose.position.y = float(self.get_parameter("y").value)
        qx, qy, qz, qw = yaw_to_quaternion(float(self.get_parameter("yaw").value))
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.pose.covariance[0] = 0.02
        msg.pose.covariance[7] = 0.02
        msg.pose.covariance[35] = 0.05
        self.pub.publish(msg)
        self.count += 1
        if self.count >= self.repeat_count:
            self.timer.cancel()
            self.get_logger().info("Initial pose published.")


def main():
    rclpy.init()
    node = InitialPosePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
