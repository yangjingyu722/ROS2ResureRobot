#!/usr/bin/env python3
"""Wait for the simulation clock or Nav2 before starting dependent processes."""

import argparse
import sys
import time

from action_msgs.msg import GoalStatusArray
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock


NAV2_LIFECYCLE_NODES = (
    "map_server",
    "amcl",
    "controller_server",
    "planner_server",
    "behavior_server",
    "bt_navigator",
)


class ReadinessNode(Node):
    def __init__(self):
        super().__init__("ican_startup_readiness_check")
        self.clock_value = None
        self.last_clock_value = None
        self.last_clock_reset_wall = time.monotonic()
        self.clock_messages = 0
        self.create_subscription(
            Clock, "/clock", self.on_clock, qos_profile_sensor_data
        )

        self.lifecycle_clients = {
            name: self.create_client(GetState, f"/{name}/get_state")
            for name in NAV2_LIFECYCLE_NODES
        }
        self.nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.create_subscription(
            GoalStatusArray,
            "/navigate_to_pose/_action/status",
            lambda _msg: None,
            10,
        )

    def on_clock(self, msg):
        value = msg.clock.sec + msg.clock.nanosec / 1e9
        if self.last_clock_value is not None and value + 0.05 < self.last_clock_value:
            self.get_logger().warn(
                f"检测到仿真时钟回跳：{self.last_clock_value:.3f} -> {value:.3f}"
            )
            self.last_clock_reset_wall = time.monotonic()
            self.clock_messages = 0
        elif self.last_clock_value is None or value > self.last_clock_value:
            self.clock_messages += 1
        self.last_clock_value = value
        self.clock_value = value

    def action_server_count(self):
        infos = self.get_publishers_info_by_topic("/navigate_to_pose/_action/status")
        return len({(info.node_namespace, info.node_name) for info in infos})


def spin_until(executor, predicate, timeout, description):
    deadline = time.monotonic() + timeout
    last_report = 0.0
    while rclpy.ok() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.2)
        ready, detail = predicate()
        if ready:
            return
        now = time.monotonic()
        if now - last_report >= 5.0:
            print(f"[等待] {description}：{detail}", flush=True)
            last_report = now
    raise TimeoutError(f"等待超时：{description}")


def wait_for_clock(node, executor, timeout, stable_seconds):
    def predicate():
        if node.clock_value is None:
            return False, "尚未收到 /clock"
        stable_for = time.monotonic() - node.last_clock_reset_wall
        ready = (
            node.clock_value >= stable_seconds
            and stable_for >= stable_seconds
            and node.clock_messages >= 10
        )
        return ready, f"clock={node.clock_value:.2f}s，连续稳定 {stable_for:.1f}s"

    spin_until(executor, predicate, timeout, "Gazebo 仿真时钟稳定")
    print(f"[就绪] Gazebo /clock 已稳定（{node.clock_value:.2f}s）", flush=True)


def get_lifecycle_state(executor, client):
    if not client.service_is_ready():
        return None
    future = client.call_async(GetState.Request())
    deadline = time.monotonic() + 1.0
    while rclpy.ok() and not future.done() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
    if not future.done() or future.exception() is not None:
        return None
    return future.result().current_state.label


def wait_for_nav2(node, executor, timeout):
    def predicate():
        states = {
            name: get_lifecycle_state(executor, client)
            for name, client in node.lifecycle_clients.items()
        }
        inactive = [f"{name}={state or '未发现'}" for name, state in states.items() if state != "active"]
        server_count = node.action_server_count()
        action_ready = node.nav_client.server_is_ready()
        ready = not inactive and action_ready and server_count == 1
        details = inactive or [f"navigate_to_pose action servers={server_count}"]
        return ready, "，".join(details)

    spin_until(executor, predicate, timeout, "Nav2 全部激活且 action server 唯一")
    print("[就绪] Nav2 lifecycle=active，/navigate_to_pose action servers=1", flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("mode", choices=("clock", "nav2"))
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument("--stable-seconds", type=float, default=5.0)
    args = parser.parse_args()

    rclpy.init()
    node = ReadinessNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        if args.mode == "clock":
            wait_for_clock(node, executor, args.timeout, args.stable_seconds)
        else:
            wait_for_nav2(node, executor, args.timeout)
    except (KeyboardInterrupt, TimeoutError) as exc:
        print(f"[失败] {exc}", file=sys.stderr, flush=True)
        return 1
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
