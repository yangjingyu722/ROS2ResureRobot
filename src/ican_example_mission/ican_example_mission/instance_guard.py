"""任务节点单实例保护。

同一张 ROS 图里同时存在两个任务节点（上一轮进程没退干净、或重复执行了
启动命令）时，两个实例会同时发布 /cmd_vel、互相取消对方的 Nav2 目标，
并在各自瞄准就绪时各自调用一次射击服务——实测表现为"机器人不动"和
"一次冒出两颗子弹"。

本模块用 /mission/instance_heartbeat 心跳做互斥：每个实例以 0.5s 周期
发布带启动时间的心跳；新实例启动后先探测几秒，若发现仍存活的其它实例
（对方未处于 done 终态）就报错退出，把双实例问题暴露在启动阶段而不是
留给赛场上排查。
"""

import os
import socket
import time

from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

HEARTBEAT_TOPIC = "/mission/instance_heartbeat"

# 探测窗口：新实例启动后观察这么久再决定是否放行。
PROBE_SECONDS = 4.0
# 心跳新鲜度：超过该时长的心跳视为已退出实例的残留（TRANSIENT_LOCAL 会
# 把最后一条心跳长期保留，必须按内容时间戳过滤，否则会误杀新实例）。
FRESH_AGE_SEC = 1.5


class InstanceGuard:
    """发布本实例心跳，并探测图中是否已有其它存活的任务实例。"""

    def __init__(self, node, kind, state_fn, probe_seconds=PROBE_SECONDS):
        self._node = node
        self._kind = str(kind)
        self._state_fn = state_fn
        self._probe_seconds = float(probe_seconds)
        self._probe_start = time.monotonic()
        self._probed = False
        self._id = f"{socket.gethostname()}:{os.getpid()}:{time.time_ns()}"
        self._last_fresh_peer_monotonic = None
        self._last_peer_payload = None

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._pub = node.create_publisher(String, HEARTBEAT_TOPIC, qos)
        self._sub = node.create_subscription(
            String, HEARTBEAT_TOPIC, self._on_heartbeat, qos
        )
        self._timer = node.create_timer(0.5, self._publish_heartbeat)
        self._publish_heartbeat()

    def _publish_heartbeat(self):
        msg = String()
        msg.data = f"{self._id}|{self._kind}|{self._state_fn()}|{time.time():.3f}"
        self._pub.publish(msg)

    def _on_heartbeat(self, msg):
        parts = msg.data.split("|")
        if len(parts) != 4 or parts[0] == self._id:
            return
        try:
            peer_wall_time = float(parts[3])
        except ValueError:
            return
        age = time.time() - peer_wall_time
        if age < 0.0 or age > FRESH_AGE_SEC:
            return
        self._last_fresh_peer_monotonic = time.monotonic()
        self._last_peer_payload = parts

    def probe_once(self):
        """启动探测：只在探测窗口结束时评估一次。

        返回 None 表示仍在探测（或已评估完毕，调用方不应再问）；True 表示
        存在仍存活的其它实例；False 表示无冲突。此后即便出现新的实例也不
        再自行退出——让对方（更新的实例）在自己的启动探测里发现我们并退出，
        避免"谁杀谁"的判定抖动。
        """
        if self._probed:
            return None
        if time.monotonic() - self._probe_start < self._probe_seconds:
            return None
        self._probed = True
        alive_peer_recently = (
            self._last_fresh_peer_monotonic is not None
            and time.monotonic() - self._last_fresh_peer_monotonic <= 2.0
        )
        peer_finished = (
            self._last_peer_payload is not None and self._last_peer_payload[2] == "done"
        )
        return alive_peer_recently and not peer_finished

    def describe_conflict(self):
        if self._last_peer_payload is None:
            return "未知实例"
        kind, instance_id = self._last_peer_payload[1], self._last_peer_payload[0]
        return f"{kind}（实例 {instance_id}）"
