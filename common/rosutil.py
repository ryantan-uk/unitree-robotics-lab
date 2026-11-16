"""在 Notebook 里用 ROS 2 的小工具: 收一段数据、跑一条命令、转一会儿节点。"""
from __future__ import annotations

import subprocess
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data  # noqa: F401

_counter = 0


def ros_init():
    """rclpy.init()，可重复调用。"""
    if not rclpy.ok():
        rclpy.init()


def unique(name: str) -> str:
    global _counter
    _counter += 1
    return f"{name}_{_counter}"


def sh(cmd: str, timeout: float = 30.0, show: bool = True) -> str:
    """跑一条 shell 命令并返回输出 (超时会被杀掉，适合 `ros2 topic echo --once` 之类)。"""
    try:
        out = subprocess.run(["bash", "-c", cmd], capture_output=True, text=True, timeout=timeout)
        text = (out.stdout + out.stderr).rstrip()
    except subprocess.TimeoutExpired as e:
        text = ((e.stdout or b"").decode(errors="replace") if isinstance(e.stdout, bytes) else (e.stdout or "")).rstrip()
        text += f"\n[超时 {timeout:.0f}s，已终止]"
    if show:
        print(text)
    return text


def spin_for(nodes, seconds: float, threads: int = 1, until=None):
    """让一个或多个节点转 seconds 秒 (或直到 until() 为真)。"""
    ros_init()
    nodes = nodes if isinstance(nodes, (list, tuple)) else [nodes]
    ex = MultiThreadedExecutor(threads) if threads > 1 else SingleThreadedExecutor()
    for n in nodes:
        ex.add_node(n)
    t_end = time.monotonic() + seconds
    try:
        while time.monotonic() < t_end and not (until and until()):
            ex.spin_once(timeout_sec=0.05)
    finally:
        for n in nodes:
            ex.remove_node(n)
        ex.shutdown()


def collect(topic: str, msg_type, seconds: float = 2.0, count: int | None = None, qos=10):
    """订阅 topic，收 seconds 秒或 count 条，返回 [(接收时刻 s, 消息), ...]。"""
    ros_init()
    node = Node(unique("collector"))
    got = []
    node.create_subscription(msg_type, topic, lambda m: got.append((time.monotonic(), m)), qos)
    spin_for(node, seconds, until=(lambda: len(got) >= count) if count else None)
    node.destroy_node()
    return got


def wait_for_topic(topic: str, timeout: float = 10.0) -> bool:
    """等到某个 topic 上出现发布者。"""
    ros_init()
    node = Node(unique("waiter"))
    ok = False
    t_end = time.monotonic() + timeout
    while time.monotonic() < t_end:
        if node.count_publishers(topic) > 0:
            ok = True
            break
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    return ok


def topic_table(node: Node | None = None, wait: float = 1.5) -> list[tuple[str, str, int, int]]:
    """当前 DDS 网络上的 [(topic, 类型, 发布者数, 订阅者数)]，相当于 `ros2 topic list -t` 加计数。"""
    ros_init()
    own = node is None
    node = node or Node(unique("lister"))
    t_end = time.monotonic() + wait
    while time.monotonic() < t_end:
        rclpy.spin_once(node, timeout_sec=0.1)
    rows = [(name, types[0], node.count_publishers(name), node.count_subscribers(name))
            for name, types in sorted(node.get_topic_names_and_types())]
    if own:
        node.destroy_node()
    return rows
