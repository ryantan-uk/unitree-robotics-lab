"""读 Rosbag (rosbag2, sqlite3) -> Python 消息列表 / NumPy。15 节讲它的原理。"""
from __future__ import annotations

from pathlib import Path

import numpy as np


def read_bag(path, topics: list[str] | None = None) -> dict[str, list]:
    """返回 {topic: [(记录时刻 s, 反序列化后的消息), ...]}"""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    out: dict[str, list] = {t: [] for t in types if topics is None or t in topics}
    while reader.has_next():
        topic, raw, stamp_ns = reader.read_next()
        if topic in out:
            out[topic].append((stamp_ns * 1e-9, deserialize_message(raw, types[topic])))
    return out


def bag_info(path) -> list[tuple[str, str, int]]:
    """[(topic, 类型, 条数)]"""
    import rosbag2_py
    info = rosbag2_py.Info().read_metadata(str(path), "sqlite3")
    return [(t.topic_metadata.name, t.topic_metadata.type, t.message_count) for t in info.topics_with_message_count]


def bag_to_arrays(path, robot: str) -> dict:
    """把一个 demo bag 变成一组对齐好的 NumPy 数组 (data/*.npz 就是这么来的)。"""
    from .parsing import lowstate_to_dict, stack_records
    odom_topic = "/sportmodestate" if robot == "go2" else "/odommodestate"
    bag = read_bag(path, ["/lowstate", odom_topic])
    low = bag["/lowstate"]
    d = stack_records([lowstate_to_dict(m, robot) for _, m in low])
    t0 = low[0][0]
    d["t"] = np.array([t for t, _ in low]) - t0
    sp = bag.get(odom_topic, [])
    d["sport_t"] = np.array([t for t, _ in sp]) - t0
    d["position"] = np.array([list(m.position) for _, m in sp])
    d["velocity"] = np.array([list(m.velocity) for _, m in sp])
    d["yaw_speed"] = np.array([m.yaw_speed for _, m in sp])
    return d


def load_npz(name: str) -> dict:
    """读 data/<name>.npz"""
    from .connect import DATA
    p = Path(name) if str(name).endswith(".npz") else DATA / f"{name}.npz"
    with np.load(p) as z:
        return {k: z[k] for k in z.files}
