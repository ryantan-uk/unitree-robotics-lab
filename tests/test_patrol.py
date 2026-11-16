"""综合实验参考解 (patrol_controller) 的决策与感知函数测试 —— 纯函数，不需要 ROS 在运行。"""
import math

import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")            # 模块顶部 import 了 rclpy 的消息类型
from unitree_lab_nodes.patrol_controller import decide, front_distance  # noqa: E402


def scan(n=360):
    return np.full(n, 5.0), -math.pi, 2 * math.pi / n


def test_front_distance_only_looks_ahead():
    r, a0, da = scan()
    r[180] = 1.2          # 正前方
    r[90] = 0.3           # 右侧 (-90 度)
    r[0] = 0.1            # 正后方
    assert front_distance(r, a0, da) == pytest.approx(1.2)


def test_front_distance_ignores_inf_and_handles_empty():
    r, a0, da = scan()
    r[:] = np.inf
    assert front_distance(r, a0, da) == math.inf
    r[185] = 2.0
    assert front_distance(r, a0, da) == pytest.approx(2.0)


@pytest.mark.parametrize("state,front,healthy,expected", [
    ("FORWARD", 5.0, True, "FORWARD"),
    ("FORWARD", 0.5, True, "TURN"),
    ("TURN", 1.0, True, "TURN"),          # 迟滞区间内保持
    ("FORWARD", 1.0, True, "FORWARD"),
    ("TURN", 2.0, True, "FORWARD"),
    ("FORWARD", 5.0, False, "HALT"),
    ("TURN", 0.1, False, "HALT"),
    ("HALT", 5.0, True, "FORWARD"),
    ("HALT", 0.5, True, "TURN"),
])
def test_decide_transitions(state, front, healthy, expected):
    new_state, _ = decide(state, front, healthy, stop_dist=0.9, clear_dist=1.4)
    assert new_state == expected


def test_decide_outputs():
    assert decide("FORWARD", 5.0, True, v=0.3, w=0.5)[1] == (0.3, 0.0)
    assert decide("FORWARD", 0.1, True, v=0.3, w=0.5)[1] == (0.0, 0.5)
    assert decide("FORWARD", 0.1, False)[1] == (0.0, 0.0)


def test_no_chatter_around_threshold():
    """前方距离在 stop_dist 附近抖动时，状态最多切换一次。"""
    state, switches = "FORWARD", 0
    for front in 0.9 + 0.05 * np.sin(np.arange(200)):
        new_state, _ = decide(state, front, True, stop_dist=0.9, clear_dist=1.4)
        switches += new_state != state
        state = new_state
    assert switches <= 1
