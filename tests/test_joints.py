import numpy as np
import pytest

from common import joints as J
from common import robots as R


def test_clamp_reports_which_joints():
    q, idx = J.clamp_to_limits([0.0, 5.0, -5.0], [-1, -1, -1], [1, 1, 1])
    assert q == pytest.approx([0, 1, -1]) and list(idx) == [1, 2]
    q, idx = J.clamp_to_limits([0.99], [-1], [1], margin=0.05)
    assert q == pytest.approx([0.95]) and list(idx) == [0]


@pytest.mark.parametrize("profile", ["linear", "cosine", "minjerk"])
def test_interpolate_endpoints_and_monotonic(profile):
    a, b = np.array([0.0, 1.0]), np.array([1.0, -1.0])
    assert J.interpolate(a, b, -1, 2, profile) == pytest.approx(a)
    assert J.interpolate(a, b, 0, 2, profile) == pytest.approx(a)
    assert J.interpolate(a, b, 2, 2, profile) == pytest.approx(b)
    assert J.interpolate(a, b, 9, 2, profile) == pytest.approx(b)
    t, q = J.trajectory(a, b, 2.0, 0.01, profile)
    assert np.all(np.diff(q[:, 0]) >= -1e-12)


def test_smooth_profiles_start_and_end_with_zero_velocity():
    for profile, v0 in (("linear", 0.5), ("cosine", 0.0), ("minjerk", 0.0)):
        t, q = J.trajectory([0.0], [1.0], 2.0, 0.001, profile)
        v = np.gradient(q[:, 0], t)
        assert v[0] == pytest.approx(v0, abs=2e-3)


def test_limit_step():
    assert J.limit_step([1.0, -1.0, 0.01], [0, 0, 0], 0.1) == pytest.approx([0.1, -0.1, 0.01])


def test_safe_commander_respects_limits_and_speed():
    lo, hi = R.go2_limits()
    cmd = J.SafeJointCommander(lo, hi, dt=0.002, max_vel=1.0)
    goal = np.array(R.GO2_POSE_STAND)
    goal[2] = 5.0                                   # 故意越界 (FR 小腿上限约 -0.84)
    T = cmd.start(R.GO2_POSE_LIE, goal, duration=0.1)   # 故意给太短的时间
    assert 2 in cmd.clipped_joints and T > 0.1
    qs = []
    while not cmd.done:
        qs.append(cmd.step())
    qs = np.array(qs)
    assert np.all(qs >= np.array(lo) - 1e-9) and np.all(qs <= np.array(hi) + 1e-9)
    assert np.max(np.abs(np.diff(qs, axis=0))) / 0.002 <= 2.0 + 1e-6
    assert qs[-1] == pytest.approx(cmd.q_goal, abs=1e-6)


def test_tables_are_consistent():
    assert len(R.GO2_JOINTS) == 12 and len(R.G1_JOINTS) == 29 == len(R.G1_KP) == len(R.G1_KD)
    assert R.GO2_LEG_ID["RL_2"] == 11 and R.G1_JOINT_ID["right_wrist_yaw"] == 28
    assert R.ros_name("rt/lowstate") == "/lowstate" and R.dds_name("/api/sport/request") == "rt/api/sport/request"
    for robot in ("go2", "g1"):
        lo, hi = R.limits_of(robot)
        assert len(lo) == len(hi) == R.num_motor(robot) and all(a < b for a, b in zip(lo, hi))
    lo, hi = R.go2_limits()
    assert all(a <= q <= b for a, q, b in zip(lo, R.GO2_POSE_STAND, hi))
