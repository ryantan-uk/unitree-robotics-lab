"""维护者工具 tools/datagen/sim.py (合成样本数据的来源) 的测试。"""
import numpy as np
import pytest

from common import robots as R
from common.parsing import parse_remote
from tools.datagen.sim import G1Sim, Go2Sim, min_wall_distance, raycast


def run(sim, seconds, dt=0.005):
    for _ in range(int(seconds / dt)):
        sim.step(dt)


def test_raycast_hits_walls():
    d = raycast(0.0, 0.0, 0.0, np.array([0.0, np.pi, np.pi / 2, -np.pi / 2]))
    assert d == pytest.approx([6.5, 1.5, 3.5, 2.5], abs=1e-6)
    assert min_wall_distance(0, 0) == pytest.approx(1.5)


def test_go2_stand_up_moves_joints_and_height():
    s = Go2Sim()
    run(s, 0.5)
    assert s.height < 0.1 and s.foot_force.mean() < 10
    s.stand_up(); run(s, 3.0)
    assert s.joints.q == pytest.approx(R.GO2_POSE_STAND, abs=0.12)
    assert s.height == pytest.approx(0.32, abs=0.01) and s.foot_force.mean() > 40


def test_go2_move_only_when_standing_and_expires():
    s = Go2Sim()
    s.move(0.5, 0, 0); run(s, 1.0)
    assert abs(s.x) < 1e-6                         # 趴着发 Move 没用
    s.stand_up(); run(s, 2.5)
    for _ in range(10):                            # 持续发 2 s
        s.move(0.5, 0, 0); run(s, 0.2)
    x1 = s.x
    assert 0.7 < x1 < 1.1
    run(s, 3.0)                                    # 不再发 -> 1 s 后停
    assert s.x - x1 < 0.6 and abs(s.vx) < 0.01


def test_go2_stops_at_wall():
    s = Go2Sim(); s.stand_up(); run(s, 2.5)
    for _ in range(150):
        s.move(1.0, 0, 0); run(s, 0.2)
    assert s.x < 6.5 - 0.29


def test_go2_low_level_requires_release_and_bad_gains_oscillate():
    s = Go2Sim()
    z = np.zeros(12)
    assert s.set_low_cmd(R.GO2_POSE_STAND, z, z + 60, z + 5, z) is False
    s.released = True

    def overshoot(kd):
        sim = Go2Sim(); sim.released = True
        peak = 0.0
        for _ in range(600):
            sim.set_low_cmd(R.GO2_POSE_STAND, z, z + 60, z + kd, z)
            sim.step(0.005)
            peak = max(peak, R.GO2_POSE_STAND[1] - sim.joints.q[1])     # 大腿关节从 1.36 降到 0.67，低于目标即超调
        return peak, abs(sim.joints.q[1] - R.GO2_POSE_STAND[1])
    over_good, err_good = overshoot(5.0)
    over_bad, _ = overshoot(0.0)
    assert err_good < 0.02 and over_good < 0.02          # 过阻尼: 不超调
    assert over_bad > 0.2                                # kd=0: 明显超调


def test_go2_hot_motor_is_the_planted_one():
    s = Go2Sim(); s.stand_up()
    for _ in range(60):
        s.move(0.5, 0, 0.3); run(s, 0.5)
    assert int(np.argmax(s.temp)) == 11 and s.temp[11] - np.median(s.temp) > 1.0


def test_remote_script_is_parseable():
    s = Go2Sim(); run(s, 2.5)
    assert parse_remote(s.remote_bytes())["keys"] == ["A"]


def test_g1_fsm_and_velocity():
    s = G1Sim(); run(s, 0.5)
    s.set_velocity(0.3, 0, 0, 1.0); run(s, 1.0)
    assert abs(s.x) < 1e-6                         # 阻尼状态不走
    assert s.set_fsm_id(R.G1_FSM["squat2standup"]) == 0; run(s, 4.0)
    assert s.posture == "stand" and s.height == pytest.approx(0.79, abs=0.02)
    assert s.set_fsm_id(R.G1_FSM["start"]) == 0
    s.set_velocity(0.3, 0, 0, 2.0); run(s, 2.0)
    assert 0.3 < s.x < 0.7
    assert s.set_fsm_id(12345) != 0


def test_g1_arm_action_and_release():
    s = G1Sim(); s.set_fsm_id(R.G1_FSM["start"]); run(s, 2.0)
    sp = R.G1_JOINT_ID["right_shoulder_pitch"]
    q0 = s.joints.q[sp]
    assert s.arm_action(18) == 0; run(s, 2.0)      # high five: 保持
    assert s.joints.q[sp] < q0 - 1.0
    run(s, 2.0); assert s.joints.q[sp] < q0 - 1.0
    assert s.arm_action(99) == 0; run(s, 2.5)      # release arm
    assert s.joints.q[sp] == pytest.approx(q0, abs=0.15)
    assert s.arm_action(12345) != 0


def test_g1_arm_sdk_weight_blends():
    s = G1Sim(); s.set_fsm_id(R.G1_FSM["start"]); run(s, 2.0)
    el = R.G1_JOINT_ID["left_elbow"]
    base = s.joints.q[el]
    q = np.zeros(29); q[el] = base + 1.0
    for w, expect in ((0.0, base), (0.5, base + 0.5), (1.0, base + 1.0)):
        for _ in range(400):
            s.set_arm_sdk(q, np.full(29, 60.0), np.full(29, 1.5), w); s.step(0.005)
        assert s.joints.q[el] == pytest.approx(expect, abs=0.12)


def test_go2_low_level_load_causes_steady_state_error_without_feedforward():
    z = np.zeros(12)

    def calf_error(tau_ff):
        sim = Go2Sim(); sim.released = True
        for _ in range(800):
            sim.set_low_cmd(R.GO2_POSE_STAND, z, z + 60, z + 5, z + tau_ff); sim.step(0.005)
        return abs(sim.joints.q[2] - R.GO2_POSE_STAND[2])
    ff = np.zeros(12); ff[2::3] = 6.0
    assert calf_error(0.0) == pytest.approx(6.0 / 60, abs=0.02)     # 误差 = 负载 / kp
    assert calf_error(ff) < 0.01                                     # 加上力矩前馈就消掉了
