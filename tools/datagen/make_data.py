#!/usr/bin/env python3
"""【维护者工具】重新生成 data/ 里的全部样本。全部是**合成数据**。

    source scripts/env.sh && python3 -m tools.datagen.make_data

做法: 在回环网卡上起一个简化的机器人替身 (tools/datagen/mock_robot.py)，用**官方 SDK 的真实调用**
(SportClient / LocoClient / G1ArmActionClient / rt/arm_sdk) 驱动它，同时用 `ros2 bag record` 录下来。
所以 bag 里的 topic 名、消息类型、RPC 请求内容与真机一致，只有数值是合成的。
"""
import json
import shutil
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

from common import robots as R
from common.connect import DATA, Background, init_sdk
from common.joints import interpolate
from tools.datagen.sim import ROOM_WALLS, G1Sim, Go2Sim

API = "/api/sport/request /api/sport/response /api/motion_switcher/request /api/motion_switcher/response"


class Session:
    """起替身 + 开始录包；退出时停掉两者。"""

    def __init__(self, robot, bag, topics):
        self.path = DATA / "bags" / bag
        shutil.rmtree(self.path, ignore_errors=True)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.mock = Background(f"exec {sys.executable} -u -m tools.datagen.mock_robot --robot {robot}")
        assert self.mock.wait_for("READY", 40), self.mock.output(20)
        time.sleep(0.5)
        self.rec = Background(f"exec ros2 bag record -o {self.path} {topics}")
        assert self.rec.wait_for("Recording", 20), self.rec.output(20)
        time.sleep(1.5)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.rec.stop()
        self.mock.stop()
        time.sleep(1.5)
        size = sum(f.stat().st_size for f in self.path.iterdir()) / 1e6
        print(f"  {self.path.name}: {size:.1f} MB")


def hold(fn, seconds, period=0.5):
    t_end = time.monotonic() + seconds
    while time.monotonic() < t_end:
        fn()
        time.sleep(min(period, max(0.0, t_end - time.monotonic())))


def go2_demo():
    from unitree_sdk2py.go2.sport.sport_client import SportClient
    with Session("go2", "go2_demo", f"/lowstate /sportmodestate /utlidar/cloud {API}"):
        c = SportClient(); c.SetTimeout(3.0); c.Init()
        assert c.StandUp() == 0; time.sleep(2.5)
        hold(lambda: c.Move(0.4, 0.0, 0.0), 4.0)
        hold(lambda: c.Move(0.0, 0.0, 0.8), 2.0)
        hold(lambda: c.Move(0.4, 0.0, 0.0), 2.0)
        c.StopMove(); time.sleep(1.5)


def go2_slam():
    from unitree_sdk2py.go2.sport.sport_client import SportClient
    with Session("go2", "go2_slam", "/sportmodestate /utlidar/cloud /lf/lowstate"):
        c = SportClient(); c.SetTimeout(3.0); c.Init()
        assert c.StandUp() == 0; time.sleep(2.5)
        turn = (np.pi / 2) / 0.8 + 0.25           # 0.25 s 补偿一阶响应
        for dist in (5.0, 2.6, 5.0, 2.6):
            hold(lambda: c.Move(0.5, 0.0, 0.0), dist / 0.5, 0.3)
            time.sleep(0.6)
            hold(lambda: c.Move(0.0, 0.0, 0.8), turn, 0.3)
            time.sleep(0.6)
        c.StopMove(); time.sleep(1.0)


def g1_demo():
    from unitree_sdk2py.core.channel import ChannelPublisher
    from unitree_sdk2py.g1.arm.g1_arm_action_client import G1ArmActionClient, action_map
    from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
    from unitree_sdk2py.utils.crc import CRC
    topics = f"/lowstate /odommodestate /utlidar/cloud /arm_sdk /api/arm/request /api/arm/response {API}"
    with Session("g1", "g1_demo", topics):
        c = LocoClient(); c.SetTimeout(3.0); c.Init()
        a = G1ArmActionClient(); a.SetTimeout(3.0); a.Init()
        print("  fsm id:", c.GetFsmId())
        c.Squat2StandUp(); time.sleep(3.0)
        c.Start(); time.sleep(1.0)
        c.SetVelocity(0.3, 0.0, 0.0, 3.0); time.sleep(3.5)
        assert a.ExecuteAction(action_map["high wave"]) == 0; time.sleep(4.0)
        # rt/arm_sdk: 抬左臂 -> 保持 -> 放回 -> 交还 (权重 1 -> 0)
        pub = ChannelPublisher("rt/arm_sdk", LowCmd_); pub.Init()
        crc = CRC()
        cmd = unitree_hg_msg_dds__LowCmd_()
        ids = R.G1_ARM_IDS + R.G1_WAIST_IDS
        rest = np.zeros(29); rest[R.G1_JOINT_ID["left_shoulder_roll"]] = 0.2
        rest[R.G1_JOINT_ID["right_shoulder_roll"]] = -0.2
        rest[R.G1_JOINT_ID["left_elbow"]] = rest[R.G1_JOINT_ID["right_elbow"]] = 0.6
        goal = rest.copy()
        goal[R.G1_JOINT_ID["left_shoulder_pitch"]] = -1.2
        goal[R.G1_JOINT_ID["left_elbow"]] = 0.2
        dt, t = 0.02, 0.0
        while t < 6.0:
            if t < 2.0:
                q, w = interpolate(rest, goal, t, 2.0), min(1.0, t / 0.5)
            elif t < 3.0:
                q, w = goal, 1.0
            elif t < 5.0:
                q, w = interpolate(goal, rest, t - 3.0, 2.0), 1.0
            else:
                q, w = rest, 1.0 - (t - 5.0)
            for i in ids:
                m = cmd.motor_cmd[i]
                m.q, m.dq, m.tau, m.kp, m.kd = float(q[i]), 0.0, 0.0, 60.0, 1.5
            cmd.motor_cmd[R.G1_ARM_SDK_WEIGHT_ID].q = float(w)
            cmd.crc = crc.Crc(cmd)
            pub.Write(cmd)
            time.sleep(dt); t += dt
        time.sleep(1.0)


def to_npz(bag, robot):
    from common.bagio import bag_to_arrays
    d = bag_to_arrays(DATA / "bags" / bag, robot)
    np.savez_compressed(DATA / f"{bag}.npz", **d)
    print(f"  {bag}.npz: {len(d['t'])} lowstate, {d['t'][-1]:.1f} s")


# ----------------------------------------------------------------- 纯仿真的样本 (不经过 DDS)
def long_run():
    """5 分钟 Go2 巡逻, 50 Hz 采样 —— 07 节分析温度 / 电池 / 足端力。"""
    s = Go2Sim(seed=1)
    dt, keep = 0.005, 4
    rows = {k: [] for k in ("t", "rpy", "q", "tau", "temp", "foot_force", "power_v", "power_a", "soc", "position")}
    s.stand_up()
    for k in range(int(300 / dt)):
        t = k * dt
        if t > 3 and k % 100 == 0:
            phase = int((t - 3) // 10) % 4               # 走 10 s / 转 10 s / 走 10 s / 站着歇 10 s
            if phase in (0, 2):
                s.move(0.5, 0.0, 0.0)
            elif phase == 1:
                s.move(0.0, 0.0, 0.9)
        s.step(dt)
        if k % keep == 0:
            for name, v in (("t", s.t), ("rpy", s.rpy.copy()), ("q", s.joints.q.copy()), ("tau", s.joints.tau.copy()),
                            ("temp", np.round(s.temp)), ("foot_force", np.round(s.foot_force)), ("power_v", s.power_v),
                            ("power_a", s.power_a), ("soc", int(s.soc)), ("position", (s.x, s.y, s.height))):
                rows[name].append(v)
    np.savez_compressed(DATA / "go2_long_run.npz", **{k: np.asarray(v, dtype=np.float32) for k, v in rows.items()})
    print("  go2_long_run.npz")


def lowlevel_runs():
    """低层关节控制的几组对照实验, 500 Hz —— 11 节用。

    每组: 同一条目标轨迹，换一种增益 / 前馈 / 插值方式，记录 q_des、q、tau。
    """
    dt = 0.002
    out = {}

    def run(sim, n, plan, kp, kd, tau_ff, seconds):
        sim.released = True
        log = {"t": [], "q_des": [], "q": [], "tau": []}
        for k in range(int(seconds / dt)):
            t = k * dt
            q_des = plan(t)
            sim.set_low_cmd(q_des, np.zeros(n), kp, kd, tau_ff)
            sim.step(dt)
            for key, v in (("t", t), ("q_des", q_des), ("q", sim.joints.q.copy()), ("tau", sim.joints.tau.copy())):
                log[key].append(v)
        return {k: np.asarray(v, dtype=np.float32) for k, v in log.items()}

    # Go2: 趴 -> 站 (官方 go2_stand_example 的动作)，kp=60 kd=5
    lie, stand = np.array(R.GO2_POSE_LIE), np.array(R.GO2_POSE_STAND)
    ff = np.zeros(12); ff[2::3] = 6.0
    smooth = lambda t: interpolate(lie, stand, t - 0.5, 2.0, "cosine")       # noqa: E731
    step = lambda t: lie if t < 0.5 else stand                               # noqa: E731
    z12 = np.zeros(12)
    for name, plan, kd, tff in (("good", smooth, 5.0, ff), ("kd0", smooth, 0.0, ff),
                                ("no_ff", smooth, 5.0, z12), ("step", step, 5.0, ff)):
        r = run(Go2Sim(), 12, plan, z12 + 60.0, z12 + kd, tff, 5.0)
        out.update({f"go2_{name}_{k}": v for k, v in r.items()})

    # G1: 悬挂状态下抬左臂 (肩 pitch 0 -> -1.2, 肘 0.6 -> 0.2)，增益用官方示例的 Kp/Kd
    kp, kd = np.array(R.G1_KP, float), np.array(R.G1_KD, float)
    z29 = np.zeros(29)

    def make_g1():
        s = G1Sim(); s.joints.q[:] = 0.0; s.joints.q[R.G1_JOINT_ID["left_elbow"]] = 0.6
        return s
    start = make_g1().joints.q.copy()
    goal = start.copy(); goal[R.G1_JOINT_ID["left_shoulder_pitch"]] = -1.2; goal[R.G1_JOINT_ID["left_elbow"]] = 0.2
    smooth = lambda t: interpolate(start, goal, t - 0.5, 2.0, "cosine")      # noqa: E731
    step = lambda t: start if t < 0.5 else goal                              # noqa: E731
    for name, plan, kd_i in (("smooth_kd", smooth, kd), ("smooth_kd0", smooth, z29),
                             ("step_kd", step, kd), ("step_kd0", step, z29)):
        r = run(make_g1(), 29, plan, kp, kd_i, z29, 4.0)
        out.update({f"g1_{name}_{k}": v for k, v in r.items()})
    np.savez_compressed(DATA / "lowlevel_runs.npz", **out)
    print("  lowlevel_runs.npz", sorted({k.rsplit('_', 1)[0] for k in out if k.endswith('_t')}))


def main():
    DATA.mkdir(exist_ok=True)
    (DATA / "room.json").write_text(json.dumps(
        {"note": "合成数据所在的虚拟房间: 墙面线段 [x1, y1, x2, y2] (m)，机器人从 (0,0) 朝 +x 出发",
         "walls": [list(w) for w in ROOM_WALLS]}, ensure_ascii=False, indent=1))
    if "--sim-only" not in sys.argv:
        init_sdk("lo")
        print("录制 bag ..."); go2_demo(); g1_demo(); go2_slam()
        print("转 npz ..."); to_npz("go2_demo", "go2"); to_npz("g1_demo", "g1")
    print("纯仿真样本 ..."); long_run(); lowlevel_runs()
    subprocess.run(["du", "-sh", str(DATA)])


if __name__ == "__main__":
    main()
