"""【维护者工具，不属于课程内容】在回环网卡 lo 上扮演一台 Go2 / G1，用来录制 data/ 里的合成样本。

    python3 -m tools.datagen.mock_robot --robot go2

它用**官方 unitree_sdk2py** 的通道和 RPC Server 收发，topic 名、消息类型、API 号全部
与真机一致 (见 common/robots.py 的出处)，所以对着它写的代码改个网卡名就能连真机。
数据本身是 common/sim.py 合成的，**不是真机数据**。

只绑定 lo，永远碰不到真机。
"""
from __future__ import annotations

import argparse
import json
import os
import signal
import sys
import time

import numpy as np

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl import default as D
from unitree_sdk2py.idl.builtin_interfaces.msg.dds_ import Time_
from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_, PointField_
from unitree_sdk2py.idl.std_msgs.msg.dds_ import Header_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_ as GoLowCmd, LowState_ as GoLowState
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_ as HgLowCmd, LowState_ as HgLowState
from unitree_sdk2py.rpc.server import Server
from unitree_sdk2py.utils.crc import CRC

from unitree_sdk2py.comm.motion_switcher import motion_switcher_api as MS
from unitree_sdk2py.g1.arm import g1_arm_action_api as ARM
from unitree_sdk2py.g1.loco import g1_loco_api as LOCO
from unitree_sdk2py.go2.sport import sport_api as SPORT

from common import robots as R
from common.connect import init_sdk
from .sim import G1_ARM_ACTION_IDS, make_sim

ERR_NOT_IMPL = 3203      # RPC_ERR_SERVER_API_NOT_IMPL
ERR_PARAM = 3204         # RPC_ERR_SERVER_API_PARAMETER


def _stamp():
    now = time.time()
    return int(now), int((now % 1) * 1e9)


class _Rpc(Server):
    """把 {api_id: 函数(dict) -> (code, 数据)} 挂到官方 RPC Server 上。"""

    def __init__(self, name, version, table, log):
        super().__init__(name)
        self._SetApiVersion(version)
        for api_id, fn in table.items():
            self._RegistHandler(api_id, self._wrap(api_id, fn, log), False)

    @staticmethod
    def _wrap(api_id, fn, log):
        def handler(parameter: str):
            try:
                p = json.loads(parameter) if parameter else {}
            except json.JSONDecodeError:
                return ERR_PARAM, ""
            out = fn(p)
            code, data = out if isinstance(out, tuple) else (out, "")
            log(f"rpc api={api_id} param={parameter} -> code={code}")
            return code, data if isinstance(data, str) else json.dumps(data)
        return handler


class MockRobot:
    def __init__(self, robot: str, rate: float, lidar: bool = True, seed: int = 0):
        self.robot, self.rate, self.dt = robot, rate, 1.0 / rate
        self.sim = make_sim(robot, seed)
        self.crc = CRC()
        self.bad_crc = 0
        self.ignored_cmd = 0
        self.lidar = lidar
        self.mode_name = "normal"          # motion_switcher 当前模式名 (合成值；真机上的名字未核实)

        go = robot == "go2"
        self.pub_low = ChannelPublisher("rt/lowstate", GoLowState if go else HgLowState)
        self.pub_low_lf = ChannelPublisher("rt/lf/lowstate", GoLowState if go else HgLowState)
        # G1 的里程计 topic 在源码中未核实 (见 robots.TOPICS['g1_odom'])
        self.pub_sport = ChannelPublisher("rt/sportmodestate" if go else "rt/odommodestate", SportModeState_)
        self.pub_sport_lf = ChannelPublisher("rt/lf/sportmodestate" if go else "rt/lf/odommodestate", SportModeState_)
        self.pub_cloud = ChannelPublisher("rt/utlidar/cloud", PointCloud2_)
        for p in (self.pub_low, self.pub_low_lf, self.pub_sport, self.pub_sport_lf, self.pub_cloud):
            p.Init()

        self.sub_cmd = ChannelSubscriber("rt/lowcmd", GoLowCmd if go else HgLowCmd)
        self.sub_cmd.Init(self._on_lowcmd, 10)
        if not go:
            self.sub_arm = ChannelSubscriber("rt/arm_sdk", HgLowCmd)
            self.sub_arm.Init(self._on_arm_sdk, 10)

        s = self.sim
        self.servers = [_Rpc(MS.MOTION_SWITCHER_SERVICE_NAME, MS.MOTION_SWITCHER_API_VERSION, {
            MS.MOTION_SWITCHER_API_ID_CHECK_MODE: lambda p: (0, {"form": "0", "name": "" if s.released else self.mode_name}),
            MS.MOTION_SWITCHER_API_ID_SELECT_MODE: self._select_mode,
            MS.MOTION_SWITCHER_API_ID_RELEASE_MODE: self._release_mode,
        }, self.log)]
        if go:
            self.servers.append(_Rpc(SPORT.SPORT_SERVICE_NAME, SPORT.SPORT_API_VERSION, {
                SPORT.SPORT_API_ID_DAMP: lambda p: s.damp(),
                SPORT.SPORT_API_ID_BALANCESTAND: lambda p: s.balance_stand(),
                SPORT.SPORT_API_ID_STOPMOVE: lambda p: s.stop_move(),
                SPORT.SPORT_API_ID_STANDUP: lambda p: s.stand_up(),
                SPORT.SPORT_API_ID_STANDDOWN: lambda p: s.stand_down(),
                SPORT.SPORT_API_ID_RECOVERYSTAND: lambda p: s.recovery_stand(),
                SPORT.SPORT_API_ID_EULER: lambda p: s.euler(p["x"], p["y"], p["z"]),
                SPORT.SPORT_API_ID_MOVE: lambda p: s.move(p["x"], p["y"], p["z"]),
                SPORT.SPORT_API_ID_SIT: lambda p: s.sit(),
                SPORT.SPORT_API_ID_RISESIT: lambda p: s.balance_stand(),
                SPORT.SPORT_API_ID_HELLO: lambda p: 0,
                SPORT.SPORT_API_ID_STRETCH: lambda p: 0,
            }, self.log))
        else:
            self.servers.append(_Rpc(LOCO.LOCO_SERVICE_NAME, LOCO.LOCO_API_VERSION, {
                LOCO.ROBOT_API_ID_LOCO_GET_FSM_ID: lambda p: (0, {"data": s.fsm_id}),
                LOCO.ROBOT_API_ID_LOCO_GET_FSM_MODE: lambda p: (0, {"data": 0}),
                LOCO.ROBOT_API_ID_LOCO_GET_BALANCE_MODE: lambda p: (0, {"data": s.balance_mode}),
                LOCO.ROBOT_API_ID_LOCO_GET_STAND_HEIGHT: lambda p: (0, {"data": s.stand_height_cmd}),
                LOCO.ROBOT_API_ID_LOCO_SET_FSM_ID: lambda p: s.set_fsm_id(p["data"]),
                LOCO.ROBOT_API_ID_LOCO_SET_BALANCE_MODE: lambda p: setattr(s, "balance_mode", p["data"]) or 0,
                LOCO.ROBOT_API_ID_LOCO_SET_STAND_HEIGHT: lambda p: setattr(s, "stand_height_cmd", p["data"]) or 0,
                LOCO.ROBOT_API_ID_LOCO_SET_VELOCITY: self._g1_velocity,
                LOCO.ROBOT_API_ID_LOCO_SET_ARM_TASK: lambda p: s.arm_task(p["data"]),
            }, self.log))
            self.servers.append(_Rpc(ARM.ARM_ACTION_SERVICE_NAME, ARM.ARM_ACTION_API_VERSION, {
                ARM.ROBOT_API_ID_ARM_ACTION_EXECUTE_ACTION: lambda p: s.arm_action(p["data"]),
                ARM.ROBOT_API_ID_ARM_ACTION_GET_ACTION_LIST: lambda p: (0, [{"id": i, "name": n} for n, i in G1_ARM_ACTION_IDS.items()]),
            }, self.log))
        for srv in self.servers:
            srv.Start()

    # ------------------------------------------------------------------ 日志
    def log(self, text):
        print(f"[mock {self.sim.t:7.2f}s] {text}", flush=True)

    # ------------------------------------------------------------------ RPC
    def _release_mode(self, p):
        self.sim.released = True
        self.sim.user_cmd = None
        return 0

    def _select_mode(self, p):
        self.mode_name = p.get("name", "normal") or "normal"
        self.sim.released = False
        # 重新接管: 从当前关节角平滑回到高层目标
        if self.sim.q_goal is not None:
            self.sim._go_to(self.sim.q_goal, 1.5)
        return 0

    def _g1_velocity(self, p):
        v = p["velocity"]
        self.sim.set_velocity(float(np.clip(v[0], -1.0, 1.0)), float(np.clip(v[1], -0.5, 0.5)),
                              float(np.clip(v[2], -1.5, 1.5)), float(p.get("duration", 1.0)))
        return 0

    # ------------------------------------------------------------------ 低层指令
    def _crc_ok(self, msg) -> bool:
        if self.crc.Crc(msg) == msg.crc:
            return True
        self.bad_crc += 1
        if self.bad_crc in (1, 100) or self.bad_crc % 1000 == 0:
            self.log(f"丢弃 CRC 错误的指令 (累计 {self.bad_crc} 条)")
        return False

    def _ignored(self, why):
        self.ignored_cmd += 1
        if self.ignored_cmd in (1, 500) or self.ignored_cmd % 5000 == 0:
            self.log(f"忽略 LowCmd: {why} (累计 {self.ignored_cmd} 条)")

    def _on_lowcmd(self, msg):
        if not self._crc_ok(msg):
            return
        n = self.sim.n
        mc = msg.motor_cmd
        if self.robot == "go2":
            if tuple(msg.head) != R.GO2_LOWCMD_HEAD or msg.level_flag != R.GO2_LEVEL_LOW:
                return self._ignored("head/level_flag 不对")
            q = np.array([mc[i].q for i in range(n)])
            kp = np.array([mc[i].kp for i in range(n)])
            q = np.where(np.abs(q) > 1e6, self.sim.joints.q, q)        # PosStopF: 不控位置
            kp = np.where(np.abs(np.array([mc[i].q for i in range(n)])) > 1e6, 0.0, kp)
            dq = np.array([0.0 if abs(mc[i].dq) > 1e3 else mc[i].dq for i in range(n)])
            enable = np.array([mc[i].mode == 0x01 for i in range(n)], float)
        else:
            if msg.mode_machine != self.sim.MODE_MACHINE:
                return self._ignored(f"mode_machine={msg.mode_machine} 与 LowState 里的 {self.sim.MODE_MACHINE} 不一致")
            self.sim.mode_pr = msg.mode_pr
            q = np.array([mc[i].q for i in range(n)])
            kp = np.array([mc[i].kp for i in range(n)])
            dq = np.array([mc[i].dq for i in range(n)])
            enable = np.array([mc[i].mode == 1 for i in range(n)], float)
        kd = np.array([mc[i].kd for i in range(n)])
        tau = np.array([mc[i].tau for i in range(n)])
        if not self.sim.set_low_cmd(q, dq, kp * enable, kd * enable, tau * enable):
            self._ignored("高层运控还没释放 (先 MotionSwitcherClient.ReleaseMode)")

    def _on_arm_sdk(self, msg):
        if not self._crc_ok(msg):
            return
        mc = msg.motor_cmd
        self.sim.set_arm_sdk([mc[i].q for i in range(29)], [mc[i].kp for i in range(29)],
                             [mc[i].kd for i in range(29)], mc[R.G1_ARM_SDK_WEIGHT_ID].q)

    # ------------------------------------------------------------------ 状态发布
    def _fill_imu(self, imu):
        s = self.sim
        imu.quaternion = [float(v) for v in s.quat]
        imu.gyroscope = [float(v) for v in s.gyro]
        imu.accelerometer = [float(v) for v in s.accel]
        imu.rpy = [float(v) for v in s.rpy]
        imu.temperature = 41

    def _lowstate(self):
        s, j = self.sim, self.sim.joints
        if self.robot == "go2":
            m = D.unitree_go_msg_dds__LowState_()
            m.head = [0xFE, 0xEF]
            m.level_flag = 0xFF
            for i in range(12):
                ms = m.motor_state[i]
                ms.mode = 0x01
                ms.q, ms.dq, ms.ddq, ms.tau_est = float(j.q[i]), float(j.dq[i]), float(j.ddq[i]), float(j.tau[i])
                ms.q_raw, ms.dq_raw = ms.q, ms.dq
                ms.temperature = int(round(s.temp[i]))
            m.foot_force = [int(v) for v in s.foot_force]
            m.foot_force_est = [int(v) for v in s.foot_force]
            m.power_v, m.power_a = float(s.power_v), float(s.power_a)
            m.bms_state.soc = int(s.soc)
            m.bms_state.current = int(-s.power_a * 1000)
            m.bms_state.cycle = 37
            m.bms_state.cell_vol = [int(s.power_v / 8 * 1000)] * 8 + [0] * 7
            m.bms_state.bq_ntc = [31, 31]
            m.bms_state.mcu_ntc = [33, 33]
            m.temperature_ntc1 = 38
        else:
            m = D.unitree_hg_msg_dds__LowState_()
            m.mode_pr = s.mode_pr
            m.mode_machine = s.MODE_MACHINE
            for i in range(29):
                ms = m.motor_state[i]
                ms.mode = 1
                ms.q, ms.dq, ms.ddq, ms.tau_est = float(j.q[i]), float(j.dq[i]), float(j.ddq[i]), float(j.tau[i])
                ms.temperature = [int(round(s.temp[i])), int(round(s.temp[i])) - 2]
                ms.vol = 53.5
        self._fill_imu(m.imu_state)
        m.tick = int(s.t * 1000)
        m.wireless_remote = list(s.remote_bytes())
        m.crc = self.crc.Crc(m)
        return m

    def _sportstate(self):
        s = self.sim
        m = D.unitree_go_msg_dds__SportModeState_()
        m.stamp.sec, m.stamp.nanosec = _stamp()
        self._fill_imu(m.imu_state)
        m.position = [float(s.x), float(s.y), float(s.height)]
        m.body_height = float(s.height)
        c, sn = np.cos(s.yaw), np.sin(s.yaw)
        m.velocity = [float(c * s.vx - sn * s.vy), float(sn * s.vx + c * s.vy), 0.0]
        m.yaw_speed = float(s.wz)
        if self.robot == "go2":
            m.foot_force = [int(v) for v in s.foot_force]
        return m

    def _cloud(self):
        pts = self.sim.lidar_points()
        sec, nsec = _stamp()
        fields = [PointField_(name, off, 7, 1) for name, off in (("x", 0), ("y", 4), ("z", 8), ("intensity", 12))]
        return PointCloud2_(header=Header_(Time_(sec, nsec), "utlidar_lidar"), height=1, width=len(pts),
                            fields=fields, is_bigendian=False, point_step=16, row_step=16 * len(pts),
                            data=pts.tobytes(), is_dense=True)

    # ------------------------------------------------------------------ 主循环
    def run(self, duration: float | None = None):
        self.log(f"READY robot={self.robot} rate={self.rate:.0f}Hz iface=lo  (合成数据, 非真机)")
        lf_every = max(1, round(self.rate / 20))
        sport_every = max(1, round(self.rate / 50))
        cloud_every = max(1, round(self.rate / 5))
        k = 0
        next_t = time.perf_counter()
        last = next_t
        while duration is None or self.sim.t < duration:
            # 用真实流逝的时间推进仿真: 即使电脑慢、达不到标称频率，仿真时间也和墙上时钟一致
            now = time.perf_counter()
            self.sim.step(min(max(now - last, 0.5 * self.dt), 0.02))
            last = now
            low = self._lowstate()
            self.pub_low.Write(low)
            if k % lf_every == 0:
                self.pub_low_lf.Write(low)
            if k % sport_every == 0:
                sp = self._sportstate()
                self.pub_sport.Write(sp)
                if k % lf_every == 0:
                    self.pub_sport_lf.Write(sp)
            if self.lidar and k % cloud_every == 0:
                self.pub_cloud.Write(self._cloud())
            k += 1
            next_t += self.dt
            delay = next_t - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
            elif delay < -0.2:
                next_t = time.perf_counter()        # 落后太多就不追了


def main(argv=None):
    ap = argparse.ArgumentParser(description="模拟一台 Go2 / G1 (仅回环网卡, 合成数据)")
    ap.add_argument("--robot", choices=["go2", "g1"], default="go2")
    ap.add_argument("--rate", type=float, default=200.0,
                    help="lowstate 发布频率 Hz (真机为 500 Hz 量级, 未核实; 模拟默认 200 以省 CPU)")
    ap.add_argument("--duration", type=float, default=None, help="运行多少秒后退出 (默认一直跑)")
    ap.add_argument("--no-lidar", action="store_true")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args(argv)

    def _stop(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _stop)
    init_sdk("lo")
    bot = MockRobot(args.robot, args.rate, lidar=not args.no_lidar, seed=args.seed)
    try:
        bot.run(args.duration)
    except KeyboardInterrupt:
        pass
    # 干净地退出 DDS: 删掉参与者，别的进程会立刻知道 "这台机器人下线了"，
    # 而不是等十秒的租约超时 (否则紧接着启动的下一个模拟机器人会和它的"幽灵"并存)
    try:
        from cyclonedds.domain import DomainParticipant
        from unitree_sdk2py.core.channel import ChannelFactory
        part = getattr(ChannelFactory, "_ChannelFactory__participant", None)
        if isinstance(part, DomainParticipant):
            part.delete()
    except Exception:
        pass
    sys.stdout.flush()
    os._exit(0)


if __name__ == "__main__":
    main()
