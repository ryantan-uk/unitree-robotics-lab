"""【维护者工具，不属于课程内容】合成样本数据用的简化物理内核 (纯 NumPy)。

!! 这是**合成数据**的来源，不是 Unitree 的动力学模型 !!
它只保证三件事，够教学用:
  1. 消息的字段、长度、单位量级与真机消息一致 (字段定义出自官方 .msg / IDL)
  2. 因果关系合理: 发 StandUp 关节会动、发 Move 位置会变、增益改坏会振荡、力矩大了会升温
  3. 完全确定 (固定随机种子)，同样的指令得到同样的曲线

真机上的数值 (足端力读数、温升速度、电池曲线……) 一定和这里不同。
"""
from __future__ import annotations

import math

import numpy as np

from common import robots as R
from common.parsing import pack_remote, rot_from_rpy, rpy_to_quat

G = 9.81

# 虚拟房间: 线段列表 (x1, y1, x2, y2)，单位 m，机器人出生在 (0, 0) 朝 +x
ROOM_WALLS = [
    (-1.5, -2.5, 6.5, -2.5), (6.5, -2.5, 6.5, 3.5), (6.5, 3.5, -1.5, 3.5), (-1.5, 3.5, -1.5, -2.5),   # 外墙 8m x 6m
    (2.0, -2.5, 2.0, -0.6),                                                                            # 隔断墙
    (3.6, 1.2, 4.6, 1.2), (4.6, 1.2, 4.6, 2.2), (4.6, 2.2, 3.6, 2.2), (3.6, 2.2, 3.6, 1.2),           # 箱子
]


def raycast(x, y, yaw, angles, walls=ROOM_WALLS, max_range=12.0):
    """从 (x, y) 沿 yaw+angles 发射射线，返回到最近墙面的距离数组。"""
    a = yaw + np.asarray(angles)
    dx, dy = np.cos(a), np.sin(a)
    best = np.full(a.shape, max_range)
    for x1, y1, x2, y2 in walls:
        ex, ey = x2 - x1, y2 - y1
        den = dx * ey - dy * ex
        ok = np.abs(den) > 1e-9
        den_s = np.where(ok, den, 1.0)
        t = ((x1 - x) * ey - (y1 - y) * ex) / den_s      # 沿射线的距离
        u = ((x1 - x) * dy - (y1 - y) * dx) / den_s      # 线段上的参数
        hit = ok & (t > 0.02) & (u >= 0) & (u <= 1)
        best = np.where(hit & (t < best), t, best)
    return best


def min_wall_distance(x, y, walls=ROOM_WALLS):
    d = math.inf
    for x1, y1, x2, y2 in walls:
        ex, ey = x2 - x1, y2 - y1
        s = max(0.0, min(1.0, ((x - x1) * ex + (y - y1) * ey) / (ex * ex + ey * ey)))
        d = min(d, math.hypot(x - (x1 + s * ex), y - (y1 + s * ey)))
    return d


class JointGroup:
    """n 个独立关节，每个是 "单位惯量 + PD + 力矩限幅"。

        tau = kp (q_des - q) + kd (dq_des - dq) + tau_ff      <- 与真机电机的控制律同形
        ddq = (tau - 负载 - 粘性摩擦) / 惯量                    <- 负载 = 撑住身体所需的力矩
    """

    def __init__(self, n, q0, inertia=0.05, friction=0.1, tau_max=45.0):
        self.n = n
        self.q = np.array(q0, dtype=float)
        self.dq = np.zeros(n)
        self.ddq = np.zeros(n)
        self.tau = np.zeros(n)
        self.inertia, self.friction, self.tau_max = inertia, friction, tau_max

    def step(self, dt, q_des, dq_des, kp, kd, tau_ff, load=0.0):
        tau = kp * (q_des - self.q) + kd * (dq_des - self.dq) + tau_ff
        self.tau = np.clip(tau, -self.tau_max, self.tau_max)
        self.ddq = (self.tau - load - self.friction * self.dq) / self.inertia
        self.dq = np.clip(self.dq + self.ddq * dt, -30.0, 30.0)
        self.q = self.q + self.dq * dt


class _Base:
    """Go2 / G1 共用: 底盘积分、IMU、温度、遥控器脚本、点云。"""
    robot = ""
    n = 0
    stand_height = 0.3
    internal_kp = 80.0
    internal_kd = 4.0

    def __init__(self, seed=0):
        self.rng = np.random.default_rng(seed)
        self.t = 0.0
        self.x = self.y = self.yaw = 0.0
        self.vx = self.vy = self.wz = 0.0                 # 机体系实际速度
        self.cmd_v = np.zeros(3)                          # 机体系指令速度
        self.cmd_until = -1.0                             # 速度指令的有效截止时间
        self.height = 0.0
        self.rpy = np.zeros(3)
        self.rpy_cmd = np.zeros(3)
        self.gyro = np.zeros(3)
        self.accel = np.array([0, 0, G])
        self.released = False                             # motion_switcher.ReleaseMode 之后为 True
        self.user_cmd = None                              # 低层指令 (q, dq, kp, kd, tau)
        self.user_cmd_time = -1.0
        self.temp = np.full(self.n, 32.0)
        self.q_goal = None
        self._traj = None                                 # (t0, T, q_start, q_goal)
        self.events: list[str] = []

    # ---- 高层: 关节目标轨迹 ------------------------------------------------
    def _go_to(self, q_goal, T):
        self._traj = (self.t, T, self.joints.q.copy(), np.array(q_goal, float))

    def _traj_target(self):
        if self._traj is None:
            return self.joints.q.copy() if self.q_goal is None else self.q_goal
        t0, T, qs, qg = self._traj
        s = min(1.0, (self.t - t0) / T)
        r = 0.5 - 0.5 * math.cos(math.pi * s)
        self.q_goal = qg
        return qs + r * (qg - qs)

    def can_move(self) -> bool:
        raise NotImplementedError

    def set_velocity(self, vx, vy, wz, duration=1.0):
        self.cmd_v = np.array([vx, vy, wz], float)
        self.cmd_until = self.t + duration

    # ---- 低层指令入口 ------------------------------------------------------
    def set_low_cmd(self, q, dq, kp, kd, tau):
        """只有 released (已释放高层运控) 时才生效，返回是否被接受。"""
        if not self.released:
            return False
        self.user_cmd = tuple(np.array(v, float) for v in (q, dq, kp, kd, tau))
        self.user_cmd_time = self.t
        return True

    # ---- 一步 --------------------------------------------------------------
    def step(self, dt):
        self.t += dt
        q_des, dq_des, kp, kd, tau_ff = self._control_targets(dt)
        self.joints.step(dt, q_des, dq_des, kp, kd, tau_ff, self._load())
        self._step_base(dt)
        self._step_imu(dt)
        # 温度: 力矩平方发热 + 向环境 (30 度) 散热。hot_motor 的发热系数被故意调大
        heat = self.heat_gain * self.joints.tau ** 2
        self.temp += dt * (heat - (self.temp - 30.0) / 60.0)

    def _control_targets(self, dt):
        n = self.n
        if self.released:
            # 低层模式: 0.5 s 没收到指令就掉电 (kp=kd=0，只剩摩擦)
            if self.user_cmd is not None and self.t - self.user_cmd_time < 0.5:
                return self.user_cmd
            return self.joints.q, np.zeros(n), np.zeros(n), np.full(n, 0.5), np.zeros(n)
        if self.passive():
            return self.joints.q, np.zeros(n), np.zeros(n), np.full(n, 2.0), np.zeros(n)
        q_des = self._traj_target() + self._gait_offset()
        q_des, kp, kd = self._overlay(q_des, np.full(n, self.internal_kp), np.full(n, self.internal_kd))
        return q_des, np.zeros(n), kp, kd, self._load()      # 内置运控自带负载前馈

    def passive(self) -> bool:
        return False

    def _overlay(self, q_des, kp, kd):
        return q_des, kp, kd

    def _gait_offset(self):
        return np.zeros(self.n)

    def _load(self):
        """撑住身体所需的关节力矩 (合成的简化模型)。低层控制时不加前馈就会有稳态误差。"""
        return np.zeros(self.n)

    def _step_base(self, dt):
        target = self.cmd_v if (self.t < self.cmd_until and self.can_move()) else np.zeros(3)
        v = np.array([self.vx, self.vy, self.wz])
        v += (target - v) * min(1.0, dt / 0.25)              # 0.25 s 的一阶响应
        self.vx, self.vy, self.wz = v
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        nx = self.x + (c * self.vx - s * self.vy) * dt
        ny = self.y + (s * self.vx + c * self.vy) * dt
        if min_wall_distance(nx, ny) > 0.30:                 # 撞墙就停住
            self.x, self.y = nx, ny
        else:
            self.vx = self.vy = 0.0
        self.yaw = (self.yaw + self.wz * dt + math.pi) % (2 * math.pi) - math.pi
        h_goal = self._height_goal()
        self.height += (h_goal - self.height) * min(1.0, dt / 0.3)

    def _height_goal(self):
        return self.stand_height

    def _step_imu(self, dt):
        speed = math.hypot(self.vx, self.vy)
        wob = 0.02 * min(1.0, speed / 0.3)
        ph = 2 * math.pi * self.gait_hz * self.t
        new_rpy = np.array([
            self.rpy_cmd[0] + wob * math.sin(ph),
            self.rpy_cmd[1] + 0.6 * wob * math.sin(2 * ph) - 0.03 * self.vx,
            self.yaw + self.rpy_cmd[2],
        ]) + self.rng.normal(0, 0.0015, 3)
        d = new_rpy - self.rpy
        d[2] = (d[2] + math.pi) % (2 * math.pi) - math.pi
        self.gyro = d / dt * 0.2 + self.gyro * 0.8           # 低通后的角速度
        self.rpy = new_rpy
        self.rpy[2] = (self.rpy[2] + math.pi) % (2 * math.pi) - math.pi
        # 加速度计测的是 "比力": 静止时读到 +g 沿机体 z
        Rwb = rot_from_rpy(*self.rpy)
        self.accel = Rwb.T @ np.array([0, 0, G]) + self.rng.normal(0, 0.05, 3)

    @property
    def quat(self):
        return rpy_to_quat(self.rpy)

    # ---- 遥控器: 固定剧本，方便 02 节解析 ------------------------------------
    def remote_bytes(self) -> bytes:
        t = self.t % 12.0
        keys = []
        if 2.0 <= t < 3.0:
            keys = ["A"]
        elif 5.0 <= t < 6.0:
            keys = ["L2", "B"]
        elif 8.0 <= t < 8.5:
            keys = ["start"]
        return pack_remote(lx=0.0, ly=round(0.6 * math.sin(2 * math.pi * t / 12.0), 3),
                           rx=round(0.4 * math.sin(2 * math.pi * t / 6.0), 3), ry=0.0, keys=keys)

    # ---- 激光点云 (机体雷达系 xyz + intensity) -------------------------------
    LIDAR_OFFSET = (0.28, 0.0, 0.05)       # 雷达在机体系的位置 (合成值，真机以 URDF/TF 为准)

    def lidar_points(self, n_az=360, elevations=(-0.05, 0.0, 0.05, 0.15)) -> np.ndarray:
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        lx = self.x + c * self.LIDAR_OFFSET[0]
        ly = self.y + s * self.LIDAR_OFFSET[0]
        az = np.linspace(-math.pi, math.pi, n_az, endpoint=False)
        rng = raycast(lx, ly, self.yaw, az) + self.rng.normal(0, 0.01, n_az)
        pts = []
        for el in elevations:
            valid = rng < 11.5
            x = rng * np.cos(az)
            y = rng * np.sin(az)
            z = rng * math.tan(el)
            inten = np.clip(1.0 - rng / 12.0, 0, 1) * 255.0
            pts.append(np.stack([x, y, z, inten], axis=1)[valid])
        return np.concatenate(pts).astype(np.float32)


# =========================================================================== Go2
class Go2Sim(_Base):
    robot = "go2"
    n = 12
    stand_height = 0.32
    gait_hz = 2.5
    internal_kp = 80.0
    internal_kd = 4.0

    def __init__(self, seed=0, hot_motor=11):
        self.joints = JointGroup(12, R.GO2_POSE_LIE, inertia=0.05, friction=0.1, tau_max=45.0)
        super().__init__(seed)
        self.mode = "lie"                 # lie / stand / damp / sit
        self.q_goal = np.array(R.GO2_POSE_LIE)
        self.heat_gain = np.full(12, 0.006)
        self.heat_gain[hot_motor] = 0.013                 # 故意埋的"异常电机": 右后小腿
        self.soc = 87.0
        self.power_v = 29.0
        self.power_a = 1.5
        self.foot_force = np.zeros(4)
        self.height = 0.07

    # ---- 高层指令 (对应 SportClient 的方法) ----------------------------------
    def stand_up(self):
        self.mode = "stand"; self._go_to(R.GO2_POSE_STAND, 1.5); return 0

    def stand_down(self):
        self.mode = "lie"; self.cmd_until = -1; self._go_to(R.GO2_POSE_LIE, 1.5); return 0

    def recovery_stand(self):
        return self.stand_up()

    def balance_stand(self):
        self.mode = "stand"; self._go_to(R.GO2_POSE_STAND, 0.8); return 0

    def damp(self):
        self.mode = "damp"; self.cmd_until = -1; self._traj = None; return 0

    def sit(self):
        self.mode = "sit"; self.cmd_until = -1
        self._go_to([0.0, 0.67, -1.3] * 2 + [0.0, 2.0, -2.5] * 2, 1.5); return 0

    def stop_move(self):
        self.cmd_until = -1; return 0

    def euler(self, roll, pitch, yaw):
        self.rpy_cmd = np.clip([roll, pitch, yaw], -0.6, 0.6); return 0

    def move(self, vx, vy, wz):
        # 假设: 一条 Move 的有效期 1 s，要持续走就得持续发 (真机的确切超时未核实)
        self.set_velocity(np.clip(vx, -1.5, 1.5), np.clip(vy, -0.6, 0.6), np.clip(wz, -2.0, 2.0), 1.0)
        return 0

    def can_move(self):
        return self.mode == "stand" and not self.released

    def passive(self):
        return self.mode == "damp"

    def _height_goal(self):
        if self.released:
            # 低层模式下高度由大腿/小腿角度粗略决定
            thigh, calf = self.joints.q[1::3].mean(), self.joints.q[2::3].mean()
            return max(0.06, 0.213 * (math.cos(thigh) + math.cos(thigh + calf)))
        return {"stand": 0.32, "lie": 0.07, "sit": 0.2, "damp": 0.07}[self.mode]

    def _load(self):
        load = np.zeros(12)
        load[2::3] = 6.0 * min(1.0, max(0.0, (self.height - 0.10) / 0.2))   # 站起来后小腿电机承重
        return load

    def _gait_offset(self):
        speed = math.hypot(self.vx, self.vy) + 0.3 * abs(self.wz)
        amp = min(0.25, 0.5 * speed)
        if amp < 1e-3:
            return np.zeros(12)
        ph = 2 * math.pi * self.gait_hz * self.t
        off = np.zeros(12)
        for leg, shift in enumerate([0.0, math.pi, math.pi, 0.0]):   # 对角小跑: FR+RL 同相, FL+RR 同相
            off[3 * leg + 1] = amp * math.sin(ph + shift)
            off[3 * leg + 2] = -1.2 * amp * max(0.0, math.cos(ph + shift))
        return off

    def step(self, dt):
        super().step(dt)
        # 足端力 (合成的原始读数): 站立时四脚均分，小跑时对角交替，趴下/阻尼时接近零
        supported = min(1.0, max(0.0, (self.height - 0.10) / 0.2))
        base = 55.0 * supported
        speed = math.hypot(self.vx, self.vy) + 0.3 * abs(self.wz)
        ff = np.full(4, base)
        if speed > 0.03 and supported > 0.5:
            ph = 2 * math.pi * self.gait_hz * self.t
            for leg, shift in enumerate([0.0, math.pi, math.pi, 0.0]):
                ff[leg] = 2 * base * (1.0 if math.sin(ph + shift) < 0 else 0.0)
        self.foot_force = np.clip(ff + self.rng.normal(0, 2.0, 4), 0, None)
        # 电池
        mech = float(np.sum(np.abs(self.joints.tau * self.joints.dq)))
        self.power_a = 1.5 + 0.02 * float(np.sum(self.joints.tau ** 2)) ** 0.5 + mech / 29.0
        self.soc = max(0.0, self.soc - dt * self.power_a / 150.0)
        self.power_v = 25.2 + 4.2 * self.soc / 100.0 - 0.05 * self.power_a


# =========================================================================== G1
def _g1_arm_pose(**kw):
    q = np.zeros(29)
    for name, v in kw.items():
        q[R.G1_JOINT_ID[name]] = v
    return q


G1_POSE_STAND = _g1_arm_pose(left_shoulder_roll=0.2, right_shoulder_roll=-0.2, left_elbow=0.6, right_elbow=0.6,
                             left_hip_pitch=-0.1, right_hip_pitch=-0.1, left_knee=0.3, right_knee=0.3,
                             left_ankle_pitch=-0.2, right_ankle_pitch=-0.2)
G1_POSE_SQUAT = _g1_arm_pose(left_shoulder_roll=0.2, right_shoulder_roll=-0.2, left_elbow=0.6, right_elbow=0.6,
                             left_hip_pitch=-1.4, right_hip_pitch=-1.4, left_knee=2.4, right_knee=2.4,
                             left_ankle_pitch=-0.8, right_ankle_pitch=-0.8)

# 上肢动作剧本: 名字 -> [(时长 s, {关节: 角度})...]  (合成的，仅示意动作形状)
G1_ARM_SCRIPTS = {
    "high wave": [(1.0, dict(right_shoulder_pitch=-2.2, right_shoulder_roll=-0.3, right_elbow=0.2)),
                  (0.4, dict(right_shoulder_pitch=-2.2, right_shoulder_roll=-0.7, right_elbow=0.2)),
                  (0.4, dict(right_shoulder_pitch=-2.2, right_shoulder_roll=-0.1, right_elbow=0.2)),
                  (0.4, dict(right_shoulder_pitch=-2.2, right_shoulder_roll=-0.7, right_elbow=0.2)),
                  (0.4, dict(right_shoulder_pitch=-2.2, right_shoulder_roll=-0.1, right_elbow=0.2))],
    "face wave": [(0.8, dict(right_shoulder_pitch=-1.0, right_elbow=-0.6)),
                  (0.4, dict(right_shoulder_pitch=-1.0, right_elbow=-0.6, right_shoulder_yaw=0.5)),
                  (0.4, dict(right_shoulder_pitch=-1.0, right_elbow=-0.6, right_shoulder_yaw=-0.3))],
    "shake hand": [(1.0, dict(right_shoulder_pitch=-0.7, right_elbow=0.3))],
    "high five": [(1.0, dict(right_shoulder_pitch=-1.9, right_elbow=0.0))],
    "hug": [(1.2, dict(left_shoulder_pitch=-1.0, right_shoulder_pitch=-1.0, left_shoulder_roll=0.5,
                       right_shoulder_roll=-0.5, left_elbow=0.2, right_elbow=0.2))],
    "hands up": [(1.2, dict(left_shoulder_pitch=-2.6, right_shoulder_pitch=-2.6))],
    "clap": [(0.8, dict(left_shoulder_pitch=-0.9, right_shoulder_pitch=-0.9, left_shoulder_yaw=-0.5,
                        right_shoulder_yaw=0.5, left_elbow=0.3, right_elbow=0.3)),
             (0.3, dict(left_shoulder_pitch=-0.9, right_shoulder_pitch=-0.9, left_shoulder_yaw=-0.2,
                        right_shoulder_yaw=0.2, left_elbow=0.3, right_elbow=0.3)),
             (0.3, dict(left_shoulder_pitch=-0.9, right_shoulder_pitch=-0.9, left_shoulder_yaw=-0.5,
                        right_shoulder_yaw=0.5, left_elbow=0.3, right_elbow=0.3))],
    "right hand up": [(1.0, dict(right_shoulder_pitch=-2.6))],
    "reject": [(0.8, dict(right_shoulder_pitch=-1.2, right_elbow=-0.3, right_shoulder_yaw=0.4))],
}
# 动作 id (出处: sdk2py g1/arm/g1_arm_action_client.py 的 action_map)
G1_ARM_ACTION_IDS = {"release arm": 99, "two-hand kiss": 11, "left kiss": 12, "right kiss": 13, "hands up": 15,
                     "clap": 17, "high five": 18, "hug": 19, "heart": 20, "right heart": 21, "reject": 22,
                     "right hand up": 23, "x-ray": 24, "face wave": 25, "high wave": 26, "shake hand": 27}
# 做完自动回位的动作 (其余的保持姿势，直到收到 release arm)。真机上哪些会保持，以官方示例里
# "ExecuteAction 之后是否跟一条 release arm" 为准，这里照抄那个规律。
G1_ARM_AUTO_RELEASE = {"high wave", "clap", "face wave"}


class G1Sim(_Base):
    robot = "g1"
    n = 29
    stand_height = 0.79
    gait_hz = 1.6
    internal_kp = 80.0
    internal_kd = 4.0
    MODE_MACHINE = 5        # 合成值。真机的 mode_machine 随机型而定，必须从 LowState 读回来再填进 LowCmd

    def __init__(self, seed=0, hot_motor=3):
        self.joints = JointGroup(29, G1_POSE_SQUAT, inertia=0.05, friction=0.1, tau_max=88.0)
        super().__init__(seed)
        self.fsm_id = R.G1_FSM["damp"]
        self.q_goal = G1_POSE_SQUAT.copy()
        self.posture = "squat"            # squat / stand
        self.heat_gain = np.full(29, 0.003)
        self.heat_gain[hot_motor] = 0.008                  # 左膝
        self.height = 0.45
        self.balance_mode = 0
        self.stand_height_cmd = 0.0
        self.arm_queue: list = []                          # 排队中的上肢关键帧
        self.arm_goal = None                               # 高层上肢动作的当前目标 (29,) 或 None
        self._arm_traj = None
        self.arm_hold = False
        self.arm_sdk = None                                # (q[29], kp[29], kd[29], weight, time)
        self.mode_pr = 0

    # ---- LocoClient -------------------------------------------------------
    def set_fsm_id(self, fsm_id):
        f = R.G1_FSM
        self.fsm_id = int(fsm_id)
        if fsm_id in (f["zero_torque"], f["damp"]):
            self.cmd_until = -1; self._traj = None
        elif fsm_id == f["sit"]:
            self.posture = "squat"; self._go_to(G1_POSE_SQUAT, 2.0)
        elif fsm_id == f["squat2standup"]:                 # 同一个 id 在蹲/站之间切换
            if self.posture == "squat":
                self.posture = "stand"; self._go_to(G1_POSE_STAND, 2.5)
            else:
                self.posture = "squat"; self._go_to(G1_POSE_SQUAT, 2.5)
        elif fsm_id == f["lie2standup"]:
            self.posture = "stand"; self._go_to(G1_POSE_STAND, 4.0)
        elif fsm_id == f["start"]:
            self.posture = "stand"; self._go_to(G1_POSE_STAND, 1.0)
        else:
            return 3204                                    # RPC_ERR_SERVER_API_PARAMETER
        return 0

    def can_move(self):
        # 假设: 只有进入主运控 (Start, fsm 500) 才响应速度指令
        return self.fsm_id == R.G1_FSM["start"] and not self.released

    def passive(self):
        return self.fsm_id in (R.G1_FSM["zero_torque"], R.G1_FSM["damp"])

    def _height_goal(self):
        if self.released:
            return self.height
        return 0.79 if self.posture == "stand" else 0.45

    def _load(self):
        load = np.zeros(29)
        if self.passive():
            return load
        # 膝关节承重: 站直约 8 N·m，蹲得越深越费力 (最深约 25 N·m)
        bend = float(np.clip((self.joints.q[3] + self.joints.q[9]) / 2 / 2.4, 0.0, 1.0))
        load[[3, 9]] = 8.0 + 17.0 * bend
        return load

    def _gait_offset(self):
        speed = math.hypot(self.vx, self.vy) + 0.3 * abs(self.wz)
        amp = min(0.35, 0.7 * speed)
        off = np.zeros(29)
        if amp < 1e-3:
            return off
        ph = 2 * math.pi * self.gait_hz * self.t
        for hip, knee, ankle, sh, shift in ((0, 3, 4, 22, 0.0), (6, 9, 10, 15, math.pi)):
            off[hip] = -amp * math.sin(ph + shift)
            off[knee] = 1.5 * amp * max(0.0, math.sin(ph + shift))
            off[ankle] = -0.5 * amp * max(0.0, math.sin(ph + shift))
            off[sh] = 0.6 * amp * math.sin(ph + shift)     # 对侧手臂摆动
        return off

    # ---- 上肢: 高层动作 (arm 服务) -------------------------------------------
    def arm_action(self, action_id):
        names = {v: k for k, v in G1_ARM_ACTION_IDS.items()}
        if action_id not in names:
            return 3204
        if self.passive():
            return 7400                                    # 合成的错误码: 当前状态不允许做动作
        name = names[action_id]
        if name == "release arm":
            self.arm_queue = [(1.0, None)]
            self.arm_hold = False
            return 0
        script = G1_ARM_SCRIPTS.get(name, [(1.0, dict(right_shoulder_pitch=-1.2, left_shoulder_pitch=-1.2))])
        self.arm_queue = [(T, _g1_arm_pose(**kw)) for T, kw in script]
        self.arm_hold = name not in G1_ARM_AUTO_RELEASE
        if not self.arm_hold:
            self.arm_queue.append((1.0, None))
        return 0

    def arm_task(self, task_id):
        """LocoClient.SetTaskId: 0/1 挥手，2/3 握手的两个阶段"""
        return self.arm_action({0: 26, 1: 26, 2: 27, 3: 99}.get(int(task_id), -1))

    def set_arm_sdk(self, q, kp, kd, weight):
        self.arm_sdk = (np.array(q, float), np.array(kp, float), np.array(kd, float),
                        float(np.clip(weight, 0, 1)), self.t)

    def _overlay(self, q_des, kp, kd):
        arm = R.G1_ARM_IDS
        # 1) 高层动作关键帧
        if self._arm_traj is None and self.arm_queue:
            T, goal = self.arm_queue.pop(0)
            start = self.joints.q.copy()
            self._arm_traj = (self.t, T, start, goal)
        if self._arm_traj is not None:
            t0, T, qs, goal = self._arm_traj
            s = min(1.0, (self.t - t0) / T)
            r = 0.5 - 0.5 * math.cos(math.pi * s)
            tgt = q_des if goal is None else goal
            q_des = q_des.copy()
            q_des[arm] = qs[arm] + r * (tgt[arm] - qs[arm])
            if s >= 1.0:
                self.arm_goal = goal
                self._arm_traj = None
        elif self.arm_goal is not None:
            q_des = q_des.copy()
            q_des[arm] = self.arm_goal[arm]
        # 2) rt/arm_sdk: 按权重与运控的目标混合 (0.5 s 没更新就失效)
        if self.arm_sdk is not None and self.t - self.arm_sdk[4] < 0.5:
            q_u, kp_u, kd_u, w, _ = self.arm_sdk
            ids = arm + R.G1_WAIST_IDS
            q_des = q_des.copy(); kp = kp.copy(); kd = kd.copy()
            q_des[ids] = (1 - w) * q_des[ids] + w * q_u[ids]
            kp[ids] = (1 - w) * kp[ids] + w * kp_u[ids]
            kd[ids] = (1 - w) * kd[ids] + w * kd_u[ids]
        return q_des, kp, kd


def make_sim(robot: str, seed: int = 0):
    return {"go2": Go2Sim, "g1": G1Sim}[robot](seed=seed)
