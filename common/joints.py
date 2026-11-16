"""关节插值与限位。所有关节级控制都必须经过这里，Notebook 里不要自己手写。"""
from __future__ import annotations

import numpy as np


def clamp_to_limits(q, lower, upper, margin: float = 0.0):
    """把目标角度夹到 [lower+margin, upper-margin]。返回 (夹紧后的 q, 被夹的关节下标)。"""
    q = np.asarray(q, dtype=float)
    lo = np.asarray(lower, dtype=float) + margin
    hi = np.asarray(upper, dtype=float) - margin
    clipped = np.clip(q, lo, hi)
    return clipped, np.flatnonzero(np.abs(clipped - q) > 1e-9)


def limit_step(q_target, q_prev, max_step: float):
    """限制单个控制周期内的最大变化量 (rad)，防止目标突变打出大力矩。"""
    q_target = np.asarray(q_target, dtype=float)
    q_prev = np.asarray(q_prev, dtype=float)
    return q_prev + np.clip(q_target - q_prev, -max_step, max_step)


def ratio_linear(s):
    """线性: 起止速度不为零 (官方示例用的就是它)"""
    return np.clip(s, 0.0, 1.0)


def ratio_cosine(s):
    """余弦缓入缓出: 起止速度为零"""
    s = np.clip(s, 0.0, 1.0)
    return 0.5 - 0.5 * np.cos(np.pi * s)


def ratio_minjerk(s):
    """五次多项式 (最小加加速度): 起止速度、加速度都为零"""
    s = np.clip(s, 0.0, 1.0)
    return 10 * s**3 - 15 * s**4 + 6 * s**5


_PROFILES = {"linear": ratio_linear, "cosine": ratio_cosine, "minjerk": ratio_minjerk}


def interpolate(q_start, q_goal, t: float, duration: float, profile: str = "cosine"):
    """t 时刻的插值目标。t<=0 返回起点，t>=duration 返回终点。"""
    q_start = np.asarray(q_start, dtype=float)
    q_goal = np.asarray(q_goal, dtype=float)
    r = _PROFILES[profile](t / duration if duration > 0 else 1.0)
    return q_start + r * (q_goal - q_start)


def trajectory(q_start, q_goal, duration: float, dt: float, profile: str = "cosine"):
    """整条轨迹: 返回 (t[N], q[N, n_joint])"""
    t = np.arange(0.0, duration + 0.5 * dt, dt)
    return t, np.stack([interpolate(q_start, q_goal, ti, duration, profile) for ti in t])


class SafeJointCommander:
    """把 "目标角度" 变成 "这一拍可以安全下发的角度"。

    依次做三件事: 限位夹紧 -> 插值 -> 单步限幅。用法见 11 节。

        cmd = SafeJointCommander(lower, upper, dt=0.002)
        cmd.start(q_now, q_goal, duration=3.0)
        while not cmd.done:
            q_des = cmd.step()      # 交给 LowCmd
    """

    def __init__(self, lower, upper, dt: float, max_vel: float = 2.0, margin: float = 0.05,
                 profile: str = "cosine"):
        self.lower, self.upper = np.asarray(lower, float), np.asarray(upper, float)
        self.dt, self.max_vel, self.margin, self.profile = dt, max_vel, margin, profile
        self.t = 0.0
        self.duration = 0.0
        self.q_start = self.q_goal = self.q_last = None
        self.clipped_joints = np.array([], dtype=int)

    def start(self, q_now, q_goal, duration: float):
        self.q_start = np.asarray(q_now, float).copy()
        self.q_goal, self.clipped_joints = clamp_to_limits(q_goal, self.lower, self.upper, self.margin)
        # 时间太短会超速: 自动拉长到最大速度允许的时长
        need = float(np.max(np.abs(self.q_goal - self.q_start))) / self.max_vel * 1.875  # minjerk 峰值速度系数
        self.duration = max(duration, need, self.dt)
        self.t = 0.0
        self.q_last = self.q_start.copy()
        return self.duration

    @property
    def done(self) -> bool:
        return self.q_goal is None or self.t >= self.duration

    def step(self):
        self.t += self.dt
        q = interpolate(self.q_start, self.q_goal, self.t, self.duration, self.profile)
        q = limit_step(q, self.q_last, self.max_vel * self.dt * 2.0)
        self.q_last = q
        return q
