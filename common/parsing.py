"""把机器人消息变成 NumPy 数组 / 字典。只依赖 struct 和 numpy，不依赖 ROS 或 DDS。

消息对象可以是 ROS 2 的 (unitree_go.msg.LowState) 也可以是 SDK2 Python 的
(unitree_sdk2py...LowState_) —— 两者字段名相同，所以这里统一用属性访问。
"""
from __future__ import annotations

import struct

import numpy as np

# ------------------------------------------------------------------ 遥控器 40 字节
# 出处: unitree_ros2/example/src/src/g1/lowlevel/gamepad.hpp (xRockerBtnDataStruct)
#   uint8 head[2]; uint16 btn; float lx, rx, ry, L2, ly; uint8 idle[16];
REMOTE_FMT = "<2B H 5f 16x"          # 小端; 2+2+20+16 = 40 字节
REMOTE_SIZE = struct.calcsize(REMOTE_FMT)
# btn 的 16 个比特，从最低位起 (xKeySwitchUnion)
REMOTE_KEYS = ["R1", "L1", "start", "select", "R2", "L2", "F1", "F2",
               "A", "B", "X", "Y", "up", "right", "down", "left"]


def parse_remote(raw) -> dict:
    """解析 LowState.wireless_remote (40 字节) -> {'lx':..,'keys':[...],...}"""
    buf = bytes(bytearray(int(b) & 0xFF for b in raw))
    if len(buf) != REMOTE_SIZE:
        raise ValueError(f"wireless_remote 应为 {REMOTE_SIZE} 字节，收到 {len(buf)}")
    h0, h1, btn, lx, rx, ry, l2, ly = struct.unpack(REMOTE_FMT, buf)
    return {
        "head": (h0, h1), "btn": btn,
        "lx": lx, "ly": ly, "rx": rx, "ry": ry, "L2": l2,
        "keys": [k for i, k in enumerate(REMOTE_KEYS) if btn >> i & 1],
    }


def pack_remote(lx=0.0, ly=0.0, rx=0.0, ry=0.0, keys=(), l2=0.0) -> bytes:
    """parse_remote 的逆操作 (生成样本数据和写测试时用)。"""
    btn = 0
    for k in keys:
        btn |= 1 << REMOTE_KEYS.index(k)
    return struct.pack(REMOTE_FMT, 0, 0, btn, lx, rx, ry, l2, ly)


# ------------------------------------------------------------------ 姿态数学
def quat_to_rpy(q) -> np.ndarray:
    """四元数 (w, x, y, z) -> 欧拉角 (roll, pitch, yaw)，ZYX 顺序，单位 rad。

    支持形状 (4,) 或 (N, 4)。Unitree IMUState.quaternion 的顺序为 w,x,y,z
    (未在源码中核实，依据官方文档；02 节让学生用 rpy 字段自己验证)。
    """
    q = np.asarray(q, dtype=float)
    w, x, y, z = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.stack([roll, pitch, yaw], axis=-1)


def rpy_to_quat(rpy) -> np.ndarray:
    """欧拉角 (roll, pitch, yaw) -> 四元数 (w, x, y, z)"""
    rpy = np.asarray(rpy, dtype=float)
    r, p, y = rpy[..., 0] / 2, rpy[..., 1] / 2, rpy[..., 2] / 2
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.stack([cr * cp * cy + sr * sp * sy,
                     sr * cp * cy - cr * sp * sy,
                     cr * sp * cy + sr * cp * sy,
                     cr * cp * sy - sr * sp * cy], axis=-1)


def rot_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """机体系 -> 世界系 的旋转矩阵 R = Rz(yaw) Ry(pitch) Rx(roll)"""
    cr, sr, cp, sp, cy, sy = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch), np.cos(yaw), np.sin(yaw)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def transform_2d(x: float, y: float, yaw: float) -> np.ndarray:
    """平面位姿 -> 3x3 齐次变换矩阵"""
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, x], [s, c, y], [0, 0, 1.0]])


def tilt_deg(roll, pitch):
    """机身相对水平面的倾角 (度): 重力方向与机体 z 轴的夹角"""
    return np.degrees(np.arccos(np.clip(np.cos(roll) * np.cos(pitch), -1, 1)))


# ------------------------------------------------------------------ LowState -> 数组
def motors_to_array(msg, n: int, field: str = "q") -> np.ndarray:
    """取前 n 个电机的某个字段。temperature 在 G1 上是长度 2 的数组，取最大值。"""
    out = np.empty(n)
    for i in range(n):
        v = getattr(msg.motor_state[i], field)
        out[i] = max(v) if hasattr(v, "__len__") else v
    return out


def lowstate_to_dict(msg, robot: str) -> dict:
    """LowState (Go2 或 G1) -> 扁平字典，值全是 numpy 数组或标量。

    两种机器人共有: tick, quat, gyro, accel, rpy, q, dq, tau, temp, remote
    仅 Go2:        foot_force, power_v, power_a, soc
    仅 G1:         mode_machine, mode_pr, motor_vol
    """
    from .robots import num_motor
    n = num_motor(robot)
    imu = msg.imu_state
    d = {
        "tick": int(msg.tick),
        "quat": np.array(imu.quaternion, dtype=float),
        "gyro": np.array(imu.gyroscope, dtype=float),
        "accel": np.array(imu.accelerometer, dtype=float),
        "rpy": np.array(imu.rpy, dtype=float),
        "q": motors_to_array(msg, n, "q"),
        "dq": motors_to_array(msg, n, "dq"),
        "tau": motors_to_array(msg, n, "tau_est"),
        "temp": motors_to_array(msg, n, "temperature"),
        "remote": np.array([int(b) & 0xFF for b in msg.wireless_remote], dtype=np.uint8),
    }
    if robot == "go2":
        d["foot_force"] = np.array(msg.foot_force, dtype=float)
        d["power_v"] = float(msg.power_v)
        d["power_a"] = float(msg.power_a)
        d["soc"] = int(msg.bms_state.soc)
    else:
        d["mode_machine"] = int(msg.mode_machine)
        d["mode_pr"] = int(msg.mode_pr)
        d["motor_vol"] = motors_to_array(msg, n, "vol")
    return d


def stack_records(records: list[dict]) -> dict:
    """[{k: v}, ...] -> {k: array(N, ...)}，方便一次性画图"""
    return {k: np.stack([np.asarray(r[k]) for r in records]) for k in records[0]}


# ------------------------------------------------------------------ 频率 / 延迟
def rate_stats(stamps) -> dict:
    """由接收时间戳 (秒) 计算频率与抖动。"""
    t = np.asarray(stamps, dtype=float)
    if len(t) < 3:
        return {"n": len(t), "hz": float("nan"), "dt_mean_ms": float("nan"),
                "dt_std_ms": float("nan"), "dt_max_ms": float("nan")}
    dt = np.diff(t)
    return {"n": len(t), "hz": 1.0 / dt.mean(), "dt_mean_ms": dt.mean() * 1e3,
            "dt_std_ms": dt.std() * 1e3, "dt_max_ms": dt.max() * 1e3}
