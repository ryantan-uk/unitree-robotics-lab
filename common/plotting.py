"""绘图。图上的文字一律用英文: 全新 Ubuntu 没有中文字体，中文会显示成方块。"""
from __future__ import annotations

import numpy as np
import matplotlib.pyplot as plt

from .connect import DATA


def room_walls():
    """录制合成数据时用的虚拟房间墙面 [(x1, y1, x2, y2), ...]，供画图对照 (data/room.json)。"""
    import json
    return [tuple(w) for w in json.loads((DATA / "room.json").read_text())["walls"]]


def _ax(ax, figsize=(9, 3.2)):
    if ax is None:
        _, ax = plt.subplots(figsize=figsize)
    return ax


def plot_rpy(t, rpy, ax=None, title="IMU attitude", degrees=True):
    """t: (N,)  rpy: (N, 3) rad"""
    ax = _ax(ax)
    rpy = np.asarray(rpy)
    k = 180 / np.pi if degrees else 1.0
    for i, name in enumerate(("roll", "pitch", "yaw")):
        ax.plot(t, rpy[:, i] * k, label=name)
    ax.set(xlabel="time [s]", ylabel="deg" if degrees else "rad", title=title)
    ax.legend(loc="upper right", ncol=3); ax.grid(alpha=0.3)
    return ax


def plot_joints(t, q, names, ax=None, title="Joint position", ylabel="rad", ids=None):
    """t: (N,)  q: (N, n)。ids 选要画哪些关节 (默认全部)。"""
    ax = _ax(ax, (9, 3.6))
    q = np.asarray(q)
    ids = range(q.shape[1]) if ids is None else ids
    for i in ids:
        ax.plot(t, q[:, i], label=names[i], lw=1.2)
    ax.set(xlabel="time [s]", ylabel=ylabel, title=title)
    ax.legend(loc="upper right", ncol=4, fontsize=7); ax.grid(alpha=0.3)
    return ax


def plot_foot_force(t, ff, ax=None, title="Go2 foot force (raw)"):
    ax = _ax(ax)
    ff = np.asarray(ff)
    for i, leg in enumerate(("FR", "FL", "RR", "RL")):
        ax.plot(t, ff[:, i], label=leg, lw=1.0)
    ax.set(xlabel="time [s]", ylabel="raw reading", title=title)
    ax.legend(loc="upper right", ncol=4); ax.grid(alpha=0.3)
    return ax


def plot_bars(values, names, ax=None, title="", ylabel="", highlight=None):
    ax = _ax(ax, (9, 3))
    colors = ["tab:red" if (highlight is not None and i == highlight) else "tab:blue" for i in range(len(values))]
    ax.bar(range(len(values)), values, color=colors)
    ax.set_xticks(range(len(values)))
    ax.set_xticklabels(names, rotation=60, ha="right", fontsize=7)
    ax.set(title=title, ylabel=ylabel); ax.grid(alpha=0.3, axis="y")
    return ax


def plot_room(ax=None, walls=None, title="Room where the sample data was recorded (synthetic)"):
    """画样本数据所在的房间 (真值)，用来对照点云和地图。"""
    ax = _ax(ax, (6, 4.6))
    walls = room_walls() if walls is None else walls
    for x1, y1, x2, y2 in walls:
        ax.plot([x1, x2], [y1, y2], "k-", lw=2)
    ax.set_aspect("equal"); ax.set(xlabel="x [m]", ylabel="y [m]", title=title); ax.grid(alpha=0.3)
    return ax


def plot_path(xy, ax=None, label="path", **kw):
    ax = plot_room() if ax is None else ax
    xy = np.asarray(xy)
    ax.plot(xy[:, 0], xy[:, 1], label=label, **kw)
    ax.plot(xy[0, 0], xy[0, 1], "go", ms=6); ax.plot(xy[-1, 0], xy[-1, 1], "r*", ms=10)
    ax.legend(loc="upper right", fontsize=8)
    return ax


def plot_hist_dt(stamps, ax=None, title="Inter-arrival time"):
    """接收间隔直方图: 一眼看出频率和抖动。"""
    ax = _ax(ax, (6, 3))
    dt = np.diff(np.asarray(stamps)) * 1e3
    ax.hist(dt, bins=50)
    ax.axvline(dt.mean(), color="r", ls="--", label=f"mean {dt.mean():.2f} ms = {1e3 / dt.mean():.0f} Hz")
    ax.set(xlabel="dt [ms]", ylabel="count", title=title); ax.legend(); ax.grid(alpha=0.3)
    return ax
