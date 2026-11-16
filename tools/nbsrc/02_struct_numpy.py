# %% [markdown]
# # 02 · 遥控器按了哪个键？机身歪了多少度？
#
# **Part 1 用 ROS 2 读懂机器人 · 第 3 节**
#
# 上一节用到的 `rpy`、`foot_force` 都是"现成的数"。但 `LowState` 里还有两种不那么现成的东西：
#
# - `wireless_remote`：**40 个原始字节**，遥控器的摇杆和按键都压在里面；
# - `quaternion`：姿态的"原始形式"，4 个数，怎么变成"歪了几度"？
#
# 这一节用两个工具把它们解开：**`struct`**（把字节变成数）和 **NumPy**（把一堆数一次算完）。
#
# 学完这一节你能：
# 1. 对照 C 结构体定义，写出 `struct` 格式串并解析出摇杆值和按键；
# 2. 用 NumPy 一次把几千个四元数转成欧拉角，并用数据验证四元数的分量顺序；
# 3. 用齐次变换把雷达坐标系里的点搬到世界坐标系，画出房间的样子。

# %%
LIVE = False

import struct
import numpy as np
import matplotlib.pyplot as plt

from common import connect
from common.rosutil import collect
from unitree_go.msg import LowState, SportModeState
from sensor_msgs.msg import PointCloud2

lab = connect("go2", live=LIVE, sdk=False)
msgs = [m for _, m in collect("/lowstate", LowState, seconds=16.0, qos=200)]
msgs.sort(key=lambda m: m.tick)
print("收到", len(msgs), "条 LowState")

# %% [markdown]
# ## 1. 先跑通：40 个字节里藏着什么
#
# 取一条消息，把 `wireless_remote` 按十六进制打印出来。

# %%
raw = bytes(msgs[len(msgs) // 2].wireless_remote)
print("长度:", len(raw), "字节")
print(" ".join(f"{b:02x}" for b in raw[:24]))
print(" ".join(f"{b:02x}" for b in raw[24:]), "  <- 后 16 字节没用到")

# %% [markdown]
# 这串字节的"说明书"在官方源码里（`unitree_ros2/example/src/src/g1/lowlevel/gamepad.hpp`），是一个 C 结构体：
#
# ```c
# typedef struct {
#   uint8_t head[2];         // 2 字节
#   xKeySwitchUnion btn;     // 2 字节: 16 个按键，每键 1 比特
#   float lx;                // 4 字节  左摇杆 x
#   float rx;                // 4 字节  右摇杆 x
#   float ry;                // 4 字节  右摇杆 y
#   float L2;                // 4 字节
#   float ly;                // 4 字节  左摇杆 y
#   uint8_t idle[16];        // 16 字节 保留
# } xRockerBtnDataStruct;    // 合计 40 字节
# ```
#
# `struct` 模块用一个**格式串**描述同样的布局，一个字符对应一个字段：
#
# | 字符 | 含义 | 字节数 |
# |---|---|---|
# | `<` | 小端（低位字节在前；x86、ARM 都是小端） | – |
# | `B` | uint8 | 1 |
# | `H` | uint16 | 2 |
# | `f` | float32 | 4 |
# | `x` | 跳过 1 字节 | 1 |
#
# **先自己写**：按上面的结构体，格式串应该是什么？写完再运行下一格对答案。

# %%
FMT = "<2B H 5f 16x"
print("格式串描述的总长度:", struct.calcsize(FMT), "字节")
h0, h1, btn, lx, rx, ry, L2, ly = struct.unpack(FMT, raw)
print(f"摇杆: lx={lx:+.3f} ly={ly:+.3f} rx={rx:+.3f} ry={ry:+.3f}")
print(f"按键位图: {btn} = 二进制 {btn:016b}")

# %% [markdown]
# 按键是 16 个比特。哪一位是哪个键，同一个头文件里有定义（`xKeySwitchUnion`，从最低位起）：
# `R1 L1 start select R2 L2 F1 F2 A B X Y up right down left`。
# 取第 `i` 位用 `btn >> i & 1`。

# %%
KEYS = ["R1", "L1", "start", "select", "R2", "L2", "F1", "F2", "A", "B", "X", "Y", "up", "right", "down", "left"]


def pressed(btn):
    return [k for i, k in enumerate(KEYS) if btn >> i & 1]


# 把整段数据都解出来: 每个时刻的 (时间, 左摇杆 y, 右摇杆 x, 按键)
t = np.array([m.tick for m in msgs]) * 1e-3
rows = [struct.unpack(FMT, bytes(m.wireless_remote)) for m in msgs]
ly_all = np.array([r[7] for r in rows]); rx_all = np.array([r[4] for r in rows])
btn_all = np.array([r[2] for r in rows])

fig, ax = plt.subplots(2, 1, figsize=(9, 4), sharex=True)
ax[0].plot(t, ly_all, label="ly"); ax[0].plot(t, rx_all, label="rx"); ax[0].legend(); ax[0].grid(alpha=.3)
ax[0].set_title("Joysticks")
for i, k in enumerate(KEYS):
    on = (btn_all >> i & 1).astype(bool)
    if on.any():
        ax[1].fill_between(t, i - .4, i + .4, where=on, label=k)
ax[1].set_yticks(range(16)); ax[1].set_yticklabels(KEYS, fontsize=6); ax[1].set_xlabel("robot time [s]")
ax[1].set_title("Buttons"); ax[1].grid(alpha=.3); plt.tight_layout(); plt.show()

changes = np.flatnonzero(np.diff(btn_all) != 0) + 1
for i in changes:
    print(f"t = {t[i]:6.2f} s  按键变为 {pressed(int(btn_all[i])) or '(全部松开)'}")

# %% [markdown]
# **停一下：** 数据里出现过一次**两个键同时按下**。是哪两个？你是从位图的哪两位看出来的？
#
# ## 2. 改坏它：`struct` 的两种典型错误
#
# **预测一**：把 `<`（小端）改成 `>`（大端），摇杆值会变成什么样？

# %%
print("小端:", [round(v, 3) for v in struct.unpack("<2B H 5f 16x", raw)[3:]])
print("大端:", [f"{v:.3g}" for v in struct.unpack(">2B H 5f 16x", raw)[3:]])

# %% [markdown]
# 不报错，但数全是垃圾。**字节序错了程序不会告诉你**，只能靠"这个数合不合理"发现。
#
# **预测二**：少写一个 `f`（漏了一个字段）呢？

# %%
try:
    struct.unpack("<2B H 4f 16x", raw)
except struct.error as e:
    print("struct.error:", e)

# %% [markdown]
# 这次直接报错：格式串描述的长度（36）和实际字节数（40）对不上。**长度对不上会报错，顺序/类型错了不会**——
# 所以写完格式串第一件事是 `struct.calcsize(FMT)` 对一下总长度。
#
# > 这套解析已经收进公共库 `common.parsing.parse_remote()`，后面直接用。
# > 第 11 节你会再遇到 `struct`：给电机指令算 CRC 校验时，SDK 用一个长得多的格式串把整条 `LowCmd` 打包成 812 字节。
#
# ## 3. NumPy：一次算完几千个姿态
#
# IMU 给了两种姿态表示：`quaternion`（4 个数）和 `rpy`（3 个角）。四元数是"原始"的，欧拉角是算出来的。
# 我们自己从四元数算一遍欧拉角，顺便回答一个文档里没写清楚的问题：**四元数的 4 个数是 (w,x,y,z) 还是 (x,y,z,w)？**
#
# 先把几千条消息里的四元数堆成一个 `(N, 4)` 的数组——这是用 NumPy 的第一步，永远是"堆成数组"。

# %%
quat = np.array([m.imu_state.quaternion for m in msgs])     # (N, 4)
rpy_msg = np.array([m.imu_state.rpy for m in msgs])         # (N, 3)  机器人自己算好的
print(quat.shape, rpy_msg.shape)


def quat_to_rpy(q):
    """q: (N, 4)，假设顺序是 (w, x, y, z)。没有 for 循环: 每一行同时算。"""
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.stack([roll, pitch, yaw], axis=1)


err_wxyz = np.abs(quat_to_rpy(quat) - rpy_msg).max()
err_xyzw = np.abs(quat_to_rpy(quat[:, [3, 0, 1, 2]]) - rpy_msg).max()      # 假设是 (x,y,z,w): 先把 w 挪到最前
print(f"假设 (w,x,y,z): 与消息里的 rpy 最大相差 {err_wxyz:.4f} rad")
print(f"假设 (x,y,z,w): 与消息里的 rpy 最大相差 {err_xyzw:.4f} rad")

# %% [markdown]
# 哪种假设对，数据自己说了。**文档没写清楚的约定，用数据验证**——这比问人可靠。
#
# > 诚实地说：你现在用的是合成数据，它本来就是按 (w,x,y,z) 生成的，所以这里只证明了"方法可行"。
# > 官方源码里没有写明这个顺序。**连上真机后把这一格重跑一遍**，才算真正验证（真机待验清单 C1）。
# （注意 ROS 2 标准消息 `geometry_msgs/Quaternion` 的字段顺序是 x,y,z,w。第 08 节做桥接时这里是个坑。）
#
# 再用加速度计验证一次。静止时加速度计读到的是重力的反作用：在**机体系**里大小约 9.8，方向取决于机身怎么歪。
# 用姿态的旋转矩阵把它转到**世界系**，应该正好是 `[0, 0, 9.8]`。

# %%
from common.parsing import rot_from_rpy

i = 100                                                   # 取一个机器人静止的时刻
acc_body = np.array(msgs[i].imu_state.accelerometer)
R_world_body = rot_from_rpy(*rpy_msg[i])                  # 3x3 旋转矩阵: 机体系 -> 世界系
print("机体系加速度:", np.round(acc_body, 2))
print("世界系加速度:", np.round(R_world_body @ acc_body, 2), "  <- 应接近 [0, 0, 9.81]")

# %% [markdown]
# ## 4. 坐标变换：把雷达看到的点搬到房间里
#
# 雷达点云 `/utlidar/cloud` 里每个点的坐标是**相对雷达**的。机器人走到哪、朝哪，由 `/sportmodestate` 的位置和偏航角给出。
# 把两者合起来，就能把点画到房间（世界系）里：
#
# $$ p_{world} = T_{world \leftarrow body} \; p_{body}, \qquad
# T = \begin{bmatrix} \cos\psi & -\sin\psi & x \\ \sin\psi & \cos\psi & y \\ 0 & 0 & 1 \end{bmatrix} $$

# %%
from unitree_lab_nodes.cloud_to_scan import cloud_to_xyz
from common.parsing import transform_2d
from common.plotting import plot_room

sport = [m for _, m in collect("/sportmodestate", SportModeState, seconds=16.0, qos=200)]
clouds = [m for _, m in collect("/utlidar/cloud", PointCloud2, seconds=16.0, qos=50)]
print(len(sport), "条里程计,", len(clouds), "帧点云; 一帧", clouds[0].width, "个点, 坐标系:", clouds[0].header.frame_id)


def stamp(m):
    s = m.stamp if hasattr(m, "stamp") else m.header.stamp
    return s.sec + s.nanosec * 1e-9


sport_t = np.array([stamp(m) for m in sport])
LIDAR_X = 0.28            # 雷达装在机身前方 0.28 m (合成数据的值；真机以 URDF 为准，第 16 节再细说)

ax = plot_room()
for c in sorted(clouds, key=stamp)[::12]:
    s = sport[int(np.argmin(np.abs(sport_t - stamp(c))))]          # 时间上最近的那条里程计
    T = transform_2d(s.position[0], s.position[1], s.imu_state.rpy[2])
    xyz = cloud_to_xyz(c)
    xyz = xyz[np.abs(xyz[:, 2]) < 0.2]                             # 只留接近水平的一层
    pts_body = np.stack([xyz[:, 0] + LIDAR_X, xyz[:, 1], np.ones(len(xyz))])   # 3 x N 齐次坐标
    pts_world = T @ pts_body
    ax.plot(pts_world[0], pts_world[1], ".", ms=1.5)
    ax.plot(s.position[0], s.position[1], "r^", ms=6)
ax.set_title("Lidar points transformed into the world frame"); plt.show()

# %% [markdown]
# 彩色的点落在黑色的墙上——说明变换是对的。红色三角是机器人在这几帧的位置。
#
# **改坏它（自己动手）**：把 `T @ pts_body` 换成 `pts_body`（不做变换）再画一次。预测一下图会变成什么样，再验证。
#
# ## 5. 换成 G1 再做一遍
#
# G1 的 `LowState` 类型不同，但遥控器那 40 字节的布局**一模一样**，所以解析代码一行不用改。

# %%
lab.close()
lab = connect("g1", live=LIVE, sdk=False)
from unitree_hg.msg import LowState as G1LowState
from common.parsing import parse_remote

g1 = [m for _, m in collect("/lowstate", G1LowState, seconds=3.0, qos=200)]
print("G1:", parse_remote(g1[-1].wireless_remote))
q = np.array([m.imu_state.quaternion for m in g1]); r = np.array([m.imu_state.rpy for m in g1])
print("G1 四元数按 (w,x,y,z) 算出的 rpy 与消息相差:", f"{np.abs(quat_to_rpy(q) - r).max():.4f} rad")
lab.close()

# %% [markdown]
# ## 小结
#
# - `struct` 把字节变成数：格式串对照 C 结构体逐字段写，**先用 `calcsize` 对总长度**；字节序错了不报错。
# - NumPy 的套路：先堆成 `(N, k)` 数组，再整列运算，不写 for 循环。
# - Unitree 的四元数顺序按 **(w, x, y, z)** 处理；验证方法是和 `rpy` 字段对比（真机上要重做一次）。
# - 坐标变换 = 乘一个矩阵；变换对不对，画出来看点落没落在该落的地方。
#
# ## 课后练习
# 1. 写 `pack_remote(lx, ly, rx, ry, keys)`，把摇杆值和按键列表**打包回** 40 字节，并用 `struct.unpack` 验证能解回来。（参考解：`common/parsing.py`）
# 2. 右摇杆 `rx` 的周期是多少秒？用 `np.fft.rfft` 求，不要数图。
# 3. 上面画点云时用的是"时间上最近的里程计"。如果机器人在转弯，这个近似会带来什么误差？怎么改进？（提示：插值。）
#
# **下一节**：`/lowstate` 每秒几百条，点云每秒 5 帧但每帧都很大。一个节点怎么同时把它们都处理好，一个都不耽误？
