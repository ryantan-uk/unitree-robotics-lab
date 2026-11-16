# %% [markdown]
# # 14 · 量一量：多快、多稳、多晚
#
# **Part 3 做出完整应用 · 第 2 节**
#
# 上一节量出了看门狗的反应时间。这一节把"量"变成习惯，回答三个问题：
#
# 1. 每个 Topic **实际**是多少 Hz？（不是文档写的，是你这条链路上测到的）
# 2. 消息来得**稳不稳**？（抖动）
# 3. 一条数据从产生到被你用上，**晚了多久**？（延迟）
#
# 然后把 IMU 姿态和关节曲线画成能放进报告的图。
#
# 学完你能：写出自己的 `topic hz`；区分频率、抖动、延迟；在两台时钟不同步的机器之间也能估计延迟的**变化**。

# %%
LIVE = False

import time
import numpy as np
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message

from common import connect, Background
from common.bagio import load_npz
from common.parsing import rate_stats
from common.plotting import plot_hist_dt, plot_joints, plot_rpy
from common.robots import GO2_JOINTS, G1_JOINTS, G1_JOINT_ID
from common.rosutil import ros_init, sh, spin_for, topic_table

lab = connect("go2", live=LIVE, sdk=False)
ros_init()

# %% [markdown]
# ## 1. 先跑通：一次量完所有 Topic 的频率
#
# 命令行有 `ros2 topic hz`，但一次只能看一个。自己写一个：对每个 Topic 记下每条消息的**到达时刻**，再算间隔。
# （`get_message("unitree_go/msg/LowState")` 能按类型名字符串拿到消息类，这样不用事先 import。）

# %%
class RateMeter(Node):
    def __init__(self, topics):
        super().__init__("rate_meter")
        self.stamps = {name: [] for name, _ in topics}
        for name, type_name in topics:
            self.create_subscription(get_message(type_name), name,
                                     lambda msg, name=name: self.stamps[name].append(time.monotonic()), 50)


topics = [(name, typ) for name, typ, n_pub, _ in topic_table() if n_pub > 0 and not name.startswith(("/rosout", "/parameter", "/events"))]
meter = RateMeter(topics)
spin_for(meter, 8.0)
print(f"{'Topic':<34}{'条数':>6}{'Hz':>8}{'间隔均值':>10}{'抖动(std)':>11}{'最大间隔':>10}")
for name, _ in topics:
    s = rate_stats(meter.stamps[name])
    if s["n"] > 2:
        print(f"{name:<34}{s['n']:>6}{s['hz']:>8.1f}{s['dt_mean_ms']:>8.1f}ms{s['dt_std_ms']:>9.1f}ms{s['dt_max_ms']:>8.1f}ms")

# %% [markdown]
# 和命令行工具对一下（它量 5 秒后我们主动结束它）：

# %%
_ = sh("timeout 6 ros2 topic hz /lowstate 2>&1 | grep -A1 'average rate' | tail -n 2")

# %% [markdown]
# **停一下：** `/lowstate` 测出来是多少？和你在 00 节课后练习里记下的数一样吗？
#
# > 回放的这段数据录制时 `/lowstate` 约 200 Hz（合成数据的生成速度）。**真机的 `rt/lowstate` 标称约 500 Hz**——
# > 这个数字来自官方文档而非源码，属于"真机待验"：上真机后运行本格，把实测值填进 `docs/real_robot_checklist.md`。
# > 这正是本节的态度：频率是测出来的。
#
# ## 2. 拆开：频率一样，稳定性可以差很多
#
# 平均 200 Hz 可以是"每 5 ms 一条"，也可以是"一口气来 20 条然后停 100 ms"。对控制来说天差地别。看间隔的分布：

# %%
fig, ax = plt.subplots(1, 2, figsize=(11, 2.8))
plot_hist_dt(meter.stamps["/lowstate"], ax=ax[0], title="/lowstate inter-arrival")
plot_hist_dt(meter.stamps["/sportmodestate"], ax=ax[1], title="/sportmodestate inter-arrival")
ax[0].set_yscale("log"); ax[1].set_yscale("log"); plt.tight_layout(); plt.show()
dt = np.diff(meter.stamps["/lowstate"]) * 1e3
print(f"/lowstate: 中位间隔 {np.median(dt):.1f} ms, 99% 的间隔小于 {np.percentile(dt, 99):.1f} ms, 最坏 {dt.max():.1f} ms")

# %% [markdown]
# 报告一个 Topic 的时序，至少要三个数：**平均频率、典型间隔（中位数）、最坏间隔**。控制器要按最坏情况设计（上一节的超时就是）。
#
# ### 改坏它：给系统加点负载
#
# **预测**：如果同一台电脑上有个程序在满负荷计算，`/lowstate` 的平均频率会掉吗？最坏间隔呢？

# %%
burn = Background("exec python3 -c 'while True: sum(i*i for i in range(10**6))'")       # 一个吃满 CPU 的进程
busy = RateMeter([("/lowstate", "unitree_go/msg/LowState")])
spin_for(busy, 6.0)
burn.stop()
for label, st in (("空闲", meter.stamps["/lowstate"]), ("有负载", busy.stamps["/lowstate"])):
    s = rate_stats(st); d = np.diff(st) * 1e3
    print(f"{label:<5} {s['hz']:6.1f} Hz   99% 间隔 < {np.percentile(d, 99):5.1f} ms   最坏 {d.max():6.1f} ms")
busy.destroy_node()

# %% [markdown]
# （结果随机器而变：核数多的电脑上可能几乎看不出差别；单核虚拟机上"最坏间隔"会明显变大。把你的结果和同学的比一比。）
# 平均值往往变化不大，**尾巴**先变坏。只看平均频率会漏掉问题。
#
# ## 3. 延迟：这条数据是多久以前的？
#
# 延迟 = 你收到的时刻 − 数据产生的时刻。难点在于这两个时刻是**两台电脑的时钟**读的，而它们通常没对过表。
#
# `LowState` 里有 `tick`：机器人自己的毫秒计数。它和你电脑的时间差一个未知的常数，所以**延迟的绝对值算不出来**——
# 但延迟的**变化**可以：`到达时刻 − tick` 如果一直不变，说明延迟恒定；它抖，就是延迟在抖。
#
# 用 `go2_demo.npz` 里的记录（`t` 是录制端的到达时刻，`tick` 是机器人时钟）：

# %%
d = load_npz("go2_demo")
offset = d["t"] - d["tick"] * 1e-3                 # 到达时刻 - 产生时刻 (含一个未知常数)
drift, c0 = np.polyfit(d["t"], offset, 1)          # 两个时钟走得不一样快 -> offset 会线性地漂; 先拟合出来扣掉
resid = offset - (drift * d["t"] + c0)
rel = (resid - resid.min()) * 1e3                  # 以最快的那一条为基准: "比最快的那条晚了多少 ms"
fig, ax = plt.subplots(1, 2, figsize=(11, 2.8))
ax[0].plot(d["t"], rel, ".", ms=2); ax[0].set(xlabel="time [s]", ylabel="extra delay [ms]", title="Delay relative to the fastest message")
ax[1].hist(rel, bins=60); ax[1].set(xlabel="extra delay [ms]", yscale="log"); [a.grid(alpha=.3) for a in ax]
plt.tight_layout(); plt.show()
print(f"相对延迟: 中位 {np.median(rel):.1f} ms, 95% 分位 {np.percentile(rel, 95):.1f} ms, 最大 {rel.max():.1f} ms")
print(f"两个时钟的相对漂移: {drift * 1e6:+.0f} ppm (每秒差 {drift * 1e6:+.0f} 微秒), 已在上图中扣除")

# %% [markdown]
# 这个技巧（用发送端计数器 + 接收端时钟看相对延迟）在真机上直接可用，不需要任何时钟同步。
# 注意代码里先做了一步直线拟合：两个时钟走得不一样快时，`到达时刻 − tick` 会随时间线性地漂，不扣掉的话漂移会被误当成"延迟越来越大"。
# （这段合成数据的漂移有几千 ppm，比真实晶振的几十 ppm 大得多——生成数据的程序跑不满标称频率造成的。真机上重做这个分析，漂移应该小得多。）
#
# ### 能测绝对延迟的情况：同一台电脑上的一个来回
#
# 发送和接收在同一台电脑上时，时钟是同一个。测一下 ROS 2 在本机传一条消息要多久，以及**消息大小**的影响：

# %%
from std_msgs.msg import UInt8MultiArray


import threading
from rclpy.executors import SingleThreadedExecutor


def loopback_latency(payload_bytes, n=150):
    node = Node("latency_probe_%d" % payload_bytes)
    sent, lat = {}, []

    def on_msg(msg):
        t_recv = time.perf_counter()                       # 回调一进来就读表
        seq = int.from_bytes(bytes(msg.data[:4]), "little")
        lat.append((t_recv - sent[seq]) * 1e3)
    node.create_subscription(UInt8MultiArray, "/lab/ping", on_msg, 10)
    pub = node.create_publisher(UInt8MultiArray, "/lab/ping", 10)
    ex = SingleThreadedExecutor(); ex.add_node(node)
    th = threading.Thread(target=ex.spin, daemon=True); th.start()     # 接收放在独立线程里一直转，测量才不受轮询间隔影响
    time.sleep(0.5)                                        # 等发布者和订阅者互相发现
    body = bytes(max(0, payload_bytes - 4))
    for seq in range(n):
        msg = UInt8MultiArray(); msg.data = seq.to_bytes(4, "little") + body
        sent[seq] = time.perf_counter(); pub.publish(msg)
        time.sleep(0.02)
    ex.shutdown(); th.join(2.0); node.destroy_node()
    return np.array(lat)


loopback_latency(64, n=50)                                 # 先空跑一次热身 (第一次建连接比较慢，不计入)
for size, what in ((64, "一条小指令"), (1200, "约等于一条 LowState"), (25_000, "约等于一帧点云"), (1_000_000, "约等于一张图片")):
    lat = loopback_latency(size)
    print(f"{size:>9} 字节 ({what:<16}): 中位 {np.median(lat):6.2f} ms, 95% 分位 {np.percentile(lat, 95):6.2f} ms")

# %% [markdown]
# 在本机回环上，几十 KB 以内的消息延迟差别不大（都在毫秒量级，主要是 Python 和调度的开销）；到 1 MB 才明显变慢——序列化、拷贝、分片都要时间。
# **真实网络上大小的影响要大得多**：百兆网线每毫秒只能传约 12 KB，Wi-Fi 更少，一帧点云就要好几毫秒，还会挤占同一条链路上的 `lowstate`。
# 所以高频回路里只传必要的小消息；点云、图像这类大消息单独走低频通道（03 节的做法）。连真机后请重跑这一格的思路去测真实链路（课后练习 3）。
#
# ## 4. 把曲线画好：IMU 姿态和关节
#
# 公共库 `common/plotting.py` 里有画姿态和关节的函数。一张合格的图：**有单位、有图例、能看出事件发生在什么时候**。

# %%
fig, ax = plt.subplots(3, 1, figsize=(10, 7.5), sharex=True)
plot_rpy(d["t"], d["rpy"], ax=ax[0], title="Go2: IMU attitude")
plot_joints(d["t"], d["q"], GO2_JOINTS, ax=ax[1], title="Go2: joint positions (front-right leg)", ids=[0, 1, 2])
plot_joints(d["t"], d["tau"], GO2_JOINTS, ax=ax[2], title="Go2: joint torques (front-right leg)", ylabel="N·m", ids=[0, 1, 2])
standing = np.argmax(d["position"][:, 2] > 0.25)
for a in ax:
    a.axvline(d["sport_t"][standing], color="g", ls="--", lw=.8)
ax[0].text(d["sport_t"][standing], ax[0].get_ylim()[1] * .8, " stood up", color="g", fontsize=8)
plt.tight_layout(); plt.show()

# %% [markdown]
# **读图：**
# 1. yaw 在哪一段从 0 变到约 90°？这段时间关节曲线的摆动幅度和直行时一样吗？
# 2. 站起来之后小腿（calf）的力矩为什么不回到 0？（11 节实验 B。）
# 3. 行走时大腿和小腿的摆动频率是多少？（数峰，或用 07 节的 FFT。）
#
# ### 换成 G1

# %%
g = load_npz("g1_demo")
fig, ax = plt.subplots(2, 1, figsize=(10, 5.5), sharex=True)
plot_joints(g["t"], g["q"], G1_JOINTS, ax=ax[0], title="G1: left leg", ids=[G1_JOINT_ID[n] for n in ("left_hip_pitch", "left_knee", "left_ankle_pitch")])
plot_joints(g["t"], g["q"], G1_JOINTS, ax=ax[1], title="G1: arms", ids=[G1_JOINT_ID[n] for n in ("left_shoulder_pitch", "right_shoulder_pitch", "right_shoulder_roll")])
plt.tight_layout(); plt.show()
dtg = np.diff(g["t"]) * 1e3
print(f"G1 /lowstate (录制时): {1e3 / dtg.mean():.0f} Hz, 最坏间隔 {dtg.max():.0f} ms  |  Go2: {1 / np.diff(d['t']).mean():.0f} Hz")

# %% [markdown]
# 对着 09、10 节的时间线，在图上认出四件事：站起、行走、挥手（哪只手？）、`arm_sdk` 抬臂（哪只手？）。
# G1 的 `/lowstate` 频率比 Go2 的低——同一台电脑生成，消息大一倍（35 个电机槽位），所以更慢。
#
# ## 小结
# - 频率、抖动、延迟是三件事。报告时序至少给：平均频率、中位间隔、最坏间隔。
# - 负载上来时先变坏的是"尾巴"。控制按最坏情况设计。
# - 时钟不同步时测不了绝对延迟，但能用"到达时刻 − 发送端计数"看延迟的变化和时钟漂移。
# - 消息越大越慢，在真实网络上尤其明显。
#
# ## 课后练习
# 1. 给 `RateMeter` 加一个方法 `report(window_s)`：只统计最近 `window_s` 秒。用它做一个每秒刷新的"频率仪表"。
# 2. 用第 3 部分的方法测 RPC 的往返时间：从 `go2_demo` 这个 bag 的 `/api/sport/request` 和 `/api/sport/response` 里按 `id` 配对，算 `StandUp` 的应答用了多久。（提示：`common.bagio.read_bag`。）
# 3. 连真机后重跑第 1、2 部分，把 `/lowstate`、`/sportmodestate`、`/utlidar/cloud` 的实测频率和最坏间隔填进真机待验清单。分别用网线和 Wi-Fi 各测一次。
#
# **下一节**：这一节的很多分析用的是事先录好的数据。数据是怎么录的、录下来是什么格式、怎么原样回放？
