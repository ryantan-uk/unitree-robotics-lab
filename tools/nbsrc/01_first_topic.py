# %% [markdown]
# # 01 · 机器人现在站没站稳？
#
# **Part 1 用 ROS 2 读懂机器人 · 第 2 节**
#
# 上一节你用命令行"听见"了机器人。这一节**自己写一个订阅者**，把 `/lowstate` 里的数变成一个判断：
# **它现在是站着，还是趴着？**
#
# 学完这一节你能：
# 1. 写出一个 ROS 2 订阅节点，并说清回调是什么时候被调用的；
# 2. 从 `LowState` 里找到 IMU 姿态和足端力，写出"站没站稳"的判据；
# 3. 解释 QoS 不匹配为什么会"一条都收不到"，队列太短为什么会"丢一部分"。
#
# 没有真机时，本节回放 `data/bags/go2_demo`（**合成数据**）：机器人先趴着，然后站起、前进、转弯、前进、停下，循环播放。
# 连着真机时，请在运行第 2 部分的录制单元期间，用遥控器让它站起来走几步。

# %%
LIVE = False

import time
import numpy as np
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node

from common import connect
from common.rosutil import ros_init, spin_for, sh

lab = connect("go2", live=LIVE, sdk=False)
ros_init()

# %% [markdown]
# ## 1. 先跑通：第一个订阅者
#
# 一个订阅节点只有三样东西：**Topic 名、消息类型、回调函数**。每来一条消息，ROS 2 就调用一次回调。

# %%
from unitree_go.msg import LowState


class FirstListener(Node):
    def __init__(self):
        super().__init__("first_listener")
        self.count = 0
        self.last = None
        #                         类型       Topic 名     回调           队列深度
        self.create_subscription(LowState, "/lowstate", self.on_state, 10)

    def on_state(self, msg):
        self.count += 1
        self.last = msg


node = FirstListener()
spin_for(node, seconds=2.0)        # "spin" = 让节点开始处理消息；这里转 2 秒
print(f"2 秒内收到 {node.count} 条 /lowstate")
msg = node.last

# %% [markdown]
# **停一下：** 如果把 `spin_for` 那一行删掉，`node.count` 会是多少？为什么？
# （回调不会自己跑。节点只有在被 *spin* 的时候才去取消息、调回调。这是初学者最常见的"我明明订阅了却没数据"。）
#
# 看看这条消息里有什么。

# %%
imu = msg.imu_state
print("tick (ms)       :", msg.tick)
print("欧拉角 rpy (rad) :", np.round(imu.rpy, 3))
print("四元数           :", np.round(imu.quaternion, 3))
print("角速度 (rad/s)   :", np.round(imu.gyroscope, 3))
print("加速度 (m/s^2)   :", np.round(imu.accelerometer, 2))
print("足端力 FR FL RR RL:", list(msg.foot_force))
print("电机数组长度      :", len(msg.motor_state), " (Go2 只用前 12 个)")
print("0 号电机          : q=%.3f rad  dq=%.3f rad/s  tau=%.2f N·m  温度=%d °C" % (
    msg.motor_state[0].q, msg.motor_state[0].dq, msg.motor_state[0].tau_est, msg.motor_state[0].temperature))

# %% [markdown]
# ## 2. 再拆开：怎么判断"站稳"？
#
# 先凭直觉想：一只四足机器人"站稳了"，至少要满足什么？
#
# - **机身大致水平** → 看 IMU 的 roll、pitch；
# - **四只脚都踩着地、撑着体重** → 看足端力 `foot_force`。
#
# 只看姿态够不够？趴在地上时机身也是水平的。所以两个条件都要。
# 先别急着定阈值——**录一段数据画出来看**，阈值是从数据里读出来的。

# %%
class Recorder(Node):
    """把每条消息里我们关心的几个数存下来。"""

    def __init__(self, msg_type):
        super().__init__("recorder")
        self.rows = []
        self.create_subscription(msg_type, "/lowstate", self.on_state, 50)

    def on_state(self, msg):
        self.rows.append((msg.tick * 1e-3, *msg.imu_state.rpy, *msg.foot_force))


rec = Recorder(LowState)
spin_for(rec, seconds=16.0)                 # 录满一整段 (bag 长约 14.5 s)
data = np.array(rec.rows)
data = data[np.argsort(data[:, 0])]         # bag 循环播放，按机器人自己的时钟 tick 排序
t, rpy, foot = data[:, 0] - data[0, 0], data[:, 1:4], data[:, 4:8]
print("录到", len(t), "条，覆盖机器人时间", round(t[-1], 1), "s")

fig, ax = plt.subplots(2, 1, figsize=(9, 5), sharex=True)
ax[0].plot(t, np.degrees(rpy[:, 0]), label="roll"); ax[0].plot(t, np.degrees(rpy[:, 1]), label="pitch")
ax[0].set_ylabel("deg"); ax[0].legend(); ax[0].grid(alpha=.3); ax[0].set_title("IMU attitude")
for i, leg in enumerate(["FR", "FL", "RR", "RL"]):
    ax[1].plot(t, foot[:, i], label=leg, lw=.8)
ax[1].set_ylabel("foot force (raw)"); ax[1].set_xlabel("robot time [s]"); ax[1].legend(ncol=4); ax[1].grid(alpha=.3)
plt.tight_layout(); plt.show()

# %% [markdown]
# **看图回答（先自己答，再往下）：**
# 1. 机器人大约在第几秒站起来的？你是从哪条曲线看出来的？
# 2. 站着不动时，每只脚的读数大约是多少？走起来以后为什么变成了"方波"？
# 3. roll、pitch 全程都在 ±2° 以内——所以单靠姿态**分不出**趴着和站着。
#
# 走路时每只脚轮流离地，单脚读数会掉到 0，但**四只脚加起来**始终撑着体重。所以判据用"总和"：

# %%
TILT_MAX_DEG = 15.0        # 倾角超过它算"歪了"
FORCE_MIN = 100.0          # 四脚足端力之和低于它算"没撑着"  (从上图读出来的: 站立时约 4 x 55)


def is_standing(roll, pitch, foot_force):
    tilt = np.degrees(np.arccos(np.cos(roll) * np.cos(pitch)))     # 机身 z 轴与竖直方向的夹角
    return (tilt < TILT_MAX_DEG) & (np.sum(foot_force, axis=-1) > FORCE_MIN)


standing = is_standing(rpy[:, 0], rpy[:, 1], foot)
fig, ax = plt.subplots(figsize=(9, 2.4))
ax.plot(t, foot.sum(axis=1), "k", lw=.8, label="sum of foot force")
ax.axhline(FORCE_MIN, color="r", ls="--", label="threshold")
ax.fill_between(t, 0, foot.sum(axis=1).max(), where=standing, alpha=.15, color="g", label="is_standing = True")
ax.set_xlabel("robot time [s]"); ax.legend(loc="lower right", fontsize=8); ax.grid(alpha=.3); plt.show()
print(f"判定为站立的时间占 {standing.mean():.0%}；第一次判定为站立是在 t = {t[np.argmax(standing)]:.2f} s")

# %% [markdown]
# > **关于单位**：`foot_force` 是传感器原始读数，不是牛顿；这里的数值是合成的，**真机上的量级会不同**。
# > 所以阈值永远要像上面那样，从你自己那台机器的数据里读出来。
#
# ## 3. 改坏它：QoS
#
# 订阅时最后那个参数 `10` 是队列深度，它是 **QoS**（服务质量）的一部分。QoS 是发布者和订阅者之间的"合同"，
# 双方条款对不上，就**一条都收不到，而且不报错**。
#
# 先看发布方开出的条款：

# %%
_ = sh("ros2 topic info /lowstate -v | grep -E 'Node name|Reliability|Durability|History' | head -n 8")

# %% [markdown]
# **预测一**：发布方的 Durability 是 `VOLATILE`（"不为晚来的人保留历史"）。
# 如果订阅方坚持要 `TRANSIENT_LOCAL`（"我要历史消息"），2 秒内能收到几条？

# %%
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy


def count_with(qos, seconds=2.0, slow=0.0):
    n = Node("qos_probe_%d" % int(time.time() * 1e3 % 1e6))
    ticks = []

    def cb(m):
        ticks.append(m.tick)
        if slow:
            time.sleep(slow)
    n.create_subscription(LowState, "/lowstate", cb, qos)
    spin_for(n, seconds)
    n.destroy_node()
    return ticks


strict = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
print("要求 TRANSIENT_LOCAL :", len(count_with(strict)), "条")
print("默认 (VOLATILE)      :", len(count_with(10)), "条")

# %% [markdown]
# 0 条。规则是：**订阅方的要求不能比发布方提供的更高**。
# 发布方 `BEST_EFFORT` + 订阅方 `RELIABLE` 也是同样的结果。连真机收不到数据时，这是第二个要查的地方（第一个是上一节的三个条件）。
#
# **预测二**：QoS 匹配，但回调很慢（每条处理 20 ms），队列深度只有 1。
# 消息以每秒一两百条的速度到来，会发生什么？

# %%
def report(name, ticks):
    gaps = np.diff(sorted(set(ticks)))
    gaps = gaps[(gaps > 0) & (gaps < 1000)]
    print(f"{name:<26} 收到 {len(ticks):>4} 条, 相邻两条间隔 中位数 {np.median(gaps):5.1f} ms, 最大 {gaps.max():5.0f} ms")


report("快回调, 队列 10", count_with(10))
report("慢回调 20 ms, 队列 1", count_with(1, slow=0.02))
report("慢回调 20 ms, 队列 100", count_with(100, slow=0.02))

# %% [markdown]
# 慢回调时收到的条数大幅减少——**处理不过来的消息被丢掉了**。
# 队列加深并不能让你"处理得更快"，只是把还没处理的消息多存一会儿（于是你处理的是**更旧**的数据）。
#
# 对控制来说，旧数据比没数据更危险。所以结论不是"把队列调大"，而是：**回调里不要做慢的事**。
# 那慢的事（画图、存盘、跑算法）放哪儿？——这是第 03 节的问题。
#
# ## 4. 换成 G1 再做一遍
#
# 把同样的 `Recorder` 用到 G1 上。先预测：哪一行会出问题？

# %%
lab.close()
lab = connect("g1", live=LIVE, sdk=False)
from unitree_hg.msg import LowState as G1LowState

rec_g1 = Recorder(G1LowState)
try:
    spin_for(rec_g1, seconds=1.0)
except AttributeError as e:
    print("出错了:", e)
rec_g1.destroy_node()

# %% [markdown]
# G1 的 `LowState` **没有 `foot_force`**——人形机器人脚底没有这个传感器字段。
# 这不是 bug，是两种机器人的真实差异：*同一个问题（站没站稳），要用不同的数据来回答。*
#
# G1 能用什么？姿态仍然有；"是不是撑着"可以看**膝关节角度**（蹲着时膝盖弯得厉害）。

# %%
class G1Recorder(Node):
    def __init__(self):
        super().__init__("g1_recorder")
        self.rows = []
        self.create_subscription(G1LowState, "/lowstate", self.on_state, 50)

    def on_state(self, msg):
        knee = (msg.motor_state[3].q + msg.motor_state[9].q) / 2       # 3 = 左膝, 9 = 右膝
        self.rows.append((msg.tick * 1e-3, *msg.imu_state.rpy, knee))


g1 = G1Recorder()
spin_for(g1, seconds=24.0)                  # G1 这段数据长约 23 s
d = np.array(g1.rows); d = d[np.argsort(d[:, 0])]
tg, knee = d[:, 0] - d[0, 0], d[:, 4]
fig, ax = plt.subplots(figsize=(9, 2.4))
ax.plot(tg, knee, label="mean knee angle [rad]")
ax.set_xlabel("robot time [s]"); ax.grid(alpha=.3); ax.legend(); plt.show()
print("膝关节角: 起始 %.2f rad -> 结束 %.2f rad" % (knee[:20].mean(), knee[-20:].mean()))

# %%
lab.close()

# %% [markdown]
# ## 小结
#
# - 订阅者 = Topic 名 + 类型 + 回调；**不 spin 就没有回调**。
# - "站没站稳"（Go2）= 倾角小 **且** 四脚足端力之和够大；阈值从数据里读，不要抄。
# - QoS 不兼容 → 0 条且不报错；回调慢 → 丢消息。
# - G1 没有足端力，同一个问题要换数据回答。
#
# ## 课后练习
# 1. 给 G1 写一个 `g1_is_standing(roll, pitch, knee)`，阈值从上面的图里读。
# 2. 把 `FORCE_MIN` 改成 150，再改成 30，各看一次绿色区域怎么变。哪个更容易"误判成没站稳"？在真机上你宁可误判成哪一种？
# 3. 用 `ros2 topic echo /sportmodestate --once` 找一个能直接告诉你机身高度的字段。用它判断"站没站起来"是不是更简单？它有什么隐患？（提示：它是谁算出来的？）
#
# **下一节**：`LowState` 里有一个 40 字节的 `wireless_remote`，是遥控器的原始字节。怎么知道按了哪个键？
