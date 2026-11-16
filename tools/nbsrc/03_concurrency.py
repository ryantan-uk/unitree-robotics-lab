# %% [markdown]
# # 03 · 几路数据同时来，怎么一个都不耽误？
#
# **Part 1 用 ROS 2 读懂机器人 · 第 4 节**
#
# 机器人同时在发好几路数据，快慢差别很大：
#
# | Topic | 大约多快 | 一条多大 |
# |---|---|---|
# | `/lowstate` | 每秒几百条 | 1 KB |
# | `/sportmodestate` | 每秒 50 条 | 很小 |
# | `/utlidar/cloud` | 每秒 5 帧 | 二十几 KB，而且**处理起来慢** |
#
# 01 节末尾我们看到：回调一慢就丢消息。这一节正面解决它——
# **处理点云很慢，怎么不让它拖累 `/lowstate`？**
#
# 学完这一节你能：
# 1. 用数据说明"单线程执行器里，一个慢回调会饿死所有回调"；
# 2. 用回调组 + 多线程执行器把快慢回调隔开；
# 3. 用"工作线程 + 只留最新一条"的模式处理重活；
# 4. 说清楚什么时候用线程、什么时候用 `asyncio`。

# %%
LIVE = False

import asyncio
import queue
import threading
import time

import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

from common import connect
from common.rosutil import ros_init, spin_for
from unitree_go.msg import LowState, SportModeState

lab = connect("go2", live=LIVE, sdk=False)
ros_init()

HEAVY = 0.15          # 假装"处理一帧点云"要 150 ms (真实的滤波、配准只会更慢)


def process_cloud(msg):
    time.sleep(HEAVY)
    return msg.width


# %% [markdown]
# ## 1. 先跑通（然后发现问题）：最直接的写法
#
# 三个订阅，三个回调，点云回调里直接做重活。`spin_for(..., threads=1)` 用的是默认的**单线程执行器**。
# 我们记录每条 `/lowstate` **到达回调的时刻**，看它们之间的间隔。

# %%
class Naive(Node):
    def __init__(self, low_group=None, cloud_group=None):
        super().__init__("naive_%d" % (time.time() * 1e3 % 1e6))
        self.low_times, self.sport_n, self.cloud_n = [], 0, 0
        self.create_subscription(LowState, "/lowstate", self.on_low, 10, callback_group=low_group)
        self.create_subscription(SportModeState, "/sportmodestate", self.on_sport, 10, callback_group=low_group)
        self.create_subscription(PointCloud2, "/utlidar/cloud", self.on_cloud, 5, callback_group=cloud_group)

    def on_low(self, msg):
        self.low_times.append(time.monotonic())

    def on_sport(self, msg):
        self.sport_n += 1

    def on_cloud(self, msg):
        process_cloud(msg)
        self.cloud_n += 1


def report(name, node, seconds):
    gaps = np.diff(node.low_times) * 1e3
    print(f"{name:<28} lowstate {len(node.low_times) / seconds:6.1f} 条/s | 最长一次没收到 lowstate: {gaps.max():6.1f} ms"
          f" | sport {node.sport_n / seconds:5.1f}/s | cloud {node.cloud_n / seconds:4.1f}/s")
    return gaps


SECONDS = 6.0
naive = Naive()
spin_for(naive, SECONDS, threads=1)
gaps_naive = report("单线程, 回调里做重活", naive, SECONDS)

# %% [markdown]
# **停一下，看"最长一次没收到 lowstate"那个数。** 它大约等于什么？
#
# 约等于 `HEAVY`（150 ms）。单线程执行器一次只能跑一个回调：点云回调睡 150 ms 的时候，
# 几十条 `/lowstate` 在队列里排队，队列只有 10 个位置，排不下的就丢了。
#
# 如果你在用这个姿态数据做平衡控制，**每秒有 5 次、每次 150 ms 是瞎的**。
#
# ## 2. 拆开：执行器和回调组
#
# - **执行器（executor）** 决定用几个线程来跑回调。
# - **回调组（callback group）** 决定哪些回调**不许同时跑**：
#   - `MutuallyExclusiveCallbackGroup`：组内的回调互斥，一次只跑一个（默认所有回调都在同一个这样的组里）；
#   - `ReentrantCallbackGroup`：组内的回调可以并行，甚至同一个回调可以同时跑好几份。
#
# 思路：把"快的"（lowstate、sport）放一组，"慢的"（点云）放另一组，再给执行器 2 个以上线程。
# 两组之间互不阻塞。
#
# **预测**：这样改之后，"最长一次没收到 lowstate" 会变成多少？点云的处理帧率会变吗？

# %%
split = Naive(low_group=MutuallyExclusiveCallbackGroup(), cloud_group=MutuallyExclusiveCallbackGroup())
spin_for(split, SECONDS, threads=3)
gaps_split = report("多线程 + 两个回调组", split, SECONDS)

# %%
import matplotlib.pyplot as plt

fig, ax = plt.subplots(figsize=(9, 2.8))
ax.plot(np.cumsum(gaps_naive) / 1e3, gaps_naive, label="single thread", lw=.8)
ax.plot(np.cumsum(gaps_split) / 1e3, gaps_split, label="2 callback groups + 3 threads", lw=.8)
ax.set(xlabel="time [s]", ylabel="gap between lowstate callbacks [ms]"); ax.legend(); ax.grid(alpha=.3); plt.show()

# %% [markdown]
# 尖刺没了。点云还是每秒处理 5 帧左右（它本来就只来这么快），但不再拖累别人。
#
# ### 改坏它：只换执行器，不分回调组
#
# **预测**：如果只把线程数改成 3，但三个回调还在默认的同一个组里，尖刺会消失吗？

# %%
same_group = Naive()                         # 不传回调组 -> 全在默认的互斥组里
spin_for(same_group, SECONDS, threads=3)
_ = report("多线程, 但还在同一个组", same_group, SECONDS)

# %% [markdown]
# 没有消失。**线程再多，同一个互斥组里的回调也只能排队。** 多线程执行器和回调组必须一起用。
#
# （那全放进一个 `ReentrantCallbackGroup` 呢？尖刺也会消失，但同一个回调可能同时被两个线程执行，
# 你在回调里改的每个变量都要加锁。能用互斥组分开就别用可重入组。）
#
# ## 3. 另一种做法：回调只"收"，重活交给工作线程
#
# 如果重活比数据来得还慢（比如处理一帧要 300 ms，数据 200 ms 来一帧），上面的做法会越积越多。
# 更稳的模式是：
#
# - 回调里**只把消息放进一个容量为 1 的队列**（满了就把旧的扔掉）——回调永远是微秒级；
# - 另起一个普通线程，从队列里取**最新的一帧**慢慢处理。
#
# 这样处理的永远是最新数据，慢了就自动跳帧，不会积压。

# %%
class LatestOnly(Node):
    def __init__(self):
        super().__init__("latest_only")
        self.low_times = []
        self.sport_n = self.cloud_n = self.cloud_in = 0
        self.box = queue.Queue(maxsize=1)                 # 只留最新一帧
        self.lock = threading.Lock()                      # 保护 cloud_n (工作线程写, 主线程读)
        self.running = True
        self.create_subscription(LowState, "/lowstate", lambda m: self.low_times.append(time.monotonic()), 10)
        self.create_subscription(SportModeState, "/sportmodestate", self.on_sport, 10)
        self.create_subscription(PointCloud2, "/utlidar/cloud", self.on_cloud, 5)
        self.worker = threading.Thread(target=self.work, daemon=True)
        self.worker.start()

    def on_sport(self, msg):
        self.sport_n += 1

    def on_cloud(self, msg):                              # 回调: 只放进盒子，立刻返回
        self.cloud_in += 1
        try:
            self.box.get_nowait()                         # 盒子里有旧的 -> 扔掉
        except queue.Empty:
            pass
        self.box.put_nowait(msg)

    def work(self):                                       # 工作线程: 慢慢处理
        while self.running:
            try:
                msg = self.box.get(timeout=0.2)
            except queue.Empty:
                continue
            process_cloud(msg)
            with self.lock:
                self.cloud_n += 1


HEAVY = 0.30                                              # 故意比数据来得慢 (200 ms 一帧)
latest = LatestOnly()
spin_for(latest, SECONDS, threads=1)                      # 注意: 单线程执行器就够了
latest.running = False; latest.worker.join()
_ = report("单线程 + 工作线程", latest, SECONDS)
print(f"点云: 收到 {latest.cloud_in} 帧, 处理了 {latest.cloud_n} 帧, 主动跳过 {latest.cloud_in - latest.cloud_n} 帧 (处理不过来就丢旧的)")
HEAVY = 0.15

# %% [markdown]
# **停一下：** `self.lock` 保护的是什么？如果去掉它，这个程序大概率还是对的——那为什么还要加？
# （`cloud_n += 1` 是"读-加-写"三步，两个线程交错时可能丢一次计数。Python 的 GIL 让这种事很少发生，但不是不发生；
# 而且换成"更新一个字典里的三个字段"时，读的人就可能看到更新了一半的状态。**跨线程共享的可变数据都加锁**，是不需要动脑子的好习惯。）
#
# ## 4. `asyncio`：同时"等"很多件事
#
# 线程适合"有活要干"；如果只是**在等**（等应答、等超时、等网络），`asyncio` 更轻：一个线程里同时等很多件事。
#
# 下面模拟"向机器人的 3 个服务各问一次状态，每次要等 0.3 s 才有应答"。

# %%
async def ask(service, delay=0.3):
    await asyncio.sleep(delay)             # 代表 "请求发出去了，在等应答"；等的时候线程是空闲的
    return f"{service}: ok"


async def one_by_one():
    return [await ask(s) for s in ("sport", "motion_switcher", "robot_state")]


async def all_at_once():
    return await asyncio.gather(*(ask(s) for s in ("sport", "motion_switcher", "robot_state")))


for fn in (one_by_one, all_at_once):
    t0 = time.monotonic()
    result = await fn()                    # Jupyter 里可以直接 await；普通脚本里写 asyncio.run(fn())
    print(f"{fn.__name__:<12} 用时 {time.monotonic() - t0:.2f} s  {result}")

# %% [markdown]
# 一个一个问要 0.9 s，同时问只要 0.3 s。
#
# rclpy 的回调也可以写成 `async def`，在里面 `await` 一个服务调用的结果——下一节调用 Service 时会用到。
#
# | 你要做的事 | 用什么 |
# |---|---|
# | 多路数据的回调互不阻塞 | 回调组 + `MultiThreadedExecutor` |
# | 回调之外的重活（算法、画图、存盘） | 工作线程 + 容量为 1 的队列 |
# | 同时等好几个应答 / 超时 | `asyncio`（或 rclpy 的 `async` 回调） |
# | 纯 Python 的密集计算想"并行加速" | 线程**没用**（GIL），用 NumPy 向量化或多进程 |
#
# ## 5. 换成 G1 再做一遍
#
# G1 的 `LowState` 有 35 个电机槽位，一条消息大一倍。把第 2 部分的对比在 G1 上重跑一次（数据是 `g1_demo`）。

# %%
lab.close()
lab = connect("g1", live=LIVE, sdk=False)
from unitree_hg.msg import LowState as G1LowState


class NaiveG1(Naive):
    def __init__(self, low_group=None, cloud_group=None):
        Node.__init__(self, "naive_g1_%d" % (time.time() * 1e3 % 1e6))
        self.low_times, self.sport_n, self.cloud_n = [], 0, 0
        self.create_subscription(G1LowState, "/lowstate", self.on_low, 10, callback_group=low_group)
        self.create_subscription(SportModeState, "/odommodestate", self.on_sport, 10, callback_group=low_group)
        self.create_subscription(PointCloud2, "/utlidar/cloud", self.on_cloud, 5, callback_group=cloud_group)


a = NaiveG1(); spin_for(a, 4.0, threads=1); _ = report("G1 单线程", a, 4.0)
b = NaiveG1(MutuallyExclusiveCallbackGroup(), MutuallyExclusiveCallbackGroup()); spin_for(b, 4.0, threads=3)
_ = report("G1 多线程 + 两个回调组", b, 4.0)
lab.close()

# %% [markdown]
# 结论一样——并发问题和机器人长什么样无关，只和"数据多快、处理多慢"有关。
# 唯一要改的是消息类型和里程计的 Topic 名（Go2 是 `/sportmodestate`，G1 这段数据里是 `/odommodestate`）。
#
# ## 小结
#
# - 单线程执行器里，一个慢回调会让所有回调一起等。
# - 多线程执行器**加上**回调组才能把快慢回调隔开；只加线程没用。
# - 回调只收数据、重活交给"只留最新一帧"的工作线程，是处理慢任务最稳的模式。
# - 线程用来干活，`asyncio` 用来等。
#
# ## 课后练习
# 1. 把第 3 部分的队列容量改成 100，`HEAVY` 保持 0.30。运行 20 秒后，工作线程正在处理的那一帧是多久以前收到的？（给消息打上接收时间戳，处理时算差值。）这说明"队列大一点更安全"这个直觉错在哪？
# 2. 给 `LatestOnly` 加一个 1 Hz 的定时器（`create_timer`），每秒打印一次 lowstate 的平均频率。定时器回调应该放在哪个回调组？
# 3. 用 `asyncio.wait_for` 给 `ask()` 加 0.2 s 超时，捕获 `asyncio.TimeoutError`。
#
# **下一节**：到目前为止我们只是在"听"。下一节第一次和机器人"对话"：问它一个问题（Service）、交给它一件要花时间的事（Action）、问它身体各部分在哪（TF）。
