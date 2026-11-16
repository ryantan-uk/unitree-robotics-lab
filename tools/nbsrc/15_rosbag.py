# %% [markdown]
# # 15 · 把这一刻录下来，以后想放几遍放几遍
#
# **Part 3 做出完整应用 · 第 3 节**
#
# 从 00 节起，没有真机时你用的数据都来自 `data/bags/` 里的 **Rosbag**。这一节打开这个黑盒：
# **怎么录、录下来的是什么、怎么回放、怎么读进 NumPy。**
#
# 为什么每个机器人工程师都离不开它：真机实验做一次很贵（要场地、要人看着、电池只有那么多），而且**不可重复**——
# 同一段路走两次数据不会一样。录下来，就能在办公桌上反复调算法，出了问题也有"现场"可查。
#
# 学完你能：
# 1. 录制指定 Topic，检查录得对不对；
# 2. 以任意倍速回放，并说出回放和"真的在跑"有什么区别；
# 3. 打开 bag 文件看到里面的结构，把数据读成 NumPy 数组。

# %%
LIVE = False

import shutil
import sqlite3
import time
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

from common import connect, Background, DATA, play_bag
from common.rosutil import ros_init, collect, sh

lab = connect("go2", live=LIVE, sdk=False)        # 数据源: 真机，或回放 go2_demo
ros_init()
OUT = Path("/tmp/unitree_lab_bags"); shutil.rmtree(OUT, ignore_errors=True); OUT.mkdir()

# %% [markdown]
# ## 1. 先跑通：录 5 秒
#
# 命令是 `ros2 bag record -o <目录> <topic1> <topic2> ...`，按 Ctrl-C 结束。Notebook 里用 `Background` 代替"开一个终端"。

# %%
rec = Background(f"exec ros2 bag record -o {OUT}/first /lowstate /sportmodestate")
assert rec.wait_for("Recording", 20), rec.output()
time.sleep(5.0)
rec.stop()                                        # 相当于 Ctrl-C
_ = sh(f"ls -la {OUT}/first; echo; ros2 bag info {OUT}/first")

# %% [markdown]
# 一个 bag 是一个**目录**：`metadata.yaml`（目录：有哪些 Topic、各多少条、起止时间）+ `.db3`（数据本体）。
#
# **录完第一件事永远是 `ros2 bag info`**：看条数对不对。5 秒的 `/lowstate` 应该有多少条？和上面的数字对得上吗？
#
# ## 2. 拆开：`.db3` 就是一个 SQLite 数据库
#
# 不用 ROS，用 Python 自带的 `sqlite3` 就能打开：

# %%
db = sqlite3.connect(next((OUT / "first").glob("*.db3")))
print("表:", [r[0] for r in db.execute("SELECT name FROM sqlite_master WHERE type='table'")])
print("\ntopics 表:")
for row in db.execute("SELECT id, name, type, serialization_format FROM topics"):
    print("  ", row)
print("\nmessages 表 (前 3 行):")
for mid, topic_id, ts, data in db.execute("SELECT id, topic_id, timestamp, data FROM messages LIMIT 3"):
    print(f"   id={mid} topic_id={topic_id} timestamp={ts} ns  data = {len(data)} 字节: {data[:16].hex(' ')} ...")
db.close()

# %% [markdown]
# 所以一个 bag 存的就是三样东西：**哪个 Topic、什么时刻收到的、原始字节**。
# `data` 是消息的 CDR 序列化结果——也就是这条消息在 DDS 网络上传输时的字节，原封不动。
#
# 把字节变回消息对象，需要知道类型（`topics` 表里有）和对应的消息定义（所以读 `unitree_go` 的 bag 必须装了 `unitree_go` 包）：

# %%
from rclpy.serialization import deserialize_message
from unitree_go.msg import LowState

db = sqlite3.connect(next((OUT / "first").glob("*.db3")))
topic_id = db.execute("SELECT id FROM topics WHERE name='/lowstate'").fetchone()[0]
ts, blob = db.execute("SELECT timestamp, data FROM messages WHERE topic_id=? LIMIT 1", (topic_id,)).fetchone()
db.close()
msg = deserialize_message(blob, LowState)
print(f"录制时刻 {ts * 1e-9:.3f} s  ->  tick={msg.tick}  rpy={np.round(msg.imu_state.rpy, 3)}")

# %% [markdown]
# 平时不用自己写 SQL。官方读取接口是 `rosbag2_py`，公共库把它包成了一个函数 `common.bagio.read_bag`（20 行，可以打开看）：

# %%
from common.bagio import read_bag, bag_info

for name, typ, n in bag_info(OUT / "first"):
    print(f"{name:<20}{typ:<34}{n:>6} 条")
bag = read_bag(OUT / "first")
t = np.array([ts for ts, _ in bag["/lowstate"]]); t -= t[0]
knee = np.array([m.motor_state[2].q for _, m in bag["/lowstate"]])
plt.figure(figsize=(8, 2.2)); plt.plot(t, knee); plt.xlabel("time since start of recording [s]"); plt.ylabel("FR_calf q [rad]")
plt.grid(alpha=.3); plt.show()

# %% [markdown]
# `data/go2_demo.npz` 就是这么来的：`common.bagio.bag_to_arrays()` 把 bag 里的消息逐条变成字典再堆成数组（14 节用的那个）。
#
# ## 3. 回放
#
# `ros2 bag play` 按录制时的时间间隔把消息重新发出来。对订阅者来说，**和机器人在线没有区别**——这就是没有真机时本课程能照常上的原因。
#
# 先把"现场数据源"关掉，再回放我们刚录的这 5 秒，证明数据确实来自 bag：

# %%
lab.close(); time.sleep(1.0)
print("数据源关掉后 2 秒内收到:", len(collect("/lowstate", LowState, seconds=2.0)), "条")
player = play_bag(str(OUT / "first"))
got = collect("/lowstate", LowState, seconds=6.5, qos=100)
player.stop()
n_recorded = dict((name, n) for name, _, n in bag_info(OUT / "first"))["/lowstate"]
print(f"回放期间收到 {len(got)} 条 (bag 里共 {n_recorded} 条)")

# %% [markdown]
# ### 改坏它 / 玩一玩
#
# **预测一**：`--rate 2.0` 两倍速回放，`/lowstate` 的频率变成多少？消息里的 `tick`（机器人时钟）的间隔会变吗？

# %%
def measure(rate):
    p = play_bag(str(OUT / "first"), rate=rate)
    got = collect("/lowstate", LowState, seconds=5.5 / rate + 1.0, qos=200)
    p.stop()
    arrive = np.array([ts for ts, _ in got]); ticks = np.array([m.tick for _, m in got])
    return len(got) / (arrive[-1] - arrive[0]), np.median(np.diff(ticks))


for rate in (1.0, 2.0, 0.5):
    hz, tick_dt = measure(rate)
    print(f"--rate {rate}: 到达频率 {hz:6.1f} Hz, 相邻消息的 tick 差 (中位) {tick_dt:.0f} ms")

# %% [markdown]
# 到达频率跟着倍速变，但**消息内容**（包括里面的时间戳）一个字节都不会变。
# 这带来一个陷阱：按"到达时间"算速度、算积分的代码，在倍速回放下结果会错；按消息自带时间戳算的不会。
#
# 还有一个更隐蔽的陷阱：**"现在几点"**。回放的是昨天的数据，消息里的时间戳是昨天的，而节点调用 `now()` 得到的是今天——
# TF 之类按时间戳对齐数据的组件会认为"这些数据太旧"而全部丢弃。解决办法是让所有节点用 bag 里的时间：
#
# ```bash
# ros2 bag play data/bags/go2_slam --clock          # 回放的同时往 /clock 发布"当时的时间"
# ros2 run ... --ros-args -p use_sim_time:=true     # 节点改用 /clock 作为"现在"
# ```
# 17 节建图时就必须这么做。
#
# **预测二**：录制时 Topic 名字打错一个字母，会报错吗？

# %%
lab = connect("go2", live=LIVE, sdk=False, quiet=True)
rec = Background(f"exec ros2 bag record -o {OUT}/typo /lowstat /sportmodestate")
rec.wait_for("Recording", 20); time.sleep(4.0); rec.stop()
for name, typ, n in bag_info(OUT / "typo"):
    print(f"{name:<20}{n:>6} 条")
print("录到的 Topic 里有 /lowstate 吗?", any(name == "/lowstate" for name, _, _ in bag_info(OUT / "typo")))

# %% [markdown]
# 不报错——它会耐心地等一个永远不会出现的 `/lowstat`。真机实验做完、机器人都收起来了，才发现最重要的 Topic 没录上，是很常见的事故。
# 所以：**录之前 `ros2 topic list` 复制名字，录的时候看一眼终端输出，录完立刻 `ros2 bag info`。**
#
# ## 4. 录什么、录多大
#
# 真机上 `/lowstate` 约 500 Hz、每条 1 KB 多，点云一帧几十 KB。估一下你刚录的 bag 的数据率：

# %%
size = sum(f.stat().st_size for f in (OUT / "first").iterdir())
dur = t[-1]
print(f"{size / 1e6:.2f} MB / {dur:.1f} s = {size / dur / 1e6 * 60:.0f} MB/分钟  (本次只录了 /lowstate 和 /sportmodestate)")
for name, typ, n in bag_info(DATA / "bags" / "go2_demo"):
    print(f"   go2_demo  {name:<32}{n:>6} 条")

# %% [markdown]
# | 目的 | 建议录 |
# |---|---|
# | 调状态估计 / 分析步态 | `/lowstate`（全频率），短时间 |
# | 长时间监控（温度、电池） | `/lf/lowstate`（低频版本），可以录很久 |
# | 建图、导航 | `/utlidar/cloud` `/sportmodestate` `/lf/lowstate`（17 节用的 `go2_slam` 就是这三个） |
# | 复现一次"指令 → 响应" | 再加上 `/api/sport/request` `/api/sport/response`（06 节的分析靠它） |
#
# 完整的真机录制命令在 `data/README.md`。录完用同名目录替换 `data/bags/` 下的样本，所有 Notebook 就会改用你自己的数据。
#
# ## 换成 G1
#
# 命令完全一样，只是 Topic 不同。看一眼 G1 样本里有什么：

# %%
lab.close()
for name, typ, n in bag_info(DATA / "bags" / "g1_demo"):
    print(f"{name:<32}{typ:<34}{n:>6} 条")

# %% [markdown]
# 注意 `/lowstate` 的类型是 `unitree_hg/msg/LowState`：**bag 记住了类型**，所以用 Go2 的消息类去读 G1 的 bag 会反序列化失败——又是 05 节那条规则。
#
# ## 小结
# - bag = 目录（`metadata.yaml` + SQLite `.db3`）；每行是 *Topic、到达时刻、原始字节*。
# - 录完立刻 `ros2 bag info`。名字打错不报错。
# - 回放对订阅者等价于在线；倍速只改到达节奏，不改内容；按时间戳对齐的系统要 `--clock` + `use_sim_time`。
# - 读数据用 `rosbag2_py`（或 `common.bagio.read_bag`）。
#
# ## 课后练习
# 1. 用 `ros2 bag play --topics /sportmodestate` 只回放一个 Topic，验证 `/lowstate` 确实没有了。
# 2. 写 `bag_to_csv(path, topic, fields)`：把某个 Topic 的若干字段（如 `imu_state.rpy`）导出成 CSV。
# 3. 录制时加 `--compression-mode file --compression-format zstd`，比较文件大小。压缩对 `/lowstate` 效果好还是对点云效果好？为什么？
#
# **下一节**：数字和曲线看够了。下一节把机器人"画"出来——URDF 模型、TF 坐标系树、点云。
