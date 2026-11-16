# %% [markdown]
# # 00 · 我的电脑能"听见"机器人吗？
#
# **Part 1 用 ROS 2 读懂机器人 · 第 1 节**
#
# 机器人开机后，会不停地往网络上"广播"自己的状态：姿态、关节角、电量……
# 这一节只做一件事：**确认你的电脑能收到这些广播**，并弄清楚它是靠什么收到的。
#
# 学完这一节你能：
# 1. 说出"连上机器人"到底需要哪三个条件；
# 2. 列出机器人正在广播哪些 Topic，各是什么类型；
# 3. 故意把连接弄断，再修好。
#
# > **没有真机也能上课。** 每个 Notebook 第一格都有一个开关 `LIVE`。
# > `LIVE = True` 时连真机。`LIVE = False`（默认）时，电脑在本机回放一段录好的 Rosbag（`data/bags/`，**合成数据**），
# > 于是后面所有"读数据"的代码都照常工作；"发指令"的代码默认只打印不下发。
# > 除了这个开关，后面的代码一个字都不用改。

# %%
LIVE = False          # False: 回放 data/ 里的 Rosbag (合成数据)   True: 连真机 (先在终端跑 python3 scripts/connect.py)

from common import connect
from common.rosutil import sh, topic_table

lab = connect("go2", live=LIVE)

# %% [markdown]
# ## 1. 先跑通：机器人在广播什么？
#
# `ros2 topic list -t` 列出当前网络上所有 Topic 和它们的消息类型。

# %%
_ = sh("ros2 topic list -t --no-daemon")

# %% [markdown]
# 只看名字不够，我们还想知道**每个 Topic 有没有人在发、有没有人在听**。
# `topic_table()` 是课程公共库里的小工具（`common/rosutil.py`，一共十几行，可以打开看）。

# %%
print(f"{'Topic':<32}{'类型':<34}{'发布者':>6}{'订阅者':>6}")
for name, typ, n_pub, n_sub in topic_table():
    print(f"{name:<32}{typ:<34}{n_pub:>6}{n_sub:>6}")

# %% [markdown]
# **停一下，看表回答：**
# - `/lowstate` 和 `/sportmodestate` 听名字都是"状态"，你猜它们的区别是什么？（一个是"底层"：电机、IMU 的原始读数；一个是"运动"：位置、速度。）
# - `/api/sport/request` 和 `/api/sport/response` 成对出现——像不像"一问一答"？（第 05 节揭晓。）
# - 这张表里**没有** `/lowcmd`。真机上它是存在的（机器人在等别人往里发关节指令），第 11 节会用到。
#
# 现在真的收一条数据。`--once` 表示收到一条就退出，`--field` 只打印其中一个字段。

# %%
_ = sh("ros2 topic echo /lowstate --once --field imu_state", timeout=20)

# %% [markdown]
# 你刚刚读到了机器人的 IMU：四元数、角速度、加速度、欧拉角。**这就是"连上了"。**
#
# ## 2. 再拆开：它是靠什么连上的？
#
# 三个条件，缺一不可。逐个看。
#
# ### 条件一：说同一种"方言"——DDS 实现要一致
#
# ROS 2 底下的通信层叫 DDS，有好几家实现。Unitree 机器人内部用的是 **CycloneDDS**，
# 所以我们的 ROS 2 也必须切到 CycloneDDS。这由一个环境变量决定：

# %%
import os
print("RMW_IMPLEMENTATION =", os.environ.get("RMW_IMPLEMENTATION"))
print("ROS_DOMAIN_ID      =", os.environ.get("ROS_DOMAIN_ID"))

# %% [markdown]
# ### 条件二：在同一个"房间"——Domain ID 要一致
#
# 同一个网络上可以有很多互不干扰的 DDS"房间"，用 Domain ID 区分。Unitree 默认在 **0** 号房间。
#
# ### 条件三：走对"门"——选对网卡
#
# 你的电脑有好几块网卡（有线、无线、虚拟机的桥接网卡、回环 `lo`……）。
# DDS 必须从**连着机器人的那一块**出去。这由 `CYCLONEDDS_URI` 指向的一个 XML 文件决定：

# %%
uri = os.environ["CYCLONEDDS_URI"]
print("CYCLONEDDS_URI =", uri, "\n")
print(open(uri.replace("file://", "")).read())

# %% [markdown]
# 没连真机时网卡是 `lo`（回环：数据只在本机内部转一圈，出不了电脑，所以绝不会碰到真机）。
# 连真机时，`scripts/connect.py` 会让你选一块网卡，然后生成一个同样结构的 XML，只是 `name` 换成你选的那块。
#
# ### 一个要记住的命名规则
#
# 在 Unitree 的 SDK 文档和源码里，Topic 叫 `rt/lowstate`；在 ROS 2 里它叫 `/lowstate`。
# **是同一个东西**：ROS 2 在 DDS 层给所有 Topic 自动加前缀 `rt`（"ROS topic"）。

# %%
from common.robots import TOPICS, ros_name

print(f"{'DDS 里的名字 (SDK 用)':<30}{'ROS 2 里的名字':<26}消息类型")
for key in ("lowstate", "lowcmd", "sportmodestate", "utlidar_cloud"):
    dds, typ, _src = TOPICS[key]
    print(f"{dds:<30}{ros_name(dds):<26}{typ}")

# %% [markdown]
# ## 3. 改坏它：预测 → 验证
#
# **预测一**：如果我把 Domain ID 改成 1，还能看到机器人的 Topic 吗？先在心里回答，再运行。

# %%
_ = sh("ROS_DOMAIN_ID=1 ros2 topic list --no-daemon")

# %% [markdown]
# 只剩下 ROS 2 自带的两个 Topic。机器人还在 0 号房间里广播，只是你走进了 1 号房间。
#
# **预测二**：如果网卡名写错（比如写成一块不存在的网卡），会怎样？

# %%
bad = uri.replace("file://", "")
xml = open(bad).read().replace('name="lo"', 'name="eth99"') if not LIVE else open(bad).read().replace('name="', 'name="nope_')
open("/tmp/bad_cyclonedds.xml", "w").write(xml)
out = sh("CYCLONEDDS_URI=file:///tmp/bad_cyclonedds.xml ros2 topic list --no-daemon 2>&1 | grep -E 'does not match|failed to create domain' | head -n 3")

# %% [markdown]
# 这次不是"看不到"，而是**直接报错**——DDS 找不到这块网卡，连域都建不起来。
# 记住这两种症状的区别，以后排查连接问题能省很多时间：
#
# | 症状 | 最可能的原因 |
# |---|---|
# | 命令报错 `failed to create domain` | 网卡名写错 / XML 写错 |
# | 不报错，但只看到 `/rosout` `/parameter_events` | Domain ID 不对、选了没连机器人的网卡、或 RMW 不是 CycloneDDS |
#
# ## 4. 换成 G1 再做一遍
#
# 同样的三步，换一台人形机器人。注意看：**Topic 名字几乎一样，但类型变了。**

# %%
lab.close()
lab = connect("g1", live=LIVE)
print(f"\n{'Topic':<32}{'类型':<34}{'发布者':>6}{'订阅者':>6}")
for name, typ, n_pub, n_sub in topic_table():
    print(f"{name:<32}{typ:<34}{n_pub:>6}{n_sub:>6}")

# %% [markdown]
# **自己找不同**（答案在下一节会用到）：
# - `/lowstate` 的类型从 `unitree_go/msg/LowState` 变成了什么？
# - Go2 的 `/sportmodestate` 在 G1 上对应哪个 Topic？G1 多出来的是哪几个（提示：和手臂有关）？
#
# `ros2 interface show` 可以看一个消息类型的定义。对比两种 `LowState`：

# %%
print("=== Go2: unitree_go/msg/LowState ===")
_ = sh("ros2 interface show unitree_go/msg/LowState --no-comments | grep -v '^\\s' ")
print("\n=== G1: unitree_hg/msg/LowState ===")
_ = sh("ros2 interface show unitree_hg/msg/LowState --no-comments | grep -v '^\\s' ")

# %%
lab.close()

# %% [markdown]
# ## 小结
#
# - "连上机器人" = **同一种 DDS（CycloneDDS）+ 同一个 Domain（0）+ 走对网卡**。
# - SDK 里的 `rt/xxx` 和 ROS 2 里的 `/xxx` 是同一个 Topic。
# - Go2 的消息在 `unitree_go` 包里，G1 的在 `unitree_hg` 包里（hg = humanoid）；`MotorState[20]` 对 `MotorState[35]`，
#   G1 的 `LowState` 里没有足端力和电池。
#
# ## 连真机（有机器人时做）
#
# ```bash
# # 1. 网线连机器人；给有线网卡配一个 192.168.123.x 的静态地址（如 192.168.123.99/24）
# # 2. 选网卡、生成配置、确认收到 Topic —— 一条命令:
# python3 scripts/connect.py
# # 3. 让当前终端用上新配置，然后重启 Jupyter
# source scripts/env.sh
# ```
# 然后把本页第一格改成 `LIVE = True`，从头运行。VMware 用户注意：虚拟机网络要选**桥接模式**，并桥接到连机器人的那块物理网卡。
#
# ## 课后练习
# 1. 不看上面的表，写出"连不上"时你会检查的三样东西及对应的命令。
# 2. 另开一个终端，`source scripts/env.sh` 后运行 `ros2 bag play --loop data/bags/go2_demo`，再开一个终端用 `ros2 topic hz /lowstate` 量它的频率。把数字记下来，第 14 节会用到。
#
# **下一节**：收到数据了，但 `LowState` 里有上百个数。我们用它回答第一个真问题——*机器人现在站没站稳？*
