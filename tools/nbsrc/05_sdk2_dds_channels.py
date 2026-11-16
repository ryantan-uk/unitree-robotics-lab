# %% [markdown]
# # 05 · 不用 ROS 2，SDK 是怎么拿到同一份数据的？
#
# **Part 2 用 SDK2 驱动机器人 · 第 1 节（Go2 / G1 共通）**
#
# Part 1 里我们用 ROS 2 读机器人。但 Unitree 官方的开发包是 **SDK2**，它不依赖 ROS 2。
# 上一节留了个问题：`posture_server` 最终是怎么让机器人站起来的？答案在 `/api/sport/request` 里。
#
# 这一节用官方 **Python SDK（`unitree_sdk2py`）** 回答三个问题：
# 1. SDK 怎么收发数据？（通道 `Channel`）
# 2. SDK 和 ROS 2 是什么关系？（**同一条 DDS 总线上的两种说法**）
# 3. `SportClient.StandUp()` 这样的"函数调用"底下到底发生了什么？（两条 Topic 上的一问一答）
#
# 学完你能：不依赖 ROS 2，用 SDK 订阅状态、发布消息，并读懂/手写一条 RPC 请求。

# %%
LIVE = False

import json
import time
import numpy as np

from common import connect, MotionGate
from common.rosutil import collect, sh

lab = connect("go2", live=LIVE)        # 这次没有 sdk=False: connect() 顺便初始化了 SDK 的 DDS 通道
gate = MotionGate(lab)

# %% [markdown]
# ## 1. 先跑通：用 SDK 订阅 `rt/lowstate`
#
# 官方示例的写法是三步（出处：`unitree_sdk2_python/example/go2/low_level/go2_stand_example.py`）：
#
# ```python
# ChannelFactoryInitialize(0, "eth0")                         # ① 初始化: domain 0, 走哪块网卡
# sub = ChannelSubscriber("rt/lowstate", LowState_)           # ② 建通道: topic 名 + 消息类型
# sub.Init(handler, 10)                                       # ③ 挂回调 (10 = 队列长度)
# ```
#
# 第 ① 步 `connect()` 已经替你做了（整个进程只能做一次）。看看它做了什么：

# %%
import inspect
from common.connect import init_sdk
print(inspect.getsource(init_sdk))

# %%
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelPublisher
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_

got = []
sub = ChannelSubscriber("rt/lowstate", LowState_)
sub.Init(lambda msg: got.append(msg), 10)       # 回调在 SDK 自己的线程里被调用，不需要 spin
time.sleep(2.0)
m = got[-1]
print(f"2 秒收到 {len(got)} 条。最新一条: tick={m.tick} rpy={np.round(m.imu_state.rpy, 3)} foot_force={list(m.foot_force)}")

# %% [markdown]
# 对比 01 节的 ROS 2 版本：**没有节点，没有 spin**——SDK 内部起了线程替你收。字段名和 ROS 2 消息完全一样。
#
# 另一种用法是不挂回调，自己去"取"（轮询）。
# （下面偶尔会看到一行 `[Reader] take sample error`：那是 SDK 在 `Close()` 订阅时自己打印的，无害。）

# %%
sub.Close()
poll = ChannelSubscriber("rt/lowstate", LowState_)
poll.Init()                                      # 不给回调
for _ in range(3):
    msg = poll.Read(1.0)                         # 最多等 1 秒
    print("Read ->", "超时" if msg is None else f"tick={msg.tick}")
poll.Close()

# %% [markdown]
# ## 2. 拆开：SDK 和 ROS 2 看到的是同一份数据
#
# 同时用 SDK 和 ROS 2 各收 2 秒 `lowstate`，比较收到的 `tick`（机器人自己的毫秒时钟）。

# %%
from unitree_go.msg import LowState          # ROS 2 的消息类

sdk_ticks = []
sub = ChannelSubscriber("rt/lowstate", LowState_)
sub.Init(lambda msg: sdk_ticks.append(msg.tick), 100)
ros_ticks = [msg.tick for _, msg in collect("/lowstate", LowState, seconds=2.0, qos=100)]
sub.Close()
common_ticks = set(sdk_ticks) & set(ros_ticks)
print(f"SDK 收到 {len(sdk_ticks)} 条, ROS 2 收到 {len(ros_ticks)} 条, tick 完全相同的有 {len(common_ticks)} 条")

# %% [markdown]
# 是同一批消息。原因在 00 节说过：两边底下都是 CycloneDDS，
# **SDK 的 `rt/lowstate` 就是 ROS 2 的 `/lowstate`**，消息类型 `unitree_go::msg::dds_::LowState_` 也正是 ROS 2 给 `unitree_go/msg/LowState` 起的 DDS 类型名。
#
# 反过来验证一次：用 SDK **发布**一个自己的 Topic，看 ROS 2 能不能收到。

# %%
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
from std_msgs.msg import String

pub = ChannelPublisher("rt/lab/hello", String_)
pub.Init()
stop = False


def talk():
    i = 0
    while not stop:
        pub.Write(String_(f"hello from SDK #{i}")); i += 1; time.sleep(0.2)


import threading
th = threading.Thread(target=talk, daemon=True); th.start()
heard = [msg.data for _, msg in collect("/lab/hello", String, seconds=2.0)]
stop = True; th.join(); pub.Close()
print("ROS 2 在 /lab/hello 上听到:", heard[:3], "...")

# %% [markdown]
# ### 改坏它：名字对、类型不对
#
# **预测**：G1 的消息类型是 `unitree_hg` 的 `LowState_`。如果用它去订阅 Go2 的 `rt/lowstate`，会收到什么？乱码？报错？

# %%
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_ as HgLowState_

wrong = []
s2 = ChannelSubscriber("rt/lowstate", HgLowState_); s2.Init(lambda msg: wrong.append(msg), 10)
s3 = ChannelSubscriber("rt/lowstat", LowState_); s3.Init(lambda msg: wrong.append(msg), 10)     # 名字少一个字母
time.sleep(1.5); s2.Close(); s3.Close()
print("类型不对 + 名字写错, 一共收到:", len(wrong), "条")

# %% [markdown]
# 0 条，而且**没有任何报错**。DDS 的规则是"Topic 名和类型都一致才算同一个 Topic"。
# 把 Go2 的代码搬到 G1 上却忘了换消息类型，就是这个症状。
#
# ## 3. 高层接口的真相：RPC = 两条 Topic 上的一问一答
#
# SDK 里让机器人站起来只要一行 `SportClient().StandUp()`，看起来是函数调用。
# 但 DDS 只有 Topic，没有"函数"。它是这样实现的（出处：`unitree_sdk2py/rpc/client_stub.py`、`core/channel_name.py`）：
#
# ```
#   你的电脑                                                机器人
#   SportClient.StandUp()
#     └─ 往 rt/api/sport/request  发 Request {id, api_id=1004, parameter="{}"}  ─────▶  运动服务
#     ┌─ 从 rt/api/sport/response 等 Response{id 相同, status.code}             ◀─────
#     └─ 返回 code
# ```
#
# 我们的数据是在一段真实的 SDK 调用过程中录下来的，**请求和应答都在里面**。把它们抓出来看。

# %%
from unitree_sdk2py.idl.unitree_api.msg.dds_ import Request_, Response_
from unitree_sdk2py.go2.sport import sport_api

API_NAME = {v: k.replace("SPORT_API_ID_", "") for k, v in vars(sport_api).items() if k.startswith("SPORT_API_ID_")}
reqs, resps = [], []
s_req = ChannelSubscriber("rt/api/sport/request", Request_); s_req.Init(lambda r: reqs.append(r), 100)
s_res = ChannelSubscriber("rt/api/sport/response", Response_); s_res.Init(lambda r: resps.append(r), 100)
time.sleep(16.0)                                   # 听满一整段
s_req.Close(); s_res.Close()

code_of = {r.header.identity.id: r.header.status.code for r in resps}
print(f"{'api_id':>6} {'名字':<12} {'parameter (JSON)':<34} {'要应答?':<7} 应答 code")
last = None
for r in sorted(reqs, key=lambda r: r.header.identity.id):
    key = (r.header.identity.api_id, r.parameter)
    if key == last:
        continue                                   # 连续重复的只打印一次
    last = key
    rid = r.header.identity.id
    print(f"{r.header.identity.api_id:>6} {API_NAME.get(r.header.identity.api_id, '?'):<12} {r.parameter:<34} "
          f"{'否' if r.header.policy.noreply else '是':<7} {code_of.get(rid, '-')}")
print(f"\n共 {len(reqs)} 条请求, {len(resps)} 条应答")

# %% [markdown]
# **停一下，对着表回答：**
# 1. `StandUp` 的 `parameter` 是空的 `{}`，`Move` 的是什么？`x`、`y`、`z` 分别是什么物理量？（看 `sport_client.py` 里 `Move` 的三个形参名。）
# 2. 为什么请求有几十条，应答只有两三条？（`Move` 是高频连续发的，SDK 用"不要应答"的方式发它：`_CallNoReply`。）
# 3. 应答 `code = 0` 表示什么？
#
# 现在你可以**不用 SportClient**，自己拼一条请求。下面拼的和 `client_base.py` 里做的完全一样：

# %%
from unitree_sdk2py.idl.unitree_api.msg.dds_ import RequestHeader_, RequestIdentity_, RequestLease_, RequestPolicy_


def make_request(api_id, params=None, noreply=False):
    identity = RequestIdentity_(time.monotonic_ns(), api_id)        # id: 任意唯一数，用来配对应答
    header = RequestHeader_(identity, RequestLease_(0), RequestPolicy_(0, noreply))
    return Request_(header, json.dumps(params or {}), [])


req = make_request(sport_api.SPORT_API_ID_MOVE, {"x": 0.3, "y": 0.0, "z": 0.0}, noreply=True)
print("手工拼出的请求:", req.header.identity.api_id, req.parameter)

req_pub = ChannelPublisher("rt/api/sport/request", Request_); req_pub.Init()
gate.call(req_pub.Write, req)          # 受闸门保护: 没确认就不会发出去
req_pub.Close()

# %% [markdown]
# `gate.call(...)` 是本课程所有运动指令的统一出口（`common/safety.py`）：
# **没连真机、或连了但没确认，就只打印不下发。** 以后每一条会让机器人动的指令都从它走。
#
# 用官方客户端是同一件事，只是省了拼请求：

# %%
from unitree_sdk2py.go2.sport.sport_client import SportClient

client = SportClient()
client.SetTimeout(3.0)       # 等应答最多 3 秒
client.Init()
gate.call(client.StandUp)
gate.call(client.Move, 0.3, 0.0, 0.0)

# %% [markdown]
# ## 4. Python 和 C++ 各怎么调
#
# 官方还有 C++ 版的 SDK2（`unitree_sdk2`），接口一一对应。本课程用 Python 讲，下表帮你日后读 C++ 示例
# （C++ 一列出处：`unitree_sdk2/example/go2/go2_low_level.cpp`、`go2_sport_client.cpp`）。
#
# | | Python (`unitree_sdk2py`) | C++ (`unitree_sdk2`) |
# |---|---|---|
# | 初始化 | `ChannelFactoryInitialize(0, "eth0")` | `ChannelFactory::Instance()->Init(0, "eth0");` |
# | 订阅 | `sub = ChannelSubscriber("rt/lowstate", LowState_)`<br>`sub.Init(handler, 10)` | `sub.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>("rt/lowstate"));`<br>`sub->InitChannel(handler, 1);` |
# | 发布 | `pub = ChannelPublisher("rt/lowcmd", LowCmd_)`<br>`pub.Init(); pub.Write(cmd)` | `pub.reset(new ChannelPublisher<...LowCmd_>("rt/lowcmd"));`<br>`pub->InitChannel(); pub->Write(cmd);` |
# | 高层 | `c = SportClient(); c.SetTimeout(10.0); c.Init()`<br>`c.StandUp()` | `SportClient c; c.SetTimeout(10.0f); c.Init();`<br>`c.StandUp();` |
# | 取字段 | `msg.imu_state.rpy[0]` | `msg.imu_state().rpy()[0]` |
#
# 仓库里有两个**现场编译**的 C++ 对照示例（`src/unitree_lab_cpp/`，用 rclcpp 写，安装脚本里 `colcon build` 编出来的）：

# %%
_ = sh("ros2 run unitree_lab_cpp lowstate_reader --ros-args -p robot:=go2 -p count:=3", timeout=25)
_ = sh("ros2 run unitree_lab_cpp sport_request --ros-args -p api_id:=1004", timeout=25)     # 默认 dry_run: 只打印

# %% [markdown]
# ## 5. 换成 G1 再做一遍
#
# **预测**：换成 G1 后，要改哪两处？（消息类型，以及…… RPC 的服务名和 API 号会一样吗？）

# %%
lab.close()
lab = connect("g1", live=LIVE)
from unitree_sdk2py.g1.loco import g1_loco_api
from unitree_sdk2py.g1.arm import g1_arm_action_api

g1_states = []
sub = ChannelSubscriber("rt/lowstate", HgLowState_); sub.Init(lambda msg: g1_states.append(msg), 10)
g1_reqs = {"sport": [], "arm": []}
subs = []
for service in g1_reqs:
    s = ChannelSubscriber(f"rt/api/{service}/request", Request_)
    s.Init(lambda r, service=service: g1_reqs[service].append(r), 100); subs.append(s)
time.sleep(24.0)
sub.Close(); [s.Close() for s in subs]

m = g1_states[-1]
print(f"G1 lowstate: {len(g1_states)} 条, 电机槽位 {len(m.motor_state)} 个, mode_machine={m.mode_machine}, 有 foot_force 字段吗? {hasattr(m, 'foot_force')}")
NAMES = {v: k.replace("ROBOT_API_ID_", "") for mod in (g1_loco_api, g1_arm_action_api) for k, v in vars(mod).items() if k.startswith("ROBOT_API_ID_")}
for service, rs in g1_reqs.items():
    seen = []
    for r in sorted(rs, key=lambda r: r.header.identity.id):
        key = (r.header.identity.api_id, r.parameter)
        if key not in seen:
            seen.append(key); print(f"  rt/api/{service}/request  api_id={key[0]}  {NAMES.get(key[0], '?'):<28} {key[1]}")
lab.close()

# %% [markdown]
# G1 的运动服务**也叫 `sport`**，但 API 号是 7xxx 一族，而且"站起来"不是一个专门的 API，而是"把状态机切到某个编号"（`SET_FSM_ID`）。
# 手臂动作走另一个服务 `arm`。09、10 节细讲。
#
# ## 小结
#
# - SDK 收发三步：`ChannelFactoryInitialize` → `ChannelSubscriber/Publisher(名字, 类型)` → `Init`。
# - SDK 的 `rt/xxx` 与 ROS 2 的 `/xxx` 是**同一个 DDS Topic**；名字和类型都对上才通。
# - 高层接口（`SportClient` 等）= 往 `rt/api/<服务>/request` 发 JSON 请求，从 `.../response` 按 `id` 收应答。
# - 所有运动指令走 `gate.call()`，默认只打印。
#
# ## 课后练习
# 1. 用 `ChannelSubscriber` 订阅 `rt/sportmodestate`（类型 `SportModeState_`），打印 5 次位置。
# 2. 从本节抓到的 `reqs` 里统计 `Move` 请求的平均发送间隔。它是谁决定的？（看 `tools/datagen/make_data.py` 里录制时的调用方式。）
# 3. 读 `unitree_sdk2py/rpc/client_base.py` 的 `_CallBase`：请求发不出去时返回哪个错误码？发出去了但应答一直不来呢？（下一节会亲眼见到其中一个。）
#
# **下一节**：用 `SportClient` 写一段完整的动作——站起、走一个 L 形、停下——并用数据检查机器人是否照做了。
