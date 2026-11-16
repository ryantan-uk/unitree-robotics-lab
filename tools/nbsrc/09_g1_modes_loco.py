# %% [markdown]
# # 09 · G1：先切对模式，才走得起来
#
# **Part 2 用 SDK2 驱动机器人 · 第 5 节（G1）**
#
# 06 节让 Go2 站起来只用了一行 `StandUp()`。把同样的想法搬到人形机器人 G1 上，你会发现**没有 `StandUp` 这个 API**。
# G1 的运动控制是一台**状态机（FSM）**：零力矩 → 阻尼 → 站起 → 主运控……要一步步"切"过去，切错顺序它就不理你。
#
# 这一节的任务：**写出 G1 从上电到走起来的正确指令序列，并用数据核对每一步。**
#
# 学完你能：
# 1. 说出 G1 的几个运行模式和切换顺序，每个模式对应的 FSM 编号；
# 2. 用 `LocoClient` 写出"阻尼 → 站起 → 进入主运控 → 行走 → 停"的脚本；
# 3. 指出 G1 的 `Move` 和 Go2 的 `Move` 在"多久停下"上的关键区别。
#
# > **G1 的安全要求比 Go2 高一个量级。** 人形机器人会摔，摔了会坏，也会伤人。
# > 上真机前：挂好保护吊架、遥控器在手、至少两个人在场。本节默认只打印不下发。

# %%
LIVE = False

import inspect
import json
import numpy as np
import matplotlib.pyplot as plt

from common import connect, MotionGate, DATA
from common.robots import G1_FSM, G1_JOINT_ID

lab = connect("g1", live=LIVE, bag=None)
gate = MotionGate(lab)

# %% [markdown]
# ## 1. 先跑通：G1 的"站起来"藏在哪
#
# G1 的高层运动客户端叫 `LocoClient`（出处：`unitree_sdk2py/g1/loco/g1_loco_client.py`）。
# 直接读它的源码，看几个便捷方法底下做了什么：

# %%
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

for name in ("ZeroTorque", "Damp", "Squat2StandUp", "Start", "Move", "StopMove"):
    src = inspect.getsource(getattr(LocoClient, name)).strip().splitlines()
    print(src[0].strip()); [print("     ", l.strip()) for l in src[1:]]

# %% [markdown]
# 除了 `Move`，其余全是同一件事：**`SetFsmId(某个数)`**。这些数字就是状态机的状态编号：

# %%
print(f"{'模式':<16}{'FSM id':>7}   含义")
for k, what in (("zero_torque", "电机不出力 (会瘫软!)"), ("damp", "阻尼: 电机像泡在油里，缓慢下落，安全的'软停'"),
                ("squat2standup", "蹲 <-> 站 之间切换"), ("start", "进入主运控: 之后才响应行走指令"),
                ("sit", "坐下"), ("lie2standup", "从躺姿站起 (要求地面硬、平、粗糙)")):
    print(f"{k:<16}{G1_FSM[k]:>7}   {what}")

# %% [markdown]
# 官方示例（`example/g1/high_level/g1_loco_client_example.py`）里站起来的顺序是：
#
# ```
#   Damp()  ──▶  Squat2StandUp()  ──▶  Start()  ──▶  Move(...)
#   阻尼          从蹲姿站起            主运控          行走
# ```
#
# 把它写成脚本。和 06 节一样，所有指令走 `gate.call()`。

# %%
import time

loco = LocoClient()
loco.SetTimeout(10.0)
loco.Init()


def g1_stand_and_walk():
    gate.call(loco.Damp)                         # 1. 先进阻尼: 一个确定、安全的起点
    if gate.will_send: time.sleep(0.5)
    gate.call(loco.Squat2StandUp)                # 2. 站起 (要几秒)
    if gate.will_send: time.sleep(3.0)
    gate.call(loco.Start)                        # 3. 进入主运控
    if gate.will_send: time.sleep(1.0)
    gate.call(loco.SetVelocity, 0.3, 0.0, 0.0, 3.0)    # 4. 以 0.3 m/s 前进，持续 3 秒
    if gate.will_send: time.sleep(3.5)
    gate.call(loco.StopMove)                     # 5. 停


g1_stand_and_walk()

# %% [markdown]
# **停一下：** 第 4 步为什么用 `SetVelocity(vx, vy, omega, duration)` 而不是 `Move(vx, vy, vyaw)`？再看一眼上面打印的 `Move` 源码：
#
# ```python
# def Move(self, vx, vy, vyaw, continous_move=False):
#     duration = 864000.0 if continous_move else 1
# ```
#
# - 默认：这条速度指令**持续 1 秒**后自动失效；
# - `continous_move=True`：持续 **864000 秒 = 10 天**。也就是"一直走，直到你发下一条指令"。
#
# **如果你的程序在发完 `Move(0.3, 0, 0, True)` 之后崩溃了，G1 会一直走下去。**
# 所以本课程的写法是：永远用带明确时长的 `SetVelocity`，或者不带 `True` 的 `Move`。
#
# ## 2. 拆开：对着数据核对每一步
#
# `data/bags/g1_demo`（**合成**）是执行一段类似脚本时录的。把请求和状态都读出来。

# %%
from common.bagio import read_bag
from unitree_sdk2py.g1.loco import g1_loco_api

bag = read_bag(DATA / "bags" / "g1_demo", ["/api/sport/request", "/api/sport/response", "/odommodestate", "/lowstate"])
t0 = bag["/lowstate"][0][0]
NAMES = {v: k.replace("ROBOT_API_ID_LOCO_", "") for k, v in vars(g1_loco_api).items() if k.startswith("ROBOT_API_ID_")}
FSM_NAME = {v: k for k, v in G1_FSM.items()}
codes = {r.header.identity.id: (r.header.status.code, r.data) for _, r in bag["/api/sport/response"]}

events = []
for t, r in bag["/api/sport/request"]:
    api, p = r.header.identity.api_id, json.loads(r.parameter)
    label = NAMES.get(api, str(api))
    if api == g1_loco_api.ROBOT_API_ID_LOCO_SET_FSM_ID:
        label = f"SET_FSM_ID {p['data']} ({FSM_NAME.get(p['data'], '?')})"
    code, data = codes.get(r.header.identity.id, ("-", ""))
    events.append((t - t0, label))
    print(f"t={t - t0:5.2f} s  {label:<34} 参数 {r.parameter:<52} 应答 code={code} {data}")

# %% [markdown]
# 第一行是一次**查询**：`GET_FSM_ID` 的应答里带着当前状态编号。动手之前先问一句"你现在在哪个状态"，是好习惯。

# %%
ot = np.array([t for t, _ in bag["/odommodestate"]]) - t0
pos = np.array([list(m.position) for _, m in bag["/odommodestate"]])
lt = np.array([t for t, _ in bag["/lowstate"]]) - t0
knee = np.array([m.motor_state[G1_JOINT_ID["left_knee"]].q for _, m in bag["/lowstate"]])
knee_tau = np.array([m.motor_state[G1_JOINT_ID["left_knee"]].tau_est for _, m in bag["/lowstate"]])

fig, ax = plt.subplots(3, 1, figsize=(9, 6), sharex=True)
ax[0].plot(ot, pos[:, 2]); ax[0].set_ylabel("pelvis height [m]")
ax[1].plot(lt, knee, label="left knee angle [rad]"); ax[1].plot(lt, knee_tau / 20, label="left knee torque / 20 [N·m]", lw=.7)
ax[1].legend(fontsize=8)
ax[2].plot(ot, pos[:, 0]); ax[2].set_ylabel("x [m]"); ax[2].set_xlabel("time [s]")
for a in ax:
    a.grid(alpha=.3)
    for te, label in events:
        a.axvline(te, color="g", ls="--", lw=.8)
for te, label in events:
    ax[0].text(te, 0.5, label.split(" (")[0], rotation=90, fontsize=7, color="g")
plt.tight_layout(); plt.show()

# %% [markdown]
# **对着图回答：**
# 1. `SET_FSM_ID 706` 之后，骨盆高度从多少升到多少，用了几秒？脚本里的 `sleep(3.0)` 合适吗？
# 2. 蹲着和站直，膝关节力矩哪个大？为什么？（想想你自己半蹲和站直哪个累。）
# 3. `SET_VELOCITY` 的 `duration` 是 3.0。位置曲线在指令发出后大约几秒开始变平？

# %%
tv = next(te for te, label in events if label.startswith("SET_VELOCITY"))
x_at = lambda ts: float(np.interp(ts, ot, pos[:, 0]))          # noqa: E731
print(f"指令: 0.3 m/s × 3 s = 0.90 m;  实际: 发指令后 3 s 走了 {x_at(tv + 3) - x_at(tv):.2f} m, 最终共走了 {pos[-1, 0] - x_at(tv):.2f} m")
v = np.gradient(pos[:, 0], ot)
stopped = ot[(ot > tv + 1) & (np.abs(v) < 0.02)][0]
print(f"速度降到 0.02 m/s 以下的时刻: 指令后 {stopped - tv:.2f} s  (指令时长 3.00 s)")

# %% [markdown]
# 时长一到，速度指令自动归零——**不需要再发 `StopMove`，机器人也不会因为你的程序卡住而一直走**。这是比 Go2 的 `Move` 更安全的设计。
#
# ### 运行模式切换：`MotionSwitcherClient`
#
# 还有一层更"粗"的模式：机器人身上的**运控服务整体**是否在运行。
# 做低层关节控制（11 节）前，必须先把运控服务**释放**掉，否则你和它会抢着给同一个电机发指令。
# 这由 `MotionSwitcherClient` 负责（出处：`unitree_sdk2py/comm/motion_switcher/`），Go2 和 G1 通用：
#
# | 方法 | 作用 |
# |---|---|
# | `CheckMode()` | 查询当前运行的运控模式名；**名字为空 = 已释放** |
# | `ReleaseMode()` | 释放当前运控（机器人会失去平衡控制！必须吊着或趴着） |
# | `SelectMode(name)` | 重新选择一个运控模式 |

# %%
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient

msc = MotionSwitcherClient()
msc.SetTimeout(2.0)
msc.Init()
code, result = msc.CheckMode()                     # 只读查询，不产生运动
print("CheckMode ->", code, result)
gate.call(msc.ReleaseMode)

# %% [markdown]
# 没有真机时 `CheckMode` 返回 3102（06 节见过：没人在听）。连着真机时返回 `0` 和一个字典，里面的 `name` 就是当前模式名。
# （**模式名在不同固件上叫什么，官方源码里没有列出**——上真机后把它打印出来记下，见真机待验清单。）
#
# ## 3. 改坏它：读源码找陷阱
#
# 这一节不能靠"把参数改坏再运行"来学（没有机器人可以摔）。换一种方式：**读源码，预测后果**。

# %%
for name in ("HighStand", "LowStand", "StandUp2Squat", "WaveHand", "BalanceStand"):
    print(inspect.getsource(getattr(LocoClient, name)).strip(), "\n")

# %% [markdown]
# **预测并回答（答案在讲稿里）：**
# 1. `StandUp2Squat()` 和 `Squat2StandUp()` 发的是**同一个** FSM id（706）。那机器人怎么知道该站还是该蹲？
#    如果它已经站着，你又调了一次 `Squat2StandUp()`，会发生什么？
# 2. `HighStand()` 传的是 `UINT32_MAX`。直接调 `SetStandHeight(0.9)` 行不行？（源码没说参数的单位——**未核实，不要猜**。）
# 3. `WaveHand()` 调的是 `SetTaskId`，API 号 7106。下一节的手臂动作客户端 `ExecuteAction` 也是 7106，但服务名不同。它们会冲突吗？
#
# ## 和 Go2 对比
#
# | | Go2 (`SportClient`) | G1 (`LocoClient`) |
# |---|---|---|
# | 服务名 | `sport` | `sport`（**同名**） |
# | API 号 | 1xxx / 2xxx | 7xxx |
# | 站起来 | `StandUp()`：专门的 API 1004 | `SetFsmId(706)`：切状态机 |
# | 走之前 | 站着就行 | 必须先 `Start()`（FSM 500） |
# | 速度指令 | `Move(vx,vy,vyaw)`，不要应答，靠连续发 | `SetVelocity(vx,vy,ω,duration)`，带时长、有应答 |
# | 急停 | `Damp()` | `Damp()`（FSM 1） |
# | 里程计 Topic | `rt/sportmodestate` | 源码未核实；本课程数据里用 `rt/odommodestate` |
#
# 因为服务名相同而 API 号不同，**把 Go2 的 `SportClient` 连到 G1 上不会报连接错误，只会得到"不认识这个 API"的返回码（3203）**。
#
# ## 上真机怎么跑
# ```python
# LIVE = True
# code, fsm = loco.GetFsmId(); print(code, fsm)        # 先查询，不动
# gate.confirm("我已清场并手握急停")
# gate.call(loco.Damp)                                 # 先只做这一步，确认机器人进入阻尼
# ```
# 确认无误后再逐步放开 `g1_stand_and_walk()` 的后续几步，每放开一步都先想清楚"如果它没照做，我怎么办"。
#
# ## 小结
# - G1 的高层运动 = 切 FSM：`Damp(1)` → `Squat2StandUp(706)` → `Start(500)` → 行走。
# - 速度指令带时长；**绝不用 `Move(..., True)`**。
# - `MotionSwitcherClient` 管"运控服务在不在"，做低层控制前要 `ReleaseMode()`。
# - 动手前先查询（`GetFsmId`、`CheckMode`）。
#
# ## 课后练习
# 1. 写 `wait_fsm(loco, target, timeout)`：轮询 `GetFsmId()` 直到等于 `target`。把 `g1_stand_and_walk` 里的 `sleep` 换成它。
# 2. 从数据里量：站起过程中膝关节力矩的峰值是多少？出现在高度曲线的哪个位置？
# 3. 画出 G1 的状态机图（节点 = FSM id，箭头 = 你认为允许的切换），标出哪些箭头是你从源码/数据里**确认**的，哪些是**猜**的。
#
# **下一节**：G1 比 Go2 多了两条手臂。怎么让它挥手？怎么让手臂按你给的轨迹动，同时腿还在自己保持平衡？
