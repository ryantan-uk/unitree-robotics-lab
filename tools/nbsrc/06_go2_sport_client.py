# %% [markdown]
# # 06 · 让 Go2 站起来、走出去、停下来
#
# **Part 2 用 SDK2 驱动机器人 · 第 2 节（Go2）**
#
# 上一节知道了 `SportClient` 的底细。这一节用它**写一段完整的动作脚本**，然后做工程师该做的事：
# **拿数据检查机器人是不是真的照做了。**
#
# 学完你能：
# 1. 用 `SportClient` 写出"站起 → 前进 → 转弯 → 前进 → 停"的脚本，并正确处理返回码；
# 2. 解释为什么 `Move` 必须连续发；
# 3. 从录下的数据里对比"我让它怎么走"和"它实际怎么走"。
#
# > **安全**：所有运动指令都经过 `gate.call()`。没有真机时只打印；有真机时也要先 `gate.confirm(...)` 才下发。

# %%
LIVE = False

import inspect
import json
import time
import numpy as np
import matplotlib.pyplot as plt

from common import connect, MotionGate, DATA
from unitree_sdk2py.go2.sport.sport_client import SportClient

lab = connect("go2", live=LIVE, bag=None)     # 本节不需要实时回放: 分析用的是录好的整段数据
gate = MotionGate(lab)

client = SportClient()
client.SetTimeout(3.0)
client.Init()

# %% [markdown]
# ## 1. 先跑通：一段动作脚本
#
# `SportClient` 有哪些方法？直接问它（都来自源码 `unitree_sdk2py/go2/sport/sport_client.py`）：

# %%
methods = [n for n, f in inspect.getmembers(SportClient, inspect.isfunction) if not n.startswith("_") and n[0].isupper()]
print(len(methods), "个高层动作:\n", ", ".join(methods))
print()
print(inspect.getsource(SportClient.Move))

# %% [markdown]
# 本节只用最基本的几个：`StandUp` `StandDown` `Move` `StopMove` `Damp`。
# 空翻、倒立之类**不在本课程范围内**，没有老师在场不要尝试。
#
# `Move(vx, vy, vyaw)`：`vx` 前进速度 (m/s)，`vy` 向左平移速度 (m/s)，`vyaw` 逆时针转向角速度 (rad/s)，都在**机体坐标系**下。
# 它发出去的是"接下来一小段时间按这个速度走"，所以**要持续走就要持续发**。把这一点包成一个小函数：

# %%
def move_for(seconds, vx, vy, vyaw, period=0.5):
    """持续 seconds 秒，每 period 秒发一次 Move。"""
    n = max(1, round(seconds / period))
    for _ in range(n):
        gate.call(client.Move, vx, vy, vyaw)
        if gate.will_send:
            time.sleep(period)


def check(code, what):
    """返回码不是 0 就停下来: 机器人没照做的时候，继续往下发指令是危险的。"""
    if code is None:           # 被闸门拦下，没发出去
        return
    if code != 0:
        raise RuntimeError(f"{what} 失败, 返回码 {code}")


def demo():
    check(gate.call(client.StandUp), "StandUp")
    if gate.will_send:
        time.sleep(2.5)                      # 等它站稳
    move_for(4.0, 0.4, 0.0, 0.0)             # 前进 4 s
    move_for(2.0, 0.0, 0.0, 0.8)             # 原地左转 2 s
    move_for(2.0, 0.4, 0.0, 0.0)             # 再前进 2 s
    check(gate.call(client.StopMove), "StopMove")


gate.verbose = False
demo()
gate.verbose = True
sent = sum(s for _, s in gate.log)
print(f"脚本共发出 {len(gate.log)} 条指令, 其中真正下发 {sent} 条。前 4 条:")
for desc, s in gate.log[:4]:
    print("  ", "[已下发]" if s else f"[未下发·{gate.why_not}]", desc)

# %% [markdown]
# **停一下：** 脚本里 `StandUp` 之后为什么要 `sleep(2.5)`？`StandUp()` 返回 0 不就表示站好了吗？
# （返回 0 只表示"机器人**收到并接受**了这个请求"，不表示动作**做完**了。想知道站没站稳，要看状态——01 节的 `is_standing`，或 04 节的 Action。）
#
# ### 返回码
#
# 没连真机时调用一个**不会让机器人动的查询接口**，亲眼看一次"调用失败"的返回码：

# %%
client.SetTimeout(1.0)
code, data = client.AutoRecoveryGet()        # 只读查询，不产生运动
client.SetTimeout(3.0)
print("返回码:", code, " 数据:", data)

# %% [markdown]
# | 返回码 | 含义 | 出处 |
# |---|---|---|
# | 0 | 成功 | |
# | 3102 | **请求没发出去**：网络上没有任何人在听 `rt/api/sport/request`（没连上 / 机器人没开） | `unitree_sdk2py/rpc/internal.py` |
# | 3104 | **等应答超时**：有人在听，但超时时间内没回话（服务卡住 / 超时设得太短） | 同上 |
# | 3203 | 机器人不认识这个 API（固件版本不支持） | 同上 |
# | 4201 / 4202 | 运动服务超时 / 未初始化 | `go2/sport/sport_api.py` |
#
# 没有真机时上面打印的是 **3102**，并且 SDK 自己还打印了一行 `send request error`；连着真机则是 0 和查询结果。
# 记住这两个码的区别：3102 查连接，3104 查机器人那一侧。
#
# ## 2. 拆开：机器人照做了吗？
#
# 下面这段数据（`data/bags/go2_demo`，**合成**）正是执行**同一个脚本**时录下的：发出去的请求和机器人的状态都在里面。
# 用 `common.bagio.read_bag` 把它按时间顺序读出来（Rosbag 的细节 15 节讲）。

# %%
from common.bagio import read_bag

bag = read_bag(DATA / "bags" / "go2_demo", ["/api/sport/request", "/sportmodestate"])
t0 = bag["/sportmodestate"][0][0]
st = np.array([t for t, _ in bag["/sportmodestate"]]) - t0
pos = np.array([list(m.position) for _, m in bag["/sportmodestate"]])          # x, y, 高度
yaw = np.unwrap([m.imu_state.rpy[2] for _, m in bag["/sportmodestate"]])
vel_world = np.array([list(m.velocity) for _, m in bag["/sportmodestate"]])

cmd_t, cmd_v, marks = [], [], []
for t, r in bag["/api/sport/request"]:
    api = r.header.identity.api_id
    if api == 1008:                                   # Move
        p = json.loads(r.parameter); cmd_t.append(t - t0); cmd_v.append((p["x"], p["y"], p["z"]))
    else:
        marks.append((t - t0, {1004: "StandUp", 1003: "StopMove"}.get(api, str(api))))
cmd_t, cmd_v = np.array(cmd_t), np.array(cmd_v)
print(f"Move 请求 {len(cmd_t)} 条, 平均间隔 {np.diff(cmd_t).mean():.2f} s; 其他请求: {marks}")

# %%
fig, ax = plt.subplots(3, 1, figsize=(9, 6.5), sharex=True)
ax[0].plot(st, pos[:, 2]); ax[0].set_ylabel("body height [m]")
# 把世界系速度转回机体系，才能和指令 (机体系) 比
vx_body = np.cos(yaw) * vel_world[:, 0] + np.sin(yaw) * vel_world[:, 1]
ax[1].plot(st, vx_body, label="measured vx (body)"); ax[1].step(cmd_t, cmd_v[:, 0], "r.", where="post", label="commanded vx")
ax[1].set_ylabel("m/s"); ax[1].legend(loc="upper right")
ax[2].plot(st, np.gradient(yaw, st), label="measured yaw rate"); ax[2].step(cmd_t, cmd_v[:, 2], "r.", where="post", label="commanded vyaw")
ax[2].set_ylabel("rad/s"); ax[2].set_xlabel("time [s]"); ax[2].legend(loc="upper right")
for a in ax:
    a.grid(alpha=.3)
    for tm, name in marks:
        a.axvline(tm, color="g", ls="--", lw=.8)
for tm, name in marks:
    ax[0].text(tm, 0.1, name, color="g", fontsize=8, rotation=90)
plt.tight_layout(); plt.show()

# %% [markdown]
# **对着图回答：**
# 1. 从 `StandUp` 到机身高度稳定，用了多久？脚本里的 `sleep(2.5)` 够吗？
# 2. 实测速度跟上指令速度大约要多久？这个延迟对"走 4 秒"的距离有什么影响？
# 3. 每个红点是一条 `Move`。两条之间机器人为什么没有停下？
#
# ### 预测 → 验证：它走了多远、转了多少度？
#
# 按指令算：前进 0.4 m/s × 4 s = 1.6 m，左转 0.8 rad/s × 2 s = 1.6 rad ≈ 92°，再前进 0.8 m。
# **预测**实际会比这多还是少？为什么？

# %%
from common.plotting import plot_path

i_turn = np.searchsorted(st, cmd_t[np.argmax(cmd_v[:, 2] > 0)])        # 开始转弯的时刻
leg1 = np.hypot(*(pos[i_turn, :2] - pos[0, :2]))
turned = np.degrees(yaw[-1] - yaw[0])
print(f"第一段实际走了 {leg1:.2f} m (指令 1.60 m); 实际转了 {turned:.1f}° (指令 {np.degrees(1.6):.1f}°)")

# 航位推算: 只用"发出去的指令"积分出一条"我以为它会走的路"
dt = 0.02; tt = np.arange(0, st[-1], dt)
idx = np.searchsorted(cmd_t, tt, side="right") - 1
v = np.where((idx >= 0)[:, None] & ((tt - cmd_t[np.clip(idx, 0, None)]) < 1.0)[:, None], cmd_v[np.clip(idx, 0, None)], 0.0)
th = np.cumsum(v[:, 2]) * dt
dead = np.stack([np.cumsum(v[:, 0] * np.cos(th)) * dt, np.cumsum(v[:, 0] * np.sin(th)) * dt], axis=1)

ax = plot_path(pos[:, :2], label="actual (odometry)", color="tab:blue")
ax.plot(dead[:, 0], dead[:, 1], "r--", label="dead reckoning from commands"); ax.legend(fontsize=8); plt.show()
print(f"终点偏差: {np.hypot(*(dead[-1] - pos[-1, :2])):.2f} m")

# %% [markdown]
# 两条路形状一样但对不齐：**指令是"我想要的"，里程计是"实际发生的"**，中间隔着机器人的响应过程。
# 所以想让机器人走到某个**位置**，不能靠"按速度发几秒"（开环），而要一边看里程计一边调整（闭环）——这是 Part 3 导航做的事。
#
# ### 改坏它（看数据回答）
#
# 最后一条 `Move` 之后脚本发了 `StopMove`。**如果没发 `StopMove`、也不再发 `Move`，机器人会一直走下去吗？**
# 在这段合成数据所用的模型里，一条 `Move` 的有效期被设为 1 秒。**真机上这个超时是多少，官方源码里查不到——
# 这是一条"真机待验"项**（见 `docs/real_robot_checklist.md`）：发一条 `Move(0.2, 0, 0)` 后什么都不做，用里程计量它多久停下。
#
# ## 3. 上真机怎么跑
#
# ```python
# LIVE = True                                   # 第一格
# gate.confirm("我已清场并手握急停")              # 逐字确认，会打印五条检查项
# demo()                                        # 这次是真的
# gate.call(client.StandDown)                   # 做完让它趴下
# ```
# 第一次上真机：把 `demo()` 里的速度都除以 2，并且只跑到 `StandUp` 为止，确认无误再放开后面的。
#
# ## 和 G1 的不同（预告）
#
# | | Go2 `SportClient` | G1 `LocoClient`（09 节） |
# |---|---|---|
# | 站起来 | `StandUp()`，一个专门的 API | 切状态机：`Damp()` → `Squat2StandUp()` → `Start()` |
# | 走 | `Move(vx, vy, vyaw)`，不要应答 | `Move(vx, vy, vyaw)` → `SetVelocity(..., duration)`，带时长 |
# | 停 | `StopMove()` | `StopMove()` = `SetVelocity(0,0,0)` |
#
# ## 小结
# - `SportClient`：`Init` 后直接调方法；**返回 0 = 接受了，不等于做完了**。
# - `Move` 是速度指令，要持续发；3104 = 没等到应答。
# - 开环按时间走会有偏差；检查机器人是否照做，要看状态数据。
#
# ## 课后练习
# 1. 写 `square(side, speed)`：用 `move_for` 走一个边长 `side` 的正方形。先用航位推算画出"以为会走的路"。
# 2. 上图里实测 `vx` 从 0 升到指令值的 63% 用了多久？这就是速度响应的时间常数。用它修正"走 1.6 m 该发多久"。
# 3. 改写 `demo()`：`StandUp` 之后不用 `sleep`，而是订阅 `rt/sportmodestate`，等机身高度超过 0.28 m 再继续（最多等 5 秒）。
#
# **下一节**：机器人走这一趟，身体内部发生了什么？电机多热、电池掉了多少、四只脚怎么轮流着地——都在 `LowState` 里。
