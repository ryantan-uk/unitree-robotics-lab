# %% [markdown]
# # 18 · 综合实验：搭一条完整的"感知 → 决策 → 控制"链路
#
# **Part 3 做出完整应用 · 第 6 节（全课收尾）**
#
# 这是你自己的作品。任务书：
#
# > **让机器人在房间里自主巡逻**：向前走；前方快撞上东西了就原地左转，转到前方空旷再继续走；
# > 只要机器人状态不正常（歪了、过热、数据断了），立刻停下。
#
# 这条链路用到了前面每一节的东西：
#
# ```
#  感知                          决策                        控制
#  /utlidar/cloud ─▶ cloud_to_scan ─▶ /scan ─┐
#   (16 节)           (17 节)                  ├─▶ 你写的 decide() ─▶ /cmd_vel ─▶ robot_bridge ─▶ Move / SetVelocity
#  /lowstate ─▶ state_aggregator ─▶ /robot_status ─┘   (本节)        (13 节)      (08、12 节)      (06、09 节)
#   (01、07 节)   (13 节)
# ```
#
# 你要交付三样东西：
# 1. **感知函数** `front_distance(scan)`：前方最近的障碍物有多远；
# 2. **决策函数** `decide(state, front, healthy)`：一个状态机；
# 3. **证据**：用录制数据证明你的决策在该转弯的时候说了"转"，并通过单元测试。
#
# 没有真机时，我们用 `go2_slam` 这段数据（机器人**按固定脚本**绕房间走了一圈）来检验你的决策：
# 把当时的感知数据喂给你的 `decide()`，看它说"该转了"的时刻，和机器人实际转弯的时刻对不对得上。指令照例只打印不下发。

# %%
LIVE = False

import math
import time
import numpy as np
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from common import connect, Background, play_bag
from common.plotting import plot_room
from common.rosutil import ros_init, sh, spin_for

lab = connect("go2", live=LIVE, sdk=False, bag=None)
ros_init()

# %% [markdown]
# ## 第 1 步 · 感知：前方有多远
#
# `/scan` 是 360 个距离值，第 `i` 个对应的方位角是 `angle_min + i * angle_increment`（0 = 正前方，逆时针为正）。
# 没有回波的方向是 `inf`。
#
# **你的任务**：返回前方 ±`half_width` 扇区内的最近距离。下面是参考实现——先自己写，再对照。

# %%
def front_distance(ranges, angle_min, angle_increment, half_width=math.radians(25)):
    r = np.asarray(ranges, dtype=float)
    ang = angle_min + angle_increment * np.arange(len(r))
    sector = r[np.abs(ang) <= half_width]
    sector = sector[np.isfinite(sector)]
    return float(sector.min()) if len(sector) else math.inf


# 先用假数据测一下 (不需要机器人、不需要 ROS —— 这就是把感知写成纯函数的好处)
fake = np.full(360, 5.0); fake[180] = 1.2; fake[90] = 0.3           # 正前方 1.2 m 有东西, 右侧 0.3 m 有东西
print("前方:", front_distance(fake, -math.pi, 2 * math.pi / 360), "m   (右侧那个 0.3 m 不该算进来)")

# %% [markdown]
# **停一下：** 扇区为什么取 ±25° 而不是只看正前方那一根射线？取 ±90° 又会怎样？
# （机器人有宽度，正前方一根线会漏掉偏一点的桌腿；太宽又会把贴着走的侧墙当成"前方有障碍"。）
#
# ## 第 2 步 · 决策：一个三状态的状态机
#
# ```
#            前方 < stop_dist                    健康异常 (任何状态下)
#   FORWARD ───────────────▶ TURN            ─────────────────────▶ HALT
#      ▲                        │                                    │
#      └────────────────────────┘            健康恢复                 │
#           前方 > clear_dist                 ◀──────────────────────┘ (回到 FORWARD)
# ```
#
# 两个阈值而不是一个（`stop_dist < clear_dist`）叫**迟滞**：如果只有一个阈值，前方距离在它附近抖动时，机器人会在"走"和"转"之间来回抽搐。
#
# **你的任务**：实现 `decide`。输入当前状态、前方距离、是否健康；输出新状态和 `(线速度, 角速度)`。

# %%
def decide(state, front, healthy, stop_dist=1.4, clear_dist=2.0, v=0.35, w=0.6):
    if not healthy:
        return "HALT", (0.0, 0.0)
    if state == "HALT":
        state = "FORWARD"
    if state == "FORWARD" and front < stop_dist:
        state = "TURN"
    elif state == "TURN" and front > clear_dist:
        state = "FORWARD"
    return state, ((v, 0.0) if state == "FORWARD" else (0.0, w))


# 单元测试: 决策是纯函数，所以可以把每条规则写成一行断言 (仓库里的正式版本在 tests/test_patrol.py)
assert decide("FORWARD", 5.0, True) == ("FORWARD", (0.35, 0.0))
assert decide("FORWARD", 1.0, True)[0] == "TURN"
assert decide("TURN", 1.7, True)[0] == "TURN"            # 迟滞: 1.4 ~ 2.0 之间保持原状态
assert decide("FORWARD", 1.7, True)[0] == "FORWARD"
assert decide("TURN", 2.5, True)[0] == "FORWARD"
assert decide("FORWARD", 5.0, False) == ("HALT", (0.0, 0.0))
assert decide("TURN", 0.2, False) == ("HALT", (0.0, 0.0))  # 不健康时无条件停
assert decide("HALT", 5.0, True)[0] == "FORWARD"
print("8 条规则全部通过")

# %% [markdown]
# ## 第 3 步 · 接上感知数据：你的决策和实际对得上吗？
#
# 启动感知流水线（桥 + 点云转激光 + 状态聚合），回放 `go2_slam`，把每一帧的感知结果记下来。

# %%
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

SIM = "false" if LIVE else "true"
LOW = "/lowstate" if LIVE else "/lf/lowstate"            # go2_slam 只录了低频状态
stack = Background(f"exec ros2 launch unitree_lab_nodes stack.launch.py robot:=go2 use_sim_time:={SIM} lowstate_topic:={LOW}",
                   log="/tmp/unitree_lab_stack18.log")
assert stack.wait_for("bridge up", 60), stack.output()
time.sleep(3.0)


class Logger(Node):
    """把链路上每一环的数据都记下来。"""

    def __init__(self):
        super().__init__("capstone_logger")
        self.scan, self.odom, self.status, self.cmd = [], [], [], []
        self.create_subscription(LaserScan, "/scan", lambda m: self.scan.append(
            (time.monotonic(), front_distance(m.ranges, m.angle_min, m.angle_increment))), qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom", lambda m: self.odom.append(
            (time.monotonic(), m.pose.pose.position.x, m.pose.pose.position.y, m.twist.twist.linear.x, m.twist.twist.angular.z)), 50)
        self.create_subscription(DiagnosticArray, "/robot_status", lambda m: self.status.append(
            (time.monotonic(), m.status[0].level)), 10)
        self.create_subscription(Twist, "/cmd_vel", lambda m: self.cmd.append(
            (time.monotonic(), m.linear.x, m.angular.z)), 10)


log = Logger()
t0 = time.monotonic()
player = None if LIVE else play_bag("go2_slam", extra="--clock")
spin_for(log, 30.0)                                       # 先只听 30 秒 (大半圈, 含两次转弯)
levels = [lv for _, lv in log.status]
print(f"前 30 s: {len(log.scan)} 帧激光, {len(log.odom)} 条里程计, {len(log.status)} 条状态 (其中 OK {levels.count(DiagnosticStatus.OK)} 条)")

# %% [markdown]
# 现在把记录下来的"前方距离"逐帧喂给 `decide()`，得到一条**决策时间线**；再和机器人**实际**的角速度比。

# %%
def replay_decisions(scan_log, **kw):
    state, out = "HALT", []
    for t, front in scan_log:
        state, (v, w) = decide(state, front, True, **kw)
        out.append((t, front, state, v, w))
    return out


def turning_intervals(times, flags):
    """把布尔序列变成 [(开始, 结束), ...]"""
    out, start = [], None
    for t, f in zip(times, flags):
        if f and start is None:
            start = t
        if not f and start is not None:
            out.append((start, t)); start = None
    if start is not None:
        out.append((start, times[-1]))
    return out


def evaluate(log, **kw):
    dec = replay_decisions(log.scan, **kw)
    st = np.array([d[0] for d in dec]) - t0; front = np.array([d[1] for d in dec])
    said_turn = turning_intervals(st, [d[2] == "TURN" for d in dec])
    od = np.array(log.odom); ot = od[:, 0] - t0
    did_turn = turning_intervals(ot, np.abs(od[:, 4]) > 0.3)
    return st, front, said_turn, did_turn


def plot_eval(st, front, said_turn, did_turn, stop_dist, clear_dist, title):
    fig, ax = plt.subplots(figsize=(10, 3))
    ax.plot(st, np.clip(front, 0, 8), "k", lw=1, label="front distance [m]")
    ax.axhline(stop_dist, color="r", ls="--", lw=.8); ax.axhline(clear_dist, color="g", ls="--", lw=.8)
    for i, (a, b) in enumerate(did_turn):
        ax.axvspan(a, b, color="tab:blue", alpha=.25, label="robot actually turning" if i == 0 else None)
    for i, (a, b) in enumerate(said_turn):
        ax.plot([a, b], [7.5, 7.5], color="tab:orange", lw=6, solid_capstyle="butt", label="my decision: TURN" if i == 0 else None)
    ax.set(xlabel="time [s]", ylim=(0, 8.2), title=title); ax.legend(fontsize=8, loc="center right"); ax.grid(alpha=.3); plt.show()


st, front, said_turn, did_turn = evaluate(log)
plot_eval(st, front, said_turn, did_turn, 1.4, 2.0, "stop_dist = 1.4 m, clear_dist = 2.0 m")
print("机器人实际转弯的时段:", [(round(a, 1), round(b, 1)) for a, b in did_turn])
print("我的决策说 TURN 的时段:", [(round(a, 1), round(b, 1)) for a, b in said_turn])

# %% [markdown]
# **读图**：蓝色带是录制时机器人真的在转弯；橙色条是你的状态机说"该转了"。
#
# - 每个蓝色带附近都有一段橙色吗？（**没漏**：每次该转的时候你都说了转。）
# - 橙色是不是在蓝色**之前或同时**开始？（**不晚**：你不会比那位"司机"更晚发现墙。）
# - 有没有不在蓝色附近的橙色？（**误报**：没墙的地方说要转。）
#
# 注意这是**开环**检验：机器人并没有听你的，它按自己的脚本走。所以橙色条的**结束**时刻不必和蓝色对齐——
# 如果机器人真的听你的，它转起来以后看到的东西就和录像里不一样了。能检验的是"**触发**得对不对"。
#
# ### 改坏它：阈值调小
#
# **预测**：把 `stop_dist` 从 1.4 改成 0.9。看上图里黑线在第一个蓝色带之前降到了多少——还会触发吗？

# %%
st2, front2, said2, did2 = evaluate(log, stop_dist=0.9, clear_dist=1.4)
plot_eval(st2, front2, said2, did2, 0.9, 1.4, "stop_dist = 0.9 m, clear_dist = 1.4 m")
first_turn = did_turn[0][0]
print(f"第一次实际转弯开始时 (t={first_turn:.1f} s), 前方距离是 {front[np.searchsorted(st, first_turn)]:.2f} m")
print("stop_dist=0.9 时, 第一次转弯前后有没有触发 TURN?", any(a < first_turn + 1.0 for a, b in said2))

# %% [markdown]
# 录像里的"司机"在离墙 1.2 米左右就开始转了；阈值设成 0.9 的你会继续往前走 30 厘米才反应。
# 来得及吗？取决于速度和反应时间：0.35 m/s × 13 节量出的零点几秒 ≈ 十几厘米，再加上机器人自身的长度……
# **阈值不是拍脑袋定的，是"速度 × 最坏反应时间 + 车身 + 余量"算出来的。**
#
# ## 第 4 步 · 接上控制：让整条链路转起来
#
# 把 `decide()` 包进一个节点：订阅 `/scan` 和 `/robot_status`，10 Hz 发 `/cmd_vel`。结构和 13 节的 `SafeController` 一样，
# 只是"上层给的目标"换成了你的状态机。

# %%
class Patrol(Node):
    def __init__(self):
        super().__init__("my_patrol")
        self.state, self.front, self.front_time = "HALT", math.inf, -math.inf
        self.healthy, self.status_time = False, -math.inf
        self.transitions = []
        self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)
        self.create_subscription(DiagnosticArray, "/robot_status", self.on_status, 10)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_timer(0.1, self.tick)

    def on_scan(self, m):
        self.front, self.front_time = front_distance(m.ranges, m.angle_min, m.angle_increment), time.monotonic()

    def on_status(self, m):
        self.healthy = m.status[0].level in (DiagnosticStatus.OK, DiagnosticStatus.WARN)
        self.status_time = time.monotonic()

    def tick(self):
        now = time.monotonic()
        fresh = (now - self.front_time < 1.0) and (now - self.status_time < 1.5)      # 看门狗: 感知和状态都得是新的
        new_state, (v, w) = decide(self.state, self.front, self.healthy and fresh)
        if new_state != self.state:
            self.transitions.append((now, self.state, new_state, self.front))
            self.state = new_state
        cmd = Twist(); cmd.linear.x, cmd.angular.z = v, w
        self.pub.publish(cmd)


patrol = Patrol()
spin_for([log, patrol], 15.0)                             # 数据的最后一段: 链路全开
if player:
    player.stop()
spin_for([log, patrol], 3.0)                              # 数据放完了 (相当于数据中断)，再转 3 秒

for t, a, b, front_d in patrol.transitions:
    print(f"t={t - t0:5.1f} s  {a:<8} -> {b:<8} (前方 {front_d:.2f} m)")

# %%
cmd = np.array(log.cmd); od = np.array(log.odom)
fig, ax = plt.subplots(2, 1, figsize=(10, 4.6), sharex=True)
ax[0].step(cmd[:, 0] - t0, cmd[:, 1], where="post", label="cmd_vel linear.x [m/s]")
ax[0].step(cmd[:, 0] - t0, cmd[:, 2], where="post", label="cmd_vel angular.z [rad/s]")
ax[0].set_title("What my chain commanded (printed, not sent)"); ax[0].legend(fontsize=8); ax[0].grid(alpha=.3)
ax[1].plot(od[:, 0] - t0, od[:, 3], label="recorded robot: forward speed"); ax[1].plot(od[:, 0] - t0, od[:, 4], label="recorded robot: yaw rate")
ax[1].set(title="What the recorded robot actually did", xlabel="time [s]"); ax[1].legend(fontsize=8); ax[1].grid(alpha=.3)
plt.tight_layout(); plt.show()
print("链路末端 robot_bridge 的日志:")
print("\n".join(l for l in stack.output().splitlines() if "未下发" in l)[-300:])

# %% [markdown]
# **检查清单（对着上面的输出逐条打勾）：**
#
# - [ ] 状态切换的日志里，`FORWARD → TURN` 发生时前方距离刚好小于 1.4 m；
# - [ ] 上图：机器人实际转弯的那些时刻，你的链路也发出了"转"的指令；
# - [ ] 数据放完之后，`/cmd_vel` 在约 1 秒内归零，日志里出现 `→ HALT`（看门狗起作用了）；
# - [ ] `robot_bridge` 的日志显示"未下发"——整条链路从感知到桥都通了，只差最后一道闸。
#
# 最后一条就是本课程对"默认安全"的定义：**链路可以全开着调试，机器人一动不动，直到你明确打开开关。**
#
# ## 第 5 步 · 换成 G1：改几个字？

# %%
stack.stop(); patrol.destroy_node(); log.destroy_node()
stack = Background(f"exec ros2 launch unitree_lab_nodes stack.launch.py robot:=g1 use_sim_time:={SIM}", log="/tmp/unitree_lab_stack18_g1.log")
assert stack.wait_for("bridge up", 60), stack.output()
time.sleep(3.0)
log, patrol = Logger(), Patrol()
t0 = time.monotonic()
player = None if LIVE else play_bag("g1_demo", extra="--clock")
spin_for([log, patrol], 14.0)
if player:
    player.stop()
print(f"G1: {len(log.scan)} 帧激光, {len(log.cmd)} 条 cmd_vel; 状态切换:", [(a, b) for _, a, b, _ in patrol.transitions])
print("\n".join(l for l in stack.output().splitlines() if "未下发" in l or "bridge up" in l)[-400:])
stack.stop(); patrol.destroy_node(); log.destroy_node(); lab.close()

# %% [markdown]
# **一个字都不用改**（除了 launch 参数 `robot:=g1`）。你的感知函数、决策函数、巡逻节点对 Go2 和 G1 完全通用——
# 因为机器人之间的差异早就被 `robot_bridge`、`state_aggregator` 关在了标准消息后面（08、12、13 节）。
# 这就是"接进 ROS 2"换来的东西。
#
# 而 G1 上真正要重新考虑的是**参数**：人形机器人转弯半径、制动距离、能安全通过的宽度都不同，`stop_dist`、速度都要重新算。
#
# ## 上真机
#
# 按这个顺序，每一步确认无误再进行下一步：
#
# 1. `LIVE = True`，机器人**趴着/蹲着**不动，跑到第 3 步：确认激光、状态都有数据，前方距离的读数和卷尺量的差不多。
# 2. 人拿一块纸板在机器人前方移近移远，看第 4 步的状态切换日志对不对（机器人仍然不动）。
# 3. 让机器人站起来（06 / 09 节），场地清空，一人手持遥控器随时接管。
# 4. 另开终端重启桥：`ros2 launch unitree_lab_nodes stack.launch.py robot:=go2 enable_motion:=true`，把 `decide` 的速度先改成 `v=0.15, w=0.3`。
# 5. 运行第 4 步。观察并记录：它在离墙多远处开始转？转了多少度后继续走？和你的预测差多少？
#
# ## 你现在能做什么
#
# 回到第一节课的目标：*独立写出一个读取状态并控制 Go2 或 G1 的 ROS 2 节点。* 你刚写的 `Patrol` 就是。
# 它读激光和机器人状态，做决策，发速度指令；它知道数据断了要停；它不关心下面是四条腿还是两条腿。
#
# ## 结课作业（三选一，或自拟）
#
# 1. **沿墙走**：把决策改成"保持右侧离墙 0.6 m 前进"。需要一个新的感知函数 `right_distance`，决策输出里角速度不再只有 0 和 `w` 两个值。
# 2. **到点巡逻**：用 17 节的规划器算出到两个目标点的路径，写一个节点沿路径逐点追踪（纯追踪法），在两点之间往返。
# 3. **G1 迎宾**：当前方 1.5 m 内出现障碍物（当作有人走近）时停下并挥手（10 节的动作库），对方离开后继续巡逻。
#
# 交付物：一个能 `ros2 run` 的节点 + 决策函数的单元测试 + 一页用录制数据画的证据图（像第 3 步那样）。
#
# 评分看四件事：**链路是否完整**、**异常时是否会停**、**有没有证据**、**Go2 / G1 的差异有没有被隔离好**。
