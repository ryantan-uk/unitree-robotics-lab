# %% [markdown]
# # 13 · 写两个靠得住的节点：一个汇总状态，一个负责发指令
#
# **Part 3 做出完整应用 · 第 1 节**
#
# Part 1、2 里我们写的都是"跑一下看看"的代码。从这一节开始搭一个**能一直跑着的应用**。
# 任何机器人应用都离不开两个角色，04 节你把它们当黑盒用过，现在自己写：
#
# - **状态聚合节点**：机器人每秒吐出几百条原始数据，别的节点不想各自去解析。它把这些浓缩成一句话——*"现在一切正常 / 歪了 / 数据断了"*。
# - **控制节点**：唯一有权往 `/cmd_vel` 发指令的地方。它的首要职责不是"让机器人动"，而是**在不该动的时候保证它停**。
#
# 贯穿本节的问题：**如果机器人的数据突然断了（网线松了、程序崩了），你的控制节点多久之后会停止发指令？**
#
# 学完你能：
# 1. 写出带参数、定时器、发布/订阅的完整节点类；
# 2. 实现并**测量**一个看门狗（watchdog）；
# 3. 用一个 launch 文件把多个节点一起拉起来。

# %%
LIVE = False

import math
import time
import numpy as np
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node

from common import connect, Background
from common.rosutil import ros_init, sh, spin_for

lab = connect("go2", live=LIVE, sdk=False)
ros_init()

# %% [markdown]
# ## 1. 状态聚合节点
#
# 设计先行。输入、输出、频率：
#
# ```
#   /lowstate        (几百 Hz) ─┐
#                               ├─▶  StateAggregator  ─▶  /robot_status  (2 Hz, diagnostic_msgs/DiagnosticArray)
#   /sportmodestate  (50 Hz)  ──┘
# ```
#
# 输出用 ROS 2 的标准诊断消息：一个 `level`（OK / WARN / ERROR / **STALE**）+ 一句 `message` + 若干键值对。
# `STALE`（"数据过期"）是关键：**没有消息不等于一切正常**，聚合节点必须能说出"我已经很久没收到数据了"。
#
# 两条来自 03 节的纪律：高频回调里**只存不算**；计算放在低频定时器里。

# %%
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from unitree_go.msg import SportModeState
from common.parsing import lowstate_to_dict, tilt_deg
from common.robots import joints_of


class StateAggregator(Node):
    def __init__(self, robot="go2"):
        super().__init__("my_state_aggregator")
        # 参数: 可以在启动时用 --ros-args -p tilt_warn_deg:=10.0 改，不用改代码
        self.tilt_warn = self.declare_parameter("tilt_warn_deg", 20.0).value
        self.temp_warn = self.declare_parameter("temp_warn_c", 70.0).value
        self.stale_after = self.declare_parameter("stale_after_s", 0.5).value
        self.robot = robot
        if robot == "go2":
            from unitree_go.msg import LowState
            odom_topic = "/sportmodestate"
        else:
            from unitree_hg.msg import LowState
            odom_topic = "/odommodestate"
        self.low = None
        self.low_time = None
        self.height = float("nan")
        self.create_subscription(LowState, "/lowstate", self.on_low, 10)
        self.create_subscription(SportModeState, odom_topic, self.on_sport, 10)
        self.pub = self.create_publisher(DiagnosticArray, "/robot_status", 10)
        self.create_timer(0.5, self.publish_status)                     # 2 Hz

    def on_low(self, msg):                                              # 高频: 只存
        self.low, self.low_time = msg, time.monotonic()

    def on_sport(self, msg):
        self.height = float(msg.position[2])

    def publish_status(self):                                           # 低频: 才算
        st = DiagnosticStatus(name=f"{self.robot}/summary", hardware_id=self.robot)
        age = math.inf if self.low_time is None else time.monotonic() - self.low_time
        if age > self.stale_after:
            st.level, st.message = DiagnosticStatus.STALE, f"no lowstate for {min(age, 999):.1f} s"
        else:
            d = lowstate_to_dict(self.low, self.robot)
            tilt = float(tilt_deg(d["rpy"][0], d["rpy"][1]))
            hot = int(np.argmax(d["temp"]))
            if tilt > self.tilt_warn:
                st.level, st.message = DiagnosticStatus.ERROR, f"tilt {tilt:.1f} deg"
            elif d["temp"][hot] > self.temp_warn:
                st.level, st.message = DiagnosticStatus.WARN, f"{joints_of(self.robot)[hot]} at {d['temp'][hot]:.0f} C"
            else:
                st.level, st.message = DiagnosticStatus.OK, "ok"
            st.values = [KeyValue(key="tilt_deg", value=f"{tilt:.2f}"), KeyValue(key="height", value=f"{self.height:.2f}"),
                         KeyValue(key="max_temp", value=f"{d['temp'][hot]:.0f}")]
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = [st]
        self.pub.publish(msg)


LEVEL = {DiagnosticStatus.OK: "OK", DiagnosticStatus.WARN: "WARN", DiagnosticStatus.ERROR: "ERROR", DiagnosticStatus.STALE: "STALE"}

agg = StateAggregator("go2")
seen = []
probe = Node("probe")
probe.create_subscription(DiagnosticArray, "/robot_status", lambda m: seen.append(m.status[0]), 10)
spin_for([agg, probe], 3.0)
for s in seen[-3:]:
    print(LEVEL[s.level], "|", s.message, "|", {kv.key: kv.value for kv in s.values})

# %% [markdown]
# ## 2. 控制节点：先想清楚"什么时候不许动"
#
# ```
#   上层想要的速度 (set_target)  ─┐
#                                 ├─▶  SafeController  ─▶  /cmd_vel (10 Hz)  ─▶  robot_bridge ─▶ 机器人
#   /robot_status ───────────────┘
# ```
#
# 规则（每 0.1 s 检查一次，**任何一条不满足就发零速**）：
# 1. 最近一条 `/robot_status` 是 OK 或 WARN；
# 2. 这条状态**足够新**（`status_timeout` 秒以内）——聚合节点自己也可能死；
# 3. 上层给的目标速度**足够新**（`target_timeout` 秒以内）——上层也可能死；
# 4. 速度不超过上限。
#
# 第 2、3 条就是**看门狗**：谁不按时"喂狗"，就当它出事了。

# %%
from geometry_msgs.msg import Twist


class SafeController(Node):
    def __init__(self):
        super().__init__("my_safe_controller")
        self.max_v = self.declare_parameter("max_linear", 0.5).value
        self.max_w = self.declare_parameter("max_angular", 1.0).value
        self.status_timeout = self.declare_parameter("status_timeout_s", 1.2).value
        self.target_timeout = self.declare_parameter("target_timeout_s", 0.5).value
        self.status_level, self.status_time = None, -math.inf
        self.target, self.target_time = (0.0, 0.0), -math.inf
        self.history = []                                           # (时刻, 实际发出的线速度, 原因)
        self.create_subscription(DiagnosticArray, "/robot_status", self.on_status, 10)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_timer(0.1, self.tick)                           # 10 Hz 控制循环

    def set_target(self, v, w):
        """上层调用: 我想以线速度 v、角速度 w 运动。必须持续调用。"""
        self.target, self.target_time = (float(v), float(w)), time.monotonic()

    def on_status(self, msg):
        self.status_level, self.status_time = msg.status[0].level, time.monotonic()

    def tick(self):
        now = time.monotonic()
        v, w = self.target
        if self.status_level not in (DiagnosticStatus.OK, DiagnosticStatus.WARN):
            v, w, why = 0.0, 0.0, "robot not healthy"
        elif now - self.status_time > self.status_timeout:
            v, w, why = 0.0, 0.0, "status too old"
        elif now - self.target_time > self.target_timeout:
            v, w, why = 0.0, 0.0, "no fresh target"
        else:
            v = max(-self.max_v, min(self.max_v, v)); w = max(-self.max_w, min(self.max_w, w)); why = "ok"
        cmd = Twist(); cmd.linear.x, cmd.angular.z = v, w
        self.pub.publish(cmd)
        self.history.append((now, v, why))


ctrl = SafeController()


def run(seconds, target=None):
    """转 seconds 秒；target 不为 None 时，模拟上层每 0.1 s 喂一次目标速度。"""
    t_end = time.monotonic() + seconds
    while time.monotonic() < t_end:
        if target is not None:
            ctrl.set_target(*target)
        spin_for([agg, ctrl], 0.1)


run(2.0, target=(0.3, 0.0))
print("最近 3 拍:", [(round(v, 2), why) for _, v, why in ctrl.history[-3:]])

# %% [markdown]
# 状态正常、目标新鲜 → 放行 0.3 m/s。链路的末端是 `robot_bridge`，它收到 `/cmd_vel` 后翻译成运动请求——
# 而它默认 `enable_motion=false`，所以**三道闸**（控制节点的规则、桥的开关、离线）之下，什么都不会真的发出去。
#
# ## 3. 改坏它：把"故障"做出来，量一量反应时间
#
# ### 故障一：上层死了（不再喂目标）
#
# **预测**：停止调用 `set_target` 后，多久输出变成 0？

# %%
def reaction(t_fault):
    """故障发生后，第一条零速指令是多久之后发出的。"""
    for t, v, why in ctrl.history:
        if t > t_fault and v == 0.0:
            return t - t_fault, why
    return None, None


run(1.0, target=(0.3, 0.0))
t_fault = time.monotonic()
run(1.5, target=None)                          # 上层"死了"
dt, why = reaction(t_fault)
print(f"上层停止喂目标后 {dt:.2f} s 输出归零, 原因: {why}   (target_timeout = {ctrl.target_timeout} s)")

# %% [markdown]
# ### 故障二：机器人的数据断了
#
# 这才是开头的问题。把数据源掐掉（离线：停掉回放；真机上相当于拔网线），上层**继续**喂目标。
#
# **预测**反应时间。提示：数据断 → 聚合节点要过 `stale_after_s` 才发现 → 它每 0.5 s 才发布一次 → 控制节点每 0.1 s 才检查一次。

# %%
run(1.0, target=(0.3, 0.0))
t_fault = time.monotonic()
lab.cut()                                      # 掐断数据 (离线: 强杀回放进程; 真机: 这一刻拔网线)
run(3.0, target=(0.3, 0.0))                    # 上层毫不知情，还在要 0.3 m/s
dt, why = reaction(t_fault)
print(f"数据中断后 {dt:.2f} s 输出归零, 原因: {why}")

h = np.array([(t - t_fault, v) for t, v, _ in ctrl.history if t > t_fault - 1.0])
fig, ax = plt.subplots(figsize=(8, 2.4))
ax.step(h[:, 0], h[:, 1], where="post"); ax.axvline(0, color="r", ls="--", label="data lost")
ax.set(xlabel="time since data loss [s]", ylabel="cmd_vel linear.x [m/s]"); ax.legend(); ax.grid(alpha=.3); plt.show()

# %% [markdown]
# **这零点几秒到一秒里，机器人在"瞎走"**：以 0.3 m/s 算是二三十厘米。能不能接受取决于场景；想缩短它，该调哪些参数？各自的代价是什么？
# （`stale_after_s` 太小 → 网络抖一下就误报；状态发布频率提高 → 更多计算和带宽。没有免费的。）
#
# 把上一格的 `LIVE`/回放恢复，再看它**自己恢复**：

# %%
lab.close()
lab = connect("go2", live=LIVE, sdk=False, quiet=True)
run(2.5, target=(0.3, 0.0))
print("恢复后最近一拍:", (round(ctrl.history[-1][1], 2), ctrl.history[-1][2]))

# %% [markdown]
# **停一下：** 数据一恢复就自动继续走——这是你想要的吗？很多系统规定"故障后必须人工确认才能恢复"（锁存）。课后练习 2。
#
# ## 4. 换成 G1：只改一个参数
#
# 聚合节点里和机器人有关的只有消息类型和里程计 Topic 名（12 节的结论），已经用 `robot` 参数隔离了；控制节点完全不用改。

# %%
lab.close(); agg.destroy_node()
lab = connect("g1", live=LIVE, sdk=False, quiet=True)
agg = StateAggregator("g1")
seen.clear()
spin_for([agg, probe, ctrl], 2.5)
s = seen[-1]
print("G1:", LEVEL[s.level], "|", s.message, "|", {kv.key: kv.value for kv in s.values})

# %% [markdown]
# ## 5. 从 Notebook 到真正的节点：`ros2 run` 和 launch
#
# Notebook 里的类搬进一个 Python 包就是正式节点。仓库里对应的成品在 `src/unitree_lab_nodes/unitree_lab_nodes/`：
# `state_aggregator.py`（多了一个 Service）、`patrol_controller.py`（18 节用）。一个节点一个进程：
#
# ```bash
# ros2 run unitree_lab_nodes state_aggregator --ros-args -p robot:=g1 -p tilt_warn_deg:=15.0
# ```
#
# 节点一多就需要 **launch 文件**一次拉起。本课程的是 `src/unitree_lab_nodes/launch/stack.launch.py`：

# %%
agg.destroy_node(); ctrl.destroy_node()
stack = Background("exec ros2 launch unitree_lab_nodes stack.launch.py robot:=g1")
assert stack.wait_for("bridge up", 40), stack.output()
time.sleep(2.0)
_ = sh("ros2 node list --no-daemon; echo; ros2 param get /state_aggregator tilt_warn_deg")
seen.clear(); spin_for(probe, 2.0)
print("launch 起来的聚合节点:", LEVEL[seen[-1].level], seen[-1].message)
stack.stop(); probe.destroy_node(); lab.close()

# %% [markdown]
# ## 小结
# - 聚合节点：高频回调只存，低频定时器才算；一定要能报 **STALE**。
# - 控制节点：默认输出是**零**，所有条件都满足才放行；看门狗管"上游是不是还活着"。
# - 反应时间 = 各环节超时与周期之和，**要量，不要猜**。
# - 机器人相关的差异用参数隔离，`ros2 run` 起单个节点，launch 起一组。
#
# ## 课后练习
# 1. 给 `StateAggregator` 加一条规则：Go2 四脚足端力之和小于阈值且高度大于 0.2 m → ERROR（"悬空"）。G1 上这条规则怎么办？
# 2. 把 `SafeController` 改成**锁存**：一旦因为 "robot not healthy" 或 "status too old" 停过，必须调用 `reset()` 才恢复。
# 3. 把故障二的反应时间压到 0.5 s 以内。你改了哪些参数？然后把回放速度调成 0.2 倍（数据变稀），看会不会误报。
#
# **下一节**："反应时间要量"——那系统里其他东西呢？每个 Topic 实际多少 Hz、抖不抖、一条消息从发出到收到要多久？
