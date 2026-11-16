# %% [markdown]
# # 04 · 问它一句、托它办件事、问它身体在哪
#
# **Part 1 用 ROS 2 读懂机器人 · 第 5 节**
#
# Topic 是"广播"：发的人不知道谁在听，也得不到回音。但很多事需要**有来有回**：
#
# | 我想…… | 特点 | ROS 2 的机制 |
# |---|---|---|
# | 问一句"你现在怎么样？" | 问一次、答一次，很快 | **Service** |
# | 说"站起来"，并且想知道站到哪了、站没站成 | 要花几秒，中途想看进度、能取消 | **Action** |
# | 问"雷达在机身的哪儿？机身在房间的哪儿？" | 坐标系之间的关系，随时间变 | **TF** |
#
# 这一节把三种都用一遍。配合我们的是课程仓库 `src/unitree_lab_nodes/` 里的三个节点（本节把它们当黑盒用，13 节会自己写）：
#
# - `state_aggregator`：提供服务 `/get_robot_summary`；
# - `posture_server`：提供动作 `/posture`；
# - `robot_bridge`：把里程计变成 TF。
#
# > **安全**：`posture_server` 默认 `enable_motion:=false`——收到"站起来"的目标后**不会给机器人发指令**，只观察机身高度是否到位。
# > 没有真机时，高度来自回放的数据（数据里机器人每 14.5 秒重复一次"趴→站→走"）。

# %%
LIVE = False

import time
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from common import connect, Background
from common.rosutil import ros_init, sh, spin_for

lab = connect("go2", live=LIVE, sdk=False)
ros_init()


def start_nodes(robot):
    """后台启动三个课程节点 (相当于开三个终端各跑一条 ros2 run)。"""
    procs = [Background(f"exec ros2 run unitree_lab_nodes {name} --ros-args -p robot:={robot}")
             for name in ("state_aggregator", "posture_server", "robot_bridge")]
    assert procs[1].wait_for("posture server up", 30), procs[1].output()
    assert procs[2].wait_for("bridge up", 30), procs[2].output()
    time.sleep(1.0)
    return procs


procs = start_nodes("go2")
_ = sh("ros2 node list --no-daemon")

# %% [markdown]
# ## 1. Service：问一次，答一次
#
# 先看这个服务"长什么样"。`---` 上面是请求（这里为空），下面是应答。

# %%
_ = sh("ros2 interface show unitree_lab_interfaces/srv/GetRobotSummary")

# %% [markdown]
# 命令行就能调：

# %%
_ = sh("ros2 service call /get_robot_summary unitree_lab_interfaces/srv/GetRobotSummary", timeout=20)

# %% [markdown]
# 在代码里调用分三步：建客户端 → 发请求（得到一个 *future*，"将来会有结果的东西"）→ spin 到 future 完成。

# %%
from unitree_lab_interfaces.srv import GetRobotSummary

me = Node("student")
client = me.create_client(GetRobotSummary, "/get_robot_summary")
assert client.wait_for_service(timeout_sec=5.0)


def ask_summary():
    future = client.call_async(GetRobotSummary.Request())          # 立刻返回，不等
    rclpy.spin_until_future_complete(me, future, timeout_sec=3.0)  # 转到应答回来为止
    return future.result()


s = ask_summary()
print(f"{s.robot}: 站姿正常={s.upright} 倾角={s.tilt_deg:.1f}° 高度={s.height:.2f} m "
      f"最热电机={s.hottest_motor} ({s.max_motor_temp:.0f} °C) 电量={s.battery_soc:.0f}%")

# %% [markdown]
# **停一下：** `call_async` 之后为什么必须 spin？
# （应答是通过网络回来的一条消息，和 Topic 一样要靠 spin 去取。不 spin，future 永远不会完成。
# rclpy 还有一个同步的 `client.call()`——在回调里调用它，而节点又是单线程执行器时，会**死锁**：
# 它在等应答，而负责取应答的线程正被它自己占着。上一节的回调组就是为了解决这类问题。）
#
# ## 2. Action：一件要花时间的事
#
# 动作的定义有三段：**目标 / 结果 / 反馈**。

# %%
_ = sh("ros2 interface show unitree_lab_interfaces/action/Posture")

# %% [markdown]
# 发一个目标，过程是：发目标 → 服务端**接受或拒绝** → 执行中不断发**反馈** → 最后给**结果**。
# 下面把这四步写出来（`send_goal` 是通用的，后面会反复用）。

# %%
from unitree_lab_interfaces.action import Posture

posture = ActionClient(me, Posture, "posture")
assert posture.wait_for_server(timeout_sec=5.0)
STATUS = {4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED"}


def send_goal(name, timeout=20.0, cancel_after=None):
    feedback = []
    goal = Posture.Goal(posture=name, timeout=float(timeout))
    f = posture.send_goal_async(goal, feedback_callback=lambda fb: feedback.append((time.monotonic(), fb.feedback)))
    rclpy.spin_until_future_complete(me, f, timeout_sec=5.0)
    handle = f.result()
    if not handle.accepted:
        print(f"目标 '{name}': 被拒绝")
        return None, feedback
    result_future = handle.get_result_async()
    t0 = time.monotonic()
    while not result_future.done():
        rclpy.spin_once(me, timeout_sec=0.1)
        if cancel_after is not None and time.monotonic() - t0 > cancel_after:
            handle.cancel_goal_async(); cancel_after = None
    res = result_future.result()
    r = res.result
    print(f"目标 '{name}': {STATUS.get(res.status, res.status)}  success={r.success} message='{r.message}' "
          f"最终高度={r.final_height:.2f} m 用时={r.elapsed:.1f} s  (收到 {len(feedback)} 条反馈)")
    return r, feedback


# 先等机器人趴下 (回放的数据每 14.5 s 回到开头的"趴着"状态)，再请求站起来并观察全过程
_, _ = send_goal("lie", timeout=20)
result, fb = send_goal("stand", timeout=10)

# %%
import matplotlib.pyplot as plt

t = np.array([x[0] for x in fb]) - fb[0][0]
fig, ax = plt.subplots(figsize=(8, 2.6))
ax.plot(t, [x[1].height for x in fb], "o-", ms=3, label="feedback.height [m]")
ax.plot(t, [x[1].progress for x in fb], "s--", ms=3, label="feedback.progress")
ax.set_xlabel("time since goal accepted [s]"); ax.legend(); ax.grid(alpha=.3); plt.show()

# %% [markdown]
# 这就是 Action 比 Service 多出来的东西：**过程是看得见的**。
#
# 看一眼节点自己的日志，确认它确实没有给机器人发指令：

# %%
print("\n".join(l for l in procs[1].output().splitlines() if "not sent" in l or "posture server up" in l)[-600:])

# %% [markdown]
# ### 改坏它：三种不成功
#
# **预测**下面三个目标各自的结局（被拒绝 / ABORTED / CANCELED），再运行。

# %%
send_goal("fly")                              # 不存在的姿态
send_goal("lie", timeout=0.5)                 # 0.5 秒内不可能等到
_ = send_goal("lie", timeout=20, cancel_after=1.0)   # 1 秒后反悔

# %% [markdown]
# 三种失败在调用方看来是**三种不同的状态**，可以分别处理。如果只用 Topic 发一句"站起来"，这些你都无从知道。
#
# Action 并不神秘：它是 **2 个服务 + 2 个 Topic** 拼起来的（平时被隐藏了）。

# %%
_ = sh("ros2 action list -t; echo; ros2 topic list --include-hidden-topics --no-daemon | grep posture; "
       "ros2 service list --include-hidden-services | grep 'posture/_action'")

# %% [markdown]
# ## 3. TF：谁在谁的哪儿
#
# `robot_bridge` 一直在广播两个坐标变换：
# - `odom → base_link`：机身在"出发点坐标系"里的位姿，**随时间变**（来自里程计）；
# - `base_link → utlidar_lidar`：雷达装在机身的哪儿，**固定不变**（静态 TF）。
#
# TF 的用法：建一个 `Buffer` 收集变换，然后**问任意两个坐标系之间的关系**，它替你把中间的链条乘起来。

# %%
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time

buf = Buffer()
listener = TransformListener(buf, me)


def lookup(parent, child):
    tr = buf.lookup_transform(parent, child, Time()).transform       # Time() = "最新的"
    yaw = 2 * np.arctan2(tr.rotation.z, tr.rotation.w)
    return np.array([tr.translation.x, tr.translation.y, tr.translation.z]), yaw


for i in range(4):
    spin_for(me, 1.5)
    (x, y, _z), yaw = lookup("odom", "base_link")
    lx, _ = lookup("odom", "utlidar_lidar")                          # 跨了两段，TF 自动连起来
    print(f"机身在 odom 系: x={x:+.2f} y={y:+.2f} 朝向={np.degrees(yaw):+6.1f}°   雷达在 odom 系: ({lx[0]:+.2f}, {lx[1]:+.2f})")

p, _ = lookup("base_link", "utlidar_lidar")
print("\n雷达相对机身 (固定):", np.round(p, 2))
print("坐标系树:\n" + buf.all_frames_as_string())

# %% [markdown]
# **停一下：** 我们从来没发布过 `odom → utlidar_lidar`，为什么能查到？
# 02 节你手工乘过一次变换矩阵（`T @ pts_body`，还手写了 `LIDAR_X = 0.28`）。TF 做的是同一件事，
# 只是把"谁相对谁、在什么时刻"都管了起来，链条再长也只要问一句。
#
# **改坏它**：问一个不存在的坐标系。

# %%
from tf2_ros import TransformException

try:
    lookup("odom", "left_hand")
except TransformException as e:
    print("TransformException:", e)

# %% [markdown]
# ## 4. 换成 G1 再做一遍
#
# 同样三个节点，参数换成 `robot:=g1`。**预测**：服务的应答里哪个字段会"没有意义"？`"lie"` 这个目标还会被接受吗？

# %%
for p in procs:
    p.stop()
lab.close()
lab = connect("g1", live=LIVE, sdk=False)
procs = start_nodes("g1")

s = ask_summary()
print(f"{s.robot}: 站姿正常={s.upright} 倾角={s.tilt_deg:.1f}° 高度={s.height:.2f} m "
      f"最热电机={s.hottest_motor} ({s.max_motor_temp:.0f} °C) 电量={s.battery_soc:.0f}")
send_goal("lie")
_ = send_goal("stand", timeout=25)

# %%
for p in procs:
    p.stop()
me.destroy_node()
lab.close()

# %% [markdown]
# - G1 的 `LowState` 里没有电池信息，所以 `battery_soc = -1`（00 节你对比过两种 `LowState` 的字段）；
# - G1 的姿态是 `stand` / `squat`，没有 `lie`——服务端直接拒绝；
# - 客户端代码一行没改。**差异被关在了服务端里**，这正是把机器人包装成 Service/Action 的意义。
#
# ## 小结
#
# - **Service**：一问一答。`call_async` → future → spin 到完成。
# - **Action**：目标 → 接受/拒绝 → 反馈 → 结果（成功 / 中止 / 取消）。适合"要花时间的事"。
# - **TF**：只发布相邻坐标系的关系，查询时任意两个都能问。
# - 默认不给机器人发运动指令；要发必须显式打开开关。
#
# ## 连真机时
# `posture_server` 加上 `-p enable_motion:=true` 才会真的下发站立/趴下指令。**这是本课程第一处会让真机动起来的地方**：
# 先完成 `common/safety.py` 里的五条检查，并且第一次请只用 Go2 做。
#
# ## 课后练习
# 1. 写 `wait_until_upright(timeout)`：每 0.5 s 调一次 `/get_robot_summary`，直到 `upright` 为真或超时。它和 Action 的反馈相比，缺点是什么？
# 2. 用 TF 求"雷达正前方 1 m 处的点"在 `odom` 系下的坐标（提示：`lookup("odom", "utlidar_lidar")` 给出雷达位姿，再用 02 节的 `transform_2d`）。
# 3. 读 `src/unitree_lab_nodes/unitree_lab_nodes/posture_server.py`，找出它用了哪种回调组和执行器，并解释如果换成默认的会怎样。
#
# **Part 1 结束。** 你已经能用 ROS 2 读懂机器人了。但还有一个问题没回答：ROS 2 并不是机器人原生的语言——
# `posture_server` 最终是怎么让机器人站起来的？**下一节**我们揭开 `/api/sport/request` 的盖子，进入 Unitree SDK2。
