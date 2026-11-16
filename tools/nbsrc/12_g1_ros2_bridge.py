# %% [markdown]
# # 12 · 把 G1 也接进 ROS 2，并盘点它和 Go2 到底差在哪
#
# **Part 2 用 SDK2 驱动机器人 · 第 8 节（G1，Part 2 收尾）**
#
# 08 节给 Go2 写了桥。这一节的问题是：**同一个桥，换个参数能不能直接用在 G1 上？哪些地方必须分开写？**
# 回答完这个问题，Part 2 一路上发现的 Go2 / G1 差异也就能汇成一张表了。
#
# 学完你能：
# 1. 把 G1 的状态和速度指令接进 ROS 2 标准消息；
# 2. 用程序而不是肉眼，找出两种机器人消息定义的全部差异；
# 3. 填出一张"Go2 vs G1"对照表，并说出每一项差异对写代码意味着什么。

# %%
LIVE = False

import time
import numpy as np

from common import connect, Background, MotionGate
from common.rosutil import ros_init, collect, sh, spin_for

lab = connect("g1", live=LIVE, sdk=False)
gate = MotionGate(lab)
ros_init()

# %% [markdown]
# ## 1. 先跑通：同一个 `robot_bridge`，`robot:=g1`

# %%
bridge = Background("exec ros2 run unitree_lab_nodes robot_bridge --ros-args -p robot:=g1")
assert bridge.wait_for("bridge up", 30), bridge.output()
print(bridge.output(tail=1))

from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry

js = [m for _, m in collect("/joint_states", JointState, seconds=2.0, qos=50)]
print(f"\n/joint_states: {len(js[0].name)} 个关节")
for group, ids in (("左腿", range(0, 6)), ("右腿", range(6, 12)), ("腰", range(12, 15)), ("左臂", range(15, 22)), ("右臂", range(22, 29))):
    print(f"  {group}: {', '.join(js[0].name[i].replace('_joint', '') for i in ids)}")

odom = [m for _, m in collect("/odom", Odometry, seconds=8.0, qos=50)]
x = [m.pose.pose.position.x for m in odom]
print(f"\n/odom: {len(odom)} 条, x 从 {min(x):.2f} 到 {max(x):.2f} m")

# %% [markdown]
# 能用。桥启动时打印的那一行里有 `odom_topic=/odommodestate`——这是 G1 和 Go2 第一处不同，也是**本课程没能从官方源码核实的一处**：
#
# > Go2 的里程计在 `rt/sportmodestate`（源码和 README 里都有）。G1 的里程计 Topic 在三个官方仓库的源码里都没有出现；
# > 本课程的数据按 Unitree 文档的说法用了 `rt/odommodestate`。**上真机第一件事：`ros2 topic list` 看它到底叫什么**，
# > 不一样就用参数改：`-p odom_topic:=/实际的名字`。把"不确定的东西做成参数"是处理这类问题的通用办法。
#
# ## 2. 拆开：桥里哪些代码是两种机器人共用的？
#
# 读 `src/unitree_lab_nodes/unitree_lab_nodes/robot_bridge.py`，数一下有几处 `if self.robot == "go2"`：

# %%
import inspect
from unitree_lab_nodes import robot_bridge

src = inspect.getsource(robot_bridge)
for i, line in enumerate(src.splitlines(), 1):
    if 'self.robot == "go2"' in line or "G1_API" in line or "GO2_API" in line:
        print(f"{i:>4}: {line.rstrip()}")

# %% [markdown]
# 只有四处分叉：**消息类型**、**里程计 Topic 名**、**关节名表**、**速度指令的 API**。其余（IMU 转换、TF、里程计）全部共用——
# 因为 `imu_state`、`motor_state[i].q` 这些字段在两种 `LowState` 里名字和含义都一样。
#
# 速度指令那一处，对比两种请求：

# %%
from rclpy.node import Node
from geometry_msgs.msg import Twist
from common.ros_rpc import RosRpcClient

me = Node("student")
rpc = RosRpcClient(me, "sport")
vx, vy, wz = 0.3, 0.0, 0.5
print("同一条 Twist(0.3, 0, 0.5):")
print("  Go2 ->", 1008, rpc.make_request(1008, {"x": vx, "y": vy, "z": wz}, noreply=True).parameter)
print("  G1  ->", 7105, rpc.make_request(7105, {"velocity": [vx, vy, wz], "duration": 1.0}, noreply=True).parameter)

pub = me.create_publisher(Twist, "/cmd_vel", 10)
cmd = Twist(); cmd.linear.x = vx; cmd.angular.z = wz
for _ in range(5):
    pub.publish(cmd); spin_for(me, 0.1)
print(bridge.output(tail=1))
bridge.stop()

# %% [markdown]
# G1 的请求自带 `duration: 1.0`：桥每收到一条 `/cmd_vel` 就续 1 秒。上层程序死了，G1 最多再走 1 秒。
# （和 08 节一样，`enable_motion` 默认 `false`，上面只打印没下发。）
#
# ## 3. 用程序找出消息定义的全部差异
#
# 00 节我们肉眼对比过两种 `LowState`。现在让程序来做：ROS 2 的每个消息类都能报告自己的字段和类型。

# %%
import unitree_go.msg as go
import unitree_hg.msg as hg


def diff(name):
    a = getattr(go, name).get_fields_and_field_types()
    b = getattr(hg, name).get_fields_and_field_types()
    print(f"── {name}")
    for f in sorted(set(a) | set(b), key=lambda f: (f not in a, f not in b, f)):
        if f not in b:
            print(f"   仅 Go2 : {f:<18} {a[f]}")
        elif f not in a:
            print(f"   仅 G1  : {f:<18} {b[f]}")
        elif a[f] != b[f]:
            print(f"   类型不同: {f:<18} Go2 {a[f]:<28} G1 {b[f]}")
    same = [f for f in a if f in b and a[f] == b[f]]
    print(f"   相同   : {', '.join(same)}")


for name in ("LowState", "MotorState", "IMUState", "LowCmd", "MotorCmd"):
    diff(name)

# %% [markdown]
# **对着输出回答：**
# 1. `MotorState.temperature`：Go2 是一个 `int8`，G1 是 `int16[2]`。07 节的"最热电机"分析搬到 G1 上，哪一行要改？
# 2. `LowCmd` 里 G1 独有的两个字段是什么？11 节你是怎么填它们的？
# 3. `MotorCmd` 的 5 个控制量（`q dq tau kp kd`）在两边一样吗？这说明"控制律相同"这句话成立吗？
#
# 公共库里处理这些差异的地方是 `common/parsing.py` 的 `lowstate_to_dict(msg, robot)`。看它在两种机器人上各给出什么：

# %%
from common.parsing import lowstate_to_dict
from unitree_hg.msg import LowState as G1LowState
from unitree_go.msg import LowState as Go2LowState

g1_msg = collect("/lowstate", G1LowState, count=1, seconds=5.0)[0][1]
d_g1 = lowstate_to_dict(g1_msg, "g1")
lab.close()
lab = connect("go2", live=LIVE, sdk=False)
go2_msg = collect("/lowstate", Go2LowState, count=1, seconds=5.0)[0][1]
d_go2 = lowstate_to_dict(go2_msg, "go2")
lab.close()

print(f"{'键':<14}{'Go2':<22}{'G1'}")
for k in sorted(set(d_g1) | set(d_go2)):
    shape = lambda d: str(np.shape(d[k]) or "标量") if k in d else "—"          # noqa: E731
    print(f"{k:<14}{shape(d_go2):<22}{shape(d_g1)}")

# %% [markdown]
# ## 4. 总表：Go2 vs G1
#
# **先自己填**（遮住下面的表，每行写一句），再对答案。这张表是 Part 2 的全部收获。
#
# | | Go2（四足） | G1（人形） | 对写代码的影响 | 在哪一节见过 |
# |---|---|---|---|---|
# | 消息包 | `unitree_go` | `unitree_hg` | 订阅/发布时类型必须对，错了静默收不到 | 00, 05 |
# | 电机数（槽位） | 12（20） | 29（35） | 遍历电机时用 `num_motor(robot)`，别写死 | 01, 11 |
# | 电机顺序 | 每腿 髋-大腿-小腿，FR FL RR RL | 左腿 6 → 右腿 6 → 腰 3 → 左臂 7 → 右臂 7 | 关节名表不同（`GO2_JOINTS` / `G1_JOINTS`） | 07, 12 |
# | 足端力 | `foot_force[4]` | 无 | "站没站稳"要换判据 | 01 |
# | 电池 | `LowState` 里有 | `LowState` 里无 | 电量监控要另找数据源 | 04, 07 |
# | 电机温度 | 1 个值 | 2 个值 | 取 `max` | 07, 12 |
# | 高层客户端 | `SportClient` | `LocoClient` + `G1ArmActionClient` | 服务名都是 `sport`，API 号不同（1xxx / 7xxx） | 05, 06, 09 |
# | 站起来 | `StandUp()` | `Damp → Squat2StandUp → Start`（切 FSM） | G1 有先后顺序，要查当前状态 | 06, 09 |
# | 速度指令 | `Move(vx,vy,vyaw)` 靠连续发 | `SetVelocity(..., duration)` 带时长 | G1 天然有超时保护 | 06, 09 |
# | 上肢 | 无 | 动作库 + `rt/arm_sdk`（权重） | — | 10 |
# | `LowCmd` | `head` `level_flag` 哨兵值 | `mode_machine` `mode_pr` | 初始化方式不同；CRC 打包长度不同 | 11 |
# | 里程计 Topic | `rt/sportmodestate` | 未核实（数据里用 `rt/odommodestate`） | 做成参数 | 12 |
# | 摔倒的代价 | 小 | 大 | G1 低层实验必须上吊架 | 09, 11 |
# | **相同的** | DDS 通道、`rt/` 命名、RPC 机制、CRC、控制律、IMU 字段、遥控器 40 字节 | | 公共代码可以共用 | 02, 05, 11 |
#
# ## 改坏它
#
# **预测**：如果启动桥时写成 `robot:=go2`，但实际连的是 G1（或回放的是 G1 的数据），`/joint_states` 会怎样？

# %%
lab = connect("g1", live=LIVE, sdk=False)
with Background("exec ros2 run unitree_lab_nodes robot_bridge --ros-args -p robot:=go2") as wrong:
    wrong.wait_for("bridge up", 30)
    n = len(collect("/joint_states", JointState, seconds=3.0, qos=50))
    print(f"robot:=go2 对着 G1 的数据: 3 秒收到 /joint_states {n} 条")
    print(sh("ros2 topic info /lowstate --no-daemon | head -n 3", show=False))
me.destroy_node(); lab.close()

# %% [markdown]
# 0 条，没有报错。`ros2 topic info` 显示 `/lowstate` 上同时有发布者和订阅者——**人都在，就是不说话**，因为类型不一样（05 节的结论）。
# 遇到"明明有 Topic 却收不到"，用 `ros2 topic info -v` 看两边的类型。
#
# ## 小结
# - 同一个桥，G1 只需改四处：消息类型、里程计 Topic、关节名表、速度 API。
# - 不确定的东西（Topic 名、外参）做成**参数**，不要写死。
# - 两种机器人**相同的远比不同的多**：DDS、RPC、CRC、控制律、IMU 都一样——这就是 `common/` 能共用的原因。
#
# ## 课后练习
# 1. 写 `is_upright(msg, robot)`：对 Go2 用足端力 + 倾角，对 G1 用倾角 + 膝关节角（01 节的作业）。
# 2. 在总表里再加两行你自己发现的差异（提示：`ros2 interface show unitree_hg/msg/HandCmd`；Go2 的 `SportModeState` 里有而 G1 用不上的字段）。
# 3. `robot_bridge` 对 G1 发布的 `/joint_states` 有 29 个关节。如果你的 G1 是 23 自由度机型，哪 6 个关节的数据没有意义？（`common/robots.py` 的 `G1_INVALID_23DOF`。）
#
# **Part 2 结束。** 你现在能用 SDK 读状态、发高层指令、发关节指令，并且两种机器人都接进了 ROS 2。
# **Part 3** 开始搭完整的应用：先写两个真正的节点——一个负责"发指令并保证安全"，一个负责"汇总状态"。
