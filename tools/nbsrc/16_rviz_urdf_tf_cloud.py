# %% [markdown]
# # 16 · 把机器人"画"出来：URDF、TF、点云
#
# **Part 3 做出完整应用 · 第 4 节**
#
# 到目前为止，机器人在我们眼里是一堆数字。这一节回答：**12 个关节角 → 机器人此刻摆成了什么姿势？脚在哪？雷达看到的东西相对身体在哪？**
#
# 三样东西合起来回答这个问题：
#
# - **URDF**：机器人的"图纸"——有哪些连杆，关节把谁和谁连在一起、轴朝哪、能转多大；
# - **`robot_state_publisher`**：拿着图纸和实时的关节角，算出每个连杆的位置，以 **TF** 的形式发布；
# - **RViz2**：把模型、TF、点云画在一个三维窗口里。
#
# > 本 Notebook 在没有图形界面的环境下也能跑完：我们用 matplotlib 把 TF 算出来的结果画成"火柴人"，
# > 这样能确认"RViz 里该出现的东西，数据上都对了"。**RViz 窗口本身需要图形界面**，步骤在最后，属于真机/图形界面待验项。
#
# 学完你能：读懂 URDF 的结构；说清 `/joint_states` → TF 这条链路上每个环节做什么；知道模型在 RViz 里"散架"或"不动"时去查哪儿。

# %%
LIVE = False

import os
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

from common import connect, Background, REPO
from common.rosutil import ros_init, collect, sh, spin_for
from common.robots import GO2_URDF_JOINTS, G1_URDF_JOINTS

URDF_DIR = Path(os.environ.get("UNITREE_LAB_DEPS", Path.home() / "unitree_lab_deps")) / "unitree_ros" / "robots"
URDF = {"go2": URDF_DIR / "go2_description/urdf/go2_description.urdf", "g1": URDF_DIR / "g1_description/g1_29dof_rev_1_0.urdf"}
assert URDF["go2"].exists(), f"没找到 URDF: {URDF['go2']}。重跑一次 bash scripts/install.sh (它会从 unitree_ros 仓库取 URDF)。"

lab = connect("go2", live=LIVE, sdk=False)
ros_init()

# %% [markdown]
# ## 1. 先跑通：URDF 里有什么
#
# URDF 是一个 XML 文件（出处：Unitree 官方 `unitree_ros` 仓库的 `robots/go2_description`）。只有两种主要元素：
# `<link>`（连杆：一块刚体）和 `<joint>`（关节：把父连杆和子连杆连起来）。

# %%
def read_urdf(path):
    root = ET.parse(path).getroot()
    links = [l.get("name") for l in root.findall("link")]
    joints = [dict(name=j.get("name"), type=j.get("type"), parent=j.find("parent").get("link"),
                   child=j.find("child").get("link"),
                   axis=(j.find("axis").get("xyz") if j.find("axis") is not None else ""),
                   limit=((j.find("limit").get("lower"), j.find("limit").get("upper")) if j.find("limit") is not None else None))
              for j in root.findall("joint")]
    return root.get("name"), links, joints


name, links, joints = read_urdf(URDF["go2"])
moving = [j for j in joints if j["type"] == "revolute"]
print(f"机器人 '{name}': {len(links)} 个连杆, {len(joints)} 个关节, 其中能转的 (revolute) {len(moving)} 个\n")
print(f"{'关节':<16}{'父连杆':<12}{'子连杆':<12}{'转轴':<8}限位 [rad]")
for j in moving[:6]:
    print(f"{j['name']:<16}{j['parent']:<12}{j['child']:<12}{j['axis']:<8}{j['limit']}")
print("   ...")
children = {j["child"] for j in joints}
print("\n根连杆 (不是任何关节的子连杆):", [l for l in links if l not in children])

# %% [markdown]
# **停一下：**
# 1. `FR_hip_joint` 绕 x 轴转，`FR_thigh_joint` 和 `FR_calf_joint` 绕 y 轴转。想象一下：哪个让腿向外张开，哪两个让腿前后摆？
# 2. 这里的限位和 `common/robots.py` 里的 `GO2_LIMITS` 是同一组数——那张表就是从这个文件抄的（`tests/test_urdf_limits.py` 会自动核对）。
#
# 关节是一棵**树**：根是 `base`，每条腿是一根树枝。把一条腿的链条打印出来：

# %%
def chain(joints, leaf):
    by_child = {j["child"]: j for j in joints}
    out = [leaf]
    while out[-1] in by_child:
        out.append(by_child[out[-1]]["parent"])
    return out[::-1]


print(" -> ".join(chain(joints, "FR_foot")))

# %% [markdown]
# ## 2. 让图纸动起来：`/joint_states` → `robot_state_publisher` → TF
#
# ```
#  /lowstate ──▶ robot_bridge ──▶ /joint_states ──▶ robot_state_publisher ──▶ /tf  (base → 每个连杆)
#  (电机角度)     (08 节)          (名字 + 角度)        (拿着 URDF 做正运动学)
#  /sportmodestate ──▶ robot_bridge ──────────────────────────────────────▶ /tf  (odom → base_link)
# ```
#
# 关键约定：`/joint_states` 里的 `name` 必须和 URDF 里的关节名**逐字相同**。先检查这一点：

# %%
urdf_names = {j["name"] for j in moving}
print("桥发布的 12 个关节名都在 URDF 里吗?", set(GO2_URDF_JOINTS) <= urdf_names)

# %% [markdown]
# 课程的 launch 文件带上 `urdf:=<路径>` 就会启动 `robot_state_publisher`（并用一个静态 TF 把桥的 `base_link` 和 URDF 的根 `base` 接起来）：

# %%
def start_stack(robot, extra=""):
    p = Background(f"exec ros2 launch unitree_lab_nodes stack.launch.py robot:={robot} urdf:={URDF[robot]} {extra}")
    assert p.wait_for("bridge up", 40), p.output()
    time.sleep(3.0)
    return p


stack = start_stack("go2")
me = Node("student")
buf = Buffer(); listener = TransformListener(buf, me)
spin_for(me, 3.0)
frames = sorted(f for f in buf.all_frames_as_string().split("Frame ")[1:])
print(f"TF 里现在有 {len(frames)} 个坐标系。前几个:")
for f in frames[:6]:
    print("  ", f.split(" exists")[0], "<- 父:", f.split("parent ")[1].split(".")[0])

# %% [markdown]
# 现在可以问任何一个连杆在哪。比如四只脚相对机身的位置：

# %%
def position(parent, child):
    tr = buf.lookup_transform(parent, child, Time()).transform.translation
    return np.array([tr.x, tr.y, tr.z])


for foot in ("FR_foot", "FL_foot", "RR_foot", "RL_foot"):
    print(f"{foot}: 相对机身 base  x={position('base', foot)[0]:+.3f}  y={position('base', foot)[1]:+.3f}  z={position('base', foot)[2]:+.3f} m")

# %% [markdown]
# `z` 是脚在机身下方多少米——也就是**机身离地高度**（站着时）。你没有量过任何高度，它是从关节角和图纸**算**出来的。
#
# ### 没有 RViz 也能"看"：火柴人
#
# 把每条腿沿链条的各个连杆位置连成线。在回放数据的不同时刻各画一次（数据每 14.5 秒循环：趴 → 站 → 走）。

# %%
LEGS = {"FR": "tab:red", "FL": "tab:orange", "RR": "tab:blue", "RL": "tab:cyan"}


def snapshot(root="base", legs=LEGS):
    return {leg: np.array([position(root, f"{leg}_{part}") for part in ("hip", "thigh", "calf", "foot")]) for leg in legs}


def draw(ax, snap, title):
    for leg, pts in snap.items():
        ax.plot(pts[:, 0], pts[:, 2], "o-", color=LEGS[leg], label=leg, ms=3)
    ax.plot([-0.3, 0.3], [0, 0], "k-", lw=4, alpha=.5)                        # 机身
    ax.set(title=title, xlabel="x forward [m]", ylabel="z up [m]", xlim=(-0.5, 0.5), ylim=(-0.45, 0.15)); ax.set_aspect("equal"); ax.grid(alpha=.3)


snaps = []
t_end = time.monotonic() + 16.0
while time.monotonic() < t_end:                                               # 每 0.5 s 抓一帧，覆盖一整圈数据
    spin_for(me, 0.5)
    snaps.append((-position("base", "FR_foot")[2], snapshot()))
low = min(snaps, key=lambda s: s[0]); high = max(snaps, key=lambda s: s[0])
fig, ax = plt.subplots(1, 2, figsize=(10, 3.4))
draw(ax[0], low[1], f"lowest (foot {low[0]:.2f} m below body)"); draw(ax[1], high[1], f"highest (foot {high[0]:.2f} m below body)")
ax[0].legend(fontsize=7); plt.tight_layout(); plt.show()

# %% [markdown]
# 左边是趴着（腿折叠），右边是站着。**这就是 RViz 里 RobotModel 显示的内容**，只是 RViz 还贴上了外壳网格。
# 火柴人对了，说明 `/joint_states` 的名字、顺序、URDF、TF 整条链路都是对的。
#
# ### 改坏它：关节顺序错位
#
# 08 节的桥按"电机序 = `GO2_URDF_JOINTS` 的顺序"给关节角配名字。**预测**：如果名字表错位了一格（每个角度配给了下一个关节的名字），火柴人会变成什么样？
# 不用重启任何节点——直接用错位的名字自己发一份 `/joint_states`，同时把正确的桥停掉。

# %%
from sensor_msgs.msg import JointState

stack.stop(); time.sleep(1.0)
only_rsp = Background(f"exec ros2 launch unitree_lab_nodes stack.launch.py robot:=go2 urdf:={URDF['go2']}")
only_rsp.wait_for("bridge up", 40)
good = collect("/joint_states", JointState, count=1, seconds=5.0)[0][1]
only_rsp.stop(); time.sleep(1.0)

# 只启动 robot_state_publisher，/joint_states 由我们手工发布 (错位一格)
rsp = Background("exec ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:=\"$(cat %s)\"" % URDF["go2"])
time.sleep(3.0)
pub = me.create_publisher(JointState, "/joint_states", 10)
bad = JointState(); bad.name = GO2_URDF_JOINTS[1:] + GO2_URDF_JOINTS[:1]; bad.position = list(good.position)
buf2 = Buffer(); listener2 = TransformListener(buf2, me)
for _ in range(20):
    bad.header.stamp = me.get_clock().now().to_msg(); pub.publish(bad); spin_for(me, 0.1)
buf, buf_ok = buf2, buf
fig, ax = plt.subplots(figsize=(5, 3.4)); draw(ax, snapshot(), "joint names shifted by one"); plt.show()
buf = buf_ok
rsp.stop()

# %% [markdown]
# 没有报错，TF 照常发布，只是机器人"骨折"了。在 RViz 里看到模型姿势怪异时，先查 `/joint_states` 的 `name` 和 `position` 是不是一一对应。
#
# | RViz 里的症状 | 先查什么 |
# |---|---|
# | 模型不显示，RobotModel 报错 | Fixed Frame 设的坐标系存在吗？`robot_description` 参数有吗？ |
# | 模型显示了但不动 | `/joint_states` 有数据吗？关节名和 URDF 一致吗？ |
# | 模型姿势怪异 | 关节名与角度错位；或弧度/角度搞混 |
# | 模型躺倒、乱转 | `odom → base_link` 的四元数顺序（08 节） |
# | 点云和模型对不上 | 雷达的外参 TF（下一部分） |
#
# ## 3. 点云：雷达看到了什么
#
# `/utlidar/cloud` 是 `sensor_msgs/PointCloud2`：一大块字节 + 一张"每个点怎么排"的说明（`fields`）。

# %%
from sensor_msgs.msg import PointCloud2
from unitree_lab_nodes.cloud_to_scan import cloud_to_xyz

stack = start_stack("go2")
cloud = collect("/utlidar/cloud", PointCloud2, count=1, seconds=5.0)[0][1]
print("坐标系:", cloud.header.frame_id, "| 点数:", cloud.width * cloud.height, "| 每个点", cloud.point_step, "字节")
print("字段:", [(f.name, f.offset, {7: "float32"}.get(f.datatype, f.datatype)) for f in cloud.fields])
xyz = cloud_to_xyz(cloud)                 # 02 节的 struct 思路 + NumPy: 按 offset 把 x/y/z 取出来
print("xyz 数组:", xyz.shape, " x 范围 %.1f ~ %.1f m" % (xyz[:, 0].min(), xyz[:, 0].max()))

# %% [markdown]
# 点的坐标是相对 `utlidar_lidar` 的。要和机器人模型画在一起，需要 `base_link → utlidar_lidar` 这段 TF（雷达的**外参**）。
# 用 TF 把点云变到 `odom` 系，和机器人的位置、房间的墙画在一起：

# %%
from common.plotting import plot_room
from unitree_lab_nodes.cloud_to_scan import transform_matrix

buf = Buffer(); listener = TransformListener(buf, me)
ax = plot_room()
for k in range(3):
    spin_for(me, 2.5)
    cloud = collect("/utlidar/cloud", PointCloud2, count=1, seconds=3.0)[0][1]
    spin_for(me, 0.3)
    M = transform_matrix(buf.lookup_transform("odom", cloud.header.frame_id, Time()).transform)      # 4x4: 雷达系 -> odom 系
    pts = cloud_to_xyz(cloud); pts = pts[np.abs(pts[:, 2]) < 0.3] @ M[:3, :3].T + M[:3, 3]
    body = position("odom", "base_link")
    ax.plot(pts[:, 0], pts[:, 1], ".", ms=1.5); ax.plot(body[0], body[1], "r^", ms=7)
ax.set_title("Point cloud in the odom frame (via TF)"); plt.show()
print("雷达外参 base_link -> utlidar_lidar:", np.round(position("base_link", "utlidar_lidar"), 3))

# %% [markdown]
# 02 节你手写过 `LIDAR_X = 0.28` 再乘矩阵；这里一句 `lookup_transform("odom", "utlidar_lidar")` 就拿到了完整的变换。
#
# > **这段外参是合成数据用的值**（雷达在机身前方 0.28 m、不带旋转）。真机 Go2 的雷达是**倾斜安装**的，真实外参要从 URDF/官方资料里取，
# > 并通过 `robot_bridge` 的参数 `lidar_xyz` 或自己发布的静态 TF 给出。**这是一条真机待验项**：外参错了，点云里的墙会斜着插进地面。
#
# ## 4. 换成 G1

# %%
stack.stop(); lab.close()
lab = connect("g1", live=LIVE, sdk=False)
name, g_links, g_joints = read_urdf(URDF["g1"])
g_moving = [j for j in g_joints if j["type"] == "revolute"]
print(f"机器人 '{name}': {len(g_links)} 个连杆, 能转的关节 {len(g_moving)} 个; 根连杆: {[l for l in g_links if l not in {j['child'] for j in g_joints}]}")
print("桥发布的 29 个关节名都在 URDF 里吗?", set(G1_URDF_JOINTS) <= {j["name"] for j in g_moving})
print("左臂的链条:", " -> ".join(chain(g_joints, "left_wrist_yaw_link")))

stack = start_stack("g1")
buf = Buffer(); listener = TransformListener(buf, me)
CHAINS = {"left leg": chain(g_joints, "left_ankle_roll_link"), "right leg": chain(g_joints, "right_ankle_roll_link"),
          "left arm": chain(g_joints, "left_wrist_yaw_link"), "right arm": chain(g_joints, "right_wrist_yaw_link")}
snaps = []
t_end = time.monotonic() + 24.0
while time.monotonic() < t_end:
    spin_for(me, 1.0)
    snaps.append({k: np.array([position("pelvis", l) for l in links_]) for k, links_ in CHAINS.items()})
squat = max(snaps, key=lambda s: s["left leg"][-1, 2]); stand = min(snaps, key=lambda s: s["left leg"][-1, 2])
arm_up = max(snaps, key=lambda s: max(s["left arm"][-1, 2], s["right arm"][-1, 2]))
fig, ax = plt.subplots(1, 3, figsize=(11, 4))
for a, snap, title in zip(ax, (squat, stand, arm_up), ("squatting", "standing", "an arm raised")):
    for k, pts in snap.items():
        a.plot(pts[:, 0], pts[:, 2], "o-", ms=3, label=k)
    a.set(title=title, xlabel="x forward [m]", xlim=(-0.6, 0.6), ylim=(-0.9, 0.7)); a.set_aspect("equal"); a.grid(alpha=.3)
ax[0].set_ylabel("z up [m]"); ax[0].legend(fontsize=7); plt.tight_layout(); plt.show()
stack.stop(); me.destroy_node(); lab.close()

# %% [markdown]
# 同一套流程，换了 URDF 和关节名表。G1 的根连杆叫 `pelvis`（骨盆），而 Go2 的叫 `base`——launch 文件里那个"恒等静态 TF"接的就是它们。
#
# ## 5. 有图形界面时：打开 RViz2
#
# 学生的 VMware 虚拟机有桌面，直接：
#
# ```bash
# source scripts/env.sh
# ros2 bag play --loop data/bags/go2_demo &                       # 或连真机
# ros2 launch unitree_lab_nodes stack.launch.py robot:=go2 urdf:=$UNITREE_LAB_DEPS/unitree_ros/robots/go2_description/urdf/go2_description.urdf
# rviz2 -d config/go2.rviz
# ```
# `config/go2.rviz` 预设了：Fixed Frame = `odom`，显示 RobotModel、TF、PointCloud2（`/utlidar/cloud`）、LaserScan（`/scan`）。
# **预期看到**：机器人模型从趴到站再走动；点云勾出房间的墙；所有坐标轴随身体运动。
# （RobotModel 的外壳网格需要 `go2_description` 包在 ROS 包路径里；找不到网格时模型显示为坐标轴，不影响 TF 和点云。）
#
# ## 小结
# - URDF = 连杆 + 关节的树；`robot_state_publisher` = 图纸 + 关节角 → 每个连杆的 TF。
# - `/joint_states` 的名字必须与 URDF 一致，且与角度一一对应；错了不报错，只会"骨折"。
# - 点云要和模型对上，靠的是雷达外参这段 TF。
# - 没有 RViz 时，用 TF 查询 + matplotlib 也能验证整条链路。
#
# ## 课后练习
# 1. 用 TF 算出 Go2 站立时的"轴距"（前后脚的 x 距离）和"轮距"（左右脚的 y 距离）。
# 2. 写 `body_height_from_tf()`：返回四只脚 z 的平均值的相反数。把它和 `/sportmodestate` 的 `position[2]` 画在一起比较。
# 3. 在有桌面的环境里打开 RViz2，截一张包含模型 + 点云的图。把 Fixed Frame 从 `odom` 改成 `base_link`，画面有什么不同？为什么？
#
# **下一节**：点云里已经能看出房间的形状了。把机器人一路走过看到的点云拼起来，就是一张**地图**；有了地图，就能问"从这儿到那儿怎么走"。
