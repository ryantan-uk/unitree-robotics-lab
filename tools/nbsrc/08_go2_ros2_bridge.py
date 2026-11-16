# %% [markdown]
# # 08 · 把 Go2 接进 ROS 2 的世界
#
# **Part 2 用 SDK2 驱动机器人 · 第 4 节（Go2）**
#
# ROS 2 生态里的工具只认**标准消息**：
#
# | 工具想要 | 标准消息 | Go2 给的 |
# |---|---|---|
# | 姿态 | `sensor_msgs/Imu` | `LowState.imu_state` |
# | 关节角（RViz 显示模型要用） | `sensor_msgs/JointState` | `LowState.motor_state[0..11]` |
# | 位置（导航要用） | `nav_msgs/Odometry` + TF | `SportModeState.position` |
# | 速度指令（导航输出的） | `geometry_msgs/Twist` 发到 `/cmd_vel` | `SportClient.Move()` |
#
# 中间缺一个"翻译"，叫**桥接（bridge）**。这一节先亲手写一个最小的桥——**用 SDK 收、用 ROS 2 发**——
# 再换用仓库里写好的完整版 `robot_bridge`。
#
# 学完你能：写出"SDK → ROS 2 标准消息"的桥接节点，并说出两个最容易错的地方。

# %%
LIVE = False

import time
import numpy as np
import rclpy
from rclpy.node import Node

from common import connect, Background, MotionGate
from common.rosutil import ros_init, collect, sh, spin_for

lab = connect("go2", live=LIVE)
gate = MotionGate(lab)
ros_init()

# %% [markdown]
# ## 1. 先跑通：20 行的桥
#
# 一边是 SDK 的 `ChannelSubscriber`（回调在 SDK 的线程里），一边是 rclpy 的 publisher。
# 在 SDK 回调里直接调 `publish()` 即可——rclpy 的 `publish` 可以从别的线程调用。

# %%
from sensor_msgs.msg import Imu, JointState
from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
from common.robots import GO2_URDF_JOINTS


class MiniBridge(Node):
    def __init__(self, fix_quaternion=True):
        super().__init__("mini_bridge")
        self.fix = fix_quaternion
        self.pub_imu = self.create_publisher(Imu, "/imu/data", 10)
        self.pub_js = self.create_publisher(JointState, "/joint_states", 10)
        self.sub = ChannelSubscriber("rt/lowstate", LowState_)       # SDK 侧
        self.sub.Init(self.on_lowstate, 10)

    def on_lowstate(self, m):                                        # 在 SDK 的线程里被调用
        now = self.get_clock().now().to_msg()
        imu = Imu()
        imu.header.stamp, imu.header.frame_id = now, "imu_link"
        w, x, y, z = m.imu_state.quaternion                          # Unitree: (w, x, y, z)
        if not self.fix:
            x, y, z, w = m.imu_state.quaternion                      # 故意留的错误版本，第 3 部分用
        imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w = float(x), float(y), float(z), float(w)
        gx, gy, gz = m.imu_state.gyroscope
        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = float(gx), float(gy), float(gz)
        ax_, ay, az = m.imu_state.accelerometer
        imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = float(ax_), float(ay), float(az)
        self.pub_imu.publish(imu)

        js = JointState()
        js.header.stamp = now
        js.name = GO2_URDF_JOINTS                                    # 名字必须和 URDF 里的关节名一致
        js.position = [float(m.motor_state[i].q) for i in range(12)]
        js.velocity = [float(m.motor_state[i].dq) for i in range(12)]
        js.effort = [float(m.motor_state[i].tau_est) for i in range(12)]
        self.pub_js.publish(js)

    def close(self):
        self.sub.Close(); self.destroy_node()


bridge = MiniBridge()
imu_msgs = [m for _, m in collect("/imu/data", Imu, seconds=2.0, qos=50)]
js_msgs = [m for _, m in collect("/joint_states", JointState, seconds=1.0, qos=50)]
print(f"/imu/data: 2 秒 {len(imu_msgs)} 条;  /joint_states: {len(js_msgs[0].name)} 个关节")
print("关节名:", js_msgs[0].name[:3], "...   角度:", np.round(js_msgs[0].position[:3], 3))

# %% [markdown]
# 现在任何 ROS 2 工具都能用这些数据了，比如命令行直接看：

# %%
_ = sh("ros2 topic echo /imu/data --once --field orientation", timeout=15)

# %% [markdown]
# **停一下：** `collect()` 在转它自己的节点，`bridge` 这个节点从来没被 spin 过——为什么它还能发消息？
# （只有**收**消息、定时器、服务这些"回调"才需要 spin。`publish()` 是你主动调用的，不需要。这里触发它的是 SDK 的线程。）
#
# ## 2. 拆开：两个最容易错的地方
#
# ### 错误一：四元数顺序
#
# 02 节验证过 Unitree 是 `(w, x, y, z)`；而 `geometry_msgs/Quaternion` 的字段是 `x, y, z, w`。
# 直接按顺序抄过去，**不会报错**。
#
# 怎么检查？用 02 节的办法：**重力**。把加速度计读数（机体系）用姿态转到世界系，应该是 `[0, 0, +9.8]`。
# 姿态要是错的，重力就会指到别的方向去。

# %%
from common.parsing import quat_to_rpy, rot_from_rpy


def gravity_in_world(seconds=2.0):
    """用 /imu/data 里的姿态把加速度转到世界系，返回平均值。"""
    out = []
    for _, m in collect("/imu/data", Imu, seconds=seconds, qos=50):
        o, a = m.orientation, m.linear_acceleration
        R = rot_from_rpy(*quat_to_rpy([o.w, o.x, o.y, o.z]))          # 按 ROS 的字段名取，再转成矩阵
        out.append(R @ [a.x, a.y, a.z])
    return np.round(np.mean(out, axis=0), 1)


print("正确的桥    : 世界系里的重力读数 =", gravity_in_world())
bridge.close()
bad = MiniBridge(fix_quaternion=False)
print("抄错顺序的桥 : 世界系里的重力读数 =", gravity_in_world())
bad.close()

# %% [markdown]
# 抄错顺序后，重力"朝天上去了"（z 变成负的）——姿态被解读成了机身肚皮朝天。
# 在 RViz 里的症状就是模型躺倒或乱转。**桥写完，先做一次重力检验。**
#
# ### 错误二：速度在哪个坐标系
#
# `SportModeState.velocity` 是**世界系**速度，而 `nav_msgs/Odometry.twist` 约定在**机体系**（`child_frame_id`）。
# 机器人朝北走时两者差一个旋转。完整版 `robot_bridge` 里有这一步（`src/unitree_lab_nodes/unitree_lab_nodes/robot_bridge.py` 的 `on_sport`）。
#
# ## 3. 用完整版 `robot_bridge`
#
# 它是纯 ROS 2 写的（订阅 `/lowstate` 而不是用 SDK），多做了里程计、TF 和 `/cmd_vel`。

# %%
node_proc = Background("exec ros2 run unitree_lab_nodes robot_bridge --ros-args -p robot:=go2")
assert node_proc.wait_for("bridge up", 30), node_proc.output()
time.sleep(1.0)

from nav_msgs.msg import Odometry
odom = [m for _, m in collect("/odom", Odometry, seconds=6.0, qos=50)]
xy = np.array([[m.pose.pose.position.x, m.pose.pose.position.y] for m in odom])
vx = np.array([m.twist.twist.linear.x for m in odom]); vy = np.array([m.twist.twist.linear.y for m in odom])
print(f"/odom {len(odom)} 条; 位置范围 x {xy[:, 0].min():+.2f}~{xy[:, 0].max():+.2f}  y {xy[:, 1].min():+.2f}~{xy[:, 1].max():+.2f}")
print(f"机体系速度: 前向 vx 最大 {vx.max():.2f} m/s, 侧向 |vy| 最大 {np.abs(vy).max():.2f} m/s  (只前进和转弯, 所以侧向应接近 0)")
_ = sh("ros2 topic list --no-daemon | grep -E 'imu|joint_states|odom|tf|cmd_vel'")

# %% [markdown]
# ### 反方向：`/cmd_vel` → 运动请求
#
# 导航之类的上层程序只会往 `/cmd_vel` 发 `Twist`。桥收到后翻译成上一节的 `Move` 请求。
# 这个桥**不用 SDK**，而是直接往 `/api/sport/request` 发 `unitree_api/Request`——05 节你已经知道这两者是一回事。
# 公共库里的 `RosRpcClient`（`common/ros_rpc.py`，40 行）就是"用 ROS 2 写的 SportClient"：

# %%
from geometry_msgs.msg import Twist
from common.ros_rpc import RosRpcClient

me = Node("student")
rpc = RosRpcClient(me, "sport")
req = rpc.make_request(1008, {"x": 0.3, "y": 0.0, "z": 0.5}, noreply=True)
print("Twist(linear.x=0.3, angular.z=0.5)  ->  Request api_id =", req.header.identity.api_id, " parameter =", req.parameter)

# 往 /cmd_vel 发 1 秒的速度指令，看桥的反应
pub = me.create_publisher(Twist, "/cmd_vel", 10)
cmd = Twist(); cmd.linear.x = 0.3; cmd.angular.z = 0.5
for _ in range(10):
    pub.publish(cmd); spin_for(me, 0.1)
print(node_proc.output(tail=2))

# %% [markdown]
# 桥收到了，但**没有下发**：`enable_motion` 默认是 `false`。这和 `gate` 是同一个原则，只是做在了节点的参数上。
#
# 连着真机并确认安全后，这样启动才会真的动：
# ```bash
# ros2 run unitree_lab_nodes robot_bridge --ros-args -p robot:=go2 -p enable_motion:=true
# ros2 topic pub -r 5 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"     # Ctrl-C 即停发
# ```
#
# ### 改坏它：频率
#
# `/lowstate` 很快，而 RViz 只需要每秒几十帧。完整版的桥每 `decimate` 条转发 1 条（默认 4）。
# **预测**：`decimate:=1` 和 `decimate:=20` 时 `/joint_states` 各是多少 Hz？

# %%
node_proc.stop()
for dec in (1, 20):
    with Background(f"exec ros2 run unitree_lab_nodes robot_bridge --ros-args -p robot:=go2 -p decimate:={dec}") as p:
        p.wait_for("bridge up", 30)
        n = len(collect("/joint_states", JointState, seconds=3.0, qos=200))
        print(f"decimate={dec:<3} /joint_states ≈ {n / 3:.0f} Hz")
me.destroy_node(); lab.close()

# %% [markdown]
# ## 小结
# - 桥 = 订阅原生消息 → 换成标准消息 → 发布。SDK 回调里可以直接 `publish()`。
# - 两个坑：**四元数顺序**（Unitree `w,x,y,z` ↔ ROS `x,y,z,w`）、**速度坐标系**（世界系 ↔ 机体系）。两个都不报错，要写检查。
# - 反方向 `/cmd_vel → Move` 默认不下发，`enable_motion:=true` 才发。
# - `JointState.name` 必须和 URDF 的关节名一致（16 节用到）。
#
# ## 课后练习
# 1. 给 `MiniBridge` 加上足端力：发布到 `/foot_force`（用 `std_msgs/Float32MultiArray`）。
# 2. 把 `MiniBridge` 改成纯 ROS 2 版本（订阅 `/lowstate` 而不是用 SDK）。两种写法各有什么好处？
# 3. 读 `robot_bridge.py` 的 `on_sport`，手算：机器人朝向 +90°、世界系速度 (0, 0.4) 时，机体系 `linear.x` 是多少？
#
# **下一节**：换机器人。同样的四件事——站起来、走、读状态、接进 ROS 2——在人形机器人 G1 上哪些一样、哪些完全不同？
