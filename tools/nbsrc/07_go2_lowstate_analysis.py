# %% [markdown]
# # 07 · 走了五分钟，机器人身体里发生了什么？
#
# **Part 2 用 SDK2 驱动机器人 · 第 3 节（Go2）**
#
# 设想你让 Go2 在走廊里巡逻了五分钟，回来后有人问你四个问题：
#
# 1. 它一路上稳不稳？（**IMU**）
# 2. 电还够跑多久？（**电池**）
# 3. 有没有哪个电机不对劲？（**电机温度**）
# 4. 它是用什么步态走的，一秒迈几步？（**足端力**）
#
# 答案全在 `LowState` 里。这一节先用 SDK 现场读一条，认清这些字段在哪；再对一段 5 分钟的记录做分析。
#
# 数据：`data/go2_long_run.npz`（**合成**，50 Hz × 300 s；过程是"走 10 s → 转 10 s → 走 10 s → 站 10 s"循环）。

# %%
LIVE = False

import time
import numpy as np
import matplotlib.pyplot as plt

from common import connect
from common.bagio import load_npz
from common.robots import GO2_JOINTS
from common.plotting import plot_bars, plot_foot_force

lab = connect("go2", live=LIVE)

# %% [markdown]
# ## 1. 先跑通：四类数据各在哪个字段
#
# 用上一节学的 SDK 订阅读一条 `rt/lowstate`。

# %%
from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_

box = []
sub = ChannelSubscriber("rt/lowstate", LowState_); sub.Init(lambda m: box.append(m), 10)
time.sleep(1.0); sub.Close()
m = box[-1]

print("① IMU     imu_state.rpy            =", np.round(m.imu_state.rpy, 3), "rad")
print("          imu_state.gyroscope      =", np.round(m.imu_state.gyroscope, 3), "rad/s")
print("          imu_state.accelerometer  =", np.round(m.imu_state.accelerometer, 2), "m/s^2")
print("② 电池    power_v / power_a        = %.2f V / %.2f A" % (m.power_v, m.power_a))
print("          bms_state.soc            =", m.bms_state.soc, "%   cycle =", m.bms_state.cycle)
print("          bms_state.cell_vol[:8]   =", list(m.bms_state.cell_vol[:8]), "mV")
print("③ 电机    motor_state[i].temperature =", [m.motor_state[i].temperature for i in range(12)], "°C")
print("          motor_state[i].tau_est     =", [round(m.motor_state[i].tau_est, 1) for i in range(12)], "N·m")
print("④ 足端力  foot_force (FR FL RR RL)  =", list(m.foot_force))
lab.close()

# %% [markdown]
# 电机顺序（出处：SDK 示例的 `LegID`）：每条腿 3 个电机 *髋 hip → 大腿 thigh → 小腿 calf*，腿的顺序 *FR 右前 → FL 左前 → RR 右后 → RL 左后*。

# %%
print({i: name for i, name in enumerate(GO2_JOINTS)})

# %% [markdown]
# 现在换成 5 分钟的记录。`common.parsing.lowstate_to_dict()` 把一条 `LowState` 摊平成字典，一段记录就是这些字典堆成的数组。

# %%
d = load_npz("go2_long_run")
t = d["t"]
print({k: v.shape for k, v in d.items()})

# %% [markdown]
# ## 2. 它稳不稳？——IMU

# %%
fig, ax = plt.subplots(figsize=(9, 2.8))
ax.plot(t, np.degrees(d["rpy"][:, 0]), lw=.6, label="roll"); ax.plot(t, np.degrees(d["rpy"][:, 1]), lw=.6, label="pitch")
ax.set(xlabel="time [s]", ylabel="deg", title="Roll / pitch over 5 minutes"); ax.legend(); ax.grid(alpha=.3); plt.show()

tilt = np.degrees(np.arccos(np.cos(d["rpy"][:, 0]) * np.cos(d["rpy"][:, 1])))
moving = np.hypot(*np.gradient(d["position"][:, :2], t, axis=0).T) > 0.1
print(f"倾角: 平均 {tilt.mean():.2f}°, 最大 {tilt.max():.2f}°")
print(f"行走时 roll 的抖动 (标准差) {np.degrees(d['rpy'][moving, 0].std()):.2f}°, 站立时 {np.degrees(d['rpy'][~moving, 0].std()):.2f}°")

# %% [markdown]
# **停一下：** 走路时机身为什么会有规律地左右晃？晃动的频率应该和什么有关？（记住这个问题，第 5 部分用足端力来回答。）
#
# ## 3. 电还够跑多久？——电池

# %%
power = d["power_v"] * d["power_a"]                       # 瞬时功率 [W]
energy_wh = np.trapz(power, t) / 3600                     # 积分得到能量 [Wh]
fig, ax = plt.subplots(1, 3, figsize=(11, 2.8))
ax[0].plot(t, d["soc"]); ax[0].set(title="State of charge [%]", xlabel="s")
ax[1].plot(t, d["power_v"]); ax[1].set(title="Voltage [V]", xlabel="s")
ax[2].plot(t, power, lw=.5); ax[2].set(title="Power [W]", xlabel="s")
[a.grid(alpha=.3) for a in ax]; plt.tight_layout(); plt.show()

rate = (d["soc"][0] - d["soc"][-1]) / (t[-1] - t[0]) * 60           # %/分钟
print(f"5 分钟耗电 {d['soc'][0] - d['soc'][-1]:.0f}% ({energy_wh:.1f} Wh), 平均功率 {power.mean():.0f} W")
print(f"按这个速度, 剩余 {d['soc'][-1]:.0f}% 还能跑约 {d['soc'][-1] / rate:.0f} 分钟")
print(f"行走时平均功率 {power[moving].mean():.0f} W, 站立时 {power[~moving].mean():.0f} W")

# %% [markdown]
# **预测 → 验证**：`soc` 是整数，5 分钟只掉了几格，用它估续航误差很大。有没有更细的信号？
# 电压也随电量下降，但它还受电流影响（电流大时电压被"拉低"）。下面只取**站立不动**（电流小且稳定）的时刻看电压：

# %%
calm = ~moving & (t > 5)
k, b = np.polyfit(t[calm], d["power_v"][calm], 1)
print(f"站立时刻的电压每分钟下降 {-k * 60 * 1000:.1f} mV")
fig, ax = plt.subplots(figsize=(8, 2.4))
ax.plot(t[calm], d["power_v"][calm], ".", ms=2); ax.plot(t, k * t + b, "r")
ax.set(xlabel="s", ylabel="V", title="Voltage while standing still"); ax.grid(alpha=.3); plt.show()

# %% [markdown]
# > 这里的电池模型是合成的（电压 = 线性开路电压 − 内阻压降）。真机电池的放电曲线不是直线，**不要把这里的数字当真机参数**，方法才是要带走的。
#
# ## 4. 有没有电机不对劲？——温度

# %%
temp = d["temp"]
rise = temp[-1] - temp[0]
hot = int(np.argmax(temp[-1]))
plot_bars(temp[-1], GO2_JOINTS, title="Motor temperature after 5 minutes", ylabel="degC", highlight=hot); plt.show()
print(f"最热的是 {hot} 号 {GO2_JOINTS[hot]}: {temp[-1, hot]:.0f} °C, 五分钟升了 {rise[hot]:.0f} °C; 其他小腿电机平均升了 {rise[[2, 5, 8]].mean():.0f} °C")

# %% [markdown]
# 有一个电机明显比同类热。**它是因为干的活多，还是因为它自己有问题？** 这两种情况的处理完全不同。
#
# 电机发热主要来自电流，而电流正比于力矩，所以发热功率 ∝ 力矩²。把每个电机的"平均力矩平方"和"温升"画在一起：

# %%
effort = (d["tau"] ** 2).mean(axis=0)
fig, ax = plt.subplots(figsize=(5.5, 3.6))
ax.scatter(effort, rise)
for i, name in enumerate(GO2_JOINTS):
    ax.annotate(name, (effort[i], rise[i]), fontsize=7, xytext=(3, 3), textcoords="offset points")
ax.set(xlabel="mean torque^2 [(N·m)^2]", ylabel="temperature rise [degC]", title="Is the hot motor working harder?")
ax.grid(alpha=.3); plt.show()
calves = [2, 5, 8, 11]
print("四个小腿电机的 力矩平方均值:", np.round(effort[calves], 1), " 温升:", np.round(rise[calves], 0))

# %% [markdown]
# 四个小腿电机出的力几乎一样，其中一个的温升却高出一截——**不是活多，是它自己散热差或损耗大**。
# 这就是从数据里发现硬件隐患的基本方法：*找一个应该相同的对照组，看谁不合群。*
#
# （这个"异常电机"是合成数据里故意埋的。在真机上做同样的分析，结论可能是"都正常"——那也是有价值的结论。）
#
# ## 5. 它怎么走的？——足端力

# %%
seg = (t > 4) & (t < 7)                                   # 取 3 秒行走片段
plot_foot_force(t[seg], d["foot_force"][seg]); plt.show()

# %% [markdown]
# **看图回答**：哪两只脚是同时着地的？这种步态叫什么？
#
# 用数字说话。先定"着地"的阈值（**从图上读**，介于 0 和满载之间），再算三样东西：

# %%
CONTACT = 30.0
walk = (t > 4) & (t < 12)
contact = d["foot_force"][walk] > CONTACT                 # (N, 4) 布尔: FR FL RR RL
tw = t[walk]

# ① 步频: 对一只脚的力做 FFT，找最强的频率
f = d["foot_force"][walk, 0]; f = f - f.mean()
spec = np.abs(np.fft.rfft(f)); freq = np.fft.rfftfreq(len(f), tw[1] - tw[0])
step_hz = freq[np.argmax(spec)]
# ② 占空比: 每只脚着地时间的比例
duty = contact.mean(axis=0)
# ③ 哪两只脚同步: 着地状态的相关系数
corr = np.corrcoef(contact.T.astype(float))
print(f"步频 {step_hz:.2f} Hz (每只脚每秒落地 {step_hz:.1f} 次)")
print("占空比 FR FL RR RL:", np.round(duty, 2))
print("FR 与 FL/RR/RL 的同步程度:", np.round(corr[0, 1:], 2), "  (+1 完全同步, -1 完全交替)")

# %% [markdown]
# FR 和 RL 同步、和另外两只交替——**对角小跑（trot）**。
# 现在回头看第 2 部分的问题：机身 roll 晃动的频率是多少？

# %%
r = d["rpy"][walk, 0]; r = r - r.mean()
roll_hz = freq[np.argmax(np.abs(np.fft.rfft(r)))]
print(f"roll 晃动的主频 {roll_hz:.2f} Hz  vs  步频 {step_hz:.2f} Hz")

# %% [markdown]
# 一样。机身的晃动就是迈步带来的。
#
# ### 改坏它：阈值选错
#
# **预测**：把 `CONTACT` 从 30 改成 150（比满载还高），占空比会变成多少？"哪两只脚同步"的结论还在吗？

# %%
for thr in (30.0, 80.0, 150.0):
    c = d["foot_force"][walk] > thr
    print(f"阈值 {thr:>5.0f}: 占空比 {np.round(c.mean(axis=0), 2)}")

# %% [markdown]
# 阈值太高时"永远没着地"，后面所有结论都失去意义，而程序不会报错。
# 所以每次用阈值做判断，都先像上面那样**把原始曲线画出来看一眼**。
#
# ## 换成 G1 会怎样
#
# G1 的 `LowState`（`unitree_hg`）里**没有** `foot_force`、`power_v`、`bms_state`；`motor_state[i].temperature` 是**两个数**（两个测温点）。
# 所以这四个问题在 G1 上：IMU ✔、温度 ✔（取两点中较高的）、电池 ✘（要另找 Topic）、步态 ✘（只能从关节角推断）。
# 公共库的 `lowstate_to_dict(msg, "g1")` 已经处理了这些差异，12 节会用到。
#
# ## 小结
# - 四类信息的位置：`imu_state` / `power_v, power_a, bms_state` / `motor_state[i].temperature, tau_est` / `foot_force`。
# - 分析套路：**画原始曲线 → 定阈值或指标 → 找对照组**。
# - 步频、占空比、对角同步都能从足端力算出来；机身晃动的频率 = 步频。
#
# ## 课后练习
# 1. 写 `health_report(d) -> dict`，返回 `{最大倾角, 平均功率, 预计剩余分钟, 最热电机名, 是否有离群电机}`。"离群"自己定义并说明理由。
# 2. 站立时四只脚的力并不完全相等。算出前后腿的受力比，推断重心偏前还是偏后。
# 3. 用 SDK 订阅 `rt/lowstate` 现场采 10 秒，套用本节的步频分析（没有真机时用回放数据）。
#
# **下一节**：这些分析都是我们自己写的 NumPy。ROS 2 生态里有大量现成工具（RViz、Nav2、rosbag……），但它们不认识 `unitree_go/LowState`。
# 怎么把机器人的数据"翻译"成 ROS 2 的标准消息？
