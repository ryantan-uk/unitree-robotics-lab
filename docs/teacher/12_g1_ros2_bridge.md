# 12 · 把 G1 也接进 ROS 2，并盘点它和 Go2 到底差在哪

**带走的一句话**：两种机器人相同的远比不同的多；不同的那几处，用参数关起来。

## 演示顺序（45 分钟）

| 时间 | 做什么 | 停在哪 |
|---|---|---|
| 0–8 | `robot_bridge` 换成 `robot:=g1`：29 个关节、`/odom` | 里程计 Topic 未核实 → "不确定的东西做成参数" |
| 8–15 | 读桥的源码，数分叉点（4 处）；两种速度请求对比 | 让学生先猜有几处 `if` |
| 15–25 | 用程序 diff 两个消息包的 5 个消息 | 三个问题 |
| 25–30 | `lowstate_to_dict` 在两种机器人上的键 | |
| 30–40 | **总表**：学生先自己填 5 分钟，再对答案 | 这是 Part 2 的收束，留足时间 |
| 40–45 | 改坏：`robot:=go2` 对着 G1 数据 → 0 条；小结 | 预测 |

## 课堂提问

1. **同一个桥用在 G1 上要改几处？**
   四处：消息类型（`unitree_go` → `unitree_hg`）、里程计 Topic 名、关节名表、速度指令的 API（1008 `Move` → 7105 `SetVelocity` 带 `duration`）。IMU、TF、里程计转换全部共用。
2. **`MotorState.temperature` 在 G1 上是 `int16[2]`，07 节的"最热电机"分析要改哪里？**
   取每个电机两个测温点的较大值：`max(ms.temperature)`。`common.parsing.motors_to_array` 已经处理了（`hasattr(v, "__len__")`）。
3. **`robot:=go2` 对着 G1 的数据，`/lowstate` 上既有发布者又有订阅者，为什么收不到？**
   类型不同（`ros2 topic info` 会列出两种类型）。DDS 按名字 + 类型匹配，两边不会配对。

## 常见误解

- **"G1 需要另写一套代码。"** 程序 diff 显示 `MotorCmd` 的 5 个控制量完全相同、IMU 字段相同、遥控器字节相同。`common/` 里绝大部分函数只需要一个 `robot` 参数。
- **"`rt/odommodestate` 是官方确认的。"** 三个仓库的源码里没有出现。课程数据采用了它，并把它做成参数。上真机第一件事是 `ros2 topic list`。
- **"29 个关节的数据都有意义。"** 23 自由度机型上有 6 个无效（`G1_INVALID_23DOF`）；腰部锁定的机型 13、14 号也无效（源码注释）。
- **总表不是背的。** 每一行都对应前面某一节亲手遇到的现象；填表时让学生说出"在哪一节撞见的"。

## 承上启下

- 承 00–11 的全部差异点；这是 Part 2 的总结课。
- 启 13：差异用参数隔离的做法直接用于写聚合节点。
- 启 18：综合实验换 G1 "一个字都不用改"的原因在这里。

## 课后作业与参考解

1. **`is_upright`**：
   ```python
   def is_upright(msg, robot):
       d = lowstate_to_dict(msg, robot)
       ok = tilt_deg(d["rpy"][0], d["rpy"][1]) < 15
       if robot == "go2":
           return ok and d["foot_force"].sum() > 100
       knee = (d["q"][G1_JOINT_ID["left_knee"]] + d["q"][G1_JOINT_ID["right_knee"]]) / 2
       return ok and knee < 1.0
   ```
2. **再加两行**：例如灵巧手（`unitree_hg` 有 `HandCmd` / `HandState`，Go2 没有对应物）；`SportModeState` 里的 `foot_force`、`foot_position_body`、`gait_type` 对 G1 没有意义。
3. **23 自由度**：下标 13、14（腰 roll、pitch）和 20、21、27、28（左右腕 pitch、yaw）。
