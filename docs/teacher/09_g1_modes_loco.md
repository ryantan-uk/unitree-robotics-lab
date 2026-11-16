# 09 · G1：先切对模式，才走得起来

**带走的一句话**：G1 的高层运动是切状态机（Damp → Squat2StandUp → Start → 走）；速度指令必须带时长。

## 演示顺序（45 分钟）

| 时间 | 做什么 | 停在哪 |
|---|---|---|
| 0–5 | 安全要求（吊架、两人在场）；G1 没有 `StandUp` | 先讲安全再讲技术 |
| 5–15 | 读 `LocoClient` 源码：都是 `SetFsmId`；FSM 编号表；官方顺序 | 让学生自己从打印的源码里读出编号 |
| 15–22 | `g1_stand_and_walk()`（只打印）；`Move(..., True)` = 10 天 | **864000 秒是多久？** 让学生心算 |
| 22–33 | 录制数据：请求时间线、高度、膝角与力矩、x 位移 | 三个看图问题 |
| 33–37 | 时长到了自动停；与 Go2 `Move` 的对比 | |
| 37–41 | `MotionSwitcherClient`：三个方法；`CheckMode` 离线得 3102 | 为 11 节铺垫 |
| 41–45 | 读源码找陷阱（三问）；Go2 对比表；小结 | |

## 课堂提问

1. **`Move(0.3, 0, 0, True)` 之后程序崩溃了，会发生什么？**
   G1 会按 0.3 m/s 一直走（指令时长 864000 s），直到有人发新指令或用遥控器接管。所以本课程只用带明确时长的 `SetVelocity` 或不带 `True` 的 `Move`。
2. **蹲着和站直，膝关节力矩哪个大？**
   蹲着大（数据里约 25 vs 8 N·m）。膝盖越弯，体重对膝关节的力臂越长。和人半蹲更累是一个道理。
3. **Go2 的 `SportClient` 连到 G1 上会怎样？**
   服务名都是 `sport`，所以请求能送达，但 API 号不同（1xxx vs 7xxx），机器人返回"未实现"（3203）。不会报连接错误——又一种静默的错。

## Notebook 里"读源码找陷阱"的答案

1. `Squat2StandUp()` 和 `StandUp2Squat()` 发的是同一个 FSM id 706，方向由机器人**当前状态**决定（蹲着就站起，站着就蹲下）。所以已经站着时再调一次 `Squat2StandUp()`，机器人会蹲下去。对策：先 `GetFsmId()` 或看高度再决定发不发（`posture_server.py` 就是这么做的）。**此行为依据源码中两个方法 id 相同推断，需在真机上确认。**
2. `HighStand()` 传 `UINT32_MAX`、`LowStand()` 传 0，说明这个参数不是以米为单位的高度，而是某种归一化/哨兵值。单位在源码里没有说明——**未核实，不要自己传 0.9 之类的数**。
3. 不冲突：RPC 按"服务名 + API 号"寻址。`LocoClient.SetTaskId` 发往 `rt/api/sport/request`，`G1ArmActionClient.ExecuteAction` 发往 `rt/api/arm/request`，是两个不同的 Topic。

## 常见误解

- **"G1 只是多了两条腿少了两条腿的 Go2。"** 控制接口的理念不同：Go2 是"一个动作一个 API"，G1 是"状态机 + 少量参数"。
- **"FSM id 是连续的 / 有规律的。"** 0、1、3、500、702、706，没有规律，只能查源码。课程只用源码里出现过的这几个。
- **"`Start()` 是可选的。"** 官方示例的序列里，行走前要进入主运控。合成数据的模型也要求 `Start` 之后才响应速度——真机在其他状态下发速度指令会怎样，属于待验项。
- **"MotionSwitcher 的模式名我可以猜。"** 不同机型/固件不同，源码里没列。上真机后 `CheckMode()` 打印出来记下。

## 承上启下

- 承 06：同样的任务（站、走、停）换一台机器人。
- 启 10：`WaveHand`（`SetTaskId`）引出手臂。
- 启 11：`MotionSwitcherClient.ReleaseMode()` 是低层控制的前置步骤。
- 启 12：里程计 Topic 未核实的问题。

## 课后作业与参考解

1. **`wait_fsm`**：
   ```python
   def wait_fsm(loco, target, timeout=8.0):
       t_end = time.monotonic() + timeout
       while time.monotonic() < t_end:
           code, fsm = loco.GetFsmId()
           if code == 0 and fsm == target:
               return True
           time.sleep(0.2)
       return False
   ```
   注意：706 是"动作"而不是稳定状态，站起后 `GetFsmId` 返回什么需在真机上看；更稳妥的是等高度。
2. **膝力矩峰值**约 25 N·m，出现在高度曲线刚开始上升处（蹲得最深时）。
3. **状态机图**：从源码能确认的只有"有哪些 id"；从官方示例能确认的箭头是 1→706、706→500。其余箭头都应标为"猜测，待真机验证"。重点是让学生区分"确认的"和"猜的"。
