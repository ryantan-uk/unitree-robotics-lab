# 05 · 不用 ROS 2，SDK 是怎么拿到同一份数据的？

**带走的一句话**：SDK 的 `rt/xxx` 和 ROS 2 的 `/xxx` 是同一条 DDS 总线上的同一个 Topic；高层接口就是两条 Topic 上的一问一答。

## 演示顺序（45 分钟）

| 时间 | 做什么 | 停在哪 |
|---|---|---|
| 0–8 | 官方三步；看 `init_sdk` 源码；`ChannelSubscriber` 回调模式和 `Read` 轮询模式 | 对比 01 节："没有节点、没有 spin" |
| 8–16 | SDK 与 ROS 2 同时收，比 `tick`；SDK 发、ROS 2 收 | "tick 完全相同的有多少条"出来时停 |
| 16–20 | 改坏：类型不对 / 名字少一个字母 → 0 条 | 预测 |
| 20–32 | RPC：画图讲机制；从数据里抓请求和应答，打印表 | 表出来后停：三个问题 |
| 32–37 | 手工拼请求；`gate.call`；官方 `SportClient` | 第一次正式介绍 `gate`，讲清规则 |
| 37–41 | Python / C++ 对照表；运行两个 C++ 示例 | |
| 41–45 | G1：消息类型、API 号 7xxx、`arm` 服务；小结 | |

## 课堂提问

1. **为什么请求有 21 条而应答只有 2 条？**
   `Move` 是连续高频发送的，SDK 用 `_CallNoReply` 发它（请求头里 `noreply=True`），机器人不回应答。只有 `StandUp`、`StopMove` 这类一次性指令有应答。
2. **应答怎么和请求配对？**
   请求头里的 `identity.id`（SDK 用 `time.monotonic_ns()` 生成）。应答原样带回这个 id，客户端按 id 找到正在等待的那次调用。
3. **用 `unitree_hg` 的 `LowState_` 订阅 Go2 的 `rt/lowstate`，为什么不是收到乱码而是什么都收不到？**
   DDS 按"名字 + 类型名"匹配。类型名不同，发布者和订阅者根本不会被配对，数据不会送过来，也就谈不上解析。

## 常见误解

- **"SDK2 和 ROS 2 是两套独立的通信。"** 底层都是 CycloneDDS。本节的三个实验（同 tick、互发、类型匹配）都是为了打掉这个误解。
- **"`SportClient.StandUp()` 是远程函数调用，有专门的协议。"** 就是一条 JSON 请求消息加一条应答消息。
- **"`ChannelFactoryInitialize` 可以调多次换网卡。"** 整个进程只能初始化一次（SDK 内部是单例）；换网卡要重启内核。
- **`[Reader] take sample error`** 是 SDK 关闭订阅时自己打印的，无害，提前告诉学生免得慌。
- **离线时 `init_sdk` 为什么要改 SDK 的配置？** 回环网卡不支持组播，且 SDK 内置配置同机最多 10 个 DDS 参与者；真机模式下走的就是官方原样的 `ChannelFactoryInitialize(0, 网卡)`。

## 承上启下

- 承 04 的悬念：`posture_server` 是往 `/api/sport/request` 发 Request。
- 启 06：用 `SportClient` 写完整脚本。
- 启 08：`RosRpcClient` = 用 ROS 2 重写的 SportClient，依据就是本节。

## 课后作业与参考解

1. **订阅 `rt/sportmodestate`**：`from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_`；`sub = ChannelSubscriber("rt/sportmodestate", SportModeState_); sub.Init(lambda m: print(list(m.position)), 10)`。
2. **`Move` 的间隔**：约 0.5 s，由调用方决定——录制脚本里是 `hold(lambda: c.Move(...), seconds, period=0.5)`。
3. **错误码**：发不出去 `RPC_ERR_CLIENT_SEND = 3102`；发出去但超时 `RPC_ERR_CLIENT_API_TIMEOUT = 3104`（`_CallBase` 里 `future.GetResult(timeout)` 失败的分支）。
