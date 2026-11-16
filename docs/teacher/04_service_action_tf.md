# 04 · 问它一句、托它办件事、问它身体在哪

**带走的一句话**：一问一答用 Service，要花时间的事用 Action，坐标系之间的关系问 TF。

## 演示顺序（45 分钟）

| 时间 | 做什么 | 停在哪 |
|---|---|---|
| 0–5 | 三种"有来有回"的对照表；启动三个节点 | 说明 `enable_motion` 默认 false |
| 5–15 | Service：`interface show`、命令行调用、代码调用 | "`call_async` 后为什么必须 spin" |
| 15–28 | Action：定义三段；`send_goal`；先等趴下再请求站起；画反馈曲线 | 反馈图出来时停：这是 Service 做不到的 |
| 28–33 | 三种不成功：拒绝 / 超时中止 / 取消；看隐藏的 Topic 和 Service | 三个都先预测 |
| 33–41 | TF：两段变换、跨段查询、坐标系树；查不存在的坐标系 | "没发布过 odom→utlidar_lidar，为什么能查到" |
| 41–45 | G1：电量 -1、`lie` 被拒绝、客户端代码没改；小结 | |

## 课堂提问

1. **"站起来"为什么用 Action 而不是 Service？**
   要花几秒；调用方想看进度（反馈）、可能要取消、需要区分"没接受 / 做到一半失败 / 被取消 / 成功"。Service 只有一次应答，这些都表达不了。
2. **`posture_server` 没有给机器人发指令，为什么 Action 还显示 SUCCEEDED？**
   `enable_motion=false` 时它只观察机身高度是否到达目标。回放数据里机器人自己会站起来，所以观察到了"到位"。结果的 `message` 里写着 `[not sent] observed only`——提醒学生读结果不能只看状态码。
3. **G1 上客户端代码一行没改，差异去哪了？**
   被关在服务端：`posture_server` 根据 `robot` 参数决定允许哪些姿态、发哪个 API。这就是把硬件包装成标准接口的意义。

## 常见误解

- **"Action 是一种新的通信机制。"** 它是 3 个 Service（发目标、取结果、取消）+ 2 个 Topic（反馈、状态）的组合，演示里用 `--include-hidden-topics` 能看到。
- **"同步 `client.call()` 更简单，用它就行。"** 在回调里用它而节点是单线程执行器时会死锁（等应答的线程正是负责取应答的线程）。`call_async` + future 或多线程执行器。
- **"TF 需要发布所有两两之间的变换。"** 只发布树上相邻的边，查询时自动连乘。
- **"离线时 Action 成功 = 指令执行成功。"** 见提问 2。

## 承上启下

- 承 03：回调组和多线程执行器在 `posture_server` 里派上用场。
- 承 02：手工的坐标变换 → TF。
- 启 05：`posture_server` 最终靠什么让机器人站起来？——`/api/sport/request`。这是 Part 1 → Part 2 的桥。
- **这是全课第一处可能让真机动的地方**（`enable_motion:=true`），务必带学生读一遍 `common/safety.py` 的五条检查。

## 课后作业与参考解

1. **`wait_until_upright`**：循环里 `ask_summary()`，`upright` 为真返回；每次间隔 0.5 s。缺点：轮询有延迟和额外流量；服务端不知道有人在等，无法主动通知或被取消；没有统一的超时/失败语义。
2. **雷达前方 1 m 的点在 odom 系**：
   ```python
   p, yaw = lookup("odom", "utlidar_lidar")
   pt = transform_2d(p[0], p[1], yaw) @ [1.0, 0.0, 1.0]
   ```
3. **`posture_server.py`** 用 `ReentrantCallbackGroup` + `MultiThreadedExecutor(4)`。换成默认的：`execute` 回调阻塞期间，高度订阅和 RPC 应答的回调都得不到执行——高度永远不更新，动作必然超时（`rpc.call` 还会等不到应答）。
