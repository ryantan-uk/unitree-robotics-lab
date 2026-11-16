# 15 · 把这一刻录下来，以后想放几遍放几遍

**带走的一句话**：bag 里每一行是"哪个 Topic、何时到达、原始字节"；录完立刻 `ros2 bag info`。

## 演示顺序（45 分钟）

| 时间 | 做什么 | 停在哪 |
|---|---|---|
| 0–4 | 为什么要录：真机实验昂贵且不可重复 | |
| 4–11 | 录 5 秒；`ls` 和 `ros2 bag info` | "5 秒的 `/lowstate` 应该多少条？对得上吗" |
| 11–20 | 用 `sqlite3` 打开 `.db3`：三张表；反序列化一条 | "读 `unitree_go` 的 bag 为什么必须装 `unitree_go`" |
| 20–24 | `read_bag` / `bag_info`；画一条曲线 | |
| 24–30 | 关掉数据源 → 0 条；回放自己录的 → 有数据 | 这一步让学生真正相信"离线 = 回放" |
| 30–37 | 倍速：到达频率变，`tick` 间隔不变；讲 `--clock` / `use_sim_time` | 预测 |
| 37–41 | 名字打错：不报错，录不到 | 预测；讲"三步检查" |
| 41–45 | 数据量估算；录什么的建议表；G1 的 bag；小结 | |

## 课堂提问

1. **bag 里存的"时间"是什么时间？**
   录制程序**收到**这条消息的时刻（接收端时钟）。消息内部如果自带时间戳（`header.stamp`、`tick`），那是发送端的，两者不同。回放按前者的间隔重发。
2. **两倍速回放时，按"到达时间"积分速度得到的位移会怎样？**
   少一半（时间间隔被压缩了一半，而速度值没变）。用消息自带的时间戳积分则不受影响。这就是为什么算法里要用消息时间而不是 `now()`。
3. **Topic 名打错为什么不报错？**
   `ros2 bag record` 的语义是"订阅这些名字"；名字现在不存在不代表以后不会出现（节点可能晚启动），所以它只会等。

## 常见误解

- **"bag 是一个文件。"** 是一个目录；拷贝、发给别人时要整个目录一起（`metadata.yaml` 丢了很麻烦）。
- **"回放的数据和实时数据在节点看来有区别。"** 在 DDS 层没有区别，Topic 名、类型、内容完全一样。区别只在时间（要 `--clock`）和"不可交互"（机器人不会响应你的指令）。
- **"录全部 Topic 最保险（`-a`）。"** 真机上点云、图像很大，几分钟就上 GB，还可能因为写盘慢而丢 `/lowstate`。按目的选 Topic。
- **"回放时自己的节点发的指令会影响机器人。"** 不会——bag 里的状态是录好的。所以回放只能做**开环**验证（18 节会强调）。
- **Humble 默认存储是 sqlite3**；新版 ROS 2 默认 MCAP。原理相同，`common/bagio.py` 里的 `storage_id` 要相应改。

## 承上启下

- 承 00 起每一节的 `LIVE = False`：现在学生知道那是什么了。
- 承 14：`go2_demo.npz` 的来历。
- 启 17：`--clock` + `use_sim_time` 是建图的前提。
- 启 18：开环验证的含义。

## 课后作业与参考解

1. **只回放一个 Topic**：`ros2 bag play <bag> --topics /sportmodestate`，另一终端 `ros2 topic list` 里没有 `/lowstate`。
2. **`bag_to_csv`**：
   ```python
   def bag_to_csv(path, topic, fields, out):
       import csv, functools
       rows = read_bag(path, [topic])[topic]
       get = lambda m, f: functools.reduce(getattr, f.split("."), m)
       with open(out, "w", newline="") as fh:
           w = csv.writer(fh); w.writerow(["t"] + fields)
           for t, m in rows:
               w.writerow([t] + [list(v) if hasattr(v := get(m, f), "__len__") else v for f in fields])
   ```
3. **压缩**：点云（浮点坐标带噪声）压缩率低；`/lowstate` 里大量保留字段是 0，压缩率高。合成数据上 `/lowstate` 的 bag 通常能压到三分之一以下。
