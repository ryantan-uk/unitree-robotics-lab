# Unitree Robotics 开发实践：从 ROS 2 基础到 Go2 / G1 SDK 真机开发

`unitree-robotics-lab` —— 一门 19 节的动手课。面向大学高年级学生和具身智能初学者。
学完后，你能独立写出一个**读取状态并控制 Go2 或 G1 的 ROS 2 节点**。

- 每节一个 Notebook，从一个问题出发（"机器人现在站没站稳？"），走同一条线：**跑通 → 拆开 → 改坏（先预测，再验证）**。
- 同一个任务在 **Go2（四足）和 G1（人形）上各做一遍**，差异自己发现，最后汇成一张表（12 节）。
- SDK 部分用官方 **Python SDK**（`unitree_sdk2_python`）；另有两个 `colcon` 现场编译的 C++ 对照示例。
- **没有真机也能上完全课**：每个 Notebook 第一格有开关 `LIVE`。`LIVE = False`（默认）时回放 `data/` 里的 Rosbag，
  运动指令只打印不下发；`LIVE = True` 连真机，代码一个字不用改。
- 每节配一页讲稿（`docs/teacher/`）：带走的一句话、演示顺序、三个提问及答案、常见误解、作业参考解。

> **关于数据**：`data/` 里的样本全部是**合成**的（生成方式见 `data/README.md`）。Topic 名、消息类型、接口签名逐项对照官方源码核对过
> （`docs/source_audit.md`）；数值（足端力读数、温度、频率……）不是真机的。必须上真机才能确认的事项列在 `docs/real_robot_checklist.md`。

## 快速开始

环境：Ubuntu 22.04（学生：Windows + VMware Workstation，网络选**桥接模式**；详见 `docs/environment.md`）。

```bash
git clone <本仓库> ~/unitree-robotics-lab        # 路径里不要有空格
cd ~/unitree-robotics-lab
bash scripts/install.sh                          # 一条命令装完: ROS 2 Humble、CycloneDDS、Unitree 消息与 SDK、课程节点 (约 20–40 分钟)
source scripts/env.sh                            # 每开一个新终端都要执行
jupyter notebook notebooks/                      # 从 00 开始
```

自检：

```bash
python3 -m pytest                                # common/ 的单元测试
bash scripts/run_all_notebooks.sh                # 从头执行全部 Notebook (约 25 分钟)；可加编号只跑几节: ... 06 07
```

连真机（有机器人时）：

```bash
python3 scripts/connect.py                       # 选网卡 -> 生成 cyclonedds.xml -> 设好环境变量 -> 确认收到 Topic
source scripts/env.sh
# 然后把 Notebook 第一格改成 LIVE = True
python3 scripts/connect.py --offline             # 切回离线
```

## 课程地图

**Part 1 · 用 ROS 2 读懂机器人**

| 节 | 问题 | 学到的工具 | 做成的事 |
|---|---|---|---|
| [00](notebooks/00_setup_connect.ipynb) | 我的电脑能"听见"机器人吗？ | DDS、Domain、网卡、`ros2 topic` | 列出机器人广播的 Topic；把连接弄断再修好 |
| [01](notebooks/01_first_topic.ipynb) | 它现在站没站稳？ | 订阅者、回调、QoS | 写出 `is_standing()`，阈值从数据里读 |
| [02](notebooks/02_struct_numpy.ipynb) | 遥控器按了哪个键？机身歪了多少度？ | `struct`、NumPy、坐标变换 | 解析 40 字节遥控器数据；把点云画到房间里 |
| [03](notebooks/03_concurrency.ipynb) | 几路数据同时来，怎么一个都不耽误？ | 执行器、回调组、线程、`asyncio` | 消除慢回调造成的 150 ms 数据空窗 |
| [04](notebooks/04_service_action_tf.ipynb) | 怎么问它一句、托它办件事、问它身体在哪？ | Service、Action、TF | 调服务、发动作目标看反馈、查坐标变换 |

**Part 2 · 用 SDK2 驱动机器人**

| 节 | 问题 | 机器人 | 做成的事 |
|---|---|---|---|
| [05](notebooks/05_sdk2_dds_channels.ipynb) | 不用 ROS 2，SDK 是怎么拿到同一份数据的？ | 共通 | 用 SDK 订阅/发布；读懂并手写一条 RPC 请求 |
| [06](notebooks/06_go2_sport_client.ipynb) | 怎么让它站起来、走出去、停下来？ | Go2 | 用 `SportClient` 写动作脚本；用数据核对它是否照做 |
| [07](notebooks/07_go2_lowstate_analysis.ipynb) | 走了五分钟，身体里发生了什么？ | Go2 | 从 `LowState` 分析姿态、电池、温度、步态；找出异常电机 |
| [08](notebooks/08_go2_ros2_bridge.ipynb) | 怎么把它接进 ROS 2 的世界？ | Go2 | 写一个 SDK → ROS 2 的桥；`/cmd_vel` → `Move` |
| [09](notebooks/09_g1_modes_loco.ipynb) | G1 为什么没有 `StandUp`？ | G1 | 写出 Damp → 站起 → 主运控 → 行走的指令序列 |
| [10](notebooks/10_g1_arm_actions.ipynb) | 手臂动起来，腿怎么还站得稳？ | G1 | 动作库；用 `rt/arm_sdk` 发自己规划的手臂轨迹 |
| [11](notebooks/11_g1_lowlevel.ipynb) | 我让关节去 1.0，它为什么停在 0.9？ | G1（对照 Go2） | 填出合法的 `LowCmd`；解释振荡与稳态误差；三道保险 |
| [12](notebooks/12_g1_ros2_bridge.ipynb) | 同一个桥能直接用在 G1 上吗？ | G1 | G1 接进 ROS 2；Go2 / G1 差异总表 |

**Part 3 · 做出完整应用**

| 节 | 问题 | 做成的事 |
|---|---|---|
| [13](notebooks/13_control_and_aggregator_nodes.ipynb) | 数据突然断了，多久之后停？ | 写状态聚合节点和带看门狗的控制节点，量出反应时间 |
| [14](notebooks/14_hz_latency_plots.ipynb) | 多快、多稳、多晚？ | 测所有 Topic 的频率、抖动、相对延迟；画姿态与关节曲线 |
| [15](notebooks/15_rosbag.ipynb) | 怎么把这一刻录下来反复用？ | 录制、检查、倍速回放；用 `sqlite3` 打开 bag 看结构 |
| [16](notebooks/16_rviz_urdf_tf_cloud.ipynb) | 12 个关节角 → 它现在什么姿势？ | URDF → TF → "火柴人"；点云变到世界系；RViz2 配置 |
| [17](notebooks/17_slam_nav2.ipynb) | 怎么画一张地图，再问"怎么走"？ | slam_toolbox 建图并与真值比；Nav2 规划路径 |
| [18](notebooks/18_capstone.ipynb) | **综合实验**：自主巡逻 | 自己搭"感知 → 决策 → 控制"完整链路，Go2 / G1 通用 |

难度是爬坡的：前一节的产出是后一节的起点（01 的判据 → 13 的聚合节点；05 的 RPC → 08 的桥 → 13 的控制节点 → 18 的巡逻）。

## 仓库结构

```
notebooks/   19 个 Notebook (学生用；不含输出)
common/      公共库: 连接、解析、关节插值与限位、运动闸门、ROS 小工具、绘图   —— Notebook 里只留教学代码
src/         ROS 2 包 (colcon 现场编译)
  unitree_lab_interfaces/   自定义 Action / Service
  unitree_lab_nodes/        桥接、状态聚合、点云转激光、姿态 Action、巡逻控制 + launch
  unitree_lab_cpp/          C++ 对照示例 (rclcpp): lowstate_reader, sport_request
data/        样本数据 (合成) + 真机录制命令
config/      DDS、slam_toolbox、Nav2 规划器、RViz 配置
docs/        讲稿 (teacher/)、源码核对表、真机待验清单、环境说明
tests/       pytest: common/ 与决策函数
scripts/     install.sh、env.sh、connect.py、run_all_notebooks.sh
tools/       维护者工具: Notebook 生成器、样本数据生成器 (学生和老师用不到)
```

## 安全

**没有显式确认，就不会有任何运动指令离开你的电脑。** 三道闸：

1. `LIVE = False` 时（默认）没有机器人，指令只打印；
2. `LIVE = True` 时，SDK 调用都经过 `gate.call()`，必须先 `gate.confirm("我已清场并手握急停")`；
3. ROS 2 侧的 `robot_bridge`、`posture_server` 默认 `enable_motion:=false`。

上真机前请读 `common/safety.py` 里的五条检查，G1 的低层与上肢实验必须使用保护吊架。

## 给老师

从 `docs/teacher/README.md` 开始。每节讲稿一页，按 45 分钟设计（11、18 两节建议各拆成两课时）。
全课 19 节，加上真机验证课，适合一个 32 学时左右的实验课程。

## 上游版本

课程内容依据以下官方仓库的指定提交核对（`scripts/install.sh` 会固定到这些版本）：

| 仓库 | 提交 | 用途 |
|---|---|---|
| `unitreerobotics/unitree_sdk2_python` | `814556d` | 课程使用的 SDK |
| `unitreerobotics/unitree_ros2` | `668d1ec` | ROS 2 消息定义（`unitree_go` / `unitree_hg` / `unitree_api`） |
| `unitreerobotics/unitree_sdk2` | `aaf69cb` | 仅用于核对接口（C++ 对照表），不安装 |
| `unitreerobotics/unitree_ros` | 安装时的 `master` | 仅取 Go2 / G1 的 URDF |
