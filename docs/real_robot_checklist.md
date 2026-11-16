# 真机待验清单

本课程是在**没有真机**的环境里开发的。下面每一项都必须在真机上确认后才能当作事实。
做法：每项先写下预期，再按步骤做，把"实测"和"与课程的出入"填进表里；有出入的回头改对应的 Notebook / 参数。

**先 Go2 后 G1。任何会动的步骤之前，完成 `common/safety.py` 的五条检查；G1 的 C、E、F 组必须上保护吊架。**

## A. 连接（不产生运动）

| # | 验证什么 | 步骤 | 预期 | 实测 |
|---|---|---|---|---|
| A1 | 网段与连接脚本 | 网线直连；给有线网卡配同网段静态地址；`python3 scripts/connect.py` | 列出网卡并标出同网段的那块；打印"收到 /lowstate"及机型 | |
| A2 | VMware 桥接 | 在 Windows + VMware 的学生环境里重复 A1 | 桥接到有线网卡后能收到；NAT 模式收不到 | |
| A3 | Topic 清单 | `ros2 topic list -t`，与 00 节的表对比 | Go2：`/lowstate` `/sportmodestate` `/utlidar/cloud` `/api/sport/*` 等存在 | |
| A4 | **G1 的里程计 Topic 名** | 连 G1，`ros2 topic list \| grep -i -E "odom\|sport"` | 课程假设 `/odommodestate`；若不同，用 `-p odom_topic:=...`，并改 `common/robots.py` 的 `TOPICS["g1_odom"]` | |
| A5 | **G1 的点云 Topic** | `ros2 topic list -t \| grep PointCloud2` | 未知。记录 Topic 名与 `frame_id` | |
| A6 | 00–05 节 `LIVE = True` 全部跑通 | 逐节运行（这几节不发运动指令） | 无报错；数值与合成数据不同属正常 | |

## B. 频率与时序（不产生运动）

| # | 验证什么 | 步骤 | 预期 | 实测 |
|---|---|---|---|---|
| B1 | `/lowstate` 频率 | 14 节第 1、2 部分，`LIVE = True` | 约 500 Hz（文档值） | |
| B2 | `/lf/lowstate`、`/sportmodestate`、`/utlidar/cloud` 频率与最坏间隔 | 同上 | 未知，记录 | |
| B3 | 网线 vs Wi-Fi | 两种连接各测一次 B1 | Wi-Fi 的最坏间隔明显更大 | |
| B4 | 时钟漂移与相对延迟 | 录 60 秒 `/lowstate` 转 npz，跑 14 节第 3 部分 | 漂移几十 ppm 量级 | |
| B5 | Python 控制循环能否达到 500 Hz | 11 节第 4 部分（不确认闸门，仍不下发） | 在学生机上多半达不到；记录实际周期 | |

## C. 数据约定（不产生运动，但需要人手搬动/摆放机器人）

| # | 验证什么 | 步骤 | 预期 | 实测 |
|---|---|---|---|---|
| C1 | 四元数顺序 | 02 节第 3 部分，`LIVE = True` | 按 (w,x,y,z) 算出的欧拉角与 `rpy` 字段一致 | |
| C2 | 遥控器字节解析 | 02 节第 1 部分；逐个按键、推摇杆 | 键名、摇杆方向与符号与实际一致 | |
| C3 | `foot_force` 的顺序与量级 | 机器人站立，用手轻抬某一只脚；录数据 | 对应下标的读数下降；记录站立时每脚读数，更新 01 节阈值 | |
| C4 | Go2 雷达外参 | 机器人正对一面平整的墙；16 节第 3 部分 | 用正确外参后墙面竖直。记录外参，填入 `robot_bridge` 的 `lidar_xyz`（旋转需另发静态 TF） | |
| C5 | URDF 与真机一致 | 16 节火柴人 / RViz；量实际机身高度 | TF 算出的脚高与卷尺一致（±2 cm） | |

## D. Go2 高层运动（会动）

| # | 验证什么 | 步骤 | 预期 | 实测 |
|---|---|---|---|---|
| D1 | `StandUp` / `StandDown` | 06 节，`gate.confirm(...)` 后只执行这两条 | 返回 0；站起用时记录 | |
| D2 | **`Move` 的失效时间** | 站立后发**一条** `Move(0.2, 0, 0)`，之后不再发；用里程计量多久停下 | 未知（合成模型为 1 s）。记录 | |
| D3 | 速度响应时间常数 | 06 节课后练习 2 | 记录，用于修正开环距离 | |
| D4 | `demo()` 全程 | 速度减半先跑一次 | 走出 L 形；实测距离、转角与指令的偏差记录 | |
| D5 | Action 服务端真的下发 | 04 节，`posture_server` 加 `-p enable_motion:=true` | `stand` / `lie` 成功且机器人照做；默认高度参数（0.32 / 0.07）是否需要调整 | |
| D6 | `/cmd_vel` 桥 | 08 节末尾的两条命令，`enable_motion:=true`，0.2 m/s | 机器人前进；Ctrl-C 后停下的时间 = D2 | |

## E. G1 高层运动与上肢（会动，上吊架）

| # | 验证什么 | 步骤 | 预期 | 实测 |
|---|---|---|---|---|
| E1 | 查询不动 | `loco.GetFsmId()`、`msc.CheckMode()` | 返回 0；**记录模式名和当前 FSM id** | |
| E2 | `Damp` → `Squat2StandUp` → `Start` | 09 节，逐条确认 | 各步返回 0；站起用时记录 | |
| E3 | **已站立时再发 706** | 吊架保护下，站立后再调一次 `Squat2StandUp()` | 课程推断会蹲下。记录实际行为 | |
| E4 | **未 `Start` 时发速度** | 站立但未 `Start`，发 `SetVelocity(0.1, 0, 0, 1.0)` | 未知。记录返回码和是否移动 | |
| E5 | `SetVelocity` 时长 | `Start` 后发 `SetVelocity(0.2, 0, 0, 2.0)` | 约 2 秒后自动停 | |
| E6 | 动作库 | 10 节：`high wave`；`shake hand` 后 `release arm` | 哪些动作自动回位、哪些需要 release，与课程所述一致 | |
| E7 | `rt/arm_sdk` | 10 节 `arm_sdk_run`，幅度减半，起点取自实时 `LowState` | 左臂平滑抬起并回位；交还时无明显抖动 | |
| E8 | **权重混合方式** | 手臂举起后把权重降到 0.5 并保持 | 若为线性混合，关节停在中间位置。记录 | |

## F. 低层控制（会动，G1 上吊架 / Go2 趴着或吊起）

| # | 验证什么 | 步骤 | 预期 | 实测 |
|---|---|---|---|---|
| F1 | `ReleaseMode` | 11 节 `demo.release_high_level()` | `CheckMode` 的 `name` 变空；机器人失去支撑力（预期内） | |
| F2 | 低层抬臂 | 11 节 `LowLevelArmDemo`，幅度 0.3 rad | 平滑到位；记录实际跟踪误差 | |
| F3 | **CRC 错误的指令** | 故意不重算 CRC 发 1 秒 | 课程假设被丢弃、机器人无反应。记录 | |
| F4 | **`mode_machine` 不匹配** | 故意填错发 1 秒 | 同上 | |
| F5 | 增益与稳态误差 | Go2 趴姿抬起一条腿到固定角度，记录误差；加力矩前馈再试 | 误差随 `kp` 增大而减小；加前馈后减小 | |
| F6 | 恢复运控 | 遥控器或 `SelectMode(E1 记录的模式名)` | 恢复高层控制 | |

## G. 应用层

| # | 验证什么 | 步骤 | 预期 | 实测 |
|---|---|---|---|---|
| G1 | 看门狗反应时间 | 13 节故障二，`LIVE = True`，在标注时刻**拔网线** | 约 0.5–1.1 s 内 `/cmd_vel` 归零 | |
| G2 | 真机录包与回放 | `data/README.md` 的录制命令；用自己的 bag 重跑 01、02、07、14 | 全部跑通；更新各节阈值 | |
| G3 | **RViz2** | 有桌面的环境：`rviz2 -d config/go2.rviz` | 模型、TF、点云、激光显示正常。**此配置文件未在图形界面下验证过** | |
| G4 | 真机建图 | 17 节"上真机"第 1、2 步，遥控慢走一圈 | 地图闭合、无重影；量房间尺寸对比 | |
| G5 | **Nav2 闭环导航** | 另装 `ros-humble-nav2-bringup`，启动其 `navigation_launch.py`（配合本仓库的建图流水线），`robot_bridge` 设 `enable_motion:=true`，限速 0.3 m/s，在 RViz 里给目标 | 机器人沿规划路径到达。**本课程完全未验证这一步**，需要自行调 Nav2 控制器参数 | |
| G6 | 综合实验闭环 | 18 节"上真机"五步 | 巡逻节点让机器人在墙前转向；记录实际触发距离 | |

## 已在无真机环境下验证过的（供对照）

- `scripts/install.sh` 在 Ubuntu 22.04 x86_64（OrbStack 虚拟机，Apple Silicon 上转译运行）上装完，6 个 colcon 包（含 C++ 示例）编译通过。
- `python3 -m pytest`：49 项通过。
- `jupyter nbconvert --execute` 逐个执行 19 个 Notebook，无报错（`LIVE = False`）。
- 官方 Python SDK 的调用（`SportClient`、`LocoClient`、`G1ArmActionClient`、`MotionSwitcherClient`、`ChannelSubscriber/Publisher`、`CRC`、`RecurrentThread`）
  均被实际执行过：生成样本数据时由它们驱动一个回环上的替身程序（`tools/datagen/`），请求、应答、状态都录进了 bag。
- slam_toolbox 在回放数据上建出与真值吻合的地图；Nav2 `planner_server` 在该地图上规划出路径。

**未做的**：在全新机器上从零重跑安装脚本的最终复验（开发中途对脚本有两处修改：`--symlink-install` 和固定 `matplotlib-inline` 版本，是在已装好的机器上手工补做的）；
由不了解构建过程的人以学生、老师身份试用并反馈。
