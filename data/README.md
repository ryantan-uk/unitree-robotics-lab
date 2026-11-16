# data/ —— 样本数据

**这里的全部数据都是合成的，不是从真机录的。** 生成方式：`python3 -m tools.datagen.make_data`
（用官方 SDK 的真实调用驱动一个简化的机器人替身并录包；topic 名、消息类型、RPC 请求内容与真机一致，数值是合成的）。
足端力读数、温升、电池曲线、关节力矩等数值的量级**不要当作真机参考**。

| 文件 | 内容 | 用在 |
|---|---|---|
| `bags/go2_demo` | Go2：趴→站→前进→左转→前进→停，约 14 s。`/lowstate` `/sportmodestate` `/utlidar/cloud` `/api/sport/*` `/api/motion_switcher/*` | 00–08、13–16 |
| `bags/g1_demo` | G1：蹲→站→Start→前进→挥手→`arm_sdk` 抬左臂→交还，约 20 s。`/lowstate` `/odommodestate` `/utlidar/cloud` `/arm_sdk` `/api/sport/*` `/api/arm/*` | 00–03、09–12 |
| `bags/go2_slam` | Go2 绕房间走一圈，约 45 s。`/sportmodestate` `/utlidar/cloud` `/lf/lowstate` | 17、18 |
| `go2_demo.npz` `g1_demo.npz` | 上面两个 bag 转成的 NumPy 数组 | 02、14 |
| `go2_long_run.npz` | Go2 巡逻 5 分钟，50 Hz：姿态、关节、力矩、温度、足端力、电池 | 07 |
| `lowlevel_runs.npz` | 低层关节控制对照实验（好增益 / kd=0 / 无前馈 / 阶跃目标），500 Hz | 11 |
| `room.json` | 合成数据所在的虚拟房间（墙面线段），画图时当"真值"对照 | 02、16、17 |

## 换成你自己的真机数据

连上真机（`python3 scripts/connect.py && source scripts/env.sh`）后，用下面的命令录同名 bag 覆盖即可，Notebook 不用改：

```bash
# Go2 (边录边用遥控器或 06 节的代码让它 站起 -> 走 -> 转 -> 停)
ros2 bag record -o data/bags/go2_demo /lowstate /sportmodestate /utlidar/cloud \
    /api/sport/request /api/sport/response /api/motion_switcher/request /api/motion_switcher/response

# G1 (G1 的里程计 topic 名未在官方源码中核实，先 ros2 topic list 确认)
ros2 bag record -o data/bags/g1_demo /lowstate /odommodestate /arm_sdk \
    /api/sport/request /api/sport/response /api/arm/request /api/arm/response

# 建图用: 让 Go2 慢速绕房间一圈
ros2 bag record -o data/bags/go2_slam /sportmodestate /utlidar/cloud /lf/lowstate

# bag -> npz
python3 -c "from common.bagio import bag_to_arrays; import numpy as np; np.savez_compressed('data/go2_demo.npz', **bag_to_arrays('data/bags/go2_demo', 'go2'))"
```

真机的 `/lowstate` 约 500 Hz，一分钟约 35 MB；录短一点，或改录 `/lf/lowstate`。
