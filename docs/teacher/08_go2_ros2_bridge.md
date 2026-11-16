# 08 · 把 Go2 接进 ROS 2 的世界

**带走的一句话**：桥 = 订阅原生消息 → 换成标准消息 → 发布；写完先做一次重力检验。

## 演示顺序（45 分钟）

| 时间 | 做什么 | 停在哪 |
|---|---|---|
| 0–5 | 标准消息对照表：工具想要什么，Go2 给什么 | |
| 5–17 | 写 `MiniBridge`（SDK 收、rclpy 发）；验证 `/imu/data` `/joint_states` | "bridge 没被 spin，为什么能发消息" |
| 17–25 | 错误一：四元数顺序；重力检验（正确 +9.8，抄错后朝天） | 先预测抄错会怎样 |
| 25–28 | 错误二：速度坐标系（讲，不演示） | |
| 28–36 | 完整版 `robot_bridge`：`/odom`、TF、Topic 列表 | 侧向速度约 0：说明转到机体系是对的 |
| 36–41 | `/cmd_vel` → Request；桥默认不下发 | 对比 `gate` |
| 41–45 | `decimate` 与频率；小结 | 预测两种 decimate 的频率 |

## 课堂提问

1. **`MiniBridge` 从没被 spin 过，为什么还能发布？**
   只有"被动触发"的东西（订阅回调、定时器、服务）需要 spin。`publish()` 是主动调用，这里由 SDK 的接收线程触发。
2. **四元数抄错顺序，程序不报错，怎么发现？**
   重力检验：用姿态把加速度计读数转到世界系，应为 [0, 0, +9.8]。抄错后 z 变成负的——机器人"肚皮朝天"。
3. **为什么 `/cmd_vel` 的桥不用 SDK 的 `SportClient`，而是直接发 `unitree_api/Request`？**
   两者是一回事（05 节）；纯 ROS 2 实现少一个依赖，也便于用 `ros2 topic echo /api/sport/request` 调试。`RosRpcClient` 只有 40 行。

## 常见误解

- **"在 SDK 回调里调 rclpy 不安全。"** `publish()` 是线程安全的。不安全的是在多个线程里同时改同一个 Python 对象——本例每次回调都新建消息对象，没有这个问题。
- **"Unitree 的四元数和 ROS 的一样。"** Unitree 是 (w,x,y,z)，`geometry_msgs/Quaternion` 的字段是 x,y,z,w。
- **"`Odometry.twist` 是世界系速度。"** 约定在 `child_frame_id`（机体）系；`SportModeState.velocity` 是世界系，要旋转。
- **"桥转发越快越好。"** RViz 每秒 30 帧，`/joint_states` 发 500 Hz 只是浪费带宽。`decimate` 就是为此。
- **`JointState.name`** 必须与 URDF 关节名逐字一致，否则 16 节的模型不动。

## 承上启下

- 承 02、05、06：四元数顺序、RPC 机制、`Move`。
- 启 09–12：换 G1，同一个桥改哪里。
- 启 13、16、17：`/odom`、TF、`/joint_states`、`/cmd_vel` 是 Part 3 所有节点的输入输出。

## 课后作业与参考解

1. **足端力**：`from std_msgs.msg import Float32MultiArray`；`pub.publish(Float32MultiArray(data=[float(v) for v in m.foot_force]))`。
2. **纯 ROS 2 版**：把 `ChannelSubscriber` 换成 `create_subscription(LowState, "/lowstate", ...)`，此时节点**需要被 spin**。SDK 版不依赖 `unitree_go` 的 ROS 消息包、回调不占执行器；纯 ROS 2 版只有一套 DDS、能用 `use_sim_time` 和 launch 的全部机制（课程成品用的是这种）。
3. **手算**：yaw = 90°，世界系 (0, 0.4)：`linear.x = cos(90°)·0 + sin(90°)·0.4 = 0.4`，`linear.y = −sin(90°)·0 + cos(90°)·0.4 = 0`。机器人朝北、向北走 = 向前走。
