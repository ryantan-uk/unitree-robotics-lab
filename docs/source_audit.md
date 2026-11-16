# 源码核对表

课程里出现的每个 Topic 名、消息类型、接口签名、API 号，都对照下列官方仓库的指定提交核对过。
**在源码里找不到的，一律标"未核实"并列入 `real_robot_checklist.md`，课程中出现时都有同样的标注。**

| 简称 | 仓库 | 提交 |
|---|---|---|
| py | `unitreerobotics/unitree_sdk2_python` | `814556d15970dd2ecf1c9984e845ca02ab07e206` |
| ros2 | `unitreerobotics/unitree_ros2` | `668d1ec5a05d1c38d3306bdca7d59f2ba3581a88` |
| cpp | `unitreerobotics/unitree_sdk2` | `aaf69cb56c9d5ed63d8b284008ec3a8ab999bec1` |
| urdf | `unitreerobotics/unitree_ros`（`robots/go2_description`、`robots/g1_description`） | 安装时的 `master`；限位由 `tests/test_urdf_limits.py` 自动对照 |

## 1. Topic

| DDS 名（SDK） | ROS 2 名 | 类型 | 出处 | 状态 |
|---|---|---|---|---|
| `rt/lowstate` | `/lowstate` | `unitree_go/LowState`（Go2）、`unitree_hg/LowState`（G1） | py `example/go2/low_level/go2_stand_example.py`、`example/g1/low_level/g1_low_level_example.py`；ros2 `example/src/src/` | 已核实 |
| `rt/lf/lowstate` | `/lf/lowstate` | 同上 | py `unitree_sdk2py/test/lowlevel/`；ros2 README（"lf" = low frequency） | 已核实（频率未核实） |
| `rt/lowcmd` | `/lowcmd` | `unitree_go/LowCmd`、`unitree_hg/LowCmd` | 同 `rt/lowstate` | 已核实 |
| `rt/sportmodestate` | `/sportmodestate` | `unitree_go/SportModeState` | cpp `example/go2/go2_sport_client.cpp`；ros2 README | 已核实 |
| `rt/lf/sportmodestate` | `/lf/sportmodestate` | 同上 | py、ros2 示例 | 已核实 |
| `rt/wirelesscontroller` | `/wirelesscontroller` | `unitree_go/WirelessController` | cpp 示例；ros2 README | 已核实（课程未使用，遥控器数据取自 `LowState.wireless_remote`） |
| `rt/utlidar/cloud` | `/utlidar/cloud` | `sensor_msgs/PointCloud2`，`frame_id: utlidar_lidar` | ros2 README | 已核实（Go2）。**G1 是否有此 Topic 未核实** |
| `rt/arm_sdk` | `/arm_sdk` | `unitree_hg/LowCmd` | py `example/g1/high_level/g1_arm7_sdk_dds_example.py`；cpp 示例 | 已核实 |
| `rt/api/<service>/request` | `/api/<service>/request` | `unitree_api/Request` | py `unitree_sdk2py/core/channel_name.py`、`rpc/client_stub.py`；ros2 示例 | 已核实 |
| `rt/api/<service>/response` | `/api/<service>/response` | `unitree_api/Response` | 同上 | 已核实 |
| `rt/odommodestate`、`rt/lf/odommodestate` | `/odommodestate` | `unitree_go/SportModeState` | 三个仓库的源码中均未出现；依据对 Unitree 文档的记忆 | **未核实**（课程数据中 G1 的里程计用它；节点里做成了参数 `odom_topic`） |

`rt/` 与 `/` 的对应是 ROS 2 的通用规则（DDS 层 Topic 名加前缀 `rt`），05 节用互发实验验证过。

## 2. 消息字段

全部来自 ros2 `cyclonedds_ws/src/unitree/{unitree_go,unitree_hg,unitree_api}/msg/*.msg`，课程通过 `colcon` 编译这些原始定义，没有改动。
12 节用程序列出了两个包的差异。要点：

- `unitree_go/LowState`：`motor_state[20]`、`foot_force[4]`、`bms_state`、`power_v`、`power_a`、`wireless_remote[40]`、`tick`、`crc`。
- `unitree_hg/LowState`：`motor_state[35]`、`mode_pr`、`mode_machine`、`wireless_remote[40]`、`tick`、`crc`；**无** `foot_force`、电池字段。
- `MotorCmd`（两包）：`mode q dq tau kp kd` 相同；`reserve` 长度不同。
- `unitree_hg/MotorState.temperature` 是 `int16[2]`；`unitree_go` 是 `int8`。

| 约定 | 出处 | 状态 |
|---|---|---|
| Go2 电机顺序 FR/FL/RR/RL × hip/thigh/calf，下标 0–11 | py `example/go2/low_level/unitree_legged_const.py`（`LegID`） | 已核实 |
| G1 29 个关节的下标与名字；23 自由度机型无效的 6 个 | py `example/g1/low_level/g1_low_level_example.py`（`G1JointIndex` 及注释） | 已核实 |
| 遥控器 40 字节布局与按键位序 | ros2 `example/src/src/g1/lowlevel/gamepad.hpp`（`xRockerBtnDataStruct`、`xKeySwitchUnion`） | 已核实 |
| CRC 打包格式（Go2 `LowCmd` 812 字节、G1 `LowCmd` 1004 字节） | py `unitree_sdk2py/utils/crc.py` | 已核实 |
| Go2 `LowCmd`：`head = [0xFE, 0xEF]`、`level_flag = 0xFF`、`PosStopF = 2.146e9`、`VelStopF = 16000` | py `go2_stand_example.py`、`unitree_legged_const.py` | 已核实 |
| G1 `LowCmd`：`mode_machine` 从 `LowState` 抄回；`mode_pr` 0 = PR、1 = AB；`motor_cmd[i].mode` 1 = 使能 | py `g1_low_level_example.py` | 已核实 |
| `rt/arm_sdk`：`motor_cmd[29].q` 为权重（`kNotUsedJoint`） | py `g1_arm7_sdk_dds_example.py` | 已核实（权重的混合方式未核实） |
| 关节限位 | urdf `go2_description.urdf`、`g1_29dof_rev_1_0.urdf` 的 `<limit>` | 已核实（自动测试） |
| 四元数分量顺序 (w, x, y, z) | 源码无说明 | **未核实**；02 节用数据自检（合成数据按此顺序生成，所以真机上需重做该自检） |
| `foot_force` 的单位与顺序 | ros2 README 只给了字段名 | **未核实**（课程按腿序 FR/FL/RR/RL 处理，并始终称其为"原始读数"） |
| `SportModeState.mode`、`gait_type` 的取值含义 | ros2 README 指向在线文档 | **未核实**（课程未使用） |

## 3. Python SDK 接口

| 接口 | 签名 / 值 | 出处 |
|---|---|---|
| `ChannelFactoryInitialize(id=0, networkInterface=None)` | 进程内只初始化一次 | py `unitree_sdk2py/core/channel.py` |
| `ChannelSubscriber(name, type)`；`.Init(handler=None, queueLen=0)`；`.Read(timeout=None)`；`.Close()` | | 同上 |
| `ChannelPublisher(name, type)`；`.Init()`；`.Write(sample, timeout=None)`；`.Close()` | | 同上 |
| `Client.SetTimeout(sec)`；内部 `_Call` / `_CallNoReply` | | py `unitree_sdk2py/rpc/client.py`、`client_base.py` |
| RPC 错误码 3102 / 3104 / 3203 等 | | py `unitree_sdk2py/rpc/internal.py` |
| `SportClient`：服务名 `sport`；`StandUp` 1004、`StandDown` 1005、`Move` 1008（`{"x","y","z"}`，不要应答）、`StopMove` 1003、`Damp` 1001、`BalanceStand` 1002、`RecoveryStand` 1006、`Euler` 1007 | | py `unitree_sdk2py/go2/sport/sport_api.py`、`sport_client.py` |
| `LocoClient`：服务名 `sport`；`GetFsmId` 7001、`SetFsmId` 7101、`SetVelocity` 7105（`{"velocity":[vx,vy,ω],"duration"}`）、`SetTaskId` 7106；`Damp`=FSM 1、`ZeroTorque`=0、`Sit`=3、`Start`=500、`Lie2StandUp`=702、`Squat2StandUp`/`StandUp2Squat`=706；`Move(..., continous_move)` 时长 1 s / 864000 s | | py `unitree_sdk2py/g1/loco/g1_loco_api.py`、`g1_loco_client.py` |
| `G1ArmActionClient`：服务名 `arm`；`ExecuteAction` 7106、`GetActionList` 7107；`action_map`（`release arm` = 99 等） | | py `unitree_sdk2py/g1/arm/` |
| `MotionSwitcherClient`：服务名 `motion_switcher`；`CheckMode` 1001、`SelectMode` 1002、`ReleaseMode` 1003 | | py `unitree_sdk2py/comm/motion_switcher/` |
| `CRC().Crc(msg)` | Linux 下调用随包附带的 `crc_amd64.so` / `crc_aarch64.so` | py `unitree_sdk2py/utils/crc.py` |
| `RecurrentThread(interval, target, name)`；`.Start()`；`.Wait(timeout)` | 基于 `timerfd`，仅 Linux | py `unitree_sdk2py/utils/thread.py` |

C++ 对照表（05 节）中的写法取自 cpp `example/go2/go2_low_level.cpp`、`go2_sport_client.cpp`；课程不安装、不编译 C++ SDK，
仓库里的 C++ 示例（`src/unitree_lab_cpp/`）用的是 rclcpp 和 `unitree_ros2` 的消息。

## 4. 未核实清单（汇总）

以下内容在三个仓库的源码中找不到依据。课程里要么没有使用，要么明确标注并做成了可改的参数：

1. G1 的里程计 Topic 名（课程数据用 `rt/odommodestate`）。
2. G1 的激光雷达 Topic、型号与外参；Go2 `utlidar_lidar` 相对机身的外参（课程用的是合成值，只有平移）。
3. `rt/lowstate` 等 Topic 的实际频率（文档称约 500 Hz；合成数据约 200 Hz）。
4. Go2 `Move` 指令的失效时间（合成数据的模型设为 1 s）。
5. G1 在未进入主运控（FSM 500）时对速度指令的反应；`SetFsmId(706)` 在已站立时的行为（由"两个方法同 id"推断为切换）。
6. `rt/arm_sdk` 权重的混合方式（合成数据按线性混合）。
7. 机器人对 CRC 错误、`mode_machine` 不匹配的 `LowCmd` 的处理（合成数据的模型是丢弃）。
8. `MotionSwitcherClient.CheckMode()` 返回的模式名。
9. `SetStandHeight` 参数的单位。
10. 四元数分量顺序、`foot_force` 的单位与顺序。
11. 机器人网口的 IP 地址段（`scripts/connect.py` 按 `192.168.123.x` 提示，来自对官方文档的记忆）。
12. 遥控器进入阻尼 / 急停的组合键（课程不给出，要求按官方手册）。

以上各项的真机验证步骤见 `real_robot_checklist.md`。
