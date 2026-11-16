"""Go2 / G1 的常量表: topic 名、关节顺序、API 号。

每一项后面都标了出处 (docs/source_audit.md 有完整对照)。标 "未核实" 的条目
没有在三个官方仓库里找到，上真机前必须自己确认。
"""
from __future__ import annotations

# --------------------------------------------------------------------------- topic
# DDS 里的名字带 "rt/" 前缀；ROS 2 里看到的名字是去掉 "rt" 后的 "/xxx"。
#   SDK2 (DDS):  rt/lowstate      <->   ROS 2:  /lowstate
DDS_PREFIX = "rt/"


def ros_name(dds_topic: str) -> str:
    """'rt/lowstate' -> '/lowstate'"""
    return "/" + dds_topic[len(DDS_PREFIX):] if dds_topic.startswith(DDS_PREFIX) else dds_topic


def dds_name(ros_topic: str) -> str:
    """'/lowstate' -> 'rt/lowstate'"""
    return DDS_PREFIX + ros_topic.lstrip("/")


TOPICS = {
    # 名字                      (DDS topic,              消息类型,                       出处)
    "lowstate":        ("rt/lowstate",          "LowState",          "sdk2py example/go2/low_level/go2_stand_example.py; example/g1/low_level/g1_low_level_example.py"),
    "lowstate_lf":     ("rt/lf/lowstate",       "LowState",          "sdk2py test/lowlevel; unitree_ros2 README (lf = low frequency)"),
    "lowcmd":          ("rt/lowcmd",            "LowCmd",            "同 lowstate"),
    "sportmodestate":  ("rt/sportmodestate",    "SportModeState",    "unitree_sdk2 example; unitree_ros2 README"),
    "sportmodestate_lf": ("rt/lf/sportmodestate", "SportModeState",  "sdk2py example; unitree_ros2 example"),
    "wirelesscontroller": ("rt/wirelesscontroller", "WirelessController", "unitree_sdk2 example; unitree_ros2 README"),
    "utlidar_cloud":   ("rt/utlidar/cloud",     "sensor_msgs/PointCloud2", "unitree_ros2 README (frame_id: utlidar_lidar)"),
    "arm_sdk":         ("rt/arm_sdk",           "unitree_hg/LowCmd", "sdk2py example/g1/high_level/g1_arm7_sdk_dds_example.py"),
    "api_request":     ("rt/api/{service}/request",  "unitree_api/Request",  "sdk2py core/channel_name.py"),
    "api_response":    ("rt/api/{service}/response", "unitree_api/Response", "sdk2py core/channel_name.py"),
    # 未核实: 三个仓库的源码里没有出现，依据 Unitree 官方文档的记忆
    "g1_odom":         ("rt/odommodestate",     "SportModeState",    "未核实"),
    "g1_odom_lf":      ("rt/lf/odommodestate",  "SportModeState",    "未核实"),
}

# --------------------------------------------------------------------------- Go2
# 出处: sdk2py example/go2/low_level/unitree_legged_const.py (LegID)
GO2_NUM_MOTOR = 12            # LowState.motor_state 长度是 20，前 12 个有效
GO2_LEGS = ["FR", "FL", "RR", "RL"]                       # 前右 前左 后右 后左
GO2_JOINTS = [f"{leg}_{part}" for leg in GO2_LEGS for part in ("hip", "thigh", "calf")]
GO2_LEG_ID = {f"{leg}_{i}": 3 * li + i for li, leg in enumerate(GO2_LEGS) for i in range(3)}
# URDF 关节名 (unitree_ros/robots/go2_description)，顺序与电机序一致
GO2_URDF_JOINTS = [f"{j}_joint" for j in GO2_JOINTS]

# 官方 go2_stand_example.py 用到的两组姿态 (rad)，按 hip, thigh, calf
GO2_POSE_LIE = [0.0, 1.36, -2.65] * 4      # _targetPos_1 的主体: 趴下
GO2_POSE_STAND = [0.0, 0.67, -1.3] * 4     # _targetPos_2: 站立

# 关节软限位 (rad)。出处: go2_description/urdf/go2_description.urdf 的 <limit>
# (安装后可用 tests/test_urdf_limits.py 对照)。
GO2_LIMITS = {
    "hip":   (-1.0472, 1.0472),
    "thigh_front": (-1.5708, 3.4907),
    "thigh_rear":  (-0.5236, 4.5379),
    "calf":  (-2.7227, -0.83776),
}


def go2_limits():
    """返回 (lower[12], upper[12])"""
    lo, hi = [], []
    for leg in GO2_LEGS:
        for part in ("hip", "thigh", "calf"):
            key = part if part != "thigh" else ("thigh_front" if leg[0] == "F" else "thigh_rear")
            lo.append(GO2_LIMITS[key][0]); hi.append(GO2_LIMITS[key][1])
    return lo, hi


# LowCmd 约定。出处: unitree_legged_const.py / go2_stand_example.py
GO2_LOWCMD_HEAD = (0xFE, 0xEF)
GO2_LEVEL_LOW = 0xFF
POS_STOP_F = 2.146e9       # 位置环"不控制"哨兵值
VEL_STOP_F = 16000.0       # 速度环"不控制"哨兵值

# --------------------------------------------------------------------------- G1
# 出处: sdk2py example/g1/low_level/g1_low_level_example.py (G1JointIndex)
G1_NUM_MOTOR = 29             # unitree_hg LowState.motor_state 长度是 35，29 自由度机型用前 29 个
G1_JOINTS = [
    "left_hip_pitch", "left_hip_roll", "left_hip_yaw", "left_knee", "left_ankle_pitch", "left_ankle_roll",
    "right_hip_pitch", "right_hip_roll", "right_hip_yaw", "right_knee", "right_ankle_pitch", "right_ankle_roll",
    "waist_yaw", "waist_roll", "waist_pitch",
    "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw", "left_elbow",
    "left_wrist_roll", "left_wrist_pitch", "left_wrist_yaw",
    "right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw", "right_elbow",
    "right_wrist_roll", "right_wrist_pitch", "right_wrist_yaw",
]
G1_JOINT_ID = {name: i for i, name in enumerate(G1_JOINTS)}
G1_URDF_JOINTS = [f"{j}_joint" for j in G1_JOINTS]
G1_LEG_IDS = list(range(0, 12))
G1_WAIST_IDS = [12, 13, 14]
G1_ARM_IDS = list(range(15, 29))
G1_ARM_SDK_WEIGHT_ID = 29     # rt/arm_sdk: motor_cmd[29].q 是权重 (kNotUsedJoint)，1=接管上肢 0=交还
# 23 自由度机型上无效的关节 (源码注释 "INVALID for g1 23dof")
G1_INVALID_23DOF = [13, 14, 20, 21, 27, 28]

# 官方低层示例使用的增益 (g1_low_level_example.py)
G1_KP = [60, 60, 60, 100, 40, 40] * 2 + [60, 40, 40] + [40] * 14
G1_KD = [1, 1, 1, 2, 1, 1] * 2 + [1, 1, 1] + [1] * 14

# 关节限位 (rad)。出处: g1_description/g1_29dof*.urdf 的 <limit>，安装后由测试对照。
G1_LIMITS = {
    "left_hip_pitch": (-2.5307, 2.8798), "left_hip_roll": (-0.5236, 2.9671), "left_hip_yaw": (-2.7576, 2.7576),
    "left_knee": (-0.087267, 2.8798), "left_ankle_pitch": (-0.87267, 0.5236), "left_ankle_roll": (-0.2618, 0.2618),
    "right_hip_pitch": (-2.5307, 2.8798), "right_hip_roll": (-2.9671, 0.5236), "right_hip_yaw": (-2.7576, 2.7576),
    "right_knee": (-0.087267, 2.8798), "right_ankle_pitch": (-0.87267, 0.5236), "right_ankle_roll": (-0.2618, 0.2618),
    "waist_yaw": (-2.618, 2.618), "waist_roll": (-0.52, 0.52), "waist_pitch": (-0.52, 0.52),
    "left_shoulder_pitch": (-3.0892, 2.6704), "left_shoulder_roll": (-1.5882, 2.2515),
    "left_shoulder_yaw": (-2.618, 2.618), "left_elbow": (-1.0472, 2.0944),
    "left_wrist_roll": (-1.9722, 1.9722), "left_wrist_pitch": (-1.6144, 1.6144), "left_wrist_yaw": (-1.6144, 1.6144),
    "right_shoulder_pitch": (-3.0892, 2.6704), "right_shoulder_roll": (-2.2515, 1.5882),
    "right_shoulder_yaw": (-2.618, 2.618), "right_elbow": (-1.0472, 2.0944),
    "right_wrist_roll": (-1.9722, 1.9722), "right_wrist_pitch": (-1.6144, 1.6144), "right_wrist_yaw": (-1.6144, 1.6144),
}


def g1_limits():
    """返回 (lower[29], upper[29])"""
    return [G1_LIMITS[j][0] for j in G1_JOINTS], [G1_LIMITS[j][1] for j in G1_JOINTS]


# G1 运动状态机 FSM id。出处: sdk2py g1/loco/g1_loco_client.py 各便捷方法里写死的数字
G1_FSM = {
    "zero_torque": 0,   # ZeroTorque()
    "damp": 1,          # Damp()
    "sit": 3,           # Sit()
    "start": 500,       # Start(): 进入主运控，之后才能 Move
    "lie2standup": 702,     # Lie2StandUp()
    "squat2standup": 706,   # Squat2StandUp() / StandUp2Squat() 用同一个 id
}

# --------------------------------------------------------------------------- 通用


def joints_of(robot: str):
    return {"go2": GO2_JOINTS, "g1": G1_JOINTS}[robot]


def num_motor(robot: str) -> int:
    return {"go2": GO2_NUM_MOTOR, "g1": G1_NUM_MOTOR}[robot]


def limits_of(robot: str):
    return go2_limits() if robot == "go2" else g1_limits()


def msg_pkg(robot: str) -> str:
    """Go2 用 unitree_go，G1 用 unitree_hg (hg = humanoid, G1/H1-2)"""
    return {"go2": "unitree_go", "g1": "unitree_hg"}[robot]
