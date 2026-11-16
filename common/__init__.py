"""unitree-robotics-lab 公共库。

    connect   连接真机；没有真机时回放 Rosbag
    robots    topic 名、关节顺序、限位、API 号 (带出处)
    parsing   消息 -> NumPy；遥控器字节解析；四元数/欧拉角
    joints    关节插值、限位、限速
    safety    运动闸门 (默认只打印不下发)
    rosutil   Notebook 里用 ROS 2 的小工具            (需要 rclpy)
    ros_rpc   用纯 ROS 2 调 Unitree 的 RPC 服务        (需要 rclpy)
    bagio     读 Rosbag / 样本                        (需要 rclpy)
    plotting  画姿态、关节、足端力、路径                (需要 matplotlib)
"""
from .connect import Background, Lab, connect, play_bag, DATA, REPO  # noqa: F401
from .safety import MotionGate  # noqa: F401
