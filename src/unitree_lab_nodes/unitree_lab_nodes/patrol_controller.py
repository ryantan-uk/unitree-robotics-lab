"""patrol_controller: 综合实验 (18 节) 的参考解 —— 一条完整的 "感知 -> 决策 -> 控制" 链路。

    感知:  /scan           前方最近障碍物距离
           /robot_status   机器人是否健康 (倾角、温度、数据是否新鲜)
    决策:  状态机 FORWARD / TURN / HALT        (纯函数 decide()，可以脱离 ROS 单独测试)
    控制:  /cmd_vel        交给 robot_bridge 翻译成 Go2 或 G1 的运动请求

    ros2 run unitree_lab_nodes patrol_controller
"""
import math

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def front_distance(ranges, angle_min, angle_increment, half_width=math.radians(25)) -> float:
    """前方 ±half_width 扇区内的最近距离。"""
    r = np.asarray(ranges, dtype=float)
    ang = angle_min + angle_increment * np.arange(len(r))
    sector = r[np.abs(ang) <= half_width]
    sector = sector[np.isfinite(sector)]
    return float(sector.min()) if len(sector) else math.inf


def decide(state: str, front: float, healthy: bool, stop_dist=0.9, clear_dist=1.4,
           v=0.35, w=0.6) -> tuple[str, tuple[float, float]]:
    """决策: (当前状态, 前方距离, 是否健康) -> (新状态, (线速度, 角速度))

    迟滞: 小于 stop_dist 才开始转，大于 clear_dist 才恢复前进，避免在阈值附近抖动。
    """
    if not healthy:
        return "HALT", (0.0, 0.0)
    if state == "HALT":
        state = "FORWARD"
    if state == "FORWARD" and front < stop_dist:
        state = "TURN"
    elif state == "TURN" and front > clear_dist:
        state = "FORWARD"
    return state, ((v, 0.0) if state == "FORWARD" else (0.0, w))


class PatrolController(Node):
    def __init__(self):
        super().__init__("patrol_controller")
        self.stop_dist = self.declare_parameter("stop_dist", 0.9).value
        self.clear_dist = self.declare_parameter("clear_dist", 1.4).value
        self.v = self.declare_parameter("speed", 0.35).value
        self.w = self.declare_parameter("turn_speed", 0.6).value
        self.state = "HALT"
        self.front = math.inf
        self.healthy = False
        self.scan_time = None
        self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)
        self.create_subscription(DiagnosticArray, "/robot_status", self.on_status, 10)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_timer(0.1, self.tick)               # 10 Hz 控制循环

    def on_scan(self, msg):
        self.front = front_distance(msg.ranges, msg.angle_min, msg.angle_increment)
        self.scan_time = self.get_clock().now()

    def on_status(self, msg):
        self.healthy = bool(msg.status) and msg.status[0].level in (DiagnosticStatus.OK, DiagnosticStatus.WARN)

    def tick(self):
        fresh = self.scan_time is not None and (self.get_clock().now() - self.scan_time).nanoseconds < 1e9
        new_state, (v, w) = decide(self.state, self.front, self.healthy and fresh,
                                   self.stop_dist, self.clear_dist, self.v, self.w)
        if new_state != self.state:
            self.get_logger().info(f"{self.state} -> {new_state} (front={self.front:.2f} m)")
            self.state = new_state
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = v, w
        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = PatrolController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())           # 退出前发一条零速
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
