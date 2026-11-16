"""state_aggregator: 把多路高频原始数据聚合成一条低频的 "机器人现在怎么样"。

    /lowstate + /sportmodestate  ->  /robot_status (diagnostic_msgs/DiagnosticArray, 2 Hz)
                                     /get_robot_summary (unitree_lab_interfaces/srv/GetRobotSummary)

    ros2 run unitree_lab_nodes state_aggregator --ros-args -p robot:=go2
"""
import math

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node

from common import robots as R
from common.parsing import lowstate_to_dict, tilt_deg
from unitree_lab_interfaces.srv import GetRobotSummary


class StateAggregator(Node):
    def __init__(self):
        super().__init__("state_aggregator")
        self.robot = self.declare_parameter("robot", "go2").value
        self.tilt_warn = self.declare_parameter("tilt_warn_deg", 20.0).value
        self.temp_warn = self.declare_parameter("temp_warn_c", 70.0).value
        self.stale_after = self.declare_parameter("stale_after_s", 1.0).value
        default_odom = "/sportmodestate" if self.robot == "go2" else "/odommodestate"
        odom_topic = self.declare_parameter("odom_topic", default_odom).value
        if self.robot == "go2":
            from unitree_go.msg import LowState
        else:
            from unitree_hg.msg import LowState
        from unitree_go.msg import SportModeState

        self.names = R.joints_of(self.robot)
        self.low = None
        self.low_time = None
        self.height = float("nan")
        self.speed = 0.0
        lowstate_topic = self.declare_parameter("lowstate_topic", "/lowstate").value
        self.create_subscription(LowState, lowstate_topic, self.on_low, 10)
        self.create_subscription(SportModeState, odom_topic, self.on_sport, 10)
        self.pub = self.create_publisher(DiagnosticArray, "/robot_status", 10)
        self.create_service(GetRobotSummary, "/get_robot_summary", self.on_summary)
        self.create_timer(0.5, self.publish_status)

    # 回调里只存数据，不做计算: 高频回调越轻越好 (03 节)
    def on_low(self, msg):
        self.low = msg
        self.low_time = self.get_clock().now()

    def on_sport(self, msg):
        self.height = float(msg.position[2])
        self.speed = math.hypot(msg.velocity[0], msg.velocity[1])

    def summary(self):
        if self.low is None:
            return None
        d = lowstate_to_dict(self.low, self.robot)
        hot = int(np.argmax(d["temp"]))
        age = (self.get_clock().now() - self.low_time).nanoseconds * 1e-9
        tilt = float(tilt_deg(d["rpy"][0], d["rpy"][1]))
        return {
            "tilt_deg": tilt, "upright": tilt < self.tilt_warn, "height": self.height, "speed": self.speed,
            "max_motor_temp": float(d["temp"][hot]), "hottest_motor": self.names[hot],
            "battery_soc": float(d.get("soc", -1)), "age": age,
        }

    def publish_status(self):
        s = self.summary()
        st = DiagnosticStatus(name=f"{self.robot}/summary", hardware_id=self.robot)
        if s is None:
            st.level, st.message = DiagnosticStatus.STALE, "no lowstate yet"
        elif s["age"] > self.stale_after:
            st.level, st.message = DiagnosticStatus.STALE, f"lowstate is {s['age']:.1f}s old"
        elif not s["upright"]:
            st.level, st.message = DiagnosticStatus.ERROR, f"tilt {s['tilt_deg']:.1f} deg"
        elif s["max_motor_temp"] > self.temp_warn:
            st.level, st.message = DiagnosticStatus.WARN, f"{s['hottest_motor']} at {s['max_motor_temp']:.0f} C"
        else:
            st.level, st.message = DiagnosticStatus.OK, "ok"
        if s is not None:
            st.values = [KeyValue(key=k, value=(f"{v:.3f}" if isinstance(v, float) else str(v))) for k, v in s.items()]
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [st]
        self.pub.publish(arr)

    def on_summary(self, request, response):
        s = self.summary()
        response.robot = self.robot
        if s is not None:
            response.upright = bool(s["upright"])
            response.tilt_deg = s["tilt_deg"]
            response.height = s["height"]
            response.max_motor_temp = s["max_motor_temp"]
            response.hottest_motor = s["hottest_motor"]
            response.battery_soc = s["battery_soc"]
        return response


def main():
    rclpy.init()
    node = StateAggregator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
