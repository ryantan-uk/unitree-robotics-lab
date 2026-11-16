"""robot_bridge: 把 Unitree 的消息翻译成 ROS 2 标准消息，把 /cmd_vel 翻译成运动请求。

    Unitree 原生                          ROS 2 标准
    /lowstate (unitree_go|hg/LowState) -> /imu/data (sensor_msgs/Imu), /joint_states (sensor_msgs/JointState)
    /sportmodestate (SportModeState)   -> /odom (nav_msgs/Odometry) + TF odom->base_link
    /api/sport/request (Request)       <- /cmd_vel (geometry_msgs/Twist)

    ros2 run unitree_lab_nodes robot_bridge --ros-args -p robot:=go2 -p enable_motion:=true

安全: enable_motion 默认 false —— /cmd_vel 只打印不下发。
"""
import math

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from common import robots as R
from common.ros_rpc import RosRpcClient

GO2_API_MOVE = 1008          # unitree_sdk2py/go2/sport/sport_api.py: SPORT_API_ID_MOVE
G1_API_SET_VELOCITY = 7105   # unitree_sdk2py/g1/loco/g1_loco_api.py: ROBOT_API_ID_LOCO_SET_VELOCITY


def yaw_to_quat(yaw):
    return 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)


class RobotBridge(Node):
    def __init__(self):
        super().__init__("robot_bridge")
        self.robot = self.declare_parameter("robot", "go2").value
        self.enable_motion = self.declare_parameter("enable_motion", False).value
        self.decimate = self.declare_parameter("decimate", 4).value            # 每 N 条 lowstate 转发 1 条
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        # G1 的里程计 topic 未在官方源码中核实，所以做成参数
        default_odom = "/sportmodestate" if self.robot == "go2" else "/odommodestate"
        odom_topic = self.declare_parameter("odom_topic", default_odom).value
        # 雷达外参: 默认是课程合成样本数据所用的值。真机请改成实测/URDF 的值 (或设 publish_lidar_tf:=false)
        self.lidar_xyz = self.declare_parameter("lidar_xyz", [0.28, 0.0, 0.05]).value
        publish_lidar_tf = self.declare_parameter("publish_lidar_tf", True).value

        if self.robot == "go2":
            from unitree_go.msg import LowState
        else:
            from unitree_hg.msg import LowState
        from unitree_go.msg import SportModeState      # G1 的里程计也用这个类型

        self.n = R.num_motor(self.robot)
        self.joint_names = R.GO2_URDF_JOINTS if self.robot == "go2" else R.G1_URDF_JOINTS
        self.count = 0
        self.blocked = 0

        self.pub_imu = self.create_publisher(Imu, "/imu/data", 10)
        self.pub_js = self.create_publisher(JointState, "/joint_states", 10)
        self.pub_odom = self.create_publisher(Odometry, "/odom", 10)
        self.tf = TransformBroadcaster(self)
        self.rpc = RosRpcClient(self, "sport")
        # 全频率的 /lowstate 很占带宽；长时间运行或回放低频录制的数据时可改成 /lf/lowstate
        lowstate_topic = self.declare_parameter("lowstate_topic", "/lowstate").value
        self.create_subscription(LowState, lowstate_topic, self.on_lowstate, 10)
        self.create_subscription(SportModeState, odom_topic, self.on_sport, 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, 10)

        if publish_lidar_tf:
            self.static_tf = StaticTransformBroadcaster(self)
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id, t.child_frame_id = self.base_frame, "utlidar_lidar"
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, self.lidar_xyz)
            t.transform.rotation.w = 1.0
            self.static_tf.sendTransform(t)
        self.get_logger().info(f"bridge up: robot={self.robot} odom_topic={odom_topic} enable_motion={self.enable_motion}")

    # -- 状态: Unitree -> 标准消息 ------------------------------------------
    def on_lowstate(self, msg):
        self.count += 1
        if self.count % self.decimate:
            return
        now = self.get_clock().now().to_msg()
        imu = Imu()
        imu.header.stamp, imu.header.frame_id = now, "imu_link"
        w, x, y, z = msg.imu_state.quaternion            # Unitree: (w, x, y, z)   ROS: (x, y, z, w)
        imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w = float(x), float(y), float(z), float(w)
        g = msg.imu_state.gyroscope
        a = msg.imu_state.accelerometer
        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = float(g[0]), float(g[1]), float(g[2])
        imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = float(a[0]), float(a[1]), float(a[2])
        self.pub_imu.publish(imu)

        js = JointState()
        js.header.stamp = now
        js.name = self.joint_names
        js.position = [float(msg.motor_state[i].q) for i in range(self.n)]
        js.velocity = [float(msg.motor_state[i].dq) for i in range(self.n)]
        js.effort = [float(msg.motor_state[i].tau_est) for i in range(self.n)]
        self.pub_js.publish(js)

    def on_sport(self, msg):
        now = self.get_clock().now().to_msg()
        yaw = float(msg.imu_state.rpy[2])
        qx, qy, qz, qw = yaw_to_quat(yaw)
        x, y = float(msg.position[0]), float(msg.position[1])

        od = Odometry()
        od.header.stamp, od.header.frame_id, od.child_frame_id = now, self.odom_frame, self.base_frame
        od.pose.pose.position.x, od.pose.pose.position.y = x, y
        od.pose.pose.orientation.z, od.pose.pose.orientation.w = qz, qw
        # SportModeState.velocity 是世界系速度；Odometry.twist 约定在 child_frame (机体系)
        c, s = math.cos(yaw), math.sin(yaw)
        vx, vy = float(msg.velocity[0]), float(msg.velocity[1])
        od.twist.twist.linear.x = c * vx + s * vy
        od.twist.twist.linear.y = -s * vx + c * vy
        od.twist.twist.angular.z = float(msg.yaw_speed)
        self.pub_odom.publish(od)

        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = now, self.odom_frame, self.base_frame
        t.transform.translation.x, t.transform.translation.y = x, y
        t.transform.rotation.z, t.transform.rotation.w = qz, qw
        self.tf.sendTransform(t)

    # -- 控制: /cmd_vel -> 运动请求 -------------------------------------------
    def on_cmd_vel(self, msg: Twist):
        vx, vy, wz = float(msg.linear.x), float(msg.linear.y), float(msg.angular.z)
        if not self.enable_motion:
            self.blocked += 1
            if self.blocked == 1 or self.blocked % 100 == 0:
                self.get_logger().warn(f"[未下发] cmd_vel=({vx:.2f}, {vy:.2f}, {wz:.2f})  enable_motion=false (累计 {self.blocked} 条)")
            return
        if self.robot == "go2":
            self.rpc.send(GO2_API_MOVE, {"x": vx, "y": vy, "z": wz})
        else:
            self.rpc.send(G1_API_SET_VELOCITY, {"velocity": [vx, vy, wz], "duration": 1.0})


def main():
    rclpy.init()
    node = RobotBridge()
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
