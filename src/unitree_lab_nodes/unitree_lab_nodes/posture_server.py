"""posture_server: 把 "站起来 / 趴下" 包装成一个 ROS 2 Action。

    Action  /posture  (unitree_lab_interfaces/action/Posture)
      目标: posture = "stand" | "lie" (Go2)   "stand" | "squat" (G1)
      反馈: 当前机身高度、进度
      结果: 是否成功、最终高度、用时

为什么是 Action 而不是 Service: 站起来要一两秒，过程中调用方想看进度、也可能想取消。

    ros2 run unitree_lab_nodes posture_server --ros-args -p robot:=go2 -p enable_motion:=true
"""
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from common import robots as R
from common.ros_rpc import RosRpcClient
from unitree_lab_interfaces.action import Posture

# 目标姿态 -> 期望机身高度 [m] (合成环境的值；真机请按实测调整参数 heights)
DEFAULT_HEIGHTS = {"go2": {"stand": 0.32, "lie": 0.07}, "g1": {"stand": 0.79, "squat": 0.45}}
GO2_API = {"stand": 1004, "lie": 1005}      # SPORT_API_ID_STANDUP / STANDDOWN
G1_API_SET_FSM_ID = 7101                    # ROBOT_API_ID_LOCO_SET_FSM_ID


class PostureServer(Node):
    def __init__(self):
        super().__init__("posture_server")
        self.robot = self.declare_parameter("robot", "go2").value
        self.enable_motion = self.declare_parameter("enable_motion", False).value
        self.tolerance = self.declare_parameter("tolerance", 0.03).value
        default_odom = "/sportmodestate" if self.robot == "go2" else "/odommodestate"
        odom_topic = self.declare_parameter("odom_topic", default_odom).value
        from unitree_go.msg import SportModeState

        self.heights = DEFAULT_HEIGHTS[self.robot]
        self.height = None
        # 关键: 执行动作的回调会阻塞好几秒。高度订阅、RPC 应答必须能同时被处理，
        # 所以放进可重入回调组，并用多线程执行器 (03 节讲过为什么)。
        group = ReentrantCallbackGroup()
        self.create_subscription(SportModeState, odom_topic, self.on_sport, 10, callback_group=group)
        self.rpc = RosRpcClient(self, "sport")
        self.server = ActionServer(self, Posture, "posture", execute_callback=self.execute,
                                   goal_callback=self.on_goal, cancel_callback=lambda g: CancelResponse.ACCEPT,
                                   callback_group=group)
        self.get_logger().info(f"posture server up: robot={self.robot} postures={list(self.heights)} "
                               f"enable_motion={self.enable_motion}")

    def on_sport(self, msg):
        self.height = float(msg.position[2])

    def on_goal(self, goal):
        if goal.posture not in self.heights:
            self.get_logger().warn(f"reject unknown posture '{goal.posture}'")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def send_command(self, posture: str):
        if self.robot == "go2":
            return self.rpc.call(GO2_API[posture], {})
        # G1: 706 这个 FSM id 在 "蹲" 和 "站" 之间切换，所以要先看当前是不是已经在目标姿态
        return self.rpc.call(G1_API_SET_FSM_ID, {"data": R.G1_FSM["squat2standup"]})

    def execute(self, goal_handle):
        goal = goal_handle.request
        target = self.heights[goal.posture]
        result = Posture.Result()
        t0 = time.monotonic()
        while self.height is None and time.monotonic() - t0 < 2.0:
            time.sleep(0.05)
        if self.height is None:
            goal_handle.abort()
            result.message = "no height feedback (is the robot publishing odometry?)"
            return result
        start = self.height

        if abs(start - target) <= self.tolerance:
            result.message = "already there"
        elif not self.enable_motion:
            # 安全默认: 不下发指令，只观察机身高度会不会到位 (离线回放数据时就是这种情况)
            self.get_logger().warn(f"[not sent] posture={goal.posture}: enable_motion=false, observing only")
            result.message = "[not sent] observed only"
        else:
            code, _ = self.send_command(goal.posture)
            if code != 0:
                goal_handle.abort()
                result.message = f"robot refused: code {code}"
                result.final_height = self.height
                return result
            result.message = "done"

        fb = Posture.Feedback()
        while time.monotonic() - t0 < goal.timeout:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.message, result.final_height = "canceled", self.height
                return result
            fb.height = self.height
            span = target - start
            fb.progress = 1.0 if abs(span) < 1e-6 else float(min(1.0, max(0.0, (self.height - start) / span)))
            goal_handle.publish_feedback(fb)
            if abs(self.height - target) <= self.tolerance:
                goal_handle.succeed()
                result.success, result.final_height, result.elapsed = True, self.height, time.monotonic() - t0
                result.message = result.message or "done"
                return result
            time.sleep(0.1)
        goal_handle.abort()
        result.message, result.final_height, result.elapsed = "timeout", self.height, time.monotonic() - t0
        return result


def main():
    rclpy.init()
    node = PostureServer()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
