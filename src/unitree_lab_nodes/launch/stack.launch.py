"""课程节点一键启动。

    ros2 launch unitree_lab_nodes stack.launch.py robot:=go2
    ros2 launch unitree_lab_nodes stack.launch.py robot:=go2 urdf:=/path/to/go2.urdf slam:=true nav:=true

参数:
    robot          go2 | g1
    enable_motion  false(默认) | true     —— true 才会把 /cmd_vel 下发给机器人
    use_sim_time   false(默认) | true     —— 回放 rosbag (ros2 bag play --clock) 时设 true
    lowstate_topic /lowstate(默认) | /lf/lowstate —— 数据源只有低频状态时用后者
    urdf           URDF 文件路径; 给了就启动 robot_state_publisher
    slam           true 时启动 slam_toolbox (参数文件 slam_params)
    nav            true 时启动 Nav2 的规划器 planner_server (参数文件 nav_params)
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

REPO = os.environ.get("UNITREE_LAB_REPO", "")


def setup(context):
    cfg = lambda name: LaunchConfiguration(name).perform(context)          # noqa: E731
    robot = cfg("robot")
    sim = cfg("use_sim_time").lower() == "true"
    common = {"use_sim_time": sim}
    low = {"lowstate_topic": cfg("lowstate_topic")}
    decimate = 1 if "lf" in cfg("lowstate_topic") else 4      # 低频状态本来就慢，不再抽稀
    nodes = [
        Node(package="unitree_lab_nodes", executable="robot_bridge", output="screen",
             parameters=[common, low, {"robot": robot, "decimate": decimate,
                                       "enable_motion": cfg("enable_motion").lower() == "true"}]),
        Node(package="unitree_lab_nodes", executable="state_aggregator", output="screen",
             parameters=[common, low, {"robot": robot}]),
        Node(package="unitree_lab_nodes", executable="cloud_to_scan", output="screen", parameters=[common]),
    ]
    urdf = cfg("urdf")
    if urdf:
        with open(urdf) as f:
            description = f.read()
        root = "base" if robot == "go2" else "pelvis"        # URDF 的根连杆名 (go2_description / g1_description)
        nodes += [
            Node(package="robot_state_publisher", executable="robot_state_publisher", output="screen",
                 parameters=[common, {"robot_description": description}]),
            # 桥发布的机身坐标系叫 base_link；URDF 的根连杆叫 base / pelvis。用一个恒等的静态 TF 把两棵树接起来
            Node(package="tf2_ros", executable="static_transform_publisher", output="screen", parameters=[common],
                 arguments=["--frame-id", "base_link", "--child-frame-id", root]),
        ]
    if cfg("slam").lower() == "true":
        nodes.append(Node(package="slam_toolbox", executable="async_slam_toolbox_node", name="slam_toolbox",
                          output="screen", parameters=[cfg("slam_params"), common]))
    if cfg("nav").lower() == "true":
        nodes += [
            Node(package="nav2_planner", executable="planner_server", name="planner_server", output="screen",
                 parameters=[cfg("nav_params"), common]),
            Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_planning",
                 output="screen", parameters=[common, {"autostart": True, "node_names": ["planner_server"]}]),
        ]
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot", default_value="go2"),
        DeclareLaunchArgument("enable_motion", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("lowstate_topic", default_value="/lowstate"),
        DeclareLaunchArgument("urdf", default_value=""),
        DeclareLaunchArgument("slam", default_value="false"),
        DeclareLaunchArgument("nav", default_value="false"),
        DeclareLaunchArgument("slam_params", default_value=os.path.join(REPO, "config", "slam_toolbox.yaml")),
        DeclareLaunchArgument("nav_params", default_value=os.path.join(REPO, "config", "nav2_planner.yaml")),
        OpaqueFunction(function=setup),
    ])
