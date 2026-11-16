"""cloud_to_scan: 三维点云 -> 二维激光 (给 slam_toolbox 和 Nav2 用)。

    /utlidar/cloud (sensor_msgs/PointCloud2) -> /scan (sensor_msgs/LaserScan, 坐标系 base_link)

做法: 用 TF 把点变到 target_frame，取高度在 [min_height, max_height] 的一层，
按方位角分箱，每箱取最近的点。
"""
import math

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2
from tf2_ros import Buffer, TransformException, TransformListener


def cloud_to_xyz(msg: PointCloud2) -> np.ndarray:
    """PointCloud2 -> (N, 3) float32。按 fields 里的偏移取 x/y/z，不假设布局。"""
    off = {f.name: f.offset for f in msg.fields}
    dtype = np.dtype({"names": ["x", "y", "z"], "formats": ["<f4"] * 3,
                      "offsets": [off["x"], off["y"], off["z"]], "itemsize": msg.point_step})
    arr = np.frombuffer(bytes(msg.data), dtype=dtype, count=msg.width * msg.height)
    xyz = np.stack([arr["x"], arr["y"], arr["z"]], axis=1)
    return xyz[np.isfinite(xyz).all(axis=1)]


def transform_matrix(t) -> np.ndarray:
    """geometry_msgs/Transform -> 4x4"""
    q, p = t.rotation, t.translation
    x, y, z, w = q.x, q.y, q.z, q.w
    M = np.eye(4)
    M[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    M[:3, 3] = [p.x, p.y, p.z]
    return M


def xyz_to_ranges(xyz, n_bins=360, min_height=-0.15, max_height=0.5, range_min=0.2, range_max=10.0):
    """(N,3) 点 -> n_bins 个方位角箱里的最近距离 (没有点的箱为 inf)。"""
    ranges = np.full(n_bins, np.inf, dtype=np.float32)
    if len(xyz) == 0:
        return ranges
    keep = (xyz[:, 2] >= min_height) & (xyz[:, 2] <= max_height)
    x, y = xyz[keep, 0], xyz[keep, 1]
    r = np.hypot(x, y)
    ok = (r >= range_min) & (r <= range_max)
    idx = ((np.arctan2(y[ok], x[ok]) + math.pi) / (2 * math.pi) * n_bins).astype(int) % n_bins
    np.minimum.at(ranges, idx, r[ok].astype(np.float32))
    return ranges


class CloudToScan(Node):
    def __init__(self):
        super().__init__("cloud_to_scan")
        self.target = self.declare_parameter("target_frame", "base_link").value
        self.n_bins = self.declare_parameter("n_bins", 360).value
        self.min_h = self.declare_parameter("min_height", -0.15).value
        self.max_h = self.declare_parameter("max_height", 0.5).value
        self.rmin = self.declare_parameter("range_min", 0.2).value
        self.rmax = self.declare_parameter("range_max", 10.0).value
        cloud_topic = self.declare_parameter("cloud_topic", "/utlidar/cloud").value
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.pub = self.create_publisher(LaserScan, "/scan", qos_profile_sensor_data)
        self.create_subscription(PointCloud2, cloud_topic, self.on_cloud, qos_profile_sensor_data)
        self.warned = False

    def on_cloud(self, msg: PointCloud2):
        try:
            tf = self.buf.lookup_transform(self.target, msg.header.frame_id, Time(), Duration(seconds=0.05))
        except TransformException as e:
            if not self.warned:
                self.get_logger().warn(f"no TF {self.target} <- {msg.header.frame_id} yet: {e}")
                self.warned = True
            return
        xyz = cloud_to_xyz(msg)
        M = transform_matrix(tf.transform)
        xyz = xyz @ M[:3, :3].T + M[:3, 3]
        scan = LaserScan()
        scan.header.stamp = msg.header.stamp
        scan.header.frame_id = self.target
        scan.angle_min = -math.pi
        scan.angle_increment = 2 * math.pi / self.n_bins
        scan.angle_max = math.pi - scan.angle_increment
        scan.range_min, scan.range_max = float(self.rmin), float(self.rmax)
        scan.ranges = xyz_to_ranges(xyz, self.n_bins, self.min_h, self.max_h, self.rmin, self.rmax).tolist()
        self.pub.publish(scan)


def main():
    rclpy.init()
    node = CloudToScan()
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
