#!/usr/bin/env python3
"""Accumulate depth points in map coordinates for RViz."""

import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener

from avp_bridge import quat_wxyz_to_R


class AvpScan(Node):
    def __init__(self):
        super().__init__('avp_scan')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cells = {}
        self.last_tf_warn = 0.0
        self.scan_pub = self.create_publisher(PointCloud2, '/avp/scan', 10)
        self.create_subscription(PointCloud2, '/rgbd_camera/points', self.cloud_cb, qos_profile_sensor_data)
        self.create_timer(2.0, self.publish_scan)

    def cloud_cb(self, msg):
        try:
            tf = self.tf_buffer.lookup_transform('map', msg.header.frame_id, Time())
        except TransformException as exc:
            now = time.monotonic()
            if now - self.last_tf_warn > 5.0:
                self.last_tf_warn = now
                self.get_logger().warn(f'Waiting for camera TF: {exc}')
            return
        xyz = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)[::24]
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        xyz = xyz[(np.linalg.norm(xyz, axis=1) > 0.3) & (np.linalg.norm(xyz, axis=1) < 6.0)]
        if len(xyz) == 0:
            return
        q, p = tf.transform.rotation, tf.transform.translation
        R = quat_wxyz_to_R((q.w, q.x, q.y, q.z))
        if R is None:
            return
        world = np.asarray(xyz, dtype=np.float32) @ R.T + np.array([p.x, p.y, p.z])
        keys = np.floor(world / 0.10).astype(np.int32)
        for key, point in zip(map(tuple, keys), world):
            self.cells[key] = tuple(float(value) for value in point)
        while len(self.cells) > 100_000:
            self.cells.pop(next(iter(self.cells)))

    def publish_scan(self):
        if not self.cells or not self.scan_pub.get_subscription_count():
            return
        header = Header(stamp=self.get_clock().now().to_msg(), frame_id='map')
        self.scan_pub.publish(point_cloud2.create_cloud_xyz32(header, list(self.cells.values())))


def main():
    rclpy.init()
    node = AvpScan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
