#!/usr/bin/env python3
"""Check AVP cloud, odometry, and camera TF without commanding the vehicle."""

import sys
import time
from itertools import islice

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener


def rotation_matrix(q):
    w, x, y, z = q.w, q.x, q.y, q.z
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
        [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)],
    ])


class LicsCheck(Node):
    def __init__(self):
        super().__init__('lics_check')
        self.declare_parameter('duration_s', 10.0)
        self.declare_parameter('roi_cx', 0.0)
        self.declare_parameter('roi_cy', 0.0)
        self.declare_parameter('roi_r', 0.0)
        self.declare_parameter('expect_floor', False)
        gp = lambda name: self.get_parameter(name).value
        self.duration = float(gp('duration_s'))
        self.roi = (float(gp('roi_cx')), float(gp('roi_cy')), float(gp('roi_r')))
        self.expect_floor = bool(gp('expect_floor'))
        if self.duration <= 0 or self.roi[2] < 0:
            raise ValueError('duration_s must be positive and roi_r nonnegative')

        self.cloud_count = 0
        self.odom_count = 0
        self.cloud = None
        self.odom_pos = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(PointCloud2, '/rgbd_camera/points', self.cloud_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/lics/odom', self.odom_cb, qos_profile_sensor_data)

    def cloud_cb(self, msg):
        self.cloud_count += 1
        self.cloud = msg

    def odom_cb(self, msg):
        self.odom_count += 1
        p = msg.pose.pose.position
        self.odom_pos = (p.x, p.y, p.z)

    def report(self, elapsed):
        log = self.get_logger()
        frame = self.cloud.header.frame_id if self.cloud is not None else ''
        points = self.cloud.width * self.cloud.height if self.cloud is not None else 0
        cloud_hz, odom_hz = self.cloud_count / elapsed, self.odom_count / elapsed
        log.info(f'[1] cloud {cloud_hz:.1f} Hz, points={points}, frame={frame or "<none>"}')
        log.info(f'[2] odom {odom_hz:.1f} Hz, position={self.odom_pos}')
        ok = cloud_hz > 3.0 and odom_hz > 10.0 and points > 0 and bool(frame)
        if not frame:
            log.error('[3] cloud frame missing; cannot check TF')
            log.info('RESULT: NOT READY')
            return False

        try:
            self.tf_buffer.lookup_transform('map', 'base_link', Time())
            base_cloud = self.tf_buffer.lookup_transform('base_link', frame, Time())
            map_cloud = self.tf_buffer.lookup_transform('map', frame, Time())
        except TransformException as exc:
            log.error(f'[3] TF map -> base_link -> {frame} missing: {exc}')
            log.info('RESULT: NOT READY')
            return False

        p = base_cloud.transform.translation
        R = rotation_matrix(base_cloud.transform.rotation)
        log.info(f'[3] TF OK; camera@base_link=({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) m')
        # OAK optical +Z is forward, +Y is down; base_link is ROS FLU.
        forward = R[:, 2]
        down = R[:, 1]
        offset = np.array([p.x, p.y, p.z])
        axes_ok = forward[0] > 0.98 and down[2] < -0.98
        mount_ok = np.linalg.norm(offset - [0.2, 0.0, 0.0]) < 0.03
        log.info(f'    optical +Z@base={forward.round(3)}, +Y@base={down.round(3)}; '
                 f'axes={axes_ok}, mount={mount_ok}')
        ok &= axes_ok and mount_ok

        samples = list(islice(point_cloud2.read_points(
            self.cloud, field_names=('x', 'y', 'z'), skip_nans=True), 0, 20000, 20))
        xyz = np.asarray([[float(v) for v in row] for row in samples], dtype=float)
        xyz = xyz[np.isfinite(xyz).all(axis=1)] if xyz.size else np.empty((0, 3))
        if len(xyz) == 0:
            log.error('[4] no finite XYZ points in sampled cloud')
            log.info('RESULT: NOT READY')
            return False
        t = map_cloud.transform.translation
        world = xyz @ rotation_matrix(map_cloud.transform.rotation).T + [t.x, t.y, t.z]
        z_min, z_max = world[:, 2].min(), world[:, 2].max()
        log.info(f'[4] cloud@map z=[{z_min:.2f}, {z_max:.2f}] m')
        if self.expect_floor:
            floor_ok = -0.5 <= z_min <= 1.0
            log.info(f'    floor near map z=0: {floor_ok}')
            ok &= floor_ok
        if self.roi[2] > 0:
            in_roi = np.hypot(world[:, 0] - self.roi[0], world[:, 1] - self.roi[1]) < self.roi[2]
            log.info(f'[5] sampled points inside ROI: {100.0 * in_roi.mean():.1f}%')
        log.info('RESULT: ' + ('READY' if ok else 'NOT READY'))
        return bool(ok)


def main():
    rclpy.init()
    node = LicsCheck()
    start = time.monotonic()
    try:
        while rclpy.ok() and time.monotonic() - start < node.duration:
            rclpy.spin_once(node, timeout_sec=0.2)
        return 0 if node.report(max(time.monotonic() - start, 1e-6)) else 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
