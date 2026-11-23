#!/usr/bin/env python3
"""Sweep around a forward tree while adjusting radius from RGB-D clearance."""

from collections import deque
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Float32, Header
from tf2_ros import Buffer, TransformException, TransformListener

from avp_bridge import quat_wxyz_to_R


class RadiusSweep(Node):
    def __init__(self):
        super().__init__('avp_radius_sweep')
        self.radius = float(self.declare_parameter('radius_m', 1.5).value)
        self.standoff = float(self.declare_parameter('standoff_m', 1.0).value)
        self.detect_range = float(self.declare_parameter('detect_range_m', 2.0).value)
        self.min_altitude = float(self.declare_parameter('min_altitude_m', 1.0).value)
        self.max_delta = float(self.declare_parameter('max_delta_m', 1.0).value)
        self.out_speed = float(self.declare_parameter('out_speed_mps', 0.35).value)
        self.in_speed = float(self.declare_parameter('in_speed_mps', 0.12).value)
        self.radial_gain = float(self.declare_parameter('radial_gain', 0.8).value)
        self.min_sweep_speed = float(self.declare_parameter('min_sweep_speed', 0.35).value)
        self.roi_radius = float(self.declare_parameter('roi_radius_m', 0.7).value)
        self.voxel = float(self.declare_parameter('voxel_m', 0.08).value)
        self.arc_sec = float(self.declare_parameter('arc_sec', 8.0).value)
        self.angle_max = math.radians(float(self.declare_parameter('angle_max_deg', 40.0).value))
        if min(self.radius, self.standoff, self.detect_range, self.min_altitude, self.max_delta, self.out_speed,
               self.in_speed, self.radial_gain, self.roi_radius, self.voxel, self.arc_sec) <= 0:
            raise ValueError('Radius sweep parameters must be positive')
        if not 0 < self.min_sweep_speed <= 1 or not 0 < self.angle_max < math.pi / 2:
            raise ValueError('Invalid sweep speed or angle')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.odom = None
        self.odom_time = 0.0
        self.world_altitude = None
        self.world_altitude_time = 0.0
        self.height_cut = False
        self.cloud_time = 0.0
        self.last_cloud_time = 0.0
        self.distance = None
        self.tip_history = deque(maxlen=3)
        self.ready = False
        self.center = None
        self.start = None
        self.heading = 0.0
        self.delta_radius = 0.0
        self.path_time = 0.0
        self.last_tick = 0.0
        self.anchor_until = 0.0
        self.last_log = 0.0
        self.last_path = 0.0
        self.base_path = Path(header=Header(frame_id='map'))
        self.sweep_path = Path(header=Header(frame_id='map'))

        self.setpoint_pub = self.create_publisher(PoseStamped, '/avp/setpoint', 10)
        self.base_pub = self.create_publisher(PoseStamped, '/avp/base_setpoint', 10)
        self.center_pub = self.create_publisher(PoseStamped, '/avp/sweep_center', 1)
        self.base_path_pub = self.create_publisher(Path, '/avp/base_path', 1)
        self.sweep_path_pub = self.create_publisher(Path, '/avp/sweep_path', 1)
        self.front_pub = self.create_publisher(PointCloud2, '/avp/front_surface', qos_profile_sensor_data)
        self.delta_pub = self.create_publisher(Float32, '/avp/radius_delta', 10)
        self.distance_pub = self.create_publisher(Float32, '/avp/front_distance', 10)
        self.create_subscription(Odometry, '/lics/odom', self.odom_cb, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/model/hummingbird_0/pose', self.world_pose_cb, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/rgbd_camera/points', self.cloud_cb, qos_profile_sensor_data)
        self.create_subscription(Bool, '/avp/path_ready', self.ready_cb, 10)
        self.create_timer(0.05, self.tick)

    def odom_cb(self, msg):
        self.odom = msg
        self.odom_time = time.monotonic()

    def world_pose_cb(self, msg):
        if math.isfinite(msg.pose.position.z):
            self.world_altitude = msg.pose.position.z
            self.world_altitude_time = time.monotonic()

    def ready_cb(self, msg):
        self.ready = bool(msg.data)

    def cloud_cb(self, msg):
        if self.center is None or self.odom is None:
            return
        try:
            tf = self.tf_buffer.lookup_transform('map', msg.header.frame_id, Time())
        except TransformException:
            return
        xyz = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)[::6]
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        if len(xyz) == 0:
            return
        q, p = tf.transform.rotation, tf.transform.translation
        R = quat_wxyz_to_R((q.w, q.x, q.y, q.z))
        if R is None:
            return
        world = np.asarray(xyz, dtype=float) @ R.T + np.array([p.x, p.y, p.z])
        actual = self.odom.pose.pose.position
        drone = np.array([actual.x, actual.y, actual.z])
        look = self.center - drone
        look_norm = np.linalg.norm(look)
        if look_norm < 0.1:
            return
        look /= look_norm
        rel = world - drone
        forward = rel @ look
        lateral = np.linalg.norm(rel - forward[:, None] * look, axis=1)
        mask = (forward > 0.25) & (forward < self.detect_range) & (lateral < self.roi_radius)
        mask &= (world[:, 2] > self.center[2] - 1.7) & (world[:, 2] < self.center[2] + 1.5)
        mask &= np.linalg.norm(world[:, :2] - self.center[:2], axis=1) < 1.3
        mask &= np.linalg.norm(rel, axis=1) < self.detect_range
        world, rel = world[mask], rel[mask]
        if len(world) < 8:
            return
        self.front_pub.publish(point_cloud2.create_cloud_xyz32(
            Header(stamp=msg.header.stamp, frame_id='map'), world[::3].astype(np.float32)))

        distances = np.linalg.norm(rel, axis=1)
        trend = float(np.quantile(distances, 0.2))
        cells, inverse, counts = np.unique(np.floor(world / self.voxel).astype(np.int32),
                                           axis=0, return_inverse=True, return_counts=True)
        cell_near = np.full(len(cells), np.inf)
        np.minimum.at(cell_near, inverse, distances)
        supported = cell_near[counts >= 2]
        self.tip_history.append(float(np.min(supported)) if len(supported) else trend)
        raw = min(trend, float(np.median(self.tip_history))) if len(self.tip_history) == 3 else trend
        now = time.monotonic()
        dt = max(0.0, min(now - self.last_cloud_time, 0.5)) if self.last_cloud_time else 1.0 / 15.0
        self.last_cloud_time = now
        self.cloud_time = now
        if self.distance is None:
            self.distance = raw
        else:
            tau = 0.25 if raw < self.distance else 1.5
            alpha = 1.0 - math.pow(0.01, dt / tau)
            self.distance += alpha * (raw - self.distance)

    def publish_pose(self, publisher, position, pitch):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = position
        cy, sy = math.cos(self.heading / 2), math.sin(self.heading / 2)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        msg.pose.orientation.w = cp * cy
        msg.pose.orientation.x = -sp * sy
        msg.pose.orientation.y = sp * cy
        msg.pose.orientation.z = cp * sy
        publisher.publish(msg)
        return msg

    def tick(self):
        now = time.monotonic()
        if not self.ready or self.odom is None or now - self.odom_time > 0.5:
            self.last_tick = now
            return
        if self.center is None:
            pose = self.odom.pose.pose
            p, q = pose.position, pose.orientation
            self.start = np.array([p.x, p.y, p.z])
            self.heading = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            self.center = self.start + np.array([self.radius * math.cos(self.heading),
                                                 self.radius * math.sin(self.heading), 0.0])
            self.anchor_until = now + 0.7
            self.last_tick = now
            self.get_logger().info(f'Sweep center={self.center.round(2)}, heading={self.heading:.2f} rad')
        self.publish_pose(self.center_pub, self.center, 0.0)
        if now < self.anchor_until:
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            msg.pose = self.odom.pose.pose
            self.setpoint_pub.publish(msg)
            self.base_pub.publish(msg)
            return
        if self.world_altitude is None or now - self.world_altitude_time > 0.5:
            self.last_tick = now
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            msg.pose = self.odom.pose.pose
            self.setpoint_pub.publish(msg)
            self.base_pub.publish(msg)
            if now - self.last_log > 2.0:
                self.last_log = now
                self.get_logger().warning('Waiting for fresh Gazebo height; holding position.')
            return
        if self.distance is not None and now - self.cloud_time > 0.5:
            self.distance = None
            self.tip_history.clear()

        dt = max(0.0, min(now - self.last_tick, 0.1))
        self.last_tick = now
        error = self.standoff - self.distance if self.distance is not None else 0.0
        sweep_speed = max(self.min_sweep_speed, 1.0 - max(0.0, error))
        radial_speed = max(-self.in_speed, min(self.out_speed, self.radial_gain * error))
        self.delta_radius = max(0.0, min(self.max_delta, self.delta_radius + radial_speed * dt))
        self.path_time += dt * sweep_speed
        phase = (self.path_time / self.arc_sec) % 4.0
        segment = int(phase)
        u = phase - segment
        s = u * u * (3.0 - 2.0 * u)
        theta = (self.angle_max * s if segment == 0 else
                 self.angle_max * (1.0 - s) if segment == 1 else
                 -self.angle_max * s if segment == 2 else
                 -self.angle_max * (1.0 - s))
        actual = self.odom.pose.pose.position
        center_altitude = self.world_altitude + self.start[2] - actual.z
        sin_floor = (self.min_altitude - center_altitude) / (self.radius + self.delta_radius)
        theta_floor = math.asin(max(-1.0, min(1.0, sin_floor)))
        was_cut = self.height_cut
        self.height_cut = theta < theta_floor
        if self.height_cut:
            theta = min(theta_floor, self.angle_max)
            if not was_cut:
                self.get_logger().warning(
                    f'Lower arc clipped at {self.min_altitude:.2f} m; holding sweep angle until the upward arc resumes.')
        forward = np.array([math.cos(self.heading), math.sin(self.heading), 0.0])
        base = self.start + self.radius * (1.0 - math.cos(theta)) * forward
        base[2] += self.radius * math.sin(theta)
        target = self.start + (self.radius - (self.radius + self.delta_radius) * math.cos(theta)) * forward
        target[2] += (self.radius + self.delta_radius) * math.sin(theta)
        base_msg = self.publish_pose(self.base_pub, base, theta)
        target_msg = self.publish_pose(self.setpoint_pub, target, theta)
        if now - self.last_path >= 0.1:
            self.last_path = now
            for path, pose, publisher in ((self.base_path, base_msg, self.base_path_pub),
                                          (self.sweep_path, target_msg, self.sweep_path_pub)):
                path.header.stamp = pose.header.stamp
                path.poses.append(pose)
                if len(path.poses) > 600:
                    del path.poses[0]
                publisher.publish(path)
        self.delta_pub.publish(Float32(data=float(self.delta_radius)))
        if self.distance is not None:
            self.distance_pub.publish(Float32(data=float(self.distance)))
        if now - self.last_log > 2.0:
            self.last_log = now
            front = f'{self.distance:.2f} m' if self.distance is not None else 'unseen'
            self.get_logger().info(f'front={front}, delta_R={self.delta_radius:.2f} m, speed={0.0 if self.height_cut else sweep_speed:.2f}')


def main():
    rclpy.init()
    node = RadiusSweep()
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
