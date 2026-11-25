#!/usr/bin/env python3
"""Scan one side of a tree with optional pitch and RGB-D clearance."""

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


TREE_CENTER_DISTANCE_M = 5.0
DETECT_RANGE_M = 6.0
MAX_DELTA_M = 1.0
OUT_SPEED_MPS, IN_SPEED_MPS = 0.35, 0.12
RADIAL_GAIN = 0.8
ROI_RADIUS_M = 0.7


class OutdoorDemo(Node):
    def __init__(self):
        super().__init__('avp_outdoor_demo')
        self.tilting_mode = str(self.declare_parameter('tilting_mode', 'off').value)
        self.standoff = float(self.declare_parameter('standoff_m', 1.0).value)
        self.altitudes = [float(x) for x in self.declare_parameter(
            'altitudes_m', [1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0]).value]
        self.total_sweep = float(self.declare_parameter('total_sweep_s', 200.0).value)
        self.level_sweep = float(self.declare_parameter('level_sweep_s', 8.0).value)
        self.tilt_angle = math.radians(float(self.declare_parameter('tilt_angle_deg', 30.0).value))
        if self.tilting_mode not in ('on', 'off') or self.standoff <= 0:
            raise ValueError('Invalid tilting_mode or standoff_m')
        if len(self.altitudes) < 2 or any(not math.isfinite(x) or x <= 0 for x in self.altitudes):
            raise ValueError('altitudes_m must contain at least two positive finite heights')
        if any(b <= a for a, b in zip(self.altitudes, self.altitudes[1:])):
            raise ValueError('altitudes_m must be strictly increasing')
        self.levels = self.altitudes + self.altitudes[-2::-1]
        if self.level_sweep <= 0 or self.total_sweep <= len(self.levels) * self.level_sweep:
            raise ValueError('total_sweep_s must exceed level_sweep_s times the number of stops')
        if not 0 <= self.tilt_angle <= math.pi / 2:
            raise ValueError('tilt_angle_deg must be in [0, 90]')
        self.transition_s = (self.total_sweep - len(self.levels) * self.level_sweep) / len(self.levels)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.odom = None
        self.odom_time = 0.0
        self.world_altitude = None
        self.world_altitude_time = 0.0
        self.initial_world_altitude = None
        self.ready = False
        self.ready_time = 0.0
        self.last_target_msg = None
        self.cloud_stale_warned = False
        self.cloud_time = 0.0
        self.last_cloud_time = 0.0
        self.distance = None
        self.tip_history = deque(maxlen=3)
        self.center = None
        self.start = None
        self.heading = 0.0
        self.delta_radius = 0.0
        self.path_time = 0.0
        self.last_tick = 0.0
        self.last_log = 0.0
        self.last_path = 0.0
        self.base_path = Path(header=Header(frame_id='map'))
        self.sweep_path = Path(header=Header(frame_id='map'))

        self.setpoint_pub = self.create_publisher(PoseStamped, '/avp/setpoint', 10)
        self.base_pub = self.create_publisher(PoseStamped, '/avp/base_setpoint', 10)
        self.center_pub = self.create_publisher(PoseStamped, '/avp/sweep_center', 1)
        self.base_path_pub = self.create_publisher(Path, '/avp/base_path', 1)
        self.sweep_path_pub = self.create_publisher(Path, '/avp/sweep_path', 1)
        self.front_pub = self.create_publisher(PointCloud2, '/avp/front_surface', 10)
        self.delta_pub = self.create_publisher(Float32, '/avp/radius_delta', 10)
        self.distance_pub = self.create_publisher(Float32, '/avp/front_distance', 10)
        self.create_subscription(Odometry, '/lics/odom', self.odom_cb, qos_profile_sensor_data)
        self.create_subscription(Bool, '/avp/path_ready', self.ready_cb, 10)
        self.create_subscription(PoseStamped, '/model/hummingbird_0/pose', self.world_pose_cb, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/rgbd_camera/points', self.cloud_cb, qos_profile_sensor_data)
        self.create_timer(0.05, self.tick)

    def odom_cb(self, msg):
        self.odom = msg
        self.odom_time = time.monotonic()

    def ready_cb(self, msg):
        self.ready = bool(msg.data)
        self.ready_time = time.monotonic()

    def world_pose_cb(self, msg):
        if math.isfinite(msg.pose.position.z):
            self.world_altitude = msg.pose.position.z
            self.world_altitude_time = time.monotonic()

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
        look = np.array([self.center[0] - drone[0], self.center[1] - drone[1], 0.0])
        look_norm = np.linalg.norm(look)
        if look_norm < 0.1:
            return
        look /= look_norm
        rel = world - drone
        forward = rel @ look
        lateral = np.abs(rel[:, 0] * look[1] - rel[:, 1] * look[0])
        mask = (forward > 0.25) & (forward < DETECT_RANGE_M) & (lateral < ROI_RADIUS_M)
        mask &= np.abs(rel[:, 2]) < 2.5
        world, rel = world[mask], rel[mask]
        if len(world) < 8:
            return
        self.front_pub.publish(point_cloud2.create_cloud_xyz32(
            Header(stamp=msg.header.stamp, frame_id='map'), world[::3].astype(np.float32)))

        trend = float(np.quantile(forward[mask], 0.2))
        self.tip_history.append(trend)
        raw = min(trend, float(np.median(self.tip_history))) if len(self.tip_history) == 3 else trend
        now = time.monotonic()
        dt = max(0.0, min(now - self.last_cloud_time, 0.5)) if self.last_cloud_time else 1.0 / 15.0
        self.last_cloud_time = now
        self.cloud_time = now
        self.cloud_stale_warned = False
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
        yaw = math.atan2(self.center[1] - position[1], self.center[0] - position[0])
        cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        msg.pose.orientation.w = cp * cy
        msg.pose.orientation.x = -sp * sy
        msg.pose.orientation.y = sp * cy
        msg.pose.orientation.z = cp * sy
        publisher.publish(msg)
        return msg

    def sweep_target(self, t):
        previous = self.initial_world_altitude
        for altitude in self.levels:
            if t < self.transition_s:
                u = max(0.0, t / self.transition_s)
                return previous + (altitude - previous) * u * u * (3.0 - 2.0 * u), 0.0
            t -= self.transition_s
            if t < self.level_sweep:
                pitch = self.tilt_angle * math.sin(2.0 * math.pi * t / self.level_sweep) if self.tilting_mode == 'on' else 0.0
                return altitude, pitch
            t -= self.level_sweep
            previous = altitude
        return self.levels[-1], 0.0

    def publish_hold(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose = self.odom.pose.pose if self.last_target_msg is None else self.last_target_msg.pose
        self.setpoint_pub.publish(msg)
        self.base_pub.publish(msg)

    def tick(self):
        now = time.monotonic()
        if self.odom is None or now - self.odom_time > 0.5:
            self.last_tick = now
            return
        if not self.ready or now - self.ready_time > 0.5:
            if self.start is not None:
                self.start = self.center = self.initial_world_altitude = None
                self.distance = self.last_target_msg = None
                self.tip_history.clear()
                self.delta_radius = self.path_time = 0.0
                self.base_path.poses.clear()
                self.sweep_path.poses.clear()
            self.last_tick = now
            return
        if self.world_altitude is None or now - self.world_altitude_time > 0.5:
            self.last_tick = now
            self.publish_hold()
            if now - self.last_log > 2.0:
                self.last_log = now
                self.get_logger().warning('Waiting for fresh Gazebo height; holding position.')
            return
        if self.start is None:
            pose = self.odom.pose.pose
            p, q = pose.position, pose.orientation
            self.start = np.array([p.x, p.y, p.z])
            self.initial_world_altitude = self.world_altitude
            self.heading = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            self.center = self.start + np.array([TREE_CENTER_DISTANCE_M * math.cos(self.heading),
                                                 TREE_CENTER_DISTANCE_M * math.sin(self.heading), 0.0])
            self.last_tick = now
            self.get_logger().info(
                f'Outdoor start={self.start.round(2)}, altitude={self.initial_world_altitude:.2f}, '
                f'levels={self.levels}, total={self.total_sweep:.1f}s, move={self.transition_s:.1f}s')
        self.publish_pose(self.center_pub, self.center, 0.0)
        if self.distance is not None and now - self.cloud_time > 0.5:
            self.distance = None
            self.tip_history.clear()
            if self.tilting_mode == 'on' and not self.cloud_stale_warned:
                self.get_logger().warning('RGB-D clearance stale; holding last target.')
                self.cloud_stale_warned = True
        if self.tilting_mode == 'on' and self.distance is None:
            self.last_tick = now
            self.publish_hold()
            return

        dt = max(0.0, min(now - self.last_tick, 0.1))
        self.last_tick = now
        error = self.standoff - self.distance if self.distance is not None else 0.0
        if self.tilting_mode == 'on':
            radial_speed = max(-IN_SPEED_MPS, min(OUT_SPEED_MPS, RADIAL_GAIN * error))
            self.delta_radius = max(-MAX_DELTA_M, min(MAX_DELTA_M, self.delta_radius + radial_speed * dt))
        if self.tilting_mode == 'off' or error <= 0.15:
            self.path_time = min(self.total_sweep, self.path_time + dt)
        altitude, pitch = self.sweep_target(self.path_time)
        forward = np.array([math.cos(self.heading), math.sin(self.heading), 0.0])
        base = self.start.copy()
        base[2] += altitude - self.initial_world_altitude
        target = base - self.delta_radius * forward
        base_msg = self.publish_pose(self.base_pub, base, pitch)
        target_msg = self.publish_pose(self.setpoint_pub, target, pitch)
        self.last_target_msg = target_msg
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
            self.get_logger().info(f'front={front}, delta_R={self.delta_radius:.2f} m, t={self.path_time:.1f}/{self.total_sweep:.1f}s')


def main():
    rclpy.init()
    node = OutdoorDemo()
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
