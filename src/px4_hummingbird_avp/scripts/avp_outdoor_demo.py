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
MAX_DELTA_M = 3.0
OUT_SPEED_MPS, IN_SPEED_MPS = 0.35, 0.12
RADIAL_GAIN = 0.8
ROI_RADIUS_M = 0.7
PATCH_LATERAL_M, PATCH_HEIGHT_M = 0.5, 1.0
CLIMB_FORWARD_PER_M = 2.1 / 2.3
RAMP_PITCH_RAD = math.radians(30.0)
RETREAT_LEAD_M = 0.15
PITCH_RAMP_S = 1.5
ARC_SEC, MOVE_SEC = 7.0, 5.0
ARC_RADIUS_M, ARC_ANGLE_RAD = 2.5, math.radians(35.0)
ARC_JOIN_SPEED_MPS, MOVE_FORWARD_M = 0.15, 1.5
LOWER_JOIN_SPEED_MPS = 0.12
LOWER_JOIN_PITCH_RATE_RAD_S = math.radians(5.0)
MIN_WORLD_ALTITUDE_M = 0.7
OVERHEAD_RADIUS_M = 1.5


class OutdoorDemo(Node):
    def __init__(self):
        super().__init__('avp_outdoor_demo')
        self.tilting_mode = str(self.declare_parameter('tilting_mode', 'off').value)
        self.standoff = float(self.declare_parameter('standoff_m', 1.0).value)
        self.max_altitude = float(self.declare_parameter('max_altitude_m', 5.0).value)
        self.total_sweep = float(self.declare_parameter('total_sweep_s', 200.0).value)
        self.level_sweep = float(self.declare_parameter('level_sweep_s', 8.0).value)
        self.tilt_angle = math.radians(float(self.declare_parameter('tilt_angle_deg', 30.0).value))
        if self.tilting_mode not in ('on', 'off') or self.standoff <= 0:
            raise ValueError('Invalid tilting_mode or standoff_m')
        if not math.isfinite(self.max_altitude) or self.max_altitude <= 0:
            raise ValueError('max_altitude_m must be positive and finite')
        lower_sweep = 2.0 * (ARC_SEC + MOVE_SEC) if self.tilting_mode == 'on' else 0.0
        top_sweep = self.level_sweep if self.tilting_mode == 'on' else 0.0
        if self.level_sweep <= 0 or self.total_sweep <= top_sweep + lower_sweep:
            raise ValueError('total_sweep_s must exceed the top and lower sweep times')
        if not 0 <= self.tilt_angle <= math.pi / 2:
            raise ValueError('tilt_angle_deg must be in [0, 90]')
        self.move_s = (self.total_sweep - top_sweep - lower_sweep) / 2.0
        self.lower_start_s = 2.0 * self.move_s + top_sweep
        # Positive ENU/FLU pitch becomes negative PX4 NED/FRD pitch (nose-down).
        self.surface_pitch = RAMP_PITCH_RAD if self.tilting_mode == 'on' else 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.odom = None
        self.odom_time = 0.0
        self.world_altitude = None
        self.initial_world_altitude = None
        self.ready = False
        self.ready_time = 0.0
        self.last_target_msg = None
        self.cloud_stale_warned = False
        self.cloud_time = 0.0
        self.last_cloud_time = 0.0
        self.distance = None
        self.overhead_distance = None
        self.overhead_time = 0.0
        self.last_overhead_time = 0.0
        self.tip_history = deque(maxlen=3)
        self.center = None
        self.start = None
        self.heading = 0.0
        self.delta_radius = 0.0
        self.delta_height = 0.0
        self.arc_radius = ARC_RADIUS_M
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
        self.create_subscription(PointCloud2, '/rgbd_camera/points', self.cloud_cb, qos_profile_sensor_data)
        self.create_timer(0.05, self.tick)

    def odom_cb(self, msg):
        if not math.isfinite(msg.pose.pose.position.z):
            return
        self.odom = msg
        self.odom_time = time.monotonic()
        self.world_altitude = msg.pose.pose.position.z

    def ready_cb(self, msg):
        self.ready = bool(msg.data)
        self.ready_time = time.monotonic()

    def cloud_cb(self, msg):
        if self.center is None or self.odom is None:
            return
        try:
            tf = self.tf_buffer.lookup_transform('map', msg.header.frame_id, Time())
        except TransformException:
            return
        q, p = tf.transform.rotation, tf.transform.translation
        R = quat_wxyz_to_R((q.w, q.x, q.y, q.z))
        if R is None:
            return
        self.cloud_time = time.monotonic()
        self.cloud_stale_warned = False
        xyz = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)[::6]
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        if len(xyz) == 0:
            self.distance = self.overhead_distance = None
            self.tip_history.clear()
            return
        world = np.asarray(xyz, dtype=float) @ R.T + np.array([p.x, p.y, p.z])
        actual = self.odom.pose.pose.position
        drone = np.array([actual.x, actual.y, actual.z])
        rel_all = world - drone
        above = (rel_all[:, 2] > 0.05) & (rel_all[:, 2] < DETECT_RANGE_M)
        above &= np.linalg.norm(rel_all[:, :2], axis=1) < OVERHEAD_RADIUS_M
        if np.count_nonzero(above) >= 8:
            raw_above = float(np.quantile(rel_all[above, 2], 0.2))
            now = time.monotonic()
            dt = max(0.0, min(now - self.last_overhead_time, 0.5)) if self.last_overhead_time else 1.0 / 15.0
            self.last_overhead_time = self.overhead_time = now
            if self.overhead_distance is None:
                self.overhead_distance = raw_above
            else:
                tau = 0.25 if raw_above < self.overhead_distance else 1.5
                self.overhead_distance += (1.0 - math.pow(0.01, dt / tau)) * (raw_above - self.overhead_distance)
        else:
            self.overhead_distance = None
        look = np.array([self.center[0] - drone[0], self.center[1] - drone[1], 0.0])
        look_norm = np.linalg.norm(look)
        if look_norm < 0.1:
            return
        look /= look_norm
        rel = world - drone
        forward = rel @ look
        lateral = np.abs(rel[:, 0] * look[1] - rel[:, 1] * look[0])
        mask = (forward > 0.05) & (forward < DETECT_RANGE_M) & (lateral < ROI_RADIUS_M)
        mask &= np.abs(rel[:, 2]) < 2.5
        world, rel = world[mask], rel[mask]
        if len(world) < 8:
            self.distance = None
            self.tip_history.clear()
            return
        self.front_pub.publish(point_cloud2.create_cloud_xyz32(
            Header(stamp=msg.header.stamp, frame_id='map'), world[::3].astype(np.float32)))

        lateral = rel[:, 0] * look[1] - rel[:, 1] * look[0]
        patch = np.column_stack((np.floor(lateral / PATCH_LATERAL_M),
                                 np.floor(rel[:, 2] / PATCH_HEIGHT_M)))
        _, inverse = np.unique(patch, axis=0, return_inverse=True)
        counts = np.bincount(inverse)
        fronts = [float(np.quantile(forward[mask][inverse == i], 0.2))
                  for i in np.flatnonzero(counts >= 8)]
        if not fronts:
            self.distance = None
            self.tip_history.clear()
            return
        if self.tilting_mode == 'on' and self.move_s <= self.path_time < self.move_s + self.level_sweep:
            self.distance = None
            self.tip_history.clear()
            return
        trend = float(np.quantile(fronts, 0.2))
        self.tip_history.append(trend)
        raw = min(trend, float(np.median(self.tip_history))) if len(self.tip_history) == 3 else trend
        now = time.monotonic()
        dt = max(0.0, min(now - self.last_cloud_time, 0.5)) if self.last_cloud_time else 1.0 / 15.0
        self.last_cloud_time = now
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
        climb = self.max_altitude - self.initial_world_altitude
        climb_forward = CLIMB_FORWARD_PER_M * climb if self.tilting_mode == 'on' else 0.0
        arc_down = self.arc_radius * math.sin(ARC_ANGLE_RAD)
        join_speed = min(LOWER_JOIN_SPEED_MPS, 1.5 * climb / self.move_s, 1.5 * arc_down / ARC_SEC) if self.tilting_mode == 'on' else 0.0
        if t < self.move_s:
            u = max(0.0, min(t / self.move_s, 1.0))
            s = u * u * (3.0 - 2.0 * u)
            r = min(t / PITCH_RAMP_S, 1.0)
            return self.initial_world_altitude + climb * s, climb_forward * s, self.surface_pitch * r * r * (3.0 - 2.0 * r)
        t -= self.move_s
        if self.tilting_mode == 'on' and t < self.level_sweep:
            u = t / self.level_sweep
            pitch = self.surface_pitch - (self.surface_pitch + self.tilt_angle) * math.sin(math.pi * u) ** 2
            return self.max_altitude, climb_forward, pitch
        if self.tilting_mode == 'on':
            t -= self.level_sweep
        if t < self.move_s:
            u = max(0.0, min(t / self.move_s, 1.0))
            s = u * u * (3.0 - 2.0 * u)
            r = min((self.move_s - t) / PITCH_RAMP_S, 1.0)
            pitch = self.surface_pitch * r * r * (3.0 - 2.0 * r)
            if self.tilting_mode == 'on' and r < 1.0:
                v = 1.0 - r
                pitch -= LOWER_JOIN_PITCH_RATE_RAD_S * PITCH_RAMP_S * (v ** 3 - v * v)
            altitude = self.max_altitude - climb * s - join_speed * self.move_s * (u ** 3 - u * u)
            return altitude, climb_forward * (1.0 - s), pitch
        if self.tilting_mode == 'off':
            return self.initial_world_altitude, 0.0, 0.0
        t -= self.move_s

        arc_forward = self.arc_radius * (1.0 - math.cos(ARC_ANGLE_RAD))
        if t < ARC_SEC:
            u = t / ARC_SEC
            s = u * u * (3.0 - 2.0 * u)
            x = arc_forward * s + ARC_JOIN_SPEED_MPS * ARC_SEC * (u ** 3 - u * u)
            altitude = self.initial_world_altitude - arc_down * s - join_speed * ARC_SEC * (u ** 3 - 2.0 * u * u + u)
            pitch = -ARC_ANGLE_RAD * s - LOWER_JOIN_PITCH_RATE_RAD_S * ARC_SEC * (u ** 3 - 2.0 * u * u + u)
            return altitude, x, pitch
        t -= ARC_SEC
        if t < MOVE_SEC:
            u = t / MOVE_SEC
            s = u * u * (3.0 - 2.0 * u)
            x = arc_forward + MOVE_FORWARD_M * s + ARC_JOIN_SPEED_MPS * MOVE_SEC * (u ** 3 - 2.0 * u * u + u)
            return self.initial_world_altitude - arc_down, x, -ARC_ANGLE_RAD
        t -= MOVE_SEC
        if t < MOVE_SEC:
            u = t / MOVE_SEC
            s = u * u * (3.0 - 2.0 * u)
            x = arc_forward + MOVE_FORWARD_M * (1.0 - s) - ARC_JOIN_SPEED_MPS * MOVE_SEC * (u ** 3 - u * u)
            return self.initial_world_altitude - arc_down, x, -ARC_ANGLE_RAD
        t -= MOVE_SEC
        if t < ARC_SEC:
            u = t / ARC_SEC
            s = u * u * (3.0 - 2.0 * u)
            x = arc_forward * (1.0 - s) - ARC_JOIN_SPEED_MPS * ARC_SEC * (u ** 3 - 2.0 * u * u + u)
            return self.initial_world_altitude - arc_down * (1.0 - s), x, -ARC_ANGLE_RAD * (1.0 - s)
        return self.initial_world_altitude, 0.0, 0.0

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
                self.distance = self.overhead_distance = self.last_target_msg = None
                self.tip_history.clear()
                self.delta_radius = self.delta_height = self.path_time = 0.0
                self.base_path.poses.clear()
                self.sweep_path.poses.clear()
            self.last_tick = now
            return
        if self.start is None:
            if self.world_altitude >= self.max_altitude - 0.05:
                self.last_tick = now
                self.publish_hold()
                if now - self.last_log > 2.0:
                    self.last_log = now
                    self.get_logger().warning('Current height is at or above max_altitude_m; holding start.')
                return
            pose = self.odom.pose.pose
            p, q = pose.position, pose.orientation
            self.start = np.array([p.x, p.y, p.z])
            self.initial_world_altitude = self.world_altitude
            if self.tilting_mode == 'on':
                self.arc_radius = min(ARC_RADIUS_M, max(0.0, (self.world_altitude - MIN_WORLD_ALTITUDE_M) / math.sin(ARC_ANGLE_RAD)))
                if self.arc_radius < ARC_RADIUS_M:
                    self.get_logger().warning(f'Lower arc radius limited to {self.arc_radius:.2f} m by ground clearance.')
            self.heading = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            self.center = self.start + np.array([TREE_CENTER_DISTANCE_M * math.cos(self.heading),
                                                 TREE_CENTER_DISTANCE_M * math.sin(self.heading), 0.0])
            self.last_tick = now
            self.get_logger().info(
                f'Outdoor start={self.start.round(2)}, GPS-local altitude={self.initial_world_altitude:.2f}, '
                f'peak={self.max_altitude:.1f}m, total={self.total_sweep:.1f}s, move={self.move_s:.1f}s')
        self.publish_pose(self.center_pub, self.center, 0.0)
        active = self.path_time < self.total_sweep
        in_top_sweep = active and self.tilting_mode == 'on' and self.move_s <= self.path_time < self.move_s + self.level_sweep
        in_lower_sweep = active and self.tilting_mode == 'on' and self.path_time >= self.lower_start_s
        current_altitude, current_forward, current_pitch = self.sweep_target(self.path_time)
        need_overhead = in_lower_sweep and current_pitch < -math.radians(20.0)
        cloud_stale = now - self.cloud_time > 0.5
        if cloud_stale:
            self.distance = self.overhead_distance = None
            self.tip_history.clear()
            if self.tilting_mode == 'on' and active and not in_top_sweep and not self.cloud_stale_warned:
                self.get_logger().warning('RGB-D point cloud stale; holding last target.')
                self.cloud_stale_warned = True
        if self.tilting_mode == 'on' and active and not in_top_sweep and cloud_stale:
            self.last_tick = now
            self.publish_hold()
            return

        dt = max(0.0, min(now - self.last_tick, 0.1))
        self.last_tick = now
        front_error = self.standoff - self.distance if self.distance is not None else 0.0
        overhead_error = self.standoff - self.overhead_distance if self.overhead_distance is not None else 0.0
        forward = np.array([math.cos(self.heading), math.sin(self.heading), 0.0])
        normal = np.array([-forward[0], -forward[1], CLIMB_FORWARD_PER_M])
        normal /= math.hypot(1.0, CLIMB_FORWARD_PER_M)
        if self.tilting_mode == 'on' and active and not in_top_sweep and not in_lower_sweep and self.distance is None:
            self.delta_radius = max(0.0, self.delta_radius - IN_SPEED_MPS * dt)
        elif self.tilting_mode == 'on' and active and not in_top_sweep and not in_lower_sweep:
            radial_speed = max(-IN_SPEED_MPS, min(OUT_SPEED_MPS, RADIAL_GAIN * front_error))
            if front_error > 0.15:
                current_base = self.start.copy() + current_forward * forward
                current_base[2] += current_altitude - self.initial_world_altitude
                actual = self.odom.pose.pose.position
                actual_pos = np.array([actual.x, actual.y, actual.z])
                retreat = np.dot(actual_pos - current_base, normal) + RETREAT_LEAD_M
                self.delta_radius = min(MAX_DELTA_M, max(self.delta_radius + radial_speed * dt, retreat))
            else:
                self.delta_radius = max(0.0, min(MAX_DELTA_M, self.delta_radius + radial_speed * dt))
        if need_overhead and self.overhead_distance is not None:
            height_speed = max(-OUT_SPEED_MPS, min(IN_SPEED_MPS, -RADIAL_GAIN * overhead_error))
            self.delta_height = max(-MAX_DELTA_M, min(MAX_DELTA_M, self.delta_height + height_speed * dt))
            if overhead_error > 0.15:
                actual_z = self.odom.pose.pose.position.z
                base_z = self.start[2] + current_altitude - self.initial_world_altitude
                retreat_z = actual_z - base_z - self.delta_radius * normal[2] - RETREAT_LEAD_M
                self.delta_height = max(-MAX_DELTA_M, min(self.delta_height, retreat_z))
        elif need_overhead:
            self.delta_height += max(-IN_SPEED_MPS * dt, min(IN_SPEED_MPS * dt, -self.delta_height))
        if not active or self.tilting_mode == 'off' or in_top_sweep or (in_lower_sweep and (not need_overhead or overhead_error <= 0.15)) or (not in_lower_sweep and front_error <= 0.15):
            self.path_time = min(self.total_sweep, self.path_time + dt)
        altitude, forward_m, pitch = self.sweep_target(self.path_time)
        base = self.start.copy() + forward_m * forward
        base[2] += altitude - self.initial_world_altitude
        target = base + self.delta_radius * normal
        if self.tilting_mode == 'on' and self.path_time >= self.lower_start_s:
            target[2] = max(target[2] + self.delta_height, self.start[2] + MIN_WORLD_ALTITUDE_M - self.initial_world_altitude)
        if self.tilting_mode == 'on':
            target[2] = min(target[2], self.start[2] + self.max_altitude - self.initial_world_altitude)
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
        x_backoff = self.delta_radius / math.hypot(1.0, CLIMB_FORWARD_PER_M)
        self.delta_pub.publish(Float32(data=float(x_backoff)))
        if self.distance is not None:
            self.distance_pub.publish(Float32(data=float(self.distance)))
        if now - self.last_log > 2.0:
            self.last_log = now
            front = f'{self.distance:.2f} m' if self.distance is not None else 'unseen'
            above = f'{self.overhead_distance:.2f} m' if self.overhead_distance is not None else 'unseen'
            self.get_logger().info(f'front={front}, overhead={above}, delta_R={x_backoff:.2f} m, delta_z={target[2] - base[2]:.2f} m, t={self.path_time:.1f}/{self.total_sweep:.1f}s')


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
