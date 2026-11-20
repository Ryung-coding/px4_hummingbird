#!/usr/bin/env python3
"""Straight AVP sweep executor. All positions and setpoints use map/ENU."""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseArray, PoseStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import HummingbirdStatus, VehicleStatus
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


def rpy_to_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy)


def quat_to_pitch(w, x, y, z):
    sinp = 2.0 * (w * y - z * x)
    return math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)


class CorridorExecutor(Node):
    def __init__(self):
        super().__init__('corridor_executor')

        def dp(name, default):
            descriptor = ParameterDescriptor(dynamic_typing=True) if isinstance(default, float) else ParameterDescriptor()
            self.declare_parameter(name, default, descriptor)

        dp('x_min', -0.75); dp('x_max', 0.75)
        dp('y_min', -1.6); dp('y_max', 2.1)
        dp('z_min', 0.45); dp('z_max', 2.1)
        dp('sweep_y_min', -1.0); dp('sweep_y_max', 1.0)
        dp('z_levels', [1.0, 1.0, 1.0, 1.0])
        dp('cruise_speed', 0.3); dp('accel', 0.3)
        dp('level_hold_s', 2.0); dp('dwell_s', 2.0)
        dp('wait_vp_s', 3.0); dp('publish_rate_hz', 100.0)
        dp('use_vp_pitch', True); dp('cam_pitch_deg', 0.0)
        dp('pitch_rate_deg_s', 30.0)
        dp('vp_merge_m', 0.25); dp('vp_level_tol_m', 0.3)
        dp('replan_min_s', 1.5); dp('clear_m', 0.5)
        dp('occ_resolution', 0.1)
        dp('arrival_tol_m', 0.1); dp('tracking_tol_m', 0.5)
        dp('odom_timeout_s', 0.5); dp('require_control', True)
        dp('setpoint_topic', '/avp/setpoint'); dp('odom_topic', '/lics/odom')

        gp = lambda name: self.get_parameter(name).value
        self.xmin = float(gp('x_min')); self.xmax = float(gp('x_max'))
        self.ymin = float(gp('y_min')); self.ymax = float(gp('y_max'))
        self.zmin = float(gp('z_min')); self.zmax = float(gp('z_max'))
        self.sweep_y_min = float(gp('sweep_y_min')); self.sweep_y_max = float(gp('sweep_y_max'))
        self.z_levels = [float(z) for z in gp('z_levels')]
        self.vc = float(gp('cruise_speed')); self.acc = float(gp('accel'))
        self.level_hold_s = float(gp('level_hold_s')); self.dwell_s = float(gp('dwell_s'))
        self.wait_vp_s = float(gp('wait_vp_s')); self.rate = float(gp('publish_rate_hz'))
        self.use_vp_pitch = bool(gp('use_vp_pitch'))
        self.cam_pitch = math.radians(float(gp('cam_pitch_deg')))
        self.pitch_rate = math.radians(float(gp('pitch_rate_deg_s')))
        self.vp_merge_m = float(gp('vp_merge_m')); self.vp_level_tol_m = float(gp('vp_level_tol_m'))
        self.replan_min = float(gp('replan_min_s')); self.clear_m = float(gp('clear_m'))
        self.arrival_tol = float(gp('arrival_tol_m')); self.tracking_tol = float(gp('tracking_tol_m'))
        self.odom_timeout = float(gp('odom_timeout_s')); self.require_control = bool(gp('require_control'))

        x = 0.5 * (self.xmin + self.xmax)
        self.waypoints = []
        for i, z in enumerate(self.z_levels):
            y0, y1 = (self.sweep_y_max, self.sweep_y_min) if i % 2 == 0 else (self.sweep_y_min, self.sweep_y_max)
            self.waypoints.extend((np.array([x, y0, z]), np.array([x, y1, z])))
        if not (self.xmin < self.xmax and self.ymin < self.ymax and self.zmin < self.zmax
                and self.ymin <= self.sweep_y_min < self.sweep_y_max <= self.ymax
                and self.z_levels and all(self.zmin <= z <= self.zmax for z in self.z_levels)
                and self.vc > 0.0 and self.acc > 0.0 and self.rate > 0.0):
            raise ValueError('Invalid straight AVP corridor or sweep parameters')

        self.occ = set()
        self.occ_xyz = np.empty((0, 3), dtype=float)
        self.occ_res = float(gp('occ_resolution'))
        if self.occ_res <= 0.0:
            raise ValueError('occ_resolution must be positive')
        rc = int(math.ceil(self.clear_m / self.occ_res))
        self.clear_off = [(i, j, k) for i in range(-rc, rc + 1)
                          for j in range(-rc, rc + 1) for k in range(-rc, rc + 1)
                          if (i * i + j * j + k * k) * self.occ_res ** 2 <= self.clear_m ** 2]
        self.phase = 'wait'
        self.leg = -1
        self.target = None
        self.target_is_vp = False
        self.start = None
        self.pitch_start = 0.0
        self.pitch_end = 0.0
        self.length = 0.0
        self.s = 0.0
        self.v = 0.0
        self.hold_s = 0.0
        self.dwell_t0 = None
        self.pos_enu = None
        self.last_odom_t = None
        self.vehicle_status = None
        self.last_status_t = None
        self.hb_status = None
        self.last_hb_t = None
        self.path_ready = False
        self.last_ready_t = None
        self.latest = None
        self.batch_dirty = False
        self.visited = []
        self.rejected = []
        self.recovering = False
        self.t_first_odom = None
        self.last_replan_t = -1e9
        self.last_warn_t = -1e9
        self.last_tick_t = None
        self.paused_at = None
        self.cur_cmd_pos = None
        self.cur_cmd_pitch = 0.0
        self.last_pitch_t = None
        self.last_base_t = -1e9
        self.trail = []
        self.last_trail_t = -1e9

        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         durability=QoSDurabilityPolicy.VOLATILE,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Odometry, str(gp('odom_topic')), self.odom_cb, qos)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.status_cb, qos)
        self.create_subscription(HummingbirdStatus, '/fmu/out/hummingbird_status', self.hb_cb, qos)
        self.create_subscription(Bool, '/avp/path_ready', self.ready_cb, 1)
        self.create_subscription(PoseArray, '/avp/viewpoints_data', self.vp_cb, 1)
        self.create_subscription(PointCloud2, '/avp/occ_voxels', self.occ_cb, 1)
        self.sp_pub = self.create_publisher(PoseStamped, str(gp('setpoint_topic')), 10)
        self.base_pub = self.create_publisher(PoseStamped, '/avp/base_setpoint', 10)
        self.path_pub = self.create_publisher(MarkerArray, '/avp/path', 1)
        self.trail_pub = self.create_publisher(MarkerArray, '/avp/trail', 1)
        self.safety_pub = self.create_publisher(MarkerArray, '/avp/safety_region', 1)
        self.create_timer(1.0, self._publish_safety)
        self.create_timer(1.0 / self.rate, self.tick)
        self.get_logger().info(f'straight AVP ready: x=[{self.xmin},{self.xmax}] y=[{self.ymin},{self.ymax}] '
                               f'z=[{self.zmin},{self.zmax}] levels={self.z_levels}')

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _warn(self, message, error=False):
        now = self._now()
        if now - self.last_warn_t >= 2.0:
            self.last_warn_t = now
            (self.get_logger().error if error else self.get_logger().warn)(message)

    def _inside(self, p):
        return (np.all(np.isfinite(p)) and self.xmin <= p[0] <= self.xmax
                and self.ymin <= p[1] <= self.ymax and self.zmin <= p[2] <= self.zmax)

    def _occupied(self, p):
        if not self.occ:
            return False
        c = np.floor(p / self.occ_res).astype(np.int64)
        return any((int(c[0] + i), int(c[1] + j), int(c[2] + k)) in self.occ
                   for i, j, k in self.clear_off)

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        self.pos_enu = np.array([float(p.x), float(p.y), float(p.z)])
        self.last_odom_t = self._now()

    def status_cb(self, msg):
        self.vehicle_status = msg
        self.last_status_t = self._now()

    def hb_cb(self, msg):
        self.hb_status = msg
        self.last_hb_t = self._now()

    def ready_cb(self, msg):
        self.path_ready = bool(msg.data)
        self.last_ready_t = self._now()

    def vp_cb(self, msg):
        self.latest = []
        for pose in msg.poses:
            p = np.array([pose.position.x, pose.position.y, pose.position.z], dtype=float)
            if not self._inside(p):
                continue
            q = pose.orientation
            self.latest.append((p, quat_to_pitch(q.w, q.x, q.y, q.z)))
        self.batch_dirty = True

    def occ_cb(self, msg):
        n = msg.width * msg.height
        if n == 0:
            self.occ = set()
            self.occ_xyz = np.empty((0, 3), dtype=float)
            return
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, msg.point_step)[:, :12].copy().view(np.float32).reshape(-1, 3)
        self.occ_xyz = arr
        self.occ = set(map(tuple, np.floor(arr / self.occ_res).astype(np.int64).tolist()))

    def _nearest_occ_sq(self, p):
        if len(self.occ_xyz) == 0:
            return math.inf
        delta = self.occ_xyz - p
        return float(np.min(np.einsum('ij,ij->i', delta, delta)))

    def _control_ready(self, now):
        if not self.require_control:
            return True
        return (self.vehicle_status is not None and self.last_status_t is not None
                and now - self.last_status_t <= 1.0
                and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
                and self.hb_status is not None and self.last_hb_t is not None
                and now - self.last_hb_t <= 1.0 and self.hb_status.dds_command_enabled
                and self.path_ready and self.last_ready_t is not None and now - self.last_ready_t <= 0.5)

    def _next_vp(self):
        if self.latest is None or self.leg < 0:
            return None
        start, end = self.waypoints[self.leg:self.leg + 2]
        dy = end[1] - start[1]
        if abs(dy) < 1e-6:
            return None
        direction = math.copysign(1.0, dy)
        candidates = []
        for p, pitch in self.latest:
            if abs(p[2] - start[2]) > self.vp_level_tol_m:
                continue
            if any(np.linalg.norm(p - used) < self.vp_merge_m for used in self.visited + self.rejected):
                continue
            if self._occupied(p):
                continue
            ahead = direction * (p[1] - self.cur_cmd_pos[1])
            if ahead < self.vp_merge_m or direction * (end[1] - p[1]) < 0.0:
                continue
            steps = max(1, int(math.ceil(np.linalg.norm(p - self.cur_cmd_pos) / self.occ_res)))
            if any(self._occupied(self.cur_cmd_pos + (p - self.cur_cmd_pos) * (i / steps))
                   for i in range(1, steps + 1)):
                continue
            candidates.append((ahead, p, pitch))
        if candidates:
            _, p, pitch = min(candidates, key=lambda item: item[0])
            return p, pitch
        return None

    def _select_target(self):
        vp = self._next_vp()
        if vp is not None:
            target, pitch = vp
            self.target_is_vp = True
        else:
            target = self.waypoints[0] if self.leg < 0 else self.waypoints[self.leg + 1]
            pitch = self.cam_pitch
            self.target_is_vp = False
        self.start = self.cur_cmd_pos.copy()
        self.target = target.copy()
        self.pitch_start = self.cur_cmd_pitch
        self.pitch_end = pitch if self.use_vp_pitch else self.cam_pitch
        self.length = float(np.linalg.norm(self.target - self.start))
        self.s = 0.0
        self.v = 0.0
        self.phase = 'fly'
        self._publish_path()

    def _recover_from_vp(self):
        self.rejected.append(self.target.copy())
        self.get_logger().warn(f'viewpoint blocked by updated map; returning to base sweep: {self.target.tolist()}')
        self.start = self.cur_cmd_pos.copy()
        self.target = self._base_point(self.cur_cmd_pos).copy()
        self.target_is_vp = False
        self.recovering = True
        self.pitch_start = self.cur_cmd_pitch
        self.pitch_end = self.cam_pitch
        self.length = float(np.linalg.norm(self.target - self.start))
        self.s = 0.0
        self.v = 0.0
        self.phase = 'fly'
        self._publish_path()

    def _advance(self):
        if self.recovering:
            self.recovering = False
            self._select_target()
            return
        if self.target_is_vp:
            self.visited.append(self.target.copy())
            self.get_logger().info(f'viewpoint {len(self.visited)} visited at {self.target.tolist()}')
        else:
            self.leg += 1
        if self.leg >= len(self.waypoints) - 1:
            self.phase = 'done'
            self.get_logger().info(f'AVP sweep complete: {len(self.visited)} viewpoints. Holding.')
        else:
            self._select_target()

    def tick(self):
        now = self._now()
        dt = min(0.05, now - self.last_tick_t) if self.last_tick_t is not None else 1.0 / self.rate
        self.last_tick_t = now
        odom_fresh = self.last_odom_t is not None and now - self.last_odom_t <= self.odom_timeout
        if not odom_fresh or not self._control_ready(now):
            if self.phase != 'wait' and self.paused_at is None:
                self.paused_at = now
                self.v = 0.0
                self._warn('AVP paused: odometry, DDS/Offboard, or initial pose not ready')
            return
        if self.paused_at is not None:
            if self.phase == 'dwell':
                self.dwell_t0 += now - self.paused_at
            self.paused_at = None
            self.last_pitch_t = now
            self.v = 0.0
            if self.phase in ('fly', 'dwell') and np.linalg.norm(self.pos_enu - self.cur_cmd_pos) > self.tracking_tol:
                self.phase = 'fly'
                self.start = self.pos_enu.copy()
                self.cur_cmd_pos = self.pos_enu.copy()
                self.pitch_start = self.cur_cmd_pitch
                self.length = float(np.linalg.norm(self.target - self.start))
                self.s = 0.0
                self._publish_path()

        if self.phase == 'wait':
            if self.t_first_odom is None:
                self.t_first_odom = now
            if not self.latest and now - self.t_first_odom < self.wait_vp_s:
                return
            if self.latest is None:
                self.latest = []
                self._warn('no viewpoints; starting straight sweep')
            if not self._inside(self.pos_enu) or self._occupied(self.pos_enu):
                self._warn('AVP start outside safe corridor; holding initial pose', error=True)
                return
            self.cur_cmd_pos = self.pos_enu.copy()
            self.cur_cmd_pitch = self.cam_pitch
            self.leg = -1
            self._select_target()
            return

        if self.phase == 'done':
            self._publish(self.cur_cmd_pos, self.cur_cmd_pitch)
            return

        if self.phase == 'dwell':
            if not self._publish(self.target, self.pitch_end):
                if self.target_is_vp:
                    self._recover_from_vp()
                return
            if np.linalg.norm(self.pos_enu - self.target) > self.arrival_tol:
                self.dwell_t0 = now
                return
            if now - self.dwell_t0 >= self.hold_s:
                self._advance()
            return

        if not self._inside(self.pos_enu):
            self.v = 0.0
            self._warn('vehicle outside straight AVP corridor; holding target', error=True)
            self._publish(self.cur_cmd_pos, self.cur_cmd_pitch)
            return
        if np.linalg.norm(self.pos_enu - self.cur_cmd_pos) > self.tracking_tol:
            self.v = 0.0
            self._publish(self.cur_cmd_pos, self.cur_cmd_pitch)
            return
        if self.batch_dirty and not self.target_is_vp and now - self.last_replan_t > self.replan_min:
            self.batch_dirty = False
            self.last_replan_t = now
            if self._next_vp() is not None:
                self._select_target()

        remain = max(0.0, self.length - self.s)
        desired_v = min(self.vc, math.sqrt(2.0 * self.acc * remain))
        self.v = max(0.0, min(desired_v, self.v + self.acc * dt))
        next_s = min(self.length, self.s + self.v * dt)
        u = next_s / self.length if self.length > 1e-6 else 1.0
        pos = self.start + (self.target - self.start) * u
        blend = u * u * (3.0 - 2.0 * u)
        pitch = self.pitch_start + (self.pitch_end - self.pitch_start) * blend
        if not self._publish(pos, pitch):
            self.v = 0.0
            if self.target_is_vp:
                self._recover_from_vp()
            return
        self.s = next_s

        if self.length - self.s <= 1e-3 and np.linalg.norm(self.pos_enu - self.target) <= self.arrival_tol:
            self.hold_s = self.dwell_s if self.target_is_vp else 0.0
            if (not self.target_is_vp and self.leg >= 0 and self.leg + 2 < len(self.waypoints)
                    and abs(self.waypoints[self.leg + 2][2] - self.waypoints[self.leg + 1][2]) > 1e-6):
                self.hold_s = self.level_hold_s
            if self.hold_s > 0.0:
                self.phase = 'dwell'
                self.dwell_t0 = now
            else:
                self._advance()

    def _base_point(self, p):
        if self.leg < 0:
            return self.waypoints[0]
        if self.leg >= len(self.waypoints) - 1:
            return self.waypoints[-1]
        start, end = self.waypoints[self.leg:self.leg + 2]
        if abs(end[1] - start[1]) > 1e-6:
            return np.array([start[0], float(np.clip(p[1], min(start[1], end[1]), max(start[1], end[1]))), start[2]])
        return np.array([start[0], start[1], float(np.clip(p[2], min(start[2], end[2]), max(start[2], end[2])))])

    def _publish(self, p, pitch):
        if not self._inside(p):
            self._warn('AVP setpoint outside corridor; holding', error=True)
            return False
        if self._occupied(p) and not (self.recovering and
                self._nearest_occ_sq(p) > self._nearest_occ_sq(self.cur_cmd_pos) + 1e-7):
            self._warn('AVP setpoint blocked by obstacle; holding', error=True)
            return False
        now = self._now()
        if self.pitch_rate > 0.0:
            dt = 1.0 / self.rate if self.last_pitch_t is None else max(0.0, min(now - self.last_pitch_t, 0.05))
            pitch = self.cur_cmd_pitch + float(np.clip(pitch - self.cur_cmd_pitch,
                                                        -self.pitch_rate * dt, self.pitch_rate * dt))
        self.last_pitch_t = now
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, p)
        w, x, y, z = rpy_to_quat(0.0, float(pitch), 0.0)  # ENU +X faces the tree row.
        msg.pose.orientation.w, msg.pose.orientation.x = w, x
        msg.pose.orientation.y, msg.pose.orientation.z = y, z
        self.sp_pub.publish(msg)

        if now - self.last_base_t >= 0.1:
            base = self._base_point(p)
            ref = PoseStamped(); ref.header = msg.header
            ref.pose.position.x, ref.pose.position.y, ref.pose.position.z = map(float, base)
            w, x, y, z = rpy_to_quat(0.0, self.cam_pitch, 0.0)
            ref.pose.orientation.w, ref.pose.orientation.x = w, x
            ref.pose.orientation.y, ref.pose.orientation.z = y, z
            self.base_pub.publish(ref)
            self.last_base_t = now
        self.cur_cmd_pos = np.array(p, dtype=float)
        self.cur_cmd_pitch = float(pitch)
        if now - self.last_trail_t >= 0.25:
            self.last_trail_t = now
            self.trail.append(self.cur_cmd_pos.copy())
            if len(self.trail) > 2000:
                self.trail.pop(0)
            self._publish_trail()
        return True

    def _publish_path(self):
        ma = MarkerArray()
        clear = Marker(); clear.action = Marker.DELETEALL; ma.markers.append(clear)
        stamp = self.get_clock().now().to_msg()
        for mid, (name, points, color) in enumerate((
                ('base_sweep', self.waypoints, (1.0, 0.65, 0.0)),
                ('corridor', (self.start, self.target), (0.5, 0.3, 1.0)))):
            m = Marker(); m.header.frame_id = 'map'; m.header.stamp = stamp
            m.ns = name; m.id = mid; m.type = Marker.LINE_STRIP; m.action = Marker.ADD
            m.scale.x = 0.05
            m.color.r, m.color.g, m.color.b, m.color.a = (*color, 0.95)
            m.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in points]
            ma.markers.append(m)
        if self.latest:
            m = Marker(); m.header.frame_id = 'map'; m.header.stamp = stamp
            m.ns = 'anchors'; m.id = 3; m.type = Marker.SPHERE_LIST; m.action = Marker.ADD
            m.scale.x = m.scale.y = m.scale.z = 0.15
            for p, _ in self.latest:
                if any(np.linalg.norm(p - visited) < self.vp_merge_m for visited in self.visited):
                    continue
                m.points.append(Point(x=float(p[0]), y=float(p[1]), z=float(p[2])))
                c = ColorRGBA(); c.r, c.g, c.b, c.a = 1.0, 0.55, 0.0, 1.0
                m.colors.append(c)
            ma.markers.append(m)
        self.path_pub.publish(ma)

    def _publish_safety(self):
        m = Marker(); m.header.frame_id = 'map'; m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'straight_bounds'; m.id = 1; m.type = Marker.LINE_LIST; m.action = Marker.ADD
        m.scale.x = 0.02
        m.color.r, m.color.g, m.color.b, m.color.a = (0.3, 0.7, 1.0, 0.7)
        corners = [np.array([x, y, z]) for z in (self.zmin, self.zmax)
                   for y in (self.ymin, self.ymax) for x in (self.xmin, self.xmax)]
        for i, a in enumerate(corners):
            for bit in (1, 2, 4):
                j = i ^ bit
                if i < j:
                    b = corners[j]
                    m.points.extend((Point(x=float(a[0]), y=float(a[1]), z=float(a[2])),
                                     Point(x=float(b[0]), y=float(b[1]), z=float(b[2]))))
        ma = MarkerArray(); ma.markers.append(m)
        self.safety_pub.publish(ma)

    def _publish_trail(self):
        m = Marker(); m.header.frame_id = 'map'; m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'trail'; m.id = 1; m.type = Marker.LINE_STRIP; m.action = Marker.ADD
        m.scale.x = 0.035
        m.color.r, m.color.g, m.color.b, m.color.a = (1.0, 0.85, 0.1, 0.9)
        m.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in self.trail]
        ma = MarkerArray(); ma.markers.append(m)
        self.trail_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = CorridorExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
