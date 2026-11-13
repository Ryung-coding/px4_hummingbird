#!/usr/bin/env python3
"""
lics_corridor_executor.py — 부채꼴 코리더 왕복 스윕 실행기 (HummingBird / path:=ext)
------------------------------------------------------------------------------
v3 (실내 조경수 환경):
  - 비행 공간: 나무 중심 기준 **부채꼴** 실린더  theta∈[theta_min, theta_max], r∈[r_min,r_max], z∈[z_min,z_max]
    (벽에 막힌 각도는 제외. theta_max-theta_min >= 360 이면 기존 환형 동작)
  - 진행: theta_min -> theta_max 스윕 후 끝에서 방향 반전 (passes 회 왕복). 패스마다 기본 고도를
    z_levels 로 바꿔가며 훑음 (예: 1.0,2.2 → 1패스 낮게, 2패스 높게)
  - 시점: /avp/viewpoints_data(ENU PoseArray, 방향에 pitch 포함) 를 코리더로 클램프, 부채꼴 밖은 버림,
    진행방향 앞의 것만 앵커로 방문(dwell). 시점의 **pitch 를 기체 자세로 그대로 명령** (fully-actuated)
  - 평상시 pitch = cam_pitch_deg, 앵커 dwell 중 pitch = 시점 pitch (ROLL/PITCH 한계는 ext 노드가 클램프)
  - yaw: 항상 center-facing
  - 입력 /lics/odom (ENU), 출력 /avp/setpoint (PoseStamped, map/ENU)

실행 (KI 실내 월드 lics_ki_indoor.sdf 기준):
  python3 ~/lics_avp/lics_corridor_executor.py --ros-args -p use_sim_time:=true \\
    -p center_x:=0.0 -p center_y:=0.0 -p theta_min_deg:=-150 -p theta_max_deg:=60 \\
    -p r_min:=2.2 -p r_max:=3.0 -p z_min:=0.8 -p z_max:=2.8 -p z_levels:="[1.2, 2.4]" -p passes:=2
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy,
                       QoSDurabilityPolicy, QoSHistoryPolicy)

from geometry_msgs.msg import PoseStamped, PoseArray, Point
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleStatus, HummingbirdStatus
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA


def wrap_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def rpy_to_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll/2), math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
    return (cr*cp*cy + sr*sp*sy, sr*cp*cy - cr*sp*sy,
            cr*sp*cy + sr*cp*sy, cr*cp*sy - sr*sp*cy)


def quat_to_pitch(w, x, y, z):
    sinp = 2.0 * (w*y - z*x)
    return math.copysign(math.pi/2, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)


def catmull_chain(way, n_per_seg=8):
    W = np.asarray(way, dtype=float)
    if len(W) < 3:
        return W.copy()
    P = np.vstack([W[0], W, W[-1]])
    out = [W[0]]
    for i in range(len(W) - 1):
        p0, p1, p2, p3 = P[i], P[i+1], P[i+2], P[i+3]
        for j in range(1, n_per_seg + 1):
            t = j / n_per_seg
            t2, t3 = t*t, t*t*t
            out.append(0.5*((2*p1) + (-p0+p2)*t
                       + (2*p0-5*p1+4*p2-p3)*t2
                       + (-p0+3*p1-3*p2+p3)*t3))
    return np.array(out)


class CorridorExecutor(Node):
    def __init__(self):
        super().__init__('corridor_executor')

        from rclpy.parameter import Parameter
        from rcl_interfaces.msg import ParameterDescriptor
        def dp(name, default):
            # 숫자 파라미터는 정수/실수 둘 다 허용 (135 vs 135.0)
            if isinstance(default, float):
                self.declare_parameter(name, default, ParameterDescriptor(dynamic_typing=True))
            else:
                self.declare_parameter(name, default)
        dp('center_x', 0.0); dp('center_y', 0.0)           # ENU
        dp('theta_min_deg', -180.0); dp('theta_max_deg', 180.0)
        dp('r_min', 2.2); dp('r_max', 3.0)
        dp('z_min', 0.8); dp('z_max', 2.8)
        dp('z_levels', [1.2, 2.4])                          # 패스별 기본 고도
        dp('passes', 2)
        dp('cruise_speed', 0.4); dp('accel', 0.3)
        dp('dwell_s', 1.5)
        dp('publish_rate_hz', 100.0)
        dp('arc_sample_deg', 10.0)
        dp('vp_merge_deg', 8.0)
        dp('guard_deg', 15.0)
        dp('replan_min_s', 1.5)
        dp('cam_pitch_deg', 0.0)                            # 순항 중 기체 pitch (+ = 기수 하향)
        dp('use_vp_pitch', True)                            # dwell 시 시점 pitch 사용
        dp('setpoint_topic', '/avp/setpoint'); dp('odom_topic', '/lics/odom')
        dp('wait_vp_s', 5.0)                               # 시점 없이도 이 시간 후 출발 (코리더 순항으로 데이터 수집)
        # [LICS] 맵 기반 안전
        dp('adaptive_corridor', True)                      # /avp/corridor_profile 로 각도별 r_min 자동 확장
        dp('obs_margin', 0.5)                              # 관측된 수관 반경 + 이 값 = 그 각도의 r_min
        dp('clear_m', 0.5)                                 # setpoint 주변 이 반경 안에 OCC 있으면 바깥으로 밀어냄
        dp('arrival_tol_m', 0.25)
        dp('tracking_tol_m', 0.75)
        dp('odom_timeout_s', 0.5)
        dp('require_control', True)

        gp = lambda n: self.get_parameter(n).value
        self.c = np.array([float(gp('center_x')), float(gp('center_y'))])
        self.th_min = math.radians(float(gp('theta_min_deg')))
        self.th_max = math.radians(float(gp('theta_max_deg')))
        if self.th_max <= self.th_min:
            self.th_max += 2.0*math.pi
        self.ring = (self.th_max - self.th_min) >= 2.0*math.pi - 1e-6
        self.rmin = float(gp('r_min')); self.rmax = float(gp('r_max'))
        self.zmin = float(gp('z_min')); self.zmax = float(gp('z_max'))
        self.z_levels = [float(z) for z in gp('z_levels')] or [0.5*(self.zmin+self.zmax)]
        self.passes = int(gp('passes'))
        self.vc = float(gp('cruise_speed')); self.acc = float(gp('accel'))
        self.dwell_s = float(gp('dwell_s'))
        self.rate = float(gp('publish_rate_hz'))
        self.d_arc = math.radians(float(gp('arc_sample_deg')))
        self.d_merge = math.radians(float(gp('vp_merge_deg')))
        self.guard = math.radians(float(gp('guard_deg')))
        self.replan_min = float(gp('replan_min_s'))
        self.cam_pitch = math.radians(float(gp('cam_pitch_deg')))
        self.use_vp_pitch = bool(gp('use_vp_pitch'))
        self.last_replan_t = -1e9
        self.rmid = 0.5*(self.rmin+self.rmax)
        self.wait_vp_s = float(gp('wait_vp_s'))
        self.t_first_odom = None
        self.adaptive = bool(gp('adaptive_corridor'))
        self.obs_margin = float(gp('obs_margin'))
        self.clear_m = float(gp('clear_m'))
        self.arrival_tol = float(gp('arrival_tol_m'))
        self.tracking_tol = float(gp('tracking_tol_m'))
        self.odom_timeout = float(gp('odom_timeout_s'))
        self.require_control = bool(gp('require_control'))
        self.profile = None                # [[r_obs per bin] per z-band]
        self.prof_z0 = 0.0; self.prof_dz = 0.5; self.prof_rmax_all = 0.0
        self.occ = set()                   # OCC 복셀 키 (0.1 m 격자)
        self.occ_res = 0.1
        rc = int(math.ceil(self.clear_m / self.occ_res))
        self._clear_off = [(i, j, k) for i in range(-rc, rc+1) for j in range(-rc, rc+1)
                           for k in range(-rc, rc+1)
                           if (i*i + j*j + k*k) * self.occ_res**2 <= self.clear_m**2]
        self._blocked_warn_t = -1e9
        self._push_f = 0.0                 # 밀어내기 반경 필터 상태 (m)
        self._push_tau = 0.8

        # 상태
        self.phase = 'wait'                # wait | fly | dwell | done
        self.pos_enu = None
        self.last_odom_t = None
        self.vehicle_status = None
        self.last_status_t = None
        self.hb_status = None
        self.last_hb_t = None
        self.latest = None                 # [(theta, r, z, pitch)]  theta ∈ [th_min, th_max]
        self.batch_dirty = False
        self.pass_i = 0
        self.lead_in = False               # 시작 위치→가까운 끝 진입 구간 (패스 수에 미포함)
        self.dir = +1                      # +1: th_min→th_max, -1: 반대
        self.theta_cur = None; self.theta_end = None
        self.r_cur = None; self.z_cur = None
        self.anchors = []                  # 진행방향 기준 정렬 [(theta, r, z, pitch)]
        self.S = None; self.cum = None; self.L = 0.0
        self.s = 0.0; self.v = 0.0
        self.seg_is_vp = False; self.seg_tgt = None
        self.dwell_t0 = None
        self.cur_cmd_pos = None; self.cur_cmd_yaw = 0.0; self.cur_cmd_pitch = 0.0
        self.last_tick_t = None
        self.visited_n = 0
        self.trail = []; self.trail_last_t = -1e9

        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         durability=QoSDurabilityPolicy.VOLATILE,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Odometry, str(gp('odom_topic')), self.odom_cb, qos)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.status_cb, qos)
        self.create_subscription(HummingbirdStatus, '/fmu/out/hummingbird_status', self.hb_cb, qos)
        self.create_subscription(PoseArray, '/avp/viewpoints_data', self.vp_cb, 1)
        self.create_subscription(Float32MultiArray, '/avp/corridor_profile', self.profile_cb, 1)
        self.create_subscription(PointCloud2, '/avp/occ_voxels', self.occ_cb, 1)
        self.sp_pub = self.create_publisher(PoseStamped, str(gp('setpoint_topic')), 10)
        self.path_pub = self.create_publisher(MarkerArray, '/avp/path', 1)
        self.trail_pub = self.create_publisher(MarkerArray, '/avp/trail', 1)
        self.safety_pub = self.create_publisher(MarkerArray, '/avp/safety_region', 1)
        self.create_timer(1.0, self._publish_safety)
        self.create_timer(1.0/self.rate, self.tick)
        self.get_logger().info(
            f'corridor_executor v3 ready. center=ENU({self.c[0]},{self.c[1]}) '
            f'theta=[{math.degrees(self.th_min):.0f},{math.degrees(self.th_max):.0f}]deg '
            f'{"(ring)" if self.ring else "(sector)"} r=[{self.rmin},{self.rmax}] '
            f'z=[{self.zmin},{self.zmax}] levels={self.z_levels} passes={self.passes}')

    # ---------------- 입력 ----------------

    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self.pos_enu = np.array([float(p.x), float(p.y), float(p.z)])
        self.last_odom_t = self._now()

    def status_cb(self, msg: VehicleStatus):
        self.vehicle_status = msg
        self.last_status_t = self._now()

    def hb_cb(self, msg: HummingbirdStatus):
        self.hb_status = msg
        self.last_hb_t = self._now()

    def profile_cb(self, msg: Float32MultiArray):
        if not self.adaptive or len(msg.data) < 4:
            return
        d = list(msg.data)
        nb = int(d[0]); nz = int(d[1]); self.prof_z0 = d[2]; self.prof_dz = d[3]
        vals = d[4:4 + nb * nz]
        if len(vals) < nb * nz:
            return
        rows = [vals[k*nb:(k+1)*nb] for k in range(nz)]
        # 각 띠: 이웃 빈과 max 스무딩. 그리고 "아래 띠들의 누적 max" 도 반영 (위는 못 봐도 아래보다 좁다고 가정하지 않음)
        sm = [[max(r[(i-1) % nb], r[i], r[(i+1) % nb]) for i in range(nb)] for r in rows]
        cum = []
        running = [0.0] * nb
        for r in sm:
            running = [max(a, b) for a, b in zip(running, r)]
            cum.append(list(running))
        self.profile = cum
        self.prof_rmax_all = max((v for r in sm for v in r), default=0.0)

    def occ_cb(self, msg: PointCloud2):
        n = msg.width * msg.height
        if n == 0:
            self.occ = set(); return
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, msg.point_step)[:, :12].copy().view(np.float32).reshape(-1, 3)
        keys = np.floor(arr / self.occ_res).astype(np.int64)
        self.occ = set(map(tuple, keys.tolist()))

    def _rmin_at(self, theta, z=None):
        """각도·고도별 유효 r_min.
        - 그 고도 띠까지 관측된 최대 반경(아래 띠 포함 누적) + obs_margin
        - 그 각도가 아직 미관측이면: 전체에서 본 최대 반경 + obs_margin (보수적)"""
        if self.profile is None:
            return self.rmin
        nb = len(self.profile[0]); nz = len(self.profile)
        if z is None:
            z = self.z_cur if self.z_cur is not None else self.zmin
        # 각도: 빈 중심 기준 선형 보간 (미관측 빈은 전체 최대로 대체 후 보간)
        u = ((theta + 2*math.pi) % (2*math.pi)) / (2*math.pi) * nb - 0.5
        b0 = int(math.floor(u)) % nb; b1 = (b0 + 1) % nb; fb = u - math.floor(u)
        # 고도: 띠 중심 기준 선형 보간
        v = (z - self.prof_z0) / self.prof_dz - 0.5 if self.prof_dz > 0 else 0.0
        z0 = max(0, min(int(math.floor(v)), nz - 1)); z1 = max(0, min(z0 + 1, nz - 1))
        fz = min(1.0, max(0.0, v - math.floor(v))) if nz > 1 else 0.0
        def val(zi, b):
            r = self.profile[zi][b]
            return r if r > 0.0 else self.prof_rmax_all
        r00, r01, r10, r11 = val(z0, b0), val(z0, b1), val(z1, b0), val(z1, b1)
        ro = (1-fz)*((1-fb)*r00 + fb*r01) + fz*((1-fb)*r10 + fb*r11)
        return max(self.rmin, ro + self.obs_margin) if ro > 0.0 else self.rmin

    def _occ_near(self, pos):
        if not self.occ:
            return False
        c = np.floor(pos / self.occ_res).astype(np.int64)
        for o in self._clear_off:
            if (int(c[0]+o[0]), int(c[1]+o[1]), int(c[2]+o[2])) in self.occ:
                return True
        return False

    def _theta_in_sector(self, th):
        """th 를 [th_min, th_max] 구간으로 unwrap. 구간 밖이면 None."""
        u = self.th_min + ((th - self.th_min) % (2.0*math.pi))
        return u if u <= self.th_max + 1e-9 else None

    def vp_cb(self, msg: PoseArray):
        vps = []
        for po in msg.poses:
            e = np.array([po.position.x, po.position.y, po.position.z])
            d = e[:2] - self.c
            th = self._theta_in_sector(math.atan2(d[1], d[0]))
            if th is None:
                continue                                  # 벽 뒤 시점 버림
            min_r = self._rmin_at(th, float(e[2]))
            if min_r > self.rmax:
                continue
            q = po.orientation
            pitch = quat_to_pitch(q.w, q.x, q.y, q.z)     # 플래너 look 방향 (+ = 아래)
            vps.append((th,
                        float(np.clip(np.linalg.norm(d), min_r, self.rmax)),
                        float(np.clip(e[2], self.zmin, self.zmax)),
                        float(pitch)))
        self.latest = vps
        self.batch_dirty = True

    # ---------------- 코리더 기하 ----------------

    def _pt(self, theta, r, z):
        r = max(r, self._rmin_at(theta, z))
        return np.array([self.c[0] + r*math.cos(theta),
                         self.c[1] + r*math.sin(theta), z])

    def _ahead(self, th):
        """진행방향 기준 theta_cur 로부터 얼마나 앞인지 (양수=앞)."""
        return (th - self.theta_cur) * self.dir

    def _rebuild_anchors(self):
        if self.latest is None:
            self.anchors = []; return
        out = []
        for th, r, z, pitch in self.latest:
            if self.ring:
                th = self.theta_cur + self.dir * (((th - self.theta_cur) * self.dir) % (2.0 * math.pi))
            a = self._ahead(th)
            if a < self.guard:
                continue
            if self._ahead(self.theta_end) - a < -1e-6:
                continue
            out.append((th, r, z, pitch))
        out.sort(key=lambda t: self._ahead(t[0]))
        merged = []
        for a in out:
            if merged and abs(a[0] - merged[-1][0]) < self.d_merge:
                continue
            merged.append(a)
        self.anchors = merged

    def _build_segment(self):
        if self.anchors:
            tgt_th, tgt_r, tgt_z, _ = self.anchors[0]
            self.seg_is_vp = True
        else:
            tgt_th, tgt_r, tgt_z = self.theta_end, self.r_cur, self.z_levels[min(self.pass_i, len(self.z_levels)-1)]
            self.seg_is_vp = False
        n = max(2, int(abs(tgt_th - self.theta_cur) / self.d_arc))
        way = []
        for i in range(n + 1):
            f = i / n
            way.append(self._pt(self.theta_cur + (tgt_th - self.theta_cur)*f,
                                self.r_cur + (tgt_r - self.r_cur)*f,
                                self.z_cur + (tgt_z - self.z_cur)*f))
        seg_len = sum(np.linalg.norm(way[i+1]-way[i]) for i in range(len(way)-1))
        if self.v > 0.05 and seg_len > 1.0 and self.S is not None and len(self.S) >= 2:
            i = int(np.searchsorted(self.cum, self.s, side='right')) - 1
            i = max(0, min(i, len(self.S) - 2))
            d = self.S[i+1] - self.S[i]; n_ = np.linalg.norm(d)
            if n_ > 1e-9:
                way = [way[0], way[0] + d/n_*max(0.3, self.v*0.8)] + way[1:]
        S = catmull_chain(way)
        seg = np.linalg.norm(np.diff(S, axis=0), axis=1)
        cum = np.concatenate([[0.0], np.cumsum(seg)])
        self.S, self.cum, self.L = S, cum, float(cum[-1])
        self.s = 0.0
        self.seg_tgt = (tgt_th, tgt_r, tgt_z)
        self._publish_path()

    # ---------------- 메인 루프 ----------------

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _start_pass(self, from_theta):
        if self.ring:
            self.theta_end = from_theta + 2.0*math.pi*self.dir
        else:
            self.theta_end = self.th_max if self.dir > 0 else self.th_min
        self.theta_cur = from_theta
        self.last_replan_t = -1e9
        self._rebuild_anchors()
        self._build_segment()
        self.phase = 'fly'
        tag = 'lead-in' if self.lead_in else f'pass {self.pass_i+1}/{self.passes}'
        self.get_logger().info(
            f'{tag} start: dir={"+" if self.dir>0 else "-"} '
            f'theta {math.degrees(self.theta_cur):.0f}->{math.degrees(self.theta_end):.0f}deg '
            f'z_level={self.z_levels[min(self.pass_i, len(self.z_levels)-1)]} anchors={len(self.anchors)}')

    def tick(self):
        now = self._now()
        dt = (now - self.last_tick_t) if self.last_tick_t else 1.0/self.rate
        dt = min(dt, 0.05)
        self.last_tick_t = now

        odom_fresh = self.last_odom_t is not None and now - self.last_odom_t <= self.odom_timeout
        control_ready = (not self.require_control or (
            self.vehicle_status is not None and self.last_status_t is not None
            and now - self.last_status_t <= 1.0
            and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
            and self.hb_status is not None and self.last_hb_t is not None
            and now - self.last_hb_t <= 1.0 and self.hb_status.dds_command_enabled))
        if not odom_fresh or not control_ready:
            if self.phase != 'wait':
                self.phase = 'wait'
                self.v = 0.0
                self.s = 0.0
                self.pass_i = 0
                self.lead_in = False
                self.t_first_odom = None
                self.cur_cmd_pos = None
                self.get_logger().warn('AVP paused: odometry or DDS/Offboard control unavailable')
            return

        if self.phase == 'wait':
            if self.pos_enu is None:
                return
            if self.t_first_odom is None:
                self.t_first_odom = now
            if self.latest is None:
                if now - self.t_first_odom < self.wait_vp_s:
                    return
                self.get_logger().warn(f'no viewpoints after {self.wait_vp_s:.0f}s — starting sweep without anchors')
                self.latest = []
            d = self.pos_enu[:2] - self.c
            th = self._theta_in_sector(math.atan2(d[1], d[0]))
            if th is None:
                # 부채꼴 밖에서 시작 → 가까운 끝으로 스냅
                th = self.th_min if abs(wrap_pi(math.atan2(d[1], d[0]) - self.th_min)) < \
                     abs(wrap_pi(math.atan2(d[1], d[0]) - self.th_max)) else self.th_max
            self.r_cur = float(np.clip(np.linalg.norm(d), self.rmin, self.rmax))
            self.z_cur = float(np.clip(self.pos_enu[2], self.zmin, self.zmax))
            if self.ring or abs(th - self.th_min) < 1e-3 or abs(th - self.th_max) < 1e-3:
                self.dir = +1 if (th - self.th_min) <= (self.th_max - th) else -1
                self._start_pass(th)
            else:
                # 가까운 끝으로 먼저 이동 (lead-in), 도착 후 반대쪽으로 전체 패스 시작
                self.dir = -1 if (th - self.th_min) <= (self.th_max - th) else +1
                self.lead_in = True
                self._start_pass(th)
            return

        if self.phase == 'done':
            if self.cur_cmd_pos is not None:
                self._publish(self.cur_cmd_pos, self.cur_cmd_yaw, self.cur_cmd_pitch)
            return

        if self.phase == 'dwell':
            pitch = self.anchors[0][3] if (self.use_vp_pitch and self.anchors) else self.cam_pitch
            self._publish(self.S[-1], self._face_center_yaw(self.S[-1]), pitch)
            if now - self.dwell_t0 >= self.dwell_s:
                if self.batch_dirty:
                    self.batch_dirty = False
                    self._rebuild_anchors()
                elif self.anchors:
                    self.anchors.pop(0)
                self._advance_or_finish()
            return

        if self.phase == 'fly':
            if self.cur_cmd_pos is not None and np.linalg.norm(self.pos_enu - self.cur_cmd_pos) > self.tracking_tol:
                self.v = 0.0
                self._publish(self.cur_cmd_pos, self.cur_cmd_yaw, self.cur_cmd_pitch)
                return
            if self.batch_dirty and (now - self.last_replan_t) > self.replan_min:
                self.batch_dirty = False; self.last_replan_t = now
                pos = self._sample_at(self.s)
                d = pos[:2] - self.c
                th = self._theta_in_sector(math.atan2(d[1], d[0]))
                if th is not None and self._ahead(th) > 0:
                    self.theta_cur = th
                self.r_cur = float(np.clip(np.linalg.norm(d), self.rmin, self.rmax))
                self.z_cur = float(np.clip(pos[2], self.zmin, self.zmax))
                old_anchors = [(round(a[0],3), round(a[1],2), round(a[2],2)) for a in self.anchors]
                self._rebuild_anchors()
                new_anchors = [(round(a[0],3), round(a[1],2), round(a[2],2)) for a in self.anchors]
                if new_anchors != old_anchors:
                    self._build_segment()
                    self.get_logger().info(
                        f'  replan: {len(self.anchors)} anchors ahead, next @ {math.degrees(self.seg_tgt[0]):.0f}deg')

            remain = self.L - self.s
            stop_dist = self.v*self.v/(2.0*self.acc)
            decel = self.seg_is_vp or (remain < 1.0 and not self.anchors)
            if decel and remain <= stop_dist + 1e-3:
                self.v = max(0.0, self.v - self.acc*dt)
            elif self.v < self.vc:
                self.v = min(self.vc, self.v + self.acc*dt)
            next_s = min(self.L, self.s + self.v*dt)
            pos = self._sample_at(next_s)
            if not self._publish(pos, self._face_center_yaw(pos), self.cam_pitch):
                self.v = 0.0
                return
            self.s = next_s

            arrived = np.linalg.norm(self.pos_enu - self.cur_cmd_pos) <= self.arrival_tol
            if arrived and ((self.s >= self.L - 1e-3 and (not decel or self.v <= 0.02))
                            or (remain < 0.05 and self.v <= 0.05)):
                self.s = self.L
                self.theta_cur, self.r_cur, self.z_cur = self.seg_tgt
                if self.seg_is_vp:
                    self.visited_n += 1
                    self.phase = 'dwell'; self.dwell_t0 = now
                    self.get_logger().info(
                        f'vp #{self.visited_n} @ {math.degrees(self.theta_cur):.0f}deg r={self.r_cur:.1f} '
                        f'z={self.z_cur:.1f} pitch={math.degrees(self.anchors[0][3]) if self.anchors else 0:.0f}deg '
                        f'({max(0,len(self.anchors)-1)} ahead)')
                else:
                    self._advance_or_finish()
            return

    def _advance_or_finish(self):
        if abs(self.theta_cur - self.theta_end) < 1e-6 or self._ahead(self.theta_end) <= 1e-6:
            if self.lead_in:
                self.lead_in = False
                self.dir = -self.dir
                self._start_pass(self.theta_cur)
                return
            self.pass_i += 1
            if self.pass_i >= self.passes:
                self.phase = 'done'
                self.get_logger().info(
                    f'sweep complete: {self.visited_n} viewpoints over {self.passes} pass(es). Holding.')
                return
            if not self.ring:
                self.dir = -self.dir
            self._start_pass(self.theta_cur)
            return
        self._build_segment()
        self.phase = 'fly'

    def _sample_at(self, arc):
        i = int(np.searchsorted(self.cum, arc, side='right')) - 1
        i = max(0, min(i, len(self.S) - 2))
        seg = self.cum[i+1] - self.cum[i]
        f = 0.0 if seg < 1e-9 else (arc - self.cum[i]) / seg
        return self.S[i] + (self.S[i+1] - self.S[i]) * f

    def _face_center_yaw(self, pos_enu):
        return math.atan2(self.c[1] - pos_enu[1], self.c[0] - pos_enu[0])

    def _clamp_corridor(self, pos_enu):
        """스플라인 보간이 r_min 안쪽/ r_max 바깥으로 파고드는 것을 막는 최종 안전 클램프."""
        d = pos_enu[:2] - self.c
        r = float(np.linalg.norm(d))
        if r < 1e-6:
            return None
        th = math.atan2(d[1], d[0])
        zc = min(max(float(pos_enu[2]), self.zmin), self.zmax)
        min_r = self._rmin_at(th, zc)
        if min_r > self.rmax:
            t_ = self._now()
            if t_ - self._blocked_warn_t > 2.0:
                self._blocked_warn_t = t_
                self.get_logger().error('AVP corridor blocked: required clearance exceeds r_max')
            return None
        rc = min(max(r, min_r), self.rmax)
        out = np.array([self.c[0] + d[0]/r*rc, self.c[1] + d[1]/r*rc, zc])
        # OCC 복셀이 가까우면 r_max 안에서만 반경 방향으로 밀어냄.
        # 밀어내기량은 1차 필터(tau)로 완만하게 → 톱니 제거
        push_target = 0.0
        if self.occ:
            u = d / r
            rr = rc
            probe = out
            while self._occ_near(probe) and rr < self.rmax:
                rr = min(rr + 0.1, self.rmax)
                probe = np.array([self.c[0] + u[0]*rr, self.c[1] + u[1]*rr, zc])
            if self._occ_near(probe):
                t_ = self._now()
                if t_ - self._blocked_warn_t > 2.0:
                    self._blocked_warn_t = t_
                    self.get_logger().warn(f'setpoint blocked by obstacle @ {math.degrees(th):.0f}deg — holding last safe target')
                return None
            push_target = rr - rc
        dt = 1.0 / self.rate
        a = dt / (self._push_tau + dt)
        # 늘어날 땐 최대 1.0 m/s 로(안전 우선, 그래도 연속), 줄어들 땐 필터로 천천히
        if push_target > self._push_f:
            self._push_f = min(push_target, self._push_f + 1.0 * dt)
        else:
            self._push_f += a * (push_target - self._push_f)
        if self._push_f > 1e-3:
            u = d / r
            rr = min(rc + self._push_f, self.rmax)
            out = np.array([self.c[0] + u[0]*rr, self.c[1] + u[1]*rr, zc])
        if self._occ_near(out):
            return None
        return out

    def _publish(self, pos_enu, yaw_enu, pitch):
        pos_enu = self._clamp_corridor(np.asarray(pos_enu, dtype=float))
        if pos_enu is None:
            return False
        pm = PoseStamped()
        pm.header.stamp = self.get_clock().now().to_msg()
        pm.header.frame_id = 'map'
        pm.pose.position.x, pm.pose.position.y, pm.pose.position.z = map(float, pos_enu)
        w, x, y, z = rpy_to_quat(0.0, float(pitch), yaw_enu)
        pm.pose.orientation.w, pm.pose.orientation.x = w, x
        pm.pose.orientation.y, pm.pose.orientation.z = y, z
        self.sp_pub.publish(pm)
        self.cur_cmd_pos = np.array(pos_enu, dtype=float)
        self.cur_cmd_yaw = float(yaw_enu); self.cur_cmd_pitch = float(pitch)
        t_ = self._now()
        if t_ - self.trail_last_t > 0.25:
            self.trail_last_t = t_
            self.trail.append(self.cur_cmd_pos.copy())
            if len(self.trail) > 2000:
                self.trail.pop(0)
            self._publish_trail()
        return True

    # ---------------- 시각화 ----------------

    def _publish_path(self):
        ma = MarkerArray()
        clr = Marker(); clr.action = Marker.DELETEALL; ma.markers.append(clr)
        stamp = self.get_clock().now().to_msg()
        # 코리더 경계 (r_min/r_max 호, 두 고도)
        for mid, (rr, zz) in enumerate([(self.rmin, self.zmin), (self.rmax, self.zmin),
                                         (self.rmin, self.zmax), (self.rmax, self.zmax)]):
            b = Marker(); b.header.frame_id = 'map'; b.header.stamp = stamp
            b.ns = 'bounds'; b.id = 10+mid; b.type = Marker.LINE_STRIP; b.action = Marker.ADD
            b.scale.x = 0.02; b.color.r, b.color.g, b.color.b, b.color.a = 0.6, 0.6, 0.6, 0.6
            nn = 40
            for i in range(nn+1):
                th = self.th_min + (self.th_max-self.th_min)*i/nn
                p = self._pt(th, rr, zz); b.points.append(Point(x=float(p[0]), y=float(p[1]), z=float(p[2])))
            ma.markers.append(b)
        line = Marker(); line.header.frame_id = 'map'; line.header.stamp = stamp
        line.ns = 'corridor'; line.id = 1; line.type = Marker.LINE_STRIP; line.action = Marker.ADD
        line.scale.x = 0.05; line.color.r, line.color.g, line.color.b, line.color.a = 0.5, 0.3, 1.0, 0.95
        for p in self.S:
            line.points.append(Point(x=float(p[0]), y=float(p[1]), z=float(p[2])))
        ma.markers.append(line)
        if self.anchors:
            sph = Marker(); sph.header.frame_id = 'map'; sph.header.stamp = stamp
            sph.ns = 'anchors'; sph.id = 2; sph.type = Marker.SPHERE_LIST; sph.action = Marker.ADD
            sph.scale.x = sph.scale.y = sph.scale.z = 0.18
            for th, r, z, _ in self.anchors:
                p = self._pt(th, r, z)
                sph.points.append(Point(x=float(p[0]), y=float(p[1]), z=float(p[2])))
                cc = ColorRGBA(); cc.r, cc.g, cc.b, cc.a = 1.0, 0.55, 0.0, 1.0
                sph.colors.append(cc)
            ma.markers.append(sph)
        self.path_pub.publish(ma)

    def _publish_safety(self):
        """안전 영역 시각화: 바깥 경계(r_max) 반투명 벽, 안쪽 경계(각도별 적응 r_min) 벽, 밑면 부채꼴."""
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        nn = 48
        ths = [self.th_min + (self.th_max - self.th_min) * i / nn for i in range(nn + 1)]

        def wall(mid, radii, rgba, ns):
            m = Marker(); m.header.frame_id = 'map'; m.header.stamp = stamp
            m.ns = ns; m.id = mid; m.type = Marker.TRIANGLE_LIST; m.action = Marker.ADD
            m.scale.x = m.scale.y = m.scale.z = 1.0
            m.color.r, m.color.g, m.color.b, m.color.a = rgba
            for i in range(nn):
                a0 = self._pt_raw(ths[i], radii[i], self.zmin); a1 = self._pt_raw(ths[i], radii[i], self.zmax)
                b0 = self._pt_raw(ths[i+1], radii[i+1], self.zmin); b1 = self._pt_raw(ths[i+1], radii[i+1], self.zmax)
                for q in (a0, b0, a1, a1, b0, b1):
                    m.points.append(Point(x=float(q[0]), y=float(q[1]), z=float(q[2])))
            return m

        r_in = [self._rmin_at(t, self.zmax) for t in ths]           # 가장 보수적(최상단 띠 누적) 경계로 표시
        r_out = [self.rmax] * (nn + 1)
        ma.markers.append(wall(1, r_in, (1.0, 0.45, 0.1, 0.35), 'inner_bound'))     # 주황: 적응형 안쪽 경계
        ma.markers.append(wall(2, r_out, (0.3, 0.7, 1.0, 0.18), 'outer_bound'))    # 하늘: 바깥 경계
        # 밑면/윗면 부채꼴 띠
        for mid, zz in ((3, self.zmin), (4, self.zmax)):
            m = Marker(); m.header.frame_id = 'map'; m.header.stamp = stamp
            m.ns = 'floor'; m.id = mid; m.type = Marker.TRIANGLE_LIST; m.action = Marker.ADD
            m.scale.x = m.scale.y = m.scale.z = 1.0
            m.color.r, m.color.g, m.color.b, m.color.a = 0.3, 0.7, 1.0, 0.12
            for i in range(nn):
                a0 = self._pt_raw(ths[i], r_in[i], zz); a1 = self._pt_raw(ths[i], r_out[i], zz)
                b0 = self._pt_raw(ths[i+1], r_in[i+1], zz); b1 = self._pt_raw(ths[i+1], r_out[i+1], zz)
                for q in (a0, b0, a1, a1, b0, b1):
                    m.points.append(Point(x=float(q[0]), y=float(q[1]), z=float(q[2])))
            ma.markers.append(m)
        # 관측된 수관 반경 프로파일 (빨간 선, z_min 높이)
        if self.profile is not None:
            m = Marker(); m.header.frame_id = 'map'; m.header.stamp = stamp
            m.ns = 'canopy_profile'; m.id = 5; m.type = Marker.LINE_STRIP; m.action = Marker.ADD
            m.scale.x = 0.04; m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.2, 0.2, 0.9
            nb = len(self.profile[0]); top = self.profile[-1]
            for i in range(nb + 1):
                th = 2*math.pi * (i % nb) / nb + math.pi / nb
                ro = top[i % nb]
                if ro <= 0.0:
                    continue
                q = self._pt_raw(th, ro, self.zmin)
                m.points.append(Point(x=float(q[0]), y=float(q[1]), z=float(q[2])))
            if len(m.points) >= 2:
                ma.markers.append(m)
        self.safety_pub.publish(ma)

    def _pt_raw(self, theta, r, z):
        return np.array([self.c[0] + r*math.cos(theta), self.c[1] + r*math.sin(theta), z])

    def _publish_trail(self):
        ma = MarkerArray()
        line = Marker(); line.header.frame_id = 'map'
        line.header.stamp = self.get_clock().now().to_msg()
        line.ns = 'trail'; line.id = 1; line.type = Marker.LINE_STRIP; line.action = Marker.ADD
        line.scale.x = 0.035; line.color.r, line.color.g, line.color.b, line.color.a = 1.0, 0.85, 0.1, 0.9
        for p in self.trail:
            line.points.append(Point(x=float(p[0]), y=float(p[1]), z=float(p[2])))
        ma.markers.append(line)
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
