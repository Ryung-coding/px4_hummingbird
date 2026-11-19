#!/usr/bin/env python3
"""Voxel-based AVP planner for a straight tree row and map/ENU corridor."""

import heapq
import math
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy,
                       QoSDurabilityPolicy, QoSHistoryPolicy)

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, Bool
import struct
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose, PoseArray
from std_msgs.msg import ColorRGBA
import tf2_ros

OCC = 1
FREE = 2

NEIGH6 = [(1,0,0),(-1,0,0),(0,1,0),(0,-1,0),(0,0,1),(0,0,-1)]
NEIGH26 = [(dx,dy,dz) for dx in (-1,0,1) for dy in (-1,0,1) for dz in (-1,0,1)
           if not (dx==0 and dy==0 and dz==0)]


class VoxelData:
    __slots__ = ('state','n_obs','dist_sum','dir_sum','first_dir','max_spread')
    def __init__(self):
        self.state = 0
        self.n_obs = 0
        self.dist_sum = 0.0
        self.dir_sum = np.zeros(3)
        self.first_dir = None
        self.max_spread = 0.0


class AvpPlanner(Node):
    def __init__(self):
        super().__init__('avp_planner')

        # 맵
        self.declare_parameter('resolution', 0.1)
        self.declare_parameter('pixel_stride', 6)
        self.declare_parameter('ray_stride', 4)
        self.declare_parameter('process_hz', 2.0)
        self.declare_parameter('max_range', 3.5)
        # SQC
        self.declare_parameter('d_min', 2.0)
        self.declare_parameter('d_max', 5.0)
        self.declare_parameter('alpha_ideal', 0.5)
        self.declare_parameter('w_dist', 1.0)
        self.declare_parameter('w_angle', 1.0)
        self.declare_parameter('w_base', 1.0)
        self.declare_parameter('w_frontier', 20.0)
        # 패치/시점
        self.declare_parameter('normal_cos_thresh', 0.5)
        self.declare_parameter('min_cluster', 15)
        self.declare_parameter('top_k', 6)
        self.declare_parameter('sqc_min', 12.0)
        self.declare_parameter('d_opt', 3.5)
        self.declare_parameter('vp_min_z', 0.5)
        self.declare_parameter('ground_z_max', 0.3)
        # Tree-row ROI and straight flight corridor, both in map/ENU.
        for name, value in (('roi_x_min', 0.2), ('roi_x_max', 1.8),
                            ('roi_y_min', -1.8), ('roi_y_max', 1.8),
                            ('roi_z_min', 0.2), ('roi_z_max', 2.5),
                            ('corridor_x_min', -0.25), ('corridor_x_max', 0.25),
                            ('corridor_y_min', -1.1), ('corridor_y_max', 1.1),
                            ('corridor_z_min', 0.45), ('corridor_z_max', 1.6)):
            self.declare_parameter(name, value)
        self.declare_parameter('max_pitch_deg', 30.0)
        self.declare_parameter('sweep_z_levels', [1.0])
        self.declare_parameter('vp_pitch_target_deg', 18.0)
        self.declare_parameter('vp_height_offset_m', 0.0)
        self.declare_parameter('require_path_ready', True)
        self.declare_parameter('vp_safety_margin', 0.6)
        # v2b 경로계획
        self.declare_parameter('plan_period_s', 2.5)
        self.declare_parameter('plan_res', 0.3)          # A* coarse 해상도
        self.declare_parameter('inflate_m', 0.6)         # OCC 팽창
        self.declare_parameter('visit_radius', 0.9)      # 도달 판정
        self.declare_parameter('visited_ttl_s', 120.0)
        self.declare_parameter('marker_max', 4000)

        gp = lambda n: self.get_parameter(n).value
        self.res = float(gp('resolution'))
        self.px_stride = int(gp('pixel_stride'))
        self.ray_stride = int(gp('ray_stride'))
        self.min_dt = 1.0 / float(gp('process_hz'))
        self.max_range = float(gp('max_range'))
        self.d_min = float(gp('d_min'))
        self.d_max = float(gp('d_max'))
        self.alpha_ideal = float(gp('alpha_ideal'))
        self.w_d = float(gp('w_dist'))
        self.w_a = float(gp('w_angle'))
        self.w_b = float(gp('w_base'))
        self.w_f = float(gp('w_frontier'))
        self.cos_th = float(gp('normal_cos_thresh'))
        self.min_cluster = int(gp('min_cluster'))
        self.top_k = int(gp('top_k'))
        self.sqc_min = float(gp('sqc_min'))
        self.d_opt = float(gp('d_opt'))
        self.vp_min_z = float(gp('vp_min_z'))
        self.ground_z = float(gp('ground_z_max'))
        self.roi_min = np.array([float(gp('roi_x_min')), float(gp('roi_y_min')), float(gp('roi_z_min'))])
        self.roi_max = np.array([float(gp('roi_x_max')), float(gp('roi_y_max')), float(gp('roi_z_max'))])
        self.max_pitch = math.radians(min(90.0, max(0.0, float(gp('max_pitch_deg')))))
        self.sweep_z_levels = [float(z) for z in gp('sweep_z_levels')]
        self.vp_pitch_target = math.radians(float(gp('vp_pitch_target_deg')))
        self.vp_height_offset = float(gp('vp_height_offset_m'))
        self.vp_margin = float(gp('vp_safety_margin'))
        self.plan_res = float(gp('plan_res'))
        self.inflate = float(gp('inflate_m'))
        self.visit_r = float(gp('visit_radius'))
        self.visited_ttl = float(gp('visited_ttl_s'))
        self.marker_max = int(gp('marker_max'))
        self.corridor_min = np.array([float(gp('corridor_x_min')), float(gp('corridor_y_min')), float(gp('corridor_z_min'))])
        self.corridor_max = np.array([float(gp('corridor_x_max')), float(gp('corridor_y_max')), float(gp('corridor_z_max'))])
        if np.any(self.roi_min >= self.roi_max) or np.any(self.corridor_min >= self.corridor_max):
            raise ValueError('AVP ROI and corridor require min < max on every axis')
        if not self.sweep_z_levels or not 0.0 <= self.vp_pitch_target <= self.max_pitch or self.vp_height_offset < 0.0:
            raise ValueError('Invalid viewpoint pitch or sweep height parameters')

        r = int(math.ceil(self.vp_margin / self.res))
        self._safe_off = [(i,j,k) for i in range(-r,r+1) for j in range(-r,r+1)
                          for k in range(-r,r+1)
                          if (i*i+j*j+k*k)*self.res*self.res <= self.vp_margin**2]

        rc = int(math.ceil(0.4 / self.res))
        self._clear_off = [(i,j,k) for i in range(-rc,rc+1) for j in range(-rc,rc+1)
                           for k in range(-rc,rc+1)
                           if (i*i+j*j+k*k)*self.res*self.res <= 0.16]
        self.vox = {}
        self.last_proc_t = -1e9
        self.last_tf_warn_t = -1e9
        self.path_ready = False
        self.last_ready_t = -1e9
        self.require_path_ready = bool(gp('require_path_ready'))
        self.frames = 0
        self.visited = []                # [(pos ENU, t)]
        self.cur_target = None           # 발행 중 목표 (ENU, snap됨)
        self.cur_patch = None            # 그 목표의 패치 중심

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         durability=QoSDurabilityPolicy.VOLATILE,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(PointCloud2, '/rgbd_camera/points',
                                 self.cloud_cb, qos)
        self.create_subscription(Bool, '/avp/path_ready', self.ready_cb, 1)

        self.pub_normals = self.create_publisher(MarkerArray, '/surface/normals', 1)
        self.pub_coverage = self.create_publisher(MarkerArray, '/surface/coverage', 1)
        self.pub_sqc = self.create_publisher(MarkerArray, '/avp/sqc', 1)
        self.pub_vp = self.create_publisher(MarkerArray, '/avp/viewpoints', 1)
        self.pub_route = self.create_publisher(PoseArray, '/avp/route', 1)
        self.pub_vpd = self.create_publisher(PoseArray, '/avp/viewpoints_data', 1)
        self.pub_route_viz = self.create_publisher(MarkerArray, '/avp/route_viz', 1)
        self.pub_occ = self.create_publisher(PointCloud2, '/avp/occ_voxels', 1)

        self.create_timer(float(gp('plan_period_s')), self.plan_cb)
        self.get_logger().info(
            f'avp_planner v2b ready. res={self.res} plan_res={self.plan_res} '
            f'd_opt={self.d_opt} sqc_min={self.sqc_min} (unknown=blocked A*) '
            + f'ROI box={self.roi_min.tolist()}..{self.roi_max.tolist()}')

    # ================= 맵 갱신 (기존 동일) =================

    def ready_cb(self, msg: Bool):
        self.path_ready = bool(msg.data)
        self.last_ready_t = self.get_clock().now().nanoseconds / 1e9

    def _ready(self, now):
        return not self.require_path_ready or (self.path_ready and now - self.last_ready_t <= 0.5)

    def cloud_cb(self, msg: PointCloud2):
        now = self.get_clock().now().nanoseconds / 1e9
        if not self._ready(now):
            return
        if now - self.last_proc_t < self.min_dt:
            return
        try:
            tfs = self.tf_buffer.lookup_transform('map', msg.header.frame_id,
                                                  rclpy.time.Time())
        except Exception as e:
            if now - self.last_tf_warn_t >= 2.0:
                self.last_tf_warn_t = now
                self.get_logger().warn(f'TF not ready: {e}')
            return
        self.last_proc_t = now

        pts = self._parse_cloud(msg)
        if pts is None or pts.shape[0] == 0:
            return
        d = np.linalg.norm(pts, axis=1)
        m = np.isfinite(d) & (d > 0.05) & (d < self.max_range)
        pts = pts[m]
        if pts.shape[0] == 0:
            return

        q = tfs.transform.rotation
        t = tfs.transform.translation
        R = self._quat_to_R(q.w, q.x, q.y, q.z)
        origin = np.array([t.x, t.y, t.z])
        pw = pts @ R.T + origin

        keys = np.floor(pw / self.res).astype(np.int64)
        dirs = pw - origin
        dists = np.linalg.norm(dirs, axis=1)
        dirs = dirs / dists[:, None]

        seen = {}
        for i in range(keys.shape[0]):
            k = (int(keys[i,0]), int(keys[i,1]), int(keys[i,2]))
            if k not in seen:
                seen[k] = i
        for k, i in seen.items():
            v = self.vox.get(k)
            if v is None:
                v = VoxelData(); self.vox[k] = v
            v.state = OCC
            v.n_obs += 1
            v.dist_sum += float(dists[i])
            v.dir_sum += dirs[i]
            if v.first_dir is None:
                v.first_dir = dirs[i].copy()
            else:
                c = float(np.clip(np.dot(v.first_dir, dirs[i]), -1.0, 1.0))
                ang = math.acos(c)
                if ang > v.max_spread:
                    v.max_spread = ang

        sub = pw[::self.ray_stride]
        sd = dists[::self.ray_stride]
        if sub.shape[0] > 0:
            step = self.res
            nmax = int(self.max_range / step)
            ts = np.arange(1, nmax) * step
            dcap = (sd - self.res)[None, :]
            tt = ts[:, None]
            valid = tt < dcap
            unit = (sub - origin) / sd[:, None]
            samples = origin[None,None,:] + tt[:,:,None] * unit[None,:,:]
            fk = np.floor(samples[valid] / self.res).astype(np.int64)
            if fk.shape[0] > 0:
                uk = np.unique(fk, axis=0)
                for row in uk:
                    k = (int(row[0]), int(row[1]), int(row[2]))
                    v = self.vox.get(k)
                    if v is None:
                        v = VoxelData(); v.state = FREE; self.vox[k] = v
                    elif v.state != OCC:
                        v.state = FREE

        self.frames += 1
        if self.frames % 20 == 0:
            self.get_logger().info(f'frames={self.frames} voxels={len(self.vox)}')

    # ================= 계획 루프 =================

    def plan_cb(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if not self._ready(now):
            return
        if not self.vox:
            return
        drone = self._drone_pos()
        if drone is None:
            return
        dc = np.floor(drone / self.res).astype(np.int64)
        for off in self._clear_off:
            kk = (int(dc[0]+off[0]), int(dc[1]+off[1]), int(dc[2]+off[2]))
            v = self.vox.get(kk)
            if v is None:
                v = VoxelData(); v.state = FREE; self.vox[kk] = v
            elif v.state != OCC:
                v.state = FREE

        self._publish_obstacles()

        # 방문 판정/정리
        now = self.get_clock().now().nanoseconds / 1e9
        self.visited = [(p, t) for (p, t) in self.visited
                        if now - t < self.visited_ttl]
        if (self.cur_target is not None
                and np.linalg.norm(drone - self.cur_target) < self.visit_r):
            self.visited.append((self.cur_patch.copy(), now))
            self.cur_target = None

        # --- 표면/법선 ---
        surf = {}
        for k, v in self.vox.items():
            if v.state != OCC:
                continue
            pos = (np.asarray(k, dtype=float) + 0.5) * self.res
            if np.any(pos < self.roi_min) or np.any(pos > self.roi_max):
                continue
            n = np.zeros(3); cnt = 0
            for d6 in NEIGH6:
                nv = self.vox.get((k[0]+d6[0], k[1]+d6[1], k[2]+d6[2]))
                if nv is not None and nv.state == FREE:
                    n += d6; cnt += 1
            if cnt > 0:
                nn = np.linalg.norm(n)
                if nn > 1e-9:
                    surf[k] = n / nn
        self._publish_surface(surf)
        if not surf:
            return

        # --- voxel SQC (+frontier 감쇠) ---
        sqc = {}
        for k, n in surf.items():
            v = self.vox[k]
            dbar = v.dist_sum / max(v.n_obs, 1)
            cd = max(0.0, dbar - self.d_max)**2 + max(0.0, self.d_min - dbar)**2
            vbar = -v.dir_sum
            nv = np.linalg.norm(vbar)
            ca = 1.0 - float(np.dot(n, vbar / nv)) if nv > 1e-9 else 1.0
            cb = math.exp(-v.max_spread / self.alpha_ideal)
            n_unk = 0
            for d6 in NEIGH6:
                if (k[0]+d6[0], k[1]+d6[1], k[2]+d6[2]) not in self.vox:
                    n_unk += 1
            cu = n_unk / 6.0
            sqc[k] = self.w_d*cd + self.w_a*ca + self.w_b*cb + self.w_f*cu

        # --- 패치 ---
        visited_set = set()
        patches = []
        for seed in surf:
            if seed in visited_set:
                continue
            q = deque([seed]); visited_set.add(seed)
            cluster = [seed]
            n_acc = surf[seed].copy()
            while q:
                cur = q.popleft()
                for d26 in NEIGH26:
                    nk = (cur[0]+d26[0], cur[1]+d26[1], cur[2]+d26[2])
                    if nk in visited_set or nk not in surf:
                        continue
                    n_mean = n_acc / np.linalg.norm(n_acc)
                    if float(np.dot(surf[nk], n_mean)) >= self.cos_th:
                        visited_set.add(nk); q.append(nk)
                        cluster.append(nk); n_acc += surf[nk]
            if len(cluster) >= self.min_cluster:
                patches.append(cluster)
        if not patches:
            return

        pinfo = []
        for cl in patches:
            vals = sorted(sqc[k] for k in cl)
            med = vals[len(vals)//2]
            center = (np.array(cl, dtype=float).mean(axis=0) + 0.5) * self.res
            pinfo.append({'keys': cl, 'sqc': med, 'center': center})
        pinfo.sort(key=lambda p: p['sqc'], reverse=True)

        # --- 시점 후보 생성 ---
        vps = []
        for p in pinfo:
            if p['sqc'] < self.sqc_min:
                break
            if p['center'][2] < self.ground_z:
                continue
            vp = None
            level = min(self.sweep_z_levels, key=lambda z: abs(z - p['center'][2]))
            heights = [p['center'][2]] + list(np.percentile(
                (np.asarray(p['keys'])[:, 2] + 0.5) * self.res, [10, 90]))
            for dmul in (1.0, 0.75, 1.25, 0.5, 1.5):
                choices = []
                for dz in (0.0, self.vp_height_offset, -self.vp_height_offset):
                    cand = np.clip(p['center'] - np.array([self.d_opt * dmul, 0.0, 0.0]),
                                   self.corridor_min, self.corridor_max)
                    cand[2] = level + dz
                    if (cand[2] < max(self.vp_min_z, self.corridor_min[2] + 0.1)
                            or not self._inside_corridor(cand) or not self._vp_safe(cand)):
                        continue
                    for aim_z in heights:
                        if aim_z < self.ground_z:
                            continue
                        aim = p['center'].copy()
                        aim[2] = aim_z
                        look = aim - cand
                        if (look[0] <= 0.0 or abs(math.atan2(look[1], look[0])) > math.radians(35.0)
                                or not self._pitch_ok(look) or not self._visible(cand, aim)):
                            continue
                        pitch = abs(math.atan2(float(look[2]), float(np.linalg.norm(look[:2]))))
                        score = abs(pitch - self.vp_pitch_target) + 0.25 * abs(dz) + 0.08 * abs(aim_z - p['center'][2])
                        choices.append((score, cand, look))
                if choices:
                    _, vp, look = min(choices, key=lambda item: item[0])
                    break
            if vp is None:
                continue
            look /= np.linalg.norm(look)
            vps.append((vp, look, p['sqc'], p['center']))
            if len(vps) >= self.top_k:
                break

        # 코리더 실행기용: 전체 시점 발행
        vpa = PoseArray()
        vpa.header.frame_id = 'map'
        vpa.header.stamp = self.get_clock().now().to_msg()
        for vp, look, s_, pc in vps:
            yaw = math.atan2(look[1], look[0])
            pitch = -math.asin(max(-1.0, min(1.0, float(look[2]))))
            cy2, sy2 = math.cos(yaw*0.5), math.sin(yaw*0.5)
            cp2, sp2 = math.cos(pitch*0.5), math.sin(pitch*0.5)
            po = Pose()
            po.position.x, po.position.y, po.position.z = map(float, vp)
            po.orientation.w = cy2*cp2; po.orientation.x = -sy2*sp2
            po.orientation.y = cy2*sp2; po.orientation.z = sy2*cp2
            vpa.poses.append(po)
        self.pub_vpd.publish(vpa)

        # 방문 필터
        cand = [c for c in vps
                if all(np.linalg.norm(c[3] - q_) > self.visit_r
                       for q_, _t in self.visited)]   # 패치 단위 blacklist

        # --- 경로계획: coarse known-free A* ---
        grid = self._build_plan_grid()
        route = None
        chosen = None
        if cand and grid is not None:
            order = self._tour_order(drone, [c[0] for c in cand])
            for oi in order:
                vp, look, s_, pc = cand[oi]
                g = self._snap(grid, vp, max_r=8)
                if g is None:
                    continue
                gpos = (np.array(g) + 0.5) * self.plan_res
                if np.linalg.norm(gpos - vp) > 2.0 * self.plan_res:
                    continue
                lk = pc - gpos
                nl = float(np.linalg.norm(lk))
                if nl < 0.5:
                    continue
                lk = lk / nl
                if not self._pitch_ok(lk):
                    continue
                if np.linalg.norm(vp - drone) < self.visit_r:
                    self.visited.append((pc.copy(), now))   # 이미 그 자리 = 방문 처리
                    continue
                path = self._astar(grid, drone, gpos)
                if path is not None:
                    route = self._string_pull(grid, [drone] + path)
                    chosen = (gpos, look)
                    self.cur_patch = pc.copy()
                    break

        # --- 발행 ---
        pa = PoseArray()
        pa.header.frame_id = 'map'
        pa.header.stamp = self.get_clock().now().to_msg()
        if route is not None and chosen is not None:
            vp, look = chosen
            self.cur_target = vp.copy()
            for w in route[1:-1]:
                po = Pose()
                po.position.x, po.position.y, po.position.z = map(float, w)
                po.orientation.w = 1.0
                pa.poses.append(po)
            yaw = math.atan2(look[1], look[0])
            pitch = -math.asin(max(-1.0, min(1.0, float(look[2]))))
            cy, sy = math.cos(yaw*0.5), math.sin(yaw*0.5)
            cp, sp = math.cos(pitch*0.5), math.sin(pitch*0.5)
            po = Pose()
            po.position.x, po.position.y, po.position.z = map(float, vp)
            po.orientation.w = cy*cp
            po.orientation.x = -sy*sp
            po.orientation.y = cy*sp
            po.orientation.z = sy*cp
            pa.poses.append(po)
        self.pub_route.publish(pa)

        n_reach = 'none' if chosen is None else 'ok'
        self.get_logger().info(
            f'surface={len(surf)} patches={len(patches)} '
            f'worst={pinfo[0]["sqc"]:.1f} vps={len(vps)} cand={len(cand)} '
            f'route={n_reach} visited={len(self.visited)}')

        self._publish_markers(pinfo, vps, route, chosen)

    # ================= v2b: 경로계획 유틸 =================

    def _drone_pos(self):
        try:
            tfs = self.tf_buffer.lookup_transform('map', 'base_link',
                                                  rclpy.time.Time())
            t = tfs.transform.translation
            return np.array([t.x, t.y, t.z])
        except Exception:
            return None

    def _build_plan_grid(self):
        """coarse 셀: FREE 포함 & (팽창)OCC 미포함 -> 통행 가능. unknown=벽."""
        ratio = self.plan_res / self.res
        occ_c = set()
        free_c = set()
        for k, v in self.vox.items():
            ck = (int(math.floor(k[0] / ratio)),
                  int(math.floor(k[1] / ratio)),
                  int(math.floor(k[2] / ratio)))
            if v.state == OCC:
                occ_c.add(ck)
            elif v.state == FREE:
                free_c.add(ck)
        infl = int(math.ceil(self.inflate / self.plan_res))
        blocked = set()
        offs = [(i,j,k) for i in range(-infl,infl+1) for j in range(-infl,infl+1)
                for k in range(-infl,infl+1)]
        for c in occ_c:
            for o in offs:
                blocked.add((c[0]+o[0], c[1]+o[1], c[2]+o[2]))
        trav = free_c - blocked
        return trav if trav else None

    def _cell(self, p):
        return (int(math.floor(p[0] / self.plan_res)),
                int(math.floor(p[1] / self.plan_res)),
                int(math.floor(p[2] / self.plan_res)))

    def _snap(self, grid, p, max_r=4):
        c0 = self._cell(p)
        if c0 in grid:
            return c0
        best, bd = None, 1e9
        for i in range(-max_r, max_r+1):
            for j in range(-max_r, max_r+1):
                for k in range(-max_r, max_r+1):
                    c = (c0[0]+i, c0[1]+j, c0[2]+k)
                    if c in grid:
                        d = i*i + j*j + k*k
                        if d < bd:
                            bd, best = d, c
        return best

    def _astar(self, grid, start, goal):
        s = self._snap(grid, start)
        g = self._snap(grid, goal)
        if s is None or g is None:
            return None
        def h(c):
            return math.sqrt((c[0]-g[0])**2 + (c[1]-g[1])**2 + (c[2]-g[2])**2)
        openq = [(h(s), 0.0, s)]
        came = {s: None}
        gsc = {s: 0.0}
        it = 0
        while openq:
            it += 1
            if it > 60000:
                return None
            _f, gc, cur = heapq.heappop(openq)
            if cur == g:
                path = []
                c = g
                while c is not None:
                    path.append((np.array(c) + 0.5) * self.plan_res)
                    c = came[c]
                return list(reversed(path))
            for d in NEIGH26:
                nb = (cur[0]+d[0], cur[1]+d[1], cur[2]+d[2])
                if nb not in grid:
                    continue
                ng = gc + math.sqrt(d[0]*d[0]+d[1]*d[1]+d[2]*d[2])
                if ng < gsc.get(nb, 1e18):
                    gsc[nb] = ng
                    came[nb] = cur
                    heapq.heappush(openq, (ng + h(nb), ng, nb))
        return None

    def _line_free(self, grid, a, b):
        L = np.linalg.norm(b - a)
        n = max(2, int(L / (self.plan_res * 0.5)))
        for i in range(1, n):
            p = a + (b - a) * (i / n)
            if self._cell(p) not in grid:
                return False
        return True

    def _string_pull(self, grid, pts):
        out = [pts[0]]
        i = 0
        while i < len(pts) - 1:
            j = len(pts) - 1
            while j > i + 1 and not self._line_free(grid, pts[i], pts[j]):
                j -= 1
            out.append(pts[j])
            i = j
        return out

    @staticmethod
    def _tour_order(start, pts):
        n = len(pts)
        rem = list(range(n)); order = []; cur = start
        while rem:
            j = min(rem, key=lambda i: np.linalg.norm(pts[i] - cur))
            order.append(j); cur = pts[j]; rem.remove(j)
        def leng(o):
            L = np.linalg.norm(pts[o[0]] - start)
            for a, b in zip(o, o[1:]):
                L += np.linalg.norm(pts[b] - pts[a])
            return L
        improved = True
        while improved and n >= 4:
            improved = False
            for i in range(n - 2):
                for j in range(i + 2, n):
                    new = order[:i+1] + order[i+1:j+1][::-1] + order[j+1:]
                    if leng(new) < leng(order) - 1e-6:
                        order = new; improved = True
        return order

    # ================= 시점 유틸 (기존) =================

    def _pitch_ok(self, look):
        pitch = math.atan2(float(look[2]), float(np.linalg.norm(look[:2])))
        return abs(pitch) <= self.max_pitch

    # Publish nearby occupied voxels for executor clearance checks.
    def _publish_obstacles(self):
        lo = np.minimum(self.roi_min, self.corridor_min) - 0.5
        hi = np.maximum(self.roi_max, self.corridor_max) + 0.5
        pts = []
        for k, v in self.vox.items():
            if v.state != OCC:
                continue
            pos = (np.asarray(k, dtype=float) + 0.5) * self.res
            if np.all(pos >= lo) and np.all(pos <= hi):
                pts.append(pos)
        hdr = Header(); hdr.frame_id = 'map'; hdr.stamp = self.get_clock().now().to_msg()
        pc = PointCloud2(); pc.header = hdr; pc.height = 1; pc.width = len(pts)
        pc.fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                     PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                     PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)]
        pc.is_bigendian = False; pc.point_step = 12; pc.row_step = 12 * len(pts); pc.is_dense = True
        pc.data = np.asarray(pts, dtype=np.float32).tobytes()
        self.pub_occ.publish(pc)

    def _vp_safe(self, vp):
        c = np.floor(vp / self.res).astype(np.int64)
        for off in self._safe_off:
            v = self.vox.get((int(c[0]+off[0]), int(c[1]+off[1]),
                              int(c[2]+off[2])))
            if v is not None and v.state == OCC:
                return False
        return True

    def _inside_corridor(self, pos):
        return bool(np.all(pos >= self.corridor_min) and np.all(pos <= self.corridor_max))

    def _visible(self, vp, target):
        d = target - vp
        L = np.linalg.norm(d)
        u = d / L
        n_steps = int(L / self.res)
        for i in range(1, n_steps):
            if (i * self.res) > (L - 2 * self.res):
                break
            s = vp + u * (i * self.res)
            k = tuple(np.floor(s / self.res).astype(np.int64))
            v = self.vox.get(k)
            if v is not None and v.state == OCC:
                return False
        return True

    # ================= 시각화 =================

    def _publish_surface(self, surf):
        stamp = self.get_clock().now().to_msg()
        normals = MarkerArray()
        coverage = MarkerArray()
        for arr in (normals, coverage):
            clear = Marker()
            clear.action = Marker.DELETEALL
            arr.markers.append(clear)
        arrow = Marker()
        arrow.header.frame_id = 'map'; arrow.header.stamp = stamp
        arrow.ns = 'normals'; arrow.id = 1
        arrow.type = Marker.LINE_LIST; arrow.action = Marker.ADD
        arrow.scale.x = 0.01
        arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = 0.2, 0.6, 1.0, 0.9
        cube = Marker()
        cube.header.frame_id = 'map'; cube.header.stamp = stamp
        cube.ns = 'coverage'; cube.id = 1
        cube.type = Marker.CUBE_LIST; cube.action = Marker.ADD
        cube.scale.x = cube.scale.y = cube.scale.z = self.res * 0.9
        for k, n in list(surf.items())[::max(1, len(surf) // self.marker_max)]:
            c = (np.array(k) + 0.5) * self.res
            tip = c + n * (2.0 * self.res)
            arrow.points.extend([Point(x=float(c[0]), y=float(c[1]), z=float(c[2])),
                                 Point(x=float(tip[0]), y=float(tip[1]), z=float(tip[2]))])
            cube.points.append(Point(x=float(c[0]), y=float(c[1]), z=float(c[2])))
            ratio = min(self.vox[k].n_obs / 10.0, 1.0)
            cube.colors.append(ColorRGBA(r=float(1.0 - ratio), g=float(ratio), b=0.0, a=0.85))
        normals.markers.append(arrow)
        coverage.markers.append(cube)
        self.pub_normals.publish(normals)
        self.pub_coverage.publish(coverage)

    def _publish_markers(self, pinfo, vps, route, chosen):
        stamp = self.get_clock().now().to_msg()

        ma = MarkerArray()
        clr = Marker(); clr.action = Marker.DELETEALL
        ma.markers.append(clr)
        cube = Marker()
        cube.header.frame_id = 'map'; cube.header.stamp = stamp
        cube.ns = 'sqc'; cube.id = 1
        cube.type = Marker.CUBE_LIST; cube.action = Marker.ADD
        s = self.res * 0.9
        cube.scale.x = cube.scale.y = cube.scale.z = s
        smin = min(p['sqc'] for p in pinfo)
        smax = max(p['sqc'] for p in pinfo)
        rng = max(smax - smin, 1e-6)
        total = sum(len(p['keys']) for p in pinfo)
        stride = max(1, total // self.marker_max)
        idx = 0
        for p in pinfo:
            r = (p['sqc'] - smin) / rng
            col = ColorRGBA()
            col.r, col.g, col.b, col.a = float(r), float(1.0-r), 0.0, 0.9
            for k in p['keys']:
                idx += 1
                if idx % stride:
                    continue
                c = (np.array(k) + 0.5) * self.res
                cube.points.append(Point(x=float(c[0]), y=float(c[1]),
                                         z=float(c[2])))
                cube.colors.append(col)
        ma.markers.append(cube)
        self.pub_sqc.publish(ma)

        mv = MarkerArray()
        clr2 = Marker(); clr2.action = Marker.DELETEALL
        mv.markers.append(clr2)
        for i, (vp, look, s_, _pc) in enumerate(vps):
            ar = Marker()
            ar.header.frame_id = 'map'; ar.header.stamp = stamp
            ar.ns = 'vp'; ar.id = i
            ar.type = Marker.ARROW; ar.action = Marker.ADD
            ar.scale.x, ar.scale.y, ar.scale.z = 0.025, 0.05, 0.08
            ar.color.r, ar.color.g, ar.color.b, ar.color.a = 1.0, 0.55, 0.0, 0.25
            p0 = Point(x=float(vp[0]), y=float(vp[1]), z=float(vp[2]))
            tip = vp + look * 1.0
            p1 = Point(x=float(tip[0]), y=float(tip[1]), z=float(tip[2]))
            ar.points = [p0, p1]
            mv.markers.append(ar)
        if chosen is not None:
            vp, look = chosen
            selected = Marker()
            selected.header.frame_id = 'map'; selected.header.stamp = stamp
            selected.ns = 'selected'; selected.id = 0
            selected.type = Marker.ARROW; selected.action = Marker.ADD
            selected.scale.x, selected.scale.y, selected.scale.z = 0.10, 0.18, 0.28
            selected.color.r, selected.color.g, selected.color.b, selected.color.a = 0.1, 1.0, 0.2, 1.0
            selected.points = [Point(x=float(vp[0]), y=float(vp[1]), z=float(vp[2])),
                               Point(x=float(vp[0] + look[0]), y=float(vp[1] + look[1]), z=float(vp[2] + look[2]))]
            mv.markers.append(selected)
        self.pub_vp.publish(mv)

        mr = MarkerArray()
        clr3 = Marker(); clr3.action = Marker.DELETEALL
        mr.markers.append(clr3)
        if route is not None:
            line = Marker()
            line.header.frame_id = 'map'; line.header.stamp = stamp
            line.ns = 'route'; line.id = 1
            line.type = Marker.LINE_STRIP; line.action = Marker.ADD
            line.scale.x = 0.05
            line.color.r, line.color.g, line.color.b, line.color.a = 0.1, 0.8, 0.2, 0.95
            for w in route:
                line.points.append(Point(x=float(w[0]), y=float(w[1]),
                                         z=float(w[2])))
            mr.markers.append(line)
        self.pub_route_viz.publish(mr)

    # ================= 파싱/수학 =================

    def _parse_cloud(self, msg):
        off = {f.name: f.offset for f in msg.fields}
        if not all(k in off for k in ('x','y','z')):
            return None
        n = msg.width * msg.height
        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, msg.point_step)
        raw = raw[::self.px_stride]
        x = raw[:, off['x']:off['x']+4].copy().view(np.float32).reshape(-1)
        y = raw[:, off['y']:off['y']+4].copy().view(np.float32).reshape(-1)
        z = raw[:, off['z']:off['z']+4].copy().view(np.float32).reshape(-1)
        return np.stack([x,y,z], axis=1)

    @staticmethod
    def _quat_to_R(w,x,y,z):
        return np.array([
            [1-2*(y*y+z*z), 2*(x*y-w*z),   2*(x*z+w*y)],
            [2*(x*y+w*z),   1-2*(x*x+z*z), 2*(y*z-w*x)],
            [2*(x*z-w*y),   2*(y*z+w*x),   1-2*(x*x+y*y)],
        ])


def main(args=None):
    rclpy.init(args=args)
    node = AvpPlanner()
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
