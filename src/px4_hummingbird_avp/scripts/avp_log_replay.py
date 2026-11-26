#!/usr/bin/env python3
"""Replay an AVP flight folder and its overlapping headless log without PX4 commands."""

import argparse
from bisect import bisect_right
import csv
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import signal
import sqlite3
import time

import cv2
import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets
import pyqtgraph as pg
import rclpy
from rclpy.signals import SignalHandlerOptions
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path as RosPath
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from avp_bridge import R_FB, R_NE, R_to_quat_wxyz, quat_wxyz_to_R


ROOTS = (Path.home() / 'Desktop/view_log',
         Path.home() / 'Desktop/px4_hummingbird/src/px4_hummingbird_avp/logs')
# Depth visualization settings (metres / pixels).
DEPTH_STEP = 2
DEPTH_MIN_M = 0.2
DEPTH_MAX_M = 4.0
DEPTH_VERTICAL_FOV_DEG = 30.0  # full vertical angle; +/- 15 deg around optical axis
MAP_VOXEL_M = 0.015  # deduplication only; no raycasting or surface filtering
MAP_PUBLISH_PERIOD_S = 0.5
MAP = 'map' 
BASE = 'base_link'
OPTICAL = 'oak_rgb_camera_optical_frame'
SIM_OPTICAL = 'hummingbird_0/base_link/rgbd_camera_optical'
CONFIG = Path(get_package_share_directory('px4_hummingbird_avp')) / 'config'
with (CONFIG / 'oak_d_pro.yaml').open(encoding='utf-8') as config_file:
    MOUNT = yaml.safe_load(config_file)['/avp_oak']['ros__parameters']



def locate(name):
    path = Path(name).expanduser()
    for candidate in (path, *(root / path for root in ROOTS)):
        if candidate.exists():
            return candidate.resolve()
        if candidate.with_suffix('.jsonl').is_file():
            return candidate.with_suffix('.jsonl').resolve()
    raise FileNotFoundError(name)


def first_time(path):
    source = path / 'events.jsonl' if path.is_dir() else path
    with source.open(encoding='utf-8') as stream:
        row = json.loads(stream.readline())
    return float(row.get('wall_time', row.get('wall_time_ns', 0) / 1e9))


def select_logs(first, second=None):
    if first is None:
        folders = [p for root in ROOTS for p in root.glob('flight_*') if (p / 'events.jsonl').is_file()]
        if not folders:
            raise FileNotFoundError('No flight_* folder found in view_log or AVP logs')
        flight = max(folders, key=first_time)
        headless = None
    else:
        path = locate(first)
        flight = path if path.is_dir() else None
        headless = path if path.is_file() else None
    if second:
        other = locate(second)
        if other.is_dir():
            flight = other
        else:
            headless = other
    if flight is None or headless is None:
        candidates = ([p for root in ROOTS for p in root.glob('flight_*') if (p / 'events.jsonl').is_file()]
                      if flight is None else
                      [p for root in ROOTS for p in root.glob('*.jsonl') if 'headless' in p.name])
        reference = headless if flight is None else flight
        candidates = [p for p in candidates if p != reference]
        if candidates:
            match = min(candidates, key=lambda p: abs(first_time(p) - first_time(reference)))
            if abs(first_time(match) - first_time(reference)) <= 300:
                if flight is None:
                    flight = match
                else:
                    headless = match
    if flight is None or not (flight / 'odometry.csv').is_file():
        raise FileNotFoundError('Matching flight_* folder with odometry.csv not found')
    if headless is not None and not headless.is_file():
        raise ValueError('Headless log must be a JSONL file')
    return flight, headless


def timed(rows, t):
    if not rows:
        return None
    idx = bisect_right([row[0] for row in rows], t) - 1
    return rows[idx] if idx >= 0 else None


def pose_message(values, stamp, frame=MAP):
    msg = PoseStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = frame
    p, q = msg.pose.position, msg.pose.orientation
    p.x, p.y, p.z = values[:3]
    q.w, q.x, q.y, q.z = values[3:7]
    return msg


def target_to_map(values):
    pos = R_NE @ np.asarray(values[:3], dtype=float)
    R = quat_wxyz_to_R(values[3:7])
    if R is None:
        return None
    q = R_to_quat_wxyz(R_NE @ R @ R_FB)
    return tuple(pos) + tuple(q)


def stop_child(process):
    if process is None:
        return
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
    try:
        process.wait(timeout=3)
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(process.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    process.wait()


class FlightData:
    def __init__(self, folder, headless):
        self.folder, self.headless = folder, headless
        self.odom, self.target, self.viewpoints, self.route, self.sweep_center = [], [], [], [], []
        self.rgb, self.depth, self.camera = [], [], None
        self.frame_dbs = {}
        self.mode = 'real'
        self.camera_frame = OPTICAL
        color_topic = '/oak/rgb/image_raw'
        depth_topic = '/oak/stereo/image_raw'
        camera_info_topic = '/oak/stereo/camera_info'
        self.signals = {name: [] for name in ('rates', 'rates_sp', 'motors', 'servos', 'status', 'opti')}
        with (folder / 'odometry.csv').open(newline='') as stream:
            for row in csv.DictReader(stream):
                if row['frame_id'] != 'map':
                    continue
                values = tuple(float(row[key]) for key in ('x', 'y', 'z', 'qw', 'qx', 'qy', 'qz'))
                if np.isfinite(values).all():
                    self.odom.append((int(row['wall_time_ns']) / 1e9, values))
        with (folder / 'setpoints.csv').open(newline='') as stream:
            for row in csv.DictReader(stream):
                if row['topic'] != '/fmu/in/trajectory_setpoint6dof':
                    continue
                values = tuple(float(row[key]) for key in ('x', 'y', 'z', 'qw', 'qx', 'qy', 'qz'))
                if np.isfinite(values).all():
                    converted = target_to_map(values)
                    if converted:
                        self.target.append((int(row['wall_time_ns']) / 1e9, converted))
        with (folder / 'events.jsonl').open(encoding='utf-8') as stream:
            for line in stream:
                row = json.loads(line)
                topic, t = row['topic'], row['wall_time_ns'] / 1e9
                if topic == 'meta':
                    self.mode = row.get('mode', self.mode)
                    if self.mode == 'sim':
                        self.camera_frame = SIM_OPTICAL
                    color_topic = row.get('color_topic', color_topic)
                    depth_topic = row.get('depth_topic', depth_topic)
                    camera_info_topic = row.get('camera_info_topic', camera_info_topic)
                elif topic == '/avp/viewpoints_data':
                    self.viewpoints.append((t, row['poses']))
                elif topic == '/avp/route':
                    self.route.append((t, row['poses']))
                elif topic == '/avp/sweep_center':
                    self.sweep_center.append((t, row['pose']))
                elif topic == color_topic and row.get('db_file'):
                    self.rgb.append((t, row['db_file'], row['db_id']))
                elif topic == color_topic and row.get('video_frame'):
                    self.rgb.append((t, None, row['video_frame'] - 1))
                elif topic == depth_topic and row.get('db_file'):
                    self.depth.append((t, row['db_file'], row['db_id'], row.get('depth_scale_m', 0.001)))
                elif topic == depth_topic and row.get('raw_path'):
                    self.depth.append((t, None, row['raw_path'], row.get('depth_scale_m', 0.001)))
                elif topic == camera_info_topic and self.camera is None:
                    self.camera = row['k']

        if headless:
            topics = {'/fmu/out/vehicle_angular_velocity': 'rates',
                      '/fmu/out/vehicle_rates_setpoint': 'rates_sp',
                      '/fmu/out/actuator_motors': 'motors',
                      '/fmu/out/actuator_servos': 'servos',
                      '/fmu/out/vehicle_status': 'status', '/opti_raw': 'opti'}
            last = {}
            with headless.open(encoding='utf-8') as stream:
                for line in stream:
                    row = json.loads(line)
                    name = topics.get(row['topic'])
                    if not name:
                        continue
                    t = row['wall_time']
                    if t - last.get(name, -1e9) < (0.02 if name == 'servos' else 0.01):
                        continue
                    self.signals[name].append((t, row['data']))
                    last[name] = t
        starts = [series[0][0] for series in (self.odom, self.target, self.rgb, self.depth)
                  if series]
        ends = [series[-1][0] for series in (self.odom, self.target, self.rgb, self.depth)
                if series]
        if headless:
            starts.append(first_time(headless))
        if not starts:
            raise ValueError('No replayable odometry, target, or camera data')
        self.start, self.end = min(starts), max(ends)
        for series in (self.odom, self.target, self.viewpoints, self.route, self.sweep_center, self.rgb, self.depth,
                       *self.signals.values()):
            series.sort(key=lambda item: item[0])

    def read_frame(self, db_file, db_id, flags):
        if db_file not in self.frame_dbs:
            path = self.folder / db_file
            self.frame_dbs[db_file] = sqlite3.connect(f'{path.as_uri()}?mode=ro', uri=True)
        row = self.frame_dbs[db_file].execute('SELECT data FROM frames WHERE id=?', (db_id,)).fetchone()
        return cv2.imdecode(np.frombuffer(row[0], dtype=np.uint8), flags) if row else None

    def close(self):
        for db in self.frame_dbs.values():
            db.close()


class ReplayRos(Node):
    def __init__(self, data, enable_planner=False):
        super().__init__('avp_log_replay')
        self.enable_planner = enable_planner
        self.map_points = {}
        self.map_dirty = False
        self.last_map_publish = 0.0
        self.data = data
        self.odom_pub = self.create_publisher(Odometry, '/avp_replay/odom', 10)
        self.actual_pub = self.create_publisher(RosPath, '/avp_replay/actual_path', 10)
        self.target_path_pub = self.create_publisher(RosPath, '/avp_replay/target_path', 10)
        self.target_pub = self.create_publisher(PoseStamped, '/avp_replay/target', 10)
        self.vp_pub = self.create_publisher(MarkerArray, '/avp_replay/viewpoints', 10)
        self.route_pub = self.create_publisher(MarkerArray, '/avp_replay/route', 10)
        self.cloud_pub = self.create_publisher(PointCloud2, '/avp_replay/points', 10)
        self.planner_cloud_pub = self.create_publisher(PointCloud2, '/rgbd_camera/points', 10)
        self.center_pub = self.create_publisher(PoseStamped, '/avp/sweep_center', 10)
        self.planner_markers = [self.create_publisher(MarkerArray, topic, 10)
                                for topic in ('/avp/sqc', '/avp/patches', '/avp/viewpoints', '/avp/route_viz', '/surface/coverage')]
        self.tf_pub = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        self.last_depth, self.last_path = None, 0.0
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id, transform.child_frame_id = BASE, data.camera_frame
        if data.mode == 'sim':
            transform.transform.translation.x = 0.20
            transform.transform.translation.z = 0.05
            q = transform.transform.rotation
            q.w, q.x, q.y, q.z = (-0.5, 0.5, -0.5, 0.5)
        else:
            self._set_oak_camera_transform(transform)
        self.static_tf.sendTransform(transform)
        q, p = transform.transform.rotation, transform.transform.translation
        self.camera_R = quat_wxyz_to_R((q.w, q.x, q.y, q.z))
        self.camera_p = np.array([p.x, p.y, p.z])

    def _set_oak_camera_transform(self, transform):
        pos, rpy = MOUNT['position_m'], MOUNT['rpy_rad']
        transform.transform.translation.x = float(pos['x'])
        transform.transform.translation.y = float(pos['y'])
        transform.transform.translation.z = float(pos['z'])
        roll, pitch, yaw = (float(rpy[axis]) for axis in ('roll', 'pitch', 'yaw'))
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        optical = quat_wxyz_to_R((-0.5, 0.5, -0.5, 0.5))
        q = transform.transform.rotation
        q.w, q.x, q.y, q.z = R_to_quat_wxyz(Rz @ Ry @ Rx @ optical)

    def publish(self, t, publish_cloud=True):
        stamp = self.get_clock().now().to_msg()
        center = timed(self.data.sweep_center, t)
        if center:
            self.center_pub.publish(pose_message(center[1], stamp))
        actual = timed(self.data.odom, t)
        if actual:
            pose = pose_message(actual[1], stamp)
            msg = Odometry()
            msg.header, msg.child_frame_id, msg.pose.pose = pose.header, BASE, pose.pose
            self.odom_pub.publish(msg)
            tf = TransformStamped()
            tf.header, tf.child_frame_id = pose.header, BASE
            tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = actual[1][:3]
            tf.transform.rotation = pose.pose.orientation
            self.tf_pub.sendTransform(tf)
        target = timed(self.data.target, t)
        if target:
            self.target_pub.publish(pose_message(target[1], stamp))
        if abs(t - self.last_path) > 0.25 or t < self.last_path:
            self.last_path = t
            for series, publisher in ((self.data.odom, self.actual_pub),
                                      (self.data.target, self.target_path_pub)):
                path = RosPath()
                path.header.stamp, path.header.frame_id = stamp, MAP
                count = bisect_right([item[0] for item in series], t)
                step = max(1, count // 1500)
                path.poses = [pose_message(series[i][1], stamp) for i in range(0, count, step)]
                publisher.publish(path)
            self.vp_pub.publish(self._markers(timed(self.data.viewpoints, t), stamp, 'viewpoint'))
            self.route_pub.publish(self._markers(timed(self.data.route, t), stamp, 'route'))
        depth = timed(self.data.depth, t)
        if publish_cloud and depth and depth[1:3] != self.last_depth and self.data.camera:
            depth_pose = timed(self.data.odom, depth[0])
            if depth_pose is not None:
                image = (self.data.read_frame(depth[1], depth[2], cv2.IMREAD_UNCHANGED) if depth[1]
                         else np.load(self.data.folder / depth[2]))
                if image is not None and image.ndim == 2:
                    self.last_depth = depth[1:3]
                    step = DEPTH_STEP
                    z = image[::step, ::step].astype(np.float32) * depth[3]
                    v, u = np.mgrid[0:image.shape[0]:step, 0:image.shape[1]:step]
                    fx, fy, cx, cy = (self.data.camera[i] for i in (0, 4, 2, 5))
                    valid = np.isfinite(z) & (z > DEPTH_MIN_M) & (z < DEPTH_MAX_M)
                    xyz = np.column_stack(((u[valid] - cx) * z[valid] / fx,
                                           (v[valid] - cy) * z[valid] / fy, z[valid]))
                    values = depth_pose[1]
                    R = quat_wxyz_to_R(values[3:7])
                    # Crop only the accumulated visualization in the camera optical frame.
                    # Keep the horizontal FOV and optional planner input unchanged.
                    vertical_limit = math.tan(math.radians(DEPTH_VERTICAL_FOV_DEG / 2.0))
                    map_xyz = xyz[np.abs(xyz[:, 1]) <= xyz[:, 2] * vertical_limit]
                    if R is not None and len(map_xyz):
                        world = (map_xyz @ self.camera_R.T + self.camera_p) @ R.T + np.asarray(values[:3])
                        keys = np.floor(world / MAP_VOXEL_M).astype(np.int64)
                        _, indices = np.unique(keys, axis=0, return_index=True)
                        for i in indices:
                            self.map_points.setdefault(tuple(keys[i]), world[i].astype(np.float32))
                        self.map_dirty = True
                    if self.enable_planner:
                        header = Header(stamp=stamp, frame_id=self.data.camera_frame)
                        self.planner_cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, xyz.astype(np.float32)))
        now = time.monotonic()
        if self.map_dirty and now - self.last_map_publish >= MAP_PUBLISH_PERIOD_S:
            self.publish_map(stamp)

    def publish_map(self, stamp=None):
        if stamp is None:
            stamp = self.get_clock().now().to_msg()
        points = np.asarray(list(self.map_points.values()), dtype=np.float32).reshape(-1, 3)
        self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(Header(stamp=stamp, frame_id=MAP), points))
        self.map_dirty = False
        self.last_map_publish = time.monotonic()

    def reset_map(self):
        self.map_points.clear()
        self.last_depth = None
        self.publish_map()

    def clear_planner_markers(self):
        clear = MarkerArray(markers=[Marker(action=Marker.DELETEALL)])
        for publisher in self.planner_markers:
            publisher.publish(clear)

    def _markers(self, row, stamp, kind):
        result = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        result.markers.append(clear)
        if not row:
            return result
        for idx, values in enumerate(row[1]):
            marker = Marker()
            marker.header.stamp, marker.header.frame_id = stamp, MAP
            marker.ns, marker.id = kind, idx
            selected = kind == 'route' and idx == len(row[1]) - 1
            marker.type = Marker.ARROW if kind == 'viewpoint' or selected else Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose = pose_message(values, stamp).pose
            if kind == 'viewpoint':
                marker.scale.x, marker.scale.y, marker.scale.z = 0.18, 0.025, 0.025
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.8, 0.8, 0.8, 0.25
            elif selected:
                marker.scale.x, marker.scale.y, marker.scale.z = 0.55, 0.10, 0.10
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.1, 1.0, 0.2, 1.0
            else:
                marker.scale.x, marker.scale.y, marker.scale.z = 0.12, 0.12, 0.12
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.65, 0.10, 0.95
            result.markers.append(marker)
        return result


class ReplayWindow(QtWidgets.QMainWindow):
    def __init__(self, data, node, rviz, planner_cmd):
        super().__init__()
        self.data, self.node, self.rviz = data, node, rviz
        self.planner_cmd, self.planner = planner_cmd, None
        self.current, self.speed, self.playing = data.start, 1.0, False
        self.last_tick = time.monotonic()
        self.rgb_video = (cv2.VideoCapture(str(data.folder / 'rgb.mp4'))
                          if any(row[1] is None for row in data.rgb) else None)
        self.last_frame = -1
        self.setWindowTitle(f'AVP log replay | {data.folder.name}')
        self.resize(1450, 900)
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)
        controls = QtWidgets.QHBoxLayout()
        layout.addLayout(controls)
        self.play_button = QtWidgets.QPushButton()
        self.play_button.setIcon(self.style().standardIcon(QtWidgets.QStyle.SP_MediaPlay))
        self.play_button.setToolTip('재생 / 일시정지')
        self.play_button.clicked.connect(self.toggle)
        controls.addWidget(self.play_button)
        self.speed_box = QtWidgets.QComboBox()
        self.speed_box.addItems(['0.25x', '0.5x', '1x', '2x', '4x'])
        self.speed_box.setCurrentText('1x')
        self.speed_box.currentTextChanged.connect(lambda value: setattr(self, 'speed', float(value[:-1])))
        controls.addWidget(self.speed_box)
        self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider.setRange(0, int((data.end - data.start) * 1000))
        self.slider.valueChanged.connect(self.seek)
        self.slider.sliderReleased.connect(self.restart_planner)
        controls.addWidget(self.slider, 1)
        self.time_label = QtWidgets.QLabel()
        controls.addWidget(self.time_label)
        self.tabs = QtWidgets.QTabWidget()
        layout.addWidget(self.tabs, 1)
        self._make_plots()
        camera_tab = QtWidgets.QWidget()
        cam_layout = QtWidgets.QHBoxLayout(camera_tab)
        self.camera_label = QtWidgets.QLabel('영상 기록 없음')
        self.camera_label.setAlignment(QtCore.Qt.AlignCenter)
        self.camera_label.setMinimumSize(480, 300)
        cam_layout.addWidget(self.camera_label)
        self.tabs.addTab(camera_tab, '카메라')
        self.state_label = QtWidgets.QLabel()
        layout.addWidget(self.state_label)
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(50)
        self.seek(0)
        self.restart_planner()

    def _make_plots(self):
        pg.setConfigOptions(antialias=True)
        trajectory = pg.GraphicsLayoutWidget()
        xy = trajectory.addPlot(title='Map / ENU 경로', labels={'bottom': 'E [m]', 'left': 'N [m]'})
        xy.setAspectLocked(True)
        xy.addLegend()
        for series, name, color in ((self.data.odom, '기체', '#2a9d8f'),
                                    (self.data.target, 'PX4 목표', '#e76f51')):
            if series:
                xy.plot([r[1][0] for r in series], [r[1][1] for r in series],
                        pen=pg.mkPen(color, width=2), name=name)
        self.actual_dot = xy.plot([], [], pen=None, symbol='o', symbolBrush='#2a9d8f', symbolSize=12)
        self.target_dot = xy.plot([], [], pen=None, symbol='o', symbolBrush='#e76f51', symbolSize=11)
        trajectory.nextRow()
        height = trajectory.addPlot(title='고도', labels={'bottom': '기록 시간 [s]', 'left': 'U [m]'})
        height.addLegend()
        for series, name, color in ((self.data.odom, '기체', '#2a9d8f'),
                                    (self.data.target, 'PX4 목표', '#e76f51')):
            if series:
                height.plot([r[0]-self.data.start for r in series], [r[1][2] for r in series],
                            pen=pg.mkPen(color, width=2), name=name)
        self.height_line = pg.InfiniteLine(pos=0, angle=90, pen=pg.mkPen('#333333', width=1))
        height.addItem(self.height_line)
        self.tabs.addTab(trajectory, '경로')

        control = pg.GraphicsLayoutWidget()
        for title, source, indices, unit in (('각속도 측정', 'rates', (0, 1, 2), 'rad/s'),
                                             ('각속도 목표', 'rates_sp', (0, 1, 2), 'rad/s'),
                                             ('모터 출력', 'motors', (0, 1, 2, 3), 'norm'),
                                             ('서보 출력', 'servos', (0, 1, 2, 3, 4, 5), 'norm')):
            plot = control.addPlot(title=title, labels={'bottom': '기록 시간 [s]', 'left': unit})
            plot.addLegend()
            series = self.data.signals[source]
            for i in indices:
                field = 'xyz' if source in ('rates', 'rates_sp') else 'control'
                ts, values = [], []
                for t, row in series:
                    items = row.get(field, [])
                    if i < len(items) and items[i] is not None and np.isfinite(items[i]):
                        ts.append(t-self.data.start)
                        values.append(items[i])
                plot.plot(ts, values, pen=pg.mkPen(pg.intColor(i, hues=6), width=1.3),
                          name=(('roll', 'pitch', 'yaw')[i] if source in ('rates', 'rates_sp') else str(i+1)))
            line = pg.InfiniteLine(pos=0, angle=90, pen=pg.mkPen('#333333', width=1))
            plot.addItem(line)
            if not hasattr(self, 'control_lines'):
                self.control_lines = []
            self.control_lines.append(line)
            control.nextRow()
        self.tabs.addTab(control, '제어')

    def restart_planner(self):
        self.node.reset_map()
        if not self.planner_cmd:
            return
        stop_child(self.planner)
        self.node.clear_planner_markers()
        self.planner = subprocess.Popen(self.planner_cmd, start_new_session=True)
        self.last_tick = time.monotonic()

    def toggle(self):
        if self.planner and self.planner.poll() is not None:
            self.restart_planner()
        self.playing = not self.playing
        icon = QtWidgets.QStyle.SP_MediaPause if self.playing else QtWidgets.QStyle.SP_MediaPlay
        self.play_button.setIcon(self.style().standardIcon(icon))
        self.last_tick = time.monotonic()

    def seek(self, milliseconds):
        self.current = self.data.start + milliseconds / 1000.0
        self._render()

    def tick(self):
        now = time.monotonic()
        if self.playing:
            target = min(self.data.end, self.current + (now-self.last_tick) * self.speed)
            self.slider.setValue(int((target-self.data.start) * 1000))
            if target >= self.data.end:
                self.toggle()
        self.last_tick = now
        self.node.publish(self.current, publish_cloud=not self.slider.isSliderDown())

    def _render(self):
        t = self.current
        self.time_label.setText(f'{t-self.data.start:6.1f} / {self.data.end-self.data.start:.1f} s')
        actual, target = timed(self.data.odom, t), timed(self.data.target, t)
        self.actual_dot.setData([actual[1][0]], [actual[1][1]]) if actual else self.actual_dot.setData([], [])
        self.target_dot.setData([target[1][0]], [target[1][1]]) if target else self.target_dot.setData([], [])
        self.height_line.setPos(t-self.data.start)
        for line in self.control_lines:
            line.setPos(t-self.data.start)
        status = timed(self.data.signals['status'], t)
        state = status[1] if status else {}
        error = np.linalg.norm(np.asarray(actual[1][:3])-target[1][:3]) if actual and target else float('nan')
        self.state_label.setText(f'기체/목표 오차 {error:.2f} m   |   nav_state {state.get("nav_state", "-")}   '
                                 f'|   armed {state.get("arming_state", "-")}   |   '
                                 f'AVP 시점 {len((timed(self.data.viewpoints, t) or (None, []))[1])}개')
        rgb = timed(self.data.rgb, t)
        if rgb and rgb[1:] != self.last_frame:
            if rgb[1]:
                frame = self.data.read_frame(rgb[1], rgb[2], cv2.IMREAD_REDUCED_COLOR_2)
                ok = frame is not None
            else:
                self.rgb_video.set(cv2.CAP_PROP_POS_FRAMES, rgb[2])
                ok, frame = self.rgb_video.read()
            if ok:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                qimage = QtGui.QImage(frame.data, frame.shape[1], frame.shape[0], frame.strides[0], QtGui.QImage.Format_RGB888)
                self.camera_label.setPixmap(QtGui.QPixmap.fromImage(qimage.copy()).scaled(
                    self.camera_label.size(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation))
            self.last_frame = rgb[1:]

    def closeEvent(self, event):
        self.timer.stop()
        if self.rgb_video:
            self.rgb_video.release()
        self.data.close()
        for process in (self.planner, self.rviz):
            stop_child(process)
        super().closeEvent(event)


def main():
    parser = argparse.ArgumentParser(description='Replay AVP flight and headless JSONL on a shared timeline')
    parser.add_argument('log', nargs='?', help='flight_* folder or *_headless.jsonl (default: latest flight)')
    parser.add_argument('other', nargs='?', help='optional matching headless JSONL or flight_* folder')
    parser.add_argument('--no-rviz', action='store_true')
    parser.add_argument('--no-planner', action='store_true', help='disable planner (default)')
    parser.add_argument('--planner', action='store_true', help='also run AVP planner')
    parser.add_argument('--check', action='store_true', help='validate inputs without opening windows')
    args = parser.parse_args()
    flight, headless = select_logs(args.log, args.other)
    data = FlightData(flight, headless)
    print(f'Flight: {flight}\nHeadless: {headless or "not found"}\nDuration: {data.end-data.start:.1f} s', flush=True)
    print(f'Odometry: {len(data.odom)}, PX4 targets: {len(data.target)}, '
          f'RGB: {len(data.rgb)}, depth: {len(data.depth)}, '
          f'AVP viewpoints: {len(data.viewpoints)}, route: {len(data.route)}', flush=True)
    if args.check:
        return 0
    os.environ['ROS_DOMAIN_ID'] = os.environ.get('AVP_REPLAY_DOMAIN_ID', '113')
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    print(f'Offline replay ROS_DOMAIN_ID={os.environ["ROS_DOMAIN_ID"]}', flush=True)
    app = QtWidgets.QApplication(sys.argv[:1])
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    signal.signal(signal.SIGTERM, lambda *_: app.quit())
    rclpy.init(args=None, signal_handler_options=SignalHandlerOptions.NO)
    node = ReplayRos(data, enable_planner=args.planner and not args.no_planner)
    rviz, window = None, None
    planner_cmd = None if not node.enable_planner else [
        'ros2', 'run', 'px4_hummingbird_avp', 'lics_avp_planner.py', '--ros-args',
        '--params-file', str(CONFIG / ('avp_outdoor.yaml' if data.sweep_center else 'avp_indoor.yaml')),
        '-p', 'require_path_ready:=false']
    try:
        if not args.no_rviz:
            rviz = subprocess.Popen(['rviz2', '-d', str(CONFIG / 'avp_replay.rviz')], start_new_session=True)
        window = ReplayWindow(data, node, rviz, planner_cmd)
        window.show()
        return app.exec_()
    finally:
        if window:
            window.close()
        else:
            stop_child(rviz)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())