#!/usr/bin/env python3
"""Record camera data and commanded AVP setpoints in simulation or flight."""

import csv
from collections import deque
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import signal
import sqlite3
import subprocess
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Float32
from px4_msgs.msg import TrajectorySetpoint6dof
from tf2_ros import Buffer, TransformException, TransformListener


def stamp_ns(header):
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


def pose_values(pose):
    p = pose.position
    q = pose.orientation
    return [p.x, p.y, p.z, q.w, q.x, q.y, q.z]


class FlightRecorder(Node):
    def __init__(self):
        super().__init__('avp_flight_recorder')
        root = Path(str(self.declare_parameter('output_dir', '~/Desktop/px4_hummingbird/src/px4_hummingbird_avp/logs').value)).expanduser()
        self.mode = str(self.declare_parameter('mode', 'real').value)
        self.color_topic = str(self.declare_parameter('color_topic', '/oak/rgb/image_raw').value)
        self.depth_topic = str(self.declare_parameter('depth_topic', '/oak/stereo/image_raw').value)
        self.camera_info_topic = str(self.declare_parameter('camera_info_topic', '/oak/stereo/camera_info').value)
        self.color_camera_info_topic = str(self.declare_parameter('color_camera_info_topic', '/oak/rgb/camera_info').value)
        self.record_fps = float(self.declare_parameter('record_fps', 10.0).value)
        if self.mode not in ('sim', 'real'):
            raise ValueError('mode must be sim or real')
        if self.record_fps <= 0:
            raise ValueError('record_fps must be positive')
        root.mkdir(parents=True, exist_ok=True)
        self.run_dir = root / datetime.now(timezone.utc).strftime('flight_%Y%m%d_%H%M%S_%fZ')
        self.run_dir.mkdir()
        self.events = (self.run_dir / 'events.jsonl').open('w', encoding='utf-8', buffering=1)
        self.rgb_frames_file = (self.run_dir / 'rgb_frames.jsonl').open('w', encoding='utf-8', buffering=1)
        self.setpoint_file = (self.run_dir / 'setpoints.csv').open('w', newline='', encoding='utf-8')
        self.odom_file = (self.run_dir / 'odometry.csv').open('w', newline='', encoding='utf-8')
        self.setpoint_csv = csv.writer(self.setpoint_file)
        self.odom_csv = csv.writer(self.odom_file)
        pose_columns = ['x', 'y', 'z', 'qw', 'qx', 'qy', 'qz']
        self.setpoint_csv.writerow(['wall_time_ns', 'stamp_ns', 'topic', 'frame_id'] + pose_columns +
                                   ['vx', 'vy', 'vz', 'ax', 'ay', 'az', 'jx', 'jy', 'jz', 'wx', 'wy', 'wz'])
        self.odom_csv.writerow(['wall_time_ns', 'stamp_ns', 'frame_id'] + pose_columns + ['vx', 'vy', 'vz'])
        self.frame_counts = {'rgb': 0, 'depth': 0, 'cloud': 0, 'camera_info': 0,
                             'rgb_pose': 0, 'rgb_pose_missing': 0}
        self.frame_slots = {}
        self.frame_start_ns = {}
        self.db_index = 0
        self.db = None
        self.db_file = None
        self._open_db()
        self.rgb_video = None
        self.video_size = None
        self.video_frames = 0
        self.video_failed = False
        self.pending_rgb = deque()
        self.last_check = time.monotonic()
        self.last_check_counts = {'rgb': 0, 'depth': 0}
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.closed = False
        self._event('meta', mode=self.mode, record_fps=self.record_fps,
                    color_topic=self.color_topic, depth_topic=self.depth_topic,
                    camera_info_topic=self.camera_info_topic,
                    color_camera_info_topic=self.color_camera_info_topic,
                    rgb_format='jpeg+mp4_h264', rgb_quality=95, rgb_resolution='source', mp4_crf=16,
                    depth_format='png16_mm', db_limit_bytes=1_000_000_000,
                    rgb_rotation_deg=180,
                    rgb_pose_frame='map', rgb_pose_order=['x', 'y', 'z', 'qw', 'qx', 'qy', 'qz'])

        self.create_subscription(Image, self.color_topic, self.color_cb, qos_profile_sensor_data)
        self.create_subscription(Image, self.depth_topic, self.depth_cb, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.camera_info_topic,
                                 lambda msg: self.camera_info_cb(self.camera_info_topic, msg), qos_profile_sensor_data)
        if self.color_camera_info_topic != self.camera_info_topic:
            self.create_subscription(CameraInfo, self.color_camera_info_topic,
                                     lambda msg: self.camera_info_cb(self.color_camera_info_topic, msg), qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/rgbd_camera/points', self.cloud_cb,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, '/lics/odom', self.odom_cb, qos_profile_sensor_data)
        for topic in ('/avp/base_setpoint', '/avp/setpoint'):
            self.create_subscription(PoseStamped, topic,
                                     lambda msg, name=topic: self.setpoint_cb(name, msg),
                                     qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/avp/sweep_center',
                                 lambda msg: self._event('/avp/sweep_center', pose=pose_values(msg.pose)), 10)
        for topic in ('/avp/viewpoints_data', '/avp/route'):
            self.create_subscription(PoseArray, topic,
                                     lambda msg, name=topic: self.poses_cb(name, msg),
                                     qos_profile_sensor_data)
        self.create_subscription(TrajectorySetpoint6dof, '/fmu/in/trajectory_setpoint6dof',
                                 self.trajectory_cb, 10)
        for topic in ('/avp/radius_delta', '/avp/front_distance'):
            self.create_subscription(Float32, topic,
                                     lambda msg, name=topic: self._event(name, value=float(msg.data)), 10)
        self.create_timer(1.0, self.flush)
        self.create_timer(10.0, self.check_recording)
        self.create_timer(0.05, self.resolve_rgb_tf)
        self.get_logger().warn(f'Recording source-size H.264 MP4, JPEG q95 and 16-bit mm PNG at up to {self.record_fps:g} fps: {self.run_dir}')

    def _event(self, topic, **data):
        row = {'topic': topic, 'wall_time_ns': time.time_ns()}
        row.update(data)
        self.events.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + '\n')

    def _open_db(self):
        self.db_index += 1
        self.db_file = f'frames_{self.db_index:04d}.sqlite3'
        self.db = sqlite3.connect(self.run_dir / self.db_file)
        self.db.execute('PRAGMA journal_mode=DELETE')
        self.db.execute('PRAGMA synchronous=NORMAL')
        self.db.execute('CREATE TABLE frames (id INTEGER PRIMARY KEY, kind TEXT NOT NULL, stamp_ns INTEGER NOT NULL, data BLOB NOT NULL)')
        self.db.commit()

    def _store_frame(self, kind, image, stamp, ext, options):
        start = self.frame_start_ns.setdefault(kind, stamp)
        slot = round((stamp - start) * self.record_fps / 1_000_000_000)
        if slot <= self.frame_slots.get(kind, -1):
            return None
        ok, encoded = cv2.imencode(ext, image, options)
        if not ok:
            raise RuntimeError(f'Cannot encode {kind} frame')
        payload = encoded.tobytes()
        pages = self.db.execute('PRAGMA page_count').fetchone()[0]
        page_size = self.db.execute('PRAGMA page_size').fetchone()[0]
        if pages * page_size + len(payload) + 8192 > 1_000_000_000:
            self.db.commit()
            self.db.close()
            self._open_db()
        frame_id = self.db.execute('INSERT INTO frames (kind, stamp_ns, data) VALUES (?, ?, ?)',
                                   (kind, stamp, sqlite3.Binary(payload))).lastrowid
        self.frame_slots[kind] = slot
        self.frame_counts[kind] += 1
        return self.db_file, frame_id

    def _write_rgb_video(self, frame):
        if self.video_failed:
            return None
        height, width = frame.shape[:2]
        if self.video_size is None:
            self.video_size = (width, height)
            try:
                self.rgb_video = subprocess.Popen([
                    'ffmpeg', '-hide_banner', '-loglevel', 'error', '-y',
                    '-f', 'rawvideo', '-pixel_format', 'bgr24', '-video_size', f'{width}x{height}',
                    '-framerate', str(self.record_fps), '-i', '-', '-an', '-c:v', 'libx264',
                    '-preset', 'veryfast', '-crf', '16', '-pix_fmt', 'yuv420p',
                    '-vf', 'pad=ceil(iw/2)*2:ceil(ih/2)*2', '-movflags', '+faststart',
                    str(self.run_dir / 'rgb.mp4')], stdin=subprocess.PIPE, stderr=subprocess.DEVNULL)
            except OSError as exc:
                self.video_failed = True
                self.get_logger().error(f'Cannot start MP4 encoder: {exc}')
                return None
        if (width, height) != self.video_size:
            self.get_logger().warn('RGB resolution changed; resizing MP4 frames to the first frame size.', once=True)
            frame = cv2.resize(frame, self.video_size)
        try:
            self.rgb_video.stdin.write(np.ascontiguousarray(frame).tobytes())
        except OSError as exc:
            self.video_failed = True
            self.get_logger().error(f'RGB MP4 encoder stopped: {exc}')
            self.rgb_video.stdin.close()
            self.rgb_video.wait(timeout=5)
            self.rgb_video = None
            return None
        self.video_frames += 1
        return self.video_frames

    def resolve_rgb_tf(self, force=False):
        while self.pending_rgb:
            source, deadline_ns = self.pending_rgb[0]
            row = dict(source, pose_frame_id='map')
            try:
                if not source['frame_id'] or source['stamp_ns'] <= 0:
                    raise ValueError('RGB image has no valid frame_id or timestamp')
                tf = self.tf_buffer.lookup_transform(
                    'map', source['frame_id'].lstrip('/'), Time(nanoseconds=source['stamp_ns']))
                p, q = tf.transform.translation, tf.transform.rotation
                pose = [p.x, p.y, p.z, q.w, q.x, q.y, q.z]
                if not all(math.isfinite(value) for value in pose):
                    raise ValueError('RGB camera TF contains non-finite values')
                row.update(tf_status='ok', tf_stamp_ns=stamp_ns(tf.header), pose_map_camera=pose)
                self.frame_counts['rgb_pose'] += 1
            except (TransformException, ValueError) as exc:
                if not force and time.monotonic_ns() < deadline_ns:
                    break
                row.update(tf_status='missing', tf_stamp_ns=None, pose_map_camera=None,
                           tf_error=str(exc))
                self.frame_counts['rgb_pose_missing'] += 1
                if self.frame_counts['rgb_pose_missing'] == 1 or self.frame_counts['rgb_pose_missing'] % 100 == 0:
                    self.get_logger().warn(f'RGB camera TF unavailable for frame {source["db_id"]}: {exc}')
            self.rgb_frames_file.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + '\n')
            self.pending_rgb.popleft()

    def color_cb(self, msg):
        if msg.encoding not in ('bgr8', 'rgb8', 'mono8'):
            self.get_logger().warn(f'Unsupported color encoding: {msg.encoding}', once=True)
            return
        channels = 1 if msg.encoding == 'mono8' else 3
        frame = np.ndarray((msg.height, msg.width, channels), dtype=np.uint8,
                           buffer=msg.data, strides=(msg.step, channels, 1))
        if msg.encoding == 'rgb8':
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        elif msg.encoding == 'mono8':
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        frame = cv2.rotate(frame, cv2.ROTATE_180)
        captured_ns, received_ns = stamp_ns(msg.header), time.time_ns()
        stored = self._store_frame('rgb', frame, captured_ns, '.jpg', [cv2.IMWRITE_JPEG_QUALITY, 95])
        if stored is None:
            return
        db_file, db_id = stored
        video_frame = self._write_rgb_video(frame)
        source = {'db_file': db_file, 'db_id': db_id, 'stamp_ns': captured_ns,
                  'wall_time_ns': received_ns, 'frame_id': msg.header.frame_id,
                  'width': msg.width, 'height': msg.height, 'video_frame': video_frame}
        self.pending_rgb.append((source, time.monotonic_ns() + 2_000_000_000))
        self.resolve_rgb_tf()
        self._event(self.color_topic, wall_time_ns=received_ns, stamp_ns=captured_ns,
                    frame_id=msg.header.frame_id, db_file=db_file, db_id=db_id,
                    video_frame=video_frame, width=msg.width, height=msg.height)

    def depth_cb(self, msg):
        if msg.encoding == '16UC1':
            dtype = np.dtype('>u2' if msg.is_bigendian else '<u2')
            depth = np.ndarray((msg.height, msg.width), dtype=dtype,
                               buffer=msg.data, strides=(msg.step, dtype.itemsize))
            depth_mm = np.ascontiguousarray(depth.astype(np.uint16))
        elif msg.encoding == '32FC1':
            dtype = np.dtype('>f4' if msg.is_bigendian else '<f4')
            depth = np.ndarray((msg.height, msg.width), dtype=dtype,
                               buffer=msg.data, strides=(msg.step, dtype.itemsize))
            valid = np.isfinite(depth) & (depth > 0) & (depth <= 65.535)
            depth_mm = np.where(valid, np.rint(depth * 1000.0), 0).astype(np.uint16)
        else:
            self.get_logger().warn(f'Unsupported depth encoding: {msg.encoding}', once=True)
            return
        if not np.any(depth_mm):
            self.get_logger().warn('Depth frame has no valid pixels', once=True)
        captured_ns = stamp_ns(msg.header)
        stored = self._store_frame('depth', depth_mm, captured_ns, '.png', [cv2.IMWRITE_PNG_COMPRESSION, 3])
        if stored is None:
            return
        db_file, db_id = stored
        self._event(self.depth_topic, stamp_ns=captured_ns, frame_id=msg.header.frame_id,
                    db_file=db_file, db_id=db_id, encoding='16UC1', depth_scale_m=0.001,
                    width=msg.width, height=msg.height)

    def camera_info_cb(self, topic, msg):
        self.frame_counts['camera_info'] += 1
        self._event(topic, stamp_ns=stamp_ns(msg.header),
                    frame_id=msg.header.frame_id, width=msg.width, height=msg.height,
                    distortion_model=msg.distortion_model, d=list(msg.d), k=list(msg.k),
                    r=list(msg.r), p=list(msg.p), binning_x=msg.binning_x,
                    binning_y=msg.binning_y,
                    roi={'x_offset': msg.roi.x_offset, 'y_offset': msg.roi.y_offset,
                         'height': msg.roi.height, 'width': msg.roi.width,
                         'do_rectify': msg.roi.do_rectify})

    def cloud_cb(self, msg):
        self.frame_counts['cloud'] += 1
        self._event('/rgbd_camera/points', stamp_ns=stamp_ns(msg.header),
                    frame_id=msg.header.frame_id, points=msg.width * msg.height)

    def setpoint_cb(self, topic, msg):
        wall_time_ns = time.time_ns()
        values = pose_values(msg.pose)
        self.setpoint_csv.writerow([wall_time_ns, stamp_ns(msg.header), topic,
                                    msg.header.frame_id] + values + [''] * 12)
        self._event(topic, stamp_ns=stamp_ns(msg.header), frame_id=msg.header.frame_id,
                    pose=values)

    def trajectory_cb(self, msg):
        values = list(msg.position) + list(msg.quaternion)
        self.setpoint_csv.writerow([time.time_ns(), int(msg.timestamp) * 1000,
                                    '/fmu/in/trajectory_setpoint6dof', 'local_ned'] + values +
                                   list(msg.velocity) + list(msg.acceleration) + list(msg.jerk) +
                                   list(msg.angular_velocity))

    def poses_cb(self, topic, msg):
        self._event(topic, stamp_ns=stamp_ns(msg.header), frame_id=msg.header.frame_id,
                    poses=[pose_values(pose) for pose in msg.poses])

    def odom_cb(self, msg):
        values = pose_values(msg.pose.pose)
        velocity = msg.twist.twist.linear
        self.odom_csv.writerow([time.time_ns(), stamp_ns(msg.header), msg.header.frame_id]
                              + values + [velocity.x, velocity.y, velocity.z])

    def check_recording(self):
        now = time.monotonic()
        elapsed = max(1.0, now - self.last_check)
        rates = {kind: (self.frame_counts[kind] - self.last_check_counts[kind]) / elapsed
                 for kind in ('rgb', 'depth')}
        self.last_check, self.last_check_counts = now, {kind: self.frame_counts[kind] for kind in rates}
        if min(rates.values()) < 0.8 * self.record_fps:
            self.get_logger().warn(f'Recording rate low: RGB={rates["rgb"]:.1f} fps, depth={rates["depth"]:.1f} fps')
        else:
            self.get_logger().info(f'Recording OK: RGB={rates["rgb"]:.1f} fps, depth={rates["depth"]:.1f} fps, {self.db_file}')

    def flush(self):
        self.db.commit()
        self.events.flush()
        self.rgb_frames_file.flush()
        self.setpoint_file.flush()
        self.odom_file.flush()

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.resolve_rgb_tf(force=True)
        self._event('end', frame_counts=self.frame_counts)
        self.flush()
        if self.rgb_video is not None:
            self.rgb_video.stdin.close()
            if self.rgb_video.wait(timeout=20) != 0:
                self.get_logger().error('RGB MP4 encoder exited with an error')
        self.db.close()
        self.events.close()
        self.rgb_frames_file.close()
        self.setpoint_file.close()
        self.odom_file.close()
        print(f'Saved flight recording: {self.run_dir} {self.frame_counts}', flush=True)


def main():
    rclpy.init()
    node = None
    try:
        node = FlightRecorder()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
