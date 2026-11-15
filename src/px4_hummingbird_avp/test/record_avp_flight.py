#!/usr/bin/env python3
"""Record camera data and commanded AVP setpoints in simulation or flight."""

import csv
from datetime import datetime, timezone
import json
from pathlib import Path
import signal
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from px4_msgs.msg import TrajectorySetpoint6dof


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
        self.video_fps = float(self.declare_parameter('video_fps', 10.0).value)
        self.depth_max_m = float(self.declare_parameter('depth_max_m', 6.0).value)
        if self.mode not in ('sim', 'real'):
            raise ValueError('mode must be sim or real')
        if self.video_fps <= 0 or self.depth_max_m <= 0:
            raise ValueError('video_fps and depth_max_m must be positive')
        root.mkdir(parents=True, exist_ok=True)
        self.run_dir = root / datetime.now(timezone.utc).strftime('flight_%Y%m%d_%H%M%S_%fZ')
        self.run_dir.mkdir()
        self.raw_depth_dir = self.run_dir / 'depth_raw'
        self.raw_depth_dir.mkdir()
        self.events = (self.run_dir / 'events.jsonl').open('w', encoding='utf-8', buffering=1)
        self.setpoint_file = (self.run_dir / 'setpoints.csv').open('w', newline='', encoding='utf-8')
        self.odom_file = (self.run_dir / 'odometry.csv').open('w', newline='', encoding='utf-8')
        self.setpoint_csv = csv.writer(self.setpoint_file)
        self.odom_csv = csv.writer(self.odom_file)
        pose_columns = ['x', 'y', 'z', 'qw', 'qx', 'qy', 'qz']
        self.setpoint_csv.writerow(['wall_time_ns', 'stamp_ns', 'topic', 'frame_id'] + pose_columns +
                                   ['vx', 'vy', 'vz', 'ax', 'ay', 'az', 'jx', 'jy', 'jz', 'wx', 'wy', 'wz'])
        self.odom_csv.writerow(['wall_time_ns', 'stamp_ns', 'frame_id'] + pose_columns + ['vx', 'vy', 'vz'])
        self.writers = {}
        self.frame_counts = {'rgb': 0, 'depth': 0, 'raw_depth': 0, 'cloud': 0, 'camera_info': 0}
        self.video_frames = {'rgb': 0, 'depth': 0}
        self.video_start_ns = {}
        self.last_video_frame = {}
        self.closed = False
        self._event('meta', mode=self.mode, video_fps=self.video_fps, depth_max_m=self.depth_max_m,
                    color_topic=self.color_topic, depth_topic=self.depth_topic,
                    camera_info_topic=self.camera_info_topic)

        self.create_subscription(Image, self.color_topic, self.color_cb, qos_profile_sensor_data)
        self.create_subscription(Image, self.depth_topic, self.depth_cb, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_cb,
                                 qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/rgbd_camera/points', self.cloud_cb,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, '/lics/odom', self.odom_cb, qos_profile_sensor_data)
        for topic in ('/avp/base_setpoint', '/avp/setpoint'):
            self.create_subscription(PoseStamped, topic,
                                     lambda msg, name=topic: self.setpoint_cb(name, msg),
                                     qos_profile_sensor_data)
        for topic in ('/avp/viewpoints_data', '/avp/route'):
            self.create_subscription(PoseArray, topic,
                                     lambda msg, name=topic: self.poses_cb(name, msg),
                                     qos_profile_sensor_data)
        self.create_subscription(TrajectorySetpoint6dof, '/fmu/in/trajectory_setpoint6dof',
                                 self.trajectory_cb, 10)
        self.create_timer(1.0, self.flush)
        self.get_logger().info(f'Flight recording: {self.run_dir}')

    def _event(self, topic, **data):
        row = {'topic': topic, 'wall_time_ns': time.time_ns()}
        row.update(data)
        self.events.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + '\n')

    def _write_video(self, name, frame, timestamp_ns):
        writer = self.writers.get(name)
        if writer is None:
            path = self.run_dir / f'{name}.mp4'
            writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*'mp4v'),
                                     self.video_fps, (frame.shape[1], frame.shape[0]))
            if not writer.isOpened():
                raise RuntimeError(f'Cannot open video file: {path}')
            self.writers[name] = writer
        self.frame_counts[name] += 1
        start_ns = self.video_start_ns.setdefault(name, timestamp_ns)
        target_frame = round((timestamp_ns - start_ns) * self.video_fps / 1_000_000_000)
        if target_frame < self.video_frames[name]:
            return None
        while self.video_frames[name] < target_frame:
            writer.write(self.last_video_frame[name])
            self.video_frames[name] += 1
        writer.write(frame)
        self.video_frames[name] += 1
        self.last_video_frame[name] = frame.copy()
        return self.video_frames[name]

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
        video_frame = self._write_video('rgb', frame, stamp_ns(msg.header))
        self._event(self.color_topic, stamp_ns=stamp_ns(msg.header),
                    frame_id=msg.header.frame_id, video_frame=video_frame)

    def depth_cb(self, msg):
        if msg.encoding == '16UC1':
            dtype = np.dtype('>u2' if msg.is_bigendian else '<u2')
            scale = 0.001
        elif msg.encoding == '32FC1':
            dtype = np.dtype('>f4' if msg.is_bigendian else '<f4')
            scale = 1.0
        else:
            self.get_logger().warn(f'Unsupported depth encoding: {msg.encoding}', once=True)
            return
        depth = np.ndarray((msg.height, msg.width), dtype=dtype,
                           buffer=msg.data, strides=(msg.step, dtype.itemsize))
        self.frame_counts['raw_depth'] += 1
        raw_path = self.raw_depth_dir / f'{self.frame_counts["raw_depth"]:06d}.npy'
        np.save(raw_path, depth)
        distance = depth.astype(np.float32) * scale
        valid = np.isfinite(distance) & (distance > 0)
        image = np.clip(distance / self.depth_max_m * 255.0, 0, 255)
        image[~valid] = 0
        preview = cv2.applyColorMap(image.astype(np.uint8), cv2.COLORMAP_TURBO)
        preview[~valid] = 0
        video_frame = self._write_video('depth', preview, stamp_ns(msg.header))
        self._event(self.depth_topic, stamp_ns=stamp_ns(msg.header),
                    frame_id=msg.header.frame_id, video_frame=video_frame,
                    raw_path=str(raw_path.relative_to(self.run_dir)),
                    encoding=msg.encoding, depth_scale_m=scale, width=msg.width,
                    height=msg.height, step=msg.step, is_bigendian=bool(msg.is_bigendian))

    def camera_info_cb(self, msg):
        self.frame_counts['camera_info'] += 1
        self._event(self.camera_info_topic, stamp_ns=stamp_ns(msg.header),
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

    def flush(self):
        self.events.flush()
        self.setpoint_file.flush()
        self.odom_file.flush()

    def close(self):
        if self.closed:
            return
        self.closed = True
        self._event('end', frame_counts=self.frame_counts, video_frames=self.video_frames)
        for writer in self.writers.values():
            writer.release()
        self.flush()
        self.events.close()
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
