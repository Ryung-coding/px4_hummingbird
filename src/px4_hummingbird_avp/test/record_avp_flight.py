#!/usr/bin/env python3
"""Record camera data and commanded AVP setpoints in simulation or flight."""

from collections import deque
from datetime import datetime, timezone
import json
import math
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
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Float32
from px4_msgs.msg import TrajectorySetpoint6dof
from tf2_ros import Buffer, TransformException, TransformListener


def stamp_ns(header):
    return (
        int(header.stamp.sec) * 1_000_000_000
        + int(header.stamp.nanosec)
    )


def pose_values(pose):
    p = pose.position
    q = pose.orientation

    return [
        p.x, p.y, p.z,
        q.w, q.x, q.y, q.z
    ]


def finite_values(values):
    return [
        float(value) if math.isfinite(value) else None
        for value in values
    ]


class FlightRecorder(Node):

    def __init__(self):
        super().__init__('avp_flight_recorder')

        root = Path(str(self.declare_parameter(
            'output_dir',
            '~/Desktop/px4_hummingbird/src/px4_hummingbird_avp/logs'
        ).value)).expanduser()

        self.mode = str(self.declare_parameter(
            'mode', 'real'
        ).value)

        self.tilting_mode = str(self.declare_parameter(
            'tilting_mode', ''
        ).value)

        if self.tilting_mode not in ('', 'on', 'off'):
            raise ValueError(
                'tilting_mode must be on, off, or empty'
            )

        self.color_topic = str(self.declare_parameter(
            'color_topic',
            '/oak/rgb/image_raw'
        ).value)

        self.depth_topic = str(self.declare_parameter(
            'depth_topic',
            '/oak/stereo/image_raw'
        ).value)

        self.camera_info_topic = str(self.declare_parameter(
            'camera_info_topic',
            '/oak/stereo/camera_info'
        ).value)

        self.color_camera_info_topic = str(
            self.declare_parameter(
                'color_camera_info_topic',
                '/oak/rgb/camera_info'
            ).value
        )

        self.video_fps = float(self.declare_parameter(
            'video_fps', 10.0
        ).value)

        self.depth_max_m = float(self.declare_parameter(
            'depth_max_m', 6.0
        ).value)

        if self.mode not in ('sim', 'real'):
            raise ValueError('mode must be sim or real')

        if self.video_fps <= 0 or self.depth_max_m <= 0:
            raise ValueError(
                'video_fps and depth_max_m must be positive'
            )

        # Recording directory

        root.mkdir(parents=True, exist_ok=True)

        self.run_dir = root / (
            datetime.now(timezone.utc).strftime(
                'flight_%Y%m%d_%H%M%S_%fZ'
            )
            + (
                f'_{self.tilting_mode}'
                if self.tilting_mode else ''
            )
        )

        self.run_dir.mkdir()

        self.raw_depth_dir = self.run_dir / 'depth_raw'
        self.raw_depth_dir.mkdir()

        self.events = (
            self.run_dir / 'events.jsonl'
        ).open(
            'w',
            encoding='utf-8',
            buffering=1
        )

        self.rgb_frames_file = (
            self.run_dir / 'rgb_frames.jsonl'
        ).open(
            'w',
            encoding='utf-8',
            buffering=1
        )

        # Recording state

        self.writers = {}

        self.frame_counts = {
            'rgb': 0,
            'depth': 0,
            'raw_depth': 0,
            'cloud': 0,
            'camera_info': 0,
            'rgb_pose': 0,
            'rgb_pose_missing': 0,
        }

        self.video_frames = {
            'rgb': 0,
            'depth': 0,
        }

        self.video_start_ns = {}
        self.last_video_frame = {}

        self.pending_rgb = deque()
        self.last_rgb_source = None

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        self.closed = False

        # Recording metadata

        self._event(
            'meta',
            mode=self.mode,
            tilting_mode=self.tilting_mode,
            video_fps=self.video_fps,
            depth_max_m=self.depth_max_m,
            rgb_codec='mp4v',
            rgb_resolution='source',
            color_topic=self.color_topic,
            depth_topic=self.depth_topic,
            camera_info_topic=self.camera_info_topic,
            color_camera_info_topic=self.color_camera_info_topic,
            rgb_video_rotation_deg=180,
            rgb_pose_frame='map',
            rgb_pose_order=[
                'x', 'y', 'z',
                'qw', 'qx', 'qy', 'qz'
            ]
        )

        # Camera subscriptions

        self.create_subscription(
            Image,
            self.color_topic,
            self.color_cb,
            qos_profile_sensor_data
        )

        self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_cb,
            qos_profile_sensor_data
        )

        self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            lambda msg: self.camera_info_cb(
                self.camera_info_topic, msg
            ),
            qos_profile_sensor_data
        )

        if self.color_camera_info_topic != self.camera_info_topic:
            self.create_subscription(
                CameraInfo,
                self.color_camera_info_topic,
                lambda msg: self.camera_info_cb(
                    self.color_camera_info_topic, msg
                ),
                qos_profile_sensor_data
            )

        self.create_subscription(
            PointCloud2,
            '/rgbd_camera/points',
            self.cloud_cb,
            qos_profile_sensor_data
        )

        # Odometry and AVP subscriptions

        self.create_subscription(
            Odometry,
            '/lics/odom',
            self.odom_cb,
            qos_profile_sensor_data
        )

        for topic in (
            '/avp/base_setpoint',
            '/avp/setpoint'
        ):
            self.create_subscription(
                PoseStamped,
                topic,
                lambda msg, name=topic: self.setpoint_cb(
                    name, msg
                ),
                qos_profile_sensor_data
            )

        for topic in (
            '/avp/viewpoints_data',
            '/avp/route'
        ):
            self.create_subscription(
                PoseArray,
                topic,
                lambda msg, name=topic: self.poses_cb(
                    name, msg
                ),
                qos_profile_sensor_data
            )

        self.create_subscription(
            TrajectorySetpoint6dof,
            '/fmu/in/trajectory_setpoint6dof',
            self.trajectory_cb,
            10
        )

        for topic in (
            '/avp/radius_delta',
            '/avp/front_distance'
        ):
            self.create_subscription(
                Float32,
                topic,
                lambda msg, name=topic: self._event(
                    name,
                    value=float(msg.data)
                ),
                10
            )

        # Timers

        self.create_timer(
            1.0,
            self.flush
        )

        self.create_timer(
            0.05,
            self.resolve_rgb_tf
        )

        self.get_logger().info(
            f'Flight recording: {self.run_dir}'
        )

    # JSON event logging

    def _event(self, topic, **data):
        row = {
            'topic': topic,
            'wall_time_ns': time.time_ns(),
        }

        row.update(data)

        self.events.write(
            json.dumps(
                row,
                ensure_ascii=False,
                allow_nan=False
            ) + '\n'
        )

    # Video recording using OpenCV

    def _write_video(self, name, frame, timestamp_ns):

        writer = self.writers.get(name)

        if writer is None:

            path = self.run_dir / f'{name}.mp4'

            writer = cv2.VideoWriter(
                str(path),
                cv2.VideoWriter_fourcc(*'mp4v'),
                self.video_fps,
                (
                    frame.shape[1],
                    frame.shape[0]
                )
            )

            if not writer.isOpened():
                writer.release()
                if name != 'rgb':
                    raise RuntimeError(f'Cannot open video file: {path}')
                path.unlink(missing_ok=True)
                (self.run_dir / 'rgb_jpeg').mkdir(exist_ok=True)
                writer = 'jpeg'
                self._event('rgb_fallback', format='jpg_q95', pattern='rgb_jpeg/%06d.jpg')
                self.get_logger().error('RGB MP4 encoder unavailable; saving full-resolution JPEG frames.')

            self.writers[name] = writer

        self.frame_counts[name] += 1

        start_ns = self.video_start_ns.setdefault(
            name,
            timestamp_ns
        )

        target_frame = round(
            (timestamp_ns - start_ns)
            * self.video_fps
            / 1_000_000_000
        )

        if target_frame < self.video_frames[name]:
            return None

        # Maintain the original video timing by duplicating
        # the previous frame when required.

        while self.video_frames[name] < target_frame:

            self._write_frame(
                name,
                writer,
                self.last_video_frame[name]
            )

            self.video_frames[name] += 1

        self._write_frame(
            name,
            writer,
            frame
        )

        self.video_frames[name] += 1
        self.last_video_frame[name] = frame.copy()

        return self.video_frames[name]

    def _write_frame(self, name, writer, frame):
        if writer == 'jpeg':
            path = self.run_dir / 'rgb_jpeg' / f'{self.video_frames[name] + 1:06d}.jpg'
            if not cv2.imwrite(str(path), frame, [cv2.IMWRITE_JPEG_QUALITY, 95]):
                raise RuntimeError(f'Cannot save RGB frame: {path}')
        else:
            writer.write(frame)

    # RGB frame to camera pose association

    def resolve_rgb_tf(self, force=False):

        while self.pending_rgb:

            (
                video_frame,
                source,
                duplicated,
                deadline_ns
            ) = self.pending_rgb[0]

            row = {
                'video_frame': video_frame,
                'source_video_frame': source['source_video_frame'],
                'duplicated': duplicated,
                'stamp_ns': source['stamp_ns'],
                'wall_time_ns': source['wall_time_ns'],
                'frame_id': source['frame_id'],
                'width': source['width'],
                'height': source['height'],
                'pose_frame_id': 'map',
            }

            try:

                if (
                    not source['frame_id']
                    or source['stamp_ns'] <= 0
                ):
                    raise ValueError(
                        'RGB image has no valid '
                        'frame_id or timestamp'
                    )

                tf = self.tf_buffer.lookup_transform(
                    'map',
                    source['frame_id'].lstrip('/'),
                    Time(
                        nanoseconds=source['stamp_ns']
                    )
                )

                p = tf.transform.translation
                q = tf.transform.rotation

                pose = [
                    p.x, p.y, p.z,
                    q.w, q.x, q.y, q.z
                ]

                if not all(
                    math.isfinite(value)
                    for value in pose
                ):
                    raise ValueError(
                        'RGB camera TF contains '
                        'non-finite values'
                    )

                row.update(
                    tf_status='ok',
                    tf_stamp_ns=stamp_ns(tf.header),
                    pose_map_camera=pose
                )

                self.frame_counts['rgb_pose'] += 1

            except (TransformException, ValueError) as exc:

                if (
                    not force
                    and time.monotonic_ns() < deadline_ns
                ):
                    break

                row.update(
                    tf_status='missing',
                    tf_stamp_ns=None,
                    pose_map_camera=None,
                    tf_error=str(exc)
                )

                self.frame_counts['rgb_pose_missing'] += 1

                if (
                    self.frame_counts['rgb_pose_missing'] == 1
                    or self.frame_counts['rgb_pose_missing'] % 100 == 0
                ):
                    self.get_logger().warn(
                        f'RGB camera TF unavailable '
                        f'for frame {video_frame}: {exc}'
                    )

            self.rgb_frames_file.write(
                json.dumps(
                    row,
                    ensure_ascii=False,
                    allow_nan=False
                ) + '\n'
            )

            self.pending_rgb.popleft()

    # RGB image callback

    def color_cb(self, msg):

        if msg.encoding not in (
            'bgr8',
            'rgb8',
            'mono8'
        ):
            self.get_logger().warn(
                f'Unsupported color encoding: {msg.encoding}',
                once=True
            )

            return

        channels = (
            1 if msg.encoding == 'mono8' else 3
        )

        frame = np.ndarray(
            (
                msg.height,
                msg.width,
                channels
            ),
            dtype=np.uint8,
            buffer=msg.data,
            strides=(
                msg.step,
                channels,
                1
            )
        )

        if msg.encoding == 'rgb8':

            frame = cv2.cvtColor(
                frame,
                cv2.COLOR_RGB2BGR
            )

        elif msg.encoding == 'mono8':

            frame = cv2.cvtColor(
                frame,
                cv2.COLOR_GRAY2BGR
            )

        # Maintain the existing 180-degree image rotation.

        frame = cv2.rotate(
            frame,
            cv2.ROTATE_180
        )

        captured_ns = stamp_ns(msg.header)
        received_ns = time.time_ns()

        previous_frame = self.video_frames['rgb']

        video_frame = self._write_video(
            'rgb',
            frame,
            captured_ns
        )

        if video_frame is not None:

            source = {
                'source_video_frame': video_frame,
                'stamp_ns': captured_ns,
                'wall_time_ns': received_ns,
                'frame_id': msg.header.frame_id,
                'width': msg.width,
                'height': msg.height,
            }

            for index in range(
                previous_frame + 1,
                video_frame + 1
            ):

                duplicated = index != video_frame

                frame_source = (
                    self.last_rgb_source
                    if duplicated else source
                )

                self.pending_rgb.append((
                    index,
                    frame_source,
                    duplicated,
                    time.monotonic_ns() + 2_000_000_000
                ))

            self.last_rgb_source = source

            self.resolve_rgb_tf()

        self._event(
            self.color_topic,
            wall_time_ns=received_ns,
            stamp_ns=captured_ns,
            frame_id=msg.header.frame_id,
            video_frame=video_frame
        )

    # Depth image callback

    def depth_cb(self, msg):

        if msg.encoding == '16UC1':

            dtype = np.dtype(
                '>u2' if msg.is_bigendian else '<u2'
            )

            scale = 0.001

        elif msg.encoding == '32FC1':

            dtype = np.dtype(
                '>f4' if msg.is_bigendian else '<f4'
            )

            scale = 1.0

        else:

            self.get_logger().warn(
                f'Unsupported depth encoding: {msg.encoding}',
                once=True
            )

            return

        depth = np.ndarray(
            (
                msg.height,
                msg.width
            ),
            dtype=dtype,
            buffer=msg.data,
            strides=(
                msg.step,
                dtype.itemsize
            )
        )

        # Save original depth data.

        self.frame_counts['raw_depth'] += 1

        raw_path = self.raw_depth_dir / (
            f'{self.frame_counts["raw_depth"]:06d}.npy'
        )

        np.save(
            raw_path,
            depth
        )

        # Generate depth preview video.

        distance = depth.astype(np.float32) * scale

        valid = (
            np.isfinite(distance)
            & (distance > 0)
        )

        image = np.clip(
            distance / self.depth_max_m * 255.0,
            0,
            255
        )

        image[~valid] = 0

        preview = cv2.applyColorMap(
            image.astype(np.uint8),
            cv2.COLORMAP_TURBO
        )

        preview[~valid] = 0

        video_frame = self._write_video(
            'depth',
            preview,
            stamp_ns(msg.header)
        )

        self._event(
            self.depth_topic,
            stamp_ns=stamp_ns(msg.header),
            frame_id=msg.header.frame_id,
            video_frame=video_frame,
            raw_path=str(
                raw_path.relative_to(self.run_dir)
            ),
            encoding=msg.encoding,
            depth_scale_m=scale,
            width=msg.width,
            height=msg.height,
            step=msg.step,
            is_bigendian=bool(msg.is_bigendian)
        )

    # Camera calibration logging

    def camera_info_cb(self, topic, msg):

        self.frame_counts['camera_info'] += 1

        self._event(
            topic,
            stamp_ns=stamp_ns(msg.header),
            frame_id=msg.header.frame_id,
            width=msg.width,
            height=msg.height,
            distortion_model=msg.distortion_model,
            d=list(msg.d),
            k=list(msg.k),
            r=list(msg.r),
            p=list(msg.p),
            binning_x=msg.binning_x,
            binning_y=msg.binning_y,
            roi={
                'x_offset': msg.roi.x_offset,
                'y_offset': msg.roi.y_offset,
                'height': msg.roi.height,
                'width': msg.roi.width,
                'do_rectify': msg.roi.do_rectify,
            }
        )

    # Point cloud logging

    def cloud_cb(self, msg):

        self.frame_counts['cloud'] += 1

        self._event(
            '/rgbd_camera/points',
            stamp_ns=stamp_ns(msg.header),
            frame_id=msg.header.frame_id,
            points=msg.width * msg.height
        )

    # AVP setpoint logging

    def setpoint_cb(self, topic, msg):

        values = pose_values(msg.pose)

        self._event(
            topic,
            stamp_ns=stamp_ns(msg.header),
            frame_id=msg.header.frame_id,
            pose=values
        )

    # PX4 trajectory setpoint logging

    def trajectory_cb(self, msg):

        self._event(
            '/fmu/in/trajectory_setpoint6dof',
            stamp_ns=int(msg.timestamp) * 1000,
            frame_id='local_ned',
            position=finite_values(msg.position),
            quaternion=finite_values(msg.quaternion),
            velocity=finite_values(msg.velocity),
            acceleration=finite_values(msg.acceleration),
            jerk=finite_values(msg.jerk),
            angular_velocity=finite_values(
                msg.angular_velocity
            )
        )

    # AVP route and viewpoint logging

    def poses_cb(self, topic, msg):

        self._event(
            topic,
            stamp_ns=stamp_ns(msg.header),
            frame_id=msg.header.frame_id,
            poses=[
                pose_values(pose)
                for pose in msg.poses
            ]
        )

    # Odometry logging

    def odom_cb(self, msg):

        values = pose_values(msg.pose.pose)

        velocity = msg.twist.twist.linear

        self._event(
            '/lics/odom',
            stamp_ns=stamp_ns(msg.header),
            frame_id=msg.header.frame_id,
            pose=finite_values(values),
            velocity=finite_values((
                velocity.x,
                velocity.y,
                velocity.z
            ))
        )

    # File management

    def flush(self):

        self.events.flush()
        self.rgb_frames_file.flush()

    def close(self):

        if self.closed:
            return

        self.closed = True

        self.resolve_rgb_tf(force=True)

        self._event(
            'end',
            frame_counts=self.frame_counts,
            video_frames=self.video_frames
        )

        # Release RGB and depth OpenCV video writers.

        for name, writer in self.writers.items():
            if writer != 'jpeg':
                writer.release()

        self.flush()

        self.events.close()
        self.rgb_frames_file.close()

        print(
            f'Saved flight recording: '
            f'{self.run_dir} {self.frame_counts}',
            flush=True
        )


def main():

    rclpy.init()

    node = None

    try:

        node = FlightRecorder()

        rclpy.spin(node)

    except (
        KeyboardInterrupt,
        ExternalShutdownException
    ):
        pass

    finally:

        signal.signal(
            signal.SIGINT,
            signal.SIG_IGN
        )

        if node is not None:

            node.close()
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
