#!/usr/bin/env python3
"""Record AVP 6DoF commands and vehicle odometry; plot a saved JSONL run."""

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import signal
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import PoseStamped


DEFAULT_LOG_DIR = '~/Desktop/px4_hummingbird/src/px4_hummingbird_avp/logs'


def finite_list(values):
    return [float(v) if math.isfinite(v) else None for v in values]


class AvpTrajectoryLogger(Node):
    def __init__(self):
        super().__init__('avp_trajectory_logger')
        log_dir = Path(str(self.declare_parameter('log_dir', DEFAULT_LOG_DIR).value)).expanduser()
        log_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S_%fZ')
        self.path = log_dir / f'avp_{stamp}.jsonl'
        self.file = self.path.open('x', encoding='utf-8', buffering=1024 * 1024)
        self.closed = False
        self._write('meta', {
            'format': 'avp_trajectory_v2',
            'frame': 'map/ENU/FLU',
        })
        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         durability=QoSDurabilityPolicy.VOLATILE,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=100)
        for topic in ('/avp/base_setpoint', '/avp/setpoint'):
            self.create_subscription(PoseStamped, topic, lambda msg, name=topic: self.pose_cb(name, msg), qos)
        self.create_timer(1.0, self.file.flush)
        self.get_logger().info(f'AVP 6DoF JSONL: {self.path}')

    def _write(self, topic, data):
        record = {'topic': topic, 'wall_time_ns': time.time_ns(),
                  'monotonic_ns': time.monotonic_ns(),
                  'ros_time_ns': self.get_clock().now().nanoseconds}
        record.update(data)
        self.file.write(json.dumps(record, ensure_ascii=True, allow_nan=False, separators=(',', ':')) + '\n')

    def pose_cb(self, topic, msg):
        self._write(topic, {
            'stamp_ns': int(msg.header.stamp.sec) * 1000000000 + int(msg.header.stamp.nanosec),
            'frame_id': msg.header.frame_id,
            'position': finite_list((msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)),
            'quaternion': finite_list((msg.pose.orientation.w, msg.pose.orientation.x,
                                       msg.pose.orientation.y, msg.pose.orientation.z)),
        })

    def close(self):
        if not self.closed:
            self._write('end', {})
            self.file.close()
            self.closed = True


def quat_to_rpy(q):
    if len(q) != 4 or any(v is None for v in q):
        return [math.nan, math.nan, math.nan]
    w, x, y, z = q
    return [math.atan2(2 * (w*x + y*z), 1 - 2 * (x*x + y*y)),
            math.asin(max(-1.0, min(1.0, 2 * (w*y - z*x)))),
            math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))]


def plot_log(path, save_path=None):
    import matplotlib.pyplot as plt
    import numpy as np

    with path.open(encoding='utf-8') as file:
        records = [json.loads(line) for line in file if line.strip()]
    rows = [r for r in records if r.get('topic') in ('/avp/base_setpoint', '/avp/setpoint')
            and r.get('frame_id') == 'map']
    if not rows:
        raise ValueError(f'No AVP path samples in {path}')
    t0 = rows[0]['monotonic_ns']

    fig = plt.figure(figsize=(15, 7))
    ax_3d = fig.add_subplot(121, projection='3d')
    ax_pos = fig.add_subplot(222)
    ax_att = fig.add_subplot(224)
    for topic, label, color in (('/avp/base_setpoint', 'Base sweep', 'tab:orange'),
                                ('/avp/setpoint', 'AVP viewpoints', 'tab:purple')):
        samples = [r for r in rows if r['topic'] == topic and
                   all(v is not None for v in r['position'] + r['quaternion'])]
        if not samples:
            continue
        t = np.array([(r['monotonic_ns'] - t0) * 1e-9 for r in samples])
        pos = np.array([r['position'] for r in samples])
        quat = np.array([r['quaternion'] for r in samples])
        rpy = np.degrees(np.unwrap(np.array([quat_to_rpy(q) for q in quat]), axis=0))
        ax_3d.plot(pos[:, 0], pos[:, 1], pos[:, 2], color=color, label=label)
        ax_3d.scatter(*pos[0], color=color, s=20)
        for i, axis in enumerate(('E', 'N', 'U')):
            ax_pos.plot(t, pos[:, i], color=color, linestyle=('-', '--', ':')[i], label=f'{label} {axis}')
        for i, axis in enumerate(('roll', 'pitch', 'yaw')):
            ax_att.plot(t, rpy[:, i], color=color, linestyle=('-', '--', ':')[i], label=f'{label} {axis}')
        for idx in np.linspace(0, len(pos) - 1, min(5, len(pos)), dtype=int):
            w, x, y, z = quat[idx] / np.linalg.norm(quat[idx])
            R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                          [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                          [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
            for axis, axis_color in enumerate(('red', 'green', 'blue')):
                ax_3d.quiver(*pos[idx], *(0.15 * R[:, axis]), color=axis_color, alpha=0.6)
    ax_3d.set(xlabel='E [m]', ylabel='N [m]', zlabel='U [m]')
    ax_3d.legend()
    ax_pos.set(xlabel='Time [s]', ylabel='Position [m]')
    ax_att.set(xlabel='Time [s]', ylabel='Attitude [deg]')
    for ax in (ax_pos, ax_att):
        ax.grid(True)
        ax.legend(fontsize='small', ncol=2)
    fig.tight_layout()
    if save_path:
        fig.savefig(save_path, dpi=150)
        print(f'Saved plot: {save_path}')
    else:
        plt.show()


def main():
    if len(sys.argv) > 1 and sys.argv[1] == '--plot':
        parser = argparse.ArgumentParser(description='Plot an AVP 6DoF JSONL log')
        parser.add_argument('file', type=Path)
        parser.add_argument('--save', type=Path, help='Save a PNG instead of opening a window')
        args = parser.parse_args(sys.argv[2:])
        plot_log(args.file.expanduser(), args.save)
        return

    rclpy.init()
    node = None
    try:
        node = AvpTrajectoryLogger()
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
