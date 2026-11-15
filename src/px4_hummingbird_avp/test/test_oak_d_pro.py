#!/usr/bin/env python3
"""Check OAK PointCloud2 data and its TF path for RViz and AVP."""

import argparse
import math
import sys
import time
from itertools import islice

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener


class CloudCheck(Node):
    def __init__(self, topic, target_frame):
        super().__init__('oak_d_pro_check')
        self.target_frame = target_frame
        self.count = 0
        self.valid_count = 0
        self.frame_id = ''
        self.tf_ok = False
        self.first_time = None
        self.last_time = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.subscription = self.create_subscription(
            PointCloud2, topic, self.cloud_cb, qos_profile_sensor_data)

    def cloud_cb(self, msg):
        now = time.monotonic()
        self.first_time = now if self.first_time is None else self.first_time
        self.last_time = now
        self.count += 1
        self.frame_id = msg.header.frame_id
        fields = {field.name for field in msg.fields}
        valid = False
        if {'x', 'y', 'z'} <= fields and msg.width * msg.height > 0:
            for x, y, z in islice(point_cloud2.read_points(
                    msg, field_names=('x', 'y', 'z'), skip_nans=True), 100):
                if all(math.isfinite(float(v)) for v in (x, y, z)):
                    valid = True
                    break
        self.valid_count += int(valid)
        if self.frame_id:
            try:
                self.tf_buffer.lookup_transform(self.target_frame, self.frame_id, Time())
                self.tf_ok = True
            except TransformException:
                pass
        if self.count == 1 or self.count % 30 == 0:
            self.get_logger().info(
                f'clouds={self.count} points={msg.width * msg.height} '
                f'valid_xyz={valid} frame={self.frame_id or "<empty>"} '
                f'TF({self.target_frame} <- cloud)={self.tf_ok}')


def main():
    parser = argparse.ArgumentParser(description='Check OAK-D Pro cloud and TF.')
    parser.add_argument('--topic', default='/rgbd_camera/points')
    parser.add_argument('--target-frame', default='base_link')
    parser.add_argument('--duration', type=float, default=10.0)
    args = parser.parse_args()
    if args.duration <= 0:
        parser.error('--duration must be positive')

    rclpy.init()
    node = CloudCheck(args.topic, args.target_frame)
    deadline = time.monotonic() + args.duration
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        hz = 0.0
        if node.count > 1 and node.last_time > node.first_time:
            hz = (node.count - 1) / (node.last_time - node.first_time)
        print(f'topic={args.topic} clouds={node.count} rate={hz:.1f} Hz '
              f'valid_clouds={node.valid_count} frame={node.frame_id or "<empty>"} '
              f'TF({args.target_frame} <- cloud)={node.tf_ok}')
        return 0 if node.valid_count > 0 and node.tf_ok else 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
