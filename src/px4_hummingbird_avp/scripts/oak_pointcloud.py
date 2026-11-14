#!/usr/bin/env python3
"""Project OAK depth images into an AVP PointCloud2 in the optical frame."""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField


class OakPointcloud(Node):
    def __init__(self):
        super().__init__('oak_pointcloud')
        self.declare_parameter('depth_image_topic', '/oak/stereo/image_raw')
        self.declare_parameter('camera_info_topic', '/oak/stereo/camera_info')
        self.declare_parameter('sample_step', 2)
        self.declare_parameter('max_depth_m', 6.0)
        image_topic = self.get_parameter('depth_image_topic').value
        info_topic = self.get_parameter('camera_info_topic').value
        self.sample_step = int(self.get_parameter('sample_step').value)
        self.max_depth_m = float(self.get_parameter('max_depth_m').value)
        if self.sample_step < 1 or self.max_depth_m <= 0.05:
            raise ValueError('sample_step must be >= 1 and max_depth_m must be > 0.05')
        self.camera_info = None
        self.frames = 0
        output_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(PointCloud2, '/rgbd_camera/points', output_qos)
        self.info_sub = self.create_subscription(CameraInfo, info_topic, self.info_cb,
                                                 qos_profile_sensor_data)
        self.image_sub = self.create_subscription(Image, image_topic, self.image_cb,
                                                  qos_profile_sensor_data)
        self.get_logger().info(
            f'{image_topic} + {info_topic} -> /rgbd_camera/points '
            f'(sample_step={self.sample_step})')

    def info_cb(self, msg):
        self.camera_info = msg

    def image_cb(self, msg):
        info = self.camera_info
        if info is None or msg.header.frame_id != info.header.frame_id:
            return
        if msg.width != info.width or msg.height != info.height:
            return
        if msg.encoding == '16UC1':
            dtype = np.dtype('>u2' if msg.is_bigendian else '<u2')
            scale = 0.001
        elif msg.encoding == '32FC1':
            dtype = np.dtype('>f4' if msg.is_bigendian else '<f4')
            scale = 1.0
        else:
            self.get_logger().warn(f'Unsupported depth encoding: {msg.encoding}')
            return
        fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
        if fx <= 0 or fy <= 0:
            return

        step = self.sample_step
        depth = np.ndarray((msg.height, msg.width), dtype=dtype,
                           buffer=msg.data, strides=(msg.step, dtype.itemsize))
        z = depth[::step, ::step].astype(np.float32) * scale
        u, v = np.meshgrid(np.arange(0, msg.width, step),
                           np.arange(0, msg.height, step))
        valid = np.isfinite(z) & (z > 0.05) & (z < self.max_depth_m)
        if not np.any(valid):
            return
        x = ((u[valid] - cx) * z[valid] / fx).astype(np.float32)
        y = ((v[valid] - cy) * z[valid] / fy).astype(np.float32)
        xyz = np.column_stack((x, y, z[valid])).astype('<f4', copy=False)

        cloud = PointCloud2()
        cloud.header = msg.header
        cloud.height = 1
        cloud.width = len(xyz)
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * cloud.width
        cloud.data = xyz.tobytes()
        cloud.is_dense = True
        self.pub.publish(cloud)
        self.frames += 1
        if self.frames == 1:
            self.get_logger().info(
                f'First cloud: {cloud.width} points, frame={cloud.header.frame_id}')


def main():
    rclpy.init()
    node = OakPointcloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
