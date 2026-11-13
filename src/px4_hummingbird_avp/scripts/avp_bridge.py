#!/usr/bin/env python3
"""PX4 NED odometry to AVP map/ENU, with optional Gazebo pose to PX4 EV."""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import VehicleOdometry
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

# world: NED <-> ENU  (symmetric, self-inverse)
R_NE = np.array([[0.0, 1.0, 0.0],
                 [1.0, 0.0, 0.0],
                 [0.0, 0.0, -1.0]])
# body: FRD <-> FLU  (self-inverse)
R_FB = np.diag([1.0, -1.0, -1.0])
C = 1.0 / math.sqrt(2.0)


def quat_wxyz_to_R(q):
    w, x, y, z = q
    n = np.sqrt(w * w + x * x + y * y + z * z)
    if n < 1e-9 or not np.isfinite(n):
        return None
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


def R_to_quat_wxyz(R):
    # robust matrix -> quaternion
    t = np.trace(R)
    if t > 0.0:
        s = np.sqrt(t + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)


class AvpBridge(Node):
    def __init__(self):
        super().__init__('avp_bridge')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('enable_ev', False)
        self.declare_parameter('pose_topic', '/model/hummingbird_0/pose')
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST, depth=5)
        self.sub = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.cb, qos)
        self.odom_pub = self.create_publisher(Odometry, '/lics/odom', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/lics/pose', 10)
        self.tf_bc = TransformBroadcaster(self)
        self.static_tf_bc = StaticTransformBroadcaster(self)
        camera_tf = TransformStamped()
        camera_tf.header.stamp = self.get_clock().now().to_msg()
        camera_tf.header.frame_id = self.base_frame
        camera_tf.child_frame_id = 'hummingbird_0/base_link/rgbd_camera'
        camera_tf.transform.translation.x = 0.20
        camera_tf.transform.translation.z = 0.05
        camera_tf.transform.rotation.w = 1.0
        self.static_tf_bc.sendTransform(camera_tf)

        if self.get_parameter('enable_ev').value:
            pose_topic = self.get_parameter('pose_topic').value
            self.ev_pub = self.create_publisher(VehicleOdometry, '/fmu/in/vehicle_visual_odometry', qos)
            self.ev_sub = self.create_subscription(PoseStamped, pose_topic, self.ev_cb, qos)
            self.ev_count = 0
            self.get_logger().info('Gazebo EV: %s -> /fmu/in/vehicle_visual_odometry' % pose_topic)
        self.get_logger().info('avp_bridge: /fmu/out/vehicle_odometry -> /lics/odom, /lics/pose, TF %s->%s'
                               % (self.map_frame, self.base_frame))

    def cb(self, m: VehicleOdometry):
        R_nf = quat_wxyz_to_R(m.q)
        if R_nf is None or not np.all(np.isfinite(m.position)):
            return
        p_enu = R_NE @ np.asarray(m.position, dtype=float)
        R_enu_flu = R_NE @ R_nf @ R_FB
        q = R_to_quat_wxyz(R_enu_flu)

        # velocity: PX4 gives NED world-frame velocity when velocity_frame==1 (NED)
        v_enu = np.zeros(3)
        if np.all(np.isfinite(m.velocity)):
            v_ned = np.asarray(m.velocity, dtype=float)
            v_enu = R_NE @ v_ned if m.velocity_frame != VehicleOdometry.VELOCITY_FRAME_BODY_FRD else R_NE @ (R_nf @ v_ned)
        w_flu = np.zeros(3)
        if np.all(np.isfinite(m.angular_velocity)):
            w_flu = R_FB @ np.asarray(m.angular_velocity, dtype=float)

        stamp = self.get_clock().now().to_msg()   # use node clock (sim time when use_sim_time)

        od = Odometry()
        od.header.stamp = stamp
        od.header.frame_id = self.map_frame
        od.child_frame_id = self.base_frame
        od.pose.pose.position.x, od.pose.pose.position.y, od.pose.pose.position.z = p_enu
        od.pose.pose.orientation.w, od.pose.pose.orientation.x, od.pose.pose.orientation.y, od.pose.pose.orientation.z = q
        od.twist.twist.linear.x, od.twist.twist.linear.y, od.twist.twist.linear.z = v_enu
        od.twist.twist.angular.x, od.twist.twist.angular.y, od.twist.twist.angular.z = w_flu
        self.odom_pub.publish(od)

        ps = PoseStamped()
        ps.header = od.header
        ps.pose = od.pose.pose
        self.pose_pub.publish(ps)

        if self.publish_tf:
            t = TransformStamped()
            t.header = od.header
            t.child_frame_id = self.base_frame
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = p_enu
            t.transform.rotation = od.pose.pose.orientation
            self.tf_bc.sendTransform(t)


    def ev_cb(self, msg: PoseStamped):
        p = msg.pose.position
        o = msg.pose.orientation
        if not np.all(np.isfinite([p.x, p.y, p.z, o.w, o.x, o.y, o.z])):
            return
        qw, qx, qy, qz = C * (o.w + o.z), C * (o.x + o.y), C * (o.x - o.y), C * (o.w - o.z)
        norm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
        if norm < 1e-9:
            return

        od = VehicleOdometry()
        now_us = int(self.get_clock().now().nanoseconds / 1000)
        od.timestamp = now_us
        od.timestamp_sample = now_us
        od.pose_frame = VehicleOdometry.POSE_FRAME_NED
        od.position = [float(p.y), float(p.x), float(-p.z)]
        od.q = [qw / norm, qx / norm, qy / norm, qz / norm]
        nan = float('nan')
        od.velocity_frame = VehicleOdometry.VELOCITY_FRAME_UNKNOWN
        od.velocity = [nan, nan, nan]
        od.angular_velocity = [nan, nan, nan]
        od.position_variance = [1e-4, 1e-4, 1e-4]
        od.orientation_variance = [1e-4, 1e-4, 1e-4]
        od.velocity_variance = [nan, nan, nan]
        od.reset_counter = 0
        od.quality = 0
        self.ev_pub.publish(od)
        self.ev_count += 1
        if self.ev_count % 500 == 0:
            self.get_logger().info('EV #%d: ned=(%.2f,%.2f,%.2f)' % (self.ev_count, p.y, p.x, -p.z))


def main():
    rclpy.init()
    n = AvpBridge()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            n.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
