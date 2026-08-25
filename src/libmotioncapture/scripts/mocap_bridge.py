#!/usr/bin/env python3
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

import motioncapture_ros as motioncapture


class MocapBridgeNode(Node):
    def __init__(self):
        super().__init__("mocap_bridge_py")

        self.declare_parameter("host_ip",    "202.169.1.197")
        self.declare_parameter("mocap_ip",   "202.169.1.100")
        self.declare_parameter("type",       "motionanalysis")
        self.declare_parameter("mocap_pub_name", "/opti_raw")
        self.declare_parameter("max_radius", 3.0)
        self.declare_parameter("fps",        500.0)

        self.host_ip    = str(self.get_parameter("host_ip").value)
        self.mocap_ip   = str(self.get_parameter("mocap_ip").value)
        self.mocap_type = str(self.get_parameter("type").value)
        self.pub_topic  = str(self.get_parameter("mocap_pub_name").value)
        self.max_radius = float(self.get_parameter("max_radius").value)
        self.target_fps = float(self.get_parameter("fps").value)

        self.min_interval = 1.0 / self.target_fps if self.target_fps > 0.0 else 0.0
        self.last_pub_time = 0.0

        self.pose_pub = self.create_publisher(PoseStamped, self.pub_topic, 10)

        cfg = {
            "hostname":       self.host_ip,
            "cortex_host":    self.mocap_ip,
            "cortex_port":    "1510",
            "multicast_port": "1511",
        }

        self.mc = motioncapture.connect(self.mocap_type, cfg)
        self.get_logger().info(
            f"Connected to {self.mocap_type} | host={self.host_ip}, mocap={self.mocap_ip}, "
            f"publishing to '{self.pub_topic}' at max {self.target_fps} Hz"
        )

    def step(self):
        self.mc.waitForNextFrame()

        bodies = self.mc.rigidBodies
        if not bodies:
            self.get_logger().warning("UDP frame received but no rigid bodies detected", throttle_duration_sec=2.0)
        else:
            for name, obj in bodies.items():
                pos_m = obj.position
                distance = np.linalg.norm(pos_m)
                self.get_logger().info(
                    f"[UDP] body='{name}' x={pos_m[0]:.3f} y={pos_m[1]:.3f} z={pos_m[2]:.3f} "
                    f"dist={distance:.3f}m {'(skipped: out of range)' if distance > self.max_radius else ''}",
                    throttle_duration_sec=1.0,
                )

        current_time = self.get_clock().now().nanoseconds * 1e-9
        if (current_time - self.last_pub_time) < self.min_interval:
            return

        self.last_pub_time = current_time

        for name, obj in self.mc.rigidBodies.items():
            pos_m = obj.position
            distance = np.linalg.norm(pos_m)

            if distance > self.max_radius:
                continue

            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            # PoseStamped has no body-name field; keep it here so downstream nodes can filter.
            msg.header.frame_id = name

            msg.pose.position.x = float(pos_m[0])
            msg.pose.position.y = float(pos_m[1])
            msg.pose.position.z = float(pos_m[2])

            msg.pose.orientation.x = float(obj.rotation.x)
            msg.pose.orientation.y = float(obj.rotation.y)
            msg.pose.orientation.z = float(obj.rotation.z)
            msg.pose.orientation.w = float(obj.rotation.w)

            self.pose_pub.publish(msg)
            self.get_logger().info(f"[PUB] body='{name}' published to '{self.pub_topic}'", throttle_duration_sec=1.0)


def run():
    rclpy.init()
    node = None

    try:
        node = MocapBridgeNode()

        while rclpy.ok():
            node.step()
            rclpy.spin_once(node, timeout_sec=0.0)
    except Exception as e:
        if node is not None:
            node.get_logger().error(f"Connection Failed: {e}")
        else:
            print(f"Connection Failed: {e}")
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    run()
