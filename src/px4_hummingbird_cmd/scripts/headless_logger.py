#!/usr/bin/env python3
"""Record HummingBird ROS 2 topics locally in viewer-compatible JSONL."""

import argparse
import json
import math
import queue
import sys
import threading
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import (
    ActuatorMotors,
    ActuatorServos,
    HummingbirdStatus,
    ManualControlSetpoint,
    VehicleAttitude,
    VehicleAttitudeSetpoint,
    VehicleAngularVelocity,
    VehicleRatesSetpoint,
    VehicleLocalPosition,
    VehicleLocalPositionSetpoint,
    VehicleOdometry,
    VehicleStatus,
    VehicleThrustSetpoint,
    VehicleTorqueSetpoint,
)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.convert import message_to_ordereddict


TOPICS = (
    ("/fmu/out/vehicle_local_position", VehicleLocalPosition),
    ("/fmu/out/vehicle_local_position_setpoint", VehicleLocalPositionSetpoint),
    ("/fmu/out/vehicle_angular_velocity", VehicleAngularVelocity),
    ("/fmu/out/vehicle_rates_setpoint", VehicleRatesSetpoint),
    ("/fmu/out/vehicle_attitude", VehicleAttitude),
    ("/fmu/out/vehicle_attitude_setpoint", VehicleAttitudeSetpoint),
    ("/fmu/out/vehicle_thrust_setpoint", VehicleThrustSetpoint),
    ("/fmu/out/vehicle_torque_setpoint", VehicleTorqueSetpoint),
    ("/fmu/out/manual_control_setpoint", ManualControlSetpoint),
    ("/fmu/out/actuator_motors", ActuatorMotors),
    ("/fmu/out/actuator_servos", ActuatorServos),
    ("/fmu/out/hummingbird_status", HummingbirdStatus),
    ("/fmu/out/vehicle_status", VehicleStatus),
    ("/fmu/out/vehicle_odometry", VehicleOdometry),
    ("/opti_raw", PoseStamped),
)


def json_value(value):
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {key: json_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_value(item) for item in value]
    return value


def viewer_data(topic, msg, opti_origin, target_body_name):
    if topic == "/opti_raw" and target_body_name and msg.header.frame_id != target_body_name:
        return None

    data = message_to_ordereddict(msg)
    if topic == "/fmu/out/vehicle_local_position":
        data["position"] = [msg.x, msg.y, msg.z]
        data["velocity"] = [msg.vx, msg.vy, msg.vz]
    elif topic == "/fmu/out/vehicle_local_position_setpoint":
        data["position"] = [msg.x, msg.y, msg.z]
        data["velocity"] = [msg.vx, msg.vy, msg.vz]
    elif topic == "/fmu/out/vehicle_rates_setpoint":
        data["xyz"] = [msg.roll, msg.pitch, msg.yaw]
    elif topic == "/fmu/out/manual_control_setpoint":
        data["roll_pitch_yaw_throttle"] = [msg.roll, msg.pitch, msg.yaw, msg.throttle]
    elif topic == "/fmu/out/vehicle_odometry":
        data["q_wxyz"] = list(msg.q)
    elif topic == "/opti_raw":
        pos = msg.pose.position
        att = msg.pose.orientation
        stamp = msg.header.stamp
        header_stamp = stamp.sec + stamp.nanosec * 1.0e-9
        norm = math.sqrt(att.w * att.w + att.x * att.x + att.y * att.y + att.z * att.z)
        q = [att.w / norm, att.x / norm, -att.y / norm, -att.z / norm] if norm > 1.0e-9 else [1.0, 0.0, 0.0, 0.0]
        # R_down = diag(1, -1, -1): position and body orientation match the viewer.
        data.update({
            "header_stamp": header_stamp,
            "frame_id": msg.header.frame_id,
            "opti_position_z_up": [pos.x, pos.y, pos.z],
            "opti_q_wxyz": [att.w, att.x, att.y, att.z],
            "px4_position_ned": [pos.x - opti_origin[0], opti_origin[1] - pos.y, opti_origin[2] - pos.z],
            "px4_q_wxyz": q,
        })
    return data


class JsonlWriter:
    def __init__(self, path):
        self.path = path
        self.start = time.monotonic()
        self.records = queue.Queue(maxsize=20000)
        self.stop = threading.Event()
        self.written = 0
        self.dropped = 0
        self.error = None
        path.parent.mkdir(parents=True, exist_ok=True)
        self.file = path.open("x", encoding="utf-8", buffering=1024 * 1024)
        self.thread = threading.Thread(target=self._run, name="headless-jsonl-writer")
        self.thread.start()

    def write(self, topic, data):
        record = {
            "wall_time": time.time(),
            "elapsed_time": time.monotonic() - self.start,
            "topic": topic,
            "data": data,
        }
        try:
            self.records.put_nowait(record)
        except queue.Full:
            self.dropped += 1

    def _run(self):
        last_flush = time.monotonic()
        try:
            while not self.stop.is_set() or not self.records.empty():
                try:
                    record = self.records.get(timeout=0.1)
                except queue.Empty:
                    record = None
                if record is not None:
                    self.file.write(json.dumps(json_value(record), separators=(",", ":")) + "\n")
                    self.written += 1
                if time.monotonic() - last_flush >= 0.5:
                    self.file.flush()
                    last_flush = time.monotonic()
            self.file.write(json.dumps({
                "wall_time": time.time(),
                "elapsed_time": time.monotonic() - self.start,
                "topic": "viewer_logger_stats",
                "data": {"written": self.written, "dropped": self.dropped},
            }) + "\n")
        except (OSError, TypeError, ValueError) as exc:
            self.error = str(exc)
        finally:
            self.file.close()

    def close(self):
        self.stop.set()
        self.thread.join()


class HeadlessLogger(Node):
    def __init__(self, writer, opti_origin, target_body_name):
        super().__init__("hummingbird_headless_logger")
        self.writer = writer
        self.opti_origin = opti_origin
        self.target_body_name = target_body_name
        self.counts = {topic: 0 for topic, _ in TOPICS}
        self.topic_subscriptions = []
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        for topic, msg_type in TOPICS:
            self.topic_subscriptions.append(self.create_subscription(
                msg_type, topic, lambda msg, name=topic: self.record(name, msg), qos,
            ))
        self.create_timer(10.0, self.report)

    def record(self, topic, msg):
        data = viewer_data(topic, msg, self.opti_origin, self.target_body_name)
        if data is not None:
            self.counts[topic] += 1
            self.writer.write(topic, data)

    def report(self):
        self.get_logger().info(
            f"records={self.writer.written} dropped={self.writer.dropped} "
            f"servos={self.counts['/fmu/out/actuator_servos']} "
            f"opti={self.counts['/opti_raw']}"
        )
        if self.writer.error:
            self.get_logger().error(f"JSONL write failed: {self.writer.error}")
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description="Log HummingBird ROS 2 topics without a GUI")
    parser.add_argument("--log-dir", type=Path, default=Path("~/Desktop/view_log"))
    parser.add_argument("--output", type=Path, help="Exact JSONL output path")
    parser.add_argument("--opti-origin", type=float, nargs=3, default=[-0.08, -0.524, 0.016], metavar=("X", "Y", "Z"))
    parser.add_argument("--target-body-name", default="Hummingbird")
    args = parser.parse_args()
    path = args.output if args.output else args.log_dir / f"{time.strftime('%m%d_%H%M%S')}_headless.jsonl"
    path = path.expanduser().resolve()

    try:
        writer = JsonlWriter(path)
    except OSError as exc:
        parser.exit(1, f"Cannot create {path}: {exc}\n")

    writer.write("viewer_meta", {
        "viewer_mode": "real",
        "logger": "headless",
        "opti_topic": "/opti_raw",
        "fmu_odom_topic": "/fmu/out/vehicle_odometry",
        "target_body_name": args.target_body_name,
        "opti_origin_z_up_m": args.opti_origin,
        "real_mode_extra_topics": True,
        "transform": "pos_px4 = R_down * (opti_pos - opti_origin), R_px4 = R_down * R_opti * R_down",
    })
    rclpy.init()
    node = None
    try:
        node = HeadlessLogger(writer, args.opti_origin, args.target_body_name)
        print(f"Logging to {path}", flush=True)
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        writer.close()
        print(f"Saved {writer.written} records, dropped {writer.dropped}: {path}", flush=True)
        if writer.error:
            print(f"JSONL write failed: {writer.error}", file=sys.stderr)
            return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
