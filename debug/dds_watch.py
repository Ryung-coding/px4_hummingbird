#!/usr/bin/env python3
"""Observe PX4 DDS traffic and local Agent/servo processes without publishing."""

import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import socket
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from px4_msgs.msg import (
    ActuatorServos,
    HummingbirdStatus,
    OffboardControlMode,
    TrajectorySetpoint6dof,
    VehicleAttitude,
    VehicleCommand,
    VehicleOdometry,
    VehicleStatus,
)


TOPICS = {
    "/fmu/out/vehicle_attitude": (VehicleAttitude, True),
    "/fmu/out/actuator_servos": (ActuatorServos, True),
    "/fmu/out/hummingbird_status": (HummingbirdStatus, False),
    "/fmu/out/vehicle_status": (VehicleStatus, False),
    "/fmu/in/offboard_control_mode": (OffboardControlMode, True),
    "/fmu/in/trajectory_setpoint6dof": (TrajectorySetpoint6dof, True),
    "/fmu/in/vehicle_visual_odometry": (VehicleOdometry, True),
    "/fmu/in/vehicle_command": (VehicleCommand, False),
}


def process_pids(executable):
    pids = []
    for entry in os.scandir("/proc"):
        if not entry.name.isdigit():
            continue
        try:
            command = (Path(entry.path) / "cmdline").read_bytes().split(b"\0", 1)[0]
        except (OSError, PermissionError):
            continue
        if command and os.fsdecode(command).rsplit("/", 1)[-1] == executable:
            pids.append(int(entry.name))
    return sorted(pids)


class DdsWatch(Node):
    def __init__(self, log_path, gap_s, expect_servos):
        super().__init__("hummingbird_dds_watch")
        self.log = log_path.open("w", encoding="utf-8", buffering=1)
        self.gap_s = gap_s
        self.expect_servos = expect_servos
        self.start_ns = time.monotonic_ns()
        self.last_report_ns = self.start_ns
        self.last_processes = None
        self.states = {
            name: {"last_ns": None, "window_count": 0, "total": 0, "max_gap_s": 0.0, "silent": False}
            for name in TOPICS
        }
        self.record("start", host=socket.gethostname(), domain=os.environ.get("ROS_DOMAIN_ID", "0"),
                    gap_s=gap_s, expect_servos=expect_servos)

        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._subscriptions = [
            self.create_subscription(msg_type, name, lambda msg, topic=name: self.on_message(topic, msg), qos)
            for name, (msg_type, _) in TOPICS.items()
        ]
        self.timer = self.create_timer(1.0, self.report)

    def record(self, kind, **values):
        entry = {"event": kind, "utc": datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
                 "mono_ns": time.monotonic_ns(), **values}
        self.log.write(json.dumps(entry, separators=(",", ":"), allow_nan=False) + "\n")

    def on_message(self, topic, msg):
        now = time.monotonic_ns()
        state = self.states[topic]
        last_ns = state["last_ns"]
        if last_ns is not None:
            gap_s = (now - last_ns) / 1e9
            state["max_gap_s"] = max(state["max_gap_s"], gap_s)
            if TOPICS[topic][1] and gap_s >= self.gap_s:
                self.record("gap", topic=topic, duration_s=round(gap_s, 6))
                print(f"GAP {topic}: {gap_s:.3f}s", flush=True)

        state["last_ns"] = now
        state["window_count"] += 1
        state["total"] += 1
        state["silent"] = False
        values = {"topic": topic, "px4_timestamp_us": getattr(msg, "timestamp", None)}
        if topic == "/fmu/out/actuator_servos":
            values["control"] = [float(value) if math.isfinite(value) else None for value in msg.control]
        elif topic == "/fmu/out/hummingbird_status":
            values.update(hb_cmd_source=msg.hb_cmd_source, hb_ctrl_mode=msg.hb_ctrl_mode,
                          hummingbird_airframe=msg.hummingbird_airframe)
        elif topic == "/fmu/out/vehicle_status":
            values.update(arming_state=msg.arming_state, nav_state=msg.nav_state)
        self.record("sample", **values)

    def report(self):
        now = time.monotonic_ns()
        interval_s = (now - self.last_report_ns) / 1e9
        self.last_report_ns = now
        processes = {"agent": process_pids("MicroXRCEAgent"),
                     "servo": process_pids("px4_servo_to_dynamixel")}
        if processes != self.last_processes:
            self.record("process_change", **processes)
            self.last_processes = processes

        snapshots = {}
        for topic, state in self.states.items():
            last_ns = state["last_ns"]
            age_s = None if last_ns is None else (now - last_ns) / 1e9
            count = state["window_count"]
            state["window_count"] = 0
            if TOPICS[topic][1] and age_s is not None and age_s >= self.gap_s and not state["silent"]:
                self.record("silence", topic=topic, duration_s=round(age_s, 6))
                print(f"SILENCE {topic}: {age_s:.3f}s", flush=True)
                state["silent"] = True
            snapshots[topic] = {"hz": round(count / interval_s, 2), "age_s": None if age_s is None else round(age_s, 3),
                                "total": state["total"], "max_gap_s": round(state["max_gap_s"], 3),
                                "publishers": self.count_publishers(topic)}

        servo = snapshots["/fmu/out/actuator_servos"]
        if self.expect_servos and servo["total"] == 0 and (now - self.start_ns) >= 3e9:
            print("WAITING: no actuator_servos sample yet", flush=True)

        self.record("health", processes=processes, topics=snapshots)
        att = snapshots["/fmu/out/vehicle_attitude"]
        servo_age = "none" if servo["age_s"] is None else f"{servo['age_s']:.3f}s"
        print(f"agent={processes['agent']} servo={processes['servo']} "
              f"att={att['hz']:.1f}Hz servo={servo['hz']:.1f}Hz servo_age={servo_age} "
              f"servo_pub={servo['publishers']}", flush=True)

    def count_publishers(self, topic):
        try:
            return len(self.get_publishers_info_by_topic(topic))
        except RuntimeError:
            return None

    def close(self):
        self.record("stop", totals={name: state["total"] for name, state in self.states.items()})
        self.log.close()


def main():
    parser = argparse.ArgumentParser(description="Record local PX4 DDS sample gaps and Agent/servo process state")
    parser.add_argument("--gap", type=float, default=0.2, help="gap warning threshold in seconds (default: 0.2)")
    parser.add_argument("--expect-servos", action="store_true", help="warn when no actuator_servos have arrived")
    parser.add_argument("--output", type=Path, help="JSONL file (default: debug/logs/dds_<UTC>.jsonl)")
    args = parser.parse_args()
    if args.gap <= 0:
        parser.error("--gap must be positive")

    log_path = args.output or Path(__file__).resolve().parent / "logs" / f"dds_{datetime.now(timezone.utc):%Y%m%d_%H%M%S}.jsonl"
    log_path.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = None
    try:
        node = DdsWatch(log_path, args.gap, args.expect_servos)
        print(f"Logging to {log_path}", flush=True)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
