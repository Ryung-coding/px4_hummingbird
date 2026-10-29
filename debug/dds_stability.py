#!/usr/bin/env python3
"""Measure continuous PX4 DDS reception on the Orin without publishing commands."""

import argparse
from collections import deque
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import socket
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import ActuatorMotors, ActuatorServos, VehicleAttitude, VehicleOdometry, VehicleStatus


STREAMS = {
    "/nokov_raw/Hummingbird/pose": PoseStamped,
    "/opti_raw": PoseStamped,
    "/fmu/in/vehicle_visual_odometry": VehicleOdometry,
    "/fmu/out/vehicle_odometry": VehicleOdometry,
    "/fmu/out/vehicle_attitude": VehicleAttitude,
    "/fmu/out/actuator_servos": ActuatorServos,
    "/fmu/out/actuator_motors": ActuatorMotors,
}
OPTI_STREAMS = {
    "/nokov_raw/Hummingbird/pose",
    "/opti_raw",
    "/fmu/in/vehicle_visual_odometry",
}


def process_pids(executable):
    pids = []
    for entry in os.scandir("/proc"):
        if not entry.name.isdigit():
            continue
        try:
            command = (Path(entry.path) / "cmdline").read_bytes().split(b"\0", 1)[0]
        except OSError:
            continue
        if command and os.fsdecode(command).rsplit("/", 1)[-1] == executable:
            pids.append(int(entry.name))
    return sorted(pids)


class DdsStability(Node):
    def __init__(self, log_path, gap_s, duration_s, expect_servos, expect_opti):
        super().__init__("hummingbird_dds_stability")
        self.log = log_path.open("w", encoding="utf-8", buffering=65536)
        self.gap_s = gap_s
        self.duration_s = duration_s
        self.expect_servos = expect_servos
        self.expect_opti = expect_opti
        self.started_ns = time.monotonic_ns()
        self.last_report_ns = self.started_ns
        self.last_flush_ns = self.started_ns
        self.armed = None
        self.last_processes = None
        self.states = {
            topic: {"last_ns": None, "last_source_us": None, "count": 0, "reported_count": 0,
                    "gaps": 0, "max_gap_ms": 0.0, "intervals_ms": deque(maxlen=2000),
                    "silent": False, "no_data": False}
            for topic in STREAMS
        }
        self.record("start", host=socket.gethostname(), domain=os.environ.get("ROS_DOMAIN_ID"),
                    gap_s=gap_s, duration_s=duration_s, expect_servos=expect_servos, expect_opti=expect_opti)

        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._subscriptions = [
            self.create_subscription(msg_type, topic, lambda msg, name=topic: self.on_sample(name, msg), qos)
            for topic, msg_type in STREAMS.items()
        ]
        self._status_sub = self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status", self.on_status, qos)
        self._watchdog = self.create_timer(0.1, self.check_silence)
        self._process_watchdog = self.create_timer(0.2, self.check_processes)
        self._report = self.create_timer(5.0, self.report)

    def record(self, event, **fields):
        entry = {"event": event, "utc": datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
                 "mono_ns": time.monotonic_ns(), **fields}
        self.log.write(json.dumps(entry, separators=(",", ":"), allow_nan=False) + "\n")

    def on_status(self, msg):
        armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        if armed != self.armed:
            self.armed = armed
            self.record("arming", armed=armed, nav_state=msg.nav_state)

    @staticmethod
    def source_timestamp_us(msg):
        if hasattr(msg, "timestamp"):
            return int(msg.timestamp)
        if hasattr(msg, "header"):
            stamp = msg.header.stamp
            value = stamp.sec * 1_000_000 + stamp.nanosec // 1000
            return value or None
        return None

    @staticmethod
    def finite_values(values):
        return [float(value) if math.isfinite(value) else None for value in values]

    def on_sample(self, topic, msg):
        now = time.monotonic_ns()
        state = self.states[topic]
        source_us = self.source_timestamp_us(msg)
        if state["last_ns"] is not None:
            gap_ms = (now - state["last_ns"]) / 1e6
            state["intervals_ms"].append(gap_ms)
            state["max_gap_ms"] = max(state["max_gap_ms"], gap_ms)
            if gap_ms >= self.gap_s * 1000.0 and self.is_expected(topic):
                if not state["silent"]:
                    state["gaps"] += 1
                last_source_us = state["last_source_us"]
                source_dt_ms = None if source_us is None or last_source_us is None else (source_us - last_source_us) / 1000.0
                self.record("gap", topic=topic, received_gap_ms=round(gap_ms, 3), source_dt_ms=source_dt_ms)
                print(f"GAP {topic}: received={gap_ms:.1f}ms source_dt={source_dt_ms}ms "
                      f"(total {state['gaps']})", flush=True)

        state["last_ns"] = now
        state["last_source_us"] = source_us
        state["count"] += 1
        state["silent"] = False
        data = {"topic": topic, "source_timestamp_us": source_us}
        if hasattr(msg, "timestamp"):
            data["px4_timestamp_us"] = int(msg.timestamp)
        if isinstance(msg, PoseStamped):
            data["frame_id"] = msg.header.frame_id
            data["position"] = self.finite_values((msg.pose.position.x, msg.pose.position.y, msg.pose.position.z))
            data["orientation_xyzw"] = self.finite_values(
                (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w))
        elif isinstance(msg, VehicleOdometry):
            data["position"] = self.finite_values(msg.position)
            data["q_wxyz"] = self.finite_values(msg.q)
            data["velocity"] = self.finite_values(msg.velocity)
        elif isinstance(msg, VehicleAttitude):
            data["q_wxyz"] = self.finite_values(msg.q)
        elif isinstance(msg, (ActuatorServos, ActuatorMotors)):
            data["control"] = self.finite_values(msg.control)
        self.record("sample", **data)

    def is_expected(self, topic):
        if topic == "/fmu/out/vehicle_attitude":
            return True
        if topic == "/fmu/out/actuator_servos":
            return self.expect_servos or self.armed is True
        if topic in OPTI_STREAMS:
            return self.expect_opti
        return self.armed is True

    def check_silence(self):
        now = time.monotonic_ns()
        for topic, state in self.states.items():
            if not self.is_expected(topic):
                continue
            last_ns = state["last_ns"]
            if last_ns is None:
                if (now - self.started_ns) >= 3e9 and not state["no_data"]:
                    state["no_data"] = True
                    self.record("no_data", topic=topic)
                    print(f"NO DATA {topic}: no samples since monitor start", flush=True)
                continue
            age_s = (now - last_ns) / 1e9
            if age_s >= self.gap_s and not state["silent"]:
                state["silent"] = True
                state["gaps"] += 1
                self.record("silence", topic=topic, age_s=round(age_s, 3))
                print(f"SILENCE {topic}: >{age_s:.3f}s (total {state['gaps']})", flush=True)
            if state["silent"]:
                state["max_gap_ms"] = max(state["max_gap_ms"], age_s * 1000.0)

        if now - self.last_flush_ns >= 1e9:
            self.log.flush()
            self.last_flush_ns = now

        if self.duration_s and (now - self.started_ns) >= self.duration_s * 1e9:
            rclpy.try_shutdown()

    @staticmethod
    def p99(intervals_ms):
        if not intervals_ms:
            return None
        ordered = sorted(intervals_ms)
        return round(ordered[math.ceil(0.99 * len(ordered)) - 1], 2)

    @staticmethod
    def current_processes():
        return {
            "agent": process_pids("MicroXRCEAgent"),
            "servo": process_pids("px4_servo_to_dynamixel"),
        }

    def check_processes(self):
        processes = self.current_processes()
        if processes == self.last_processes:
            return
        previous = self.last_processes
        self.last_processes = processes
        self.record("process_change", previous=previous, **processes)
        print(f"PROCESS CHANGE agent={processes['agent']} servo={processes['servo']}", flush=True)

    def report(self):
        now = time.monotonic_ns()
        interval_s = (now - self.last_report_ns) / 1e9
        self.last_report_ns = now
        processes = self.last_processes or self.current_processes()

        snapshots = {}
        for topic, state in self.states.items():
            age_ms = None if state["last_ns"] is None else (now - state["last_ns"]) / 1e6
            samples = state["count"] - state["reported_count"]
            state["reported_count"] = state["count"]
            snapshots[topic] = {"hz": round(samples / interval_s, 2), "age_ms": None if age_ms is None else round(age_ms, 2),
                                "samples": state["count"], "gaps": state["gaps"],
                                "max_gap_ms": round(state["max_gap_ms"], 2), "p99_ms": self.p99(state["intervals_ms"]),
                                "publishers": len(self.get_publishers_info_by_topic(topic))}

        self.record("health", processes=processes, armed=self.armed, streams=snapshots)
        self.log.flush()
        def short(topic):
            values = snapshots[topic]
            age = "none" if values["age_ms"] is None else f"{values['age_ms']:.0f}ms"
            p99 = "none" if values["p99_ms"] is None else f"{values['p99_ms']:.0f}ms"
            return f"{values['hz']:.1f}Hz age={age} p99={p99} gaps={values['gaps']} max={values['max_gap_ms']:.0f}ms"

        elapsed_s = (now - self.started_ns) / 1e9
        print(f"{elapsed_s:.0f}s agent={'up' if processes['agent'] else 'DOWN'} "
              f"nokov[{short('/nokov_raw/Hummingbird/pose')}] "
              f"opti[{short('/opti_raw')}] "
              f"visual_in[{short('/fmu/in/vehicle_visual_odometry')}] "
              f"odom_out[{short('/fmu/out/vehicle_odometry')}] "
              f"att[{short('/fmu/out/vehicle_attitude')}] "
              f"servo[{short('/fmu/out/actuator_servos')}] "
              f"servo_node={'up' if processes['servo'] else 'off'}", flush=True)

    def close(self):
        totals = {topic: {"samples": state["count"], "gaps": state["gaps"],
                          "max_gap_ms": round(state["max_gap_ms"], 2), "p99_ms": self.p99(state["intervals_ms"])}
                  for topic, state in self.states.items()}
        self.record("stop", elapsed_s=round((time.monotonic_ns() - self.started_ns) / 1e9, 2), streams=totals)
        self.log.close()
        for topic, values in totals.items():
            print(f"TOTAL {topic}: {values}", flush=True)


def main():
    parser = argparse.ArgumentParser(description="Trace HummingBird Opti, PX4 DDS, Agent, and servo-node stability")
    parser.add_argument("--duration", type=float, default=600.0, help="seconds to monitor (default: 600, 0 until Ctrl-C)")
    parser.add_argument("--gap", type=float, default=0.05, help="gap report threshold in seconds (default: 0.05)")
    parser.add_argument("--expect-servos", action="store_true", help="also require servo messages while disarmed")
    parser.add_argument("--expect-opti", action="store_true", help="require the Opti-to-FMU visual-odometry path")
    parser.add_argument("--output", type=Path, help="JSONL file (default: debug/logs/dds_<UTC>.jsonl)")
    args = parser.parse_args()
    if args.duration < 0 or args.gap <= 0:
        parser.error("--duration must be nonnegative and --gap must be positive")

    log_path = args.output or Path(__file__).resolve().parent / "logs" / f"dds_{datetime.now(timezone.utc):%Y%m%d_%H%M%S}.jsonl"
    log_path.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = None
    try:
        node = DdsStability(log_path, args.gap, args.duration, args.expect_servos, args.expect_opti)
        print(f"DDS path monitor: {args.duration:g}s, gap threshold {args.gap:g}s, "
              f"expect_servos={args.expect_servos}, expect_opti={args.expect_opti}, log {log_path}", flush=True)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
