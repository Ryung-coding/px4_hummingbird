#!/usr/bin/env python3
import json
import math
import sys
import threading
import time
from collections import deque
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from px4_msgs.msg import ActuatorMotors, ActuatorServos, HummingbirdStatus, ManualControlSetpoint
from px4_msgs.msg import VehicleAttitude, VehicleAttitudeSetpoint
from px4_msgs.msg import VehicleLocalPosition, VehicleLocalPositionSetpoint
from px4_msgs.msg import VehicleThrustSetpoint, VehicleTorqueSetpoint

try:
    from px4_msgs.msg import VehicleOdometry
except ImportError:
    VehicleOdometry = None

from PyQt5 import QtCore, QtWidgets
import pyqtgraph as pg
try:
    from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
    from matplotlib.backends.backend_qtagg import NavigationToolbar2QT as NavigationToolbar
except ImportError:
    from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
    from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure

WINDOW_SEC = 10.0
MAX_SAMPLES = 20000
UPDATE_MS = 50
RAD2DEG = 180.0 / math.pi
DEFAULT_LOG_DIR = "~/Desktop/px4_hummingbird/src/px4_hummingbird_cmd/scripts/viewer_logs"
R_DOWN = np.diag([1.0, -1.0, -1.0])
DISPLAY_Z_UP = np.diag([1.0, 1.0, -1.0])
FRAME_AXIS_LEN = 0.25
ATTITUDE_FRAME_AXIS_LEN = 0.60
REAL_3D_XY_LIMIT_M = 2.0
REAL_3D_Z_MIN_M = 0.0
REAL_3D_Z_MAX_M = 3.0
REPLAY_SLIDER_STEPS = 10000

C_R = "#e6194b"
C_G = "#3cb44b"
C_B = "#4363d8"
C_O = "#f58231"
C_P = "#911eb4"
C_C = "#00a3a3"
C3 = [C_R, C_G, C_B]
C4 = [C_R, C_G, C_B, C_O]

pg.setConfigOption("background", "w")
pg.setConfigOption("foreground", "k")


class Home3DToolbar(NavigationToolbar):
    def __init__(self, canvas, parent, home_callback):
        super().__init__(canvas, parent)
        self._home_callback = home_callback

    def home(self, *args):
        if self._home_callback is not None:
            self._home_callback()
            return
        super().home(*args)


class Ring:
    def __init__(self, cap, width):
        self._cap = cap
        self._buf = np.full((cap, width), np.nan, dtype=np.float64)
        self._idx = 0
        self._n = 0

    def push(self, row):
        self._buf[self._idx] = row
        self._idx = (self._idx + 1) % self._cap
        self._n = min(self._n + 1, self._cap)

    def get(self):
        if self._n == 0:
            return np.empty((0, self._buf.shape[1]), dtype=np.float64)
        if self._n < self._cap:
            return self._buf[:self._n].copy()
        return np.concatenate([self._buf[self._idx:], self._buf[:self._idx]], axis=0)


class JsonlLogger:
    def __init__(self, enabled, log_dir, metadata):
        self.enabled = bool(enabled)
        self.path = ""
        self._fh = None
        self._lock = threading.Lock()
        self._count = 0

        if not self.enabled:
            return

        try:
            log_root = Path(log_dir).expanduser()
            log_root.mkdir(parents=True, exist_ok=True)
            stamp = time.strftime("%Y%m%d_%H%M%S")
            mode = metadata.get("viewer_mode", "viewer")
            self.path = str(log_root / f"{stamp}_px4_hummingbird_{mode}.jsonl")
            self._fh = open(self.path, "a", encoding="utf-8", buffering=1)
            self.write("viewer_meta", metadata)
        except OSError:
            self.enabled = False
            self.path = ""
            self._fh = None

    def write(self, topic, data):
        if not self.enabled or self._fh is None:
            return

        record = {
            "wall_time": time.time(),
            "topic": topic,
            "data": data,
        }

        with self._lock:
            self._fh.write(json.dumps(record, separators=(",", ":"), ensure_ascii=True) + "\n")
            self._count += 1
            if self._count % 200 == 0:
                self._fh.flush()

    def close(self):
        if self._fh is None:
            return
        with self._lock:
            self._fh.flush()
            self._fh.close()
            self._fh = None


def wrap_deg(x):
    return (x + 180.0) % 360.0 - 180.0


def normalize_quat(q_in):
    q = np.array(q_in, dtype=np.float64)
    n = np.linalg.norm(q)
    if not math.isfinite(float(n)) or n < 1.0e-9:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    return q / n


def quat_to_rot(q_in):
    w, x, y, z = normalize_quat(q_in)
    return np.array([
        [1.0 - 2.0 * (y * y + z * z),       2.0 * (x * y - w * z),       2.0 * (x * z + w * y)],
        [      2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z),       2.0 * (y * z - w * x)],
        [      2.0 * (x * z - w * y),       2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)],
    ], dtype=np.float64)


def rot_to_quat(R):
    tr = float(np.trace(R))
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        q = np.array([
            0.25 * s,
            (R[2, 1] - R[1, 2]) / s,
            (R[0, 2] - R[2, 0]) / s,
            (R[1, 0] - R[0, 1]) / s,
        ], dtype=np.float64)
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(max(1.0 + R[0, 0] - R[1, 1] - R[2, 2], 0.0)) * 2.0
        q = np.array([
            (R[2, 1] - R[1, 2]) / s,
            0.25 * s,
            (R[0, 1] + R[1, 0]) / s,
            (R[0, 2] + R[2, 0]) / s,
        ], dtype=np.float64)
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(max(1.0 + R[1, 1] - R[0, 0] - R[2, 2], 0.0)) * 2.0
        q = np.array([
            (R[0, 2] - R[2, 0]) / s,
            (R[0, 1] + R[1, 0]) / s,
            0.25 * s,
            (R[1, 2] + R[2, 1]) / s,
        ], dtype=np.float64)
    else:
        s = math.sqrt(max(1.0 + R[2, 2] - R[0, 0] - R[1, 1], 0.0)) * 2.0
        q = np.array([
            (R[1, 0] - R[0, 1]) / s,
            (R[0, 2] + R[2, 0]) / s,
            (R[1, 2] + R[2, 1]) / s,
            0.25 * s,
        ], dtype=np.float64)
    return normalize_quat(q)


def quat_to_rpy_deg(q_in):
    w, x, y, z = normalize_quat(q_in)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return np.array([roll, pitch, yaw], dtype=np.float64) * RAD2DEG


def yaw_to_quat(yaw):
    return np.array([math.cos(0.5 * yaw), 0.0, 0.0, math.sin(0.5 * yaw)], dtype=np.float64)


def mocap_pose_to_px4(opti_pos, opti_att, opti_origin):
    pos_px4 = R_DOWN @ (opti_pos - opti_origin)
    R_px4 = R_DOWN @ quat_to_rot(opti_att) @ R_DOWN
    return pos_px4, rot_to_quat(R_px4)


def finite_n(values, width):
    out = np.zeros(width, dtype=np.float64)
    vals = [] if values is None else list(values)
    for i, value in enumerate(vals[:width]):
        try:
            v = float(value)
        except (TypeError, ValueError):
            v = 0.0
        out[i] = 0.0 if not math.isfinite(v) else v
    return out


def finite4(values):
    return finite_n(values, 4)


def json_float(value):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def json_list(values, width=None):
    vals = list(values)
    if width is not None:
        vals = vals[:width]
    return [json_float(v) for v in vals]


def stamp_to_sec(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def cmd_source_label(value):
    return {0: "RC", 1: "DDS"}.get(int(value), "UNKNOWN")


def ctrl_mode_label(value):
    return {0: "CONV", 1: "FULLY"}.get(int(value), "UNKNOWN")


def vec_from_log(data, key, width):
    return finite_n(data.get(key, []), width)


class Px4ViewerNode(Node):
    def __init__(self):
        super().__init__("px4_multirotor_viewer")
        self.lock = threading.Lock()
        self.t0 = self.get_clock().now().nanoseconds * 1e-9
        self.viewer_subscriptions_ = []

        self.viewer_mode = str(self.declare_parameter("viewer_mode", "sim").value).strip().lower()
        if self.viewer_mode not in ("sim", "real"):
            self.get_logger().warning(f"Unknown viewer_mode='{self.viewer_mode}'. Falling back to sim.")
            self.viewer_mode = "sim"
        self.real_mode = self.viewer_mode == "real"

        self.beta_limit_rad = self.declare_parameter("beta_limit_rad", math.pi).value
        self.alpha_limit_rad = self.declare_parameter("alpha_limit_rad", math.pi / 6.0).value
        self.max_rotor_thrust = self.declare_parameter("max_rotor_thrust", 24.0).value

        self.opti_topic = str(self.declare_parameter("opti_topic", "/opti_raw").value)
        self.fmu_odom_topic = str(self.declare_parameter("fmu_odom_topic", "/fmu/out/vehicle_odometry").value)
        self.target_body_name = str(self.declare_parameter("target_body_name", "hummingbird").value)
        self.replay_log_path = str(self.declare_parameter("replay_log_path", "").value).strip()
        self.replay_enabled = bool(self.replay_log_path)
        self.replay_records = []
        self.replay_duration = 0.0
        self.current_replay_time = 0.0
        opti_origin = list(self.declare_parameter("opti_origin", [1.4, 1.4, 0.0]).value)
        if len(opti_origin) != 3:
            self.get_logger().warning("opti_origin must have 3 values. Falling back to [1.4, 1.4, 0.0].")
            opti_origin = [1.4, 1.4, 0.0]
        self.opti_origin = np.array(opti_origin, dtype=np.float64)

        log_enabled = bool(self.declare_parameter("log_enabled", True).value)
        if self.replay_enabled:
            log_enabled = False
        log_dir = str(self.declare_parameter("log_dir", DEFAULT_LOG_DIR).value)
        self.log_dir = str(Path(log_dir).expanduser())
        self.logger = JsonlLogger(log_enabled, self.log_dir, {
            "viewer_mode": self.viewer_mode,
            "window_sec": WINDOW_SEC,
            "max_samples": MAX_SAMPLES,
            "opti_topic": self.opti_topic,
            "fmu_odom_topic": self.fmu_odom_topic,
            "target_body_name": self.target_body_name,
            "opti_origin_z_up_m": json_list(self.opti_origin),
            "real_mode_extra_topics": self.real_mode,
            "replay_log_path": self.replay_log_path,
            "transform": "pos_px4 = R_down * (opti_pos - opti_origin), R_px4 = R_down * R_opti * R_down",
        })

        self.last_pos_sp = np.zeros(3, dtype=np.float64)
        self.last_att_sp_deg = np.zeros(3, dtype=np.float64)
        self.last_thrust_sp = np.zeros(3, dtype=np.float64)
        self.last_torque_sp = np.zeros(3, dtype=np.float64)
        self.last_motor_controls = np.zeros(4, dtype=np.float64)
        self.last_motor_forces = np.zeros(4, dtype=np.float64)
        self.last_servo_controls = np.zeros(6, dtype=np.float64)
        self.last_dynamixel_controls = np.full(6, np.nan, dtype=np.float64)
        self.last_manual = np.zeros(4, dtype=np.float64)
        self.last_manual_valid = False
        self.last_manual_source = 0
        self.last_sticks_moving = False

        self.last_hb_cmd_source = -1
        self.last_hb_ctrl_mode = -1
        self.last_dds_enabled = False
        self.last_hummingbird_airframe = False
        self.last_hb_enabled = False

        self.last_topic_rx = {}
        self.last_opti_rx = None
        self.last_opti_pos = None
        self.last_opti_att = None
        self.last_fmu_pose_rx = None
        self.last_fmu_odom_rx = None
        self.last_fmu_pose_pos = None
        self.last_fmu_pose_att = None
        self.last_fmu_pose_source = "none"
        self.last_arrival_delay_ms = math.nan
        self.last_value_delay_ms = math.nan
        self.last_match_error_m = math.nan
        self.last_live_error_m = math.nan
        self.recent_opti = deque(maxlen=MAX_SAMPLES)

        self.buf_pos = Ring(MAX_SAMPLES, 4)
        self.buf_pos_sp = Ring(MAX_SAMPLES, 4)
        self.buf_rpy = Ring(MAX_SAMPLES, 4)
        self.buf_att_sp = Ring(MAX_SAMPLES, 4)
        self.buf_force = Ring(MAX_SAMPLES, 4)
        self.buf_torque = Ring(MAX_SAMPLES, 4)
        self.buf_motor = Ring(MAX_SAMPLES, 5)
        self.buf_beta = Ring(MAX_SAMPLES, 3)
        self.buf_alpha = Ring(MAX_SAMPLES, 5)
        self.buf_dynamixel_beta = Ring(MAX_SAMPLES, 3)
        self.buf_dynamixel_alpha = Ring(MAX_SAMPLES, 5)

        self.buf_opti_pos = Ring(MAX_SAMPLES, 4)
        self.buf_fmu_pose_pos = Ring(MAX_SAMPLES, 4)
        self.buf_opti_rpy = Ring(MAX_SAMPLES, 4)
        self.buf_fmu_pose_rpy = Ring(MAX_SAMPLES, 4)
        self.buf_delay = Ring(MAX_SAMPLES, 4)
        self.buf_pos_error = Ring(MAX_SAMPLES, 5)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        if self.replay_enabled:
            self._load_replay_records()
        else:
            self._subscribe(VehicleLocalPosition, "/fmu/out/vehicle_local_position", self._cb_pos, qos)
            self._subscribe(VehicleLocalPositionSetpoint, "/fmu/out/vehicle_local_position_setpoint", self._cb_pos_sp, qos)
            self._subscribe(VehicleAttitude, "/fmu/out/vehicle_attitude", self._cb_att, qos)
            self._subscribe(VehicleAttitudeSetpoint, "/fmu/out/vehicle_attitude_setpoint", self._cb_att_sp, qos)
            self._subscribe(VehicleThrustSetpoint, "/fmu/out/vehicle_thrust_setpoint", self._cb_thrust, qos)
            self._subscribe(VehicleTorqueSetpoint, "/fmu/out/vehicle_torque_setpoint", self._cb_torque, qos)
            self._subscribe(ManualControlSetpoint, "/fmu/out/manual_control_setpoint", self._cb_manual, qos)
            self._subscribe(ActuatorMotors, "/fmu/out/actuator_motors", self._cb_motors, qos)
            self._subscribe(ActuatorServos, "/fmu/out/actuator_servos", self._cb_servos, qos)
            self._subscribe(ActuatorServos, "/dynamixel_tilt_mea", self._cb_dynamixel_servos, qos)
            self._subscribe(HummingbirdStatus, "/fmu/out/hummingbird_status", self._cb_status, qos)

            if self.real_mode:
                self._subscribe(PoseStamped, self.opti_topic, self._cb_opti, qos)
                if VehicleOdometry is not None:
                    self._subscribe(VehicleOdometry, self.fmu_odom_topic, self._cb_fmu_odom, qos)
                else:
                    self.get_logger().warning("px4_msgs/msg/VehicleOdometry is not available. Real tab will use vehicle_local_position fallback only.")
                    self.logger.write("viewer_warning", {"message": "VehicleOdometry import failed"})

        self.get_logger().info(f"Viewer mode: {self.viewer_mode}")
        if self.replay_enabled:
            self.get_logger().info(f"Replaying viewer log: {self.replay_log_path}")
        if self.logger.enabled:
            self.get_logger().info(f"Logging viewer data to {self.logger.path}")

    def _subscribe(self, msg_type, topic, callback, qos):
        try:
            sub = self.create_subscription(msg_type, topic, callback, qos)
        except Exception as exc:
            self.get_logger().warning(f"Failed to subscribe {topic}: {exc}")
            self.logger.write("viewer_subscription_error", {"topic": topic, "error": str(exc)})
            return None

        self.viewer_subscriptions_.append(sub)
        self.logger.write("viewer_subscription", {"topic": topic, "type": getattr(msg_type, "__name__", str(msg_type))})
        return sub

    def _t(self):
        return self.get_clock().now().nanoseconds * 1e-9 - self.t0

    def _mark_rx_unlocked(self, key, t):
        self.last_topic_rx[key] = t

    def topic_age(self, key):
        now = self.current_replay_time if self.replay_enabled else self._t()
        with self.lock:
            last = self.last_topic_rx.get(key)
        return None if last is None else now - last

    def _cb_pos(self, msg):
        t = self._t()
        pos = np.array([msg.x, msg.y, msg.z], dtype=np.float64)
        self.logger.write("/fmu/out/vehicle_local_position", {
            "timestamp": int(msg.timestamp),
            "timestamp_sample": int(msg.timestamp_sample),
            "valid": {"xy": bool(msg.xy_valid), "z": bool(msg.z_valid), "v_xy": bool(msg.v_xy_valid), "v_z": bool(msg.v_z_valid)},
            "position": json_list(pos),
            "velocity": json_list([msg.vx, msg.vy, msg.vz]),
            "heading": json_float(msg.heading),
        })

        with self.lock:
            self._mark_rx_unlocked("vehicle_local_position", t)
            self.buf_pos.push([t, pos[0], pos[1], pos[2]])
            use_fallback = self.real_mode and (self.last_fmu_odom_rx is None or (t - self.last_fmu_odom_rx) > 1.0)

        if use_fallback:
            quat = yaw_to_quat(float(msg.heading)) if math.isfinite(float(msg.heading)) else np.array([1.0, 0.0, 0.0, 0.0])
            self._record_fmu_pose(t, pos, quat, "vehicle_local_position")

    def _cb_pos_sp(self, msg):
        t = self._t()
        self.last_pos_sp = np.array([msg.x, msg.y, msg.z], dtype=np.float64)
        self.logger.write("/fmu/out/vehicle_local_position_setpoint", {
            "timestamp": int(msg.timestamp),
            "position": json_list(self.last_pos_sp),
            "yaw": json_float(getattr(msg, "yaw", math.nan)),
        })
        with self.lock:
            self._mark_rx_unlocked("vehicle_local_position_setpoint", t)
            self.buf_pos_sp.push([t, self.last_pos_sp[0], self.last_pos_sp[1], self.last_pos_sp[2]])

    def _cb_att(self, msg):
        t = self._t()
        rpy = quat_to_rpy_deg(msg.q)
        self.logger.write("/fmu/out/vehicle_attitude", {
            "timestamp": int(msg.timestamp),
            "timestamp_sample": int(msg.timestamp_sample),
            "q": json_list(msg.q, 4),
            "rpy_deg": json_list(rpy),
        })
        with self.lock:
            self._mark_rx_unlocked("vehicle_attitude", t)
            self.buf_rpy.push([t, rpy[0], rpy[1], rpy[2]])

    def _cb_att_sp(self, msg):
        t = self._t()
        self.last_att_sp_deg = quat_to_rpy_deg(msg.q_d)
        self.logger.write("/fmu/out/vehicle_attitude_setpoint", {
            "timestamp": int(msg.timestamp),
            "q_d": json_list(msg.q_d, 4),
            "rpy_d_deg": json_list(self.last_att_sp_deg),
            "thrust_body": json_list(getattr(msg, "thrust_body", [])),
        })
        with self.lock:
            self._mark_rx_unlocked("vehicle_attitude_setpoint", t)
            self.buf_att_sp.push([t, self.last_att_sp_deg[0], self.last_att_sp_deg[1], self.last_att_sp_deg[2]])

    def _cb_thrust(self, msg):
        t = self._t()
        self.last_thrust_sp = np.array(msg.xyz, dtype=np.float64)
        self.logger.write("/fmu/out/vehicle_thrust_setpoint", {
            "timestamp": int(msg.timestamp),
            "xyz": json_list(msg.xyz, 3),
        })
        with self.lock:
            self._mark_rx_unlocked("vehicle_thrust_setpoint", t)
            self.buf_force.push([t, self.last_thrust_sp[0], self.last_thrust_sp[1], self.last_thrust_sp[2]])

    def _cb_torque(self, msg):
        t = self._t()
        self.last_torque_sp = np.array(msg.xyz, dtype=np.float64)
        self.logger.write("/fmu/out/vehicle_torque_setpoint", {
            "timestamp": int(msg.timestamp),
            "xyz": json_list(msg.xyz, 3),
        })
        with self.lock:
            self._mark_rx_unlocked("vehicle_torque_setpoint", t)
            self.buf_torque.push([t, self.last_torque_sp[0], self.last_torque_sp[1], self.last_torque_sp[2]])

    def _cb_motors(self, msg):
        t = self._t()
        controls = finite4(msg.control)
        force = np.square(np.clip(controls, 0.0, 1.0)) * self.max_rotor_thrust
        self.logger.write("/fmu/out/actuator_motors", {
            "timestamp": int(msg.timestamp),
            "timestamp_sample": int(msg.timestamp_sample),
            "control": json_list(msg.control),
            "force_n_est": json_list(force, 4),
        })
        with self.lock:
            self._mark_rx_unlocked("actuator_motors", t)
            self.last_motor_controls = controls
            self.last_motor_forces = force
            self.buf_motor.push([t, force[0], force[1], force[2], force[3]])

    def _cb_manual(self, msg):
        t = self._t()
        manual = np.array([msg.roll, msg.pitch, msg.yaw, msg.throttle], dtype=np.float64)
        self.logger.write("/fmu/out/manual_control_setpoint", {
            "timestamp": int(msg.timestamp),
            "valid": bool(msg.valid),
            "data_source": int(msg.data_source),
            "sticks_moving": bool(msg.sticks_moving),
            "roll_pitch_yaw_throttle": json_list(manual),
        })
        with self.lock:
            self._mark_rx_unlocked("manual_control_setpoint", t)
            self.last_manual = manual
            self.last_manual_valid = bool(msg.valid)
            self.last_manual_source = int(msg.data_source)
            self.last_sticks_moving = bool(msg.sticks_moving)

    def _cb_servos(self, msg):
        t = self._t()
        controls = finite_n(msg.control, 6)
        beta = finite_n(msg.control[0:2], 2) * self.beta_limit_rad * RAD2DEG
        alpha = finite_n(msg.control[2:6], 4) * self.alpha_limit_rad * RAD2DEG
        self.logger.write("/fmu/out/actuator_servos", {
            "timestamp": int(msg.timestamp),
            "timestamp_sample": int(msg.timestamp_sample),
            "control": json_list(msg.control),
            "beta_deg": json_list(beta, 2),
            "alpha_deg": json_list(alpha, 4),
        })
        with self.lock:
            self._mark_rx_unlocked("actuator_servos", t)
            self.last_servo_controls = controls
            self.buf_beta.push([t, beta[0], beta[1]])
            self.buf_alpha.push([t, alpha[0], alpha[1], alpha[2], alpha[3]])

    def _cb_dynamixel_servos(self, msg):
        t = self._t()
        controls = finite_n(msg.control, 6)
        beta = finite_n(msg.control[0:2], 2) * self.beta_limit_rad * RAD2DEG
        alpha = finite_n(msg.control[2:6], 4) * self.alpha_limit_rad * RAD2DEG
        self.logger.write("/dynamixel_tilt_mea", {
            "timestamp": int(msg.timestamp),
            "timestamp_sample": int(msg.timestamp_sample),
            "control": json_list(msg.control),
            "beta_deg": json_list(beta, 2),
            "alpha_deg": json_list(alpha, 4),
        })
        with self.lock:
            self._mark_rx_unlocked("dynamixel_tilt_mea", t)
            self.last_dynamixel_controls = controls
            self.buf_dynamixel_beta.push([t, beta[0], beta[1]])
            self.buf_dynamixel_alpha.push([t, alpha[0], alpha[1], alpha[2], alpha[3]])

    def _cb_status(self, msg):
        hb_enabled = getattr(
            msg,
            "hummingbird_control_enabled",
            getattr(msg, "fully_actuated_control_enabled", False),
        )
        airframe = bool(getattr(msg, "hummingbird_airframe", hb_enabled))
        self.logger.write("/fmu/out/hummingbird_status", {
            "timestamp": int(msg.timestamp),
            "hb_cmd_source": int(msg.hb_cmd_source),
            "hb_ctrl_mode": int(msg.hb_ctrl_mode),
            "hummingbird_airframe": bool(airframe),
            "dds_command_enabled": bool(msg.dds_command_enabled),
            "hummingbird_control_enabled": bool(hb_enabled),
        })
        with self.lock:
            self._mark_rx_unlocked("hummingbird_status", self._t())
            self.last_hb_cmd_source = int(msg.hb_cmd_source)
            self.last_hb_ctrl_mode = int(msg.hb_ctrl_mode)
            self.last_dds_enabled = bool(msg.dds_command_enabled)
            self.last_hummingbird_airframe = bool(airframe)
            self.last_hb_enabled = bool(hb_enabled)

    def _cb_opti(self, msg):
        if self.target_body_name and msg.header.frame_id != self.target_body_name:
            return

        t = self._t()
        opti_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype=np.float64)
        opti_att = np.array([msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z], dtype=np.float64)
        pos_px4, att_px4 = mocap_pose_to_px4(opti_pos, opti_att, self.opti_origin)
        rpy_px4 = quat_to_rpy_deg(att_px4)

        self.logger.write(self.opti_topic, {
            "header_stamp": stamp_to_sec(msg.header.stamp),
            "frame_id": msg.header.frame_id,
            "opti_position_z_up": json_list(opti_pos),
            "opti_q_wxyz": json_list(opti_att, 4),
            "px4_position_ned": json_list(pos_px4),
            "px4_q_wxyz": json_list(att_px4, 4),
            "px4_rpy_deg": json_list(rpy_px4),
        })

        with self.lock:
            self._mark_rx_unlocked("opti_raw", t)
            self.last_opti_rx = t
            self.last_opti_pos = pos_px4.copy()
            self.last_opti_att = att_px4.copy()
            self.recent_opti.append((t, pos_px4.copy(), att_px4.copy()))
            self.buf_opti_pos.push([t, pos_px4[0], pos_px4[1], pos_px4[2]])
            self.buf_opti_rpy.push([t, rpy_px4[0], rpy_px4[1], rpy_px4[2]])

    def _cb_fmu_odom(self, msg):
        t = self._t()
        pos = np.array(msg.position, dtype=np.float64)
        quat = normalize_quat(msg.q)
        self.logger.write(self.fmu_odom_topic, {
            "timestamp": int(msg.timestamp),
            "timestamp_sample": int(msg.timestamp_sample),
            "pose_frame": int(msg.pose_frame),
            "position": json_list(msg.position, 3),
            "q_wxyz": json_list(msg.q, 4),
            "velocity_frame": int(msg.velocity_frame),
            "velocity": json_list(msg.velocity, 3),
            "angular_velocity": json_list(msg.angular_velocity, 3),
            "position_variance": json_list(msg.position_variance, 3),
            "orientation_variance": json_list(msg.orientation_variance, 3),
            "quality": int(msg.quality),
        })
        with self.lock:
            self._mark_rx_unlocked("vehicle_odometry", t)
            self.last_fmu_odom_rx = t
        self._record_fmu_pose(t, pos, quat, "vehicle_odometry")

    def _record_fmu_pose(self, t, pos, quat, source):
        if not np.all(np.isfinite(pos)):
            return

        rpy = quat_to_rpy_deg(quat)
        with self.lock:
            self.last_fmu_pose_source = source
            self.last_fmu_pose_rx = t
            self.last_fmu_pose_pos = pos.copy()
            self.last_fmu_pose_att = normalize_quat(quat).copy()
            self.buf_fmu_pose_pos.push([t, pos[0], pos[1], pos[2]])
            self.buf_fmu_pose_rpy.push([t, rpy[0], rpy[1], rpy[2]])

            if self.last_opti_pos is not None:
                err = pos - self.last_opti_pos
                err_norm = float(np.linalg.norm(err))
                self.last_live_error_m = err_norm
                self.buf_pos_error.push([t, err[0], err[1], err[2], err_norm])

            if self.last_opti_rx is not None:
                self.last_arrival_delay_ms = (t - self.last_opti_rx) * 1000.0

            candidates = [sample for sample in self.recent_opti if 0.0 <= (t - sample[0]) <= 2.0]
            if candidates:
                best_t, best_pos, _ = min(candidates, key=lambda sample: np.linalg.norm(pos - sample[1]))
                match_error = float(np.linalg.norm(pos - best_pos))
                self.last_value_delay_ms = (t - best_t) * 1000.0
                self.last_match_error_m = match_error
                self.buf_delay.push([t, self.last_arrival_delay_ms, self.last_value_delay_ms, match_error])

    def _load_replay_records(self):
        path = Path(self.replay_log_path).expanduser()
        if not path.is_absolute() and path.parent == Path("."):
            path = Path(self.log_dir) / path
        records = []
        first_wall_time = None
        try:
            with open(path, "r", encoding="utf-8") as fh:
                for line in fh:
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        record = json.loads(line)
                    except json.JSONDecodeError:
                        continue
                    if record.get("topic") == "viewer_meta":
                        continue
                    wall_time = json_float(record.get("wall_time"))
                    if wall_time is None:
                        continue
                    if first_wall_time is None:
                        first_wall_time = wall_time
                    record["_replay_t"] = wall_time - first_wall_time
                    records.append(record)
        except OSError as exc:
            self.get_logger().warning(f"Failed to open replay log {path}: {exc}")

        self.replay_records = records
        self.replay_duration = float(records[-1].get("_replay_t", 0.0)) if records else 0.0
        self.current_replay_time = self.replay_duration
        self.get_logger().info(f"Loaded {len(records)} replay records from {path}")

    def apply_replay_window(self, cursor_time, window_sec):
        if not self.replay_enabled:
            return

        cursor_time = max(0.0, min(float(cursor_time), self.replay_duration))
        window_sec = max(0.1, float(window_sec))
        start_time = max(0.0, cursor_time - window_sec)

        with self.lock:
            self._reset_replay_view_state_unlocked()
            self.current_replay_time = cursor_time

        for record in self.replay_records:
            t = float(record.get("_replay_t", 0.0))
            if t > cursor_time:
                break
            if t >= start_time:
                self._ingest_replay_record(record)

        with self.lock:
            self.current_replay_time = cursor_time

    def _reset_replay_view_state_unlocked(self):
        self.last_pos_sp = np.zeros(3, dtype=np.float64)
        self.last_att_sp_deg = np.zeros(3, dtype=np.float64)
        self.last_thrust_sp = np.zeros(3, dtype=np.float64)
        self.last_torque_sp = np.zeros(3, dtype=np.float64)
        self.last_motor_controls = np.zeros(4, dtype=np.float64)
        self.last_motor_forces = np.zeros(4, dtype=np.float64)
        self.last_servo_controls = np.zeros(6, dtype=np.float64)
        self.last_dynamixel_controls = np.full(6, np.nan, dtype=np.float64)
        self.last_manual = np.zeros(4, dtype=np.float64)
        self.last_manual_valid = False
        self.last_manual_source = 0
        self.last_sticks_moving = False
        self.last_hb_cmd_source = -1
        self.last_hb_ctrl_mode = -1
        self.last_dds_enabled = False
        self.last_hummingbird_airframe = False
        self.last_hb_enabled = False
        self.last_topic_rx = {}
        self.last_opti_rx = None
        self.last_opti_pos = None
        self.last_opti_att = None
        self.last_fmu_pose_rx = None
        self.last_fmu_odom_rx = None
        self.last_fmu_pose_pos = None
        self.last_fmu_pose_att = None
        self.last_fmu_pose_source = "none"
        self.last_arrival_delay_ms = math.nan
        self.last_value_delay_ms = math.nan
        self.last_match_error_m = math.nan
        self.last_live_error_m = math.nan
        self.recent_opti = deque(maxlen=MAX_SAMPLES)
        self.buf_pos = Ring(MAX_SAMPLES, 4)
        self.buf_pos_sp = Ring(MAX_SAMPLES, 4)
        self.buf_rpy = Ring(MAX_SAMPLES, 4)
        self.buf_att_sp = Ring(MAX_SAMPLES, 4)
        self.buf_force = Ring(MAX_SAMPLES, 4)
        self.buf_torque = Ring(MAX_SAMPLES, 4)
        self.buf_motor = Ring(MAX_SAMPLES, 5)
        self.buf_beta = Ring(MAX_SAMPLES, 3)
        self.buf_alpha = Ring(MAX_SAMPLES, 5)
        self.buf_dynamixel_beta = Ring(MAX_SAMPLES, 3)
        self.buf_dynamixel_alpha = Ring(MAX_SAMPLES, 5)
        self.buf_opti_pos = Ring(MAX_SAMPLES, 4)
        self.buf_fmu_pose_pos = Ring(MAX_SAMPLES, 4)
        self.buf_opti_rpy = Ring(MAX_SAMPLES, 4)
        self.buf_fmu_pose_rpy = Ring(MAX_SAMPLES, 4)
        self.buf_delay = Ring(MAX_SAMPLES, 4)
        self.buf_pos_error = Ring(MAX_SAMPLES, 5)

    def _ingest_replay_record(self, record):
        topic = str(record.get("topic", ""))
        data = record.get("data", {})
        t = float(record.get("_replay_t", 0.0))

        if topic == "/fmu/out/vehicle_local_position":
            pos = vec_from_log(data, "position", 3)
            with self.lock:
                self._mark_rx_unlocked("vehicle_local_position", t)
                self.buf_pos.push([t, pos[0], pos[1], pos[2]])
            if self.real_mode:
                heading = json_float(data.get("heading"))
                quat = yaw_to_quat(heading) if heading is not None else np.array([1.0, 0.0, 0.0, 0.0])
                self._record_fmu_pose(t, pos, quat, "vehicle_local_position")
            return

        if topic == "/fmu/out/vehicle_local_position_setpoint":
            pos_sp = vec_from_log(data, "position", 3)
            with self.lock:
                self._mark_rx_unlocked("vehicle_local_position_setpoint", t)
                self.last_pos_sp = pos_sp
                self.buf_pos_sp.push([t, pos_sp[0], pos_sp[1], pos_sp[2]])
            return

        if topic == "/fmu/out/vehicle_attitude":
            q = vec_from_log(data, "q", 4)
            rpy = vec_from_log(data, "rpy_deg", 3) if "rpy_deg" in data else quat_to_rpy_deg(q)
            with self.lock:
                self._mark_rx_unlocked("vehicle_attitude", t)
                self.buf_rpy.push([t, rpy[0], rpy[1], rpy[2]])
            return

        if topic == "/fmu/out/vehicle_attitude_setpoint":
            q = vec_from_log(data, "q_d", 4)
            rpy = vec_from_log(data, "rpy_d_deg", 3) if "rpy_d_deg" in data else quat_to_rpy_deg(q)
            with self.lock:
                self._mark_rx_unlocked("vehicle_attitude_setpoint", t)
                self.last_att_sp_deg = rpy
                self.buf_att_sp.push([t, rpy[0], rpy[1], rpy[2]])
            return

        if topic == "/fmu/out/vehicle_thrust_setpoint":
            thrust = vec_from_log(data, "xyz", 3)
            with self.lock:
                self._mark_rx_unlocked("vehicle_thrust_setpoint", t)
                self.last_thrust_sp = thrust
                self.buf_force.push([t, thrust[0], thrust[1], thrust[2]])
            return

        if topic == "/fmu/out/vehicle_torque_setpoint":
            torque = vec_from_log(data, "xyz", 3)
            with self.lock:
                self._mark_rx_unlocked("vehicle_torque_setpoint", t)
                self.last_torque_sp = torque
                self.buf_torque.push([t, torque[0], torque[1], torque[2]])
            return

        if topic == "/fmu/out/actuator_motors":
            controls = vec_from_log(data, "control", 4)
            force = vec_from_log(data, "force_n_est", 4) if "force_n_est" in data else np.square(np.clip(controls, 0.0, 1.0)) * self.max_rotor_thrust
            with self.lock:
                self._mark_rx_unlocked("actuator_motors", t)
                self.last_motor_controls = controls
                self.last_motor_forces = force
                self.buf_motor.push([t, force[0], force[1], force[2], force[3]])
            return

        if topic == "/fmu/out/manual_control_setpoint":
            manual = vec_from_log(data, "roll_pitch_yaw_throttle", 4)
            with self.lock:
                self._mark_rx_unlocked("manual_control_setpoint", t)
                self.last_manual = manual
                self.last_manual_valid = bool(data.get("valid", False))
                self.last_manual_source = int(data.get("data_source", 0))
                self.last_sticks_moving = bool(data.get("sticks_moving", False))
            return

        if topic in ("/fmu/out/actuator_servos", "/dynamixel_tilt_mea"):
            controls = vec_from_log(data, "control", 6)
            beta = vec_from_log(data, "beta_deg", 2) if "beta_deg" in data else controls[0:2] * self.beta_limit_rad * RAD2DEG
            alpha = vec_from_log(data, "alpha_deg", 4) if "alpha_deg" in data else controls[2:6] * self.alpha_limit_rad * RAD2DEG
            with self.lock:
                key = "actuator_servos" if topic == "/fmu/out/actuator_servos" else "dynamixel_tilt_mea"
                self._mark_rx_unlocked(key, t)
                if topic == "/fmu/out/actuator_servos":
                    self.last_servo_controls = controls
                    self.buf_beta.push([t, beta[0], beta[1]])
                    self.buf_alpha.push([t, alpha[0], alpha[1], alpha[2], alpha[3]])
                else:
                    self.last_dynamixel_controls = controls
                    self.buf_dynamixel_beta.push([t, beta[0], beta[1]])
                    self.buf_dynamixel_alpha.push([t, alpha[0], alpha[1], alpha[2], alpha[3]])
            return

        if topic == "/fmu/out/hummingbird_status":
            with self.lock:
                self._mark_rx_unlocked("hummingbird_status", t)
                self.last_hb_cmd_source = int(data.get("hb_cmd_source", -1))
                self.last_hb_ctrl_mode = int(data.get("hb_ctrl_mode", -1))
                self.last_dds_enabled = bool(data.get("dds_command_enabled", False))
                self.last_hummingbird_airframe = bool(data.get("hummingbird_airframe", data.get("hummingbird_control_enabled", False)))
                self.last_hb_enabled = bool(data.get("hummingbird_control_enabled", False))
            return

        if "px4_position_ned" in data:
            pos = vec_from_log(data, "px4_position_ned", 3)
            q = vec_from_log(data, "px4_q_wxyz", 4)
            rpy = vec_from_log(data, "px4_rpy_deg", 3) if "px4_rpy_deg" in data else quat_to_rpy_deg(q)
            with self.lock:
                self._mark_rx_unlocked("opti_raw", t)
                self.last_opti_rx = t
                self.last_opti_pos = pos.copy()
                self.last_opti_att = normalize_quat(q).copy()
                self.recent_opti.append((t, pos.copy(), q.copy()))
                self.buf_opti_pos.push([t, pos[0], pos[1], pos[2]])
                self.buf_opti_rpy.push([t, rpy[0], rpy[1], rpy[2]])
            return

        if topic == self.fmu_odom_topic or topic == "/fmu/out/vehicle_odometry":
            pos = vec_from_log(data, "position", 3)
            q = vec_from_log(data, "q_wxyz", 4)
            with self.lock:
                self._mark_rx_unlocked("vehicle_odometry", t)
                self.last_fmu_odom_rx = t
            self._record_fmu_pose(t, pos, q, "vehicle_odometry")

    def close_logger(self):
        self.logger.close()


def _pen(color, width=3):
    return pg.mkPen(color=color, width=width, style=QtCore.Qt.SolidLine)


def _cmd_pen(width=3):
    return pg.mkPen(color="k", width=width, style=QtCore.Qt.DashLine)


def _mea_pen(color, width=3):
    return pg.mkPen(color=color, width=width, style=QtCore.Qt.DashLine)


def _style_plot(plot, ylabel, xlabel=None):
    plot.showGrid(x=True, y=True, alpha=0.3)
    plot.setLabel("left", ylabel)
    if xlabel is not None:
        plot.setLabel("bottom", xlabel)
    plot.getAxis("left").enableAutoSIPrefix(False)
    plot.getAxis("bottom").enableAutoSIPrefix(False)
    for axis in ("bottom", "left"):
        plot.getAxis(axis).setPen(pg.mkPen("k"))
        plot.getAxis(axis).setTextPen(pg.mkPen("k"))
    plot.addLegend(offset=(-10, 5))
    return plot


def _mkplot(glw, row, col, title, ylabel, rowspan=1, colspan=1):
    plot = glw.addPlot(row=row, col=col, rowspan=rowspan, colspan=colspan, title=title)
    return _style_plot(plot, ylabel)


def _mk_plot_widget(title, ylabel, xlabel=None):
    return _style_plot(pg.PlotWidget(title=title), ylabel, xlabel)


def _front(curve):
    curve.setZValue(5)
    return curve


class ViewerWindow(QtWidgets.QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle(f"PX4 HummingBird DDS Viewer ({node.viewer_mode})")
        self.resize(1800, 1050)

        central = QtWidgets.QWidget()
        root = QtWidgets.QVBoxLayout(central)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(6)
        self.setCentralWidget(central)

        self.status_widget = QtWidgets.QWidget()
        status_layout = QtWidgets.QHBoxLayout(self.status_widget)
        status_layout.setContentsMargins(6, 2, 6, 2)
        status_layout.setSpacing(8)

        title = QtWidgets.QLabel("HummingBird Status")
        title.setStyleSheet("font-size: 15pt; font-weight: 700; color: #111;")
        status_layout.addWidget(title)

        self.badge_mode = QtWidgets.QLabel()
        self.badge_log = QtWidgets.QLabel()
        self.badge_hb = QtWidgets.QLabel()
        self.badge_dds = QtWidgets.QLabel()
        self.badge_manual = QtWidgets.QLabel()
        self.badge_cmd = QtWidgets.QLabel()
        self.badge_ctrl = QtWidgets.QLabel()
        self.badge_opti = QtWidgets.QLabel()
        self.badge_fmu = QtWidgets.QLabel()
        self.badge_delay = QtWidgets.QLabel()
        self.badge_error = QtWidgets.QLabel()

        badges = [
            self.badge_mode,
            self.badge_log,
            self.badge_hb,
            self.badge_dds,
            self.badge_manual,
            self.badge_cmd,
            self.badge_ctrl,
        ]
        if node.real_mode:
            badges.extend([self.badge_opti, self.badge_fmu, self.badge_delay, self.badge_error])

        for badge in badges:
            badge.setAlignment(QtCore.Qt.AlignCenter)
            badge.setMinimumWidth(104)
            badge.setMinimumHeight(32)
            status_layout.addWidget(badge)

        status_layout.addStretch(1)
        root.addWidget(self.status_widget)

        tabs = QtWidgets.QTabWidget()
        root.addWidget(tabs, 1)

        self.cv = {}
        self.state_plots = []
        self.act_plots = []
        self.real_plots = []
        self.real_time_plots = []
        self.real_bottom_time_plots = []
        self.window_sec = WINDOW_SEC
        self.replay_position_slider = None
        self.replay_window_slider = None
        self.replay_position_label = None
        self.replay_window_label = None
        self.real_3d_enabled = False
        self.real_3d_path_canvas = None
        self.real_3d_path_ax = None
        self.real_3d_att_canvas = None
        self.real_3d_att_ax = None
        self.real_3d_path_toolbar = None
        self.real_3d_att_toolbar = None
        self.real_3d_view_initialized = False
        self.real_3d_interacting = False
        self.real_3d_last_draw_t = 0.0

        self.state_glw = pg.GraphicsLayoutWidget()
        self.act_glw = pg.GraphicsLayoutWidget()
        tabs.addTab(self.state_glw, "State")
        tabs.addTab(self.act_glw, "Actuator")
        self._build_state_tab()
        self._build_act_tab()

        if node.real_mode:
            self.real_tab = QtWidgets.QWidget()
            self.real_layout = QtWidgets.QGridLayout(self.real_tab)
            self.real_layout.setContentsMargins(0, 0, 0, 0)
            self.real_layout.setSpacing(6)
            for col in range(4):
                self.real_layout.setColumnStretch(col, 1)
            for row in range(3):
                self.real_layout.setRowStretch(row, 1)
            tabs.addTab(self.real_tab, "Real Opti")
            self._build_real_tab()
            self.real_3d_tab = self._build_real_3d_tab()
            tabs.addTab(self.real_3d_tab, "Real 3D v2")

        if node.replay_enabled:
            root.addWidget(self._build_replay_controls())
            self._on_replay_controls_changed()

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self._update)
        self.timer.start(UPDATE_MS)

    def _build_replay_controls(self):
        widget = QtWidgets.QWidget()
        layout = QtWidgets.QGridLayout(widget)
        layout.setContentsMargins(6, 2, 6, 2)
        layout.setHorizontalSpacing(10)
        layout.setVerticalSpacing(2)

        self.replay_position_label = QtWidgets.QLabel()
        self.replay_window_label = QtWidgets.QLabel()
        self.replay_position_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.replay_window_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)

        self.replay_position_slider.setRange(0, REPLAY_SLIDER_STEPS)
        self.replay_position_slider.setValue(REPLAY_SLIDER_STEPS)
        max_window = max(1, int(math.ceil(max(self.node.replay_duration, WINDOW_SEC))))
        self.replay_window_slider.setRange(1, max_window)
        self.replay_window_slider.setValue(min(max_window, int(round(WINDOW_SEC))))

        self.replay_position_slider.valueChanged.connect(self._on_replay_controls_changed)
        self.replay_window_slider.valueChanged.connect(self._on_replay_controls_changed)

        layout.addWidget(QtWidgets.QLabel("Log position"), 0, 0)
        layout.addWidget(self.replay_position_slider, 0, 1)
        layout.addWidget(self.replay_position_label, 0, 2)
        layout.addWidget(QtWidgets.QLabel("Window"), 1, 0)
        layout.addWidget(self.replay_window_slider, 1, 1)
        layout.addWidget(self.replay_window_label, 1, 2)
        layout.setColumnStretch(1, 1)
        return widget

    def _on_replay_controls_changed(self):
        if not self.node.replay_enabled or self.replay_position_slider is None:
            return

        duration = max(self.node.replay_duration, 0.0)
        cursor = duration * float(self.replay_position_slider.value()) / float(REPLAY_SLIDER_STEPS) if duration > 0.0 else 0.0
        window = float(self.replay_window_slider.value())
        self.window_sec = window
        self.node.apply_replay_window(cursor, window)

        if self.replay_position_label is not None:
            self.replay_position_label.setText(f"{cursor:.1f} / {duration:.1f} s")
        if self.replay_window_label is not None:
            self.replay_window_label.setText(f"{window:.0f} s")
        self._update()

    @staticmethod
    def _set_bool_badge(label, name, enabled):
        bg = "#2e7d32" if enabled else "#c62828"
        state = "ON" if enabled else "OFF"
        label.setText(f"{name}: {state}")
        label.setStyleSheet(
            f"background:{bg}; color:white; border-radius:6px; "
            "padding:5px 10px; font-size:11pt; font-weight:700;"
        )

    @staticmethod
    def _set_value_badge(label, name, value, bg="#424242"):
        label.setText(f"{name}: {value}")
        label.setStyleSheet(
            f"background:{bg}; color:white; border-radius:6px; "
            "padding:5px 10px; font-size:11pt; font-weight:700;"
        )

    @staticmethod
    def _set_age_badge(label, name, age):
        if age is None:
            state = "NO DATA"
            bg = "#ef6c00"
        elif age < 0.5:
            state = "OK"
            bg = "#2e7d32"
        elif age < 2.0:
            state = "STALE"
            bg = "#ef6c00"
        else:
            state = "LOST"
            bg = "#c62828"
        ViewerWindow._set_value_badge(label, name, state, bg)

    def _update_status_bar(self):
        nd = self.node
        with nd.lock:
            hb_airframe = nd.last_hummingbird_airframe
            dds = nd.last_dds_enabled
            manual = nd.last_manual_valid
            cmd = nd.last_hb_cmd_source
            ctrl = nd.last_hb_ctrl_mode
            source = nd.last_fmu_pose_source
            value_delay_ms = nd.last_value_delay_ms
            match_error_m = nd.last_match_error_m
            live_error_m = nd.last_live_error_m

        view_value = "REPLAY" if nd.replay_enabled else nd.viewer_mode.upper()
        self._set_value_badge(self.badge_mode, "VIEW", view_value, "#6a1b9a" if nd.replay_enabled else "#1565c0" if nd.real_mode else "#455a64")
        self.badge_mode.setToolTip("viewer_mode: sim=Gazebo/SITL, real=onboard Wi-Fi + OptiTrack, replay=JSONL playback")

        log_value = "REPLAY" if nd.replay_enabled else "ON" if nd.logger.enabled else "OFF"
        log_bg = "#6a1b9a" if nd.replay_enabled else "#2e7d32" if nd.logger.enabled else "#757575"
        self._set_value_badge(self.badge_log, "LOG", log_value, log_bg)
        self.badge_log.setToolTip(nd.replay_log_path if nd.replay_enabled else nd.logger.path if nd.logger.path else "logging disabled")

        self._set_bool_badge(self.badge_hb, "HB AIR", hb_airframe)
        self.badge_hb.setToolTip("hummingbird_status.hummingbird_airframe")
        self._set_bool_badge(self.badge_dds, "DDS IN", dds)
        self.badge_dds.setToolTip("hummingbird_status.dds_command_enabled")
        self._set_bool_badge(self.badge_manual, "RC IN", manual)
        self.badge_manual.setToolTip("manual_control_setpoint.valid")

        cmd_label = cmd_source_label(cmd)
        cmd_bg = "#1565c0" if cmd_label == "DDS" else "#455a64" if cmd_label == "RC" else "#757575"
        self._set_value_badge(self.badge_cmd, "CMD", cmd_label, cmd_bg)
        self.badge_cmd.setToolTip("HB_CMD_SOURCE: RC=0, DDS=1")

        ctrl_label = ctrl_mode_label(ctrl)
        ctrl_bg = "#2e7d32" if ctrl_label == "FULLY" else "#455a64" if ctrl_label == "CONV" else "#757575"
        self._set_value_badge(self.badge_ctrl, "CTRL", ctrl_label, ctrl_bg)
        self.badge_ctrl.setToolTip("HB_CTRL_MODE: CONV=0, FULLY=1")

        if nd.real_mode:
            odom_age = nd.topic_age("vehicle_odometry")
            local_age = nd.topic_age("vehicle_local_position")
            fmu_age = odom_age if odom_age is not None and odom_age < 1.0 else local_age if local_age is not None else odom_age
            self._set_age_badge(self.badge_opti, "OPTI", nd.topic_age("opti_raw"))
            self.badge_opti.setToolTip(f"raw mocap topic: {nd.opti_topic}, frame_id={nd.target_body_name}")
            self._set_age_badge(self.badge_fmu, "FMU", fmu_age)
            self.badge_fmu.setToolTip(f"preferred={nd.fmu_odom_topic}, fallback=/fmu/out/vehicle_local_position, current={source}")
            if math.isfinite(value_delay_ms):
                bg = "#2e7d32" if value_delay_ms < 50.0 else "#ef6c00" if value_delay_ms < 150.0 else "#c62828"
                self._set_value_badge(self.badge_delay, "DLY", f"{value_delay_ms:.0f}ms", bg)
                self.badge_delay.setToolTip(f"value-match delay, source={source}, match_error={match_error_m:.4f}m")
            else:
                self._set_value_badge(self.badge_delay, "DLY", "WAIT", "#757575")
                self.badge_delay.setToolTip("waiting for comparable Opti/FMU position samples")

            if math.isfinite(live_error_m):
                bg = "#2e7d32" if live_error_m < 0.05 else "#ef6c00" if live_error_m < 0.15 else "#c62828"
                self._set_value_badge(self.badge_error, "ERR", f"{live_error_m:.3f}m", bg)
                self.badge_error.setToolTip(f"live FMU-Opti position error, match_error={match_error_m:.4f}m")
            else:
                self._set_value_badge(self.badge_error, "ERR", "WAIT", "#757575")
                self.badge_error.setToolTip("waiting for Opti/FMU position samples")

    def _build_state_tab(self):
        pos_lbl = ["x", "y", "z"]
        rpy_lbl = ["roll", "pitch", "yaw"]
        trq_lbl = ["Mx", "My", "Mz"]
        frc_lbl = ["Fx", "Fy", "Fz"]

        for i in range(3):
            p = _mkplot(self.state_glw, 0, i, f"{pos_lbl[i]} / sp", f"{pos_lbl[i]} [m]")
            self.cv[f"pos{i}"] = p.plot(pen=_pen(C3[i]), name=pos_lbl[i])
            self.cv[f"pos_sp{i}"] = _front(p.plot(pen=_cmd_pen(), name="sp"))
            self.state_plots.append(p)

        for i in range(3):
            p = _mkplot(self.state_glw, 1, i, f"{rpy_lbl[i]} / sp", f"{rpy_lbl[i]} [deg]")
            self.cv[f"rpy{i}"] = p.plot(pen=_pen(C3[i]), name=rpy_lbl[i])
            self.cv[f"att_sp{i}"] = _front(p.plot(pen=_cmd_pen(), name="sp"))
            self.state_plots.append(p)

        for i in range(3):
            p = _mkplot(self.state_glw, 2, i, trq_lbl[i], "norm")
            self.cv[f"torque{i}"] = p.plot(pen=_pen(C3[i]), name=trq_lbl[i])
            self.state_plots.append(p)

        for i in range(3):
            p = _mkplot(self.state_glw, 3, i, f"Body thrust {frc_lbl[i]}", "norm")
            self.cv[f"force{i}"] = p.plot(pen=_pen(C3[i]), name=frc_lbl[i])
            self.state_plots.append(p)

        for p in self.state_plots[1:]:
            p.setXLink(self.state_plots[0])

        for p in self.state_plots:
            p.hideAxis("bottom")

        for p in self.state_plots[-3:]:
            p.showAxis("bottom")
            p.setLabel("bottom", "time [s]")

    def _add_angle_plot(self, row, col, title, cmd_key, mea_key, color, y_min, y_max):
        p = _mkplot(self.act_glw, row, col, title, "angle [deg]")
        self.cv[cmd_key] = p.plot(pen=_pen(color), name="cmd")
        self.cv[mea_key] = _front(p.plot(pen=_mea_pen(color), name="mea"))
        p.setYRange(y_min, y_max, padding=0.03)
        self.act_plots.append(p)
        return p

    def _build_act_tab(self):
        self._add_angle_plot(0, 0, "beta1: cmd / mea", "beta0", "dxl_beta0", C_R, -190.0, 190.0)
        self._add_angle_plot(0, 1, "beta2: cmd / mea", "beta1", "dxl_beta1", C_G, -190.0, 190.0)

        p_force = _mkplot(self.act_glw, 0, 2, "Rotor thrust f1-f4", "force [N]", colspan=2)
        for i, color in enumerate(C4):
            self.cv[f"motor_all{i}"] = p_force.plot(pen=_pen(color), name=f"f{i + 1}")
        self.act_plots.append(p_force)

        for i, color in enumerate(C4):
            self._add_angle_plot(
                1, i,
                f"alpha{i + 1}: cmd / mea",
                f"alpha{i}",
                f"dxl_alpha{i}",
                color,
                -35.0,
                35.0,
            )

        for p in self.act_plots[1:]:
            p.setXLink(self.act_plots[0])

        for p in self.act_plots:
            p.hideAxis("bottom")

        for p in self.act_plots[-4:]:
            p.showAxis("bottom")
            p.setLabel("bottom", "time [s]")

    def _build_real_tab(self):
        p_xy = _mk_plot_widget("Opti / FMU XY", "y [m]", "x [m]")
        p_xy.setAspectLocked(True, ratio=1)
        self.cv["opti_xy"] = p_xy.plot(pen=_pen(C_G), name="opti")
        self.cv["fmu_xy"] = p_xy.plot(pen=_pen(C_B), name="fmu")
        self.real_layout.addWidget(p_xy, 0, 0, 1, 4)
        self.real_plots.append(p_xy)

        pos_lbl = ["x", "y", "z"]
        for i, label in enumerate(pos_lbl):
            p = _mk_plot_widget(f"{label}: opti / fmu", f"{label} [m]")
            self.cv[f"opti_pos{i}"] = p.plot(pen=_pen(C_G), name="opti")
            self.cv[f"fmu_pos{i}"] = p.plot(pen=_pen(C_B), name="fmu")
            self.real_layout.addWidget(p, 1, i)
            self.real_plots.append(p)
            self.real_time_plots.append(p)

        p_delay = _mk_plot_widget("Opti to FMU delay", "delay [ms]")
        self.cv["delay_arrival"] = p_delay.plot(pen=_pen(C_O), name="arrival")
        self.cv["delay_value"] = p_delay.plot(pen=_pen(C_P), name="value-match")
        self.real_layout.addWidget(p_delay, 1, 3)
        self.real_plots.append(p_delay)
        self.real_time_plots.append(p_delay)

        rpy_lbl = ["roll", "pitch", "yaw"]
        for i, label in enumerate(rpy_lbl):
            p = _mk_plot_widget(f"{label}: opti / fmu", f"{label} [deg]", "time [s]")
            self.cv[f"opti_rpy{i}"] = p.plot(pen=_pen(C_G), name="opti")
            self.cv[f"fmu_rpy{i}"] = p.plot(pen=_pen(C_B), name="fmu")
            self.real_layout.addWidget(p, 2, i)
            self.real_plots.append(p)
            self.real_time_plots.append(p)
            self.real_bottom_time_plots.append(p)

        p_error = _mk_plot_widget("Position error", "error [m]", "time [s]")
        self.cv["err_norm"] = p_error.plot(pen=_pen(C_R), name="live norm")
        self.cv["match_error"] = p_error.plot(pen=_pen(C_C), name="match norm")
        self.real_layout.addWidget(p_error, 2, 3)
        self.real_plots.append(p_error)
        self.real_time_plots.append(p_error)
        self.real_bottom_time_plots.append(p_error)

        for p in self.real_time_plots[1:]:
            p.setXLink(self.real_time_plots[0])

        for p in self.real_time_plots:
            p.hideAxis("bottom")

        for p in self.real_bottom_time_plots:
            p.showAxis("bottom")
            p.setLabel("bottom", "time [s]")

    def _build_real_3d_tab(self):
        tab = QtWidgets.QWidget()
        layout = QtWidgets.QHBoxLayout(tab)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(8)

        path_panel, self.real_3d_path_canvas, self.real_3d_path_ax, self.real_3d_path_toolbar = self._make_3d_panel(
            "3D position: Opti / FMU", self._home_real_3d_path)
        att_panel, self.real_3d_att_canvas, self.real_3d_att_ax, self.real_3d_att_toolbar = self._make_3d_panel(
            "Attitude frames: Opti / FMU", self._home_real_3d_att)
        layout.addWidget(path_panel, 1)
        layout.addWidget(att_panel, 1)

        for canvas in (self.real_3d_path_canvas, self.real_3d_att_canvas):
            canvas.mpl_connect("button_press_event", self._on_real_3d_mouse_press)
            canvas.mpl_connect("button_release_event", self._on_real_3d_mouse_release)

        self.real_3d_enabled = True
        self._redraw_real_3d(np.empty((0, 4)), np.empty((0, 4)))
        return tab

    @staticmethod
    def _make_3d_canvas(title):
        fig = Figure(figsize=(8.0, 7.0), dpi=100)
        fig.patch.set_facecolor("white")
        canvas = FigureCanvas(fig)
        ax = fig.add_subplot(111, projection="3d")
        ax.set_title(title, fontsize=12, fontweight="bold")
        return canvas, ax

    @staticmethod
    def _make_3d_panel(title, home_callback):
        panel = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(2)
        canvas, ax = ViewerWindow._make_3d_canvas(title)
        toolbar = Home3DToolbar(canvas, panel, home_callback)
        layout.addWidget(toolbar)
        layout.addWidget(canvas, 1)
        return panel, canvas, ax, toolbar

    def _on_real_3d_mouse_press(self, _event):
        self.real_3d_interacting = True

    def _on_real_3d_mouse_release(self, _event):
        self.real_3d_interacting = False
        self.real_3d_last_draw_t = 0.0

    def _home_real_3d_path(self):
        if self.real_3d_path_ax is None:
            return
        self._restore_3d_axis_view(self.real_3d_path_ax, self._default_position_3d_view())
        self.real_3d_path_canvas.draw_idle()

    def _home_real_3d_att(self):
        if self.real_3d_att_ax is None:
            return
        self._restore_3d_axis_view(self.real_3d_att_ax, self._default_attitude_3d_view())
        self.real_3d_att_canvas.draw_idle()

    def _update(self):
        nd = self.node

        with nd.lock:
            data = {
                "pos": nd.buf_pos.get(),
                "pos_sp": nd.buf_pos_sp.get(),
                "rpy": nd.buf_rpy.get(),
                "att_sp": nd.buf_att_sp.get(),
                "force": nd.buf_force.get(),
                "torque": nd.buf_torque.get(),
                "motor": nd.buf_motor.get(),
                "beta": nd.buf_beta.get(),
                "alpha": nd.buf_alpha.get(),
                "dxl_beta": nd.buf_dynamixel_beta.get(),
                "dxl_alpha": nd.buf_dynamixel_alpha.get(),
            }
            if nd.real_mode:
                data.update({
                    "opti_pos": nd.buf_opti_pos.get(),
                    "fmu_pose_pos": nd.buf_fmu_pose_pos.get(),
                    "opti_rpy": nd.buf_opti_rpy.get(),
                    "fmu_pose_rpy": nd.buf_fmu_pose_rpy.get(),
                    "delay": nd.buf_delay.get(),
                    "pos_error": nd.buf_pos_error.get(),
                })

        tn = 0.0
        for arr in data.values():
            if arr.shape[0]:
                tn = max(tn, arr[-1, 0])

        if nd.replay_enabled:
            tn = nd.current_replay_time
            tl = max(0.0, tn - self.window_sec)
        else:
            if tn <= 0.0:
                self._update_status_bar()
                return
            tl = tn - self.window_sec

        self._update_status_bar()

        def trim(arr):
            return arr[arr[:, 0] >= tl] if arr.shape[0] else arr

        for key in data:
            data[key] = trim(data[key])

        for i in range(3):
            self._set3(data["pos"], f"pos{i}", i)
            self._set3(data["pos_sp"], f"pos_sp{i}", i)
            self._set3(data["rpy"], f"rpy{i}", i)
            self._set3(data["att_sp"], f"att_sp{i}", i)
            self._set3(data["force"], f"force{i}", i)
            self._set3(data["torque"], f"torque{i}", i)

        for i in range(4):
            self._set4(data["motor"], f"motor_all{i}", i)
            self._set4(data["alpha"], f"alpha{i}", i)
            self._set4(data["dxl_alpha"], f"dxl_alpha{i}", i)

        for i in range(2):
            self._set3(data["beta"], f"beta{i}", i)
            self._set3(data["dxl_beta"], f"dxl_beta{i}", i)

        if nd.real_mode:
            self._update_real_tab(data)

        if self.state_plots:
            self.state_plots[0].setXRange(tl, tn, padding=0)
        if self.act_plots:
            self.act_plots[0].setXRange(tl, tn, padding=0)
        if self.real_time_plots:
            self.real_time_plots[0].setXRange(tl, tn, padding=0)

    def _update_real_tab(self, data):
        opti_pos = data["opti_pos"]
        fmu_pos = data["fmu_pose_pos"]
        delay = data["delay"]
        pos_error = data["pos_error"]

        if opti_pos.shape[0] and "opti_xy" in self.cv:
            self.cv["opti_xy"].setData(opti_pos[:, 1], opti_pos[:, 2])
        if fmu_pos.shape[0] and "fmu_xy" in self.cv:
            self.cv["fmu_xy"].setData(fmu_pos[:, 1], fmu_pos[:, 2])

        self._update_real_3d(opti_pos, fmu_pos)

        for i in range(3):
            self._set3(opti_pos, f"opti_pos{i}", i)
            self._set3(fmu_pos, f"fmu_pos{i}", i)
            self._set3(data["opti_rpy"], f"opti_rpy{i}", i)
            self._set3(data["fmu_pose_rpy"], f"fmu_rpy{i}", i)

        self._set_col(delay, "delay_arrival", 1)
        self._set_col(delay, "delay_value", 2)
        self._set_col(delay, "match_error", 3)
        self._set_col(pos_error, "err_norm", 4)

    def _update_real_3d(self, opti_pos, fmu_pos):
        if not self.real_3d_enabled or self.real_3d_interacting:
            return

        now = time.monotonic()
        if now - self.real_3d_last_draw_t < 0.10:
            return
        self.real_3d_last_draw_t = now
        self._redraw_real_3d(opti_pos, fmu_pos)

    def _redraw_real_3d(self, opti_pos, fmu_pos):
        if self.real_3d_path_ax is None or self.real_3d_att_ax is None:
            return

        with self.node.lock:
            opti_att = None if self.node.last_opti_att is None else self.node.last_opti_att.copy()
            fmu_att = None if self.node.last_fmu_pose_att is None else self.node.last_fmu_pose_att.copy()

        path_ax = self.real_3d_path_ax
        att_ax = self.real_3d_att_ax
        path_view = self._capture_3d_axis_view(path_ax) if self.real_3d_view_initialized else None
        att_view = self._capture_3d_axis_view(att_ax) if self.real_3d_view_initialized else None
        self._setup_position_3d_axis(path_ax, path_view)
        self._setup_attitude_3d_axis(att_ax, att_view)

        has_path = False
        if opti_pos.shape[0]:
            points = self._to_3d_display(opti_pos)
            path_ax.plot(points[:, 0], points[:, 1], points[:, 2], color=C_G, linewidth=4.5, label="Opti")
            path_ax.scatter(points[-1, 0], points[-1, 1], points[-1, 2], color=C_G, s=90, depthshade=False)
            if opti_att is not None:
                self._draw_body_frame(path_ax, points[-1], opti_att, "O", FRAME_AXIS_LEN)
            has_path = True

        if fmu_pos.shape[0]:
            points = self._to_3d_display(fmu_pos)
            path_ax.plot(points[:, 0], points[:, 1], points[:, 2], color=C_B, linewidth=4.5, label="FMU")
            path_ax.scatter(points[-1, 0], points[-1, 1], points[-1, 2], color=C_B, s=90, marker="^", depthshade=False)
            if fmu_att is not None:
                self._draw_body_frame(path_ax, points[-1], fmu_att, "F", FRAME_AXIS_LEN)
            has_path = True

        if has_path:
            path_ax.legend(loc="upper left")

        frame_center = np.zeros(3, dtype=np.float64)
        att_ax.scatter([0.0], [0.0], [0.0], color="black", s=35, depthshade=False)
        att_ax.text(-1.15, -0.78, 0.72, "Opti: outer axes", color=C_G, fontsize=11, fontweight="bold")
        att_ax.text(-1.15, -0.78, 0.55, "FMU: inner axes", color=C_B, fontsize=11, fontweight="bold")
        if opti_att is not None:
            self._draw_body_frame(att_ax, frame_center, opti_att, "O", ATTITUDE_FRAME_AXIS_LEN, 3.4)
        if fmu_att is not None:
            self._draw_body_frame(att_ax, frame_center, fmu_att, "F", ATTITUDE_FRAME_AXIS_LEN * 0.70, 2.5)

        self.real_3d_view_initialized = True
        self.real_3d_path_canvas.draw_idle()
        self.real_3d_att_canvas.draw_idle()

    @staticmethod
    def _to_3d_display(arr):
        return np.column_stack((arr[:, 1], arr[:, 2], -arr[:, 3]))

    @staticmethod
    def _ned_vec_to_display(v):
        return DISPLAY_Z_UP @ np.asarray(v, dtype=np.float64)

    @staticmethod
    def _default_position_3d_view():
        return {
            "xlim": (-REAL_3D_XY_LIMIT_M, REAL_3D_XY_LIMIT_M),
            "ylim": (-REAL_3D_XY_LIMIT_M, REAL_3D_XY_LIMIT_M),
            "zlim": (REAL_3D_Z_MIN_M, REAL_3D_Z_MAX_M),
            "elev": 24.0,
            "azim": -55.0,
            "roll": 0.0,
        }

    @staticmethod
    def _default_attitude_3d_view():
        return {
            "xlim": (-1.25, 1.25),
            "ylim": (-0.9, 0.9),
            "zlim": (-0.9, 0.9),
            "elev": 22.0,
            "azim": -55.0,
            "roll": 0.0,
        }

    @staticmethod
    def _capture_3d_axis_view(ax):
        return {
            "xlim": tuple(ax.get_xlim()),
            "ylim": tuple(ax.get_ylim()),
            "zlim": tuple(ax.get_zlim()),
            "elev": float(ax.elev),
            "azim": float(ax.azim),
            "roll": float(getattr(ax, "roll", 0.0)),
        }

    @staticmethod
    def _restore_3d_axis_view(ax, view):
        ax.set_xlim(*view["xlim"])
        ax.set_ylim(*view["ylim"])
        ax.set_zlim(*view["zlim"])
        try:
            ax.view_init(elev=view["elev"], azim=view["azim"], roll=view["roll"])
        except TypeError:
            ax.view_init(elev=view["elev"], azim=view["azim"])

    @staticmethod
    def _setup_position_3d_axis(ax, view=None):
        ax.clear()
        ax.set_title("3D position: Opti / FMU", fontsize=12, fontweight="bold")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_zlabel("height [-NED z] [m]")
        ax.grid(True)
        try:
            ax.set_box_aspect((4.0, 4.0, 3.0))
        except Exception:
            pass
        ViewerWindow._restore_3d_axis_view(ax, view if view is not None else ViewerWindow._default_position_3d_view())

    @staticmethod
    def _setup_attitude_3d_axis(ax, view=None):
        ax.clear()
        ax.set_title("Attitude frames only", fontsize=12, fontweight="bold")
        ax.set_xlabel("display x")
        ax.set_ylabel("display y")
        ax.set_zlabel("display z")
        ax.grid(True)
        try:
            ax.set_box_aspect((2.5, 1.8, 1.8))
        except Exception:
            pass
        ViewerWindow._restore_3d_axis_view(ax, view if view is not None else ViewerWindow._default_attitude_3d_view())

    def _draw_body_frame(self, ax, pos_display, q_wxyz, prefix, axis_len, linewidth=3.2):
        R = quat_to_rot(q_wxyz)
        colors = ["#d32f2f", "#2e7d32", "#1565c0"]
        names = ["x", "y", "z"]
        for i, color in enumerate(colors):
            v = self._ned_vec_to_display(R[:, i]) * axis_len
            ax.quiver(
                pos_display[0], pos_display[1], pos_display[2],
                v[0], v[1], v[2],
                color=color, linewidth=linewidth, arrow_length_ratio=0.25, normalize=False,
            )
            ax.text(
                pos_display[0] + v[0] * 1.15,
                pos_display[1] + v[1] * 1.15,
                pos_display[2] + v[2] * 1.15,
                f"{prefix}{names[i]}",
                color=color, fontsize=10, fontweight="bold",
            )

    def _set3(self, arr, name, idx):
        if arr.shape[0] and name in self.cv:
            self.cv[name].setData(arr[:, 0], arr[:, 1 + idx])

    def _set4(self, arr, name, idx):
        if arr.shape[0] and name in self.cv:
            self.cv[name].setData(arr[:, 0], arr[:, 1 + idx])

    def _set_col(self, arr, name, col):
        if arr.shape[0] and name in self.cv:
            self.cv[name].setData(arr[:, 0], arr[:, col])


def main():
    rclpy.init()
    node = Px4ViewerNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    app = QtWidgets.QApplication(sys.argv)
    win = ViewerWindow(node)
    win.show()
    code = app.exec_()
    node.destroy_node()
    rclpy.shutdown()
    node.close_logger()
    sys.exit(code)


if __name__ == "__main__":
    main()
