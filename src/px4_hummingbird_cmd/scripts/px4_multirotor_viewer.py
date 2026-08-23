#!/usr/bin/env python3
import math
import sys
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from px4_msgs.msg import ActuatorMotors, ActuatorServos, HummingbirdStatus, ManualControlSetpoint
from px4_msgs.msg import VehicleAttitude, VehicleAttitudeSetpoint
from px4_msgs.msg import VehicleLocalPosition, VehicleLocalPositionSetpoint
from px4_msgs.msg import VehicleThrustSetpoint, VehicleTorqueSetpoint

from PyQt5 import QtCore, QtWidgets
import pyqtgraph as pg

WINDOW_SEC = 10.0
MAX_SAMPLES = 5000
UPDATE_MS = 50
RAD2DEG = 180.0 / math.pi

C_R = "#e6194b"
C_G = "#3cb44b"
C_B = "#4363d8"
C_O = "#f58231"
C3 = [C_R, C_G, C_B]
C4 = [C_R, C_G, C_B, C_O]

pg.setConfigOption("background", "w")
pg.setConfigOption("foreground", "k")


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


def wrap_deg(x):
    return (x + 180.0) % 360.0 - 180.0


def quat_to_rpy_deg(q):
    w, x, y, z = q
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


def finite_n(values, width):
    out = np.zeros(width, dtype=np.float64)
    for i, value in enumerate(values[:width]):
        out[i] = 0.0 if not math.isfinite(float(value)) else float(value)
    return out


def finite4(values):
    return finite_n(values, 4)


class Px4ViewerNode(Node):
    def __init__(self):
        super().__init__("px4_multirotor_viewer")
        self.lock = threading.Lock()
        self.t0 = self.get_clock().now().nanoseconds * 1e-9

        self.beta_limit_rad = self.declare_parameter("beta_limit_rad", math.pi).value
        self.alpha_limit_rad = self.declare_parameter("alpha_limit_rad", math.pi / 6.0).value
        self.max_rotor_thrust = self.declare_parameter("max_rotor_thrust", 24.0).value

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

        # Hummingbird status indicators used by the top status bar.
        self.last_hb_cmd_source = -1
        self.last_hb_ctrl_mode = -1
        self.last_dds_enabled = False
        self.last_hb_enabled = False

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

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1", self._cb_pos, qos)
        self.create_subscription(VehicleLocalPositionSetpoint, "/fmu/out/vehicle_local_position_setpoint", self._cb_pos_sp, qos)
        self.create_subscription(VehicleAttitude, "/fmu/out/vehicle_attitude", self._cb_att, qos)
        self.create_subscription(VehicleAttitudeSetpoint, "/fmu/out/vehicle_attitude_setpoint_v1", self._cb_att_sp, qos)
        self.create_subscription(VehicleThrustSetpoint, "/fmu/out/vehicle_thrust_setpoint", self._cb_thrust, qos)
        self.create_subscription(VehicleTorqueSetpoint, "/fmu/out/vehicle_torque_setpoint", self._cb_torque, qos)
        self.create_subscription(ManualControlSetpoint, "/fmu/out/manual_control_setpoint", self._cb_manual, qos)
        self.create_subscription(ActuatorMotors, "/fmu/out/actuator_motors", self._cb_motors, qos)
        self.create_subscription(ActuatorServos, "/fmu/out/actuator_servos", self._cb_servos, qos)
        self.create_subscription(ActuatorServos, "/dynamixel_tilt_mea", self._cb_dynamixel_servos, qos)
        self.create_subscription(HummingbirdStatus, "/fmu/out/hummingbird_status", self._cb_status, qos)

    def _t(self):
        return self.get_clock().now().nanoseconds * 1e-9 - self.t0

    def _cb_pos(self, msg):
        t = self._t()
        pos = np.array([msg.x, msg.y, msg.z], dtype=np.float64)
        with self.lock:
            self.buf_pos.push([t, pos[0], pos[1], pos[2]])

    def _cb_pos_sp(self, msg):
        t = self._t()
        self.last_pos_sp = np.array([msg.x, msg.y, msg.z], dtype=np.float64)
        with self.lock:
            self.buf_pos_sp.push([t, self.last_pos_sp[0], self.last_pos_sp[1], self.last_pos_sp[2]])

    def _cb_att(self, msg):
        t = self._t()
        rpy = quat_to_rpy_deg(msg.q)
        with self.lock:
            self.buf_rpy.push([t, rpy[0], rpy[1], rpy[2]])

    def _cb_att_sp(self, msg):
        t = self._t()
        self.last_att_sp_deg = quat_to_rpy_deg(msg.q_d)
        with self.lock:
            self.buf_att_sp.push([t, self.last_att_sp_deg[0], self.last_att_sp_deg[1], self.last_att_sp_deg[2]])

    def _cb_thrust(self, msg):
        t = self._t()
        self.last_thrust_sp = np.array(msg.xyz, dtype=np.float64)
        with self.lock:
            self.buf_force.push([t, self.last_thrust_sp[0], self.last_thrust_sp[1], self.last_thrust_sp[2]])

    def _cb_torque(self, msg):
        t = self._t()
        self.last_torque_sp = np.array(msg.xyz, dtype=np.float64)
        with self.lock:
            self.buf_torque.push([t, self.last_torque_sp[0], self.last_torque_sp[1], self.last_torque_sp[2]])

    def _cb_motors(self, msg):
        t = self._t()
        controls = finite4(msg.control)
        force = np.square(np.clip(controls, 0.0, 1.0)) * self.max_rotor_thrust
        with self.lock:
            self.last_motor_controls = controls
            self.last_motor_forces = force
            self.buf_motor.push([t, force[0], force[1], force[2], force[3]])

    def _cb_manual(self, msg):
        manual = np.array([msg.roll, msg.pitch, msg.yaw, msg.throttle], dtype=np.float64)
        with self.lock:
            self.last_manual = manual
            self.last_manual_valid = bool(msg.valid)
            self.last_manual_source = int(msg.data_source)
            self.last_sticks_moving = bool(msg.sticks_moving)

    def _cb_servos(self, msg):
        t = self._t()
        controls = finite_n(msg.control, 6)
        beta = finite_n(msg.control[0:2], 2) * self.beta_limit_rad * RAD2DEG
        alpha = finite_n(msg.control[2:6], 4) * self.alpha_limit_rad * RAD2DEG
        with self.lock:
            self.last_servo_controls = controls
            self.buf_beta.push([t, beta[0], beta[1]])
            self.buf_alpha.push([t, alpha[0], alpha[1], alpha[2], alpha[3]])

    def _cb_dynamixel_servos(self, msg):
        t = self._t()
        controls = finite_n(msg.control, 6)
        beta = finite_n(msg.control[0:2], 2) * self.beta_limit_rad * RAD2DEG
        alpha = finite_n(msg.control[2:6], 4) * self.alpha_limit_rad * RAD2DEG
        with self.lock:
            self.last_dynamixel_controls = controls
            self.buf_dynamixel_beta.push([t, beta[0], beta[1]])
            self.buf_dynamixel_alpha.push([t, alpha[0], alpha[1], alpha[2], alpha[3]])

    def _cb_status(self, msg):
        hb_enabled = getattr(
            msg,
            "hummingbird_control_enabled",
            getattr(msg, "fully_actuated_control_enabled", False),
        )
        with self.lock:
            self.last_hb_cmd_source = int(msg.hb_cmd_source)
            self.last_hb_ctrl_mode = int(msg.hb_ctrl_mode)
            self.last_dds_enabled = bool(msg.dds_command_enabled)
            self.last_hb_enabled = bool(hb_enabled)


def _pen(color, width=3):
    return pg.mkPen(color=color, width=width, style=QtCore.Qt.SolidLine)


def _cmd_pen(width=3):
    return pg.mkPen(color="k", width=width, style=QtCore.Qt.DashLine)


def _mea_pen(color, width=3):
    return pg.mkPen(color=color, width=width, style=QtCore.Qt.DashLine)


def _mkplot(glw, row, col, title, ylabel, rowspan=1, colspan=1):
    plot = glw.addPlot(row=row, col=col, rowspan=rowspan, colspan=colspan, title=title)
    plot.showGrid(x=True, y=True, alpha=0.3)
    plot.setLabel("left", ylabel)
    plot.getAxis("left").enableAutoSIPrefix(False)
    plot.getAxis("bottom").enableAutoSIPrefix(False)
    for axis in ("bottom", "left"):
        plot.getAxis(axis).setPen(pg.mkPen("k"))
        plot.getAxis(axis).setTextPen(pg.mkPen("k"))
    plot.addLegend(offset=(-10, 5))
    return plot


def _front(curve):
    curve.setZValue(5)
    return curve



class ViewerWindow(QtWidgets.QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle("PX4 HummingBird DDS Viewer")
        self.resize(1800, 1050)

        # Top-level layout: status indicators + tabs.
        central = QtWidgets.QWidget()
        root = QtWidgets.QVBoxLayout(central)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(6)
        self.setCentralWidget(central)

        self.status_widget = QtWidgets.QWidget()
        status_layout = QtWidgets.QHBoxLayout(self.status_widget)
        status_layout.setContentsMargins(6, 2, 6, 2)
        status_layout.setSpacing(10)

        title = QtWidgets.QLabel("HummingBird Status")
        title.setStyleSheet("font-size: 15pt; font-weight: 700; color: #111;")
        status_layout.addWidget(title)

        self.badge_hb = QtWidgets.QLabel()
        self.badge_dds = QtWidgets.QLabel()
        self.badge_manual = QtWidgets.QLabel()
        self.badge_cmd = QtWidgets.QLabel()
        self.badge_ctrl = QtWidgets.QLabel()

        for badge in (self.badge_hb, self.badge_dds, self.badge_manual, self.badge_cmd, self.badge_ctrl):
            badge.setAlignment(QtCore.Qt.AlignCenter)
            badge.setMinimumWidth(125)
            badge.setMinimumHeight(32)
            status_layout.addWidget(badge)

        status_layout.addStretch(1)
        root.addWidget(self.status_widget)

        tabs = QtWidgets.QTabWidget()
        root.addWidget(tabs, 1)

        self.state_glw = pg.GraphicsLayoutWidget()
        self.act_glw = pg.GraphicsLayoutWidget()
        tabs.addTab(self.state_glw, "State")
        tabs.addTab(self.act_glw, "Actuator")

        self.cv = {}
        self.state_plots = []
        self.act_plots = []
        self._build_state_tab()
        self._build_act_tab()

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self._update)
        self.timer.start(UPDATE_MS)

    @staticmethod
    def _set_bool_badge(label, name, enabled):
        bg = "#2e7d32" if enabled else "#c62828"
        state = "ON" if enabled else "OFF"
        label.setText(f"{name}: {state}")
        label.setStyleSheet(
            f"background:{bg}; color:white; border-radius:6px; "
            "padding:5px 10px; font-size:12pt; font-weight:700;"
        )

    @staticmethod
    def _set_value_badge(label, name, value):
        label.setText(f"{name}: {value}")
        label.setStyleSheet(
            "background:#424242; color:white; border-radius:6px; "
            "padding:5px 10px; font-size:12pt; font-weight:700;"
        )

    def _update_status_bar(self):
        nd = self.node
        with nd.lock:
            hb = nd.last_hb_enabled
            dds = nd.last_dds_enabled
            manual = nd.last_manual_valid
            cmd = nd.last_hb_cmd_source
            ctrl = nd.last_hb_ctrl_mode

        self._set_bool_badge(self.badge_hb, "HB", hb)
        self._set_bool_badge(self.badge_dds, "DDS", dds)
        self._set_bool_badge(self.badge_manual, "MANUAL", manual)
        self._set_value_badge(self.badge_cmd, "CMD SRC", cmd)
        self._set_value_badge(self.badge_ctrl, "CTRL MODE", ctrl)

    def _build_state_tab(self):
        pos_lbl = ["x", "y", "z"]
        rpy_lbl = ["roll", "pitch", "yaw"]
        trq_lbl = ["Mx", "My", "Mz"]
        frc_lbl = ["Fx", "Fy", "Fz"]

        # Row 0: position + setpoint
        for i in range(3):
            p = _mkplot(self.state_glw, 0, i, f"{pos_lbl[i]} / sp", f"{pos_lbl[i]} [m]")
            self.cv[f"pos{i}"] = p.plot(pen=_pen(C3[i]), name=pos_lbl[i])
            self.cv[f"pos_sp{i}"] = _front(p.plot(pen=_cmd_pen(), name="sp"))
            self.state_plots.append(p)

        # Row 1: attitude + setpoint
        for i in range(3):
            p = _mkplot(self.state_glw, 1, i, f"{rpy_lbl[i]} / sp", f"{rpy_lbl[i]} [deg]")
            self.cv[f"rpy{i}"] = p.plot(pen=_pen(C3[i]), name=rpy_lbl[i])
            self.cv[f"att_sp{i}"] = _front(p.plot(pen=_cmd_pen(), name="sp"))
            self.state_plots.append(p)

        # Row 2: body moment
        for i in range(3):
            p = _mkplot(self.state_glw, 2, i, trq_lbl[i], "norm")
            self.cv[f"torque{i}"] = p.plot(pen=_pen(C3[i]), name=trq_lbl[i])
            self.state_plots.append(p)

        # Row 3: body translational thrust
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
        # 7 plots total:
        # beta1, beta2, alpha1~alpha4, and one combined f1~f4 plot.
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

        # Bottom row (alpha1~alpha4) gets the time axis.
        for p in self.act_plots[-4:]:
            p.showAxis("bottom")
            p.setLabel("bottom", "time [s]")

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

        tn = 0.0
        for arr in data.values():
            if arr.shape[0]:
                tn = max(tn, arr[-1, 0])

        self._update_status_bar()

        if tn <= 0.0:
            return

        tl = tn - WINDOW_SEC

        def trim(arr):
            return arr[arr[:, 0] >= tl] if arr.shape[0] else arr

        for key in data:
            data[key] = trim(data[key])

        # State tab
        for i in range(3):
            self._set3(data["pos"], f"pos{i}", i)
            self._set3(data["pos_sp"], f"pos_sp{i}", i)
            self._set3(data["rpy"], f"rpy{i}", i)
            self._set3(data["att_sp"], f"att_sp{i}", i)
            self._set3(data["force"], f"force{i}", i)
            self._set3(data["torque"], f"torque{i}", i)

        # Actuator tab: six servo angles (cmd + measured) and combined rotor thrust.
        for i in range(4):
            self._set4(data["motor"], f"motor_all{i}", i)
            self._set4(data["alpha"], f"alpha{i}", i)
            self._set4(data["dxl_alpha"], f"dxl_alpha{i}", i)

        for i in range(2):
            self._set3(data["beta"], f"beta{i}", i)
            self._set3(data["dxl_beta"], f"dxl_beta{i}", i)

        if self.state_plots:
            self.state_plots[0].setXRange(tl, tn, padding=0)
        if self.act_plots:
            self.act_plots[0].setXRange(tl, tn, padding=0)

    def _set3(self, arr, name, idx):
        if arr.shape[0] and name in self.cv:
            self.cv[name].setData(arr[:, 0], arr[:, 1 + idx])

    def _set4(self, arr, name, idx):
        if arr.shape[0] and name in self.cv:
            self.cv[name].setData(arr[:, 0], arr[:, 1 + idx])


def main():
    rclpy.init()
    node = Px4ViewerNode()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    app = QtWidgets.QApplication(sys.argv)
    win = ViewerWindow(node)
    win.show()
    code = app.exec_()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()