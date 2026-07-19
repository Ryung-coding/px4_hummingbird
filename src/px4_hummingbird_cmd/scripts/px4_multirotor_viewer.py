#!/usr/bin/env python3
import math
import sys
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from px4_msgs.msg import ActuatorMotors, ActuatorServos, HummingbirdStatus
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


def finite4(values):
    return np.array([0.0 if not math.isfinite(float(v)) else float(v) for v in values[:4]], dtype=np.float64)


class Px4ViewerNode(Node):
    def __init__(self):
        super().__init__("px4_multirotor_viewer")
        self.lock = threading.Lock()
        self.t0 = self.get_clock().now().nanoseconds * 1e-9

        self.theta_limit_rad = self.declare_parameter("theta_limit_rad", math.pi).value
        self.phi_limit_rad = self.declare_parameter("phi_limit_rad", math.pi / 6.0).value
        self.max_rotor_thrust = self.declare_parameter("max_rotor_thrust", 24.0).value

        self.last_pos_sp = np.zeros(3, dtype=np.float64)
        self.last_att_sp_deg = np.zeros(3, dtype=np.float64)
        self.last_thrust_sp = np.zeros(3, dtype=np.float64)
        self.last_torque_sp = np.zeros(3, dtype=np.float64)
        self.last_status = "HB cmd: ?  ctrl: ?"

        self.max_pos_err_abs = np.zeros(3, dtype=np.float64)
        self.max_att_err_abs = np.zeros(3, dtype=np.float64)
        self.max_pos_err_norm = 0.0
        self.max_att_err_norm = 0.0
        self.max_pos_err_time = 0.0
        self.max_att_err_time = 0.0

        self.buf_pos = Ring(MAX_SAMPLES, 4)
        self.buf_pos_sp = Ring(MAX_SAMPLES, 4)
        self.buf_pos_err = Ring(MAX_SAMPLES, 4)
        self.buf_rpy = Ring(MAX_SAMPLES, 4)
        self.buf_att_sp = Ring(MAX_SAMPLES, 4)
        self.buf_att_err = Ring(MAX_SAMPLES, 4)
        self.buf_force = Ring(MAX_SAMPLES, 4)
        self.buf_torque = Ring(MAX_SAMPLES, 4)
        self.buf_motor = Ring(MAX_SAMPLES, 5)
        self.buf_theta = Ring(MAX_SAMPLES, 5)
        self.buf_phi = Ring(MAX_SAMPLES, 5)
        self.buf_thrust_body = Ring(MAX_SAMPLES, 4)

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
        self.create_subscription(ActuatorMotors, "/fmu/out/actuator_motors", self._cb_motors, qos)
        self.create_subscription(ActuatorServos, "/fmu/out/actuator_servos", self._cb_servos, qos)
        self.create_subscription(HummingbirdStatus, "/fmu/out/hummingbird_status", self._cb_status, qos)

    def _t(self):
        return self.get_clock().now().nanoseconds * 1e-9 - self.t0

    def _cb_pos(self, msg):
        t = self._t()
        pos = np.array([msg.x, msg.y, msg.z], dtype=np.float64)
        err = self.last_pos_sp - pos
        err_abs = np.abs(err)
        err_norm = np.linalg.norm(err)
        with self.lock:
            self.max_pos_err_abs = np.maximum(self.max_pos_err_abs, err_abs)
            if err_norm > self.max_pos_err_norm:
                self.max_pos_err_norm = err_norm
                self.max_pos_err_time = t
            self.buf_pos.push([t, pos[0], pos[1], pos[2]])
            self.buf_pos_err.push([t, err[0], err[1], err[2]])

    def _cb_pos_sp(self, msg):
        t = self._t()
        self.last_pos_sp = np.array([msg.x, msg.y, msg.z], dtype=np.float64)
        with self.lock:
            self.buf_pos_sp.push([t, self.last_pos_sp[0], self.last_pos_sp[1], self.last_pos_sp[2]])

    def _cb_att(self, msg):
        t = self._t()
        rpy = quat_to_rpy_deg(msg.q)
        err = np.array([
            wrap_deg(self.last_att_sp_deg[0] - rpy[0]),
            wrap_deg(self.last_att_sp_deg[1] - rpy[1]),
            wrap_deg(self.last_att_sp_deg[2] - rpy[2]),
        ], dtype=np.float64)
        err_abs = np.abs(err)
        err_norm = np.linalg.norm(err)
        with self.lock:
            self.max_att_err_abs = np.maximum(self.max_att_err_abs, err_abs)
            if err_norm > self.max_att_err_norm:
                self.max_att_err_norm = err_norm
                self.max_att_err_time = t
            self.buf_rpy.push([t, rpy[0], rpy[1], rpy[2]])
            self.buf_att_err.push([t, err[0], err[1], err[2]])

    def _cb_att_sp(self, msg):
        t = self._t()
        self.last_att_sp_deg = quat_to_rpy_deg(msg.q_d)
        thrust_body = np.array(msg.thrust_body, dtype=np.float64)
        with self.lock:
            self.buf_att_sp.push([t, self.last_att_sp_deg[0], self.last_att_sp_deg[1], self.last_att_sp_deg[2]])
            self.buf_thrust_body.push([t, thrust_body[0], thrust_body[1], thrust_body[2]])

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
            self.buf_motor.push([t, force[0], force[1], force[2], force[3]])

    def _cb_servos(self, msg):
        t = self._t()
        theta = finite4(msg.control[0:4]) * self.theta_limit_rad * RAD2DEG
        phi = finite4(msg.control[4:8]) * self.phi_limit_rad * RAD2DEG
        with self.lock:
            self.buf_theta.push([t, theta[0], theta[1], theta[2], theta[3]])
            self.buf_phi.push([t, phi[0], phi[1], phi[2], phi[3]])

    def _cb_status(self, msg):
        with self.lock:
            self.last_status = f"HB cmd: {msg.hb_cmd_source}  ctrl: {msg.hb_ctrl_mode}"


def _pen(color, width=2):
    return pg.mkPen(color=color, width=width, style=QtCore.Qt.SolidLine)


def _cmd_pen(width=2):
    return pg.mkPen(color="k", width=width, style=QtCore.Qt.DashLine)


def _mkplot(glw, row, col, title, ylabel):
    plot = glw.addPlot(row=row, col=col, title=title)
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

        tabs = QtWidgets.QTabWidget()
        self.setCentralWidget(tabs)
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

    def _build_state_tab(self):
        pos_lbl = ["x", "y", "z"]
        rpy_lbl = ["roll", "pitch", "yaw"]
        frc_lbl = ["Tx", "Ty", "Tz"]
        trq_lbl = ["Mx", "My", "Mz"]

        for i in range(3):
            p = _mkplot(self.state_glw, 0, i, f"{pos_lbl[i]} / sp", f"{pos_lbl[i]} [m]")
            self.cv[f"pos{i}"] = p.plot(pen=_pen(C3[i]), name=pos_lbl[i])
            self.cv[f"pos_sp{i}"] = _front(p.plot(pen=_cmd_pen(), name="sp"))
            self.state_plots.append(p)
        for i in range(3):
            p = _mkplot(self.state_glw, 1, i, f"{pos_lbl[i]} error", "err [m]")
            self.cv[f"pos_err{i}"] = p.plot(pen=_pen(C3[i]), name="err")
            self.state_plots.append(p)
        for i in range(3):
            p = _mkplot(self.state_glw, 2, i, frc_lbl[i], "norm")
            self.cv[f"force{i}"] = p.plot(pen=_pen(C3[i]), name=frc_lbl[i])
            self.state_plots.append(p)
        for i in range(3):
            p = _mkplot(self.state_glw, 3, i, f"{rpy_lbl[i]} / sp", f"{rpy_lbl[i]} [deg]")
            self.cv[f"rpy{i}"] = p.plot(pen=_pen(C3[i]), name=rpy_lbl[i])
            self.cv[f"att_sp{i}"] = _front(p.plot(pen=_cmd_pen(), name="sp"))
            self.state_plots.append(p)
        for i in range(3):
            p = _mkplot(self.state_glw, 4, i, f"{rpy_lbl[i]} error", "err [deg]")
            self.cv[f"att_err{i}"] = p.plot(pen=_pen(C3[i]), name="err")
            self.state_plots.append(p)
        for i in range(3):
            p = _mkplot(self.state_glw, 5, i, trq_lbl[i], "norm")
            self.cv[f"torque{i}"] = p.plot(pen=_pen(C3[i]), name=trq_lbl[i])
            self.state_plots.append(p)

        for p in self.state_plots[1:]:
            p.setXLink(self.state_plots[0])
        for p in self.state_plots:
            p.hideAxis("bottom")
        for p in self.state_plots[-3:]:
            p.showAxis("bottom")
            p.setLabel("bottom", "time [s]")

    def _build_act_tab(self):
        for i, color in enumerate(C4):
            p = _mkplot(self.act_glw, 0, i, f"f{i + 1}", "force [N]")
            self.cv[f"motor{i}"] = p.plot(pen=_pen(color), name=f"f{i + 1}")
            self.act_plots.append(p)
        for i, color in enumerate(C4):
            p = _mkplot(self.act_glw, 1, i, f"theta{i + 1}", "theta [deg]")
            self.cv[f"theta{i}"] = p.plot(pen=_pen(color), name=f"theta{i + 1}")
            self.act_plots.append(p)
        for i, color in enumerate(C4):
            p = _mkplot(self.act_glw, 2, i, f"phi{i + 1}", "phi [deg]")
            self.cv[f"phi{i}"] = p.plot(pen=_pen(color), name=f"phi{i + 1}")
            self.act_plots.append(p)

        p_all = _mkplot(self.act_glw, 3, 0, "f1-f4", "force [N]")
        for i, color in enumerate(C4):
            self.cv[f"motor_all{i}"] = p_all.plot(pen=_pen(color), name=f"f{i + 1}")
        self.act_plots.append(p_all)

        p_theta = _mkplot(self.act_glw, 3, 1, "theta1-theta4", "theta [deg]")
        for i, color in enumerate(C4):
            self.cv[f"theta_all{i}"] = p_theta.plot(pen=_pen(color), name=f"theta{i + 1}")
        self.act_plots.append(p_theta)

        p_phi = _mkplot(self.act_glw, 3, 2, "phi1-phi4", "phi [deg]")
        for i, color in enumerate(C4):
            self.cv[f"phi_all{i}"] = p_phi.plot(pen=_pen(color), name=f"phi{i + 1}")
        self.act_plots.append(p_phi)

        p_tb = _mkplot(self.act_glw, 3, 3, "thrust_body", "norm")
        for i, color in enumerate(C3):
            self.cv[f"tb{i}"] = p_tb.plot(pen=_pen(color), name=["x", "y", "z"][i])
        self.act_plots.append(p_tb)

        self.max_label = pg.LabelItem(justify="left")
        self.act_glw.addItem(self.max_label, row=4, col=0, colspan=4)

        for p in self.act_plots[1:]:
            p.setXLink(self.act_plots[0])
        for p in self.act_plots:
            p.hideAxis("bottom")
        for p in self.act_plots[-4:]:
            p.showAxis("bottom")
            p.setLabel("bottom", "time [s]")

    def _update_label(self):
        nd = self.node
        with nd.lock:
            pos = nd.max_pos_err_abs.copy()
            att = nd.max_att_err_abs.copy()
            pos_n = nd.max_pos_err_norm
            att_n = nd.max_att_err_norm
            pos_t = nd.max_pos_err_time
            att_t = nd.max_att_err_time
            status = nd.last_status
        self.max_label.setText(
            "<div style='font-size:13pt; color:#111;'>"
            f"<b>{status}</b><br>"
            f"pos err max [m]: x {pos[0]:.3f}, y {pos[1]:.3f}, z {pos[2]:.3f}, norm {pos_n:.3f} @ {pos_t:.2f}s<br>"
            f"att err max [deg]: roll {att[0]:.2f}, pitch {att[1]:.2f}, yaw {att[2]:.2f}, norm {att_n:.2f} @ {att_t:.2f}s"
            "</div>"
        )

    def _update(self):
        nd = self.node
        with nd.lock:
            data = {
                "pos": nd.buf_pos.get(),
                "pos_sp": nd.buf_pos_sp.get(),
                "pos_err": nd.buf_pos_err.get(),
                "rpy": nd.buf_rpy.get(),
                "att_sp": nd.buf_att_sp.get(),
                "att_err": nd.buf_att_err.get(),
                "force": nd.buf_force.get(),
                "torque": nd.buf_torque.get(),
                "motor": nd.buf_motor.get(),
                "theta": nd.buf_theta.get(),
                "phi": nd.buf_phi.get(),
                "tb": nd.buf_thrust_body.get(),
            }

        tn = 0.0
        for arr in data.values():
            if arr.shape[0]:
                tn = max(tn, arr[-1, 0])
        if tn <= 0.0:
            return
        tl = tn - WINDOW_SEC

        def trim(arr):
            return arr[arr[:, 0] >= tl] if arr.shape[0] else arr

        for key in data:
            data[key] = trim(data[key])

        for i in range(3):
            self._set3(data["pos"], f"pos{i}", i)
            self._set3(data["pos_sp"], f"pos_sp{i}", i)
            self._set3(data["pos_err"], f"pos_err{i}", i)
            self._set3(data["rpy"], f"rpy{i}", i)
            self._set3(data["att_sp"], f"att_sp{i}", i)
            self._set3(data["att_err"], f"att_err{i}", i)
            self._set3(data["force"], f"force{i}", i)
            self._set3(data["torque"], f"torque{i}", i)
            self._set3(data["tb"], f"tb{i}", i)
        for i in range(4):
            self._set4(data["motor"], f"motor{i}", i)
            self._set4(data["motor"], f"motor_all{i}", i)
            self._set4(data["theta"], f"theta{i}", i)
            self._set4(data["theta"], f"theta_all{i}", i)
            self._set4(data["phi"], f"phi{i}", i)
            self._set4(data["phi"], f"phi_all{i}", i)

        self._update_label()
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
