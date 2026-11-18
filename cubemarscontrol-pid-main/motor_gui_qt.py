#!/usr/bin/env python3
"""
motor_gui_qt.py - PyQt6 GUI for the 6-motor CubeMars leg rig.

Thin GUI wrapper around MotorLink from motor_ctl.py: it reuses the exact same
serial protocol, heartbeat, reader thread and high-level commands, so the GUI
and the CLI behave identically. The CLI (motor_ctl.py) is untouched.

Per motor you get:
  - Enable / Passive / Zero buttons
  - Jog (relative position move) with +/- and an adjustable step
  - Torque and velocity feedforward fields applied to the jog
  - A live position readout (rad + deg), updated continuously

Plus a big global E-STOP that makes every motor go limp.

Blocking MotorLink calls (enable soft-start, wait-for-feedback moves) run on a
background worker thread so the UI never freezes. The live readout is polled on
the Qt main thread via a QTimer and only reads MotorLink's lock-protected state.

Usage:
    pip install pyserial PyQt6
    ./motor_gui_qt.py
"""

import math
import queue
import re
import sys
import threading
import time

from PyQt6.QtCore import Qt, QObject, QTimer, QThread, pyqtSignal
from PyQt6.QtGui import QFont
from PyQt6.QtWidgets import (
    QApplication, QComboBox, QDoubleSpinBox, QFrame, QGridLayout, QGroupBox,
    QHBoxLayout, QLabel, QLayout, QPlainTextEdit, QPushButton, QScrollArea,
    QSizePolicy, QVBoxLayout, QWidget,
)

from motor_ctl import MotorLink, autodetect_ports

RAD2DEG = 180.0 / math.pi
PAIR_RAMP_SECONDS = 30
PAIR_SETTLE_SECONDS = 10
M1_ASSIST_DEADBAND_DEG = 1.0
M1_ASSIST_NM_PER_DEG = 0.10
M1_ASSIST_MAX_NM = 0.35
M1_ASSIST_RISE_NM_S = 0.15

MOTORS = [
    (1, "HipPitch", "CAN 1"),
    (2, "HipRoll", "CAN 2"),
    (3, "HipYaw", "CAN 3"),
    (4, "Knee", "CAN 4"),
    (5, "FootDiff1", "CAN 5"),
    (6, "FootDiff2", "CAN 6"),
]


class EmittingStream(QObject):
    """A tee for sys.stdout that mirrors MotorLink's prints into the GUI log
    while still writing to the real terminal. Emits one signal per line."""
    text_written = pyqtSignal(str)

    def __init__(self, real_stream):
        super().__init__()
        self._real = real_stream
        self._buf = ""

    def write(self, text):
        if self._real is not None:
            self._real.write(text)
        self._buf += text
        while "\n" in self._buf:
            line, self._buf = self._buf.split("\n", 1)
            if line:
                self.text_written.emit(line)

    def flush(self):
        if self._real is not None:
            self._real.flush()


class CommandWorker(QThread):
    """Runs submitted callables sequentially off the GUI thread so blocking
    MotorLink calls (sleeps, wait-for-feedback) never freeze the UI."""

    def __init__(self):
        super().__init__()
        self._q = queue.Queue()
        self._stop = False

    def submit(self, fn):
        self._q.put(fn)

    def run(self):
        while not self._stop:
            try:
                fn = self._q.get(timeout=0.1)
            except queue.Empty:
                continue
            if fn is None:
                break
            try:
                fn()
            except Exception as e:  # keep the worker alive on command errors
                print(f"  [gui] command error: {e}")

            finally:
                self._q.task_done()

    def busy(self):
        return bool(self._q.unfinished_tasks)

    def stop(self):
        self._stop = True
        self._q.put(None)
        self.wait(1000)


class MotorPanel(QGroupBox):
    def __init__(self, idx, name, meta, gui):
        super().__init__(f"M{idx}  {name}")
        self.idx = idx
        self.gui = gui

        root = QVBoxLayout(self)

        meta_lbl = QLabel(meta)
        meta_lbl.setStyleSheet("color: #888;")
        root.addWidget(meta_lbl)

        # --- live position readout ---
        self.pos_lbl = QLabel("—.———— rad")
        f = QFont("monospace")
        f.setPointSize(14)
        f.setBold(True)
        self.pos_lbl.setFont(f)
        self.pos_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.pos_lbl.setFrameShape(QFrame.Shape.StyledPanel)
        self.pos_lbl.setMargin(3)
        self.pos_lbl.setSizePolicy(QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Minimum)
        self.effort_lbl = QLabel()
        self.effort_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.effort_lbl.setFrameShape(QFrame.Shape.StyledPanel)
        self.effort_lbl.setFont(f)
        self.effort_lbl.setMargin(3)
        self.effort_lbl.setSizePolicy(QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Minimum)
        readings = QHBoxLayout()
        readings.addWidget(self.pos_lbl, 1)
        readings.addWidget(self.effort_lbl, 1)
        root.addLayout(readings)

        self.state_lbl = QLabel("passive")
        self.state_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        root.addWidget(self.state_lbl)

        # --- enable / passive / zero ---
        btn_row = QHBoxLayout()
        self.en_btn = QPushButton("Enable")
        self.pa_btn = QPushButton("Passive")
        self.ze_btn = QPushButton("Zero")
        self.en_btn.clicked.connect(lambda: self.gui.run(lambda: self.gui.link.enable(self.idx)))
        self.pa_btn.clicked.connect(lambda: self.gui.run(lambda: self.gui.link.passive(self.idx)))
        self.ze_btn.clicked.connect(lambda: self.gui.run(lambda: self.gui.link.origin(self.idx)))
        for b in (self.en_btn, self.pa_btn, self.ze_btn):
            btn_row.addWidget(b)
        root.addLayout(btn_row)

        # --- jog step + feedforward fields ---
        grid = QGridLayout()
        grid.addWidget(QLabel("step (rad)"), 0, 0)
        self.step = self._spin(-6.2832, 6.2832, 0.1, 0.1)
        grid.addWidget(self.step, 0, 1)

        grid.addWidget(QLabel("vel FF (rad/s)"), 1, 0)
        self.vel = self._spin(-50.0, 50.0, 0.1, 0.0)
        grid.addWidget(self.vel, 1, 1)

        grid.addWidget(QLabel("torque FF (Nm)"), 2, 0)
        self.torque = self._spin(-20.0, 20.0, 0.1, 0.0)
        grid.addWidget(self.torque, 2, 1)
        root.addLayout(grid)

        # --- jog buttons (press-and-hold for continuous motion) ---
        jog_row = QHBoxLayout()
        self.jog_minus = QPushButton("Jog −")
        self.jog_plus = QPushButton("Jog +")
        self._jog_timer = QTimer(self)
        self._jog_timer.setInterval(120)   # ms between repeat steps while held
        self._jog_sign = 0
        self.jog_minus.pressed.connect(lambda: self._start_jog(-1))
        self.jog_minus.released.connect(self._stop_jog)
        self.jog_plus.pressed.connect(lambda: self._start_jog(+1))
        self.jog_plus.released.connect(self._stop_jog)
        self._jog_timer.timeout.connect(self._repeat_jog)
        jog_row.addWidget(self.jog_minus)
        jog_row.addWidget(self.jog_plus)
        root.addLayout(jog_row)

        self._controls = [
            self.en_btn, self.pa_btn, self.ze_btn,
            self.jog_minus, self.jog_plus,
        ]
        self.set_connected(False)

    @staticmethod
    def _spin(lo, hi, step, default):
        s = QDoubleSpinBox()
        s.setRange(lo, hi)
        s.setSingleStep(step)
        s.setDecimals(4)
        s.setValue(default)
        return s

    def _start_jog(self, sign):
        self._jog_sign = sign
        self._jog(sign)          # fire immediately on press
        self._jog_timer.start()  # then repeat while held

    def _repeat_jog(self):
        if not self.gui.worker.busy():
            self._jog(self._jog_sign)

    def _stop_jog(self):
        self._jog_timer.stop()
        self._jog_sign = 0

    def _jog(self, sign):
        dpos = sign * self.step.value()
        vel = self.vel.value() or None
        torque = self.torque.value() or None
        self.gui.run(lambda: self.gui.link.move(
            self.idx, dpos=dpos, torque=torque, vel=vel))

    def set_connected(self, connected):
        for b in self._controls:
            b.setEnabled(connected)

    def update_readout(self, pos_rad, enabled, line=""):
        match = re.search(r"(?:^|\s)(tau|I)=([+-]?\d+(?:\.\d+)?)(?=\s|$)", line)
        torque = match[1] == "tau" if match else self.idx <= 3
        label, unit = ("Torque", "N·m") if torque else ("Current", "A")
        value = f"{float(match[2]):+.2f}" if match else "—"
        self.effort_lbl.setText(f"{label}\n{value} {unit}")
        if pos_rad is None:
            self.pos_lbl.setText("—.———— rad")
        else:
            self.pos_lbl.setText(f"{pos_rad:+.4f} rad\n{pos_rad * RAD2DEG:+.1f}°")
        if enabled:
            self.state_lbl.setText("ENABLED")
            self.state_lbl.setStyleSheet("color: #1a7f1a; font-weight: bold;")
        else:
            self.state_lbl.setText("passive")
            self.state_lbl.setStyleSheet("color: #888;")


class MotorGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("CubeMars 6-Motor Control")
        self.link = None
        self.command_generation = 0
        self.worker = CommandWorker()
        self.worker.start()

        root = QVBoxLayout(self)

        # --- connection row ---
        conn = QHBoxLayout()
        conn.addWidget(QLabel("Port:"))
        self.port_combo = QComboBox()
        self.port_combo.setEditable(True)
        self.port_combo.setMinimumWidth(180)
        conn.addWidget(self.port_combo)

        self.refresh_btn = QPushButton("Refresh")
        self.refresh_btn.clicked.connect(self.refresh_ports)
        conn.addWidget(self.refresh_btn)

        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.toggle_connect)
        conn.addWidget(self.connect_btn)

        self.status_lbl = QLabel("Disconnected")
        conn.addWidget(self.status_lbl)
        conn.addStretch(1)
        root.addLayout(conn)

        # --- E-STOP ---
        self.estop_btn = QPushButton("E-STOP  (all motors limp)")
        self.estop_btn.setMinimumHeight(56)
        self.estop_btn.setStyleSheet(
            "background-color: #c0392b; color: white; font-size: 16px; font-weight: bold;")
        self.estop_btn.clicked.connect(self.estop)
        root.addWidget(self.estop_btn)

        # --- global motion speed ---
        speed_row = QHBoxLayout()
        speed_row.addWidget(QLabel("Motion speed (rad/s):"))
        self.speed_spin = QDoubleSpinBox()
        self.speed_spin.setRange(0.01, 10.0)
        self.speed_spin.setSingleStep(0.05)
        self.speed_spin.setDecimals(2)
        self.speed_spin.setValue(0.3)
        speed_row.addWidget(self.speed_spin)
        self.speed_btn = QPushButton("Set")
        self.speed_btn.clicked.connect(self._set_speed)
        speed_row.addWidget(self.speed_btn)
        self.pair_move = None
        self.pair_btn = QPushButton(f"M1 + M4: move by −90° ({PAIR_RAMP_SECONDS} s)")
        self.pair_btn.clicked.connect(self.start_pair_move)
        speed_row.addWidget(self.pair_btn)
        self.motion_status = QLabel()
        self.motion_status.setWordWrap(True)
        speed_row.addWidget(self.motion_status, 1)
        root.addLayout(speed_row)

        # --- unknown CAN IDs ---
        self.unknown_box = QGroupBox("Unknown CAN IDs")
        self.unknown_box.setVisible(False)
        ub_layout = QHBoxLayout(self.unknown_box)
        self.unknown_lbl = QLabel("")
        self.unknown_lbl.setStyleSheet("font-family: monospace; color: #c0392b;")
        ub_layout.addWidget(self.unknown_lbl)
        ub_layout.addStretch(1)
        root.addWidget(self.unknown_box)

        # Allow the grid to scroll when the screen cannot fit both rows.
        motor_scroll = QScrollArea()
        motor_scroll.setFrameShape(QFrame.Shape.NoFrame)
        motor_scroll.setWidgetResizable(True)
        motor_grid = QWidget()
        grid = QGridLayout(motor_grid)
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setSizeConstraint(QLayout.SizeConstraint.SetMinimumSize)
        self.panels = {}
        for i, (idx, name, meta) in enumerate(MOTORS):
            panel = MotorPanel(idx, name, meta, self)
            self.panels[idx] = panel
            grid.addWidget(panel, i // 3, i % 3)
        motor_scroll.setWidget(motor_grid)
        root.addWidget(motor_scroll, 1)

        # --- log ---
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(500)
        self.log.setFixedHeight(5 * self.log.fontMetrics().lineSpacing() + 12)
        self.log.setStyleSheet("font-family: monospace; font-size: 11px;")
        root.addWidget(self.log)

        # Tee stdout so MotorLink's prints show up in the log too.
        self._stream = EmittingStream(sys.stdout)
        self._stream.text_written.connect(self.log.appendPlainText)
        sys.stdout = self._stream

        # Poll live state on the GUI thread.
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.poll_state)
        self.timer.setInterval(100)

        self.refresh_ports()
        self.set_connected(False)

    # ---- helpers -----------------------------------------------------------
    def run(self, fn):
        """Queue a (possibly blocking) MotorLink call on the worker thread."""
        if self.link is None:
            print("  [gui] not connected")
            return
        generation = self.command_generation
        def run_if_current():
            if self.command_generation == generation:
                fn()
        self.worker.submit(run_if_current)

    def refresh_ports(self):
        self.port_combo.clear()
        ports = autodetect_ports()
        self.port_combo.addItems(ports)
        if not ports:
            print("  [gui] no serial ports found")

    def toggle_connect(self):
        if self.link is None:
            self.connect()
        else:
            self.disconnect()

    def connect(self):
        port = self.port_combo.currentText().strip()
        if not port:
            print("  [gui] no port selected")
            return
        try:
            self.link = MotorLink(port)
        except Exception as e:
            print(f"  [gui] could not open {port}: {e}")
            self.link = None
            return
        self.status_lbl.setText(f"Connected: {self.link.ser.port}")
        self.connect_btn.setText("Disconnect")
        self.set_connected(True)
        self.timer.start()
        print(f"  [gui] connected on {self.link.ser.port}")

    def disconnect(self):
        self.command_generation += 1
        self.motion_status.clear()
        self.pair_move = None
        self.timer.stop()
        link, self.link = self.link, None
        if link is not None:
            link.close()
        self.status_lbl.setText("Disconnected")
        self.connect_btn.setText("Connect")
        self.set_connected(False)
        print("  [gui] disconnected")

    def start_pair_move(self):
        motors = (1, 4)
        if self.link is None or self.pair_move is not None or self.worker.busy():
            self.motion_status.setText("Wait for the current command, then click again.")
            return
        if not all(n in self.link.enabled for n in motors):
            self.motion_status.setText("Enable M1 and M4, then click the paired move.")
            return
        start = self.link.recent_positions(motors)
        if start is None:
            self.motion_status.setText("Waiting for fresh M1 and M4 positions. Retry when both update.")
            return
        now = time.monotonic()
        self.pair_move = (now, start)
        self.assist_updated = now
        self.m1_assist_nm = 0.0
        self.pair_btn.setEnabled(False)
        self.speed_btn.setEnabled(False)
        self.motion_status.setText("M1 + M4 ramp starting")
        for n in motors:
            panel = self.panels[n]
            panel._stop_jog()
            panel.set_connected(False)
        print(f"  Starting {PAIR_RAMP_SECONDS}s relative -90 deg move from "
              f"M1={math.degrees(start[1]):.1f}°, M4={math.degrees(start[4]):.1f}°. "
              f"M1 torque assist limited to {M1_ASSIST_MAX_NM:.2f} N·m.")

    def advance_pair_move(self):
        began, start = self.pair_move
        now = time.monotonic()
        elapsed = now - began
        u = min(elapsed / PAIR_RAMP_SECONDS, 1.0)
        # Quintic interpolation gives zero velocity and acceleration at both ends.
        offset = -math.pi / 2 * u**3 * (10 - 15*u + 6*u*u)
        target = {n: p + offset for n, p in start.items()}
        samples = self.link.recent_samples(start)
        try:
            if samples is None or not all(n in self.link.enabled for n in start):
                raise RuntimeError("enable state or fresh encoder feedback lost")
            actual = {n: sample[0] for n, sample in samples.items()}
            worst = max(start, key=lambda n: abs(target[n] - actual[n]))
            error = abs(target[worst] - actual[worst])
            if error > math.radians(5) or elapsed > PAIR_RAMP_SECONDS + PAIR_SETTLE_SECONDS:
                reason = (f"M{worst} tracking error {math.degrees(error):.1f}° exceeds 5°"
                          if error > math.radians(5) else "final settling timed out")
                for n in start:
                    print(f"  M{n} at {elapsed:.1f}s: encoder {math.degrees(actual[n]):+.2f}°, "
                          f"last command {math.degrees(self.link.cmd_pos.get(n, start[n])):+.2f}°, "
                          f"ramp target {math.degrees(target[n]):+.2f}°")
                    _, received, line = samples[n]
                    age_ms = (time.monotonic() - received) * 1000
                    print(f"    Feedback received {age_ms:.0f} ms ago: {line}")
                print(f"  Last M1 torque assist command: {-self.m1_assist_nm:+.3f} N·m")
                raise RuntimeError(reason)
            # This trajectory travels in the negative direction. Positive lag
            # means M1 has not yet reached the requested encoder position.
            lag_deg = math.degrees(actual[1] - target[1])
            demand_nm = max(0.0, lag_deg - M1_ASSIST_DEADBAND_DEG) * M1_ASSIST_NM_PER_DEG
            dt = min(now - self.assist_updated, self.timer.interval() / 1000.0)
            self.m1_assist_nm = min(demand_nm, M1_ASSIST_MAX_NM,
                                    self.m1_assist_nm + dt * M1_ASSIST_RISE_NM_S)
            self.assist_updated = now
            torque = -self.m1_assist_nm if self.m1_assist_nm else 0.0
            self.link.send(f"1V0\n1T{torque:.4f}\n1P{target[1]:.5f}\n"
                           f"4V0\n4T0\n4P{target[4]:.5f}\n")
            self.motion_status.setText(
                f"M1 + M4 ramp: {min(PAIR_RAMP_SECONDS, int(elapsed))} / {PAIR_RAMP_SECONDS} s "
                f"| M1 assist {torque:+.2f} N·m")
            self.link.cmd_pos.update(target)
            if u == 1 and error <= math.radians(1):
                self.pair_move = None
                self.set_connected(True)
                self.motion_status.setText("M1 + M4 complete")
                print("  Paired move complete. Holding final positions.")
        except Exception as exc:
            print(f"  Paired move stopped: {exc}")
            self.estop()
            self.motion_status.setText(f"E-STOP: {exc}. Click Enable to rearm.")

    def _set_speed(self):
        if self.link is None:
            return
        v = self.speed_spin.value()
        self.link.send(f"0L{v:.4f}\n")
        print(f"  [gui] motion speed set to {v:.3f} rad/s")

    def estop(self):
        # Call directly (not via the worker queue) so it fires immediately even
        # if a slow enable/move is in flight. MotorLink.send is lock-guarded.
        self.pair_move = None
        if self.link is not None:
            self.command_generation += 1
            for panel in self.panels.values():
                panel._stop_jog()
            self.link.estop()
            self.set_connected(True)
            self.motion_status.setText("E-STOP active. Click Enable to rearm.")
            self.poll_state()

    def set_connected(self, connected):
        self.pair_btn.setEnabled(connected)
        self.estop_btn.setEnabled(connected)
        self.speed_btn.setEnabled(connected)
        for p in self.panels.values():
            p.set_connected(connected)
            if not connected:
                p.update_readout(None, False)

    def poll_state(self):
        if self.link is None:
            return
        if self.pair_move is not None:
            self.advance_pair_move()
        if not self.link._estopped and self.motion_status.text().startswith("E-STOP"):
            self.motion_status.clear()
        enabled = set(self.link.enabled)
        for idx, panel in self.panels.items():
            jog_allowed = not self.link._estopped and (self.pair_move is None or idx not in (1, 4))
            state = self.link.drive_state.get(idx)
            stopped = state and state[1] != "none"
            if stopped:
                panel._stop_jog()
                jog_allowed = False
            panel.jog_plus.setEnabled(jog_allowed)
            panel.jog_minus.setEnabled(jog_allowed)
            position, line = self.link.get_readout(idx)
            panel.update_readout(position, idx in enabled, line)
            if stopped:
                panel.state_lbl.setText(f"Stopped: {state[1]} | Enable to rearm")
        unknown = self.link.get_unknown_ids()
        if unknown:
            ide_str = {0: "STD", 4: "EXT"}
            parts = []
            for cid, v in sorted(unknown.items()):
                ide_label = ide_str.get(v['ide'], f"ide={v['ide']}")
                parts.append(f"CAN {cid} ({ide_label})  \u00d7{v['count']}")
            self.unknown_lbl.setText("   ".join(parts))
            self.unknown_box.setVisible(True)
        else:
            self.unknown_box.setVisible(False)

    def closeEvent(self, event):
        self.timer.stop()
        if self.link is not None:
            self.link.close()
            self.link = None
        self.worker.stop()
        sys.stdout = self._stream._real
        super().closeEvent(event)


def main():
    app = QApplication(sys.argv)
    gui = MotorGUI()
    gui.resize(900, 640)
    gui.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
