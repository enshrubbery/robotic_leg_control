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
import sys
import threading

from PyQt6.QtCore import Qt, QObject, QTimer, QThread, pyqtSignal
from PyQt6.QtGui import QFont
from PyQt6.QtWidgets import (
    QApplication, QComboBox, QDoubleSpinBox, QFrame, QGridLayout, QGroupBox,
    QHBoxLayout, QLabel, QPlainTextEdit, QPushButton, QVBoxLayout, QWidget,
)

from motor_ctl import MotorLink, autodetect_ports

RAD2DEG = 180.0 / math.pi

MOTORS = [
    (1, "HipPitch", "CAN 10"),
    (2, "HipRoll", "CAN 11"),
    (3, "HipYaw", "CAN 2"),
    (4, "Knee", "CAN 106"),
    (5, "FootDiff1", "CAN 104"),
    (6, "FootDiff2", "CAN 105"),
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
        self._real.write(text)
        self._buf += text
        while "\n" in self._buf:
            line, self._buf = self._buf.split("\n", 1)
            if line:
                self.text_written.emit(line)

    def flush(self):
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
        self.pos_lbl.setMinimumHeight(40)
        root.addWidget(self.pos_lbl)

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
        self._jog_timer.timeout.connect(lambda: self._jog(self._jog_sign))
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

    def update_readout(self, pos_rad, enabled):
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
        speed_row.addStretch(1)
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

        # --- motor grid (2 rows x 3) ---
        grid = QGridLayout()
        self.panels = {}
        for i, (idx, name, meta) in enumerate(MOTORS):
            panel = MotorPanel(idx, name, meta, self)
            self.panels[idx] = panel
            grid.addWidget(panel, i // 3, i % 3)
        root.addLayout(grid)

        # --- log ---
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(500)
        self.log.setFixedHeight(140)
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
        self.worker.submit(fn)

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
        self.timer.stop()
        link, self.link = self.link, None
        if link is not None:
            link.close()
        self.status_lbl.setText("Disconnected")
        self.connect_btn.setText("Connect")
        self.set_connected(False)
        print("  [gui] disconnected")

    def _set_speed(self):
        if self.link is None:
            return
        v = self.speed_spin.value()
        self.link.send(f"0L{v:.4f}\n")
        print(f"  [gui] motion speed set to {v:.3f} rad/s")

    def estop(self):
        # Call directly (not via the worker queue) so it fires immediately even
        # if a slow enable/move is in flight. MotorLink.send is lock-guarded.
        if self.link is not None:
            self.link.estop()

    def set_connected(self, connected):
        self.estop_btn.setEnabled(connected)
        self.speed_btn.setEnabled(connected)
        for p in self.panels.values():
            p.set_connected(connected)

    def poll_state(self):
        if self.link is None:
            return
        enabled = set(self.link.enabled)
        for idx, panel in self.panels.items():
            panel.update_readout(self.link.get_pos(idx), idx in enabled)
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
