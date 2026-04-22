"""
MD245MW Servo Controller — PyQt5 GUI.

Run:
    python md245mw_gui.py
"""
from __future__ import annotations

import json
import sys
import time
from pathlib import Path
from typing import Optional

from PyQt5.QtCore import Qt, QThread, pyqtSignal, pyqtSlot
from PyQt5.QtGui import QDoubleValidator, QFont
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QPushButton, QComboBox,
    QSpinBox, QDoubleSpinBox, QSlider, QGroupBox, QGridLayout, QHBoxLayout,
    QVBoxLayout, QPlainTextEdit, QMessageBox, QStatusBar, QSizePolicy,
    QLineEdit,
)

from md245mw import MD245MW, Transport, list_available_ports


CONFIG_FILE = Path(__file__).resolve().parent / "md245mw_config.json"

DEFAULT_CONFIG = {
    "open_angle_deg": 5.0,
    "close_angle_deg": 78.5,
    "cover_speed": 300,
}


def load_config() -> dict:
    cfg = dict(DEFAULT_CONFIG)
    if CONFIG_FILE.exists():
        try:
            cfg.update(json.loads(CONFIG_FILE.read_text()))
        except Exception:
            pass
    return cfg


def save_config(cfg: dict):
    CONFIG_FILE.write_text(json.dumps(cfg, indent=2))


# ---------------------------------------------------------------------------
# Status polling worker
# ---------------------------------------------------------------------------

class StatusPoller(QThread):
    status_ready = pyqtSignal(dict)
    error = pyqtSignal(str)

    def __init__(self, servo: MD245MW, interval_ms: int = 500):
        super().__init__()
        self._servo = servo
        self._interval = interval_ms / 1000.0
        self._stop = False

    def stop(self):
        self._stop = True

    def run(self):
        while not self._stop:
            try:
                st = self._servo.get_status()
                self.status_ready.emit(st)
            except Exception as e:
                self.error.emit(str(e))
            time.sleep(self._interval)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def make_angle_lineedit(initial: float) -> QLineEdit:
    le = QLineEdit(f"{initial:.2f}")
    le.setValidator(QDoubleValidator(0.0, 360.0, 2))
    le.setMaximumWidth(90)
    return le


# ---------------------------------------------------------------------------
# Main window
# ---------------------------------------------------------------------------

class MainWindow(QMainWindow):

    STATUS_FIELDS = [
        ("position_deg",  "Position (deg)",   "{:.2f}"),
        ("position_raw",  "Position (raw)",   "{}"),
        ("velocity",      "Velocity",         "{}"),
        ("torque",        "Torque",           "{}"),
        ("voltage_v",     "Voltage (V)",      "{:.2f}"),
        ("temperature_c", "Temperature (C)",  "{}"),
        ("turn_count",    "Turn count",       "{}"),
        ("run_mode",      "Run mode",         "{}"),
        ("max_speed",     "Max speed",        "{}"),
        ("firmware",      "Firmware",         "{}"),
    ]

    def __init__(self):
        super().__init__()
        self.setWindowTitle("MD245MW Servo Controller")
        self.resize(880, 780)

        self._servo: Optional[MD245MW] = None
        self._poller: Optional[StatusPoller] = None
        self._status_labels: dict[str, QLabel] = {}
        self._config = load_config()

        self._build_ui()
        self._refresh_ports()
        self._set_connected_ui(False)

    # ---- UI construction ----

    def _build_ui(self):
        root = QWidget()
        self.setCentralWidget(root)
        layout = QVBoxLayout(root)

        layout.addWidget(self._build_connection_box())
        layout.addWidget(self._build_status_box())
        layout.addWidget(self._build_control_box())
        layout.addWidget(self._build_cover_box())
        layout.addWidget(self._build_limits_box())
        layout.addWidget(self._build_log_box(), stretch=1)

        self.setStatusBar(QStatusBar())

    def _build_connection_box(self) -> QGroupBox:
        box = QGroupBox("Connection")
        h = QHBoxLayout(box)

        h.addWidget(QLabel("Port:"))
        self.cb_port = QComboBox()
        self.cb_port.setMinimumWidth(240)
        h.addWidget(self.cb_port)

        self.btn_refresh = QPushButton("Refresh")
        self.btn_refresh.clicked.connect(self._refresh_ports)
        h.addWidget(self.btn_refresh)

        h.addWidget(QLabel("Transport:"))
        self.cb_transport = QComboBox()
        for t in Transport:
            self.cb_transport.addItem(t.value, t)
        h.addWidget(self.cb_transport)

        h.addWidget(QLabel("Servo ID:"))
        self.sb_id = QSpinBox()
        self.sb_id.setRange(0, 254)
        self.sb_id.setValue(0)
        h.addWidget(self.sb_id)

        h.addStretch(1)

        self.btn_connect = QPushButton("Connect")
        self.btn_connect.clicked.connect(self._on_connect_clicked)
        h.addWidget(self.btn_connect)

        self.btn_disconnect = QPushButton("Disconnect")
        self.btn_disconnect.clicked.connect(self._on_disconnect_clicked)
        h.addWidget(self.btn_disconnect)

        return box

    def _build_status_box(self) -> QGroupBox:
        box = QGroupBox("Status")
        grid = QGridLayout(box)
        mono = QFont("Consolas", 13)
        mono.setStyleHint(QFont.Monospace)

        for i, (key, label, _) in enumerate(self.STATUS_FIELDS):
            row, col = divmod(i, 2)
            grid.addWidget(QLabel(label + ":"), row, col * 2)
            lbl = QLabel("—")
            lbl.setFont(mono)
            lbl.setMinimumWidth(120)
            self._status_labels[key] = lbl
            grid.addWidget(lbl, row, col * 2 + 1)

        return box

    def _build_control_box(self) -> QGroupBox:
        box = QGroupBox("Manual Control")
        outer = QVBoxLayout(box)

        # Position row
        pos_row = QHBoxLayout()
        pos_row.addWidget(QLabel("Target angle (deg):"))
        self.ds_angle = QDoubleSpinBox()
        self.ds_angle.setRange(0.0, 360.0)
        self.ds_angle.setDecimals(1)
        self.ds_angle.setSingleStep(1.0)
        self.ds_angle.setValue(90.0)
        pos_row.addWidget(self.ds_angle)

        self.sl_angle = QSlider(Qt.Horizontal)
        self.sl_angle.setRange(0, 3600)  # 0.1° resolution
        self.sl_angle.setValue(900)
        self.sl_angle.valueChanged.connect(lambda v: self.ds_angle.setValue(v / 10.0))
        self.ds_angle.valueChanged.connect(lambda v: self.sl_angle.setValue(int(v * 10)))
        pos_row.addWidget(self.sl_angle, stretch=1)

        self.btn_pos_read = QPushButton("Read")
        self.btn_pos_read.clicked.connect(self._on_pos_read_clicked)
        pos_row.addWidget(self.btn_pos_read)

        self.btn_move = QPushButton("Move")
        self.btn_move.clicked.connect(self._on_move_clicked)
        pos_row.addWidget(self.btn_move)

        outer.addLayout(pos_row)

        # Speed row
        spd_row = QHBoxLayout()
        spd_row.addWidget(QLabel("Max speed (0–4095):"))
        self.sb_speed = QSpinBox()
        self.sb_speed.setRange(0, 4095)
        self.sb_speed.setValue(300)
        spd_row.addWidget(self.sb_speed)

        self.sl_speed = QSlider(Qt.Horizontal)
        self.sl_speed.setRange(0, 4095)
        self.sl_speed.setValue(300)
        self.sl_speed.valueChanged.connect(self.sb_speed.setValue)
        self.sb_speed.valueChanged.connect(self.sl_speed.setValue)
        spd_row.addWidget(self.sl_speed, stretch=1)

        self.btn_speed_read = QPushButton("Read")
        self.btn_speed_read.clicked.connect(self._on_speed_read_clicked)
        spd_row.addWidget(self.btn_speed_read)

        self.btn_apply_speed = QPushButton("Apply")
        self.btn_apply_speed.clicked.connect(self._on_speed_apply_clicked)
        spd_row.addWidget(self.btn_apply_speed)

        outer.addLayout(spd_row)

        # Quick preset buttons
        preset_row = QHBoxLayout()
        preset_row.addWidget(QLabel("Quick move:"))
        for angle in (0, 45, 90, 135, 180, 225, 270, 315, 360):
            btn = QPushButton(f"{angle}°")
            btn.clicked.connect(lambda _, a=angle: self._move_to(a))
            preset_row.addWidget(btn)
        preset_row.addStretch(1)
        outer.addLayout(preset_row)

        # Misc
        misc_row = QHBoxLayout()
        self.btn_ping = QPushButton("Ping")
        self.btn_ping.clicked.connect(self._on_ping_clicked)
        misc_row.addWidget(self.btn_ping)

        self.btn_read_once = QPushButton("Read Status Once")
        self.btn_read_once.clicked.connect(self._read_once)
        misc_row.addWidget(self.btn_read_once)

        misc_row.addWidget(QLabel("Auto-refresh:"))
        self.sb_poll_ms = QSpinBox()
        self.sb_poll_ms.setRange(100, 5000)
        self.sb_poll_ms.setSingleStep(100)
        self.sb_poll_ms.setValue(500)
        self.sb_poll_ms.setSuffix(" ms")
        misc_row.addWidget(self.sb_poll_ms)

        self.btn_auto = QPushButton("Start Auto-refresh")
        self.btn_auto.setCheckable(True)
        self.btn_auto.toggled.connect(self._on_auto_toggled)
        misc_row.addWidget(self.btn_auto)
        misc_row.addStretch(1)
        outer.addLayout(misc_row)

        return box

    def _build_cover_box(self) -> QGroupBox:
        box = QGroupBox("Cover Presets (persisted to md245mw_config.json)")
        grid = QGridLayout(box)

        # Open row
        grid.addWidget(QLabel("Open position (deg):"), 0, 0)
        self.le_open = make_angle_lineedit(self._config["open_angle_deg"])
        self.le_open.editingFinished.connect(self._on_preset_edited)
        grid.addWidget(self.le_open, 0, 1)

        self.btn_open = QPushButton("Open")
        self.btn_open.clicked.connect(self._on_open_clicked)
        self.btn_open.setMinimumWidth(120)
        grid.addWidget(self.btn_open, 0, 2)

        # Close row
        grid.addWidget(QLabel("Close position (deg):"), 1, 0)
        self.le_close = make_angle_lineedit(self._config["close_angle_deg"])
        self.le_close.editingFinished.connect(self._on_preset_edited)
        grid.addWidget(self.le_close, 1, 1)

        self.btn_close = QPushButton("Close")
        self.btn_close.clicked.connect(self._on_close_clicked)
        self.btn_close.setMinimumWidth(120)
        grid.addWidget(self.btn_close, 1, 2)

        # Cover-specific speed (uses current max-speed by default, but can override)
        grid.addWidget(QLabel("Preset speed:"), 2, 0)
        self.sb_cover_speed = QSpinBox()
        self.sb_cover_speed.setRange(0, 4095)
        self.sb_cover_speed.setValue(int(self._config["cover_speed"]))
        self.sb_cover_speed.valueChanged.connect(self._on_preset_edited)
        grid.addWidget(self.sb_cover_speed, 2, 1)

        self.btn_save_presets = QPushButton("Save Now")
        self.btn_save_presets.clicked.connect(self._save_presets)
        grid.addWidget(self.btn_save_presets, 2, 2)

        grid.setColumnStretch(3, 1)
        return box

    def _build_limits_box(self) -> QGroupBox:
        box = QGroupBox("Position Limits (written to servo)")
        h = QHBoxLayout(box)

        h.addWidget(QLabel("Min (deg):"))
        self.ds_limit_min = QDoubleSpinBox()
        self.ds_limit_min.setRange(0.0, 360.0)
        self.ds_limit_min.setDecimals(1)
        self.ds_limit_min.setValue(0.0)
        h.addWidget(self.ds_limit_min)

        h.addWidget(QLabel("Max (deg):"))
        self.ds_limit_max = QDoubleSpinBox()
        self.ds_limit_max.setRange(0.0, 360.0)
        self.ds_limit_max.setDecimals(1)
        self.ds_limit_max.setValue(360.0)
        h.addWidget(self.ds_limit_max)

        self.btn_limits_read = QPushButton("Read")
        self.btn_limits_read.clicked.connect(self._on_limits_read_clicked)
        h.addWidget(self.btn_limits_read)

        self.btn_limits_apply = QPushButton("Apply")
        self.btn_limits_apply.clicked.connect(self._on_limits_apply_clicked)
        h.addWidget(self.btn_limits_apply)

        h.addStretch(1)
        return box

    def _build_log_box(self) -> QGroupBox:
        box = QGroupBox("Log")
        v = QVBoxLayout(box)

        header = QHBoxLayout()
        header.addStretch(1)
        self.btn_clear_log = QPushButton("Clear Log")
        self.btn_clear_log.clicked.connect(lambda: self.txt_log.clear())
        header.addWidget(self.btn_clear_log)
        v.addLayout(header)

        self.txt_log = QPlainTextEdit()
        self.txt_log.setReadOnly(True)
        self.txt_log.setMaximumBlockCount(500)
        mono = QFont("Consolas", 13)
        mono.setStyleHint(QFont.Monospace)
        self.txt_log.setFont(mono)
        self.txt_log.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        v.addWidget(self.txt_log)
        return box

    # ---- connection handling ----

    def _refresh_ports(self):
        current = self.cb_port.currentData()
        self.cb_port.clear()
        for info in list_available_ports():
            self.cb_port.addItem(info["label"], info["device"])
        if current is not None:
            idx = self.cb_port.findData(current)
            if idx >= 0:
                self.cb_port.setCurrentIndex(idx)
        self._log(f"Found {self.cb_port.count()} serial port(s)")

    def _on_connect_clicked(self):
        port = self.cb_port.currentData()
        if not port:
            QMessageBox.warning(self, "Connect", "No port selected.")
            return
        transport = self.cb_transport.currentData()
        servo_id = self.sb_id.value()
        self._log(f"Connecting to {port} (id={servo_id}, transport={transport.value}) ...")
        try:
            servo = MD245MW(port, servo_id=servo_id, transport=transport)
            servo.connect()
        except Exception as e:
            self._log(f"  FAIL: {e}")
            QMessageBox.critical(self, "Connect failed", str(e))
            return

        self._servo = servo
        self._log(f"  connected ({servo.transport.value})")
        self._set_connected_ui(True)
        self._read_once()

    def _on_disconnect_clicked(self):
        self._stop_poller()
        if self._servo:
            try:
                self._servo.disconnect()
            except Exception:
                pass
            self._servo = None
            self._log("Disconnected")
        self._set_connected_ui(False)
        for lbl in self._status_labels.values():
            lbl.setText("—")

    def _set_connected_ui(self, connected: bool):
        self.btn_connect.setEnabled(not connected)
        self.btn_disconnect.setEnabled(connected)
        self.btn_refresh.setEnabled(not connected)
        self.cb_port.setEnabled(not connected)
        self.cb_transport.setEnabled(not connected)
        self.sb_id.setEnabled(not connected)

        for w in (
            self.btn_move, self.btn_apply_speed, self.btn_ping,
            self.btn_read_once, self.btn_auto,
            self.btn_pos_read, self.btn_speed_read,
            self.btn_open, self.btn_close,
            self.btn_limits_read, self.btn_limits_apply,
        ):
            w.setEnabled(connected)
        if not connected:
            self.btn_auto.setChecked(False)

        self.statusBar().showMessage("Connected" if connected else "Disconnected")

    # ---- control actions ----

    def _require_servo(self) -> Optional[MD245MW]:
        if self._servo is None:
            QMessageBox.information(self, "Not connected", "Connect to the servo first.")
            return None
        return self._servo

    def _move_to(self, angle_deg: float, speed: Optional[int] = None):
        s = self._require_servo()
        if not s:
            return
        try:
            if speed is not None:
                s.set_speed(speed)
                self._log(f"set_speed({speed})")
            s.set_position(angle_deg)
            self._log(f"set_position({angle_deg:.2f}°)")
        except Exception as e:
            self._log(f"move failed: {e}")

    def _on_move_clicked(self):
        self._move_to(self.ds_angle.value())

    def _on_pos_read_clicked(self):
        s = self._require_servo()
        if not s:
            return
        try:
            pos = s.get_position()
            if pos is None:
                self._log("read position: no reply")
                return
            self.ds_angle.setValue(pos)
            self._log(f"read position → {pos:.2f}°")
        except Exception as e:
            self._log(f"read position failed: {e}")

    def _on_speed_apply_clicked(self):
        s = self._require_servo()
        if not s:
            return
        try:
            s.set_speed(self.sb_speed.value())
            self._log(f"set_speed({self.sb_speed.value()})")
        except Exception as e:
            self._log(f"set_speed failed: {e}")

    def _on_speed_read_clicked(self):
        s = self._require_servo()
        if not s:
            return
        try:
            sp = s.get_speed()
            if sp is None:
                self._log("read speed: no reply")
                return
            self.sb_speed.setValue(int(sp))
            self._log(f"read speed → {sp}")
        except Exception as e:
            self._log(f"read speed failed: {e}")

    def _on_ping_clicked(self):
        s = self._require_servo()
        if not s:
            return
        self._log(f"ping → {s.ping()}")

    def _read_once(self):
        s = self._require_servo()
        if not s:
            return
        try:
            self._apply_status(s.get_status())
        except Exception as e:
            self._log(f"read failed: {e}")

    # ---- cover presets ----

    def _parse_angle(self, le: QLineEdit, fallback: float) -> float:
        text = le.text().strip().replace(",", ".")
        try:
            val = float(text)
        except ValueError:
            return fallback
        return max(0.0, min(val, 360.0))

    def _on_open_clicked(self):
        angle = self._parse_angle(self.le_open, self._config["open_angle_deg"])
        self._move_to(angle, speed=self.sb_cover_speed.value())

    def _on_close_clicked(self):
        angle = self._parse_angle(self.le_close, self._config["close_angle_deg"])
        self._move_to(angle, speed=self.sb_cover_speed.value())

    def _on_preset_edited(self):
        """Called whenever a preset value changes — auto-save to disk."""
        self._save_presets(log=False)

    def _save_presets(self, log: bool = True):
        self._config["open_angle_deg"] = self._parse_angle(
            self.le_open, self._config["open_angle_deg"])
        self._config["close_angle_deg"] = self._parse_angle(
            self.le_close, self._config["close_angle_deg"])
        self._config["cover_speed"] = int(self.sb_cover_speed.value())
        try:
            save_config(self._config)
            # Reflect normalized values back to UI
            self.le_open.setText(f"{self._config['open_angle_deg']:.2f}")
            self.le_close.setText(f"{self._config['close_angle_deg']:.2f}")
            if log:
                self._log(f"Presets saved: open={self._config['open_angle_deg']}°, "
                          f"close={self._config['close_angle_deg']}°, "
                          f"speed={self._config['cover_speed']}")
        except Exception as e:
            self._log(f"save presets failed: {e}")

    # ---- limits ----

    def _on_limits_read_clicked(self):
        s = self._require_servo()
        if not s:
            return
        try:
            limits = s.get_position_limits()
            if limits is None:
                self._log("read limits: no reply")
                return
            lo, hi = limits
            self.ds_limit_min.setValue(lo)
            self.ds_limit_max.setValue(hi)
            self._log(f"read limits → min={lo:.2f}°, max={hi:.2f}°")
        except Exception as e:
            self._log(f"read limits failed: {e}")

    def _on_limits_apply_clicked(self):
        s = self._require_servo()
        if not s:
            return
        lo = self.ds_limit_min.value()
        hi = self.ds_limit_max.value()
        if hi < lo:
            QMessageBox.warning(self, "Limits", "Max must be ≥ Min.")
            return
        try:
            s.set_position_limits(lo, hi)
            self._log(f"set_position_limits({lo:.2f}°, {hi:.2f}°)")
        except Exception as e:
            self._log(f"set_limits failed: {e}")

    # ---- auto polling ----

    def _on_auto_toggled(self, on: bool):
        if on:
            self._start_poller()
            self.btn_auto.setText("Stop Auto-refresh")
        else:
            self._stop_poller()
            self.btn_auto.setText("Start Auto-refresh")

    def _start_poller(self):
        if self._servo is None or self._poller is not None:
            return
        self._poller = StatusPoller(self._servo, self.sb_poll_ms.value())
        self._poller.status_ready.connect(self._apply_status)
        self._poller.error.connect(lambda e: self._log(f"poll error: {e}"))
        self._poller.start()

    def _stop_poller(self):
        if self._poller is not None:
            self._poller.stop()
            self._poller.wait(1000)
            self._poller = None

    # ---- status rendering ----

    @pyqtSlot(dict)
    def _apply_status(self, status: dict):
        for key, label, fmt in self.STATUS_FIELDS:
            val = status.get(key)
            if val is None:
                self._status_labels[key].setText("—")
            else:
                try:
                    self._status_labels[key].setText(fmt.format(val))
                except Exception:
                    self._status_labels[key].setText(str(val))

    # ---- log ----

    def _log(self, msg: str):
        self.txt_log.appendPlainText(f"[{time.strftime('%H:%M:%S')}] {msg}")

    # ---- shutdown ----

    def closeEvent(self, event):
        self._stop_poller()
        self._save_presets(log=False)
        if self._servo:
            try:
                self._servo.disconnect()
            except Exception:
                pass
        super().closeEvent(event)


def main():
    app = QApplication(sys.argv)
    font = app.font()
    font.setPointSize(13)
    app.setFont(font)
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
