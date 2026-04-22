"""
MD245MW Servo Controller — Low-Level Driver

Unified module that covers:
  * RS485 wire protocol to the Hitec MD245MW servo
  * Transport layer: standard USB-to-RS485 adapter OR Hitec DPC-20 programmer
  * High-level API: positions, speed, status readout

Two transports are supported transparently:

  1. Direct USB-to-RS485 adapter (CH340 / FT232 / CP2102 etc.)
     Uses pyserial directly. Code path identical to the original md245mw_rs485.py.

  2. Hitec DPC-20 USB programmer (VID 2E3C : PID 5740)
     The DPC-20 is a single-function CDC-ACM device exposed as a COM port, but
     its firmware wraps the RS485 bytes in its own framing. This module handles
     the wrapping internally via a background heartbeat thread and a small
     serial.Serial-compatible shim.

Usage:

    from md245mw import MD245MW, Transport

    # Via standard USB-to-RS485 adapter (auto-detected when transport="auto"):
    with MD245MW(port="COM3", servo_id=0) as servo:
        print(servo.get_position())

    # Explicitly via DPC-20:
    with MD245MW(port="COM6", servo_id=0, transport=Transport.DPC20) as servo:
        servo.set_position(90)

The constructor's `transport="auto"` inspects USB VID/PID and picks DPC-20 when
the port matches Hitec's adapter, otherwise uses direct RS485.
"""

from __future__ import annotations

import enum
import threading
import time
from typing import Optional

import serial
import serial.tools.list_ports


# ---------------------------------------------------------------------------
# Transport type
# ---------------------------------------------------------------------------

class Transport(enum.Enum):
    AUTO = "auto"
    RS485 = "rs485"    # standard USB-to-RS485 adapter, transparent
    DPC20 = "dpc20"    # Hitec DPC-20 programmer, wrapped framing


# DPC-20 USB identity
DPC20_VID = 0x2E3C
DPC20_PID = 0x5740


def detect_transport(port: str) -> Transport:
    """Pick RS485 or DPC20 based on USB VID/PID of the given COM port."""
    for p in serial.tools.list_ports.comports():
        if p.device.upper() == port.upper():
            if p.vid == DPC20_VID and p.pid == DPC20_PID:
                return Transport.DPC20
            return Transport.RS485
    return Transport.RS485


# ---------------------------------------------------------------------------
# DPC-20 wire protocol primitives
# ---------------------------------------------------------------------------

def _crc8_maxim(data: bytes, init: int = 0xFF) -> int:
    """CRC-8/MAXIM reflected: poly=0x8C, init=0xFF."""
    crc = init
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8C if (crc & 0x01) else (crc >> 1)
    return crc & 0xFF


def _invert(data: bytes) -> bytes:
    return bytes(b ^ 0xFF for b in data)


def _build_frame(inner: bytes, stx: int, etx: int) -> bytes:
    """Wrap inner payload as STX + inner + CRC + len + ETX."""
    return bytes([stx]) + inner + bytes([_crc8_maxim(inner), len(inner), etx])


def _extract_frame(buf: bytearray, stx: int, etx: int):
    """
    Parse one complete STX...ETX frame from buf.
    Returns (inner_bytes, consumed_len) on success, (None, discard_len) otherwise.
    """
    if len(buf) < 4:
        return None, 0
    for start in range(len(buf)):
        if buf[start] != stx:
            continue
        for end in range(start + 3, min(start + 200, len(buf))):
            if buf[end] != etx:
                continue
            inner_len = buf[end - 1]
            crc_byte = buf[end - 2]
            actual_inner_len = end - start - 3
            if inner_len != actual_inner_len:
                continue
            inner = bytes(buf[start + 1: end - 2])
            if _crc8_maxim(inner) != crc_byte:
                continue
            return inner, end + 1
        return None, 0
    return None, len(buf)  # no STX at all — discard everything


# ---------------------------------------------------------------------------
# DPC-20 adapter
# ---------------------------------------------------------------------------

class _DPC20Adapter:
    """
    Low-level DPC-20 driver. Wraps each RS485 frame in the DPC-20's KSO3
    envelope and runs a background KWAU heartbeat thread.

    Internal API only; user code talks to MD245MW.
    """

    STX_OUT, ETX_OUT = 0x09, 0x0A
    STX_IN, ETX_IN = 0x0B, 0x0C

    HEARTBEAT_INTERVAL_S = 0.4

    def __init__(self, port: str, baudrate: int = 115200):
        self.port = port
        self.baudrate = baudrate
        self.ser: Optional[serial.Serial] = None
        self._hb_stop = threading.Event()
        self._hb_thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        self._rx_buf = bytearray()

    def open(self):
        self.ser = serial.Serial()
        self.ser.port = self.port
        self.ser.baudrate = self.baudrate
        self.ser.timeout = 0.1
        self.ser.write_timeout = 1.0
        # Start with DTR/RTS low; we pulse them to wake DPC-20 after open
        self.ser.dtr = False
        self.ser.rts = False
        self.ser.open()
        time.sleep(0.05)

        # Cold-start wake-up sequence — mirrors the official HiTEC app's CDC
        # SET_CONTROL_LINE_STATE pattern captured via USBPcap:
        #   pulse RTS high (with DTR low) a few times → clear both → DTR high, RTS low
        # After this, DPC-20 is in "ready" state and replies to KWAU.
        # With DTR=1 & RTS=1 (our previous code) a cold-booted DPC-20 stays mute.
        for _ in range(3):
            self.ser.dtr = False
            self.ser.rts = True
            time.sleep(0.005)
        self.ser.dtr = False
        self.ser.rts = False
        time.sleep(0.005)
        self.ser.dtr = True
        self.ser.rts = False
        time.sleep(0.05)

        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        if not self._ping():
            raise RuntimeError("DPC-20 did not respond to KWAU keepalive")

        # Warm-up: DPC-20's TTL bridge stays disabled after cold boot until it
        # sees a stream of KWAU + KP3S + KP3I00 frames (the official app does
        # ~8 s of these before its first KSO3). Without this pre-roll every
        # subsequent KSO3 is dropped. ~1.2 s at 100 Hz is enough in practice.
        self._warmup()

        self._hb_stop.clear()
        self._hb_thread = threading.Thread(target=self._heartbeat_loop, daemon=True)
        self._hb_thread.start()

    def _warmup(self, duration_s: float = 1.2):
        deadline = time.monotonic() + duration_s
        i = 0
        while time.monotonic() < deadline:
            with self._lock:
                self._send(b"KWAU")
                time.sleep(0.008)
                self._send(b"KP3S")
                time.sleep(0.008)
                if i % 3 == 0:
                    self._send(b"KP3I00")
                    time.sleep(0.008)
                # Drain incoming kAPP etc so the buffer doesn't overflow
                self._read_frame(timeout=0.005)
            i += 1

    def close(self):
        if self._hb_thread:
            self._hb_stop.set()
            self._hb_thread.join(timeout=1.0)
            self._hb_thread = None
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.ser = None

    def _send(self, inner: bytes):
        self.ser.write(_build_frame(inner, self.STX_OUT, self.ETX_OUT))
        self.ser.flush()

    def _read_frame(self, timeout: float = 0.3) -> Optional[bytes]:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            n = self.ser.in_waiting
            if n:
                self._rx_buf.extend(self.ser.read(n))
            inner, consumed = _extract_frame(self._rx_buf, self.STX_IN, self.ETX_IN)
            if inner is not None:
                del self._rx_buf[:consumed]
                return inner
            if consumed and consumed != len(self._rx_buf):
                del self._rx_buf[:consumed]
            time.sleep(0.005)
        return None

    def _ping(self) -> bool:
        with self._lock:
            self._send(b"KWAU")
            inner = self._read_frame(0.3)
            return inner is not None and inner[:4] == b"kAPP"

    def _heartbeat_loop(self):
        last = 0.0
        while not self._hb_stop.is_set():
            if time.monotonic() - last >= self.HEARTBEAT_INTERVAL_S:
                try:
                    with self._lock:
                        self._send(b"KWAU")
                        self._read_frame(0.05)  # drain kAPP, non-critical
                except Exception:
                    pass
                last = time.monotonic()
            time.sleep(0.05)

    def send_rs485(self, rs485_frame: bytes, timeout: float = 0.5) -> Optional[bytes]:
        """
        Transmit a raw RS485 packet via DPC-20 and return the servo's 7-byte
        response (or None on timeout / no reply).
        """
        if len(rs485_frame) not in (5, 7):
            raise ValueError(f"RS485 frame must be 5 or 7 bytes, got {len(rs485_frame)}")
        expect_response = rs485_frame[3] == 0  # read request

        with self._lock:
            self._send(b"KP3I00")   # 3-pin init for servo ID 0 — observed before every op
            time.sleep(0.002)

            inverted = _invert(rs485_frame)
            trailer = bytes([0x0A, 0x0A, 0x14]) if len(rs485_frame) == 5 else bytes([0x00, 0x00, 0x00])
            self._send(b"KSO3" + bytes([len(rs485_frame)]) + inverted + trailer)

            if not expect_response:
                return None

            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                inner = self._read_frame(max(0.05, deadline - time.monotonic()))
                if inner is None:
                    continue
                if inner[:4] == b"kVs3" and len(inner) >= 11:
                    return bytes(inner[4:11])
            return None


class _DPC20SerialShim:
    """
    Thin serial.Serial-compatible wrapper around _DPC20Adapter.

    MD245MW writes and reads bytes as if it were talking to a transparent RS485
    COM port; this shim collects the bytes into an RS485 frame, dispatches it
    to the DPC-20, and exposes the response for read().
    """

    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 1.0):
        self._adapter = _DPC20Adapter(port=port, baudrate=baudrate)
        self._tx_buf = bytearray()
        self._rx_buf = bytearray()
        self.timeout = timeout
        self.is_open = False

    def open(self):
        self._adapter.open()
        self.is_open = True

    def close(self):
        self._adapter.close()
        self.is_open = False

    @property
    def in_waiting(self):
        return len(self._rx_buf)

    def reset_input_buffer(self):
        self._rx_buf.clear()

    def reset_output_buffer(self):
        self._tx_buf.clear()

    def write(self, data: bytes) -> int:
        self._tx_buf.extend(data)
        self._dispatch_pending()
        return len(data)

    def flush(self):
        self._dispatch_pending()

    def read(self, size: int = 1) -> bytes:
        deadline = time.monotonic() + (self.timeout or 0)
        while len(self._rx_buf) < size and time.monotonic() < deadline:
            time.sleep(0.005)
        n = min(size, len(self._rx_buf))
        out = bytes(self._rx_buf[:n])
        del self._rx_buf[:n]
        return out

    def _dispatch_pending(self):
        """Parse 0x96-headed RS485 frames from _tx_buf and dispatch them."""
        while self._tx_buf:
            if self._tx_buf[0] != 0x96:
                del self._tx_buf[0]
                continue
            if len(self._tx_buf) < 4:
                return
            reg_len = self._tx_buf[3]
            if reg_len == 0:
                frame_len = 5
            elif reg_len == 2:
                frame_len = 7
            else:
                del self._tx_buf[0]
                continue
            if len(self._tx_buf) < frame_len:
                return
            frame = bytes(self._tx_buf[:frame_len])
            del self._tx_buf[:frame_len]
            resp = self._adapter.send_rs485(frame, timeout=self.timeout)
            if resp:
                self._rx_buf.extend(resp)


# ---------------------------------------------------------------------------
# MD245MW high-level controller
# ---------------------------------------------------------------------------

class MD245MW:
    """
    Hitec MD245MW servo controller — high-level API.

    Works over either a transparent RS485 adapter or the Hitec DPC-20 USB
    programmer. Transport is auto-detected from USB VID/PID by default.
    """

    # ---- RS485 protocol constants ----
    HEADER_WRITE = 0x96
    HEADER_READ = 0x69

    # Register addresses — AUTHORITATIVE per Hitec CAN/RS485 Protocol Manual v2.5.
    # Earlier mappings from the dpc_d.exe decompile were misleading; addresses
    # below supersede them. See servo_protocol_eng_manual_v2_5.pdf for details.

    # ---- Status (read-only) ----
    REG_POSITION = 0x0C            # current position, 0-16383 = 0-360°
    REG_VELOCITY = 0x0E            # posdiff/100ms, signed
    REG_TORQUE = 0x10              # current PWM duty, 0-4095 = 0-100%
    REG_VOLTAGE = 0x12             # supply voltage, 100 = 1.00 V
    REG_MCU_TEMPER = 0x14          # MCU temp (°C)
    REG_CURRENT = 0x16             # motor current, in mA (current-sensor models only)
    REG_TURN_COUNT = 0x18          # accumulated turn count (R/W if FW>=1.4)
    REG_32BITS_POSITION_L = 0x1A   # 32-bit absolute position, low word
    REG_32BITS_POSITION_H = 0x1C   # 32-bit absolute position, high word
    REG_EMERGENCY_STATUS = 0x48    # READ-ONLY emergency status bit flags
    REG_EMERGENCY_STOP = REG_EMERGENCY_STATUS  # backwards-compat alias (deprecated)
    REG_PRODUCT_NO = 0x74          # NOTE: 0x74 per v2.5 manual (differs from dpc_d decomp)
    REG_MOTOR_TEMP = 0xD0          # motor temperature raw
    REG_TEMP = 0xD2                # internal servo temperature
    REG_HUM = 0xD4                 # internal humidity
    REG_TIME_L = 0xC8              # operation time (sec), low word
    REG_TIME_H = 0xCA              # operation time high word (1 = 65536 sec)
    REG_VERSION = 0xFC             # firmware version
    REG_VERSION_INVERSE = 0xFE     # firmware version (bitwise inverse, for integrity check)

    # ---- Targets (R/W) ----
    REG_POSITION_NEW = 0x1E        # ABSOLUTE target position, 0-16383 = 0-360°
    REG_TURN_NEW = 0x24            # target turn count (MULTI-TURN mode only)
    REG_VELOCITY_TARGET = 0x60     # target velocity (SPEED mode; FW>=2.3(3))

    # ---- Mode & comms config (R/W, save+reset required) ----
    REG_ID = 0x32                  # servo ID (1-254; 0 = broadcast)
    REG_BAUDRATE = 0x38            # per v2.5: 0-8 selector (NOT 0x34 from decomp)
    # ---- RUN_MODE address caveat ----
    # v2.5 manual says run_mode is at 0x44, but MD245MW FW empirically returns
    # garbage (0x1789) there. 0x4A returns valid mode values (0-3). Use 0x4A.
    # (Possibly 0x44 = hidden status word on this FW; 0x4A = action/mode register.)
    REG_RUN_MODE = 0x4A            # 0=Multi-Turn 1=Servo 2=CR 3=Speed — save+reset
    REG_ACTION_MODE = REG_RUN_MODE # alias

    # ---- Emergency / power control (R/W) ----
    REG_POWER_CONFIG = 0x46        # bits 10:9 = forced emergency stop, bit 0 = sw reset
    # Emergency stop values for bits 10:9 of REG_POWER_CONFIG:
    EMG_OFF = 0                    #   disable forced e-stop
    EMG_MOTOR_FREE = 1             #   cut motor power (true kill switch)
    EMG_SPEED_DOWN = 2             #   decelerate then hold
    EMG_MOTOR_HOLD = 3             #   freeze at current pos

    # ---- Position limits & zero point (R/W, save+reset required) ----
    # Soft limits: firmware clamps set_position() targets to [MIN, MAX]. Use these.
    REG_POSITION_MAX = 0x50        # soft upper — default 16383 (≈360°)
    REG_POSITION_MIN = 0x52        # soft lower — default 0
    # Hard limits: verified NOT to clamp set_position() on this firmware (mechanical/safety only).
    REG_POSITION_MAX_LIMIT = 0xB0
    REG_POSITION_MIN_LIMIT = 0xB2
    REG_POSITION_MID = 0xC2        # CR/Speed zero reference, default 8192 (180°)

    # ---- Speed / torque / current limits (R/W, save+reset required) ----
    REG_VELOCITY_MAX = 0x54        # max speed, 0-4095 (posdiff/100ms, 4096=90°)
    REG_TORQUE_MAX = 0x56          # PWM duty cap, 0-4095 = 0-100% (NOT current)
    REG_CURRENT_MAX = 0xD8         # current limit in mA (current-sensor models)

    # ---- Voltage protection (R/W) ----
    REG_VOLTAGE_MAX = 0x58         # 100 = 1.00 V; motor stops if exceeded
    REG_VOLTAGE_MIN = 0x5A         # 100 = 1.00 V; motor stops if below
    REG_VOLTAGE_VERY_LOW = 0x9F    # 0.1 V units; motor off below this (FW>=1.11)

    # ---- Temperature protection (R/W) ----
    REG_MCU_TEMPER_MAX = 0x5C      # motor stops above this °C; 0 = disabled
    REG_MCU_TEMPER_MIN = 0x6C      # motor stops below this °C; 0 = disabled
    REG_MOTOR_TEMP_MAX = 0x5D      # motor temp shutoff °C (FW>=1.10)
    REG_MOTOR_TEMP_HYS = 0x6D      # hysteresis for motor temp recovery

    # ---- Output Limit Point (OLP) & stall protection (R/W) ----
    REG_POS_LOCK_TIME = 0x9A       # sec before OLP triggers; 0 = always
    REG_POS_LOCK_TORQUE_RATIO = 0x9C  # torque % during OLP (0-100)

    # ---- Ramp / acceleration (R/W) ----
    REG_SPEED_UP = 0xDC            # accel ramp time (ms); 0 = immediate
    REG_SPEED_DN = 0xDE            # decel ramp time (ms)
    REG_SPEED_ES = 0xE0            # emergency-stop decel time (ms)
    REG_SPEED_VOLTAGE = 0xDA       # auto-derate speed if supply below (100mV)

    # ---- Other configurable (R/W) ----
    REG_DEADBAND = 0x4E            # position deadband (4096=90°; ≤20 recommended)
    REG_INERTIA_RANGE = 0x64       # 0=off+100%, 1=SmartSense auto, 2-4095=fixed
    REG_USER_1 = 0xCC              # non-volatile user register
    REG_USER_2 = 0xCE              # non-volatile user register
    REG_ECHO = 0xC6                # volatile user scratch (cleared on power loss)

    # ---- Admin / action (W) ----
    REG_CONFIG_SAVE = 0x70         # write 0xFFFF to save all to NVM
    REG_DEFAULT = 0x6E             # write 3855 (0x0F0F) = factory reset;
    REG_FACTORY_DEFAULT = REG_DEFAULT  # backwards-compat alias
                                   #   or 0xFFFF = restore last saved

    # ---- Deprecated aliases (from pre-v2.5 / decomp guesses; kept for back compat) ----
    REG_FIRMWARE_VERSION = 0x04    # (kept)
    REG_IC_SERIAL_SUB = 0x06       # (kept)
    REG_IC_SERIAL_MAIN = 0x08      # (kept)
    REG_STATUS = 0x0A              # (kept)
    REG_TYPE = 0x30                # (kept)
    REG_CCW_CW = 0x5E              # (kept; verify before use — not in v2.5)
    REG_LOCK = 0x72                # (kept; verify before use)
    REG_FW_VER = 0xF6              # (kept)
    REG_FIRMWARE_UPGRADE = 0x78    # (kept; admin-only)

    # Geometry: 4096 position units per 90 degrees
    POSITION_PER_90_DEG = 4096
    POSITION_PER_DEGREE = POSITION_PER_90_DEG / 90.0
    MAX_POSITION = 16383

    # Run modes
    MODE_MULTI_TURN = 0
    MODE_SERVO = 1
    MODE_CR = 2
    MODE_SPEED = 3

    def __init__(
        self,
        port: str,
        servo_id: int = 0,
        baudrate: int = 115200,
        timeout: float = 1.0,
        transport: Transport | str = Transport.AUTO,
    ):
        self.port = port
        self.servo_id = servo_id
        self.baudrate = baudrate
        self.timeout = timeout
        self.transport = Transport(transport) if not isinstance(transport, Transport) else transport
        if self.transport is Transport.AUTO:
            self.transport = detect_transport(port)
        self._serial = None  # pyserial.Serial or _DPC20SerialShim

    # ---- lifecycle ----

    def connect(self) -> bool:
        if self.transport is Transport.DPC20:
            self._serial = _DPC20SerialShim(
                port=self.port, baudrate=self.baudrate, timeout=self.timeout
            )
            self._serial.open()
        else:
            self._serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.timeout,
            )
            time.sleep(0.1)
        return True

    def disconnect(self):
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *args):
        self.disconnect()
        return False

    @property
    def is_connected(self) -> bool:
        return self._serial is not None and getattr(self._serial, "is_open", False)

    # ---- RS485 framing ----

    @staticmethod
    def _checksum(servo_id: int, address: int, reg_len: int,
                  data_low: int = 0, data_high: int = 0) -> int:
        return (servo_id + address + reg_len + data_low + data_high) & 0xFF

    def _require_open(self):
        if not self.is_connected:
            raise RuntimeError("Serial port not connected")

    def _write_register(self, address: int, value: int,
                        servo_id: Optional[int] = None) -> bool:
        self._require_open()
        sid = self.servo_id if servo_id is None else servo_id
        data_low = value & 0xFF
        data_high = (value >> 8) & 0xFF
        packet = bytes([
            self.HEADER_WRITE, sid, address, 0x02,
            data_low, data_high,
            self._checksum(sid, address, 0x02, data_low, data_high),
        ])
        self._serial.reset_input_buffer()
        self._serial.write(packet)
        self._serial.flush()
        return True

    def _read_register(self, address: int,
                       servo_id: Optional[int] = None,
                       retries: int = 2) -> Optional[int]:
        """Read a register; retry on malformed responses (DPC-20 occasionally
        returns a short/misaligned kVs3 the first time after warmup)."""
        for attempt in range(retries + 1):
            val = self._read_register_once(address, servo_id)
            if val is not None:
                return val
            time.sleep(0.02)
        return None

    def _read_register_once(self, address: int,
                            servo_id: Optional[int] = None) -> Optional[int]:
        self._require_open()
        sid = self.servo_id if servo_id is None else servo_id
        packet = bytes([
            self.HEADER_WRITE, sid, address, 0x00,
            self._checksum(sid, address, 0x00),
        ])
        self._serial.reset_input_buffer()
        self._serial.write(packet)
        self._serial.flush()
        time.sleep(0.02)

        raw = b""
        for _ in range(10):
            n = self._serial.in_waiting
            if n:
                raw += self._serial.read(n)
            if len(raw) >= 12:
                break
            time.sleep(0.01)

        # Some adapters echo the TX; skip the echo if present
        if len(raw) >= len(packet) and raw[: len(packet)] == packet:
            raw = raw[len(packet):]
        if len(raw) < 7:
            return None
        resp = raw[:7]
        if resp[0] != self.HEADER_READ or resp[2] != address or resp[3] != 0x02:
            return None
        if resp[6] != self._checksum(resp[1], resp[2], resp[3], resp[4], resp[5]):
            return None
        return resp[4] | (resp[5] << 8)

    # ---- position ----

    # Empirical write calibration (established from user-measured data points):
    #   physical_deg = WRITE_CAL_SLOPE * commanded_deg + WRITE_CAL_OFFSET
    # Derived from observed (cmd, pos) pairs: (50,130), (60,161.35),
    # (70,192.81), (80,224.38) → slope ≈ 3.15, offset ≈ -27.5.
    # The inverse (what this code applies) is:
    #   commanded_deg = (target_physical_deg - WRITE_CAL_OFFSET) / WRITE_CAL_SLOPE
    WRITE_CAL_SLOPE = 3.15
    WRITE_CAL_OFFSET = -27.5

    # Physical mechanical range observed on this unit — clamp target here
    # to prevent stalling against the hard stops.
    SAFE_MIN_DEG = 80.0   # just above lower mechanical stop ~78.7°
    SAFE_MAX_DEG = 280.0  # just below upper mechanical stop ~281.84°

    def set_position(self, angle_deg: float) -> bool:
        """
        Set target to a physical position in degrees, with calibration applied.

        `angle_deg` is the DESIRED PHYSICAL angle (what the encoder will read back).
        This method applies the inverse of the empirical calibration
        `physical = 3.15*cmd - 27.5`, so that the servo ends up at the physical
        angle the caller asked for.

        Clamped to [SAFE_MIN_DEG, SAFE_MAX_DEG] to keep off the mechanical stops.
        """
        angle_deg = max(self.SAFE_MIN_DEG, min(angle_deg, self.SAFE_MAX_DEG))
        cmd_deg = (angle_deg - self.WRITE_CAL_OFFSET) / self.WRITE_CAL_SLOPE
        raw = int(cmd_deg * self.POSITION_PER_DEGREE)
        raw = max(0, min(raw, self.MAX_POSITION))
        return self._write_register(self.REG_POSITION_NEW, raw)

    def set_position_raw(self, position: int) -> bool:
        """Write raw value directly to REG_POSITION_NEW, bypassing calibration."""
        position = max(0, min(position, self.MAX_POSITION))
        return self._write_register(self.REG_POSITION_NEW, position)

    def get_position(self) -> Optional[float]:
        """Read physical position (encoder reads 14-bit, scale POSITION_PER_DEGREE=45.511)."""
        raw = self._read_register(self.REG_POSITION)
        return raw / self.POSITION_PER_DEGREE if raw is not None else None

    def get_position_raw(self) -> Optional[int]:
        return self._read_register(self.REG_POSITION)

    # ---- speed / torque ----

    def set_speed(self, speed: int) -> bool:
        speed = max(0, min(speed, 4095))
        return self._write_register(self.REG_VELOCITY_MAX, speed)

    def get_speed(self) -> Optional[int]:
        return self._read_register(self.REG_VELOCITY_MAX)

    def set_torque_max(self, torque: int) -> bool:
        torque = max(0, min(torque, 4095))
        return self._write_register(self.REG_TORQUE_MAX, torque)

    def get_torque(self) -> Optional[int]:
        return self._read_register(self.REG_TORQUE)

    def set_acceleration(self, ms: int) -> bool:
        return self._write_register(self.REG_SPEED_UP, ms)

    def set_deceleration(self, ms: int) -> bool:
        return self._write_register(self.REG_SPEED_DN, ms)

    def get_velocity(self) -> Optional[int]:
        v = self._read_register(self.REG_VELOCITY)
        if v is None:
            return None
        return v - 65536 if v > 32767 else v

    # ---- status readouts ----

    def get_voltage(self) -> Optional[float]:
        v = self._read_register(self.REG_VOLTAGE)
        return v / 100.0 if v is not None else None

    def get_temperature(self) -> Optional[int]:
        return self._read_register(self.REG_MCU_TEMPER)

    def get_turn_count(self) -> Optional[int]:
        v = self._read_register(self.REG_TURN_COUNT)
        if v is None:
            return None
        return v - 65536 if v > 32767 else v

    def get_firmware_version(self) -> Optional[int]:
        return self._read_register(self.REG_VERSION)

    # ---- mode & limits ----

    def set_run_mode(self, mode: int) -> bool:
        """Set run/action mode. Requires save_config + power cycle to take effect."""
        return self._write_register(self.REG_ACTION_MODE, mode)

    def get_run_mode(self) -> Optional[int]:
        return self._read_register(self.REG_ACTION_MODE)

    # ---- current / limits (v2.5 corrected) ----

    def get_current(self) -> Optional[int]:
        """Motor current draw in mA (REG_CURRENT = 0x16). Current-sensor models only."""
        return self._read_register(self.REG_CURRENT)

    def set_current_limit(self, current_ma: int) -> bool:
        """
        Set motor current limit in mA (REG_CURRENT_MAX = 0xD8).
        This is the REAL current limiter (unlike REG_TORQUE_MAX which only caps PWM duty).
        Requires save_config + reset to persist. Current-sensor models only.
        """
        current_ma = max(0, min(int(current_ma), 65535))
        return self._write_register(self.REG_CURRENT_MAX, current_ma)

    def get_current_limit(self) -> Optional[int]:
        return self._read_register(self.REG_CURRENT_MAX)

    # ---- emergency stop (v2.5 corrected: REG_POWER_CONFIG bits 10:9) ----

    def emergency_stop(self, mode: int = None) -> bool:
        """
        Trigger emergency stop via REG_POWER_CONFIG (0x46) bits 10:9.
          mode = EMG_MOTOR_FREE (1) : cut motor power — true kill switch
          mode = EMG_SPEED_DOWN (2) : decelerate then hold
          mode = EMG_MOTOR_HOLD (3) : freeze immediately
        """
        if mode is None:
            mode = self.EMG_MOTOR_FREE
        # Preserve other bits of REG_POWER_CONFIG; clear bits 10:9 then set
        current = self._read_register(self.REG_POWER_CONFIG) or 0
        new_val = (current & ~(0x3 << 9)) | ((mode & 0x3) << 9)
        return self._write_register(self.REG_POWER_CONFIG, new_val)

    def release_emergency(self) -> bool:
        """Clear the forced emergency-stop bits (bits 10:9 → 0)."""
        current = self._read_register(self.REG_POWER_CONFIG) or 0
        new_val = current & ~(0x3 << 9)
        return self._write_register(self.REG_POWER_CONFIG, new_val)

    def software_reset(self) -> bool:
        """Software reset via REG_POWER_CONFIG (0x46) bit 0. Reboots servo firmware."""
        current = self._read_register(self.REG_POWER_CONFIG) or 0
        return self._write_register(self.REG_POWER_CONFIG, current | 0x1)

    def get_emergency_status(self) -> Optional[int]:
        """Read REG_EMERGENCY_STATUS (0x48) — bit flags indicating trip causes."""
        return self._read_register(self.REG_EMERGENCY_STATUS)

    # ---- misc accessors ----

    def get_deadband(self) -> Optional[int]:
        return self._read_register(self.REG_DEADBAND)

    def get_temper_max(self) -> Optional[int]:
        return self._read_register(self.REG_MCU_TEMPER_MAX)

    def get_voltage_trip(self) -> Optional[tuple[float, float]]:
        """Return (min_V, max_V) voltage trip points (REG_VOLTAGE_MIN / REG_VOLTAGE_MAX)."""
        lo = self._read_register(self.REG_VOLTAGE_MIN)
        hi = self._read_register(self.REG_VOLTAGE_MAX)
        if lo is None or hi is None:
            return None
        return (lo / 100.0, hi / 100.0)

    def get_product_info(self) -> dict:
        """Read-only identification registers."""
        return {
            "product_no": self._read_register(self.REG_PRODUCT_NO),
            "firmware_version": self._read_register(self.REG_VERSION),
            "firmware_version_short": self._read_register(self.REG_FW_VER),
            "type": self._read_register(self.REG_TYPE),
            "ic_serial_main": self._read_register(self.REG_IC_SERIAL_MAIN),
            "ic_serial_sub": self._read_register(self.REG_IC_SERIAL_SUB),
        }

    def set_position_limits(self, min_deg: float, max_deg: float) -> bool:
        """Set the soft position window. Firmware clamps set_position() targets to [min, max]."""
        min_pos = max(0, min(int(min_deg * self.POSITION_PER_DEGREE), self.MAX_POSITION))
        max_pos = max(0, min(int(max_deg * self.POSITION_PER_DEGREE), self.MAX_POSITION))
        ok1 = self._write_register(self.REG_POSITION_MIN, min_pos)
        ok2 = self._write_register(self.REG_POSITION_MAX, max_pos)
        return ok1 and ok2

    def get_position_limits(self) -> Optional[tuple[float, float]]:
        """Return (min_deg, max_deg) current soft position limits, or None on failure."""
        lo = self._read_register(self.REG_POSITION_MIN)
        hi = self._read_register(self.REG_POSITION_MAX)
        if lo is None or hi is None:
            return None
        return (lo / self.POSITION_PER_DEGREE, hi / self.POSITION_PER_DEGREE)

    # ---- config ----

    def save_config(self) -> bool:
        """
        Save all registers to non-volatile memory (v2.5 manual §Save Config).
        Caller MUST wait ≥1 second and then power-cycle (or software_reset())
        for changes to take effect on re-init.
        """
        return self._write_register(self.REG_CONFIG_SAVE, 0xFFFF)

    def factory_reset(self) -> bool:
        """Write 3855 (0x0F0F) to REG_DEFAULT → restore factory values. save+reset to apply."""
        return self._write_register(self.REG_DEFAULT, 3855)

    def restore_last_saved(self) -> bool:
        """Write 0xFFFF to REG_DEFAULT → restore last saved (roll back in-memory edits)."""
        return self._write_register(self.REG_DEFAULT, 0xFFFF)

    # ---- utility ----

    def ping(self) -> bool:
        try:
            return self._read_register(self.REG_POSITION) is not None
        except Exception:
            return False

    def get_status(self) -> dict:
        return {
            "position_deg": self.get_position(),
            "position_raw": self.get_position_raw(),
            "velocity": self.get_velocity(),
            "torque": self.get_torque(),
            "voltage_v": self.get_voltage(),
            "temperature_c": self.get_temperature(),
            "turn_count": self.get_turn_count(),
            "run_mode": self.get_run_mode(),
            "max_speed": self.get_speed(),
            "firmware": self.get_firmware_version(),
        }


# ---------------------------------------------------------------------------
# Port discovery helper (useful for GUIs)
# ---------------------------------------------------------------------------

def list_available_ports() -> list[dict]:
    """Return a list of connected serial ports with transport hints."""
    out = []
    for p in serial.tools.list_ports.comports():
        is_dpc20 = p.vid == DPC20_VID and p.pid == DPC20_PID
        out.append({
            "device": p.device,
            "description": p.description,
            "vid": p.vid,
            "pid": p.pid,
            "is_dpc20": is_dpc20,
            "label": f"{p.device} — {'DPC-20' if is_dpc20 else p.description or 'unknown'}",
        })
    return out


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(description="MD245MW servo quick CLI")
    ap.add_argument("--port", default="COM6")
    ap.add_argument("--id", type=int, default=0)
    ap.add_argument("--transport", choices=[t.value for t in Transport], default="auto")
    sub = ap.add_subparsers(dest="cmd")
    sub.add_parser("status")
    m = sub.add_parser("move"); m.add_argument("angle", type=float)
    s = sub.add_parser("speed"); s.add_argument("speed", type=int)
    args = ap.parse_args()

    with MD245MW(args.port, args.id, transport=args.transport) as servo:
        if args.cmd == "move":
            servo.set_position(args.angle)
            time.sleep(0.2)
            print(f"pos = {servo.get_position():.2f}°")
        elif args.cmd == "speed":
            servo.set_speed(args.speed)
            print(f"max speed set to {servo.get_speed()}")
        else:
            for k, v in servo.get_status().items():
                print(f"  {k:14} = {v}")
