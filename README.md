# MD245MW Servo Controller

Python controller for the Hitec MD245MW servo. Works through either a standard
USB-to-RS485 adapter or the **Hitec DPC-20 USB programmer**, with transport
auto-detected from USB VID/PID. Ships with a PyQt5 GUI.

## Files

| File               | Description                                              |
|--------------------|----------------------------------------------------------|
| `md245mw.py`       | Driver: RS485 protocol, DPC-20 transport, high-level API |
| `md245mw_gui.py`   | PyQt5 GUI — port selector, live status, control panel    |

## Install

```bash
pip install pyserial PyQt5
```

## Quick Start

### GUI

```bash
python md245mw_gui.py
```

Pick the COM port, click **Connect**, then use the angle / speed controls.
The DPC-20 programmer is auto-identified and enabled without any extra setup.

### Library

```python
from md245mw import MD245MW

# Auto-detects DPC-20 vs. plain RS485 by USB VID/PID
with MD245MW("COM6", servo_id=0) as servo:
    servo.set_speed(300)
    servo.set_position(90.0)
    print(servo.get_status())
```

Force a specific transport:

```python
from md245mw import MD245MW, Transport

with MD245MW("COM3", servo_id=0, transport=Transport.RS485) as servo:
    ...
```

### Command-line

```bash
python md245mw.py --port COM6 status             # dump all registers
python md245mw.py --port COM6 move 90            # move to 90°
python md245mw.py --port COM6 speed 500          # set max speed
```

## Hardware

### Standard USB-to-RS485 adapter

Wire A/B to the servo's RS485 pair, supply BAT+/BAT- externally.

### Hitec DPC-20 USB programmer (VID 2E3C : PID 5740)

- Connect DPC-20 to PC via USB-C → shows up as a single COM port
- Plug the servo into DPC-20's 3-pin TTL socket
- Provide servo power via the 4-pin BAT+/BAT- terminals

No driver install or Silicon Labs USBXpress package is needed — the standard
Windows `usbser` CDC driver is used and this library speaks the DPC-20 wire
protocol directly.

## Protocol Reference

### RS485 (servo native)

```
Write :  0x96 + ID + Addr + 0x02 + DataLow + DataHigh + Checksum
Read  :  0x96 + ID + Addr + 0x00 + Checksum
Resp. :  0x69 + ID + Addr + 0x02 + DataLow + DataHigh + Checksum
CRC   :  (ID + Addr + Len + DataLow + DataHigh) & 0xFF
Pos   :  4096 raw units = 90°    (≈ 45.51 units/deg, max 16383 = 360°)
Baud  :  115200 8N1 by default
```

### DPC-20 (wrapper around RS485)

- CDC-ACM COM port, must be opened with `DTR=True`, `RTS=True`
- Outer framing: `STX + inner + CRC8 + len + ETX`
  - CRC-8/MAXIM reflected (poly `0x8C`, init `0xFF`)
  - Host→DPC: `STX=0x09`, `ETX=0x0A`
  - DPC→Host: `STX=0x0B`, `ETX=0x0C`
- Session:
  - `KWAU` → `kAPP` heartbeat (auto-sent every ~400 ms by the library)
  - `KP3I<id>` precedes each servo op
- Servo command: `KSO3 + <len> + <bitwise-inverted RS485 bytes> + trailer`
  - Read (5B RS485): trailer `0A 0A 14`
  - Write (7B RS485): trailer `00 00 00`
- Servo response: `kVs3 + <raw 7B RS485 response> + 0x00`

## Key Registers

| Name              | Addr   | Notes                         |
|-------------------|--------|-------------------------------|
| REG_POSITION      | 0x0C   | current position (read-only)  |
| REG_VELOCITY      | 0x0E   | current velocity              |
| REG_POSITION_NEW  | 0x1E   | target position               |
| REG_RUN_MODE      | 0x44   | 0=Multi-Turn 1=Servo 2=CR 3=Speed |
| REG_VELOCITY_MAX  | 0x54   | max speed (0-4095)            |
| REG_VOLTAGE       | 0x12   | supply voltage, 0.01 V units  |
| REG_MCU_TEMPER    | 0x14   | MCU temp (°C)                 |
| REG_VERSION       | 0xFC   | firmware version              |

## Troubleshooting

- **No response / timeouts on DPC-20**: make sure the servo has BAT+/BAT- power;
  the signal line alone is sometimes enough to answer reads but not to drive
  the motor.
- **Wrong port / permission errors on Linux**: `sudo usermod -a -G dialout $USER`,
  then log out/in.
- **Direct RS485 adapter gives no reply**: check A/B wiring polarity and servo
  ID (default is 0).
