# MD245MW Register Map

Source: decompile of `dpc_d-2024_11_13_02-V3.2.1.exe` (official Hitec DPC-UI,
D-series covering MD245MW). Verified against USBPcap captures and live reads.

Values marked **(*)** were wrong in the pre-decompile version of this codebase.

## Identity / status (read-only)

| Addr | Name                | Description                          |
|------|---------------------|--------------------------------------|
| 0x00 | PRODUCT_NO          | product number                       |
| 0x02 | PRODUCT_VERSION     | product version                      |
| 0x04 | FIRMWARE_VERSION    | firmware version (primary)           |
| 0x06 | IC_SERIAL_SUB       | IC serial — sub                      |
| 0x08 | IC_SERIAL_MAIN      | IC serial — main                     |
| 0x0A | STATUS              | status word                          |
| 0x0C | POSITION            | current position (0–16383 = 0–360°)  |
| 0x0E | VELOCITY            | current velocity                     |
| 0x10 | TORQUE              | current PWM duty (0–4095 = 0–100%)   |
| 0x12 | VOLTAGE             | supply voltage (×0.01 V units)       |
| 0x14 | TEMPER              | MCU temperature (°C)                 |
| 0x18 | (dsPIC_OHP_IN_TEMPER) | IC-specific thermal metric         |
| 0x1A | CURRENT             | current draw (raw, ≈ mA)             |
| 0x1C | (dsPIC_OHP_OUTPUT_RATE) | IC thermal output rate            |

## Parameter / user (read-only version regs, read-write user slots)

| Addr | Name            | Description                    |
|------|-----------------|--------------------------------|
| 0x1E | POSITION_NEW    | target position (R/W)          |
| 0x20 | PARAM_VERSION_1 | parameter set version          |
| 0x22 | X22_34          | reserved/user-configurable     |
| 0x24 | PARAM_VERSION_2 | parameter set version 2        |
| 0x26 | USER_1          | user scratch register 1 (R/W)  |
| 0x28 | USER_2          | user scratch register 2 (R/W)  |
| 0x2A | USER_3          | user scratch register 3 (R/W)  |

## Identification / communication (R/W)

| Addr | Name                 | Notes                              |
|------|----------------------|------------------------------------|
| 0x30 | TYPE                 | servo model type                   |
| 0x32 | ID                   | servo ID 0–254                     |
| 0x34 | BAUDRATE             | baud rate setting                  |
| 0x36 | SIGNAL_MODE          | PWM/TTL/485 selector               |
| 0x38 | SIMPLE_RETURN_DELAY  | response delay (simple frames)     |
| 0x3A | NORMAL_RETURN_DELAY  | response delay (normal frames)     |

## Behavior / modes (R/W)

| Addr | Name               | Notes                                       |
|------|--------------------|---------------------------------------------|
| 0x46 | POWER_CONFIG       | power-on behavior configuration             |
| 0x48 | EMERGENCY          | emergency stop state                        |
| **0x4A** | **ACTION_MODE** *(was RUN_MODE)* | **0=Multi-Turn, 1=Servo, 2=CR, 3=Speed** — save + power-cycle to apply |
| 0x4C | FAILSAFE           | failsafe target position                    |
| 0x4E | DEADBAND           | position deadband                           |
| 0x50 | POSITION_MAX       | soft max (different from hard limit 0xB0)   |
| 0x52 | POSITION_MIN       | soft min (different from hard limit 0xB2)   |
| 0x54 | VELOCITY_MAX       | max velocity, 0–4095 (pos-units per 100 ms) |
| 0x56 | TORQUE_MAX         | max torque / PWM duty, 0–4095               |
| 0x58 | VOLTAGE_MAX        | high-voltage trip (×0.01 V)                 |
| 0x5A | VOLTAGE_MIN        | low-voltage trip (×0.01 V)                  |
| 0x5C | TEMPER_MAX         | over-temperature trip (°C)                  |
| 0x5E | CCW_CW             | rotation direction: 0=CW, 1=CCW             |
| 0x60 | START_SPEED        | soft-start speed                            |
| 0x62 | POWER_DOWN_TIME    | auto-idle / power-save timeout              |
| 0x64 | POSITION_SLOPE     | position ramp slope                         |

## Action / admin (write)

| Addr | Name              | Usage                                    |
|------|-------------------|------------------------------------------|
| 0x6E | FACTORY_DEFAULT   | write `0x0F0F` → factory reset           |
| 0x70 | CONFIG_SAVE       | write `0xFFFF` → save to NVM             |
| 0x72 | LOCK              | servo lock flag                          |
| 0x74 | ACTION            | trigger preset action                    |
| 0x78 | FIRMWARE_UPGRADE  | firmware upgrade entry                   |
| 0x7A | RX_BYTE_INTERVAL  | RX byte inter-arrival timeout            |
| 0x7C | SIMPLE_RETURN_LIMIT_TIME | response-time limit                |
| 0x7E | PWM_EXT_DELAY_TIME | PWM extension delay                     |

## Vibration / tuning (advanced, R/W)

| Addr | Name                    | Description                        |
|------|-------------------------|------------------------------------|
| 0x3C | VIB_SIGN_CHANGE_MARGIN  | vibration sign-change margin       |
| 0x3E | VIB_MIN_MAX_MARGIN      | vibration min/max margin           |
| 0x40 | VIB_GOOD_CHECK_NO       | vibration good-check count         |
| 0x42 | VIB_SPEED_CHECK_NO      | vibration speed-check count        |
| 0x44 | VIB_D_GAIN_MIN          | **NOT run mode — this is vibration-D gain min** |
| 0x66 | VIB_DEADBAND_MIN        |                                    |
| 0x68 | VIB_DEADBAND_MAX        |                                    |
| 0x6A | VIB_DEADBAND_DELAY      |                                    |
| 0x6C | VIB_DEADBAND_P_GAIN     |                                    |

## Position limits (R/W)

| Addr | Name                 | Description                       |
|------|----------------------|-----------------------------------|
| 0xB0 | POSITION_MAX_LIMIT   | hard upper limit (position units) |
| 0xB2 | POSITION_MIN_LIMIT   | hard lower limit                  |

## Ramping (R/W)

| Addr | Name       | Description                |
|------|------------|----------------------------|
| 0xDC | SPEED_UP   | acceleration time (ms)     |
| 0xDE | SPEED_DN   | deceleration time (ms)     |

## Extended / firmware-internal (R/W)

These typically appear above 0x80 with 16-bit unsigned address, cast as
negative in the decompile due to signed-short encoding. Listed as unsigned:

| Addr | Name                          |
|------|-------------------------------|
| 0xF6 | FW_VER                        |
| 0xFC | VERSION (firmware version full) |
| 0xFFA0 (0xA0*) | POS_LOCK_LIMIT      |
| 0xFFA2 (0xA2*) | POS_LOCK_TIME       |
| 0xFFA4 (0xA4*) | PID_SAMPLING_TIME   |
| 0xFFA6 (0xA6*) | VELOCITY_SAMPLING_TIME |
| 0xFFA8 (0xA8*) | HUM_SAMPLING_TIME   |
| 0xFFAA (0xAA*) | ADC_SAMPLING_TIME   |
| 0xFFAC (0xAC*) | TEMPER_25_DEG       |
| 0xFFAE (0xAE*) | TEMPER_50_DEG       |
| 0xFFB0 (0xB0*) | POS_PWM_MAX         |
| 0xFFB2 (0xB2*) | POS_PWM_MIN         |
| 0xFFB4 (0xB4*) | POS_VIRTUAL_BIT     |
| 0xFFB6 (0xB6*) | POS_OVERSAMPLING_BIT|
| 0xFFB8 (0xB8*) | PID_POS_PARA_P_GAIN |
| 0xFFBA (0xBA*) | MOTOR_DEADBAND_OFFSET |
| 0xFFBC (0xBC*) | I_COMP_MAX          |
| 0xFFC0 (0xC0*) | MOTOR_PWM_PRESCALER |
| 0xFFC2 (0xC2*) | POS_PWM_MID         |
| 0xFFC4 (0xC4*) | PWM_IN_SIGNAL_RANGE |
| 0xFFC6 (0xC6*) | SYS_CONFIG          |
| 0xFFCC (0xCC*) | SERIAL_SUB          |
| 0xFFCE (0xCE*) | VIB_CHECK_MAX_NO    |
| 0xFFD0 (0xD0*) | VIB_PWM_IN_DEADBAND |
| 0xFFD2 (0xD2*) | VIB_PWM_GOOD_NO     |
| 0xFFD4 (0xD4*) | PID_P_VIB           |
| 0xFFD6 (0xD6*) | PID_D_VIB           |
| 0xFFD8 (0xD8*) | POS_TARGET_LIMIT_MAX |
| 0xFFDA (0xDA*) | POS_TARGET_LIMIT_MIN |
| 0xFFDC (0xDC*) | RX_PACKET_INTERVAL  |
| 0xFFE8 (0xE8*) | TORQUE_MIN          |
| 0xFFEA (0xEA*) | POSITION_IN_NEW     |
| 0xFFEC (0xEC*) | PID_GAIN            |
| 0xFFEE (0xEE*) | TIME_RUN            |
| 0xFFF2 (0xF2*) | PID_CONFIG          |
| 0xFFF4 (0xF4*) | PZ                  |
| 0xFFF8 (0xF8*) | PPM_CONF_2          |
| 0xFFFA (0xFA*) | SPEED_CONF          |
| 0xFFFC (0xFC*) | SYS_CONFIG_2        |
| 0xFFFE (0xFE*) | PPM_CONF            |
| 0xFF80 (0x80*) | PWM_NORMAL_ACK_MIN  |
| 0xFF82 (0x82*) | PWM_NORMAL_HOLD_LIMIT |
| 0xFF84 (0x84*) | MOTOR_TURN_DIRECTION |
| 0xFF86 (0x86*) | MOTOR_PWM_PERIOD    |
| 0xFF88 (0x88*) | MOTOR_PWM_DEADTIME  |
| 0xFF8A (0x8A*) | PID_P               |
| 0xFF8C (0x8C*) | PID_D               |
| 0xFF8E (0x8E*) | PID_I               |
| 0xFF90 (0x90*) | PID_POS_DEADBAND    |
| 0xFF92 (0x92*) | POS_MIN_MAX_MARGIN  |
| 0xFF94 (0x94*) | POSITION_4095       |
| 0xFF96 (0x96*) | POSITION_0          |
| 0xFF98 (0x98*) | POS_LOCK_LIMIT      |
| 0xFF9A (0x9A*) | POS_LOCK_TIME       |
| 0xFF9C (0x9C*) | POS_LOCK_RATIO      |
| 0xFF9E (0x9E*) | TORQUE_LOCK_TIME    |

\* Real 8-bit register address — decompile sign-extends these. On the wire
the address byte is the unsigned form (0x80–0xFE).

## Protocol framing constants (from dpc_d)

| Name                      | Value |
|---------------------------|-------|
| `STX` (base)              | 2     |
| `ETX` (base)              | 3     |
| `DPC_SERIAL_CMD_MORE`     | 7  → STX=0x09, ETX=0x0A (host → DPC-20) |
| `DPC_SERIAL_RETURN_MORE`  | 9  → STX=0x0B, ETX=0x0C (DPC-20 → host) |
| `DPC_SERIAL_REPORT_MORE`  | 11 → STX=0x0D, ETX=0x0E (unsolicited)   |
| `Count_Send_Repeat`       | 3 (retry count for TX) |
| `Serial_Bits_Per_One_Byte`| 11 (8N1 + start + stop) |
| `com_none`                | 0  |
| `com_dpc_11`              | 1  |
| `com_dpc_485`             | 2  |
| `com_dpc20_3pin`          | 3  |
| `com_dpc20_4pin`          | 4  |
