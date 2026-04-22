# MD245MW Servo Control — 项目指南

## 项目概述

通过 RS485/TTL 协议控制 Hitec MD245MW 伺服电机的 Python 项目。支持两种传输层：
标准 USB-to-RS485 适配器，以及 Hitec DPC-20 USB 编程器（根据 USB VID/PID 自动识别）。

## 当前状态

- **最后更新**: 2026-04-21
- **进度**: 功能完成。DPC-20 协议已逆向并落地，GUI 可用，舵机读写均验证通过。
- **已完成**: 底层驱动 (`md245mw.py`)、PyQt5 GUI (`md245mw_gui.py`)、DPC-20 协议支持
- **下一步（可选）**:
  - 接通 BAT+/BAT- 电源后重测 voltage/temperature/run_mode
  - 扫描动作测试、限位设置界面
  - 打包为 pip 安装的库 / exe

## 文件说明

| 文件 | 说明 |
|------|------|
| `md245mw.py` | 底层驱动：RS485 协议 + DPC-20 传输层 + 高层 API，可作为库或 CLI 使用 |
| `md245mw_gui.py` | PyQt5 GUI：端口选择、实时状态、位置/速度控制、后台定时刷新 |
| `tests/test_minimal_move.py` | +1° 最小移动测试（含过冲/电流保护） |
| `docs/manuals/servo_protocol_eng_manual_v2_5.pdf` | Hitec CAN/RS485 协议文档 v2.5（本地保留，未提交） |
| `docs/manuals/Hitec-DPC-20-*.pdf` | DPC-20 编程器使用说明（本地保留，未提交） |
| `notes/SESSION.md`, `notes/TODO.md` | 工作笔记（本地保留，未提交） |

## 关键技术信息

### RS485 协议（舵机原生）
- 写包: `0x96 + ID + Addr + 0x02 + DataL + DataH + Checksum`
- 读请求: `0x96 + ID + Addr + 0x00 + Checksum`
- 读响应: `0x69 + ID + Addr + 0x02 + DataL + DataH + Checksum`
- Checksum: `(ID + Addr + Len + DataL + DataH) & 0xFF`
- 位置分辨率: 4096 = 90°，MAX=16383 (~360°)

### DPC-20 适配器协议（已逆向并验证）
- VID:PID = `2E3C:5740`（ArteryTek AT32 MCU）
- 单接口 CDC-ACM，映射为 COM 口；**必须** `DTR=True, RTS=True` 才会应答
- 外层帧: `STX + inner + CRC8 + len + ETX`
  - CRC-8/MAXIM reflected（poly=0x8C, init=0xFF）
  - OUT: STX=0x09, ETX=0x0A；IN: STX=0x0B, ETX=0x0C
- 会话心跳: `KWAU` → `kAPP`，约 400 ms 一次（库内部自动处理）
- 3-pin 模式初始化: 每次舵机操作前先发 `KP3I<id>`
- 舵机命令封包: `KSO3 + <len> + <按位取反的 RS485 帧> + <trailer>`
  - 读 (5B)：trailer = `0A 0A 14`
  - 写 (7B)：trailer = `00 00 00`
- 响应: `kVs3 + <原始 7B RS485 响应> + 0x00`

### 硬件连接
- DPC-20 通过 USB-C 连接 PC（当前 COM6）
- 舵机接 DPC-20 的 3-pin (PWM/TTL) 口
- 舵机电源通过 4-pin 口的 BAT+/BAT- 供电

## 运行方式

```bash
# GUI（推荐）
python md245mw_gui.py

# CLI
python md245mw.py --port COM6 status      # 读所有状态
python md245mw.py --port COM6 move 90     # 移动到 90°
python md245mw.py --port COM6 speed 300   # 设置最大速度
```

## 库用法

```python
from md245mw import MD245MW

with MD245MW("COM6", servo_id=0) as servo:
    servo.set_speed(300)
    servo.set_position(90)
    print(servo.get_status())
```

自动识别传输层（DPC-20 或标准 RS485）。若要强制指定：

```python
from md245mw import MD245MW, Transport
MD245MW("COM3", transport=Transport.RS485)
```
