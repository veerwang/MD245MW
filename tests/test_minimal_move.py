#!/usr/bin/env python3
"""
Minimal +1° move test with 30 mA current limit (already set via save_config).

Abort conditions:
  - |actual - start| > 3° (overshoot window)
  - current > 50 mA (safety margin above 30 mA limit)
  - runtime > 3 s
"""
import time
from md245mw import MD245MW

DELTA_DEG = 1.0
OVERSHOOT_LIMIT = 3.0
CURRENT_ABORT_MA = 50
MAX_TIME_S = 3.0

with MD245MW('COM6', servo_id=0) as s:
    # Pre-checks
    cur_limit = s.get_current_limit()
    mode = s.get_run_mode()
    start = s.get_position()
    start_target = s._read_register(0x1E) / MD245MW.POSITION_PER_DEGREE
    print(f'Pre: pos={start:.2f}°  target={start_target:.2f}°  mode={mode}  CURRENT_MAX={cur_limit}')
    if cur_limit != 30:
        raise SystemExit(f'CURRENT_MAX is {cur_limit}, expected 30 — abort')
    if mode != 1:
        raise SystemExit(f'mode is {mode}, expected 1 (Servo) — abort')

    target = start + DELTA_DEG
    print(f'\nCommand target = {target:.2f}°  (Δ = {DELTA_DEG:+.2f}°)')
    s.set_position(target)
    t0 = time.monotonic()

    abort_reason = None
    try:
        while time.monotonic() - t0 < MAX_TIME_S:
            time.sleep(0.1)
            pos = s.get_position()
            cur = s.get_current()
            tq = s._read_register(0x10)
            tq_s = tq - 65536 if tq > 32767 else tq
            err = pos - target
            rel = time.monotonic() - t0
            print(f'  t={rel:4.2f}s  pos={pos:7.3f}°  err={err:+6.2f}°  current={cur}mA  torque_out={tq_s:+5}')

            if abs(pos - start) > OVERSHOOT_LIMIT:
                abort_reason = f'OVERSHOOT |pos-start|={abs(pos-start):.2f}°'
                break
            if cur is not None and cur > CURRENT_ABORT_MA:
                abort_reason = f'CURRENT {cur}mA > {CURRENT_ABORT_MA}mA'
                break
            if abs(err) < 0.2:
                print(f'  -> arrived')
                break
    finally:
        final_pos = s.get_position()
        # Always sync target back to actual so motor doesn't keep fighting
        s.set_position(final_pos)
        time.sleep(0.3)
        print(f'\nTarget synced to {final_pos:.2f}° to stop any residual fighting.')
        print(f'Final: pos={final_pos:.2f}°  torque={s.get_torque()}  current={s.get_current()}mA')
        if abort_reason:
            print(f'\n*** ABORTED: {abort_reason} ***')
        else:
            print('\nOK — no abort triggered')
