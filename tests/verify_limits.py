#!/usr/bin/env python3
"""Verify that firmware-side position limits actually clamp set_position().

Strategy: put a narrow ±15° window around the current position, then ask
the servo to go 10° past each edge; a working firmware must keep it inside
the window. Restores the original limits/speed/position at the end.

Run with:
    python tests/verify_limits.py                # default /dev/ttyACM0
    python tests/verify_limits.py /dev/ttyACM0
    python tests/verify_limits.py COM6
"""
import os
import sys
import time

# Allow running from the project root without install
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from md245mw import MD245MW

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
SERVO_ID = 0
TEST_WINDOW_DEG = 15.0
OVERSHOOT_DEG = 10.0
SETTLE_S = 6.0
POLL_S = 0.15
TOLERANCE_DEG = 1.5
TEST_SPEED = 50


def wait_settled(servo, expected, timeout=SETTLE_S):
    t0 = time.monotonic()
    last = None
    while time.monotonic() - t0 < timeout:
        pos = servo.get_position()
        if pos is not None:
            if last is not None and abs(pos - last) < 0.1 and abs(pos - expected) < 0.5:
                return pos
            last = pos
        time.sleep(POLL_S)
    return servo.get_position()


def main():
    print(f"Opening {PORT}...")
    with MD245MW(PORT, servo_id=SERVO_ID) as s:
        print(f"Transport  : {s.transport.value}")

        mode = s.get_run_mode()
        if mode != 1:
            print(f"run_mode={mode} -> switching to 1 (Servo)")
            s.set_run_mode(1)
            time.sleep(0.3)
        else:
            print("run_mode   : 1 (Servo) [ok]")

        orig_lo_hi = s.get_position_limits()
        orig_speed = s.get_speed()
        p0 = s.get_position()
        print(f"Position   : {p0:.2f}°")
        print(f"Orig limits: {orig_lo_hi}")
        print(f"Orig speed : {orig_speed}")

        lo = max(1.0, p0 - TEST_WINDOW_DEG)
        hi = min(359.0, p0 + TEST_WINDOW_DEG)
        print(f"\nApplying TEST window: [{lo:.2f}°, {hi:.2f}°]  (±{TEST_WINDOW_DEG}° around current)")

        s.set_speed(TEST_SPEED)
        s.set_position_limits(lo, hi)
        time.sleep(0.3)

        read_back = s.get_position_limits()
        print(f"Read-back  : {read_back}")
        if read_back is None:
            print("FAIL: could not read back limits -> aborting")
            return 1

        cases = [
            ("below_min", lo - OVERSHOOT_DEG, lo),
            ("above_max", hi + OVERSHOOT_DEG, hi),
            ("inside",    p0,                 p0),
        ]

        results = []
        print()
        print(f"{'case':<11}  {'target':>9}  {'expected':>10}  {'actual':>9}  {'diff':>6}  verdict")
        print("-" * 68)
        for name, target, expected in cases:
            s.set_position(target)
            actual = wait_settled(s, expected)
            if actual is None:
                diff = float('nan')
                verdict = "FAIL(no-read)"
            else:
                diff = abs(actual - expected)
                verdict = "PASS" if diff <= TOLERANCE_DEG else "FAIL"
            print(f"{name:<11}  {target:>8.2f}°  {expected:>9.2f}°  {actual!s:>9}  {diff:>5.2f}°  {verdict}")
            results.append((name, verdict))

        print("\nRestoring original state...")
        if orig_lo_hi:
            s.set_position_limits(*orig_lo_hi)
        if orig_speed:
            s.set_speed(orig_speed)
        s.set_position(p0)
        time.sleep(0.3)
        print(f"  restored limits -> {s.get_position_limits()}")
        print(f"  restored speed  -> {s.get_speed()}")
        print(f"  final position  -> {s.get_position():.2f}°")

        passed = sum(1 for _, v in results if v == "PASS")
        total = len(results)
        print()
        print(f"=== {passed}/{total} cases passed ===")
        if passed == total:
            print("VERDICT: Position Limits WORK — firmware clamps out-of-range targets.")
            return 0
        else:
            print("VERDICT: SOMETHING IS WRONG — see failing case(s) above.")
            return 2


if __name__ == "__main__":
    sys.exit(main())
