#!/usr/bin/env python3
"""Demo: open/close cover by moving MD245MW between two preset angles."""
import time
from md245mw import MD245MW

PORT = "COM6"
SERVO_ID = 0

OPEN_ANGLE_DEG = 5
CLOSE_ANGLE_DEG = 78.5
OPEN_SPEED = 300
CLOSE_SPEED = 5
DWELL_S = 2.0


def main():
    with MD245MW(PORT, servo_id=SERVO_ID) as servo:
        print(f"Cover Demo (transport={servo.transport.value})")
        print("=" * 30)

        print(f"Opening cover ({OPEN_ANGLE_DEG}°)...")
        servo.set_speed(OPEN_SPEED)
        servo.set_position(OPEN_ANGLE_DEG)
        time.sleep(DWELL_S)
        print(f"  position: {servo.get_position():.1f}°")

        print(f"Closing cover ({CLOSE_ANGLE_DEG}°)...")
        servo.set_speed(CLOSE_SPEED)
        servo.set_position(CLOSE_ANGLE_DEG)
        time.sleep(DWELL_S)
        print(f"  position: {servo.get_position():.1f}°")

        print("Done")


if __name__ == "__main__":
    main()
