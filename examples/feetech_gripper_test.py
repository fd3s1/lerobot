#!/usr/bin/env python

"""
Interactive test for a two-motor Feetech STS3215 gripper.

The script uses the Min_Position_Limit, Max_Position_Limit and Homing_Offset
already stored on the motors as the normalization range. It does not run the
SO-ARM calibration flow.

Example:
    python lerobot/examples/feetech_gripper_test.py
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import replace
from pathlib import Path


REPO_SRC = Path(__file__).resolve().parents[1] / "src"
if str(REPO_SRC) not in sys.path:
    sys.path.insert(0, str(REPO_SRC))

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus, MODEL_RESOLUTION


LEFT_MOTOR = "gripper_left"
RIGHT_MOTOR = "gripper_right"
MOTOR_MODEL = "sts3215"
DEFAULT_PORT = "/dev/ttyACM0"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--port",
        default=DEFAULT_PORT,
        help=f"Serial port, for example /dev/ttyUSB0 or COM3. Default: {DEFAULT_PORT}.",
    )
    parser.add_argument("--left-id", type=int, default=1, help="ID of the left gripper motor.")
    parser.add_argument("--right-id", type=int, default=2, help="ID of the right gripper motor.")
    parser.add_argument(
        "--left-inverted",
        action="store_true",
        help="Invert the left motor normalized direction, so 0/100 are swapped in software.",
    )
    parser.add_argument(
        "--right-inverted",
        action="store_true",
        help="Invert the right motor normalized direction, so 0/100 are swapped in software.",
    )
    parser.add_argument(
        "--no-configure",
        action="store_true",
        help="Skip FeetechMotorsBus.configure_motors(). Useful if you only want read/write testing.",
    )
    parser.add_argument(
        "--settle-time",
        type=float,
        default=0.5,
        help="Seconds to wait after each command before printing the measured position.",
    )
    return parser.parse_args()


def raw_to_degrees(raw_position: float) -> float:
    resolution = MODEL_RESOLUTION[MOTOR_MODEL]
    return raw_position * 360.0 / (resolution - 1)


def print_positions(bus: FeetechMotorsBus) -> None:
    normalized = bus.sync_read("Present_Position")
    raw = bus.sync_read("Present_Position", normalize=False)

    print("\nCurrent positions")
    print(f"{'motor':<14} {'norm(0-100)':>12} {'deg(raw)':>12} {'raw register':>14}")
    for motor in (LEFT_MOTOR, RIGHT_MOTOR):
        print(
            f"{motor:<14} "
            f"{normalized[motor]:>12.2f} "
            f"{raw_to_degrees(raw[motor]):>12.2f} "
            f"{raw[motor]:>14}"
        )


def main() -> None:
    args = parse_args()

    bus = FeetechMotorsBus(
        port=args.port,
        motors={
            LEFT_MOTOR: Motor(args.left_id, MOTOR_MODEL, MotorNormMode.RANGE_0_100),
            RIGHT_MOTOR: Motor(args.right_id, MOTOR_MODEL, MotorNormMode.RANGE_0_100),
        },
    )

    bus.connect()
    try:
        calibration = bus.read_calibration()
        if args.left_inverted:
            calibration[LEFT_MOTOR] = replace(calibration[LEFT_MOTOR], drive_mode=1)
        if args.right_inverted:
            calibration[RIGHT_MOTOR] = replace(calibration[RIGHT_MOTOR], drive_mode=1)
        bus.calibration = calibration

        print("Using calibration read from the motors:")
        for motor, cal in calibration.items():
            direction = "inverted" if cal.drive_mode else "normal"
            print(
                f"  {motor}: id={cal.id}, min={cal.range_min}, max={cal.range_max}, "
                f"homing_offset={cal.homing_offset}, direction={direction}"
            )

        if not args.no_configure:
            bus.configure_motors()

        print_positions(bus)

        while True:
            user_input = input("\nCommand gripper position 0-100, or q to quit: ").strip()
            if user_input.lower() in {"q", "quit", "exit"}:
                break

            try:
                target = float(user_input)
            except ValueError:
                print("Please enter a number from 0 to 100.")
                continue

            if not 0 <= target <= 100:
                print("Value out of range. Use 0 for fully closed and 100 for fully open.")
                continue

            bus.sync_write(
                "Goal_Position",
                {
                    LEFT_MOTOR: target,
                    RIGHT_MOTOR: target,
                },
            )
            time.sleep(args.settle_time)
            print_positions(bus)
    finally:
        bus.disconnect()


if __name__ == "__main__":
    main()
