#!/usr/bin/env python3
#
# Two-finger HLS gripper constant-force example.
#
# The two servos must be mechanically mirrored. Put both HLS servos in EleMode
# and command equal-magnitude opposite-sign torque/current. Start with a low
# torque value, confirm the closing directions, then tune upward.

import argparse
import sys
import time

sys.path.append("..")
from scservo_sdk import *  # noqa: E402,F403


CURRENT_MA_PER_UNIT = 6.5


def parse_args():
    parser = argparse.ArgumentParser(
        description="Hold a two-servo HLS gripper with constant inward torque."
    )
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=1000000)
    parser.add_argument("--left-id", type=int, default=1)
    parser.add_argument("--right-id", type=int, default=2)
    parser.add_argument(
        "--left-sign",
        type=int,
        choices=(-1, 1),
        default=1,
        help="Sign that closes the left finger in EleMode.",
    )
    parser.add_argument(
        "--right-sign",
        type=int,
        choices=(-1, 1),
        default=-1,
        help="Sign that closes the right finger in EleMode.",
    )
    parser.add_argument(
        "--torque",
        type=int,
        default=120,
        help="Target torque/current command. HLS examples imply about 6.5 mA/unit.",
    )
    parser.add_argument("--ramp-step", type=int, default=10)
    parser.add_argument("--ramp-period", type=float, default=0.05)
    parser.add_argument("--hold-hz", type=float, default=50.0)
    parser.add_argument("--print-period", type=float, default=0.5)
    parser.add_argument(
        "--max-current",
        type=int,
        default=0,
        help="Stop if abs(present current) exceeds this raw value. 0 disables.",
    )
    parser.add_argument("--max-temp", type=int, default=70)
    parser.add_argument("--left-min", type=int, default=None)
    parser.add_argument("--left-max", type=int, default=None)
    parser.add_argument("--right-min", type=int, default=None)
    parser.add_argument("--right-max", type=int, default=None)
    return parser.parse_args()


def check(packet, result, error, label):
    if result != COMM_SUCCESS:
        raise RuntimeError(f"{label}: {packet.getTxRxResult(result)}")
    if error:
        raise RuntimeError(f"{label}: {packet.getRxPacketError(error)}")


def read_feedback(packet, servo_id):
    length = HLS_PRESENT_CURRENT_H - HLS_PRESENT_POSITION_L + 1
    data, result, error = packet.readTxRx(servo_id, HLS_PRESENT_POSITION_L, length)
    check(packet, result, error, f"read feedback id={servo_id}")

    def word(addr):
        off = addr - HLS_PRESENT_POSITION_L
        return packet.scs_makeword(data[off], data[off + 1])

    return {
        "pos": packet.scs_tohost(word(HLS_PRESENT_POSITION_L), 15),
        "speed": packet.scs_tohost(word(HLS_PRESENT_SPEED_L), 15),
        "load": packet.scs_tohost(word(HLS_PRESENT_LOAD_L), 10),
        "voltage": data[HLS_PRESENT_VOLTAGE - HLS_PRESENT_POSITION_L],
        "temp": data[HLS_PRESENT_TEMPERATURE - HLS_PRESENT_POSITION_L],
        "moving": data[HLS_MOVING - HLS_PRESENT_POSITION_L],
        "current": packet.scs_tohost(word(HLS_PRESENT_CURRENT_L), 15),
    }


def write_ele(packet, servo_id, torque):
    result, error = packet.WriteEle(servo_id, torque)
    check(packet, result, error, f"write torque id={servo_id}")


def in_range(value, min_value, max_value):
    if min_value is not None and value < min_value:
        return False
    if max_value is not None and value > max_value:
        return False
    return True


def safety_check(args, left_fb, right_fb):
    if not in_range(left_fb["pos"], args.left_min, args.left_max):
        raise RuntimeError(f"left position limit reached: {left_fb['pos']}")
    if not in_range(right_fb["pos"], args.right_min, args.right_max):
        raise RuntimeError(f"right position limit reached: {right_fb['pos']}")
    if args.max_current:
        if abs(left_fb["current"]) > args.max_current:
            raise RuntimeError(f"left current limit reached: {left_fb['current']}")
        if abs(right_fb["current"]) > args.max_current:
            raise RuntimeError(f"right current limit reached: {right_fb['current']}")
    if left_fb["temp"] >= args.max_temp:
        raise RuntimeError(f"left temperature limit reached: {left_fb['temp']}")
    if right_fb["temp"] >= args.max_temp:
        raise RuntimeError(f"right temperature limit reached: {right_fb['temp']}")


def main():
    args = parse_args()

    port = PortHandler(args.port)
    packet = hls(port)

    if not port.openPort():
        raise RuntimeError(f"failed to open port {args.port}")
    if not port.setBaudRate(args.baud):
        raise RuntimeError(f"failed to set baudrate {args.baud}")

    target_left = args.left_sign * args.torque
    target_right = args.right_sign * args.torque
    period = 1.0 / args.hold_hz
    next_print = 0.0

    try:
        for servo_id in (args.left_id, args.right_id):
            model, result, error = packet.ping(servo_id)
            check(packet, result, error, f"ping id={servo_id}")
            print(f"id={servo_id} model={model}")

            result, error = packet.EnableTorque(servo_id, 0)
            check(packet, result, error, f"disable torque id={servo_id}")

            result, error = packet.EleMode(servo_id)
            check(packet, result, error, f"set EleMode id={servo_id}")

            result, error = packet.EnableTorque(servo_id, 1)
            check(packet, result, error, f"enable torque id={servo_id}")

        max_steps = max(abs(target_left), abs(target_right), 1)
        for raw in range(0, max_steps + args.ramp_step, args.ramp_step):
            left_cmd = max(-abs(target_left), min(abs(target_left), raw)) * args.left_sign
            right_cmd = max(-abs(target_right), min(abs(target_right), raw)) * args.right_sign
            write_ele(packet, args.left_id, left_cmd)
            write_ele(packet, args.right_id, right_cmd)
            time.sleep(args.ramp_period)

        print(
            "holding: "
            f"left={target_left}, right={target_right}, "
            f"approx_current={args.torque * CURRENT_MA_PER_UNIT:.0f}mA"
        )

        while True:
            left_fb = read_feedback(packet, args.left_id)
            right_fb = read_feedback(packet, args.right_id)
            safety_check(args, left_fb, right_fb)

            write_ele(packet, args.left_id, target_left)
            write_ele(packet, args.right_id, target_right)

            now = time.monotonic()
            if now >= next_print:
                next_print = now + args.print_period
                print(
                    "L "
                    f"pos={left_fb['pos']} cur={left_fb['current']} "
                    f"load={left_fb['load']} temp={left_fb['temp']} | "
                    "R "
                    f"pos={right_fb['pos']} cur={right_fb['current']} "
                    f"load={right_fb['load']} temp={right_fb['temp']}"
                )

            time.sleep(period)
    finally:
        for servo_id in (args.left_id, args.right_id):
            try:
                write_ele(packet, servo_id, 0)
            except Exception as exc:
                print(f"failed to stop id={servo_id}: {exc}", file=sys.stderr)
        port.closePort()


if __name__ == "__main__":
    main()
